#include "stm32f411xe.h"
#include "RTT/SEGGER_RTT.h"
#include "printf/printf.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/log.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <sys/types.h>


// Initalizes hardware resources needed before main runs
void system_init(void) {
    // Enable the FPU
    SCB->CPACR |= (0xFUL << 20);
    __DSB();
    __ISB();

    // Set the flash latency, enable I and D caches, as well as the instruction prefetch buffer
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);
    __DSB();
    __ISB();

    // Disable the PLLs
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the PLLs be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR &= ~PWR_CR_VOS;
    PWR->CR |= (PWR_CR_VOS_1 | PWR_CR_VOS_0);

#ifdef USE_HSE
    // Enable the HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure the PLL to provide a clock of 100MHz, derived from the HSE
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ);
    RCC->PLLCFGR |= (HSE_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSE) |
                    (4 << RCC_PLLCFGR_PLLQ_Pos);
#else
    // Enable the HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure the PLL to provide a clock of 100MHz, derived from the HSI
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ);
    RCC->PLLCFGR |= (HSI_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI) |
                    (4 << RCC_PLLCFGR_PLLQ_Pos);
#endif

    // Set the bus prescalers.
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Ensure the VOSRDY bit reads 1 after the PLLs have been enabled
    while (!(PWR->CSR & PWR_CSR_VOSRDY));

    // Use the PLL as the SYSCLK source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    __DSB();
    __ISB();

    // Enable the CSS to monitor the HSE
    RCC->CR |= RCC_CR_CSSON;

#ifdef USE_HSE
    // Disable the HSI since not in use
    RCC->CR &= ~RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY);
#else
    // Disable the HSE since not in use
    RCC->CR &= ~RCC_CR_HSEON;
    while (RCC->CR & RCC_CR_HSERDY);
#endif

    // Enable the bus fault and usage fault exceptions
    SCB->SHCSR |= (SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Enable exceptions on divide by 0 and unaligned memory accesses
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk);

    system_core_clock_update();
    SEGGER_RTT_Init();

    LOGI("System_Init", "--------------- Done with FPU, PLL, prescalers and system clocks setup ---------------");
}

// Provide a weak main function
[[__gnu__::__noreturn__, __gnu__::__weak__]] int main(void) {
    LOGE("Main", "Application failed to provide a main function. Using the weak stub instead");
    HALT();
}

// Fault Handlers
[[__gnu__::__naked__]] void HardFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b hard_fault_dump\n");
}

[[__gnu__::__naked__]] void BusFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b bus_fault_dump\n");
}

[[__gnu__::__naked__]] void UsageFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b usage_fault_dump\n");
}

void NMI_Handler(void) {
    LOGI("CPU Exception", "Non Maskable Interrupt fired.");

    // Check if the interrupt was from the Clock Security System
    if (RCC->CIR & RCC_CIR_CSSF) {
        RCC->CIR |= RCC_CIR_CSSC;

        // When this interrupt occurs, it means the HSE had a failure.
        // The CSS automatically switches the SYSCLK source to the HSI.
        // To continue normal operation, we reconfigure the SYSCLK to
        // use the PLL, which in turn will be derived from the HSI.
        const char* TAG = "CSS";

        // Make sure the SYSCLK is indeed fed from the HSI
        if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
            LOGE(TAG, "CSS not automatically switched to HSI after HSE fault");
            PANIC();
        }

        LOGI(TAG, "Reconfiguring the PLL as SYSCLK source to acheive a bus clock of 100MHz, derived from the HSI");

        // Ensure the PLLs are disabled
        RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
        while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

        // Configure the PLL to provide a clock of 100MHz, derived from the HSI
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ);
        RCC->PLLCFGR |= (HSI_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) |
                        (RCC_PLLCFGR_PLLSRC_HSI) | (4 << RCC_PLLCFGR_PLLQ_Pos);

        // Set the bus prescalers.
        RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
        RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

        // Enable the PLL
        RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY));

        // Use the PLL as the SYSCLK source
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

        __DSB();
        __ISB();

        system_core_clock_update();
        LOGI(TAG, "PLL and clock prescalers configured.");
        LOGI(TAG,
             "AHB matrix clock: %luMHz, APB1 clock: %luMHz, APB2 clock: %luMHz",
             (SystemCoreClock / 1'000'000),
             (APB1CoreClock / 1'000'000),
             (APB2CoreClock / 1'000'000));
        LOGI(TAG, "Resuming normal operation with the HSI");
    }
}

// Fault state dumps
[[__gnu__::__noreturn__, __gnu__::__weak__, __gnu__::__used__]] void hard_fault_dump(const unsigned int* frame) {
    LOGE("CPU Exception", "Hard fault.");

    const unsigned int r0   = frame[0];
    const unsigned int r1   = frame[1];
    const unsigned int r2   = frame[2];
    const unsigned int r3   = frame[3];
    const unsigned int r12  = frame[4];
    const unsigned int lr   = frame[5];
    const unsigned int pc   = frame[6];
    const unsigned int psr  = frame[7];
    const unsigned int cfsr = SCB->CFSR;

    LOGE("Fault", "R0: 0x%X", r0);
    LOGE("Fault", "R1: 0x%X", r1);
    LOGE("Fault", "R2: 0x%X", r2);
    LOGE("Fault", "R3: 0x%X", r3);
    LOGE("Fault", "R12: 0x%X", r12);
    LOGE("Fault", "LR: 0x%X", lr);
    LOGE("Fault", "PC: 0x%X", pc);
    LOGE("Fault", "PSR: 0x%X", psr);
    LOGE("Fault", "CFSR: 0x%X", cfsr);

    HALT();
}

[[__gnu__::__noreturn__, __gnu__::__weak__, __gnu__::__used__]] void bus_fault_dump(const unsigned int* frame) {
    LOGE("CPU Exception", "Bus fault.");

    const unsigned int r0   = frame[0];
    const unsigned int r1   = frame[1];
    const unsigned int r2   = frame[2];
    const unsigned int r3   = frame[3];
    const unsigned int r12  = frame[4];
    const unsigned int lr   = frame[5];
    const unsigned int pc   = frame[6];
    const unsigned int cfsr = SCB->CFSR;
    const unsigned int bfar = SCB->BFAR;

    LOGE("Fault", "R0: 0x%X", r0);
    LOGE("Fault", "R1: 0x%X", r1);
    LOGE("Fault", "R2: 0x%X", r2);
    LOGE("Fault", "R3: 0x%X", r3);
    LOGE("Fault", "R12: 0x%X", r12);
    LOGE("Fault", "LR: 0x%X", lr);
    LOGE("Fault", "PC: 0x%X", pc);
    LOGE("Fault", "CFSR: 0x%X", cfsr);
    LOGE("Fault", "BFAR: 0x%X", bfar);

    HALT();
}

[[__gnu__::__noreturn__, __gnu__::__weak__, __gnu__::__used__]] void usage_fault_dump(const unsigned int* frame) {
    LOGE("CPU Exception", "Usage fault.");

    const unsigned int r0   = frame[0];
    const unsigned int r1   = frame[1];
    const unsigned int r2   = frame[2];
    const unsigned int r3   = frame[3];
    const unsigned int r12  = frame[4];
    const unsigned int lr   = frame[5];
    const unsigned int pc   = frame[6];
    const unsigned int cfsr = SCB->CFSR;

    LOGE("Fault", "R0: 0x%X", r0);
    LOGE("Fault", "R1: 0x%X", r1);
    LOGE("Fault", "R2: 0x%X", r2);
    LOGE("Fault", "R3: 0x%X", r3);
    LOGE("Fault", "R12: 0x%X", r12);
    LOGE("Fault", "LR: 0x%X", lr);
    LOGE("Fault", "PC: 0x%X", pc);
    LOGE("Fault", "CFSR: 0x%X", cfsr);

    HALT();
}

// Stub the syscalls needed by newlibc
[[__gnu__::__weak__]] int _close(int fd) {
    (void)fd;
    errno = EBADF;
    return -1;
}

[[__gnu__::__weak__]] off_t _lseek(int fd, off_t offset, int whence) {
    (void)fd;
    (void)offset;
    (void)whence;
    errno = EBADF;
    return -1;
}

[[__gnu__::__weak__]] int _read(int fd, void* buf, size_t count) {
    (void)fd;
    (void)buf;
    (void)count;
    errno = EBADF;
    return -1;
}

[[__gnu__::__weak__, __gnu__::__noreturn__]] void _exit(int status) {
    (void)status;
    LOGE("Exit", "_exit() is called.");
    PANIC();
}

[[__gnu__::__weak__]] _ssize_t _write(int fd, const void* buf, size_t len) {
    (void)fd;
    return (_ssize_t)SEGGER_RTT_Write(RTT_BUFFER_INDEX, buf, len);
}

[[__gnu__::__weak__]] int _kill(pid_t pid, int sig) {
    (void)pid;
    (void)sig;
    errno = ESRCH;
    return -1;
}

[[__gnu__::__weak__]] pid_t _getpid(void) {
    return -1;
}

[[__gnu__::__weak__]] caddr_t _sbrk(ptrdiff_t increment) {
    (void)increment;
    errno = ENOMEM;
    return (caddr_t)-1;
}

[[__gnu__::__weak__]] int _fstat(int fd, struct stat* st) {
    (void)fd;
    (void)st;
    errno = EBADF;
    return -1;
}

[[__gnu__::__weak__]] int _isatty(int fd) {
    (void)fd;
    errno = EBADF;
    return 0;
}

// Needed by the printf library
void putchar_(char c) {
    SEGGER_RTT_PutChar(RTT_BUFFER_INDEX, c);
}
