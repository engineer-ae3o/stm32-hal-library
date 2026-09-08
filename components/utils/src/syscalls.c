#include "stm32f411xe.h"
#include "RTT/SEGGER_RTT.h"
#include "printf/printf.h"
#include "utils/common.h"
#include "utils/log.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <sys/types.h>


// These are extern declared in the CMSIS headers. Need to be defined here.
// At startup, the HSI feeds the SYSCLK, and since there are no prescalers or divider
// active at boot, the values of the HCLK, PCLK1 and PCLK2 are equal to the HSI value
uint32_t SystemCoreClock = HSI_VALUE_MHZ * 1'000'000;
uint32_t APB1CoreClock   = HSI_VALUE_MHZ * 1'000'000;
uint32_t APB2CoreClock   = HSI_VALUE_MHZ * 1'000'000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


// Initalizes hardware resources needed before main runs
void system_init(void) {
    // Enable the FPU
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
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

    // Disable the HSI since not in use
    RCC->CR &= ~RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY);

    __DSB();
    __ISB();

    // Enable the bus fault and usage fault exceptions
    SCB->SHCSR |= (SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Enable exceptions on divide by 0 and unaligned memory accesses
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk);

    system_core_clock_update();
    SEGGER_RTT_Init();
}

// Update the buses' clock frequency variables
void system_core_clock_update(void) {
    uint32_t sysclk = 0;

    // Get the SYSCLK clock source
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSI:
            sysclk = HSI_VALUE_MHZ * 1'000'000;
            break;

        case RCC_CFGR_SWS_HSE:
            sysclk = HSE_VALUE_MHZ * 1'000'000;
            break;

        case RCC_CFGR_SWS_PLL:
            // Get the PLL clock source
            if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_Pos) {
                // The HSE is the PLL clock source
                const uint32_t pllm   = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
                const uint32_t pllvco = (HSE_VALUE_MHZ * 1'000'000 / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                sysclk                = pllvco / pllp;
            } else {
                // The HSI is the PLL clock source
                const uint32_t pllm   = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
                const uint32_t pllvco = (HSI_VALUE_MHZ * 1'000'000 / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                sysclk                = pllvco / pllp;
            }
            break;

        default:
            sysclk = HSI_VALUE_MHZ * 1'000'000;
            break;
    }

    // Compute the HCLK, APB1 and APB2 bus frequencies
    __disable_irq();
    SystemCoreClock = sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    APB1CoreClock   = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
    APB2CoreClock   = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    __enable_irq();
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
