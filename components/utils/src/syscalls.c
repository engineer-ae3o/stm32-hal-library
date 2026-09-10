#include "stm32f411xe.h"
#include "RTT/SEGGER_RTT.h"
#include "printf/printf.h"
#include "system_stm32f4xx.h"
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
    // Enable the I and D caches, as well as the instruction prefetch buffer
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
    __DSB();
    __ISB();

    // Enable the FPU
    SCB->CPACR |= (0xFUL << 20);
    __DSB();
    __ISB();

    // Enable the bus fault and usage fault exceptions
    SCB->SHCSR |= (SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Enable exceptions on divide by 0 and unaligned memory accesses
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk);

    // Configure the system clock to 100MHz, derived from the HSE. Also ensure the audio PLL is disabled at startup
    system_core_clock_config(HSE_PLL_100MHz);
    audio_pll_clock_config(AUDIO_PLL_DISABLE);

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
    LOGI("CPU Exception", "The Non Maskable Interrupt triggered.");

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

        LOGI(TAG, "Clock Security System fault: HSE failure. Reconfiguring to use the HSI as the PLL clock source");

        // Reconfigure the main PLL back to whatever value it was on, but its HSI equivalent
        switch (SystemCoreClockType) {
            case HSE_PLL_100MHz:
                system_core_clock_config(HSI_PLL_100MHz);
                break;
            case HSE_PLL_96MHz:
                system_core_clock_config(HSI_PLL_96MHz);
                break;
            case HSE_PLL_84MHz:
                system_core_clock_config(HSI_PLL_84MHz);
                break;
            case HSE_PLL_64MHz:
                system_core_clock_config(HSI_PLL_64MHz);
                break;
            case HSE_PLL_48MHz:
                system_core_clock_config(HSI_PLL_48MHz);
                break;
            case HSE_PLL_DIRECT:
                system_core_clock_config(HSI_PLL_DIRECT);
                break;
            default:
                system_core_clock_config(SystemCoreClockType);
                break;
        }

        // Reconfigure the audio PLL to run at whatever value it was before the CSS fault
        audio_pll_clock_config(AudioPLLCoreClockType);

        LOGI(TAG, "Resuming normal operation with the HSI with a system clock of %luMHz", SystemCoreClock / 1'000'000U);
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
