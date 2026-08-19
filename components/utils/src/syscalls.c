#include "stm32f411xe.h"
#include "utils/common.h"
#include "SEGGER_RTT.h"
#include "utils/log.h"
#include "printf.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <sys/types.h>


// These are extern declared in the CMSIS headers. Need to be defined here.
uint32_t      SystemCoreClock   = 16'000'000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


// Initalizes hardware resources needed before main runs
void system_init(void) {
    // Enable the FPU
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // Barriers to ensure all memory accesses are completed
    __DSB();
    __ISB();

    // Set flash latency, enable I and D caches, as well as the instruction prefetch buffer
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);

    // Disable the PLLs
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the PLLs be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR |= PWR_CR_VOS;

#ifdef USE_HSE
    // Enable the HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure the PLL
    RCC->PLLCFGR = (HSE_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSE) |
                   (4 << RCC_PLLCFGR_PLLQ_Pos);
#else
    // Enable the HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure the PLL
    RCC->PLLCFGR = (HSI_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI) |
                   (4 << RCC_PLLCFGR_PLLQ_Pos);
#endif

    // Bus prescaler
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Switch to the PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    __DSB();
    __ISB();

    // Ensure the VOSRDY bit reads 1 before proceeding
    while (!(PWR->CSR & PWR_CSR_VOSRDY));

    // Update the system clock variable
    SystemCoreClock = CLOCK_SPEED_HZ;

    // Enable bus fault and usage fault exceptions
    SCB->SHCSR |= (SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Enable exceptions on divide by 0 and unaligned trapping
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk);

    // Initialize SEGGER RTT
    SEGGER_RTT_Init();
}

// Fault Handlers
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b hard_fault_dump\n");
}

__attribute__((naked)) void BusFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b bus_fault_dump\n");
}

__attribute__((naked)) void UsageFault_Handler(void) {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b usage_fault_dump\n");
}

// Fault state dumps
[[noreturn]] void hard_fault_dump(const unsigned int* frame) {
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

    __asm("bkpt #0");

    // Halt manually if the debugger is not attached
    while (true);
}

[[noreturn]] void bus_fault_dump(const unsigned int* frame) {
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

    __asm("bkpt #0");

    // Halt manually if the debugger is not attached
    while (true);
}

[[noreturn]] void usage_fault_dump(const unsigned int* frame) {
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

    __asm("bkpt #0");

    // Halt manually if the debugger is not attached
    while (true);
}

// Stub the syscalls needed by newlibc
int _close(int fd) {
    (void)fd;
    errno = EBADF;
    return -1;
}

off_t _lseek(int fd, off_t offset, int whence) {
    (void)fd;
    (void)offset;
    (void)whence;
    errno = EBADF;
    return -1;
}

int _read(int fd, void* buf, size_t count) {
    (void)fd;
    (void)buf;
    (void)count;
    errno = EBADF;
    return -1;
}

[[noreturn]] void _exit(int status) {
    (void)status;
    LOGE("Exit", "_exit() is called.");
    PANIC();
}

_ssize_t _write(int fd, const void* buf, size_t len) {
    (void)fd;
    return (_ssize_t)SEGGER_RTT_Write(RTT_BUFFER_INDEX, buf, len);
}

int _kill(pid_t pid, int sig) {
    (void)pid;
    (void)sig;
    errno = ESRCH;
    return -1;
}

pid_t _getpid(void) {
    return -1;
}

caddr_t _sbrk(ptrdiff_t increment) {
    (void)increment;
    errno = ENOMEM;
    return (caddr_t)-1;
}

int _fstat(int fd, struct stat* st) {
    (void)fd;
    (void)st;
    errno = EBADF;
    return -1;
}

int _isatty(int fd) {
    (void)fd;
    errno = EBADF;
    return 0;
}

// Needed by the printf library
void putchar_(char c) {
    SEGGER_RTT_PutChar(RTT_BUFFER_INDEX, c);
}
