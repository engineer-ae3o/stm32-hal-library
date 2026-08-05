#include "stm32f411xe.h"
#include "common.h"

#include "SEGGER_RTT.h"

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

// These are extern declared in the HAL headers. Need to be defined here.
uint32_t      SystemCoreClock   = {}; // System Clock Frequency
const uint8_t AHBPrescTable[16] = {}; // AHB prescalers table
const uint8_t APBPrescTable[8]  = {}; // APB prescalers table

void system_init(void) {
    // Enable the FPU
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // Barriers to ensure all memory accesses are completed
    __DSB();
    __ISB();

    // Set flash latency, enable I and D caches, as well as enable instruction prefetching
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);

    // Disable the PLLs
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the PLLs be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR |= PWR_CR_VOS;

#ifdef USE_HSE
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure PLL
    RCC->PLLCFGR = (HSE_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (4 << RCC_PLLCFGR_PLLQ_Pos);
#else
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure PLL
    RCC->PLLCFGR = (HSI_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) |
                   (RCC_PLLCFGR_PLLSRC_HSI) | (4 << RCC_PLLCFGR_PLLQ_Pos);
#endif

    // Bus prescaler
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    __DSB();
    __ISB();

    // Ensure the VOSRDY bit reads 1 before proceeding
    while (!(PWR->CSR & PWR_CSR_VOSRDY));

    // Update system clock
    SystemCoreClock = CLOCK_SPEED_HZ;

    // Enable bus fault and usage fault exceptions
    SCB->SHCSR |= (SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Enable exceptions on divide by 0 and unaligned trapping
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk);
}

const char* hal_err_to_string(hal_err_t err) {
    switch (err) {
        case HAL_OK:
            return "HAL_OK";
        case HAL_FAIL:
            return "HAL_FAIL";
        case HAL_INVALID_ARG:
            return "HAL_INVALID_ARG";
        case HAL_INVALID_STATE:
            return "HAL_INVALID_STATE";
        case HAL_TIMEOUT:
            return "HAL_TIMEOUT";
        case HAL_TX_ERROR:
            return "HAL_TX_ERROR";
        case HAL_RX_ERROR:
            return "HAL_RX_ERROR";
        case HAL_I2C_DEVICE_NOT_FOUND:
            return "HAL_I2C_DEVICE_NOT_FOUND";
        case HAL_I2C_ARBITRATION_LOST:
            return "HAL_I2C_ARBITRATION_LOST";
        case HAL_SPI_TXE_FAILED_TO_SET:
            return "HAL_SPI_TXE_FAILED_TO_SET";
        case HAL_SPI_BSY_FAILED_TO_CLEAR:
            return "HAL_SPI_BSY_FAILED_TO_CLEAR";
        case HAL_UART_TC_FAILED_TO_SET:
            return "HAL_UART_TC_FAILED_TO_SET";
        case HAL_DMA_TE:
            return "HAL_DMA_TE";
        case HAL_DMA_DME:
            return "HAL_DMA_DME";
        case HAL_DMA_ERR_UNKNOWN:
            return "HAL_DMA_ERR_UNKNOWN";
        default:
            return "";
    }
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
    while (true) {
    }
}

_ssize_t _write(int fd, const void* buf, size_t len) {
    (void)fd;
    return (int)SEGGER_RTT_Write(0, buf, len);
}

int _kill(pid_t pid, int sig) {
    (void)pid;
    (void)sig;
    errno = ESRCH;
    return -1;
}

pid_t _getpid() {
    return 1;
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

// Fault Handlers
__attribute__((naked)) void HardFault_Handler() {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b hard_fault_dump\n");
}

__attribute__((naked)) void BusFault_Handler() {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b bus_fault_dump\n");
}

__attribute__((naked)) void UsageFault_Handler() {
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "b usage_fault_dump\n");
}

[[noreturn]] void hard_fault_dump(const uint32_t* frame) {
    [[maybe_unused]] const volatile uint32_t r0   = frame[0];
    [[maybe_unused]] const volatile uint32_t r1   = frame[1];
    [[maybe_unused]] const volatile uint32_t r2   = frame[2];
    [[maybe_unused]] const volatile uint32_t r3   = frame[3];
    [[maybe_unused]] const volatile uint32_t r12  = frame[4];
    [[maybe_unused]] const volatile uint32_t lr   = frame[5];
    [[maybe_unused]] const volatile uint32_t pc   = frame[6];
    [[maybe_unused]] const volatile uint32_t psr  = frame[7];
    [[maybe_unused]] const volatile uint32_t cfsr = SCB->CFSR;
    __asm volatile("bkpt #0");
    while (true) {
    }
}

[[noreturn]] void bus_fault_dump(const uint32_t* frame) {
    [[maybe_unused]] const volatile uint32_t r0   = frame[0];
    [[maybe_unused]] const volatile uint32_t r1   = frame[1];
    [[maybe_unused]] const volatile uint32_t r2   = frame[2];
    [[maybe_unused]] const volatile uint32_t r3   = frame[3];
    [[maybe_unused]] const volatile uint32_t r12  = frame[4];
    [[maybe_unused]] const volatile uint32_t lr   = frame[5];
    [[maybe_unused]] const volatile uint32_t pc   = frame[6];
    [[maybe_unused]] const volatile uint32_t cfsr = SCB->CFSR;
    [[maybe_unused]] const volatile uint32_t bfar = SCB->BFAR;
    __asm volatile("bkpt #0");
    while (true) {
    }
}

[[noreturn]] void usage_fault_dump(const uint32_t* frame) {
    [[maybe_unused]] const volatile uint32_t r0   = frame[0];
    [[maybe_unused]] const volatile uint32_t r1   = frame[1];
    [[maybe_unused]] const volatile uint32_t r2   = frame[2];
    [[maybe_unused]] const volatile uint32_t r3   = frame[3];
    [[maybe_unused]] const volatile uint32_t r12  = frame[4];
    [[maybe_unused]] const volatile uint32_t lr   = frame[5];
    [[maybe_unused]] const volatile uint32_t pc   = frame[6];
    [[maybe_unused]] const volatile uint32_t cfsr = SCB->CFSR;
    __asm volatile("bkpt #0");
    while (true) {
    }
}
