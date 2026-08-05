#include "stm32f411xe.h"
#include "common.h"

#include "SEGGER_RTT.h"

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <sys/types.h>


// Use the HSE
#define USE_HSE


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

    // Configure TIM2 as our tick source
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure TIM2 for 1ms interrupts at 100MHz
    // Prescaler: 100MHz / 100 = 1MHz, so a PSC of 100 - 1 = 99
    // Auto reload: 1kHz, so an ARR of 1000 - 1 = 999
    TIM2->PSC = 99;
    TIM2->ARR = 999;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;

    // Configure NVIC settings for TIM2
    NVIC_SetPriority(TIM2_IRQn, 15);
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Use TIM2 as our tick timer source
static atomic_uint s_tick_timer_ms = 0;

uint32_t get_tick_ms(void) {
    return atomic_load_explicit(&s_tick_timer_ms, memory_order_relaxed);
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag and increment the tick counter
        TIM2->SR &= ~TIM_SR_UIF;
        atomic_fetch_add_explicit(&s_tick_timer_ms, 1, memory_order_relaxed);
    }
}

// Convert error codes to string
const char* err_code_lut[] = {
    [HAL_OK]                      = "HAL_OK",
    [HAL_FAIL]                    = "HAL_FAIL",
    [HAL_INVALID_ARG]             = "HAL_INVALID_ARG",
    [HAL_INVALID_STATE]           = "HAL_INVALID_STATE",
    [HAL_TIMEOUT]                 = "HAL_TIMEOUT",
    [HAL_TX_ERROR]                = "HAL_TX_ERROR",
    [HAL_RX_ERROR]                = "HAL_RX_ERROR",
    [HAL_I2C_DEVICE_NOT_FOUND]    = "HAL_I2C_DEVICE_NOT_FOUND",
    [HAL_I2C_ARBITRATION_LOST]    = "HAL_I2C_ARBITRATION_LOST",
    [HAL_SPI_TXE_FAILED_TO_SET]   = "HAL_SPI_TXE_FAILED_TO_SET",
    [HAL_SPI_BSY_FAILED_TO_CLEAR] = "HAL_SPI_BSY_FAILED_TO_CLEAR",
    [HAL_UART_TC_FAILED_TO_SET]   = "HAL_UART_TC_FAILED_TO_SET",
    [HAL_DMA_TE]                  = "HAL_DMA_TE",
    [HAL_DMA_DME]                 = "HAL_DMA_DME",
    [HAL_DMA_ERR_UNKNOWN]         = "HAL_DMA_ERR_UNKNOWN",
};

const char* hal_err_to_string(hal_err_t err) {
    return err_code_lut[err];
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
    __asm volatile("bkpt #0");
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

pid_t _getpid(void) {
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
