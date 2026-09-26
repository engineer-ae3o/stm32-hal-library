#include "stm32f411xe.h"
#include "RTT/SEGGER_RTT.h"
#include "printf/printf.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <sys/types.h>


// Initalizes hardware resources needed before main runs. Called in the reset handler
void system_init(void) {
    // Enable the I and D caches, as well as the instruction prefetch buffer
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
    __DSB();
    __ISB();

    // Enable the FPU
    SCB->CPACR |= (0xFUL << 20);
    __DSB();
    __ISB();

    // Enable exceptions on division by 0
    SCB->CCR |= (SCB_CCR_DIV_0_TRP_Msk);

    // Enable the MemManage, bus fault, and usage fault exceptions.
    SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

    // Initialize the logging interface (SEGGER RTT), the debug trace counter and the SysTick counter
    SEGGER_RTT_Init();
    systick_init();
    dwt_cnt_init();

    // Configure the system clock to 100MHz, derived from the HSE. Also ensure the audio PLL is disabled at startup
    system_core_clock_config(HSE_PLL_100MHz);
    audio_pll_clock_config(AUDIO_PLL_DISABLE);

    LOGI("System_Init", "------------------- FPU, PLL and system clock setup complete -------------------");
    LOGI("System_Init",
         "System Clock: %luMHz, APB1 Bus Clock: %luMHz, APB2 Bus Clock: %luMHz from the %s",
         (get_system_core_clock() / 1'000'000U),
         (get_apb1_core_clock() / 1'000'000U),
         (get_apb2_core_clock() / 1'000'000U),
         is_sysclk_on_hse() ? "HSE" : "HSI");
}

// Provide a weak main function
[[__gnu__::__noreturn__, __gnu__::__weak__]] int main(void) {
    LOGE("Main", "Application failed to provide a main function. Using the weak stub instead");
    halt();
}

// Fault Handlers
//
// All four route through decode_fault() via a trampoline that resolves whether the
// fault frame is on the MSP or PSP (EXC_RETURN bit 2) and tags which fault it came
// from, since CFSR alone can't disambiguate a genuine HardFault from one escalated
// from a disabled configurable fault handler.
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} fault_frame_t;

typedef enum {
    FAULT_TYPE_HARD = 0,
    FAULT_TYPE_MEMMANAGE,
    FAULT_TYPE_BUS,
    FAULT_TYPE_USAGE,
} fault_type_t;

#define DEFINE_FAULT_TRAMPOLINE(handler_name, fault_id)                                                                                              \
    [[__gnu__::__naked__]] void handler_name(void) {                                                                                                 \
        __asm volatile("tst lr, #4\n"                                                                                                                \
                       "ite eq\n"                                                                                                                    \
                       "mrseq r0, msp\n"                                                                                                             \
                       "mrsne r0, psp\n"                                                                                                             \
                       "mov r1, lr\n"                                                                                                                \
                       "movs r2, %0\n"                                                                                                               \
                       "b decode_fault\n" ::"i"(fault_id)                                                                                            \
                       : "r0", "r1", "r2");                                                                                                          \
    }

DEFINE_FAULT_TRAMPOLINE(HardFault_Handler, FAULT_TYPE_HARD)
DEFINE_FAULT_TRAMPOLINE(BusFault_Handler, FAULT_TYPE_BUS)
DEFINE_FAULT_TRAMPOLINE(UsageFault_Handler, FAULT_TYPE_USAGE)
DEFINE_FAULT_TRAMPOLINE(MemManage_Handler, FAULT_TYPE_MEMMANAGE)

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
        const system_clock_t system_clock = get_system_core_clock_type();
        switch (system_clock) {
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
                system_core_clock_config(HSI_PLL_MATCH_HSE);
                break;
            case HSE_PLL_MATCH_HSI:
                system_core_clock_config(HSI_PLL_DIRECT);
                break;
            default:
                system_core_clock_config(system_clock);
                break;
        }

        LOGI(TAG, "Resuming normal operation with the HSI with a system clock of %luMHz", get_system_core_clock() / 1'000'000U);
    }
}

// Fault decoder. A single entry point for all four handlers above. Reads CFSR/HFSR
// and, for MemManage/BusFault, the address registers, and logs the specific cause
// bit by bit rather than a raw hex dump. Weak so a specific application can override
// with its own recovery logic if it ever needs to.
[[__gnu__::__noreturn__, __gnu__::__weak__, __gnu__::__used__]] void
decode_fault(const fault_frame_t* frame, uint32_t exc_return, fault_type_t fault_type) {
    const uint32_t cfsr = SCB->CFSR;

    switch (fault_type) {
        case FAULT_TYPE_HARD: {
            LOGE("CPU Exception", "Hard fault.");

            const uint32_t hfsr = SCB->HFSR;
            if (hfsr & SCB_HFSR_VECTTBL_Msk) {
                LOGE("HardFault", "Bus error reading the vector table itself");
            }
            if (hfsr & SCB_HFSR_FORCED_Msk) {
                LOGE("HardFault", "Escalated from a configurable fault");
            }
            break;
        }

        case FAULT_TYPE_MEMMANAGE:
            LOGE("CPU Exception", "MPU fault.");

            if (cfsr & SCB_CFSR_IACCVIOL_Msk) {
                LOGE("MemFault", "Instruction access violation");
            }
            if (cfsr & SCB_CFSR_DACCVIOL_Msk) {
                LOGE("MemFault", "Data access violation");
            }
            if (cfsr & SCB_CFSR_MUNSTKERR_Msk) {
                LOGE("MemFault", "MPU violation on exception return (unstacking)");
            }
            if (cfsr & SCB_CFSR_MSTKERR_Msk) {
                LOGE("MemFault", "MPU violation on exception entry (stacking). Possible stack overflow into a guarded region");
            }
            if (cfsr & SCB_CFSR_MLSPERR_Msk) {
                LOGE("MemFault", "MPU violation during lazy FP state preservation");
            }
            if (cfsr & SCB_CFSR_MMARVALID_Msk) {
                LOGE("MemFault", "MMFAR: 0x%X", (unsigned int)SCB->MMFAR);
            }
            break;

        case FAULT_TYPE_BUS:
            LOGE("CPU Exception", "Bus fault.");

            if (cfsr & SCB_CFSR_IBUSERR_Msk) {
                LOGE("BusFault", "Instruction bus error");
            }
            if (cfsr & SCB_CFSR_PRECISERR_Msk) {
                LOGE("BusFault", "Precise data bus error: The PC is the faulting instruction");
            }
            if (cfsr & SCB_CFSR_IMPRECISERR_Msk) {
                LOGE("BusFault", "Imprecise data bus error: The PC is not reliable");
            }
            if (cfsr & SCB_CFSR_UNSTKERR_Msk) {
                LOGE("BusFault", "Bus error on exception return (unstacking)");
            }
            if (cfsr & SCB_CFSR_STKERR_Msk) {
                LOGE("BusFault", "Bus error on exception entry (stacking)");
            }
            if (cfsr & SCB_CFSR_LSPERR_Msk) {
                LOGE("BusFault", "Bus error during lazy FP state preservation");
            }
            if (cfsr & SCB_CFSR_BFARVALID_Msk) {
                LOGE("BusFault", "BFAR: 0x%X", (unsigned int)SCB->BFAR);
            }
            break;

        case FAULT_TYPE_USAGE:
            LOGE("CPU Exception", "Usage fault.");

            if (cfsr & SCB_CFSR_UNDEFINSTR_Msk) {
                LOGE("UsageFault", "Undefined instruction");
            }
            if (cfsr & SCB_CFSR_INVSTATE_Msk) {
                LOGE("UsageFault", "Invalid EPSR state (bad Thumb bit / IT block)");
            }
            if (cfsr & SCB_CFSR_INVPC_Msk) {
                LOGE("UsageFault", "Invalid PC load / corrupt exception return");
            }
            if (cfsr & SCB_CFSR_NOCP_Msk) {
                LOGE("UsageFault", "Coprocessor access fault (FPU used without CPACR enabling it)");
            }
            if (cfsr & SCB_CFSR_UNALIGNED_Msk) {
                LOGE("UsageFault", "Unaligned access trap");
            }
            if (cfsr & SCB_CFSR_DIVBYZERO_Msk) {
                LOGE("UsageFault", "Division by zero");
            }
            break;
    }

    LOGE("Fault", "R0: 0x%X", (unsigned int)frame->r0);
    LOGE("Fault", "R1: 0x%X", (unsigned int)frame->r1);
    LOGE("Fault", "R2: 0x%X", (unsigned int)frame->r2);
    LOGE("Fault", "R3: 0x%X", (unsigned int)frame->r3);
    LOGE("Fault", "R12: 0x%X", (unsigned int)frame->r12);
    LOGE("Fault", "LR: 0x%X", (unsigned int)frame->lr);
    LOGE("Fault", "PC: 0x%X", (unsigned int)frame->pc);
    LOGE("Fault", "PSR: 0x%X", (unsigned int)frame->xpsr);
    LOGE("Fault", "EXC_RETURN: 0x%X", (unsigned int)exc_return);
    LOGE("Fault", "CFSR: 0x%X", (unsigned int)cfsr);

    // Clear all set bits
    SCB->CFSR = cfsr;
    halt();
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

[[__gnu__::__weak__, __gnu__::__noreturn__]] void _exit(int exit_code) {
    LOGE("Exit", "Program exited with an exit code of %d", exit_code);
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

// Use the heap allocation API in utils/heap.h (uses o1heap)
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
