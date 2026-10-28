# STM32F411 HAL

A hardware abstraction library/framework for embedded development on the STM32F411, written from scratch in C (tests are in C++). It uses the chip's registers directly, using the CMSIS headers.

I wrote it for the STM32F411 Black Pill, so I have a set of drivers I plan to use in future projects.

## Example

```c
#include "drivers/uart.h"

uart_config_t cfg = {
    .over_sampling = UART_OVER_SAMPLING_16,
    .baud_rate     = 115200,
    .tx_pin        = BOARD_UART1_TX_PA9,
    .rx_pin        = BOARD_UART1_RX_PA10,
};

TRY(uart_init(USART1, &cfg));
TRY(uart_transmit_poll(USART1, data, size));
```

Every function returns a `hal_err_t`. `HAL_OK` means success. Anything else says what went wrong, for example `HAL_ERR_INVALID_ARG` or `HAL_ERR_TIMEOUT`. Bad arguments are rejected before any register is touched.

## How the project is packaged

The core library is a **static library** (`libf411-hal.a`). It is not a program. You link it into your own application, which supplies its own `main()`. The archive bundles the drivers, utils, startup code and the third-party code, so you only link one file. The linker script is `stm32f411_linker.ld`.

Because a library has no `main()`, the `debug` and `release` presets only produce the `.a` file. They do **not** produce an ELF.

The `tests-debug` and `tests-release` presets are different. They build the test firmware from `test_main.cpp` into an ELF (`f411-hal.elf`), along with a `.bin`, a `.map` file and a disassembly. That is the file you flash to the board.

## What it supports

| Peripheral | What works |
|---|---|
| GPIO | Pin modes, alternate functions, pull-ups, speed, external interrupts (EXTI) |
| Timers | One-shot and periodic timers, pause and resume, output compare, input capture, pulse counter, quadrature decoder |
| PWM | Edge and center aligned, complementary outputs, dead time, break input (advanced timers), PWM input mode |
| ADC | One-shot reads, continuous DMA, injected group, analog watchdog, internal sensors (temperature, Vbat, Vref) |
| DMA | All streams, callbacks, memory-to-memory copies |
| UART | Polling send, DMA send and receive |
| SPI | Master mode, 8 and 16 bit, polling and DMA, all four clock modes |
| I2C | Master mode, 100 kHz and 400 kHz, digital filter |
| I2S | Send and receive, DMA, double buffering, audio PLL |
| CRC | Hardware CRC, fed by the CPU or by DMA |
| IWDG | Watchdog setup and reload |

**Not supported yet:** UART polling receive and idle line, I2C and SPI slave mode, I2S slave mode and full duplex, USB, SDIO, RTC and the WWDG.

### Clock setup

One call sets the system clock. You can run from the external crystal (HSE) or the internal oscillator (HSI), with or without the PLL.

| Preset family | Presets |
|---|---|
| From HSE | `HSE_PLL_100MHz`, `HSE_PLL_96MHz`, `HSE_PLL_84MHz`, `HSE_PLL_64MHz`, `HSE_PLL_48MHz`, `HSE_PLL_MATCH_HSI`, `HSE_PLL_DIRECT` |
| From HSI | `HSI_PLL_100MHz`, `HSI_PLL_96MHz`, `HSI_PLL_84MHz`, `HSI_PLL_64MHz`, `HSI_PLL_48MHz`, `HSI_PLL_MATCH_HSE`, `HSI_PLL_DIRECT` |

The 96 MHz and 48 MHz presets give exactly 48 MHz for USB. There are also audio PLL presets for I2S (`AUDIO_PLL_76_8MHz`, `AUDIO_PLL_135_5MHz`, `AUDIO_PLL_151MHz`, `AUDIO_PLL_172MHz`). Pick the one that matches your sample rate family (48 kHz or 44.1 kHz) and whether you need the MCLK output.

```c
#include "utils/clock.h"
#include "utils/tick.h"

// Already done at boot by system_init() (HSE_PLL_100MHz). Only call these to change it.
system_core_clock_config(HSE_PLL_48MHz);    // switch to 48 MHz for USB
audio_pll_clock_config(AUDIO_PLL_76_8MHz);  // only if you use I2S

uint32_t sysclk = get_system_core_clock();  // Hz
uint32_t apb1   = get_apb1_core_clock();
uint32_t apb2   = get_apb2_core_clock();

delay_ms(100);
```

**What a preset does.** Each one sets everything that has to change together, so you can't end up with a half-valid setup:

- Flash wait states and the voltage regulator scale for that speed
- The PLL (always divided down to a 1 MHz input) and the AHB, APB1 and APB2 prescalers
- Safe switching: it moves to the HSI while the PLL is being reconfigured
- The unused oscillator is switched off to save power, and the Clock Security System is turned on whenever the HSE is in use
- The audio PLL is restored afterwards, so changing the system clock doesn't kill I2S
- The tracked clock values and the SysTick reload are updated automatically, so `delay_ms()` and `ms_since_boot()` stay correct after a switch

**Safety checks.** The PLL ranges are checked at compile time and again at run time. If the crystal or PLL fails to start, the clock falls back to the HSI and logs an error, so the board keeps running.

### Logging over RTT

Logs go out through RTT, so you need no UART pin. There are three levels (`LOGI`, `LOGW`, `LOGE`), color coded in the terminal. Set `LOG_LEVEL` to `LOG_LEVEL_NONE` to compile them all out, or any other appropriate level to minimize log output.

```c
#include "utils/log.h"

LOGI("Main", "Started. Temp: %.1f, count: %d", temp, count);
LOGE("Main", "Sensor failed with error %d", err);
```

### Error Propagation macros

```c
TRY*(...) // macros for error propagation for functions that return hal_err_t
          // Read utils/common.h for the full list of available macros
```

### DMA memory copy

`dma_memcpy` copies memory using a DMA stream, so the CPU is free while it runs. You either poll a flag or get a callback.

```c
#include "utils/memcpy.h"

volatile memcpy_state_t done;
TRY(dma_memcpy(dest, src, len, &done));

// ... do other work ...

if (dma_memcpy_wait_for_flag(&done, 1000) == DMA_MEMCPY_DONE) {
    // copy finished
}
```

### Heap

`lib_malloc`, `lib_calloc`, `lib_realloc` and `lib_free` wrap the o1heap allocator. `get_heap_stats` and `print_heap_stats` report peak use and out-of-memory counts. Same semantics as malloc and friends from the C standard library.

### More code samples

**PWM output** (TIM3 channel 1 on PA6, 1 kHz, 50% duty):

```c
#include "drivers/pwm.h"
#include "drivers/timer.h"

TRY(timx_clk_enable(TIM3, true));

pwm_gp_timer_config_t pwm = {
    .pwm_mode       = PWM_MODE_1,
    .pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED,
    .channels = {{
        .output_polarity   = false,
        .output_idle_state = false,
        .channel           = TIMER_CHANNEL_1,
        .gpio_pin          = BOARD_TIM3_CH1_PA6,
    }},
    .num_channels = 1,
};

uint32_t max_duty = 0;
TRY(pwm_gp_timer_init(TIM3, &pwm));
TRY(pwm_start(TIM3, 1000, &max_duty));
TRY(pwm_set_duty_cycle(TIM3, TIMER_CHANNEL_1, max_duty / 2));
```

**ADC.** Reading a channel takes a few setup steps first. They only need to be done once:

```c
#include "drivers/adc.h"

// 1. Clock the ADC peripheral and choose the ADC clock (APB2 / prescaler)
TRY(adcx_clk_enable(ADC1, true));
adc_clk_configure(ADC_CLK_PRESCALER_4);   // keep ADCCLK within the datasheet limit (36MHz max ADCCLK)

// 2. Put the pin into analog mode (PA0 is ADC_CHANNEL_0)
adc_configure_channel(ADC_CHANNEL_0);

// 3. Alignment, resolution and sample time. This also powers the ADC on
adc_config_t cfg = {
    .alignment       = ADC_RIGHT_ALIGN,
    .resolution      = ADC_RES_12_BITS,
    .sampling_cycles = ADC_SAMPLE_84_CYCLES,
};
TRY(adc_configure(ADC1, &cfg));

// 4. Read it: raw counts, then volts
uint16_t raw   = 0;
float    volts = 0;
TRY(adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_0, &raw));
TRY(adc_get_value_right_aligned(ADC1, raw, ADC_RES_12_BITS, &volts));
```

Things worth knowing:

- **Sample time** applies to all the external channels (higher means more accurate on high-impedance sources, but slower). The internal channels always use 480 cycles, because they need a long sampling time.
- **Volts are measured against the real supply.** `adc_get_value_right_aligned()` measures VDDA from the internal reference each time it runs, so it stays accurate even if your 3.3 V rail isn't exactly 3.3 V. The cost is an extra conversion per call.
- **Internal channels:** `adc_get_temp_celsius()`, `adc_get_vdda()`, `adc_get_v_bat()`, `adc_get_temperature()` and `adc_get_v_ref_internal()` need only the setup above. The driver powers the sensor itself. The temperature and VDDA functions switch to 12 bit for the reading, since the factory calibration was done at 12 bit, then restore your resolution.
- **Everything times out** instead of hanging: a stuck conversion returns `HAL_ERR_TIMEOUT`.

**Continuous sampling with DMA** (several channels, double buffered, no CPU work between samples):

```c
static uint16_t buf0[256], buf1[256];
const adc_channel_t seq[] = {ADC_CHANNEL_0, ADC_CHANNEL_1};

adc_configure_channel(ADC_CHANNEL_0);
adc_configure_channel(ADC_CHANNEL_1);
adc_enable_nvic_irq(true);

adc_continuous_config_t cont = {
    .channels         = {.sequence = seq, .num_of_channels = 2},
    .trigger          = ADC_RG_TRIGGER_SOFTWARE,   // or a timer or EXTI trigger
    .trigger_polarity = ADC_POLARITY_NONE,         // needed for hardware triggers
    .buffer_0         = buf0,
    .buffer_1         = buf1,
    .buffer_size      = 256,
    .priority         = DMA_PRIORITY_LOW,
    .circular_mode    = DMA_MODE_DOUBLE_BUFFERS,   // or DMA_MODE_CIRCULAR, DMA_MODE_ONESHOT
    .callbacks = {
        .on_buffer_full = on_buffer_full,          // (void* arg, uint8_t filled_buffer_idx)
        .on_data_overrun = on_overrun,
        .user = NULL,
    },
};
TRY(adc_regular_group_cont_start_conv(ADC1, &cont));
// Do other work ...
TRY(adc_regular_group_cont_end_conv(ADC1));
```

With more than one channel, the samples arrive interleaved in sequence order. The callbacks tell you which buffer just filled. DMA errors and overruns end the conversion, and you restart it by calling `adc_regular_group_cont_start_conv()` again. Starting while a conversion is already running returns `HAL_ERR_INVALID_STATE`.

The **injected group** (`adc_injected_group_start_conv()`) works the same way: up to four channels, a trigger source, optional per-channel offsets, and a completion callback. Read the results with `adc_injected_group_get_result()`. The **analog watchdog** (`adc_analog_wdg_start()`) calls you back when a reading leaves a min/max window.

**I2C** (I2C3 at 100 kHz, talking to an AHT20 sensor):

```c
i2c_master_config_t cfg = {
    .use_pullups    = true,
    .frequency      = I2C_FREQ_100kHz,
    .digital_filter = I2C_DIGITAL_FILTER_0,
    .sda_pin        = BOARD_I2C3_SDA_PB8,
    .scl_pin        = BOARD_I2C3_SCL_PA8,
};

TRY(i2cx_clk_enable(I2C3, true));
TRY(i2c_master_init(I2C3, &cfg));

const uint8_t measure[] = {0xAC, 0x33, 0x00};   // AHT20 "start measurement"
uint8_t       data[7];
TRY(i2c_master_transmit(I2C3, 0x38, measure, sizeof(measure)));
delay_ms(80); // Polling delay. Used as a sample
TRY(i2c_master_receive(I2C3, 0x38, data, sizeof(data)));
```

Check the headers in `components/drivers/` for the full list of config fields and options.

## Pin definitions: `board.h`

`components/utils/board.h` holds everything that belongs to the specific chip and board, so the drivers don't need hard-coded numbers.

**Pin names.** Every usable pin for every peripheral is a named definition, in the form `BOARD_<PERIPHERAL>_<SIGNAL>_<PIN>`. Each one holds the GPIO port, the pin number and the alternate function (AF) number in a `board_pin_t`:

```c
typedef struct {
    GPIO_TypeDef* port;
    gpio_pin_t    pin;
    uint8_t       af;
} board_pin_t;

#define BOARD_UART1_TX_PA9   {GPIOA, GPIO_PIN_9, 7}
#define BOARD_UART1_TX_PB6   {GPIOB, GPIO_PIN_6, 7}
#define BOARD_I2C2_SDA_PB11  {GPIOB, GPIO_PIN_11, 4}
#define BOARD_I2C2_SDA_PB3   {GPIOB, GPIO_PIN_3, 9}
```

This helps in a few ways:

- **No datasheet lookup.** You don't need to know that USART1 TX on PA9 is AF7. You pick the pin by name and the AF comes with it. It also handles the odd cases, like I2C2 SDA using AF4 on PB11 but AF9 on PB3.
- **Typos don't compile.** If a peripheral can't use a pin, there is no definition for it, so the mistake is caught at build time and not on the bench.
- **The driver does the pin setup.** You pass the pin in the config struct. The driver turns on the port clock and sets the mode, AF, speed and pull-up itself.
- **Moving a signal is one line.** To move UART1 TX from PA9 to PB6, change the name in the config and nothing else.

```c
.tx_pin = BOARD_UART1_TX_PA9,   // change to BOARD_UART1_TX_PB6 to move it
```

Pin definitions cover UART1, 2 and 6, I2C1 to 3, SPI1 to 5, I2S2 to 5 and timers TIM1 to TIM5 and TIM9 to TIM11, including the complementary and break pins on TIM1. ADC pins are not in this list: they follow the channel number (`ADC_CHANNEL_0` is PA0), and `adc_channel_get_gpio()` returns the pin.

**Chip constants.** The same file holds the numbers the drivers need, taken from the datasheet and reference manual:

- Oscillator values: HSE is 25 MHz (the Black Pill crystal), HSI 16 MHz, LSI 32 kHz and LSE 32.768 kHz
- Clock limits: 100 MHz system, 50 MHz APB1, 100 MHz APB2 and 192 MHz audio PLL
- ADC internals: the temperature sensor constants, the Vbat divider ratio, startup times and the addresses of the factory calibration values
- I2C: the bus rise times and the minimum APB1 clock for 100 kHz and 400 kHz

The clock code, for example, builds its PLL settings from `HSE_VALUE_MHz` and checks them at compile time. If your board has a different crystal, change that one number. The file is guarded by `#if defined(STM32F411xE)` and fails with an `#error` for any other chip, so it is also where you would add another target.

## What runs before `main`

The startup file (`stm32f411_startup.s`) handles the reset. In order, it:

1. Sets up the stack pointer
2. Copies `.data` from flash to RAM and zeroes `.bss`
3. Calls `system_init()` (below)
4. Initializes the heap which runs as a constructor function (`[[__gnu__::__constructor__(101)]]`)
5. Runs C++ static constructors
6. Calls `main()`

**`system_init()`** (in `utils/src/syscalls.c`) gets the chip ready, so `main` starts in a known, fast state:

- Turns on the instruction and data caches and the prefetch buffer
- Turns on the FPU
- Makes division by zero a fault
- Enables the MemManage, bus fault and usage fault exceptions
- Starts RTT logging, SysTick and the DWT cycle counter
- Sets the system clock to **100 MHz from the HSE** and disables the audio PLL
- Logs the resulting clock speeds

So your application doesn't need to set up the clocks. It starts at 100 MHz and can switch if it needs to. If you don't provide a `main()`, a weak stub logs an error and halts.

By default, the application has a heap size of 32kB. Set to 0 (in utils/common.h) to disable the heap). 

**Interrupt vectors.** Every exception and interrupt has a weak default handler, so you only define the ones you use. Any unhandled interrupt lands in a default handler that masks interrupts and sleeps.

### Tick and delays

SysTick runs at 1 kHz (from HCLK/8) at the highest interrupt priority.

| Function | What it does |
|---|---|
| `ms_since_boot()` | Milliseconds since boot, as a 64-bit value, safe to read from anywhere |
| `delay_ms(ms)` | Busy-wait delay based on the tick |
| `delay_us(us)` | Busy-wait delay based on the DWT cycle counter |
| `prof_start()` / `prof_end()` | Cycle counting helpers for profiling |

A clock change re-programs SysTick automatically, so these stay accurate.

### Fault handling

The four CPU fault handlers (HardFault, MemManage, BusFault and UsageFault) all lead to one decoder, `decode_fault()`. A short assembly stub works out whether the fault happened on the main or process stack, and tells the decoder which fault it was. That matters because a HardFault can be a real one or one escalated from a disabled handler.

The decoder reads the fault status registers and logs the **specific cause**, not a raw hex dump. Examples: "Precise data bus error: the PC is the faulting instruction", "Division by zero", "Coprocessor access fault (FPU used without CPACR enabling it)" and "Possible stack overflow into a guarded region". When the hardware recorded the bad address, it prints that too (`MMFAR` or `BFAR`). It then prints the stacked registers (R0 to R3, R12, LR, PC, PSR), `EXC_RETURN` and `CFSR`.

After logging, it clears the fault flags, then **halts at a breakpoint in debug builds** and **reboots in release builds**.

To find the line that crashed, take the logged `PC` and run:

```
arm-none-eabi-addr2line -e build/tests-debug/f411-hal.elf 0x<PC>
```

`decode_fault()` is weak, so an application can replace it with its own recovery logic.

**NMI and the Clock Security System.** The NMI handler deals with the clock monitor. If the crystal fails while running, the hardware switches the system to the HSI by itself. The handler then reconfigures the PLL to the HSI equivalent of whatever preset was active (for example, `HSE_PLL_100MHz` becomes `HSI_PLL_100MHz`), so the board keeps running at the same speed.

### Syscalls, `printf` and the heap

`syscalls.c` provides the stubs that newlib needs so `printf` and friends work on bare metal:

- **Output goes to RTT.** `_write()` and the `printf` library's `putchar_()` both write to the RTT buffer, so `printf` shows up in the same console as `LOGI`.
- **No newlib heap.** `_sbrk()` always fails, so plain `malloc` is deliberately unavailable. Use `lib_malloc` and friends from `utils/heap.h` (o1heap).
- **Everything else is a stub.** `_read`, `_close`, `_lseek`, `_fstat`, `_isatty`, `_kill` and `_getpid` just report an error. `_exit()` logs the exit code and panics.
- **All of them are weak**, so your application can override any of them.

`utils/common.h` also gives you `ASSERT(cond)`, `PANIC()`, `REBOOT()` and `halt()` (which stops at a breakpoint in debug builds).

## Tests

The tests use [Unity](https://github.com/ThrowTheSwitch/Unity) and run **on the Black Pill itself**. You flash the test firmware and read the results over RTT.

### What the tests cover

| Driver | What is tested |
|---|---|
| GPIO | Every mode setter, alternate functions, pull-ups, interrupt setup for every port and edge, shared and dedicated EXTI handlers, software-triggered interrupts |
| DMA | Stream lookup for every stream, flag decoding, null-safe setters, end-to-end memory-to-memory transfers, `dma_memcpy` with flag and with callback |
| Timers | Clock enable for every timer, init and deinit, up and down counting, one-shot and periodic, pause, resume, restart, shared IRQ lines |
| PWM | Register setup for every channel, edge and center aligned frequency accuracy, dead-time range, break input and callback, PWM input mode |
| Input capture and output compare | Period and pulse width measurement, edge selection, digital filter, input prescaler, overcapture, compare callbacks, toggle output |
| Pulse counter and quadrature | Counting on either edge, filter, prescaler, overflow callback, direction, inverted direction |
| ADC | Every resolution, alignment and sample time, prescalers, one-shot reads, internal channels, continuous DMA, injected group, analog watchdog, overrun handling |
| UART | Clock enable, invalid arguments, polling send, DMA send and receive with callbacks, deinit |
| SPI | Polling loopback at 8 and 16 bit, all four CPOL/CPHA modes, DMA with callback, MOSI-only and MISO-only |
| I2C | Fast mode selection, digital filter, bus busy and missing-device errors, 1 and 2 byte reads, repeated sensor reads, init/deinit stress |
| I2S | Prescaler and clock limits, frame sizes, MCLK pin, one-shot send and receive, double-buffer DMA |
| CRC | CPU path and DMA path against a software reference |
| IWDG | Reload and prescaler values, boundary limits, reset flag, debug freeze |
| Clock | Every system and audio preset, VCO limits, exact 48 MHz for USB, idempotent reconfiguration |
| Heap | Invalid arguments, zeroed `calloc`, `realloc` data preservation, packing at full capacity |

Most suites also check that bad arguments are rejected cleanly and that deinit puts the registers back.

### Choosing which tests run

Open `components/test/src/runner.cpp`. The test groups are listed there, and you uncomment the ones you want. I keep most of them commented out while I work on one area, so **only the groups left uncommented will run**. Two more entries in `test_main.cpp`, `profile::all()` and `demo::all()`, are also off by default.

### Build, flash and read the results

```
cmake --preset tests-debug
cmake --build --preset tests-debug
cmake --build --preset tests-debug --target flash
```

The `flash` target uses OpenOCD with an ST-Link. The pass and fail output is printed through RTT.

### Debug with GDB and the RTT console

`scripts/gdb.sh` does the whole build, flash and debug routine in one command. Run it from the repo root:

```
./scripts/gdb.sh
```

It does the following:

1. Builds the `tests-debug` preset and flashes it.
2. Starts OpenOCD in the background with the ST-Link (its log goes to `/tmp/openocd.log`) and waits for the GDB port (3333).
3. Opens a new terminal window showing the RTT log output (connected to port 9090). It picks `$TERMINAL` if you set it, or else the first one it finds from `gnome-terminal`, `kgx` (GNOME Console), `konsole`, `xfce4-terminal`, `alacritty` or `xterm`. If none is found, it tells you to run `nc localhost 9090` yourself.
4. Starts `arm-none-eabi-gdb` with `.gdbinit`, which connects to OpenOCD, resets and halts the chip, loads the firmware, loads the SVD file, runs to `main`, starts the RTT server and continues.
5. When you exit GDB, OpenOCD is shut down automatically.

I wrote and use it on Arch with GNOME, so the terminal detection is tuned for that setup. If your terminal isn't picked up, set it yourself, for example `TERMINAL=gnome-terminal ./scripts/gdb.sh`.

**What you need:** `openocd`, `arm-none-eabi-gdb`, netcat (`nc`) and an ST-Link. For the register viewer in `.gdbinit`, also clone the [PyCortexMDebug](https://github.com/bnahill/PyCortexMDebug) GDB plugin into `~/.gdb-scripts/PyCortexMDebug`. It reads `.vscode/STM32F411.svd`, so you can inspect peripheral registers by name from GDB (for example `svd GPIOA`).

Inside GDB, the `flash` command (defined in `.gdbinit`) reloads the freshly built ELF and restarts, without leaving the session.

**VS Code:** `.vscode/` has a Cortex-Debug launch config that does the same job. It builds, flashes through OpenOCD, loads the SVD for a register view, enables live watch and shows the RTT log in the editor.

### Test wiring

Some tests need jumper wires or a sensor. Others need nothing at all.

**Tests that need wiring:**

| Test | Wiring |
|---|---|
| UART (USART1) | PA9 (TX) to PA10 (RX) |
| SPI (SPI2) | PB15 (MOSI) to PB14 (MISO). PB10 is the clock and needs no connection |
| I2C (I2C3) | AHT20 sensor: SDA on PB8, SCL on PA8, address `0x38`. The MCU's internal pull-ups are used |
| Timer loopback | See below |

**Timer loopback (TIM3 drives TIM2):**

TIM3, a 16-bit timer, is the signal source. TIM2, a 32-bit timer, is the one being tested. The pins are wired together like this:

| TIM3 output (source) | TIM2 input (tested) |
|---|---|
| PB4 (CH1) | PA5 (CH1) **and** PA15 (ETR) |
| PB5 (CH2) | PB3 (CH2) |

Sometimes TIM3 generates a real clock or PWM signal. Other times the CPU takes over PB4 and PB5 as plain GPIO outputs and bit-bangs exact pulse trains and quadrature sequences, so the expected counts are exact, not approximate. The input capture, output compare, pulse counter, quadrature decoder and PWM input tests all use this loopback.

**Tests that need no wiring:** ADC, I2S, DMA, CRC, GPIO, IWDG, clock, heap, the timer basics and the core PWM output tests. They check registers, internal channels and on-chip behavior. For PWM output you can probe the pin with a scope if you want to see the signal, but nothing needs to be connected.

## PWM demo

`components/test/src/demo.cpp` is a demo for a logic analyzer or oscilloscope. It does not test anything. It runs a complementary PWM sweep on TIM1, with dead time between each output and its complement. To run it, uncomment `demo::all()` in `test_main.cpp`.

| Signal | Pin |
|---|---|
| CH1, CH2, CH3, CH4 | PA8, PA9, PA10, PA11 |
| CH1N, CH2N, CH3N (complementary) | PB13, PB14, PB15 |
| Break input (BKIN) | PA6 |

Defaults: 20 kHz, 1000 ns dead time, center aligned, and a triangle wave sweeping the duty cycle from 0 to 100% in 100 steps (20 ms each). The channels are spread evenly in phase. All of this can be changed in the config struct at the top of the file. Other options are ramp up and ramp down, a duty floor and ceiling, and a fixed phase lag between channels. Pull the break pin low and the outputs shut off, and a warning is logged.

## Profiling

Two reports are in `profile_results/`. Both ran on the Black Pill at 100 MHz and used the DWT cycle counter for timing. The runs are repeatable, so reruns give identical numbers. To run them, uncomment `profile::all()` in `test_main.cpp`.

**o1heap allocator** (`o1heap_malloc.md`): 1,000 random allocations of 8 to 511 bytes in a 32 KB arena, with 64 allocations live at all times.

| Build | Alloc average | Alloc worst case | Free average | Free worst case |
|---|---|---|---|---|
| `-O0`, cache on | 4,073 cycles | 5,449 cycles | 3,057 cycles | 5,192 cycles |
| `-O3`, cache on | 1,368 cycles | 1,757 cycles | 1,022 cycles | 1,707 cycles |

Optimization level matters most (about 3x faster at `-O3`). Turning the cache off adds a roughly fixed cost per call. Masking interrupts made almost no difference. The o1heap project quotes about 120 cycles on a Cortex-M33, and this test did not reproduce that on the Cortex-M4F. The report discusses why.

**RTT logging** (`segger_rtt_logging.md`): a log line with five arguments (including a `%.3f` double) at 40 Hz, with a 6 KB RTT buffer.

| Build | Average per log call | Worst case |
|---|---|---|
| `-O0` | about 275 us | about 279 us |
| `-O2` | about 113 us | about 116 us |

The three RTT modes (skip, trim, block) were equivalent at this rate, because the buffer never filled. Adding one `%.3f` double costs about 14 to 19% extra. The report also warns about the case where the buffer fills: `BLOCK_IF_FIFO_FULL` once stalled the CPU for about 808 ms in an unthrottled test.

## Building

Needs:

- STM32F411 Black Pill
- An ST-Link (or compatible) debug probe and OpenOCD
- Arm GNU Toolchain (`arm-none-eabi-gcc`, developed with 16.1)
- CMake 4.0 or newer, plus Ninja or Make

| Preset | What it builds |
|---|---|
| `debug` | `libf411-hal.a`, no optimization, with debug info |
| `release` | `libf411-hal.a`, optimized for size (`-Os`) with LTO |
| `tests-debug` | The test firmware ELF, debug |
| `tests-release` | The test firmware ELF, release |

```
cmake --preset release
cmake --build --preset release
```

The library is built as C23 and the tests as C++23.

### Third-party libraries

Third-party code lives in `components/third-party/` and is part of the repository:

| Folder | What it is |
|---|---|
| `CMSIS` | Arm and ST device headers ([CMSIS Headers](https://github.com/STMicroelectronics/cmsis-device-f4)) |
| `Unity` | Test framework ([Unity](https://github.com/throwtheswitch/unity)) |
| `RTT` | SEGGER RTT, for logging ([SEGGER RTT](https://github.com/SEGGERMicro/RTT)) |
| `o1heap` | The heap allocator ([o1heap](https://github.com/pavel-kirienko/o1heap)) |
| `printf` | Embedded `printf` ([printf](https://github.com/mpaland/printf)) |

## Code checks

```
cmake --build --preset tests-debug --target format     # format the code
cmake --build --preset tests-debug --target cppcheck   # static analysis
```

CI runs on every push. It builds all four presets, checks formatting with clang-format and runs cppcheck. It does not run the hardware tests, because they need a board.

## Folder layout

```
components/
  drivers/        Peripheral drivers (headers + src/)
  utils/          Clocks, tick, logging, heap, DMA memcpy, error codes
  test/           Hardware tests and the PWM demo
  third-party/    CMSIS, Unity, RTT, o1heap, printf
profile_results/  Heap and logging benchmarks
scripts/gdb.sh    Build, flash, GDB and RTT console in one command
.gdbinit          GDB setup used by gdb.sh
.vscode/          Cortex-Debug config, build tasks and the STM32F411 SVD file
test_main.cpp     Entry point for the test firmware
stm32f411_startup.s, stm32f411_linker.ld
```

## License

MIT. See `LICENSE`.
