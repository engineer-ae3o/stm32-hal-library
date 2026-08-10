# RTT Logging System: Profiling Report

**Target:** STM32F411 (Cortex-M4F, Core clock speed of 100 MHz)
**Transport:** SEGGER RTT, UP buffer channel 0, 6kB buffer (bumped up from the 1kB default)
**Timing method:** The DWT cycle counter present on most ARM Cortex-M cores

---

## 1. Test format string

```c
LOGI("Profile_Logs",
     "Typical log. Lets also do this: %.3f,%d,%d. Even more stuff. Add a format string as well. %s long enough. Iteration: %lu",
     3.0, 4, 5, "This should be", i);
```

5 variadic arguments: one `double`, two `int`, one `const char*`, one `uint32_t`. This is intentionally near the worst case for a typical log line as real world calls with fewer/simpler arguments should be faster on average.

---

## 2. Results: 1,000 sample run (primary dataset)

40 Hz call rate (25 ms delay between logs), the 3 RTT modes and optimization levels of O0 and O2.

| RTT mode | Opt | WCET (μs) | Best (μs) | Avg (μs) |
|---|---|---|---|---|
| `NO_BLOCK_SKIP` | -O0 | 278.230 | 253.690 | 275.530 |
| `NO_BLOCK_TRIM` | -O0 | 278.140 | 234.750 | 275.490 |
| `BLOCK_IF_FIFO_FULL` | -O0 | 278.680 | 253.470 | 275.300 |
| `NO_BLOCK_SKIP` | -O2 | 115.960 | 104.180 | 113.120 |
| `NO_BLOCK_TRIM` | -O2 | 115.370 | 103.500 | 112.960 |
| `BLOCK_IF_FIFO_FULL` | -O2 | 116.850 | 105.150 | 114.640 |

**RTT mode has no measurable effect** at this write rate/buffer size: spread across modes is <2 μs at both optimization levels: noise, not signal. All three modes gave statistically equivalent results, confirming the CPU never needed to block on the buffer at 40 Hz with a 6kB buffer.

**The optimization level has a large, consistent effect:** ~2.42× speedup from -O0 to -O2 (275.44 μs → 113.57 μs average, across modes).

---

## 3. Cross checks against earlier runs

Two earlier datasets used a shorter format string (5 args: 3×`%d`, 1×`%s`, 1×`%lu` (no float/double)) and are cited here to confirm the 1,000 sample result generalizes across sample sizes:

| Run | Samples | Rate | Args | WCET (μs) | Best (μs) | Avg (μs) |
|---|---|---|---|---|---|---|
| Steady-state, -O0 | 10,000 | 40 Hz | no float | 245.350 | 227.440 | 242.180 |
| Steady-state, -O2 | 10,000 | 40 Hz | no float | 97.070 | 89.290 | 95.720 |
| Steady-state, -O0 | 100 | ~66 Hz | no float | 246.240 | 228.860 | 242.610 |

The 100 sample and 10,000 sample runs agree closely with each other (avg 242.61 μs vs. 242.18 μs): sample size does not materially change the measured per call cost once the RTT buffer isn't under backpressure. This gives confidence that the 1,000 sample float dataset above is not an artifact of sample count either.

An additional 10,000 sample **unthrottled burst** run (no delay between calls) is *not* used as a baseline: it measured RTT buffer backpressure, not logging cost (see Caveats, §5).

---

## 4. Cost of `%.3f` double formatting

Isolated by comparing the float inclusive 1,000 sample dataset (§2) against the non float baseline (§3), same call rate:

| Opt | Non-float avg (μs) | Float avg (μs) | Δ (μs) | Δ (%) |
|---|---|---|---|---|
| -O0 | 242.18 | 275.44 | +33.26 | +13.7% |
| -O2 | 95.72 | 113.57 | +17.85 | +18.7% |

Adding a single `%.3f` double argument adds a consistent cost: larger in absolute terms at -O0, larger in relative terms at -O2 (float formatting appears to benefit less from -O2's optimizations than the integer/string handling does). It is not the dominant cost in either case: it remains under 20% of the total call time: but it is the single most expensive individual format specifier tested.

---

## 5. Caveats and assumptions

- **Variadic float promotion:** the C/C++ default argument promotion rules mean any `float` passed through `LOGI`'s `...` is promoted to `double` regardless of the argument's declared type. The test used `double` directly (`3.0`), so this measures `%.3f` on double formatting cost.
- **Precision:** `%.3f` (3 decimal digits) was used deliberately, not the printf default of 6 digits. A different precision would shift the float formatting cost somewhat.
- **RTT lock mode:** tests used `SEGGER_RTT_WriteNoLock`. Safe for this profiling context because logging only occurs from thread context, never from an ISR, in this test.
- **Buffer size:** 6kB, not the SEGGER default of 1kB. Real world buffer backpressure behavior (§ below) is a function of this size vs. write rate; changing the buffer size changes the frequency at which blocking/backpressure would occur.
- **RTT mode equivalence is rate dependent, not universal.** The finding "mode doesn't matter" holds specifically because the buffer never filled at 40 Hz with 6kB of headroom. At higher write rates, or with a smaller buffer, or with a slower/disconnected host drain, mode selection matters a great deal:
  - `BLOCK_IF_FIFO_FULL` stalls the CPU until the host drains the buffer. An earlier unthrottled burst test (10,000 calls, no delay, no float, smaller buffer) produced an outlier of **~808 ms**: this is the real observed worst case under buffer saturation.
  - `NO_BLOCK_SKIP`/`NO_BLOCK_TRIM` avoid the stall but silently drop or truncate log data instead.
- **Real-world call rate:** the 40 Hz test rate is higher than expected real world logging frequencies. At realistic (lower) rates, the 6kB buffer would have even more headroom, making buffer saturation blocking/data loss less likely: but not impossible if a burst of log calls (e.g., an error cascade) occurs.
- **ART accelerator and instruction cache (1kB icache, 128B dcache, and a prefetch buffer of 16B) was not isolated or controlled for.** This does not undermine the mode or optimization level comparisons: cache/prefetch state is constant across the RTT-mode variants (identical code, differing only in a config flag), and any warm up transient is amortized away across 1,000 identical loop iterations. It would matter for an *absolute* cross part or cross toolchain performance claim, which this report does not make.
- **`printf` backend float support:** the embedded printf implementation used was the one provided by [mpaland/printf](https://github.com/mpaland/printf "printf(...) and snprintf(...) implementation for embedded systems")
