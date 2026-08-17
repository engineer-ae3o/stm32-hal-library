# O1Heap Allocator: Profiling Report

**Target:** STM32F411 (Cortex M4F, core clock speed of 100 MHz)
**Allocator:** o1heap, arena size 32 KB, requires O1HEAP_ALIGNMENT aligned base
**Timing method:** The DWT cycle counter present on most ARM Cortex M cores

---

## 1. Test workload

```c++
constexpr uint32_t SAMPLE_SIZE = 1000;
constexpr uint32_t POOL_SIZE   = 64;   // outstanding allocations kept live
constexpr size_t   MIN_ALLOC   = 8;
constexpr size_t   MAX_ALLOC   = 512;
```

Each iteration frees a pseudo randomly chosen slot from a pool of 64 outstanding allocations, then immediately allocates a new pseudo random size (8 to 511 bytes) into that same slot. The pool starts primed with 64 live allocations so the heap is fragmented and non empty from the first measured sample, not starting from a clean, unfragmented arena.

The PRNG is a fixed seed xorshift32 (seed 0xC0FFEE01), chosen instead of `rand()` to avoid libc call overhead polluting the timing. Because the seed is fixed, the entire sequence of sizes and free slot choices is fully deterministic. Every rerun of an identical build reproduces identical cycle counts for every sample, confirmed by repeated resets, see section 4.

Alloc and free are timed as two separate operations per iteration, each wrapped individually in `prof_start()`/`prof_end()`, with interrupts masked around the measured region in the later runs (see section 3).

---

## 2. Results across build configurations

| Config | Op | WCET (cycles) | Best (cycles) | Avg (cycles) | Spread (cycles) |
|---|---|---|---|---|---|
| -O0, cache on | Alloc | 5449 | 3470 | 4073 | 1979 |
| -O0, cache on | Free | 5192 | 2562 | 3057 | 2630 |
| -O0, cache off | Alloc | 6228 | 3993 | 4693 | 2235 |
| -O0, cache off | Free | 6064 | 2970 | 3550 | 3094 |
| -O3, cache on (IRQ unmasked) | Alloc | 1813 | 1191 | 1369 | 622 |
| -O3, cache on (IRQ unmasked) | Free | 1718 | 879 | 1034 | 839 |
| -O3, cache on (IRQ masked) | Alloc | 1757 | 1190 | 1368 | 567 |
| -O3, cache on (IRQ masked) | Free | 1707 | 868 | 1022 | 839 |
| -O3, cache off | Alloc | 2681 | 1776 | 2060 | 905 |
| -O3, cache off | Free | 2644 | 1292 | 1538 | 1352 |

Spread is WCET minus Best, included here as a proxy for how much the cost of an individual call varies across the run, not just its average.

**Optimization level has the largest effect.** -O0 to -O3, cache on: alloc average drops from 4073 to 1368 cycles, roughly a 3.0x speedup. Free drops from 3057 to 1022, also roughly 3.0x. This roughly matches the magnitude seen in the RTT logging report for the same build flag change.

**Disabling the instruction and data caches adds a roughly fixed absolute cost per call, independent of optimization level.** At -O0, cache off adds about 620 cycles to alloc average and 493 to free average. At -O3, cache off adds about 692 cycles to alloc average and 516 to free average. The two deltas are close to each other despite the -O3 baseline being roughly a third the size of the -O0 baseline, so the cache cost reads as a small percentage at -O0 (about 15 percent) and a large percentage at -O3 (about 51 percent). The percentage difference is a consequence of the smaller -O3 denominator, not evidence that the cache matters more to more optimized code in absolute terms.

**Masking interrupts around the timed region had little effect.** Alloc WCET moved from 1813 to 1757 cycles, a 3 percent change. Free WCET was unchanged at the cycle level within rounding. This rules out interrupt preemption as a significant contributor to the spread seen in every configuration.

---

## 3. Interrupt and cache isolation

Masking interrupts around the timed region had little effect. Alloc WCET moved from 1813 to 1757 cycles, a 3 percent change. Free WCET was unchanged at the cycle level within rounding. Interrupt preemption is not a significant contributor to the spread seen in any configuration.

Disabling the instruction and data caches did not close the gap to the published figure either. Cache off increases the average and WCET cost at every optimization level tested, it does not reduce it. If the published figure assumed cache disabled or a colder cache state than this test's steady state loop, the actual numbers here move further from that figure, not closer.

---

## 4. Determinism

Every configuration was rerun multiple times via board reset. Within a given build (same optimization level, same cache state, same interrupt masking), every rerun produced bit identical results across every reported statistic, including WCET, Best, Average, and Total. This is expected given the fixed PRNG seed and a heap that starts in the same state on every boot, and it has a direct consequence for interpretation.

Because the run is fully deterministic, the reported WCET for a given configuration is not a statistical estimate with an associated confidence bound. It is the exact worst case cycle count for that specific fixed input sequence, pool size, and size range. Changing the seed, pool size, or size range could produce a different exact worst case, not a resampling of the same underlying distribution.

---

## 5. Comparison against the published figure

The o1heap project documents an allocation cost of approximately 120 cycles on an RP2350 (Cortex M33). The best result obtained in this report, -O3 with cache enabled, is roughly 1368 cycles average and 1757 cycles WCET for alloc, and roughly 1022 cycles average and 1707 cycles WCET for free. That is an order of magnitude higher than the published figure.

The published figure is not replicated by this test, on this part, under any of the four build configurations measured, including the one closest to how o1heap would actually be deployed in a release build. Cortex M4F versus Cortex M33 accounts for some of the gap, but a difference of roughly 10 to 15x is a large amount to attribute to core and clock differences alone.

---

## 6. Caveats and assumptions

- **Arena sizing.** With a pool of 64 outstanding allocations and a maximum request size of 512 bytes, the worst case live allocation footprint is close to the 32 KB arena boundary once per block header overhead is included. No out of memory condition was observed in the runs recorded here, but the margin is not large.
- **Fragmentation pattern is one specific churn pattern, not an adversarial one.** The random free and reallocate pattern used here produces a plausible, moderately fragmented heap, but does not specifically construct the worst case bin occupancy pattern the allocator's design would need to defend against.
- **Cache off configurations do not represent real deployment conditions** and are included only to check whether disabling the cache would make a substantial difference. It does not.
