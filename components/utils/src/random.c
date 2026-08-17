#include "utils/random.h"

#include <stdatomic.h>


_Atomic uint64_t rng_state = 0xC0FFEE01C0FFEE01ULL;

uint64_t random_get() {
    uint64_t rand = (rng_state += 0x9E3779B97F4A7C15ULL);

    rand = (rand ^ (rand >> 30)) * 0xBF58476D1CE4E5B9ULL;
    rand = (rand ^ (rand >> 27)) * 0x94D049BB133111EBULL;
    return rand ^ (rand >> 31);
}
