#include "utils/random.h"
#include "utils/common.h"

#include <stddef.h>


void rng_seed(pcg32_rng_t* rng, uint64_t seed, uint64_t seq) {
    if (rng) {
        rng->state = 0;
        rng->inc   = (seq << 1U) | 1U;
        rng_get(rng); // Discard once to mix
        rng->state += seed;
        rng_get(rng);
    }
}

uint32_t rng_get(pcg32_rng_t* rng) {
    if (rng) {
        const uint64_t old        = rng->state;
        rng->state                = old * 6364136223846793005ULL + rng->inc;
        const uint32_t xorshifted = (uint32_t)(((old >> 18U) ^ old) >> 27U);
        const uint32_t rot        = (uint32_t)(old >> 59U);
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }
    return UINT32_MAX;
}

uint32_t rng_get_range(pcg32_rng_t* rng, uint32_t min_inclusive, uint32_t max_inclusive) {
    if (rng == NULL || max_inclusive <= min_inclusive) {
        return UINT32_MAX;
    }
    return min_inclusive + ((rng_get(rng)) % (max_inclusive - min_inclusive + 1));
}
