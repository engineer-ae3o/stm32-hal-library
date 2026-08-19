#ifndef RANDOM_H_
#define RANDOM_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef struct {
    uint64_t state;
    uint64_t inc;
} pcg32_rng_t;

void     rng_seed(pcg32_rng_t* rng, uint64_t seed, uint64_t seq);
uint32_t rng_get(pcg32_rng_t* rng);
uint32_t rng_get_range(pcg32_rng_t* rng, uint32_t min_inclusive, uint32_t max_inclusive);


#ifdef __cplusplus
}
#endif


#endif // RANDOM_H_