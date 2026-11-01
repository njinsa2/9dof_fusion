#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <stdint.h>

void DWT_Init(void);
void Benchmark_Mahony(float *, float *, float *, float *, uint8_t);
#endif
