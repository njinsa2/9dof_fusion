#ifndef FUSION_H_
#define FUSION_H_


#include <stdint.h>

void Mahony_Update(float *, float *, float *, float *, uint8_t);
void Mahony_GetEuler(const float *, float *, float *, float *);
void Mahony_Init(float *, float *, float *);


void AHRS_Reset_Benchmark(void);
uint8_t AHRS_IsSettled(void);
uint32_t AHRS_GetTransientTimeMs(void);
#endif
