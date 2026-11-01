#include "benchmark.h"
#include "stm32f4xx_hal.h"
#include "fusion.h"
#include "usb.h"

//initialize DWT hardware peripheral
void DWT_Init(void) {
    // Enable Trace Unit (TRCENA) in CoreDebug DEMCR register
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset cycle counter register to 0
    DWT->CYCCNT = 0;

    // Enable cycle counter inside DWT Control register
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Benchmark_Mahony(float *q, float *accel_g, float *gyro_rad, float *m_gauss, uint8_t mag_data_ready) {
    uint32_t start_cycles;
    uint32_t elapsed_cycles;
    float execution_time_us;

    // Capture starting cycle count
    start_cycles = DWT->CYCCNT;

    // Run sensor fusion update
    Mahony_Update(q, accel_g, gyro_rad, m_gauss, mag_data_ready);

    // Compute elapsed CPU clock cycles
    elapsed_cycles = DWT->CYCCNT - start_cycles;

    // Convert clock cycles to microseconds (us)
    execution_time_us = (float)elapsed_cycles / (SystemCoreClock / 1000000.0f);
    USB_Print("Execution time: %f\n", execution_time_us);
}
