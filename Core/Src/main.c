#include "main.h"
#include "usb.h"
#include "usb_device.h"
#include "peripheral.h"
#include "stm32f4xx_hal.h"
#include "ism330dhcx.h"
#include "mmc5983ma.h"
#include "fusion.h"
#include "benchmark.h"

volatile uint8_t imu_data_ready = 0;
volatile uint8_t mag_data_ready = 0;
volatile uint8_t imu_tick_count = 0;

volatile uint8_t mag_trigger = 0;

static uint8_t  waiting = 0;
static uint32_t t_trig  = 0;

static float m_gauss[3];
static float accel_g[3];
static float gyro_rads[3];

float q[4] = {1, 0, 0, 0};

int main(void)
{
	Peripheral_Init();
	HAL_Delay(3000);

	//uncomment for dwt benchmarking
	//DWT_Init();

	//boot-up initial static orientation sampling
	MMC_Trigger_Measurement();
	HAL_Delay(20);
	if (MMC_Read_Status() & STATUS_MEAS_M_DONE) {
		MMC_Read_Data(m_gauss);
		//sensor axes are physically misaligned, z_accel = -z_mag
		m_gauss[2] = -m_gauss[2];
	}
	ISM_Read_Data(accel_g, gyro_rads);

	//seed quaternion with ~zero-transient orientation
	Mahony_Init(q, accel_g, m_gauss);


	// 1. Start benchmark clock
	AHRS_Reset_Benchmark();

	while (1)
	{
		if(imu_data_ready){
			imu_data_ready = 0;
			ISM_Read_Data(accel_g, gyro_rads);

			//uncomment for accel calibration (comment out the quaternion stream)
			//ISM_Read_Data_NonCalibrated(accel_g, gyro_rads);
			//USB_Print("%f, %f, %f\r\n", accel_g[0], accel_g[1], accel_g[2]);

			//uncomment for gyro calibration (comment out the quaternion stream)
			//ISM_Read_Data_NonCalibrated(accel_g, gyro_rads); //gyroscope readings are in dps, which fits the calibration code in ism330dhcx.h
			//USB_Print("%f, %f, %f\r\n", gyro_rads[0], gyro_rads[1], gyro_rads[2]);

			//uncomment for DWT benchmarking of the mahony filter (comment out Mahony_Update call)
			//Benchmark_Mahony(q, accel_g, gyro_rads, m_gauss, mag_data_ready);

			Mahony_Update(q, accel_g, gyro_rads, m_gauss, mag_data_ready);
			if (AHRS_IsSettled()) {
			        uint32_t t_settled = AHRS_GetTransientTimeMs();
			        USB_Print("T_settled = %d\r\n", t_settled);
			    }

			USB_Print("%f, %f, %f, %f\r\n", q[0], q[1], q[2], q[3]);
			if(mag_data_ready){
				mag_data_ready = 0;
			}
		}

		if (mag_trigger)
		{
			mag_trigger = 0;
			MMC_Trigger_Measurement();
			t_trig  = HAL_GetTick();
			waiting = 1;
		}

		if (waiting)
		{
			uint32_t elapsed = HAL_GetTick() - t_trig;
			//don't poll for 4ms (the measurement time of the magnetometer)
			if(elapsed >= 5 && (MMC_Read_Status() & STATUS_MEAS_M_DONE))
			{
				waiting = 0;
				MMC_Read_Data(m_gauss);
				mag_data_ready = 1;

				//uncomment for magnetometer calibration (comment out the quaternion stream and MMC_Read_Data call above)
				//MMC_Read_Data_NonCalibrated
				//USB_Print("%f, %f, %f\r\n", m_gauss[0], m_gauss[1], m_gauss[2]);

				//sensor axes are physically misaligned, z_accel = -z_mag
				m_gauss[2] = -m_gauss[2];

			}
			else if (HAL_GetTick() - t_trig > 20)
			{
				//skip the sample instead of hanging after timeout
				waiting = 0;
			}
		}

	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //imu interrupt fires at 208Hz master rate
	if (GPIO_Pin == AINT_Pin)
    {
        //master timing decimation for the magnetometer
        imu_tick_count++;
        imu_data_ready = 1;

        //~69.3Hz update rate
        if (imu_tick_count >= 3)
        {
            imu_tick_count = 0;
            mag_trigger = 1;
        }
    }
}
