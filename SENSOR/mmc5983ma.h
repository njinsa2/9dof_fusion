#ifndef MMC5983MA_H_
#define MMC5983MA_H_

#include <stdint.h>

#define MMC_Read_Bit (1 << 7)
#define MMC_Write_Bit (0 << 7)

//configuration registers
#define CTRL0 0x09
#define CTRL1 0x0A
#define CTRL2 0x0B
#define STATUS 0x08
#define PRODUCT_ID1 0x2F
#define CTRL3 0x0C

//starting data register
#define XOUT0 0x00

//notable register fields
#define STATUS_MEAS_M_DONE (1 << 0)
#define STATUS_MEAS_T_DONE (1 << 1)
#define CTRL1_SW_RST (1 << 7)
#define CTRL1_BW_4MS (1 << 0)


#define CTRL0_TM_M (1 << 0)

void MMC_Init();
uint8_t MMC_Read_Status();
uint8_t MMC_Read_Product_ID();
void MMC_Self_Test();
void MMC_Trigger_Measurement();
void MMC_Read_Data(float *);
void MMC_Read_Data_NonCalibrated(float *);
#endif
