#include "mmc5983ma.h"
#include "usb.h"
#include "spi.h"
#include "main.h"
#include "error.h"
#include <math.h>

#define MCS_LOW HAL_GPIO_WritePin(MCS_GPIO_Port, MCS_Pin, 0);
#define MCS_HIGH HAL_GPIO_WritePin(MCS_GPIO_Port, MCS_Pin, 1);


//hard-iron bias
const float mag_bias[3] = {
    0.6827626794f,
    1.7337214840f,
    0.1369067605f
};

//soft-iron correction matrix
const float mag_matrix[3][3] = {
    {  1.04008430f, -0.01171117f, -0.01418406f },
    { -0.01171117f,  0.94577819f,  0.02488249f },
    { -0.01418406f,  0.02488249f,  1.01756295f }
};


ERROR_CODE MMC_Write_Byte(uint8_t address, uint8_t data){
	//address is a 6-bit value
	uint8_t cmd_byte = MMC_Write_Bit | address;
	uint8_t data_to_send[2] = {cmd_byte, data};

	MCS_LOW

	ERROR_CODE error_code = SPI_Write_Buffer(data_to_send,  2);

	if(error_code != NO_ERROR){
		MCS_HIGH
		Error_Handler(error_code);
	}

	MCS_HIGH

	return error_code;
};

static ERROR_CODE MMC_Read_Byte(uint8_t address, uint8_t * data){
	uint8_t cmd_byte = MMC_Read_Bit | address;

	MCS_LOW

	ERROR_CODE error_code = SPI_Write_Byte(cmd_byte);

	if (error_code == NO_ERROR){
		error_code = SPI_Read_Byte(data);
	}

	MCS_HIGH

	return error_code;
};

uint8_t MMC_Read_Status(){
	uint8_t status = 0;

	ERROR_CODE error_code = MMC_Read_Byte(STATUS, &status);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	return status;
};

uint8_t MMC_Read_Product_ID(){
	uint8_t product_id = 0;

	ERROR_CODE error_code = MMC_Read_Byte(PRODUCT_ID1, &product_id);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	return product_id;
};


void MMC_Self_Test()
{
    float x[3], y[3], z[3];
    float out_gauss[3];

    //self-test coil off
    MMC_Write_Byte(CTRL3, 0);
    HAL_Delay(5000);

    MMC_Read_Data(out_gauss);
    x[0] = out_gauss[0];
    y[0] = out_gauss[1];
    z[0] = out_gauss[2];

    //self-test coil with positive current
    MMC_Write_Byte(CTRL3, 1 << 1);
    HAL_Delay(50);

    MMC_Read_Data(out_gauss);
    x[1] = out_gauss[0];
    y[1] = out_gauss[1];
    z[1] = out_gauss[2];

    //self-test coil with negative current
    MMC_Write_Byte(CTRL3, 1 << 2);
    HAL_Delay(50);

    MMC_Read_Data(out_gauss);
    x[2] = out_gauss[0];
    y[2] = out_gauss[1];
    z[2] = out_gauss[2];

    //self-test coil off again
    MMC_Write_Byte(CTRL3, 0);

    USB_Print("MMC self test coil off: X = %f, Y = %f, Z = %f \n", x[0], y[0], z[0]);

    USB_Print("MMC self test coil positive: X = %f, Y = %f, Z = %f \n", x[1], y[1], z[1]);

    USB_Print("MMC self test coil negative: X = %f, Y = %f, Z = %f \n", x[2], y[2], z[2]);
}


void MMC_Init(void)
{
    ERROR_CODE error_code;

    //software reset
    error_code = MMC_Write_Byte(CTRL1, CTRL1_SW_RST);
    if (error_code != NO_ERROR){
    	Error_Handler(error_code);
    }
    HAL_Delay(15);

    //set measurement time to 4ms (200Hz bandwidth)
    error_code = MMC_Write_Byte(CTRL1, CTRL1_BW_4MS);
    if (error_code != NO_ERROR){
    	Error_Handler(error_code);
    }

    //explicitly disable continuous mode
    error_code = MMC_Write_Byte(CTRL2, 0x00);
    if (error_code != NO_ERROR){
    	Error_Handler(error_code);
    }

    //clear measurement status flags so the interrupt pin starts low
    error_code = MMC_Write_Byte(STATUS, STATUS_MEAS_T_DONE | STATUS_MEAS_M_DONE);
    if (error_code != NO_ERROR){
    	Error_Handler(error_code);
    }

}

void MMC_Trigger_Measurement(){
	uint8_t cmd_byte = MMC_Write_Bit | CTRL0_TM_M;
	MMC_Write_Byte(CTRL0, cmd_byte);
};


static void MMC_Apply_Calibration(float *mag)
{
    //remove hard-iron offsets
    float x = mag[0] - mag_bias[0];
    float y = mag[1] - mag_bias[1];
    float z = mag[2] - mag_bias[2];

    //apply the soft-iron correction matrix
    float calibrated_x =
          mag_matrix[0][0] * x
        + mag_matrix[0][1] * y
        + mag_matrix[0][2] * z;

    float calibrated_y =
          mag_matrix[1][0] * x
        + mag_matrix[1][1] * y
        + mag_matrix[1][2] * z;

    float calibrated_z =
          mag_matrix[2][0] * x
        + mag_matrix[2][1] * y
        + mag_matrix[2][2] * z;

    //store calibrated values
    mag[0] = calibrated_x;
    mag[1] = calibrated_y;
    mag[2] = calibrated_z;
}

void MMC_Read_Data(float *out_gauss) {
	uint8_t cmd_byte = MMC_Read_Bit | XOUT0;
    uint8_t raw_data[7];

    MCS_LOW
    ERROR_CODE error_code = SPI_Write_Byte(cmd_byte);

    if(error_code != NO_ERROR){
    	Error_Handler(error_code);
    }

    error_code = SPI_Read_Buffer(raw_data, 7);

    if(error_code != NO_ERROR){
    	Error_Handler(error_code);
    }
    MCS_HIGH

    uint32_t raw_x = ((uint32_t)raw_data[0] << 10) |
                     ((uint32_t)raw_data[1] << 2)  |
                     (((uint32_t)raw_data[6] >> 6) & 0x03);

    uint32_t raw_y = ((uint32_t)raw_data[2] << 10) |
                     ((uint32_t)raw_data[3] << 2)  |
                     (((uint32_t)raw_data[6] >> 4) & 0x03);

    uint32_t raw_z = ((uint32_t)raw_data[4] << 10) |
                     ((uint32_t)raw_data[5] << 2)  |
                     (((uint32_t)raw_data[6] >> 2) & 0x03);

    out_gauss[0] = ((float)raw_x - 131072.0f) / 16384.0f;
    out_gauss[1] = ((float)raw_y - 131072.0f) / 16384.0f;
    out_gauss[2] = ((float)raw_z - 131072.0f) / 16384.0f;

    MMC_Apply_Calibration(out_gauss);
}



void MMC_Read_Data_NonCalibrated(float *out_gauss)
{
    uint8_t cmd_byte = MMC_Read_Bit | XOUT0;
    uint8_t raw_data[7] = {0};

    MCS_LOW
    ERROR_CODE error_code = SPI_Write_Byte(cmd_byte);

    if (error_code != NO_ERROR){
        Error_Handler(error_code);
    }

    error_code = SPI_Read_Buffer(raw_data, 7);

    if (error_code != NO_ERROR){
        Error_Handler(error_code);
    }
    MCS_HIGH

    uint32_t raw_x = ((uint32_t)raw_data[0] << 10) |
                     ((uint32_t)raw_data[1] << 2)  |
                     (((uint32_t)raw_data[6] >> 6) & 0x03);

    uint32_t raw_y = ((uint32_t)raw_data[2] << 10) |
                     ((uint32_t)raw_data[3] << 2)  |
                     (((uint32_t)raw_data[6] >> 4) & 0x03);

    uint32_t raw_z = ((uint32_t)raw_data[4] << 10) |
                     ((uint32_t)raw_data[5] << 2)  |
                     (((uint32_t)raw_data[6] >> 2) & 0x03);

    out_gauss[0] = ((float)raw_x - 131072.0f) / 16384.0f;
    out_gauss[1] = ((float)raw_y - 131072.0f) / 16384.0f;
    out_gauss[2] = ((float)raw_z - 131072.0f) / 16384.0f;
}
