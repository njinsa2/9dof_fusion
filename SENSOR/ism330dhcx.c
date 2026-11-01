#include "ism330dhcx.h"
#include "spi.h"
#include "usb.h"
#include "main.h"
#include "error.h"


#define ISM_Read_Bit (1 << 7)
#define ISM_Write_Bit (0 << 7)

//config registers
#define CTRL3_C 0x12
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define INT1_CTRL 0x0D
#define CTRL4_C 0x13
#define CTRL6_C 0x15
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18
#define WHO_AM_I 0x0F
#define WHO_AM_I_VALUE 0x6B
#define COUNTER_BDR_REG1 0x0B

//starting data register
#define OUTX_L_G 0x22

#define ACS_LOW HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 0);
#define ACS_HIGH HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 1);

static const float DEG_TO_RAD = 0.01745329252f;


const float gyro_bias[3] = {0.130187f, -0.547194f, -0.009274f};
const float accel_bias[3] = {0.006282f, -0.010334f, 0.013791f};
const float accel_scale[3] = {1.005713f, 1.012573f, 1.014720f};


static ERROR_CODE ISM_Write_Byte(uint8_t address, uint8_t data){
	uint8_t cmd_byte = ISM_Write_Bit | address;
	uint8_t data_to_send[2] = {cmd_byte, data};

	ACS_LOW

	ERROR_CODE error_code = SPI_Write_Buffer(data_to_send,  2);

	ACS_HIGH

	return error_code;
};

static ERROR_CODE ISM_Read_Byte(uint8_t address, uint8_t * data){
	uint8_t cmd_byte = ISM_Read_Bit | address;

	ACS_LOW

	ERROR_CODE error_code = SPI_Write_Byte(cmd_byte);

	if (error_code == NO_ERROR){
		error_code = SPI_Read_Byte(data);
	}

	ACS_HIGH

	return error_code;
};

uint8_t ISM_Who_Am_I(){
	uint8_t who_am_i = 0;

	ERROR_CODE error_code = ISM_Read_Byte(WHO_AM_I, &who_am_i);

	if (error_code != NO_ERROR){
		ACS_HIGH
		Error_Handler(error_code);
	}

	USB_Print("ISM WHO AM I = %xb\n", who_am_i);

	return who_am_i;
};

void ISM_Read_Data(float * accel, float * gyro) {
	uint8_t cmd_byte = ISM_Read_Bit | OUTX_L_G;
	uint8_t raw_data[12];
	float accel_raw[3], gyro_raw[3];
	ERROR_CODE error_code;

	ACS_LOW;

	error_code = SPI_Write_Byte(cmd_byte);
	if (error_code != NO_ERROR) {
		ACS_HIGH;
		Error_Handler(ISM_MEASUREMENT_ERROR);
	}

	error_code = SPI_Read_Buffer(raw_data, 12);

	ACS_HIGH;

	if (error_code != NO_ERROR) {
		Error_Handler(ISM_MEASUREMENT_ERROR);
	}


	int16_t raw_x_gyro = (int16_t)((raw_data[1] << 8) | raw_data[0]);
	int16_t raw_y_gyro = (int16_t)((raw_data[3] << 8) | raw_data[2]);
	int16_t raw_z_gyro = (int16_t)((raw_data[5] << 8) | raw_data[4]);

	int16_t raw_x_accel = (int16_t)((raw_data[7] << 8) | raw_data[6]);
	int16_t raw_y_accel = (int16_t)((raw_data[9] << 8) | raw_data[8]);
	int16_t raw_z_accel = (int16_t)((raw_data[11] << 8) | raw_data[10]);

	//convert raw values to physical units (x, y, z in this order)
	accel_raw[0] = (float)raw_x_accel * 0.000122f;
	accel_raw[1] = (float)raw_y_accel * 0.000122f;
	accel_raw[2] = (float)raw_z_accel * 0.000122f;

	//apply accelerometer calibration
	for (int i = 0; i < 3; i++) {
	    accel[i] = (accel_raw[i] - accel_bias[i]) * accel_scale[i];
	}


	//gyroscope raw readings to degrees per second
	gyro_raw[0] = (float)raw_x_gyro * 0.070f;
	gyro_raw[1] = (float)raw_y_gyro * 0.070f;
	gyro_raw[2] = (float)raw_z_gyro * 0.070f;

	//apply gyroscope bias correction and convert from dps to rads for mahony
	for (int i = 0; i < 3; i++) {
	    gyro[i] = gyro_raw[i] - gyro_bias[i];
	    gyro[i] *= DEG_TO_RAD;
	}

}


void ISM_Read_Data_NonCalibrated(float * accel, float * gyro) {
	uint8_t cmd_byte = ISM_Read_Bit | OUTX_L_G;
	uint8_t raw_data[12];
	ERROR_CODE error_code;

	ACS_LOW;

	error_code = SPI_Write_Byte(cmd_byte);
	if (error_code != NO_ERROR) {
		ACS_HIGH;
		Error_Handler(ISM_MEASUREMENT_ERROR);
	}

	error_code = SPI_Read_Buffer(raw_data, 12);

	ACS_HIGH;

	if (error_code != NO_ERROR) {
		Error_Handler(ISM_MEASUREMENT_ERROR);
	}


	int16_t raw_x_gyro = (int16_t)((raw_data[1] << 8) | raw_data[0]);
	int16_t raw_y_gyro = (int16_t)((raw_data[3] << 8) | raw_data[2]);
	int16_t raw_z_gyro = (int16_t)((raw_data[5] << 8) | raw_data[4]);

	int16_t raw_x_accel = (int16_t)((raw_data[7] << 8) | raw_data[6]);
	int16_t raw_y_accel = (int16_t)((raw_data[9] << 8) | raw_data[8]);
	int16_t raw_z_accel = (int16_t)((raw_data[11] << 8) | raw_data[10]);

	//convert raw values to physical units (x, y, z in this order)
	accel[0] = (float)raw_x_accel * 0.000122f;
	accel[1] = (float)raw_y_accel * 0.000122f;
	accel[2] = (float)raw_z_accel * 0.000122f;

	//gyroscope raw readings to degrees per second
	gyro[0] = (float)raw_x_gyro * 0.070f;
	gyro[1] = (float)raw_y_gyro * 0.070f;
	gyro[2] = (float)raw_z_gyro * 0.070f;
}

void ISM_Init(){
	//software reset
	ERROR_CODE error_code = ISM_Write_Byte(CTRL3_C, 1 << 0);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	HAL_Delay(10);

	//enable block data update and auto increment
	error_code = ISM_Write_Byte(CTRL3_C, (1 << 6) | (1 << 2));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//enable proper device configuration (keeps default DEN bits)
	error_code = ISM_Write_Byte(CTRL9_XL, (1 << 7) | (1 << 6) | (1 << 5) | (1 << 1));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//disable i2c interface and enable gyro lpf1
	error_code = ISM_Write_Byte(CTRL4_C, (1 << 2) | (1 << 1));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//set gyro lpf1 cutoff to 67Hz at odr of 208Hz (ftype = 000)
	error_code = ISM_Write_Byte(CTRL6_C, 0);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//set accel lpf2 bandwidth to odr/10
	error_code = ISM_Write_Byte(CTRL8_XL, 1 << 5);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//configure full scale to +-4g with odr of 208Hz for accel and enable lpf2
	error_code = ISM_Write_Byte(CTRL1_XL, (1 << 6) | (1 << 4) | (1 << 3) | (1 << 1));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//configure full scale to +-2000dps with odr of 208Hz for gyro
	error_code = ISM_Write_Byte(CTRL2_G, (1 << 6) | (1 << 4) | (1 << 3) | (1 << 2));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//set data ready signal to pulsed mode so the interrupt pin always returns low
	error_code = ISM_Write_Byte(COUNTER_BDR_REG1, 1 << 7);

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}

	//enable interrupts on int1 pin
	error_code = ISM_Write_Byte(INT1_CTRL, (1 << 1) | (1 << 0));

	if (error_code != NO_ERROR){
		Error_Handler(error_code);
	}
};
