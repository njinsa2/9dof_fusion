#ifndef ERROR_H_
#define ERROR_H_

typedef enum
{
	NO_ERROR,

	USB_ERROR,

	/* SPI Error Group */
	SPI_TRANSMIT_ERROR,
	SPI_RECEIVE_ERROR,
    SPI_INIT_ERROR,

	/* MPU Error Group */
	ISM_INIT_ERROR,
	ISM_MEASUREMENT_ERROR,

	/* QMC Error Group */
	MMC_INIT_ERROR,
	MMC_MEASUREMENT_ERROR,

	CLOCK_CONFIG_ERROR
} ERROR_CODE;

void Error_Handler(ERROR_CODE);

#endif /* ERROR_H_ */
