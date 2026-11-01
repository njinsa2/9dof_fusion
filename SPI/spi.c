#include "spi.h"
#include "stm32f4xx_hal.h"

#define SPI_TRANSMIT_TIMEOUT_MS 5000
#define SPI_RECEIVE_TIMEOUT_MS 5000

SPI_HandleTypeDef hspi2;




void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler(SPI_INIT_ERROR);
  }
}



ERROR_CODE SPI_Write_Byte(uint8_t byte)
{
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2,
												&byte,
												1,
												SPI_TRANSMIT_TIMEOUT_MS);

	ERROR_CODE error = NO_ERROR;
	if (HAL_OK != status)
	{
		error = SPI_TRANSMIT_ERROR;
	}

	return error;
}


ERROR_CODE SPI_Write_Buffer(uint8_t * buffer, uint32_t size)
{
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2,
												buffer,
												size,
												SPI_TRANSMIT_TIMEOUT_MS);

	ERROR_CODE error = NO_ERROR;
	if (HAL_OK != status)
	{
		error = SPI_TRANSMIT_ERROR;
		Error_Handler(error);
	}

	return error;
}


ERROR_CODE SPI_Read_Byte(uint8_t * byte)
{
	HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi2,
											   byte,
											   1,
											   SPI_RECEIVE_TIMEOUT_MS);

	ERROR_CODE error = NO_ERROR;
	if (HAL_OK != status)
	{
		error = SPI_RECEIVE_ERROR;
	}

	return error;
}

ERROR_CODE SPI_Read_Buffer(uint8_t * buffer, uint32_t size)
{
	HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi2,
											   buffer,
											   size,
											   SPI_RECEIVE_TIMEOUT_MS);

	ERROR_CODE error = NO_ERROR;
	if (HAL_OK != status)
	{
		error = SPI_RECEIVE_ERROR;
		Error_Handler(error);
	}

	return error;
}
