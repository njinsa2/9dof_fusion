#ifndef SPI_H_
#define SPI_H_

#include "error.h"
#include "stdint.h"

void MX_SPI2_Init(void);
ERROR_CODE SPI_Write_Buffer(uint8_t *, uint32_t);
ERROR_CODE SPI_Write_Byte(uint8_t);
ERROR_CODE SPI_Read_Byte(uint8_t *);
ERROR_CODE SPI_Read_Buffer(uint8_t *, uint32_t);

#endif
