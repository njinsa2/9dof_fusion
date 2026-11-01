#ifndef ISM330DHCX_H_
#define ISM330DHCX_H_

#include <stdint.h>


void ISM_Init();
uint8_t ISM_Who_Am_I(void);
void ISM_Read_Data(float *, float *);
void ISM_Read_Data_NonCalibrated(float *, float *);
#endif
