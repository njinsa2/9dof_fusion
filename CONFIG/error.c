#include "error.h"
#include "usb.h"
#include "stm32f4xx_hal.h"



void Error_Handler(ERROR_CODE error_code)
{
  __disable_irq();

  while (1)
  {
	  switch (error_code)
	  {
	  case USB_ERROR:
		  USB_Print("--- USB Error ---");
	  case SPI_TRANSMIT_ERROR:
	  	  USB_Print("--- Error during SPI transmission ---");
	  case SPI_RECEIVE_ERROR:
	  	  USB_Print("--- Error during SPI receive ---");
	  case SPI_INIT_ERROR:
	  	  USB_Print("--- Error during SPI initialization ---");
	  case ISM_INIT_ERROR:
		  USB_Print("--- Error during ISM initialization ---");
	  case ISM_MEASUREMENT_ERROR:
	  	  USB_Print("--- Error during ISM measurement ---");
	  case MMC_INIT_ERROR:
		  USB_Print("--- Error during MMC initialization ---");
	  case MMC_MEASUREMENT_ERROR:
	  	  USB_Print("--- Error during MMC measurement ---");
	  case CLOCK_CONFIG_ERROR:
		  USB_Print("--- ERROR during System Clock configuration (peripheral_init) ---");
	  case NO_ERROR:
		  return;
	  }

	  HAL_Delay(1000);
  }
}
