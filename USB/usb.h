#ifndef __CUSTOM_USB_H__
#define __CUSTOM_USB_H__

#include <stdint.h>
/**
 * @brief Transmits a formatted string over USB CDC (printf-style).
 * @param fmt Format string
 * @param ... Variable arguments
 */
void USB_Print(char *fmt, ...);

/**
 * @brief Sends raw byte buffer over USB CDC with a timeout.
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return 0 on success, 1 on timeout/failure
 */
uint8_t USB_Transmit(uint8_t *data, uint16_t len);

#endif /* __CUSTOM_USB_H__ */
