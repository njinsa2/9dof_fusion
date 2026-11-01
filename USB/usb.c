#include "usb.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "misc.h"
#include <stdarg.h>
#include <string.h>

#define USB_TIMEOUT_MS 50 // Max wait time per transmission before dropping packet

uint8_t USB_Transmit(uint8_t *data, uint16_t len)
{
    if (len == 0) return 0;

    uint32_t start_tick = HAL_GetTick();

    // Loop until endpoint is free or timeout occurs
    while (CDC_Transmit_FS(data, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start_tick) >= USB_TIMEOUT_MS)
        {
            return 1; // Timeout
        }
    }
    return 0; // Success
}

void USB_Print(char *str, ...)
{
    uint8_t out_buf[256];
    uint16_t out_idx = 0;

    uint8_t rstr[40];
    uint16_t counter = 0;
    uint16_t arg_type;
    uint32_t utmp32;
    uint32_t *p_uint32;
    char *p_char;
    va_list vl;

    va_start(vl, str);
    while (str[counter] != 0x00 && out_idx < 250)
    {
        if (str[counter] == '%')
        {
            if (0 != str[counter + 1])
            {
                switch (str[counter + 1])
                {
                    case ('b'):
                    {
                        if (str[counter + 2] == 'b')      { utmp32 = va_arg(vl, int); arg_type = PRINT_ARG_TYPE_BINARY_BYTE; }
                        else if (str[counter + 2] == 'h') { utmp32 = va_arg(vl, int); arg_type = PRINT_ARG_TYPE_BINARY_HALFWORD; }
                        else if (str[counter + 2] == 'w') { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_BINARY_WORD; }
                        else                              { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_BINARY_WORD; counter--; }

                        counter++;
                        p_uint32 = &utmp32;
                        break;
                    }
                    case ('d'):
                    {
                        if (str[counter + 2] == 'b')      { utmp32 = va_arg(vl, int); arg_type = PRINT_ARG_TYPE_DECIMAL_BYTE; }
                        else if (str[counter + 2] == 'h') { utmp32 = va_arg(vl, int); arg_type = PRINT_ARG_TYPE_DECIMAL_HALFWORD; }
                        else if (str[counter + 2] == 'w') { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_DECIMAL_WORD; }
                        else                              { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_DECIMAL_WORD; counter--; }

                        counter++;
                        p_uint32 = &utmp32;
                        break;
                    }
                    case ('c'):
                    {
                        char tchar = va_arg(vl, int);
                        if (out_idx < 255) out_buf[out_idx++] = tchar;
                        arg_type = PRINT_ARG_TYPE_CHARACTER;
                        break;
                    }
                    case ('s'):
                    {
                        p_char = va_arg(vl, char *);
                        uint16_t s_idx = 0;
                        while (p_char[s_idx] != 0 && out_idx < 255)
                        {
                            out_buf[out_idx++] = p_char[s_idx++];
                        }
                        arg_type = PRINT_ARG_TYPE_STRING;
                        break;
                    }
                    case ('f'):
                    {
                        uint64_t utmp64 = va_arg(vl, uint64_t);
                        uint32_t tmp1 = utmp64 & 0x00000000FFFFFFFF;
                        tmp1 = tmp1 >> 29;
                        utmp32 = utmp64 >> 32;
                        utmp32 &= 0x07FFFFFF;
                        utmp32 = utmp32 << 3;
                        utmp32 |= tmp1;

                        if (utmp64 & 0x8000000000000000) utmp32 |= 0x80000000;
                        if (utmp64 & 0x4000000000000000) utmp32 |= 0x40000000;

                        p_uint32 = &utmp32;
                        arg_type = PRINT_ARG_TYPE_FLOAT;
                        break;
                    }
                    case ('x'):
                    {
                        if (str[counter + 2] == 'b')      { utmp32 = (uint32_t)va_arg(vl, int); arg_type = PRINT_ARG_TYPE_HEXADECIMAL_BYTE; }
                        else if (str[counter + 2] == 'h') { utmp32 = (uint32_t)va_arg(vl, int); arg_type = PRINT_ARG_TYPE_HEXADECIMAL_HALFWORD; }
                        else if (str[counter + 2] == 'w') { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_HEXADECIMAL_WORD; }
                        else                              { utmp32 = va_arg(vl, uint32_t); arg_type = PRINT_ARG_TYPE_HEXADECIMAL_WORD; counter--; }

                        counter++;
                        p_uint32 = &utmp32;
                        break;
                    }
                    default:
                    {
                        utmp32 = 0;
                        p_uint32 = &utmp32;
                        arg_type = PRINT_ARG_TYPE_UNKNOWN;
                        break;
                    }
                }

                if (arg_type & PRINT_ARG_TYPE_MASK_CHAR_STRING)
                {
                    misc_get_string_from_number(arg_type, p_uint32, rstr);
                    uint16_t r_idx = 0;
                    while (rstr[r_idx] != 0 && out_idx < 255)
                    {
                        out_buf[out_idx++] = rstr[r_idx++];
                    }
                }

                counter++;
            }
        }
        else
        {
            if (str[counter] == '\n')
            {
                if (out_idx < 255) out_buf[out_idx++] = '\r';
                if (out_idx < 255) out_buf[out_idx++] = '\n';
            }
            else
            {
                if (out_idx < 255) out_buf[out_idx++] = str[counter];
            }
        }

        counter++;
    }

    va_end(vl);

    // Send the full accumulated string over USB CDC in a single packet
    USB_Transmit(out_buf, out_idx);
}
