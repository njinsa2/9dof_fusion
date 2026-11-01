#ifndef MISC_H_
#define MISC_H_

#include "stm32f4xx.h"

#define MAX_PRINT_STRING_SIZE					1024

#define PRINT_ARG_TYPE_BINARY_BYTE				0x0001
#define PRINT_ARG_TYPE_BINARY_HALFWORD			0x0002
#define PRINT_ARG_TYPE_BINARY_WORD				0x0004
#define PRINT_ARG_TYPE_DECIMAL_BYTE				0x0008
#define PRINT_ARG_TYPE_DECIMAL_HALFWORD			0x0010
#define PRINT_ARG_TYPE_DECIMAL_WORD				0x0020
#define PRINT_ARG_TYPE_CHARACTER				0x0040
#define PRINT_ARG_TYPE_STRING					0x0080
#define PRINT_ARG_TYPE_FLOAT					0x0100
#define PRINT_ARG_TYPE_HEXADECIMAL_BYTE			0x0200
#define PRINT_ARG_TYPE_HEXADECIMAL_HALFWORD		0x0400
#define PRINT_ARG_TYPE_HEXADECIMAL_WORD			0x0800
#define PRINT_ARG_TYPE_UNKNOWN					0x0000

#define PRINT_ARG_TYPE_MASK_CHAR_STRING			~((PRINT_ARG_TYPE_CHARACTER)|(PRINT_ARG_TYPE_STRING))

/** @brief  Calculate decimal digits from integer number 'num' and store them as
  * 		ASCII char in 'decimal_num' array
  *
  * @param  num - integer number
  * 		decimal_num - array that contains decimal digits as ASCII char
  *
  * @retval None
  */
void misc_get_decimal_num(uint8_t * decimal_num, uint32_t num);

/** @brief  Add unsigned integer number digits into string
  *
  * @param  integer - integer number
  * 		num_of_char - num of chars in string
  * 		str - string representing a the given integer
  *
  * @retval None
  */
void misc_get_string_from_int(uint8_t * integer,
							  uint16_t * num_of_char,
							  uint8_t * str);

/** @brief  print text and one signed integer or float number
  *
  * @param  type - data type
  * 		num - number to print
  * 		str - string representing the given number to print
  *
  * @retval None
  */
void misc_get_string_from_number(uint16_t type, uint32_t  * num, uint8_t * str);

#endif /* __MISC_H_ */
