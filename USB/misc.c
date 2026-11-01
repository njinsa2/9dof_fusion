#include "misc.h"

void misc_get_decimal_num(uint8_t * decimal_num, uint32_t num)
{
    uint8_t counter;
    uint32_t step = 1000000000;

    for (counter = 0; counter < 10; counter++)
    {
    	decimal_num[9 - counter] = 48;

        while (num >= step)
        {
        	decimal_num[9 - counter]++;
            num -= step;
        }

        step /= 10;
    }
}

void misc_get_string_from_int(uint8_t * integer,
							  uint16_t * num_of_char,
							  uint8_t * str)
{
    uint8_t counter = 0, flag = 0;

    for (counter = 0; counter < 10; counter++)
    {
        if (flag == 0)
        {
            if ((integer[9 - counter] != 48) || (counter == 9))
            {
                flag = 1;
                str[(*num_of_char)] = integer[9 - counter];
                (*num_of_char)++;
            }
        }
        else
        {
        	str[(*num_of_char)] = integer[9 - counter];
            (*num_of_char)++;
        }
    }

    str[(*num_of_char)] = 0x00;
}

void misc_get_string_from_number(uint16_t type, uint32_t  * num, uint8_t * str)
{
    int32_t * p_int32;
    uint8_t dnum[11];
    uint8_t counter;
    uint16_t num_of_char = 0;
    str[0] = 0x00;

    if (PRINT_ARG_TYPE_BINARY_BYTE == type)
    {
    	for(counter = 0; counter < 8; counter++)
		{
			if((*num) & 0x00000080)
			{
				str[counter] = '1';
			}
			else
			{
				str[counter] = '0';
			}

			*num = (*num) << 1;
		}

		str[counter] = 0x00;
    }
    else if (PRINT_ARG_TYPE_BINARY_HALFWORD == type)
    {
    	for(counter = 0; counter < 16; counter++)
		{
			if((*num) & 0x00008000)
			{
				str[counter] = '1';
			}
			else
			{
				str[counter] = '0';
			}

			*num = (*num) << 1;
		}

    	str[counter] = 0x00;
    }
    else if (PRINT_ARG_TYPE_BINARY_WORD == type)
    {
    	for(counter = 0; counter < 32; counter++)
		{
			if((*num) & 0x80000000)
			{
				str[counter] = '1';
			}
			else
			{
				str[counter] = '0';
			}

			*num = (*num) << 1;
		}

    	str[counter] = 0x00;
    }
    else if (PRINT_ARG_TYPE_DECIMAL_BYTE == type
    		 || PRINT_ARG_TYPE_DECIMAL_HALFWORD == type
			 || PRINT_ARG_TYPE_DECIMAL_WORD == type)
    {
    	p_int32 = (int32_t *)num;

		if (*p_int32 < 0)
		{
			str[num_of_char] = '-';
			num_of_char++;
			*p_int32 = (*p_int32)*(-1);
		}

		misc_get_decimal_num(dnum, (uint32_t)(*p_int32));
		misc_get_string_from_int(dnum, &num_of_char, str);
    }
    else if (PRINT_ARG_TYPE_FLOAT == type)
    {
    	float f_step;
		float f_num = *((float *)num);
		uint8_t sige = ' ';
		uint16_t e_part;
		uint32_t d_part, i_part;

		if (f_num < 0)
		{
			str[num_of_char] = '-';
			num_of_char++;
			f_num = (-1) * f_num;
		}

		if (f_num < 1)
		{
			sige = '-';
			f_step = 0.1;
			e_part = 1;
			while (f_num < f_step)
			{
				f_step = f_step * 0.1;
				e_part++;
			}
		}
		else
		{
			f_step = 1.0;
			e_part = 0;
			while (f_num > f_step)
			{
				f_step = f_step * 10;
				e_part++;
			}
			f_step = f_step / 10;
			e_part--;
		}

		f_num = f_num / f_step;
		i_part = (uint32_t)f_num;

		f_num = f_num - (float)i_part;
		d_part = (uint32_t)(1000 * f_num);

		misc_get_decimal_num(dnum, i_part);
		misc_get_string_from_int(dnum, &num_of_char, str);

		misc_get_decimal_num(dnum, d_part);
		dnum[3] = '.';
		misc_get_string_from_int(dnum, &num_of_char, str);

		str[num_of_char] = 'e';
		num_of_char++;

		if (sige == '-')
		{
			str[num_of_char] = '-';
			num_of_char++;
		}

		misc_get_decimal_num(dnum, e_part);
		misc_get_string_from_int(dnum, &num_of_char, str);
    }
    else if (PRINT_ARG_TYPE_HEXADECIMAL_BYTE == type)
    {
    	uint8_t t_rez;

		for (counter = 0; counter < 2; counter++)
		{
			t_rez = ((*num) & 0x000000F0) >> 4;

			if (t_rez < 0x0A)
			{
				str[num_of_char] = t_rez + 0x30;
			}
			else
			{
				str[num_of_char] = t_rez + 0x41 - 0x0A;
			}

			(*num) = (*num) << 4;
			num_of_char++;
		}

		str[num_of_char] = 0x00;
    }
    else if (PRINT_ARG_TYPE_HEXADECIMAL_HALFWORD == type)
    {
    	uint8_t t_rez;

		for (counter = 0; counter < 4; counter++)
		{
			t_rez = ((*num) & 0x0000F000) >> 12;

			if (t_rez < 0x0A)
			{
				str[num_of_char] = t_rez + 0x30;
			}
			else
			{
				str[num_of_char] = t_rez + 0x41 - 0x0A;
			}

			(*num) = (*num) << 4;
			num_of_char++;
		}

		str[num_of_char] = 0x00;
    }
    else if (PRINT_ARG_TYPE_HEXADECIMAL_WORD == type)
    {
    	uint8_t t_rez;
		for (counter = 0; counter < 8; counter++)
		{
			t_rez = ((*num) & 0xF0000000) >> 28;

			if (t_rez < 0x0A)
			{
				str[num_of_char] = t_rez + 0x30;
			}
			else
			{
				str[num_of_char] = t_rez + 0x41 - 0x0A;
			}

			(*num) = (*num) << 4;
			num_of_char++;
		}
		str[num_of_char] = 0x00;
    }
    else
    {
    	str[0] = 0x00;
    }
}
