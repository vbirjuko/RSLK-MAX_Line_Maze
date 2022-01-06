/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "configure.h"
#include "i2c_drv.h"
#include "main.h"
#include "Maze.h"
#include "Logging.h"
#include "UART_drv.h"
#include "UART0.h"
#include "UART1.h"
#include "SPI_EEProm.h"
#include "ADC.h"
#include "Reflectance.h"
#include "commandline.h"
#include "crc.h"
#include "fonts_oled.h"
#include "FRAM_Logging.h"

#define NULL ((void* )0 )
#define UNUSED(x) (void) x



instance_t BGX_UART1 = {
	.input_string = "\0",
	.base = 10,
	.stack_idx = 0, .str_index = 0, .esc_sequence = 0,
	.UART_OutString = &UART1_OutString,
	.UART_OutChar   = &UART1_OutChar,
	.UART_OutUDec   = &UART1_OutUDec,
};

instance_t USBUART = {
    "\0",
    10,
    {0}, 0, 0, 0,
    &UART0_OutString,
    &UART0_OutChar,
    &UART0_OutUDec,
};

uint32_t list_cmd(instance_t *, int * none);
uint32_t list_values(instance_t *, int * none);

//unsigned char input_string[64];
//unsigned int base = 10;
//uint32_t stack[FORTH_STACK_SIZE], stack_idx = 0;
//uint16_t log_size = 0, fps;

unsigned int input_command(instance_t *instance, char inbyte) {
    static const char backstr[] = {0x08, ' ', 0x08, 0};
    if (instance->esc_sequence == 0){
        switch (inbyte) {
            case 0x03:
                instance->str_index = 0;
                instance->input_string[instance->str_index] = '\0';
                return 1;

            case 0x0D:
                instance->input_string[instance->str_index] = '\0';
                instance->str_index = 0;
                return 1;

            case 0x08:
            case 0x7F:
                if (instance->str_index) {
                    instance->UART_OutString((char *)backstr); /* Echo data back to PC */
                    instance->str_index--;
                }
                return 0;

            case 0x1B:
                instance->esc_sequence = 1;
                return 0;

            default: //    if (isASCII(inbyte)) {
                instance->input_string[instance->str_index] = inbyte;
                instance->UART_OutChar(instance->input_string[instance->str_index]); /* Echo data back to PC */
                if (++(instance->str_index) >= 63) {
                    instance->input_string[instance->str_index] = '\0';
                    instance->str_index = 0;
                    return 1;
                }
        }
    } else {
        instance->esc_sequence = 0; // затычка
        return 0;
    }
    return 0;
}

uint32_t dump_log(instance_t *instance, int * none) {
    uint16_t log_buffer[128], *log_buffer_ptr;
    int16_t coordX = 0, coordY = 0;
    unsigned int mask, timecode = 0, start_time = 0, end_time = 0xffffffff, segment = 0, eeprom_address, log_size, i, j;
    char timecode_str[10], *timecode_ptr;

    if (none) {
        if (instance->stack_idx < 2) return 1;
        start_time = instance->stack[instance->stack_idx - 2];
        end_time =   instance->stack[instance->stack_idx - 1];
        instance->stack_idx--;
        instance->stack_idx--;
    }

	eeprom_address = LOG_ADDRESS;
    spi_read_eeprom(eeprom_address, (uint8_t *)&log_buffer, sizeof(log_buffer));
    log_buffer_ptr = log_buffer;

    log_size = *log_buffer_ptr++;
    log_size += *log_buffer_ptr++ << 16;

    if (log_size > 6*64*1024) {
        instance->UART_OutString("Illegal log size. Aborted\r\n");
        return 1;
    }

    for (i=2; i<log_size; i++) {
        if (timecode < start_time || timecode > end_time) {
            if (*log_buffer_ptr & 0x0700) {
                timecode +=25;
            }
        } else {
            if (*log_buffer_ptr & 0x0700) {
                unsigned int divider = 100, outnum, leading_zero = 1;

                timecode_ptr = timecode_str;
                outnum = timecode / 10000;
                while (divider) {
                    *timecode_ptr = (outnum / divider) + '0';
                    if (leading_zero && *timecode_ptr == '0') *timecode_ptr = ' ';
                    else leading_zero = 0;
                    timecode_ptr++;
                    outnum %= divider;
                    divider /= 10;
                    if (divider == 1) leading_zero = 0;
                }
                *timecode_ptr++ = '.';
                outnum = timecode % 10000;
                divider = 1000;
                while (divider) {
                    *timecode_ptr++ = (outnum / divider) + '0';
                    outnum %= divider;
                    divider /= 10;
                }
                *timecode_ptr = ',';
                *timecode_ptr = '\0';
                instance->UART_OutString(timecode_str);

                mask = 0x80;
                for (j = 0; j < 8; j++) {
                    if (*log_buffer_ptr & mask) instance->UART_OutChar('*');
                    else       					instance->UART_OutChar('.');
                    mask >>= 1;
                }
                instance->UART_OutChar(',');
                if (*log_buffer_ptr & LEFT_MASK << 8)       instance->UART_OutChar('L'); else instance->UART_OutChar(' ');
                if (*log_buffer_ptr & STRAIGHT_MASK << 8)   instance->UART_OutChar('S'); else instance->UART_OutChar(' ');
                if (*log_buffer_ptr & RIGHT_MASK << 8)      instance->UART_OutChar('R'); else instance->UART_OutChar(' ');
                instance->UART_OutChar(',');
                if ((*log_buffer_ptr & 0x0700) == (Entrance << 8)) instance->UART_OutString("Entrance");
                if ((*log_buffer_ptr & 0x0700) == (Solve << 8)) instance->UART_OutString("Solve");
                if ((*log_buffer_ptr & 0x0700) == (Segment << 8)) instance->UART_OutString("Segment");
                if ((*log_buffer_ptr & 0x0700) == (Turn << 8)) instance->UART_OutString("Turn");
                instance->UART_OutChar(0x0d);
                instance->UART_OutChar(0x0a);
                timecode += 25;
            } else {
                if ((*log_buffer_ptr & 0xff00) == (coord_east_lsb << 8)) coordX = (coordX & 0xff00) | (*log_buffer_ptr & 0x00ff);
                if ((*log_buffer_ptr & 0xff00) == (coord_east_msb << 8)) coordX = (coordX & 0x00ff) | ((*log_buffer_ptr & 0x00ff) << 8);
                if ((*log_buffer_ptr & 0xff00) == (coord_north_lsb << 8)) coordY = (coordY & 0xff00) | (*log_buffer_ptr & 0x00ff);
                if ((*log_buffer_ptr & 0xff00) == (coord_north_msb << 8)) coordY = (coordY & 0x00ff) | ((*log_buffer_ptr & 0x00ff) << 8);
                if ((*log_buffer_ptr & 0xff00) == (segm_length_lsb << 8)) segment = (segment & 0xff00) | (*log_buffer_ptr & 0x00ff);
                if ((*log_buffer_ptr & 0xff00) == (segm_length_msb << 8)) segment = (segment & 0x00ff) | ((*log_buffer_ptr & 0x00ff) << 8);
                if ((*log_buffer_ptr & 0xff00) == (node_idx << 8)) {
                    instance->UART_OutString("Segment Length = ");
                    instance->UART_OutUDec(segment);
                    instance->UART_OutString("\r\nCurrent index = ");
                    instance->UART_OutUDec(*log_buffer_ptr & 0x00FF);
                    instance->UART_OutString(" , Coord X = ");
                    if (coordX < 0) { instance->UART_OutChar('-'); instance->UART_OutUDec(-coordX); }
                    else { instance->UART_OutUDec(coordX); }
                    instance->UART_OutString(" , Coord Y = ");
                    if (coordY < 0) { instance->UART_OutChar('-'); instance->UART_OutUDec(-coordY); }
                    else { instance->UART_OutUDec(coordY); }
                    instance->UART_OutChar(0x0d);
                    instance->UART_OutChar(0x0a);
                    segment = 0u;
                    coordX = coordY = 0;
                }
            }
        }
        if (++log_buffer_ptr > &log_buffer[127]) {
            eeprom_address += 256;
            spi_read_eeprom(eeprom_address, (uint8_t *)&log_buffer, sizeof(log_buffer));
            log_buffer_ptr = log_buffer;
        }
    }
    return 0;
}

uint32_t show_path(instance_t * instance, int * none) {
    rotation_dir_t *turn_ptr;
    int   *length_ptr, i;

    turn_ptr = data.path;
    length_ptr = data.length;

    for (i = 0; i < data.pathlength; i++) {
        instance->UART_OutUDec(i);
        instance->UART_OutChar('\t');

        instance->UART_OutUDec(*length_ptr);
        instance->UART_OutChar('\t');

        switch (*turn_ptr) {
        case straight:
            instance->UART_OutString("straight\r\n");
            break;
        case right:
            instance->UART_OutString("right\r\n");
            break;
        case left:
            instance->UART_OutString("left\r\n");
            break;
        case back:
            instance->UART_OutString("BACK\r\n");
            break;
        }
        turn_ptr++;
        length_ptr++;
    }
    return 0;
}

uint32_t write_eeprom_config(instance_t *instance, int * none) {
		UNUSED(none);

		data.crc32 = calc_crc32((uint8_t*)&data, sizeof(data)-4);
		spi_write_eeprom(EEPROM_COPY_ADDRESS, (uint8_t*) &data, sizeof(data));  // сначала записываем резерв
		spi_write_eeprom(EEPROM_CONFIG_ADDRESS, (uint8_t*) &data, sizeof(data));	// теперь, нормальный вариант.
//		if (FRAM_log_Start(0x30000)) return 1;
//		if (FRAM_log_write((uint8_t*) &data, NULL, sizeof(data))) return 1;
//		if (FRAM_log_Stop()) return 1;
    return 0;
}

/*
uint32_t read_eeprom_log(int * none) {
    uint8_t status;
    status = i2c_rd_reg16(LOG_EEPROM_ADDR, 0xFFFC, (unsigned char *)&log_size, sizeof(log_size));
    if (status != I2C_SUCCESS) {
        UART1_OutString("\r\nEEPROM not respond");
        return 1;
    }
    log_size ^= 0xffff;
    if (log_size == 0) {
        UART1_OutString("\r\nEEPROM empty"); 
        return 1;
    }
    status = i2c_rd_reg16(LOG_EEPROM_ADDR, 0xFFFE, (unsigned char *)&fps, sizeof(fps));
    if (status) return 1;
    if (log_size > sizeof(log_array)) log_size = sizeof(log_array);
    status = i2c_rd_reg16(LOG_EEPROM_ADDR, 0x0000, log_array, log_size);
    if (status) return 1;
    return 0;
}
*/
uint32_t put_on_stack(instance_t *instance, int * arg) {
    instance->stack[instance->stack_idx] = (uint32_t) arg;
    if (++instance->stack_idx < FORTH_STACK_SIZE) return 0;
    else {
			instance->stack_idx = FORTH_STACK_SIZE - 1;
			return 1;
		}		
}

uint32_t drop_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx) {
        instance->stack_idx--;
        return 0;
    } else
        return 1;
}

uint32_t dup_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx == 0) return 1;
    else {
        instance->stack[instance->stack_idx] = instance->stack[instance->stack_idx - 1];
        if (++instance->stack_idx < FORTH_STACK_SIZE) return 0;
        else {
					instance->stack_idx = FORTH_STACK_SIZE -1;
					return 1;
				}
    }    
}

uint32_t read_mem(instance_t *instance, int * none) {
    uint32_t val;
		UNUSED(none);
    if (instance->stack_idx == 0) return 1;
    val = instance->stack[instance->stack_idx - 1];
    instance->stack[instance->stack_idx - 1] = (uint32_t)(*(int32_t *)val);
    return 0;
}

uint32_t write_mem(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    *(int32_t *)instance->stack[instance->stack_idx - 1] = (int32_t)instance->stack[instance->stack_idx - 2];
    instance->stack_idx -= 2;
    return 0;
}

uint32_t write_byte(instance_t *instance, int * none) {
    UNUSED(none);
    if (instance->stack_idx < 2) return 1;
    *(int8_t *)instance->stack[instance->stack_idx - 1] = ((int32_t)instance->stack[instance->stack_idx - 2]) & 0xff;
    instance->stack_idx -= 2;
    return 0;
}

uint32_t write_word(instance_t *instance, int * none) {
    UNUSED(none);
    if (instance->stack_idx < 2) return 1;
    *(int16_t *)instance->stack[instance->stack_idx - 1] = ((int32_t)instance->stack[instance->stack_idx - 2]) & 0xffff;
    instance->stack_idx -= 2;
    return 0;
}

uint32_t swap_stack(instance_t *instance, int * none) {
    uint32_t val;
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    val = instance->stack[instance->stack_idx - 1];
    instance->stack[instance->stack_idx - 1] = instance->stack[instance->stack_idx - 2];
    instance->stack[instance->stack_idx - 2] = val;
    return 0;
}

uint32_t sum_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    instance->stack[instance->stack_idx - 2] += instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t minus_stack(instance_t *instance, int * none) {
    (void) none;
    if (instance->stack_idx < 2) return 1;
    instance->stack[instance->stack_idx - 2] -= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t lshift_stack(instance_t *instance, int * none) {
    (void) none;
    if (instance->stack_idx < 2) return 1;
    instance->stack[instance->stack_idx - 2] <<= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t mult_stack(instance_t *instance, int * none) {
    (void) none;
    if (instance->stack_idx < 2) return 1;
    instance->stack[instance->stack_idx - 2] *= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t div_stack(instance_t *instance, int * none) {
    (void) none;
    if (instance->stack_idx < 2) return 1;
    instance->stack[instance->stack_idx - 2] /= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t and_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    instance->stack[instance->stack_idx - 2] &= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t or_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    instance->stack[instance->stack_idx - 2] |= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t xor_stack(instance_t *instance, int * none) {
		UNUSED(none);
    if (instance->stack_idx < 2) return 1;    
    instance->stack[instance->stack_idx - 2] ^= instance->stack[instance->stack_idx - 1];
    instance->stack_idx--;
    return 0;
}

uint32_t set_base(instance_t *instance, int * base_val) {
    if ((uint32_t)base_val < 2) return 1;
    instance->base = (uint32_t)base_val;
    return 0;
}

uint32_t show_stack(instance_t *instance, int * none) {
    char str[11];
    unsigned int val, i, j;
    UNUSED(none);
		for (i = instance->stack_idx ; i > 0 ; i--) {
        val = instance->stack[i - 1];
        for (j = 8; j > 0; j--) {
            str[j-1] = (val & 0x0000000Ful) + '0';
            if (str[j-1] > '9') str[j-1] += ('@'-'9');
            val >>= 4;
        }
        str[8] = '\r';
        str[9] = '\n';
				str[10] = '\0';
        instance->UART_OutString(str); /* Echo data back to PC */
    }
    return 0;
}

uint32_t print_space(instance_t *instance, int * none) {
		UNUSED(none);
		instance->UART_OutChar(' ');
		return 0;
}

uint32_t print_newline(instance_t *instance, int * none) {
		UNUSED(none);
		instance->UART_OutString("\r\n");
		return 0;
}

void print_digit(instance_t *instance, uint32_t n){
// This function uses recursion to convert number
//   of unspecified length as an ASCII string
		if(n >= instance->base){
				print_digit(instance, n/instance->base);
				n = n%instance->base;
		}
		if (n < 10)
				instance->UART_OutChar(n+'0'); // n is between 0 and 9
		else 
				instance->UART_OutChar((n-10) + 'A'); // n is over 10 
}

uint32_t OutNum(instance_t *instance, int * none) {
		uint32_t val;
		UNUSED(none);

		if (instance->stack_idx == 0) return 1;
		val = instance->stack[--instance->stack_idx];

		print_digit(instance, val);
		return 0;
}


#define ISALPHA(X)  (((X) >= ' ') && ((X) <= 0x7f))

void print_row(instance_t *instance, unsigned int addr, unsigned int addrsize, uint8_t *scratchpad, unsigned int rowsize){
    unsigned int val, ii, jj;
    char str[10];

    val = addr;
    for (ii = addrsize; ii > 0; ii--) {
        str[ii-1] = hextable[val & 0x0F];
        val >>= 4;
    }
    str[addrsize] = ' ';
    str[addrsize+1] = '\0';
    instance->UART_OutString(str);
    for (jj = 0; jj < rowsize; jj++) {
        instance->UART_OutChar(hextable[(scratchpad[jj] >> 4) &0x0F]);
        instance->UART_OutChar(hextable[scratchpad[jj] & 0x0F]);
        instance->UART_OutChar(' ');
    }
    for (jj = 0; jj < rowsize; jj++) {
        if (ISALPHA(scratchpad[jj])) instance->UART_OutChar(scratchpad[jj]);
        else                         instance->UART_OutChar('.');
    }
    instance->UART_OutString("\r\n");
}

uint32_t fram_rdsr(instance_t *instance, int * none) {
    uint8_t status_value;
    UNUSED(none);

    if (FRAM_rdsr(&status_value)) return 1;
    if (instance->stack_idx < FORTH_STACK_SIZE - 1) {
            instance->stack[instance->stack_idx++] = status_value;
            return 0;
    } else {
            return 1;
    }
}

uint32_t fram_wrsr(instance_t *instance, int * none) {
    uint8_t status_value;
    UNUSED(none);

    if (instance->stack_idx < 1) return 1;
    status_value = instance->stack[--instance->stack_idx];
    if (FRAM_wrsr(status_value)) return 1;
    return 0;
}

uint32_t dump_fram(instance_t *instance, int * none) {
        uint32_t addr, size,  rowsize, addrsize = 4, ii, mask;
        uint8_t scratchpad[16];
        UNUSED(none);

        if (instance->stack_idx < 2) return 1;
        size = instance->stack[--instance->stack_idx];
        addr = instance->stack[--instance->stack_idx];

        mask = (0x0Ful << addrsize*4);
        for (ii = addrsize; ii < 8; ii++){
            if ((addr+size) & mask) addrsize = ii+1;
            mask <<= 4;
        }

        if (FRAM_read_Start(addr)) return 1;
        while (size) {
            rowsize = 16 - (addr % 16);
            if (rowsize > size) rowsize = size;
            if (FRAM_log_write(scratchpad, scratchpad, rowsize)) return 1;
            print_row(instance, addr, addrsize, scratchpad, rowsize);
            addr += rowsize;
            size -= rowsize;
        }
        return FRAM_log_Stop();
}

uint32_t fram_log_decode (instance_t *instance, int *none) {
    static data_buffer_t log_buffer;
    unsigned int record_count = 0, ii, jj, mask;

    if (FRAM_read_Start(0x00000)) return 1;
    FRAM_wait_EOT();
    instance->UART_OutChar('.');
    if (FRAM_log_write((uint8_t*)&record_count, (uint8_t*)&record_count, sizeof(record_count))) return 1;
    FRAM_wait_EOT();
    instance->UART_OutString(".\r\n");

    for (ii = 0; ii < record_count; ii++) {
        if (FRAM_log_write((uint8_t*)&log_buffer, (uint8_t*)&log_buffer, sizeof(log_buffer))) return 1;
        FRAM_wait_EOT();
        instance->UART_OutUDec(log_buffer.Time);
        instance->UART_OutChar(',');
        mask = 1;
        for (jj = 0; jj < 8; jj++) {
            if (log_buffer.sensors & mask) instance->UART_OutChar('#');
            else                           instance->UART_OutChar(' ');
            mask <<= 1;
        }
        instance->UART_OutChar(',');
        instance->UART_OutUDec(log_buffer.vbat);
        instance->UART_OutChar(',');
        if (log_buffer.setspeedLeft < 0) {
            log_buffer.setspeedLeft = -log_buffer.setspeedLeft;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.setspeedLeft);
        instance->UART_OutChar(',');
        if (log_buffer.setspeedRight < 0) {
            log_buffer.setspeedRight = -log_buffer.setspeedRight;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.setspeedRight);
        instance->UART_OutChar(',');
        if (log_buffer.RealSpeedLeft < 0) {
            log_buffer.RealSpeedLeft = -log_buffer.RealSpeedLeft;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.RealSpeedLeft);
        instance->UART_OutChar(',');
        if (log_buffer.RealSpeedRight < 0) {
            log_buffer.RealSpeedRight = -log_buffer.RealSpeedRight;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.RealSpeedRight);
        instance->UART_OutChar(',');

        if (log_buffer.StepsLeft < 0) {
            log_buffer.StepsLeft = -log_buffer.StepsLeft;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.StepsLeft);
        instance->UART_OutChar(',');

        if (log_buffer.StepsRight < 0) {
            log_buffer.StepsRight = -log_buffer.StepsRight;
            instance->UART_OutChar('-');
        }
        instance->UART_OutUDec(log_buffer.StepsRight);

        instance->UART_OutString("\r\n");
    }


    return FRAM_log_Stop();
}

uint32_t dump_mem(instance_t *instance, int * none) {
        uint32_t addr, size,  rowsize, addrsize = 4, ii, mask;
        UNUSED(none);

        if (instance->stack_idx < 2) return 1;
        size = instance->stack[--instance->stack_idx];
        addr = instance->stack[--instance->stack_idx];

        mask = (0x0Ful << addrsize*4);
        for (ii = addrsize; ii < 8; ii++){
            if ((addr+size) & mask) addrsize = ii+1;
            mask <<= 4;
        }

        while (size) {
            rowsize = 16 - (addr % 16);
            if (rowsize > size) rowsize = size;
            print_row(instance, addr, addrsize, (uint8_t *)addr, rowsize);
            addr += rowsize;
            size -= rowsize;
        }
        return 0;
}

uint32_t crc_calculate(instance_t *instance, int * none) {
        UNUSED(none);
    if (instance->stack_idx < 2) return 1;
    instance->stack[instance->stack_idx - 2] = calc_crc32(
                                (uint8_t*)instance->stack[instance->stack_idx - 2],
                                          instance->stack[instance->stack_idx - 1]);
    instance->stack_idx--;
    return 0;
}

uint32_t Battery(instance_t *instance, int * none) {
		
		UNUSED(none);
		if (instance->stack_idx < FORTH_STACK_SIZE - 1) {
				instance->stack[instance->stack_idx++] = get_battery_voltage();
				return 0;
		} else {
				return 1;
		}		
}

uint32_t dump_map(instance_t *instance, int * none)	{
	map_cell_t *ptr;
	unsigned int i, j;
	
	UNUSED(none);
    spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
    ptr = (map_cell_t*) &map;
    for (i=0; i<data.map_size; i++) {
        if (ptr->coordinate.east < 0) {
            instance->UART_OutChar('-');
            instance->UART_OutUDec(-(ptr->coordinate.east));
        } else instance->UART_OutUDec(ptr->coordinate.east);
        instance->UART_OutChar('\t');
        if (ptr->coordinate.north < 0) {
            instance->UART_OutChar('-');
            instance->UART_OutUDec(-(ptr->coordinate.north));
        } else instance->UART_OutUDec(ptr->coordinate.north);
        instance->UART_OutChar('\t');
        for (j=0; j<4; j++) {
            if (ptr->node_link[j] == UNKNOWN) {
                instance->UART_OutChar('-');
                instance->UART_OutChar('1');
            } else instance->UART_OutUDec(ptr->node_link[j]);
        instance->UART_OutChar('\t');
        }
        for (j=0; j<4; j++) {
            instance->UART_OutUDec(ptr->pass_count[j]);
            instance->UART_OutChar('\t');
        }
        instance->UART_OutChar('\r');
        instance->UART_OutChar('\n');
        ptr++;
    }
	return 0;
}

uint32_t put_threshold(instance_t *instance, int * none) {
  unsigned int val;
  UNUSED(none);

  if (instance->stack_idx == 0) return 1;
  val = instance->stack[--instance->stack_idx];
  data.threshold = val;
  set_threshold(val);
  return 0;
}

uint32_t search_way(instance_t *instance, int * none) {
    Search_Short_Way_with_turns();
    return 0;
}


typedef struct {
    char CmdName[12]; // name of command
    uint32_t (*fnctPt)(instance_t *, int *); // to execute this command
    int *param;
}Cmd_t;

const Cmd_t Table[]={
	{"drop",            &drop_stack, NULL},
	{"dup",             &dup_stack, NULL},
	{"swap",            &swap_stack, NULL},
	{"@",               &read_mem, NULL},
	{"+",               &sum_stack, NULL},
    {"*",               &mult_stack, NULL},
    {"/",               &div_stack, NULL},
    {"-",               &minus_stack, NULL},
    {"<<",              &lshift_stack, NULL},
	{"!",               &write_mem, NULL},
    {"b!",              &write_byte, NULL},
    {"w!",              &write_word, NULL},
	{"&",               &and_stack, NULL},
	{"|",               &or_stack, NULL},
	{"^",               &xor_stack, NULL},
	{".",				&OutNum, NULL},
	{"sp",				&print_space, NULL},
	{"cr",				&print_newline, NULL},
	{"show",            &show_stack, NULL},
	{"hex",             &set_base, (int *) 0x10},
	{"bin",             &set_base, (int *) 0x02},
	{"decimal",         &set_base, (int *) 0x0a},
	{"vbat",			&Battery,	NULL},
	{"crc",             &crc_calculate, NULL},
	{"threshold",       &put_threshold, NULL},
	{"Threshold",       &put_on_stack, &data.threshold},
	{"MAXspeed",        &put_on_stack, &data.maxspeed},
	{"MINspeed",        &put_on_stack, &data.minspeed},
	{"MAXmotor",        &put_on_stack, &data.maxmotor},
    {"TurnSpeed",       &put_on_stack, &data.turnspeed},
	{"Accelerat",       &put_on_stack, &data.acceleration},
	{"K*error",         &put_on_stack, &data.k_error},
	{"K*de/dt",         &put_on_stack, &data.k_diff},
    {"Kintegral",       &put_on_stack, &data.k_integral},
	{"MotorKprop",      &put_on_stack, &data.motor_Kprop},
	{"MotorKint",       &put_on_stack, &data.motor_Kint},
	{"OnWay",           &put_on_stack, &data.on_way},
	{"RunNumber",       &put_on_stack, &data.runnumber},
	{"PathLength",		&put_on_stack, &data.pathlength},
	{"MapSize", 		&put_on_stack, &data.map_size},
    {"StartPoint",      &put_on_stack, &data.green_cell_nr},
    {"EndPoint",        &put_on_stack, &data.red_cell_nr},
	{"IRlevel", 		&put_on_stack, &data.ir_led_level},
	{"map", 		    &put_on_stack, (int*)&map},
	{"LeftHand", 		&put_on_stack, &data.lefthand},
	{"ColorRed", 		&put_on_stack, &data.color_red_thr},
	{"ColorGreen", 		&put_on_stack, &data.color_green_thr},
	{"ColorBlue", 		&put_on_stack, &data.color_blue_thr},
	{"ColorThr", 		&put_on_stack, &data.color_threshold},
	{"ColorSat",        &put_on_stack, &data.color_saturation},
	{"DstToleran", 		&put_on_stack, &data.tolerance},
	{"GuardDist", 		&put_on_stack, &data.guarddist},
	{"SensorOffs", 		&put_on_stack, &data.sensor_offset},
	{"IgnoreError", 	&put_on_stack, &data.ignore_coordinate_error},
	{"BrakePath", 		&put_on_stack, &data.brake_path},
	{"TimeToRun", 		&put_on_stack, &data.timetorun},
	{"TurnCost", 		&put_on_stack, &data.turncost},
	{"CrossCost", 		&put_on_stack, &data.crosscost},
    {"StepCost",        &put_on_stack, &data.stepcost},
    {"StrAccel",        &put_on_stack, &data.str_accel},
    {"CellStep",        &put_on_stack, &data.p_step},
    {"Volt_calibr",     &put_on_stack, &data.volt_calibr},
    {"Bat_low_lvl",     &put_on_stack, &data.low_battery_level},
    {"Length_tab",      &put_on_stack,  data.length},
    {"Watermark",       &put_on_stack, &data.log_watermark},
    {"searchWay",       &search_way, NULL},
    {"save_conf",       &write_eeprom_config, NULL},
    {"dump",            &dump_mem,  NULL},
    {"dump_log",        &dump_log, (int *) 0},
    {"dump_log_ft",     &dump_log, (int *) 1},
    {"fram_log",        &fram_log_decode, NULL},
	{"dump_map",		&dump_map, NULL},
	{"dump_fram",       &dump_fram, NULL},
    {"fram_rdsr",       &fram_rdsr, NULL},
    {"fram_wrsr",       &fram_wrsr, NULL},
    {"show_path",       &show_path, NULL},
    {"list",            &list_values, NULL},
	{"words",           &list_cmd, NULL},
};
#define TABLE_SIZE (sizeof(Table)/sizeof(Table[0]))

uint32_t list_cmd(instance_t *instance, int * none) {
    char string_buf[64], *CmdName_ptr, *buf_ptr;
    unsigned int charcount, i;
    
    buf_ptr = string_buf;
    for (i = 0; i < TABLE_SIZE; i++) {
        charcount = 12;
        CmdName_ptr = (char *)&(Table[i].CmdName);
        while (*CmdName_ptr) {
            *buf_ptr++ = *CmdName_ptr++;
            charcount--;
        }
        while (charcount--) *buf_ptr++ = ' ';
        if ((((i+1) % 5) == 0) || (i == (TABLE_SIZE-1))) {
            *buf_ptr++ = '\r';
            *buf_ptr++ = '\n';
						*buf_ptr = '\0';
            instance->UART_OutString(string_buf);
            buf_ptr = string_buf;
        }
    }
    return 0;
}

uint32_t list_values(instance_t *instance, int * none) {
    char string_buf[14], *buf_ptr, *CmdName_ptr;
    unsigned int charcount, i;
    
    buf_ptr = string_buf;
    for (i = 0; i< TABLE_SIZE; i++) {
        if (Table[i].fnctPt != &put_on_stack) continue;
        charcount = 14;
        buf_ptr  =  string_buf;
        CmdName_ptr = (char *)&(Table[i].CmdName);
        while (*CmdName_ptr) {
            *buf_ptr++ = *CmdName_ptr++;
            charcount--;
        }
        while (charcount--) *buf_ptr++ = ' ';
				*buf_ptr = '\0';
        instance->UART_OutString(string_buf);
        if (Table[i].fnctPt(instance, Table[i].param)) return 1;
        if (read_mem(instance, NULL)) return 1;
        if (OutNum(instance, NULL)) return 1;
        if (print_newline(instance, NULL)) return 1;
    }
    return 0;
}

void parse_string(instance_t *instance) {
    unsigned int i, j, input_num, digit;
    unsigned char *str_ptr, *in_ptr;
    str_ptr = instance->input_string;
		instance->UART_OutString("\r\n");

		while(*str_ptr) {
        while (*str_ptr == ' ') ++str_ptr;
        if (*str_ptr == '\0') break;
        for (i = 0; i < TABLE_SIZE; i++) {
            j = 0;
            in_ptr = str_ptr;
            while (Table[i].CmdName[j]) {
                if (*in_ptr != Table[i].CmdName[j]) break;
                in_ptr++; j++;
            }
            if ((Table[i].CmdName[j] == '\0') && ((*in_ptr == '\0') || (*in_ptr == ' '))) {
                str_ptr = in_ptr;
                if (Table[i].fnctPt(instance, Table[i].param)) {
                    instance->UART_OutString("Command error: \0");
                    instance->UART_OutString((char *)Table[i].CmdName);
                    *in_ptr = '\0'; // обрываем строку
                }
                break;
            }
        }
        if (i == TABLE_SIZE) {
            unsigned int iserror = 0;
            input_num = 0;
            while ((*in_ptr != ' ') && (*in_ptr != '\0')) {
                digit = *in_ptr - '0';
                if (digit > 9) digit -= ('@'-'9');
                if (digit < instance->base) {
                    input_num = input_num*instance->base + digit;
                    in_ptr++;
                } else {
                    iserror = 1;
                    while ((*in_ptr != '\0') && (*in_ptr != ' ')) ++in_ptr;
                    *in_ptr = '\0'; // обрываем строку
										instance->UART_OutString("Illegal command: \0");
                    instance->UART_OutString((char*)str_ptr);
                    *str_ptr = '\0'; // обрываем строку
                    break;
                }
            }
            if (!iserror) {
                if (put_on_stack(instance, (int *) input_num)) {
                    instance->UART_OutString("Stack overflow\r\n");
                    *in_ptr = '\0'; // обрываем строку
                }
            }
            str_ptr = in_ptr;
        }
    }
}

void BlueTooth_parse(void) {
#ifdef WITH_BGX
	if (UART_InStatus(&bgx_uart)) {
		uint8_t control_char;
		control_char = UART1_InChar();
		if (STREAM_BIT) {
			if (input_command(&BGX_UART1, control_char)) {
				parse_string(&BGX_UART1);
				BGX_UART1.UART_OutString("\r\nGriever>"); // Echo data back to PC
			}
		}
	}	
#endif
    if (UART_InStatus(&usb_uart)) {
        uint8_t control_char;
        control_char = UART0_InChar();
        if (input_command(&USBUART, control_char)) {
            parse_string(&USBUART);
            USBUART.UART_OutString("\r\nGriever>"); // Echo data back to PC
        }
    }
}


/* [] END OF FILE */
