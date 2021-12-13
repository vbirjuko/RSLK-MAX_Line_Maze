/*
 * FRAM_Logging.h
 *
 *  Created on: 5 мая 2021 г.
 *      Author: wl
 */

#ifndef FRAM_LOGGING_H
#define FRAM_LOGGING_H


void FRAM_Logging_Init(void);
unsigned int FRAM_log_Start(uint32_t FRAM_Addr);
unsigned int FRAM_read_Start(uint32_t FRAM_Addr);
unsigned int FRAM_log_Stop(void);
unsigned int FRAM_log_write(uint8_t *wr_data_ptr, uint8_t *rd_data_ptr, unsigned int wr_data_size);
unsigned int FRAM_rdsr(uint8_t *block_protection);
unsigned int FRAM_wrsr(uint8_t block_protection);
void FRAM_log_data(void);

#endif
