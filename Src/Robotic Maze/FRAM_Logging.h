/*
 * FRAM_Logging.h
 *
 *  Created on: 5 мая 2021 г.
 *      Author: wl
 */

#ifndef FRAM_LOGGING_H
#define FRAM_LOGGING_H

    typedef struct data_buffer {
        int16_t setspeedLeft;       // 2
        int16_t setspeedRight;      // 2
        int16_t RealSpeedLeft;      // 2
        int16_t RealSpeedRight;     // 2
        int32_t StepsLeft;          // 4
        int32_t StepsRight;         // 4
        uint32_t Time;              // 4
        uint8_t vbat;
        uint8_t sensors;
    }data_buffer_t;

void FRAM_Logging_Init(void);
unsigned int FRAM_log_Start(uint32_t FRAM_Addr);
unsigned int FRAM_read_Start(uint32_t FRAM_Addr);
unsigned int FRAM_log_Stop(void);
unsigned int FRAM_log_write(uint8_t *wr_data_ptr, uint8_t *rd_data_ptr, unsigned int wr_data_size);
void FRAM_wait_EOT(void);
unsigned int FRAM_rdsr(uint8_t *block_protection);
unsigned int FRAM_wrsr(uint8_t block_protection);
void FRAM_log_data(void);
extern unsigned int frames_to_go;

#endif
