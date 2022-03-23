/*
 * FRAM_Logging.h
 *
 *  Created on: 5 мая 2021 г.
 *      Author: wl
 */

#ifndef FRAM_LOGGING_H
#define FRAM_LOGGING_H

#include "resources.h"
#include "Maze.h"

// Define FRAM size in kilobytes.
// From this size depends how many records to write.
// now 22 bytes per record writes 400 times in second gives approx. 30 seconds.
//If logging is not necessary define as 0.
#ifndef FRAM_SIZE
#define FRAM_SIZE   (0)
#endif


    typedef struct data_buffer {
        uint32_t Time;              // 4
        union{
            struct{
                int16_t setspeedLeft;       // 2
                int16_t setspeedRight;      // 2
                int16_t RealSpeedLeft;      // 2
                int16_t RealSpeedRight;     // 2
                int32_t StepsLeft;          // 4
                int32_t StepsRight;         // 4
                uint8_t vbat;
                uint8_t sensors;
            };
            struct{
                int coordX;                 // 4
                int coordY;                 // 4
                unsigned int segmentLength; // 4
                int nodeNum;                // 4
            };
        };
        uint8_t whereami;
    }  __attribute__((packed)) data_buffer_t;

void FRAM_Logging_Init(void);

//unsigned int FRAM_log_Start(uint32_t FRAM_Addr);
//unsigned int FRAM_read_Start(uint32_t FRAM_Addr);
//unsigned int FRAM_log_Stop(void);
//unsigned int FRAM_log_write(uint8_t *wr_data_ptr, uint8_t *rd_data_ptr, unsigned int wr_data_size);
//void FRAM_wait_EOT(void);
//unsigned int FRAM_rdsr(uint8_t *block_protection);
//unsigned int FRAM_wrsr(uint8_t block_protection);

void FRAM_log_data(void);
void FRAM_log_slow(coordinate_t coordinate,  int length, unsigned int index);
extern unsigned int frames_to_go;


unsigned int FRAM_dma_log_write(uint8_t *wr_data_ptr, unsigned int wr_data_size);
unsigned int FRAM_dma_log_Stop(void);
unsigned int FRAM_dma_log_Start(uint32_t FRAM_Addr);
unsigned int FRAM_dma_read_Start(uint32_t FRAM_Addr);
unsigned int FRAM_dma_log_read(uint8_t *rd_data_ptr, unsigned int rd_data_size);
void FRAM_dma_wait_EOT(void);
unsigned int FRAM_dma_wrsr(uint8_t block_protection);
unsigned int FRAM_dma_rdsr(uint8_t *block_protection);

#endif
