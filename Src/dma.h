/*
 * dma.h
 *
 *  Created on: 22 нояб. 2020 г.
 *      Author: wl
 */

#ifndef DMA_H_
#define DMA_H_

#include "driverlib.h"

extern DMA_ControlTable DMAControlTable[8];
extern volatile unsigned int     dma_copy_busy;

void DMA_Init(void);
void copy_data_dma (uint8_t * src, uint8_t * dst, uint16_t count);
void copy_path_dma (uint8_t * src, uint32_t * dst, uint16_t count);
void fill_data32_dma(uint32_t value, uint32_t * dst, uint16_t count);
void fill_data8_dma(uint8_t value, uint8_t * dst, uint16_t count);

#endif /* DMA_H_ */
