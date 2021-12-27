/*
 * Timer32.h
 *
 *  Created on: 7 янв. 2021 г.
 *      Author: wl
 */

#ifndef TIMER32_H
#define TIMER32_H

void Timer32_Init(void);
void delay_us(uint32_t delay);

void benchmark_start(void) ;
unsigned int benchmark_stop(void);

#endif /* TIMER32_H_ */
