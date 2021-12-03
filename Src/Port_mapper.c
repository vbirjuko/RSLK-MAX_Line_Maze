/*
 * Port_mapper.c
 *
 *  Created on: 17 мар. 2019 г.
 *      Author: wl
 */
#include "msp.h"
#include "driverlib.h"

const uint8_t p7map[] = {PMAP_NONE,
                         PMAP_NONE,
                         PMAP_TA1CCR2A,
                         PMAP_TA1CCR3A,
                         PMAP_TA1CCR4A,
                         PMAP_TA1CCR1A,
                         PMAP_NONE,
                         PMAP_NONE
};


void Port_Mapper_Init(void) {
    MAP_PMAP_configurePorts(p7map, PMAP_P7MAP, 1, PMAP_DISABLE_RECONFIGURATION);

/*

    PMAPKEYID = PMAP_KEYID_VAL;

    P7MAP->PMAP_REGISTER7 = PMAP_NONE;     // отключаем устройство от порта по умолчанию
    P7MAP->PMAP_REGISTER6 = PMAP_NONE;     // отключаем устройство от порта по умолчанию
    P7MAP->PMAP_REGISTER5 = PMAP_TA1CCR1A; 
		P7MAP->PMAP_REGISTER4 = PMAP_TA1CCR4A;
    P7MAP->PMAP_REGISTER3 = PMAP_TA1CCR3A; // и подключаем к другому порту
		P7MAP->PMAP_REGISTER2 = PMAP_TA1CCR2A;
    P7MAP->PMAP_REGISTER1 = PMAP_NONE;     // отключаем устройство от порта по умолчанию
    P7MAP->PMAP_REGISTER0 = PMAP_NONE;     // отключаем устройство от порта по умолчанию
		
    PMAPKEYID = 0;

*/
}
