/*
 * dma.c
 *
 *  Created on: 22 нояб. 2020 г.
 *      Author: wl
 *
 *  DMA channel usage:
 *
 *  Channel 0 - SPI FRAM EUSCI_B0 TX
 *  Channel 1 - SPI FRAM EUSCI_B0 RX
 *  Channel 2 - Reserved for I2c EUSCI_B1 TX
 *  Channel 3 - Reserved for I2c EUSCI_B1 RX
 *  Channel 4 - CRC
 *  Channel 5 - RAM to RAM
 *  Channel 6 - SPI EEPROM over EUSCI_B3 TX, SPI Display over EUSCI_A3 TX
 *  Channel 7 - SPI EEPROM over EUSCI_B3 RX.
 *
 *      DMA IRQ Usage:
 *      INT3 - memory to memory
 *      INT2 - display
 *      INT1 - CRC32
 *      INT0 - SPI FRAM
 */
#include "driverlib.h"

volatile unsigned int     dma_copy_busy = 0;

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif

DMA_ControlTable DMAControlTable[8];

void DMA_Init(void) {
    /* Configuring DMA module */
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(DMAControlTable);

}

void copy_data_dma (uint8_t * src, uint8_t * dst, uint16_t count) {

    while (dma_copy_busy) continue;

    dma_copy_busy = 1;

    /* Setting Control Indexes. In this case we will set the source of the
     * DMA transfer to our random data array and the destination to the
     * destination data array. Set as auto mode with no need to retrigger
     * after each arbitration */

    if ((((uint32_t)src | (uint32_t)dst | (uint32_t)count) & 0x03UL) == 0) {
        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CHANNEL_5,
                UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 | UDMA_ARB_4);
        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CHANNEL_5, UDMA_MODE_AUTO, src,
                dst, count >> 2);
    } else if ((((uint32_t)src | (uint32_t)dst | (uint32_t)count) & 0x01UL) == 0) {
        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CHANNEL_5,
                UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_16 | UDMA_ARB_4);
        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CHANNEL_5, UDMA_MODE_AUTO, src,
                dst, count >> 1);
    } else {
        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CHANNEL_5,
                UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_8 | UDMA_ARB_4);
        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CHANNEL_5, UDMA_MODE_AUTO, src,
                dst, count);
    }
    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT3, DMA_CHANNEL_5);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT3);
    MAP_Interrupt_disableSleepOnIsrExit();

    /* Enabling DMA Channel 0 */
    MAP_DMA_enableChannel(DMA_CHANNEL_5);

    /* Forcing a software transfer on DMA Channel 0 */
    MAP_DMA_requestSoftwareTransfer(DMA_CHANNEL_5);

}

/* Completion interrupt for DMA */
void DMA_INT3_IRQHandler(void)
{
    MAP_DMA_disableChannel(DMA_CHANNEL_5);
    dma_copy_busy = 0;
}


void fill_data32_dma(uint32_t value, uint32_t * dst, uint16_t count) {
    static uint32_t value_store;

    while (dma_copy_busy) continue;

    dma_copy_busy = 1;

    value_store = value;

    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CHANNEL_5,
            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_4);
    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CHANNEL_5, UDMA_MODE_AUTO, &value_store,
            dst, count);

    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT3, DMA_CHANNEL_5);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT3);
    MAP_Interrupt_disableSleepOnIsrExit();

    /* Enabling DMA Channel 0 */
    MAP_DMA_enableChannel(DMA_CHANNEL_5);

    /* Forcing a software transfer on DMA Channel 0 */
    MAP_DMA_requestSoftwareTransfer(DMA_CHANNEL_5);
}

void fill_data8_dma(uint8_t value, uint8_t * dst, uint16_t count) {
    static uint8_t value_store;

    while (dma_copy_busy) continue;

    dma_copy_busy = 1;

    value_store = value;

    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CHANNEL_5,
            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);
    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CHANNEL_5, UDMA_MODE_AUTO, &value_store,
            dst, count);

    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT3, DMA_CHANNEL_5);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT3);
    MAP_Interrupt_disableSleepOnIsrExit();

    /* Enabling DMA Channel 0 */
    MAP_DMA_enableChannel(DMA_CHANNEL_5);

    /* Forcing a software transfer on DMA Channel 0 */
    MAP_DMA_requestSoftwareTransfer(DMA_CHANNEL_5);
}
