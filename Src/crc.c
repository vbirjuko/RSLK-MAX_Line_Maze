/*
 * crc.c
 *
 *  Created on: 2 янв. 2021 г.
 *      Author: wl
 */
#include "driverlib.h"
//#include "dma.h"
#include "CortexM.h"

#define CRC32_INIT  0xFFFFFFFF

volatile unsigned int dma_crc_busy = 0;

uint32_t calc_crc32(uint8_t* ptr, uint32_t size) {

    uint32_t transferbytes;
    MAP_CRC32_setSeed(CRC32_INIT, CRC32_MODE);

    while (size) {
        if (size > 1024) transferbytes = 1024;
        else             transferbytes = size;
        size -= transferbytes;

        /* Setting Control Indexes. In this case we will set the source of the
         * DMA transfer to our random data array and the destination to the
         * CRC32 data in register address*/
        MAP_DMA_setChannelControl(DMA_CH4_RESERVED0 | UDMA_PRI_SELECT,
                UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

        MAP_DMA_setChannelTransfer(DMA_CH4_RESERVED0 | UDMA_PRI_SELECT,
                UDMA_MODE_AUTO, ptr,
                (void*) (CRC32_BASE + OFS_CRC32DI), transferbytes);

        ptr += transferbytes;

        /* Assigning/Enabling Interrupts */
        DMA_assignInterrupt(DMA_INT1, 4);
        MAP_Interrupt_enableInterrupt(INT_DMA_INT1);
        MAP_Interrupt_enableMaster();

        /* Enabling DMA Channel 0 */
        MAP_DMA_enableChannel(4);

        /* Forcing a software transfer on DMA Channel 0 */
        dma_crc_busy = 1;
        MAP_DMA_requestSoftwareTransfer(4);

        while(dma_crc_busy) {
            WaitForInterrupt();
        }
    }
    return MAP_CRC32_getResultReversed(CRC32_MODE);
}

/* Completion interrupt for DMA */
void DMA_INT1_IRQHandler(void)
{
    MAP_DMA_disableChannel(4);
    dma_crc_busy = 0;
}
