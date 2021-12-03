#include "dma.h"
#include "SPI_EEProm.h"
#include "Maze.h"
#include "main.h"
#include "configure.h"
#include "Logging.h"
#include "Reflectance.h"
#include "Timer32.h"

typedef struct  {
	uint8_t 	command[4];
	uint16_t	data[128];
} spi_buffer_t;
// Available
spi_buffer_t spi_buffer[2];
static uint16_t *buffer_ptr;
static uint32_t eeprom_address, current_buffer = 0, log_state = 0, log_size, log_watermark, log_place_available;
static unsigned int buffer_count = 0;

void data_log_init(void) {
    uint8_t erase_buffer[4];
    unsigned int i;

    current_buffer = 0;
    eeprom_address = LOG_ADDRESS;

    for (i=0; i < 6; i++) {
        erase_buffer[0] = eeprom_write_enable;
        spi_exchange(erase_buffer, 1);

        erase_buffer[0] = eeprom_block_erase;
        erase_buffer[1] = ((eeprom_address & 0x0ff0000) >> 16) & 0xFF;
        erase_buffer[2] = ((eeprom_address & 0x000ff00) >>  8) & 0xFF;
        erase_buffer[3] = ((eeprom_address & 0x00000ff) >>  0) & 0xFF;
        spi_exchange(erase_buffer, 4);

        do {
            delay_us(200*500);
            erase_buffer[0] = eeprom_read_status_register;
            spi_exchange(erase_buffer, 1);
        } while (erase_buffer[0] & EEPROM_STATUS_WIP);
        eeprom_address += 0x10000;
        if (eeprom_address > LOG_END_ADDRESS) break;
    }
    log_place_available = 1;
    eeprom_address = LOG_ADDRESS;
    log_size = 0;
    current_buffer  = 0;
    buffer_ptr = spi_buffer[current_buffer].data;
    buffer_count = 0;
    log_watermark = 0;
    data_log(0xffff, 1);
    data_log(0xffff, 1); // placeholder for log size
}

void data_log(uint16_t datatowrite, unsigned int write_enable) {

    if (write_enable && log_place_available) {
        *buffer_ptr++ = datatowrite;
        log_size++;
    }

    if (++buffer_count > 127) {

        current_buffer ^= 1;
        buffer_count = 0;
        buffer_ptr = &spi_buffer[current_buffer].data[0];
        log_state = 1;
    }
    switch (log_state) {
    case 1:
        if (DMA_getChannelMode(6) != UDMA_MODE_STOP) break;
        spi_buffer[current_buffer ^ 1].command[0] = eeprom_write_enable;
        spi_exchange(spi_buffer[current_buffer ^ 1].command, 1);
        log_state = 2;
        break;

    case 2:
        if (DMA_getChannelMode(6) != UDMA_MODE_STOP) break;
        spi_buffer[current_buffer ^ 1].command[0] = eeprom_page_program;
        spi_buffer[current_buffer ^ 1].command[1] = ((eeprom_address & 0x0ff0000) >> 16) & 0xFF;
        spi_buffer[current_buffer ^ 1].command[2] = ((eeprom_address & 0x000ff00) >>  8) & 0xFF;
        spi_buffer[current_buffer ^ 1].command[3] = ((eeprom_address & 0x00000ff) >>  0) & 0xFF;
        spi_exchange(spi_buffer[current_buffer ^ 1].command, 256+4);
        eeprom_address += 256;
        if ((eeprom_address + 256) > LOG_END_ADDRESS) log_place_available = 0;
        log_state = 3;
        break;

    case 3:
        if (DMA_getChannelMode(6) != UDMA_MODE_STOP) break;
        spi_buffer[current_buffer ^ 1].command[0] = eeprom_read_status_register;
        spi_exchange(spi_buffer[current_buffer ^ 1].command, 1);
        if (spi_buffer[current_buffer ^ 1].command[0] & EEPROM_STATUS_WIP) break;
        if (log_watermark < buffer_count) log_watermark = buffer_count;
        log_state = 0;
        break;
    }
}

void data_log_finish(void) {

        // Если запись предыдущего буфера в процессе,
        // то ждём его завершения.
        while (log_state) {
            if (photo_data_ready) {
                photo_data_ready = 0;
                data_log(0xFFFF, 0);
            }
        }
        // Теперь завершаем текущий буфер и ждём
        // завершения уже его записи.
        buffer_count = 255;
        data_log(0xFFFF, 0);
        while (log_state) {
            if (photo_data_ready) {
                photo_data_ready = 0;
                data_log(0xFFFF, 0);
            }
        }

        // записываем в самое начала размер сохранённого журнала
        spi_buffer[0].data[0] = (log_size >>  0) & 0x0000ffff;
        spi_buffer[0].data[1] = (log_size >> 16) & 0x0000ffff;
        spi_buffer[0].command[0] = eeprom_write_enable;
        spi_exchange(spi_buffer[0].command, 1);

        spi_buffer[0].command[0] = eeprom_page_program;
        spi_buffer[0].command[1] = ((LOG_ADDRESS & 0x0ff0000) >> 16) & 0xFF;
        spi_buffer[0].command[2] = ((LOG_ADDRESS & 0x000ff00) >>  8) & 0xFF;
        spi_buffer[0].command[3] = ((LOG_ADDRESS & 0x00000ff) >>  0) & 0xFF;
        spi_exchange(spi_buffer[0].command, 4+4);

        do {
            while (!photo_data_ready) continue;
            photo_data_ready = 0;
            spi_buffer[0].command[0] = eeprom_read_status_register;
            spi_exchange(spi_buffer[0].command, 1);
        } while (spi_buffer[0].command[0] & EEPROM_STATUS_WIP);
        data.log_watermark = log_watermark;

}
