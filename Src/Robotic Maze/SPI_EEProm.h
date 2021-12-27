#ifndef SPI_EEPROM_H
#define SPI_EEPROM_H

void SPI_EEProm_Init(void);

enum eeprom_commands {
	eeprom_write_enable = 0x06,
	eeprom_write_disable = 0x04,
	eeprom_read_status_register = 0x05,
	eeprom_write_status_register = 0x01,
	eeprom_read_data = 0x03,
	eeprom_page_program = 0x02,
	eeprom_block_erase = 0xd8,
	eeprom_sector_erase = 0x20,
	eeprom_chip_erase = 0xc7,
	eeprom_power_down = 0xb9,
	eeprom_release_power_down = 0xab
};

#define EEPROM_STATUS_WIP	(1 << 0)

void spi_write_eeprom(unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size);
unsigned int spi_read_eeprom (unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size);
void  spi_exchange(unsigned char * send_data_ptr, uint16_t count);

#endif
