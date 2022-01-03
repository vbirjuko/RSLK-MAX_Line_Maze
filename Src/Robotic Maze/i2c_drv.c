#include "msp.h"
#include "Clock.h"
#include "i2c_drv.h"
#include "resources.h"
#include "Timer32.h"

#define I2C_DEV         EUSCI_B1
#define I2C_IRQn				EUSCIB1_IRQn
#define I2C_IRQHandler	EUSCIB1_IRQHandler

#define SDA_SCL_PORT    P6
#define SDA_PIN         (1u << 4)
#define SCL_PIN         (1u << 5)

typedef enum {
    send_reg_hi,
    send_reg_lo,
    send_data,

    rcv_reg_hi,
    rcv_reg_lo,
    rcv_restart,
    rcv_data,
    rcv_stop,

    i2c_stop,
    i2c_idle,
} i2c_state_t;

volatile i2c_state_t i2c_state;
volatile unsigned int data_count, i2c_register;
unsigned char * volatile data_ptr;
volatile t_i2c_status i2c_error_code;

void i2c_master_init(void) {
    // включаем внутренние подтяжки на всякий случай.
    // И тогда в некоторых случаях можно обойтись и без внешних резисторов.
    SDA_SCL_PORT->REN  |= (SDA_PIN | SCL_PIN);
    SDA_SCL_PORT->OUT  |= SCL_PIN;
    delay_us(10);
    SDA_SCL_PORT->OUT  |= SDA_PIN;

    // выполняем clock toggle для вывода ds3231 в состояние незанимающее шину.
    while ((SDA_SCL_PORT->IN & SDA_PIN) == 0) {
        SDA_SCL_PORT->OUT &= ~SCL_PIN;
        SDA_SCL_PORT->DIR |=  SCL_PIN;
        delay_us(10);
        SDA_SCL_PORT->DIR &= ~SCL_PIN;
        SDA_SCL_PORT->OUT |=  SCL_PIN;
        delay_us(10);
    }

    // подключаем выводы к EUSCI -
    // ИЗМЕНИТЬ В СЛУЧАЕ ВЫБОРА ДРУГОГО ПОРТА!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    SDA_SCL_PORT->SEL1 &= ~(SDA_PIN | SCL_PIN);
    SDA_SCL_PORT->SEL0 |=  (SDA_PIN | SCL_PIN);

    // Конфигурируем EUSCI
    I2C_DEV->CTLW0 |= UCSWRST;
    I2C_DEV->CTLW0  = UCMST | EUSCI_B_CTLW0_MODE_3 | EUSCI_B_CTLW0_SSEL__SMCLK | UCSYNC | UCSWRST;
    I2C_DEV->BRW    = 30; // 12000 kHz / 30 = 400 bps

    I2C_DEV->CTLW0 &= ~UCSWRST;
    NVIC_SetPriority(I2C_IRQn, I2C_Priority);
    NVIC_EnableIRQ(I2C_IRQn);
}

void I2C_IRQHandler(void) {

    switch (I2C_DEV->IV) {
        case 0x02: // Arbitration lost; Interrupt Flag: UCALIFG; Interrupt
            i2c_error_code = I2C_ERROR;
            i2c_state = i2c_idle;
            break;
        case 0x04: // Not acknowledgment; Interrupt Flag: UCNACKIFG
            i2c_error_code = I2C_ERROR;
            i2c_state = i2c_stop;
            I2C_DEV->CTLW0 |= UCTXSTP;
            break;
        case 0x06: // Start condition received; Interrupt Flag: UCSTTIFG
            break;
        case 0x08: // Stop condition received; Interrupt Flag: UCSTPIFG
            i2c_state = i2c_idle;
            break;
        case 0x0A: // Slave 3 Data received; Interrupt Flag: UCRXIFG3
        case 0x0C: // Slave 3 Transmit buffer empty; Interrupt Flag: UCTXIFG3
        case 0x0E: // Slave 2 Data received; Interrupt Flag: UCRXIFG2
        case 0x10: // Slave 2 Transmit buffer empty; Interrupt Flag: UCTXIFG2
        case 0x12: // Slave 1 Data received; Interrupt Flag: UCRXIFG1
        case 0x14: // Slave 1 Transmit buffer empty; Interrupt Flag: UCTXIFG1
            break;
        case 0x16: // Data received; Interrupt Flag: UCRXIFG0
            *data_ptr++ = I2C_DEV->RXBUF;
            if (--data_count == 1) {
                i2c_state = rcv_stop;
            }
            break;
        case 0x18: // Transmit buffer empty; Interrupt Flag: UCTXIFG0
            switch  (i2c_state) {
            case  rcv_reg_hi:
                i2c_state = rcv_reg_lo;
                I2C_DEV->TXBUF = (i2c_register >> 8) & 0xFFu;
                break;
            case  rcv_reg_lo:
                i2c_state =  rcv_restart;
                I2C_DEV->TXBUF = i2c_register & 0xFFu;
                break;
            case  rcv_restart:
                i2c_state = rcv_data;
                if (data_count == 1) i2c_state = rcv_stop;
                I2C_DEV->CTLW0 &= ~UCTR;
                I2C_DEV->CTLW0 |= UCTXSTT;
                break;
            case send_reg_hi:
                i2c_state = send_reg_lo;
                I2C_DEV->TXBUF = (i2c_register >> 8) & 0xFFu;
                break;
            case send_reg_lo:
                i2c_state = send_data;;
                I2C_DEV->TXBUF = i2c_register & 0xFFu;
                break;
            case send_data:
                if (data_count--) {
                    I2C_DEV->TXBUF = *data_ptr++;
                } else {
                    i2c_state = i2c_stop;
                    I2C_DEV->CTLW0 |= UCTXSTP;
                }
                break;
			default:
				break;
            }
            break;
        case 0x1A: // Byte counter zero; Interrupt Flag: UCBCNTIFG
        case 0x1C: // Clock low timeout; Interrupt Flag: UCCLTOIFG
        case 0x1E: // 9th bit position; Interrupt Flag: UCBIT9IFG
            break;
    }
}


t_i2c_status i2c_wr_reg(uint8_t address, uint8_t reg_addr, unsigned char * data, unsigned int length) {

    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
    i2c_register = reg_addr;
    i2c_state = send_reg_lo;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~UCSLA10;
    I2C_DEV->CTLW0 |= UCTR | UCTXSTT;
    I2C_DEV->IE    |= UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) continue;
    I2C_DEV->IE  &= ~(UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE);
    return i2c_error_code;
}

t_i2c_status i2c_wr_reg16(uint8_t address, uint16_t reg_addr, unsigned char * data, unsigned int length) {
    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
    i2c_register = reg_addr;
    i2c_state = send_reg_hi;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~UCSLA10;
    I2C_DEV->CTLW0 |= UCTR | UCTXSTT;
    I2C_DEV->IE    |= UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) continue;
    I2C_DEV->IE  &= ~(UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE);
    return i2c_error_code;
}

t_i2c_status i2c_wr(uint8_t address, unsigned char * data, unsigned int length) {

    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
//  i2c_register = reg_addr;
    i2c_state = send_data;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~UCSLA10;
    I2C_DEV->CTLW0 |= UCTR | UCTXSTT;
    I2C_DEV->IE    |= UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) continue;
    I2C_DEV->IE  &= ~(UCTXIE0 | UCNACKIE | UCSTPIE | UCALIE);
    return i2c_error_code;
}

t_i2c_status i2c_rd_reg(uint8_t address, uint8_t reg_addr, unsigned char * data, unsigned int length) {

    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
    i2c_register = reg_addr;
    i2c_state = rcv_reg_lo;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~UCSLA10;
    I2C_DEV->CTLW0 |= UCTXSTT | UCTR ;
    I2C_DEV->IE    |= UCTXIE0 | UCRXIE | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) {
        if (i2c_state == rcv_stop) {
            while (I2C_DEV->CTLW0 & UCTXSTT) continue;
            i2c_state = i2c_stop;
            I2C_DEV->CTLW0 |= UCTXSTP;
        }
    }
    I2C_DEV->IE  &= ~(UCTXIE0 | UCRXIE | UCNACKIE | UCSTPIE | UCALIE);
    if (data_count) i2c_error_code = I2C_ERROR;
    return i2c_error_code;
}

t_i2c_status i2c_rd_reg16(uint8_t address, uint16_t reg_addr, unsigned char * data, unsigned int length) {

    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
    i2c_register = reg_addr;
    i2c_state = rcv_reg_hi;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~UCSLA10;
    I2C_DEV->CTLW0 |= UCTXSTT | UCTR ;
    I2C_DEV->IE    |= UCTXIE0 | UCRXIE | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) {
        if (i2c_state == rcv_stop) {
            while (I2C_DEV->CTLW0 & UCTXSTT) continue;
            i2c_state = i2c_stop;
            I2C_DEV->CTLW0 |= UCTXSTP;
        }
    }
    I2C_DEV->IE  &= ~(UCTXIE0 | UCRXIE | UCNACKIE | UCSTPIE | UCALIE);
    if (data_count) i2c_error_code = I2C_ERROR;
    return i2c_error_code;
}

t_i2c_status i2c_rd(uint8_t address, unsigned char * data, unsigned int length) {

    if (I2C_DEV->STATW & UCBBUSY) {
        I2C_DEV->CTLW0 |= UCTXSTP;
        while (I2C_DEV->CTLW0 & UCTXSTP) {continue;}
    }
    if (I2C_DEV->STATW & (UCBBUSY | UCSCLLOW)) return I2C_BUSY;

    data_ptr = data;
    data_count = length;
//    i2c_register = reg_addr;
    if (length > 1) i2c_state = rcv_data;
    else            i2c_state = rcv_stop;
    i2c_error_code = I2C_SUCCESS;

    I2C_DEV->I2CSA = address >> 1;
    I2C_DEV->CTLW0 &= ~(UCSLA10 | UCTR);
    I2C_DEV->CTLW0 |= UCTXSTT;
    I2C_DEV->IE    |= UCRXIE | UCNACKIE | UCSTPIE | UCALIE;

    while (i2c_state != i2c_idle) {
        if (i2c_state == rcv_stop) {
            while (I2C_DEV->CTLW0 & UCTXSTT) continue;
            i2c_state = i2c_stop;
            I2C_DEV->CTLW0 |= UCTXSTP;
        }
    }
    I2C_DEV->IE  &= ~(UCTXIE0 | UCRXIE | UCNACKIE | UCSTPIE | UCALIE);
    if (data_count) i2c_error_code = I2C_ERROR;
    return i2c_error_code;
}
