/*
  i2c.c - I2C support for keypad and Trinamic plugins

  Part of grblHAL driver for MSP432P401R

  Copyright (c) 2018-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/


#include "i2c.h"

#if I2C_ENABLE

#include <string.h>

#include "serial.h"
#include "grbl/protocol.h"

#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define I2C_ADR_I2CBRIDGE 0x47
#endif

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
    I2CState_SendRegisterAddress,
    I2CState_AwaitCompletion,
    I2CState_ReceiveNext,
    I2CState_ReceiveNextToLast,
    I2CState_ReceiveLast,
} i2c_state_t;

typedef struct {
    volatile i2c_state_t state;
    uint8_t count;
    uint8_t *data;
    keycode_callback_ptr keycode_callback;
    uint8_t buffer[8];
} i2c_tr_trans_t;

static i2c_tr_trans_t i2c;

#define i2cIsBusy ((i2c.state != I2CState_Idle) || (I2C_PORT->CTLW0 & EUSCI_B_CTLW0_TXSTP))

// Power on self test, attempt to release bus if stuck.
bool i2c_selftest (void)
{
    // BIT4 = SDA, BIT5 = SCL

    if((P6->IN & (BIT4|BIT5)) != (BIT4|BIT5)) {

        uint32_t i = 10;

        P6->DIR |= BIT5;
        P6->OUT |= BIT5;

        do {
            P6->OUT &= BIT5;
            __delay_cycles(2400000);
            P6->OUT |= BIT5;
         } while(--i);

        // Start condition
        __delay_cycles(2400000);
        P6->DIR |= BIT4;
        P6->OUT &= ~BIT4;
        __delay_cycles(2400000);
        P6->OUT &= BIT5;
        // Stop condition
        __delay_cycles(2400000);
        P6->OUT |= BIT5;
        __delay_cycles(2400000);
        P6->OUT |= BIT5;
        __delay_cycles(2400000);

        P6->DIR &= ~(BIT4|BIT5);
    }

    return (P6->IN & (BIT4|BIT5)) == (BIT4|BIT5);
}

bool i2c_probe (i2c_address_t i2cAddr)
{
    return true;
}

bool i2c_send (i2c_address_t addr, uint8_t *buf, size_t size, bool block)
{
    while(i2cIsBusy);

    i2c.count = size;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = size == 1 ? I2CState_SendLast : (size == 2 ? I2CState_SendNext : I2CState_SendNext);
    I2C_PORT->I2CSA = addr;
    I2C_PORT->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags
    I2C_PORT->IE |= EUSCI_B_IE_TXIE0;
    I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);

    return true;
}

bool i2c_receive (i2c_address_t i2cAddr, uint8_t *buf, size_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.data  = buf ? buf : i2c.buffer;
    i2c.count = bytes;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    I2C_PORT->I2CSA = i2cAddr;
    I2C_PORT->CTLW0 &= ~EUSCI_B_CTLW0_TR;                       // Set read mode
    I2C_PORT->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);  // Clear interrupt flags
    I2C_PORT->IE |= (EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
    if(bytes == 1)
        I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP;
    else
        I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);

    return true;
}

#if (TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C) || ATC_ENABLE

static uint8_t *I2C_ReadRegister (i2c_address_t i2cAddr, uint8_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = i2c.buffer;
    i2c.state = I2CState_SendRegisterAddress;
    I2C_PORT->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags
    I2C_PORT->IE |= (EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
    I2C_PORT->I2CSA = i2cAddr;
    I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

#endif

/* could not get ACK polling to work...
static void WaitForACK (void)
{
    while(EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);

    do {
        EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;     // I2C TX, start condition

        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTT) {               // Ensure stop condition got sent
            if(!(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG))           // Break out if ACK received
              break;
        }
//        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
//        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);               // Ensure stop condition got sent
        __delay_cycles(5000);
    } while(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG);
//    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
//    while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);               // Ensure stop condition got sent
}
*/
bool i2c_transfer (i2c_transfer_t *i2c, bool read)
{
    bool single = i2c->count == 1;

    EUSCI_B1->I2CSA = i2c->address;                                     // Set EEPROM address and MSB part of data address
    EUSCI_B1->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_TXIE0);               // Diasable and
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // clear interrupts
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;            // Transmit start condition and address
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // Wait for TX completed
    EUSCI_B1->TXBUF = i2c->word_addr;                                   // Transmit data address LSB
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // Wait for TX completed

    if(read) {                                                          // Read data from EEPROM:
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // Transmit STOP condtition
        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                  // and wait for it to complete
        EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;                           // Set read mode
        if(single)                                                      // and issue
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP; // restart and stop condition if single byte read
        else                                                            // else
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;                     // restart condition only

        while(i2c->count) {                                             // Read data...
            if(!single && i2c->count == 1) {
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
                while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP) {
                    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
                }
            } else
                while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
            i2c->count--;
            *i2c->data++ = EUSCI_B1->RXBUF;
        }
    } else {                                                            // Write data to EEPROM:
        while (i2c->count--) {
            EUSCI_B1->TXBUF = *i2c->data++;
            while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
        }
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // I2C stop condition
 //       WaitForACK();
        hal.delay_ms(5, 0);                                             // Wait a bit for the write cycle to complete
    }
    while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                      // Ensure stop condition got sent

    return true;
}

bool i2c_get_keycode (uint_fast16_t i2cAddr, keycode_callback_ptr callback)
{
    while(i2cIsBusy);

    i2c.keycode_callback = callback;

    return i2c_receive(i2cAddr, NULL, 1, false);
}

#if TRINAMIC_ENABLE && TRINAMIC_I2C

#include "trinamic/tmc_i2c_interface.h"

static uint8_t axis = 0xFF;

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    uint8_t *res;
    TMC_spi_status_t status = 0;

    while(i2cIsBusy);

    if(driver.axis != axis) {
        i2c.buffer[0] = driver.axis | 0x80;
        i2c_send(I2C_ADR_I2CBRIDGE, NULL, 1, true);

        axis = driver.axis;
    }

    memset(i2c.buffer, 0, sizeof(i2c.buffer));
    i2c.buffer[0] = datagram->addr.idx;
    res = I2C_ReadRegister(I2C_ADR_I2CBRIDGE, 5, true);

    status = (uint8_t)*res++;
    datagram->payload.value = ((uint8_t)*res++ << 24);
    datagram->payload.value |= ((uint8_t)*res++ << 16);
    datagram->payload.value |= ((uint8_t)*res++ << 8);
    datagram->payload.value |= (uint8_t)*res++;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status = 0;

    while(i2cIsBusy);

    if(driver.axis != axis) {
        i2c.buffer[0] = driver.axis | 0x80;
        i2c_send(I2C_ADR_I2CBRIDGE, NULL, 1, true);

        while(i2cIsBusy);

        axis = driver.axis;
    }

    datagram->addr.write = 1;
    i2c.buffer[0] = datagram->addr.value;
    i2c.buffer[1] = (datagram->payload.value >> 24) & 0xFF;
    i2c.buffer[2] = (datagram->payload.value >> 16) & 0xFF;
    i2c.buffer[3] = (datagram->payload.value >> 8) & 0xFF;
    i2c.buffer[4] = datagram->payload.value & 0xFF;
    datagram->addr.write = 0;

    i2c_send(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    return status;
}

#endif // TRINAMIC_ENABLE

#define I2C_SCL_PIN 4
#define I2C_SDA_PIN 5

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(cap.started)
        return cap;

    memset(&i2c, 0, sizeof(i2c_tr_trans_t));

    P6->SEL0 |= (1<<I2C_SCL_PIN)|(1<<I2C_SDA_PIN);                                  // Assign I2C pins to USCI_B1

    if(!i2c_selftest()) {
        task_run_on_startup(report_warning, "I2C bus error!");
        system_raise_alarm(Alarm_SelftestFailed);
        return cap;
    }

    I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_SWRST;                                         // Put I2C_PORT in reset state
    I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_MST| EUSCI_B_CTLW0_SYNC;  // I2C master mode, SMCLK
    I2C_PORT->BRW = 240;                                                            // baudrate 100 KHZ (SMCLK = 48MHz)
    I2C_PORT->CTLW0 &=~ EUSCI_B_CTLW0_SWRST;                                        // clear reset register
//    I2C_PORT->IE |= EUSCI_B_IE_NACKIE;                                              // NACK interrupt enable

    NVIC_EnableIRQ(I2C_INT);       // Enable I2C interrupt and
    NVIC_SetPriority(I2C_INT, 4);  // set priority

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = PC,
        .pin = I2C_SCL_PIN + 8,
        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = PC,
        .pin = I2C_SDA_PIN + 8,
        .mode = { .mask = PINMODE_OD }
    };

    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);

    cap.started = cap.tx_non_blocking = On;

    return cap;
}

void I2C_IRQHandler (void)
{
    // based on code from https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/169882

    uint32_t ifg = I2C_PORT->IFG;

    I2C_PORT->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags

//    if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)

//    if(ifg & (EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0))
    switch(i2c.state) {

        case I2CState_Idle:
            I2C_PORT->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
            break;

        case I2CState_SendNext:
            I2C_PORT->TXBUF = *i2c.data++;
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            I2C_PORT->TXBUF = *i2c.data++;
            while(!(I2C_PORT->IFG & EUSCI_B_IFG_TXIFG0));   // Wait for start of TX
            I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_SendRegisterAddress:
            I2C_PORT->IE &= ~EUSCI_B_IE_TXIE0;
            I2C_PORT->TXBUF = *i2c.data;
            i2c.state = i2c.count == 1 ? I2CState_ReceiveLast : (i2c.count == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);
            while(!(I2C_PORT->IFG & EUSCI_B_IFG_TXIFG0));   // Wait for start of TX
            I2C_PORT->CTLW0 &= ~EUSCI_B_CTLW0_TR;                           // Set read mode
            if(i2c.state == I2CState_ReceiveLast)           // and issue
                I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP; // restart and stop condition if single byte read
            else                                                            // else
                I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTT;                     // restart condition only
            break;

        case I2CState_AwaitCompletion:
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            I2C_PORT->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
            break;

        case I2CState_ReceiveNext:
            *i2c.data++ = I2C_PORT->RXBUF;
            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = I2C_PORT->RXBUF;
            I2C_PORT->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            *i2c.data = I2C_PORT->RXBUF;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            I2C_PORT->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
            if(i2c.keycode_callback) {
                i2c.keycode_callback(i2c.data[0]);
                i2c.keycode_callback = NULL;
            }
            break;
    }
}

#endif
