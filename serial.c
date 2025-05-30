/*

  serial.c - MSP432 low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

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

#ifndef __SERIAL0_h__
#define __SERIAL0_h__

#include "serial.h"

#include "grbl/protocol.h"

#define TXBUSY(uart) ((uart->IE & EUSCI_A_IE_TXIE) || (uart->STATW & EUSCI_A_STATW_BUSY))

#define eusci(p) eusciM(p)
#define eusciM(p) EUSCI_ ## p
#define eusciINT(p) eusciI(p)
#define eusciI(p) EUSCI ## p ## _IRQn
#define eusciHANDLER(p) eusciH(p)
#define eusciH(p) EUSCI ## p ## _IRQHandler

// Define serial port pins and modules

#define SERIAL0_MOD         A0
#define SERIAL0_MODULE      eusci(SERIAL0_MOD)
#define SERIAL0_MODULE_INT  eusciINT(SERIAL0_MOD)
#define SERIAL0_IRQHandler  eusciHANDLER(SERIAL0_MOD)
#define SERIAL0_PORT        P1
#define SERIAL0_RX_PIN      2
#define SERIAL0_TX_PIN      3

static stream_tx_buffer_t txbuffer = {};
static stream_rx_buffer_t rxbuffer = {};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static const io_stream_t *serialInit (uint32_t baud_rate);

#ifdef SERIAL1_PORT

#define SERIAL2_MOD         A2
#define SERIAL2_MODULE      eusci(SERIAL2_MOD)
#define SERIAL2_MODULE_INT  eusciINT(SERIAL2_MOD)
#define SERIAL2_IRQHandler  eusciHANDLER(SERIAL2_MOD)
#define SERIAL2_PORT        P3
#define SERIAL2_RX_PIN      2
#define SERIAL2_TX_PIN      3

static stream_tx_buffer_t txbuffer2 = {};
static stream_rx_buffer_t rxbuffer2 = {};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;

static const io_stream_t *serial2Init (uint32_t baud_rate);

#endif

#ifdef RTS_PORT
static volatile uint8_t rts_state = 0;
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = Off,
      .claim = serialInit
    },
#ifdef SERIAL2_MOD
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

static int16_t serialGetC (void);

/*
//
// Returns number of characters in serial output buffer
//
static uint16_t serialTxCount (void)
{
    uint16_t tail = txbuffer.tail;

    return BUFCOUNT(txbuffer.head, tail, TX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
    uint16_t tail = rxbuffer.tail, head = rxbuffer.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}
*/

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    unsigned int tail = rxbuffer.tail, head = rxbuffer.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuffer.head = rxbuffer.tail = 0;
    hal.stream.read = serialGetC; // restore normal input

#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serialPutCNonBlocking (const char c)
{
    bool ok;

    if((ok = !TXBUSY(SERIAL0_MODULE)))
        SERIAL0_MODULE->TXBUF = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c) {

    uint32_t next_head;

    if(txbuffer.head != txbuffer.tail || !serialPutCNonBlocking(c)) {   // Try to send character without buffering...

        next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);         // .. if not, set and update head pointer

        while(txbuffer.tail == next_head) {                             // While TX buffer full
            SERIAL0_MODULE->IE |= EUSCI_A_IE_TXIE;                       // Enable TX interrupts???
            if(!hal.stream_blocking_callback())                         // check if blocking for space,
                return false;                                           // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuffer.data[txbuffer.head] = c;                               // Add data to buffer
        txbuffer.head = next_head;                                      // and update head pointer

        SERIAL0_MODULE->IE |= EUSCI_A_IE_TXIE;                           // Enable TX interrupts
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

/*
//
// Writes a null terminated string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWriteLn (const char *s)
{
    serialWriteS(s);
    serialWriteS(ASCII_EOL);
}

//
// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
static void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}
*/

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    __disable_interrupts();

    uint32_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        __enable_interrupts();
        return -1; // no data available else EOF
    }

    char data = rxbuffer.data[bptr++];              // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer
    __enable_interrupts();

#ifdef RTS_PORT
    if (rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) // Clear RTS if below LWM
        BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 0;
#endif

    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static bool serialDisable (bool disable)
{
    SERIAL0_MODULE->IE = disable ? 0 : EUSCI_A_IE_RXIE;

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed || baud_rate != 115200)
        return NULL;

    serial[0].flags.claimed = On;

    SERIAL0_MODULE->CTLW0 = EUSCI_A_CTLW0_SWRST|EUSCI_A_CTLW0_SSEL__SMCLK;
    SERIAL0_MODULE->BRW = 6;
    SERIAL0_MODULE->MCTLW = (0x20 << 8) | (8 << 4) | 1;
    SERIAL0_MODULE->IFG = ~EUSCI_A_IFG_RXIFG;
    SERIAL0_MODULE->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
    SERIAL0_MODULE->IE = EUSCI_A_IE_RXIE;

    NVIC_SetPriority(SERIAL0_MODULE_INT, 3);
    NVIC_EnableIRQ(SERIAL0_MODULE_INT);

    SERIAL0_PORT->SEL0 = (1<<SERIAL0_RX_PIN)|(1<<SERIAL0_TX_PIN);    // set 2-UART pins as second function

    __enable_interrupts();

#ifdef RTS_PORT
    RTS_PORT->DIR |= RTS_BIT;
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port = SERIAL0_PORT,
        .pin = SERIAL0_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = SERIAL0_PORT,
        .pin = SERIAL0_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

//
void SERIAL0_IRQHandler (void)
{
    uint32_t bptr;

    switch(SERIAL0_MODULE->IV) {

        case 0x04:
            bptr = txbuffer.tail;                           // Temp tail position (to avoid volatile overhead)
            SERIAL0_MODULE->TXBUF = txbuffer.data[bptr++];   // Send a byte from the buffer
            bptr &= (TX_BUFFER_SIZE - 1);                   // and update
            txbuffer.tail = bptr;                           // tail position
            if (bptr == txbuffer.head)                      // Turn off TX interrupt
                SERIAL0_MODULE->IE &= ~EUSCI_A_IE_TXIE;      // when buffer empty
            break;

        case 0x02:;
            uint16_t data = SERIAL0_MODULE->RXBUF;                   // Read character received
            if(!enqueue_realtime_command((char)data)) {             // Enqueued as real-time command?
                bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer
                if(bptr == rxbuffer.tail)                           // If buffer full
                    rxbuffer.overflow = 1;                          // flag overflow,
                else {
                    rxbuffer.data[rxbuffer.head] = (char)data;      // else add data to buffer
                    rxbuffer.head = bptr;                           // and update pointer
                }
            }
        #ifdef RTS_PORT
            if (!rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) // Set RTS if at or above HWM
                BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 1;
        #endif
            break;
    }
}

#ifdef SERIAL2_MOD

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial2RxFree (void)
{
    uint32_t tail = rxbuffer2.tail, head = rxbuffer2.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial2RxCount (void)
{
    uint32_t tail = rxbuffer2.tail, head = rxbuffer2.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial2RxFlush (void)
{
    rxbuffer2.tail = rxbuffer2.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial2RxCancel (void)
{
    rxbuffer2.data[rxbuffer2.head] = ASCII_CAN;
    rxbuffer2.tail = rxbuffer2.head;
    rxbuffer2.head = (rxbuffer2.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serial2PutCNonBlocking (const char c)
{
    bool ok;

    if((ok = !TXBUSY(SERIAL2_MODULE)))
        SERIAL2_MODULE->TXBUF = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
static bool serial2PutC (const char c) {

    uint32_t next_head;

    if(txbuffer2.head != txbuffer2.tail || !serial2PutCNonBlocking(c)) {    // Try to send character without buffering...

        next_head = (txbuffer2.head + 1) & (TX_BUFFER_SIZE - 1);            // .. if not, set and update head pointer

        while(txbuffer2.tail == next_head) {                                // While TX buffer full
            SERIAL0_MODULE->IE |= EUSCI_A_IE_TXIE;                           // Enable TX interrupts???
        }

        txbuffer2.data[txbuffer2.head] = c;                                 // Add data to buffer
        txbuffer2.head = next_head;                                         // and update head pointer

        SERIAL2_MODULE->IE |= EUSCI_A_IE_TXIE;                              // Enable TX interrupts
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial2WriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

//
// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
static void serial2Write(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial2TxFlush (void)
{
    SERIAL2_MODULE->IE &= ~EUSCI_A_IE_TXIE;
    txbuffer2.tail = txbuffer2.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial2TxCount (void)
{
    uint32_t tail = txbuffer2.tail, head = txbuffer2.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (TXBUSY(SERIAL2_MODULE) ? 1 : 0);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial2GetC (void)
{
    uint16_t bptr = rxbuffer2.tail;

    if(bptr == rxbuffer2.head)
        return -1; // no data available else EOF

    char data = rxbuffer2.data[bptr++];             // Get next character, increment tmp pointer
    rxbuffer2.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serial2Init(baud_rate);
        init_ok = true;
    }

    switch(baud_rate)
    {
        case 19200:
            SERIAL2_MODULE->BRW = 39;
            SERIAL2_MODULE->MCTLW = (0x0 << 8) | (1 << 4) | 1;
            break;

        default: // 115200
            SERIAL2_MODULE->BRW = 6;
            SERIAL2_MODULE->MCTLW = (0x20 << 8) | (8 << 4) | 1;
            break;
    }

    return true;
}

static bool serial2Disable (bool disable)
{
    SERIAL2_MODULE->IE = disable ? 0 : EUSCI_A_IE_RXIE;

    return true;
}

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

static const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .is_connected = stream_connected,
        .read = serial2GetC,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .write_all = serial2WriteS,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
    //    .suspend_read = serial2SuspendInput,
        .set_baud_rate = serial2SetBaudRate,
        .disable_rx = serial2Disable,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    SERIAL2_MODULE->CTLW0 = EUSCI_A_CTLW0_SWRST|EUSCI_A_CTLW0_SSEL__SMCLK;

    switch(baud_rate)
    {
        case 19200:
            SERIAL2_MODULE->BRW = 39;
            SERIAL2_MODULE->MCTLW = (0x0 << 8) | (1 << 4) | 1;
            break;

        default: // 115200
            SERIAL2_MODULE->BRW = 6;
            SERIAL2_MODULE->MCTLW = (0x20 << 8) | (8 << 4) | 1;
            break;
    }

    SERIAL2_MODULE->IFG = ~EUSCI_A_IFG_RXIFG;
    SERIAL2_MODULE->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    NVIC_SetPriority(SERIAL2_MODULE_INT, 3);
    NVIC_EnableIRQ(SERIAL2_MODULE_INT);

    SERIAL2_PORT->SEL0 = (1<<SERIAL2_RX_PIN)|(1<<SERIAL2_TX_PIN);    // set 2-UART pins as second function

#if MODBUS_ENABLE
    SERIAL2_MODULE->IE = EUSCI_A_IE_RXIE;
#endif

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .port = SERIAL2_PORT,
        .pin = SERIAL2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .port = SERIAL2_PORT,
        .pin = SERIAL2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

void SERIAL2_IRQHandler (void)
{
    uint32_t data, bptr;

    switch(SERIAL2_MODULE->IV) {

        case 0x04:
            bptr = txbuffer2.tail;                          // Temp tail position (to avoid volatile overhead)
            SERIAL2_MODULE->TXBUF = txbuffer2.data[bptr++]; // Send a byte from the buffer
            bptr &= (TX_BUFFER_SIZE - 1);                   // and update
            txbuffer2.tail = bptr;                          // tail position
            if (bptr == txbuffer2.head)                     // Turn off TX interrupt
                SERIAL2_MODULE->IE &= ~EUSCI_A_IE_TXIE;     // when buffer empty
            break;

        case 0x02:
            data = SERIAL2_MODULE->RXBUF;                           // Read character received
            if(!enqueue_realtime_command2((char)data)) {            // Enqueued as real-time command?
                bptr = (rxbuffer2.head + 1) & (RX_BUFFER_SIZE - 1); // Temp head position (to avoid volatile overhead)
                if(bptr == rxbuffer2.tail)                          // If buffer full
                    rxbuffer2.overflow = 1;                         // flag overflow
                else {
                    rxbuffer2.data[rxbuffer2.head] = data;          // else data to buffer
                    rxbuffer2.head = bptr;                          // and update pointer
                }
            }
            break;
    }
}
#endif

#endif // __SERIAL_h__
