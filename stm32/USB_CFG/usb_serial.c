/*

  usb_serial.c - USB serial port implementation for STM32F103C8 ARM processors

  Part of GrblHAL

  Copyright (c) 2019-2020 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/
#if 0 //USB_SERIAL_UART && USB_SERIAL_CDC

#define USBCDC 1  //USB CDC VCP
#define HWUART 2  //HW UART

uint8_t last_steam = 0;

//
// Writes a null terminated string to active output stream, blocks if buffer full
//
int16_t multiSteamGetC (void)
{
	int16_t c = -1;

	if(isUsbCDCConnected())
	{
		if(hal.stream.switchable ? true : last_steam == USBCDC)
		{
			c = usbGetC();
			last_steam = USBCDC;
			hal.stream.switchable = (c == -1);
		}
	}
	else
	{
		if(last_steam == USBCDC) //在CDC已经断开连接的情况下,应该将steam指向转到 硬件串口 HWUART
		{
			hal.stream.switchable = true;
			c = '\n'; //强行补换行符防止命令被截断,或者污染后续的命令字符串
			return c;
		}
	}

	if(c == -1 )
	{
		if(hal.stream.switchable ? true : last_steam == HWUART)
		{
			c = serialGetC();
			last_steam = HWUART;
			hal.stream.switchable = (c == -1);
		}
	}
	return c;
}

//
// Writes a null terminated string to active stream, blocks if buffer full
//
void multiSteamWriteS (const char *s)
{
	if(last_steam == USBCDC)
	{
		if(isUsbCDCConnected())
		    usbWriteS(s); //仅在VCP连接的情况下发送字符串,否则会造成发送缓冲溢出阻塞
	}
	else if(last_steam == HWUART)
		serialWriteS(s);
}

//
// Writes a null terminated string to all output stream, blocks if buffer full
//
void multiSteamWriteSAll (const char *s)
{
	if(isUsbCDCConnected())
		usbWriteS(s); //仅在VCP连接的情况下发送字符串,否则会造成发送缓冲溢出阻塞
	serialWriteS(s);
}

//
// Returns number of free characters in active input buffer
//
uint16_t multiSteamRxFree (void)
{
	//BUG:可能存在steam切换时的干扰
	uint16_t uf = usbRxFree();
	uint16_t sf = serialRxFree();
	return (uf > sf) ? sf : uf;
}

//
// Flushes the serial input buffer
//
void multiSteamRxFlush (void)
{
	usbRxFlush();
	serialRxFlush();
}

//
// Flushes and adds a CAN character to active input buffer
//
void multiSteamRxCancel (void)
{
	usbRxCancel();
	serialRxCancel();
}

//
// Suspend/resume the active input buffer
//
bool multiSteamSuspendInput (bool suspend)
{
	return usbSuspendInput(suspend) && serialSuspendInput(suspend);
}

#endif

#if 0

#include "driver.h"
#include "serial.h"
#include "../grbl/grbl.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static char txdata2[BLOCK_TX_BUFFER_SIZE]; // Secondary TX buffer (for double buffering)
static bool use_tx2data = false;
static stream_rx_buffer_t rxbuf = {0}, rxbackup;
static stream_block_tx_buffer_t txbuf = {0};

void usbInit (void)
{
    MX_USB_DEVICE_Init();

    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;
}

//
// Returns number of free characters in the input buffer
//
uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
void usbRxFlush (void)
{
    rxbuf.head = rxbuf.tail = 0;
}

//
// Flushes and adds a CAN character to the input buffer
//
void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool usb_write (void)
{
    static uint8_t dummy = 0;

    txbuf.s = use_tx2data ? txdata2 : txbuf.data;

    while(CDC_Transmit_FS((uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    if(txbuf.length % 64 == 0) {
        while(CDC_Transmit_FS(&dummy, 0) == USBD_BUSY) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }

    use_tx2data = !use_tx2data;
    txbuf.s = use_tx2data ? txdata2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//

void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if((length + txbuf.length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    memcpy(txbuf.s, s, length);
    txbuf.length += length;
    txbuf.s += length;

    if(s[length - 1] == ASCII_LF) {
        if(!usb_write())
            return;
    }
}

//
// usbGetC - returns -1 if no data available
//
int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];             // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

// "dummy" version of serialGetC
static int16_t usbGetNull (void)
{
    return -1;
}

bool usbSuspendInput (bool suspend)
{
    if(suspend)
        hal.stream.read = usbGetNull;
    else if(rxbuf.backup)
        memcpy(&rxbuf, &rxbackup, sizeof(stream_rx_buffer_t));

    return rxbuf.tail != rxbuf.head;
}

void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {

        uint_fast16_t next_head = (rxbuf.head + 1)  & (RX_BUFFER_SIZE - 1); // Get and increment buffer pointer

        if(rxbuf.tail == next_head) {                                       // If buffer full
            rxbuf.overflow = 1;                                             // flag overflow
        } else {
            if(*data == CMD_TOOL_ACK && !rxbuf.backup) {

                memcpy(&rxbackup, &rxbuf, sizeof(stream_rx_buffer_t));
                rxbuf.backup = true;
                rxbuf.tail = rxbuf.head;
                hal.stream.read = usbGetC; // restore normal input

            } else if(!hal.stream.enqueue_realtime_command(*data)) {        // Check and strip realtime commands,
                rxbuf.data[rxbuf.head] = *data;                             // if not add data to buffer
                rxbuf.head = next_head;                                     // and update pointer
            }
        }
        data++;                                                             // next
    }
}

#endif
