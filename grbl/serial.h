/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2018-2019 Thomas Truong

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

#ifndef serial_h
#define serial_h


extern uint32_t serial_rx_buffer_head ;
extern uint32_t serial_rx_buffer_tail ;

extern uint32_t usb_serial_rx_buffer_head ;
extern uint32_t usb_serial_rx_buffer_tail ;

#ifdef STM32
	#define RX_BUFFER_SIZE 512
	#define TX_BUFFER_SIZE 254

	void process_it_char(uint8_t data);

#elif ATMEGA328P
	#ifndef RX_BUFFER_SIZE
		#define RX_BUFFER_SIZE 128
	#endif
	#ifndef TX_BUFFER_SIZE
		#ifdef USE_LINE_NUMBERS
			#define TX_BUFFER_SIZE 112
		#else
			#define TX_BUFFER_SIZE 104
		#endif
	#endif
#endif

#define SERIAL_NO_DATA 0xff


void serial_init();

// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);

void serial_write_all(uint8_t data);
// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read();

// Reset and empty data in read buffer. Used by e-stop and reset.
static inline void serial_reset_read_buffer()
{
#if USE_DOUBLE_SERIAL
  serial_rx_buffer_tail = serial_rx_buffer_head;
#endif
  usb_serial_rx_buffer_tail = usb_serial_rx_buffer_head;
}

// Returns the number of bytes available in the RX serial buffer.
uint32_t serial_get_rx_buffer_available();

// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint32_t serial_get_rx_buffer_count();

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint32_t serial_get_tx_buffer_count();

extern uint8_t steamSwitchAble;
void OnUsbDataRx(uint8_t* dataIn, uint8_t length);
#ifdef STM32
void HandleUartIT(uint8_t data ,uint8_t steam);
#endif

#endif
