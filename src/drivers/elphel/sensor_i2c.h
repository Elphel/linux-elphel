/*******************************************************************************
* FILE NAME  : sensor_i2c.h
* DESCRIPTION: Interface to FPGA-based i2c sequencer for sensor ports
* Copyright 2016 (C) Elphel, Inc.
* -----------------------------------------------------------------------------*
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/

/* Reserve i2c page (1 of 256 for a sensor port)*/
int i2c_page_alloc(int chn);

/* Free i2c page */
void i2c_page_free(int chn, int page);

/* Set i2c table entry to raw data (will just overwrite tbl_mode = 2) */
void set_sensor_i2c_raw(int chn,
		                int page,   // index in lookup table
					    u32 data); // Bit delay - number of mclk periods in 1/4 of the SCL period

/* Set i2c table entry for write operation */
void set_sensor_i2c_wr(int chn,
		              int page,        // index in lookup table
					  int sa,          // slave address (7 bit)
					  int rah,         // High byte of the i2c register address
					  int num_bytes,   //Number of bytes to write (1..10)
					  int bit_delay); // Bit delay - number of mclk periods in 1/4 of the SCL period

/* Set i2c table entry for read operation */
void set_sensor_i2c_rd(int chn,
		              int page,          // index in lookup table
					  int two_byte_addr, // Number of address bytes (0 - one byte, 1 - two bytes)
					  int num_bytes,     // Number of bytes to read (1..8, 0 means 8)
					  int bit_delay);// Bit delay - number of mclk periods in 1/4 of the SCL period
/*
// Write i2c command to the i2c command sequencer
// I2C command sequencer, block of 16 DWORD slots for absolute frame numbers (modulo 16) and 15 slots for relative ones
// 0 - ASAP, 1 next frame, 14 -14-th next.
// Data written depends on context:
// 1 - I2C register write: index page (MSB), 3 payload bytes. Payload bytes are used according to table and sent
//     after the slave address and optional high address byte. Other bytes are sent in descending order (LSB- last).
//     If less than 4 bytes are programmed in the table the high bytes (starting with the one from the table) are
//     skipped.
//     If more than 4 bytes are programmed in the table for the page (high byte), one or two next 32-bit words
//     bypass the index table and all 4 bytes are considered payload ones. If less than 4 extra bytes are to be
//     sent for such extra word, only the lower bytes are sent.
//
// 2 - I2C register read: index page, slave address (8-bit, with lower bit 0) and one or 2 address bytes (as programmed
//     in the table. Slave address is always in byte 2 (bits 23:16), byte1 (high register address) is skipped if
//     read address in the table is programmed to be a single-byte one
 */

/* Write one or multiple DWORDs to i2c relative (modulo16) address. Use offs = 0 for immediate (ASAP) command */
/* Length of data is determined by the page data already preset */
int write_sensor_i2c_rel (int chn,
                          int offs, // 4 bits
                          u32 * data);

/* Same to absolute (modulo16) address */
int write_sensor_i2c_abs (int chn,
			 		      int offs, // 4 bits
						  u32 * data);

/* Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_sensor_reg16  (int chn,
		                  int page, // page (8 bits)
                          int addr, // low 8 bits
	    				  u32 data); // 16 or 8-bit data (LSB aligned)

/* Initiate sensor i2c read in immediate mode (data itself has to be read from FIFO with read_sensor_i2c_fifo)*/
void  read_sensor_i2c  (int chn,
		               int page, // page (8 bits)
					   int sa7,  // 7-bit i2c slave address
                       int addr); // 8/16 bit address

/* Read next byte from the channel i2c FIFO. Return byte or -1 if no data available */
/* Sensor channel status should be in auto update mode (3) */
int read_sensor_i2c_fifo(int chn);
