/***************************************************************************//**
* @file      sensor_i2c.h
* @brief     Interface to FPGA-based i2c sequencer for sensor ports
* @copyright Copyright 2016 (C) Elphel, Inc.
* @par <b>License</b>
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/

/** I2C device description to be used with i2c sequencer*/
typedef struct{
	      char                         name[32];      ///< Device class name (up to 31 characters)
	      u8                           slave7;        ///< Device class base slave address (7-bit). Instances may have it
	                                                  ///< with offset added.
	      u8                           address_bytes; ///< Number of address bytes (1/2, used for read operations)
	      u8                           data_bytes;    ///< Number of data bytes (1..10), for writes it includes register address bytes
	      int                          scl_khz;       ///< maximal SCL frequency in KHz (currently limited by 200KHz slowest)
} x393_i2c_device_t;

/** Reserve i2c page (1 of 256) for a sensor port
 * @param chn sensor port number (0..3) */
int i2c_page_alloc(int chn);

/* Free i2c page */
void i2c_page_free(int chn, int page);

/* Set i2c table entry to raw data (will just overwrite tbl_mode = 2)*/
void set_xi2c_raw(int chn,
	             int page,   // index in lookup table
				 u32 data);  // Bit delay - number of mclk periods in 1/4 of the SCL period

/*
 *  Set i2c table entry for write operation using known devices
 * Get device with xi2c_dev_get(), copy and modify, if needed to
 * offset slave address or change number of bytes to write, SCL frequency
 */
void set_xi2c_wrc( x393_i2c_device_t * dc,   // device class
		          int                  chn,  // sensor port
		          int                  page, // index in lookup table
				  int                  rah); // High byte of the i2c register address

/*
 * Set i2c table entry for read operation using known devices
 * Get device with xi2c_dev_get(), copy and modify, if needed to
 * offset slave address or change number of bytes to write, SCL frequency
 */
void set_xi2c_rdc(x393_i2c_device_t * dc,    // device class
        		 int                 chn,    // sensor port
				 int                 page);  // index in lookup table

/* Set i2c table entry for read operation */
void set_xi2c_rd(int chn,
		         int page,          // index in lookup table
				 int two_byte_addr, // Number of address bytes (0 - one byte, 1 - two bytes)
				 int num_bytes,     // Number of bytes to read (1..8, 0 means 8)
				 int bit_delay);    // Bit delay - number of mclk periods in 1/4 of the SCL period

/* Write one or multiple DWORDs to i2c relative (modulo16) address. Use offs = 0 for immediate (ASAP) command */
/* Length of data is determined by the page data already preset */
int write_xi2c_rel (int chn,
                    int offs,    // 4 bits
                    u32 * data);

int write_xi2c_abs (int chn,
                    int offs, // 4 bits
                    u32 * data);
/* Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_xi2c_reg16  (int chn,
	 	                 int page,  // page (8 bits)
                         int addr,  // low 8 bits
	      				 u32 data); // 16 or 8-bit data (LSB aligned)

/* Initiate sensor i2c read in immediate mode (data itself has to be read from FIFO with read_xi2c_fifo)
 * Use slave address from provided class structure. */
void  read_xi2c (x393_i2c_device_t * dc,     // device class
		         int chn,
		         int page,  // page (8 bits)
                 int addr); // 8/16 bit address

/* Initiate sensor i2c read in immediate mode (data itself has to be read from FIFO with read_xi2c_fifo)*/
void  read_xi2c_sa7 (int chn,
		             int page,  // page (8 bits)
					 int sa7,   // 7-bit i2c slave address
                     int addr); // 8/16 bit address

/* Read next byte from the channel i2c FIFO. Return byte or -1 if no data available */
/* Sensor channel status should be in auto update mode (3) */
int read_xi2c_fifo(int chn);

/* Handling classes of i2c devices */
x393_i2c_device_t * xi2c_dev_get(const char * name);

/* Single-command i2c write/read register using pre-defined device classes */
int x393_xi2c_write_reg(const char * cname,    // device class name
						int          chn,      // sensor port number
				        int          sa7_offs, // slave address (7-bit) offset from the class defined slave address
				        int          reg_addr, // register address (width is defined by class)
				        int          data);    // data to write (width is defined by class)

int x393_xi2c_read_reg( const char * cname,    // device class name
						int          chn,      // sensor port number
				        int          sa7_offs, // slave address (7-bit) offset from the class defined slave address
				        int          reg_addr, // register address (width is defined by class)
				        int *        datap);   // pointer to a data receiver (read data width is defined by class)
