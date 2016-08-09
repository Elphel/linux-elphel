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

#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H
#define I2C_CMD_STOP  0
#define I2C_CMD_RUN   1
#define I2C_CMD_RESET 2
#define SDA_DRIVE_HIGH 1
#define SDA_RELEASE    1



/** I2C device description to be used with i2c sequencer*/
typedef struct{
	      char                         name[32];      ///< Device class name (up to 31 characters)
	      u8                           slave7;        ///< Device class base slave address (7-bit). Instances may have it
	                                                  ///< with offset added.
	      u8                           address_bytes; ///< Number of address bytes (1/2, used for read operations)
	      u8                           data_bytes;    ///< Number of data bytes (1..10), for writes it includes register address bytes
	      int                          scl_khz;       ///< maximal SCL frequency in KHz (currently limited by 200KHz slowest)
} x393_i2c_device_t;

int i2c_stop_run_reset(int chn, int cmd);
int i2c_drive_mode    (int chn, int sda_drive_high, int sda_release);

int i2c_page_alloc   (int chn);
int i2c_page_register(int chn,  int page);
void i2c_page_free   (int chn, int page);
x393_i2c_device_t * xi2c_dev_get(const char * name);
void set_xi2c_raw    (int chn, int page, u32 data);
void set_xi2c_wr     (int chn,int page, int sa7, int rah, int num_bytes, int bit_delay);
void set_xi2c_wrc    (x393_i2c_device_t * dc, int chn, int page, int rah);
void set_xi2c_rdc    (x393_i2c_device_t * dc, int chn, int page);
void set_xi2c_rd     (int chn, int page, int two_byte_addr, int num_bytes, int bit_delay);
int write_xi2c_rel   (int chn, int offs, u32 * data);
int write_xi2c_abs   (int chn, int offs, u32 * data);
void  write_xi2c_reg16  (int chn, int page, int addr, u32 data);
void  write_xi2c_reg16_rel (int chn, int page, int frame, int addr, u32 data);
void  write_xi2c_reg16_abs (int chn, int page, int frame, int addr, u32 data);
void  write_xi2c_reg16_abs_asap (int chn, int page, int frame, int addr, u32 data);
void  read_xi2c      (x393_i2c_device_t * dc, int chn, int page, int addr);
void  read_xi2c_sa7  (int chn, int page, int sa7, int addr);
int   read_xi2c_fifo (int chn);
x393_i2c_device_t * xi2c_dev_get(const char * name);
int x393_xi2c_write_reg(const char * cname, int chn, int sa7_offs, int reg_addr, int   data);
int x393_xi2c_read_reg (const char * cname, int chn, int sa7_offs, int reg_addr, int * datap);
int legacy_read_i2c_reg(int chn, int page,           int sa7,      int reg_addr, int len, int * datap);
#endif
