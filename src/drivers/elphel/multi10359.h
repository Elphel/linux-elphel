/***************************************************************************//**
* @file      multi10359.h
* @brief     Control of the 10359 multiplexer board
* @copyright Copyright 2010-2016 (C) Elphel, Inc.
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
/*
                // Add known devices: name, slave address (7-bit), number of address bytes, number of data bytes, SCL frequency (kHz)
                elphel393-sensor-i2c,i2c_devices = "mt9f002 0x10 2 2 500",
                                                   "mt9p006 0x48 1 2 500",
                                                   "el10359 0x08 1 2 500",
                                                   "pca9500_eeprom 0x50 1 1 100",
                                                   "cy22393 0x69 1 1 100";

 */

#define I2C359_CLK_NUMBER 4 ///< OK with NC393, clock is ANDed with 3

//multisensor.h
//#define I2C359_INC                    2   //< slave address increment between sensors in 10359A board (broadcast, 1,2,3) (7 bits SA) moved to sensor_common
#define I2C359_SLAVEADDR           0x08   ///< slave address of the 10359A board (move?) - 393 - use el10359 device class (7 bits SA)
#define I2C359_VERSION             0x00   ///< register address: 32-bit FPGA bitstream version
#define I2C359_MINVERSION             0x0359104b ///< Minimal FPGA version compatible with this software
#define I2C359_DCM_SYSTEM          0x01   ///< register address: System DCM control
#define I2C359_DCM_SDRAM           0x02   ///< register address: SDRAM DCM control
#define I2C359_DCM_SENSOR1         0x03   ///< register address: sensor 1 DCM control
#define I2C359_DCM_SENSOR2         0x04   ///< register address: sensor 2 DCM control
#define I2C359_DCM_SENSOR3         0x05   ///< register address: sensor 3 DCM control
  #define I2C359_DCM_INC             0x01 ///< DCM control bits: increment phase by fine step
  #define I2C359_DCM_DEC             0x02 ///< DCM control bits: decrement phase by fine step
  #define I2C359_DCM_RESET           0x03 ///< DCM control bits: reset fine phase
  #define I2C359_DCM_INC90           0x04 ///< DCM control bits: increment phase by 90 degrees
  #define I2C359_DCM_DEC90           0x08 ///< DCM control bits: decrement phase by 90 degrees
  #define I2C359_DCM_RESET90         0x0c ///< DCM control bits: reset 90 degrees phase
  #define I2C359_DCM_HACT_INC        0x10 ///< DCM control bits: increment HACT phase
  #define I2C359_DCM_HACT_DEC        0x20 ///< DCM control bits: decrement HACT phase
  #define I2C359_DCM_HACT_RESET      0x30 ///< DCM control bits: reset HACT phase
#define I2C359_CHNSEQ              0x06   ///< register address: channel select
  #define I2C359_SEQ(x,y,z) (((x)&3) | (((y)&3)<<2)  | (((z)&3)<<4)) ///< Channels control: x - direct channel (no memory),
                                                                     ///< y - second, z - third. 0- disabled, 1..3 channels (J2,J3,J4)
//#define I2C359_I2CMUX              0x07
//  #define I2C359_I2CMUX_2MEM          0x1
//  #define I2C359_I2CMUX_2SENSORS     0x0
#define I2C359_CLKSRC              0x08    ///< register address: clock source // does not read back
  #define I2C359_CLKSRC_SYSTEM        0x00 ///< clock source: system (from the system board over)
  #define I2C359_CLKSRC_LOCAL         0x01 ///< clock source: local (clock generator on the 10359 board
#define I2C359_MODE                0x09    ///< register address: mode register
#define I2C359_MODE_RESET             0x01 ///< mode register bit: 1 - reset, 0 - normal operation (persistent)
#define I2C359_MODE_DISABLE           0x02 ///< mode register bit:  1 - output disable reset, 0 - normal operation
#define I2C359_MODE_MUTI_EN           0x04 ///< mode register bit:  1 - enable multisensor mode, 0 - single sensor
#define I2C359_MODE_TEST_PATTERN      0x20 ///< mode register bit:  1 - enable test pattern from 10359, 0 - normal mode, sensor data
#define I2C359_MODE_INDIVIDUAL        0x10 ///< mode register bit:  0 - single frame sync for all channels, 1 - each sensor frame with individual frame sync
#define I2C359_MODE_PATTERN           0x20 ///< mode register bit:  1 - test pattern, 0 - sensor image
#define I2C359_HACT_WIDTH             0x0a ///< register address: HACT duration when internally generated (I2C359_HACT_MODE==1), default 2596
#define I2C359_INTERFRAME_DELAY       0x0c ///< register address: delay between frames output, in lines. 16 bits .
                                           ///< Needed in separate frames mode, large frames, non jp4
#define I2C359_HACT_DLY               0x0d ///< register address: bit 0 - delay HACT in frame 2 by 1 clock cycle,
                                           ///<  bit 1 - delay HACT in frame 3 by 1 c.c (to compensate for Bayer during hor. flips)
#define I2C359_HACT_MODE              0x0e ///< register address: 0 - use sensor HACT, 1 - use leading edge from the sensor,
                                           ///< generate duration. Bit 0 sensor 0, 1 - 1, 2 - 2
#define I2C359_WIDTH2                 0x13 ///< register address: line length  for memory channel 1 (2-nd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 2596
#define I2C359_HEIGHT2                0x14 ///< register address: frame height for memory channel 1 (2-nd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 1940
#define I2C359_HBLANK2                0x15 ///< register address: horizontal blank for memory channel 1 (2-nd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 256
#define I2C359_VBLANK2                0x16 ///< register address: vertical blank (before active frame) for memory channel 1 (2-nd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 0
#define I2C359_WIDTH3                 0x23 ///< register address: line length  for memory channel 3 (3-rd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 2596
#define I2C359_HEIGHT3                0x24 ///< register address: frame height for memory channel 3 (3-rd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 1940
#define I2C359_HBLANK3                0x25 ///< register address: horizontal blank for memory channel 3 (3-rd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 256
#define I2C359_VBLANK3                0x26 ///< register address: vertical blank (before active frame) for memory channel 3 (3-rd frame in sequence).
                                           ///< 16 bit (no I2C359_MSW), default 0

#define I2C359_SDRAM_CHEN             0x40 ///< register address: SDRAM channels enable, LSW of 32-bit, should be sent after I2C359_MSW, will be applied together
  #define I2C359_SDRAM_RUN(x)   (2 << (x <<1))  ///< SDRAM channels enable for channel x (0..7), enable=1, init=0
  #define I2C359_SDRAM_STOP(x)  (1 << (x <<1))  ///< SDRAM channels enable for channel x (0..7), enable=0, init=1
  #define I2C359_SDRAM_PAUSE(x) (3 << (x <<1))  ///< SDRAM channels enable for channel x (0..7), enable=0, init=0
#define I2C359_SDRAM_MANCMD           0x41 ///< Manual commands (mode set) for SDRAM, LSW of 32-bit, should be sent after I2C359_MSW, will be applied together
#define I2C359_SDRAM_MAGIC43          0x43 ///< register address: Undocumented

#define I2C359_SDRAM_START0           0x44 ///< register address: SDRAM channel 0 start page number (each page 4096 pixels)
#define I2C359_SDRAM_START1           0x45 ///< register address: SDRAM channel 1 start page number (each page 4096 pixels)
#define I2C359_SDRAM_START2           0x46 ///< register address: SDRAM channel 2 start page number (each page 4096 pixels)
#define I2C359_SDRAM_START3           0x47 ///< register address: SDRAM channel 3 start page number (each page 4096 pixels)

#define I2C359_SDRAM_CONF01A          0x4c ///< register address: SDRAM channels 0,1 configuration A (with I2C359_MSW)
#define I2C359_SDRAM_CONF23A          0x4d ///< register address: SDRAM channels 2,3 configuration A (with I2C359_MSW)
#define I2C359_SDRAM_CONF01B          0x4e ///< register address: SDRAM channels 0,1 configuration B (with I2C359_MSW)
#define I2C359_SDRAM_CONF23B          0x4f ///< register address: SDRAM channels 2,3 configuration B (with I2C359_MSW)

#define I2C359_MSW                    0x50 ///< register address: write 16 high bits of the next 32 bit word. Will be combined with other 16-bit writes
#define I2C359_SDRAM_PAGE_WR          0x63 ///< register address: write any data to start 64-word page write to buffer
#define I2C359_SDRAM_PAGE_RD          0x64 ///< register address: any data to start 64-word page read to buffer
#define I2C359_SDRAM_DATA             0x70 ///< register address: 16-bit data to be written to channel 4 or read from channel5

#define I2C359_DCM_STATUS             0x0F ///< register address: readonly, status of DCMs
#define I2C359_DCM_OFL(x,status)    ((status) & (1<<((((x) & 3 ) <<2) +0))) ///< detect overflow, x = 0 - SDRAM, 1..3 - sensors (status - data read from I2C359_DCM_STATUS)
#define I2C359_DCM_DONE(x,status)   ((status) & (1<<((((x) & 3 ) <<2) +1))) ///< detect DONE, x = 0 - SDRAM, 1..3 - sensors
#define I2C359_DCM_LOCKED(x,status) ((status) & (1<<((((x) & 3 ) <<2) +2))) ///< detect DCM locked, x = 0 - SDRAM, 1..3 - sensors
  #define I2C359_DCM_ERR_UNKNOWN  2 ///< Error code UNKNOWN
  #define I2C359_DCM_ERR_OVFL     3 ///< Error code OVERFLOW
  #define I2C359_DCM_ERR_NODONE   4 ///< Error code NOT DONE
  #define I2C359_DCM_ERR_NOLOCKED 5 ///< Error code NOT LOCKED

/*
CHEN_LOW,CHEN_HIGH - control 8 SDRAM channels + refresh (in CHEN_HIGH)
each dibit:
 0 - nop
 1 - disable
 2 - enable, init
 3 - disable, init
 MANCMD_HIGH,MANCMD_LOW - data to be directly applied to SDRAM for 1 clock cycle
*/



//#define I2C359_CLK_NUMBER         4 // system clock number for the 10359A on-board clock generator
void multi10359_set_device(struct device *dev);
int multisensor_pgm_detectsensor   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
