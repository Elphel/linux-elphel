/***************************************************************************//**
* @file      mt9x001.h
* @brief     Handles Micron/Aptina/On Semiconductor MT9M*, MT9D*,MT9T*, andMT9P*
* image sensors
* @copyright Copyright 2004-2016 (C) Elphel, Inc.
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

#ifndef _LEPTON_H
#define _LEPTON_H

#define LEPTON35_I2C_ADDR    0x2a ///< Lepton I2C slave address (7 bit)

/* i2c Lepton 3.5 registers will be defined here. Only even addresses are used by Lepton */
#define P_LEPTON_POWER       0x00  ///< Power On/Off register
#define P_LEPTON_STATUS      0x02  ///< Status register
#define P_LEPTON_COMMAND_ID  0x04  ///< Command ID register
#define P_LEPTON_DATA_LENGTH 0x06  ///< Data length register
#define P_LEPTON_DATA00      0x08  ///< Data0
#define P_LEPTON_DATA01      0x0a  ///< Data1
#define P_LEPTON_DATA02      0x0c  ///< Data2
#define P_LEPTON_DATA03      0x0e  ///< Data3
#define P_LEPTON_DATA04      0x10  ///< Data4
#define P_LEPTON_DATA05      0x12  ///< Data5
#define P_LEPTON_DATA06      0x14  ///< Data6
#define P_LEPTON_DATA07      0x16  ///< Data7
#define P_LEPTON_DATA08      0x18  ///< Data8
#define P_LEPTON_DATA09      0x1a  ///< Data9
#define P_LEPTON_DATA10      0x1c  ///< Data10
#define P_LEPTON_DATA11      0x1e  ///< Data11
#define P_LEPTON_DATA12      0x20  ///< Data12
#define P_LEPTON_DATA13      0x22  ///< Data13
#define P_LEPTON_DATA14      0x24  ///< Data14
#define P_LEPTON_DATA15      0x26  ///< Data15

// Data model for Lepton does not match 393 convention (1 page for "important" registers,
// so just 1 register of each of the 256-words is registered
#define P_LEPTON_DATAF8  0xf8  ///< 0xf800 to 0xf8ff
#define P_LEPTON_DATAF9  0xf9  ///< 0xf900 to 0xf9ff
#define P_LEPTON_DATAFA  0xfa  ///< 0xfa00 to 0xfaff
#define P_LEPTON_DATAFB  0xfb  ///< 0xfb00 to 0xfbff
#define P_LEPTON_DATAFC  0xfc  ///< 0xfc00 to 0xfcff
#define P_LEPTON_DATAFD  0xfd  ///< 0xfd00 to 0xfdff
#define P_LEPTON_DATAFE  0xfe  ///< 0xfe00 to 0xfeff
#define P_LEPTON_DATAFF  0xff  ///< 0xff00 to 0xffff

// Actual register address ranges (probably, only even are used too?)
#define P_REG_LEPTON_DATAF8  0xf800  ///< 0xf800 to 0xf8ff
#define P_REG_LEPTON_DATAF9  0xf900  ///< 0xf900 to 0xf9ff
#define P_REG_LEPTON_DATAFA  0xfa00  ///< 0xfa00 to 0xfaff
#define P_REG_LEPTON_DATAFB  0xfb00  ///< 0xfb00 to 0xfbff
#define P_REG_LEPTON_DATAFC  0xfc00  ///< 0xfc00 to 0xfcff
#define P_REG_LEPTON_DATAFD  0xfd00  ///< 0xfd00 to 0xfdff
#define P_REG_LEPTON_DATAFE  0xfe00  ///< 0xfe00 to 0xfeff
#define P_REG_LEPTON_DATAFF  0xff00  ///< 0xff00 to 0xffff

typedef union {
    struct {
          u32            busy: 1; // [    0] BUSY bit (should be zero before issuing commands)
          u32       boot_mode: 1; // [    1] Boot Mode (should be 1 - booted from internal ROM)
          u32     boot_status: 1; // [    2] 1 - booted, 0 - in progress
          u32            rsv5: 5; // [ 7: 3] Reserved, should read zero
          u32           error: 8; // [15  8] Error code
    };
    struct {
          u32             d32:16; // [15: 0] (0) cast to u32
    };
} lepton_status_t;

typedef union {
    struct {
          u32            type: 2; // [ 1: 0] 0 - get, 1 - set, 2 - run, 3 - invalid
          u32              id: 6; // [ 7: 2] command ID
          u32          module: 4; // [11: 8] module: 1-AGC, 2-SYS, 3-VID, 8-OEM, 0xe - RAD
          u32                : 2; // [13:12] Reserved
          u32      protection: 1; // [   14] OEM bit, set for modules: RAD and OEM
          u32                : 1; // [   15] Reserved
    };
    struct {
          u32             d32:16; // [15: 0] (0) cast to u32
    };
} lepton_command_t;

typedef enum LEPTON_MODULES {
	LEPTON_AGC = 1,
	LEPTON_SYS = 2,
	LEPTON_VID = 3,
	LEPTON_OEM = 8,
	LEPTON_RAD = 14
} lepton_modules_t;

typedef enum LEPTON_TYPES {
	LEPTON_GET = 0,
	LEPTON_SET = 1,
	LEPTON_RUN = 2
} lepton_types_t;

// large segments - 0xf800-0xfbff, 0xfc00 -0xffff not yet supported

/**
LUT to map SENSOR_REGSxxx to internal sensor register addresses
  * needed for any sensor
  * For better manual mapping:
      - even elements are SENSOR_REGSxxx,
      - odd elements are sensor's register addresses.
  * has to be at least 16-bit/entry for 16 bit addresses
  * (for MT9X001 it's a 1-to-1 mapping)
*/
extern const unsigned short lepton_par2addr[];
extern const unsigned short lepton_pages[];
extern const unsigned short lepton_ahead_tab[];

int lepton_pgm_detectsensor    (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
;
void lepton_set_device(struct device *dev);

int lepton_wait_ready    (int sensor_port,       ///< sensor port number (0..3)
                          int sa7,               ///< I2C slave address
                          int    num_retries );  ///< number of retries, 0 - forever
                                             ///< @return > 0 number of retries0 - OK, negative - error

void lepton_set_reg_nowait(int               sensor_port,    ///< sensor port number (0..3)
                           int               frame,          ///< frame number to apply, <0 - ASAP
						   lepton_modules_t  cmd_module,     ///< Lepton command module
                           int               cmd_id,         ///< Lepton command id
                           int               data         ); ///< data to write



#endif
