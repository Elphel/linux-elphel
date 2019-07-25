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
#define P_LEPTON_DATAF8      0x28  ///< 0xf800 to 0xf8ff
#define P_LEPTON_DATAF9      0x29  ///< 0xf900 to 0xf9ff
#define P_LEPTON_DATAFA      0x2a  ///< 0xfa00 to 0xfaff
#define P_LEPTON_DATAFB      0x2b  ///< 0xfb00 to 0xfbff
#define P_LEPTON_DATAFC      0x2c  ///< 0xfc00 to 0xfcff
#define P_LEPTON_DATAFD      0x2d  ///< 0xfd00 to 0xfdff
#define P_LEPTON_DATAFE      0x2e  ///< 0xfe00 to 0xfeff
#define P_LEPTON_DATAFF      0x2f  ///< 0xff00 to 0xffff

// Registers 0x40.. 0xff will be used for non-i2c Lepton internal registers, such as VSYNC, telemetry
#define P_LEPTON_GP3VSYNC       0x40
#define P_LEPTON_TELEN          0x41
#define P_LEPTON_TELLOC         0x42
#define P_LEPTON_FFC_RUN        0x43


// actual values for registers (when reading)
#define P_REG_LEPTON_GP3VSYNC     0x0854
#define P_REG_LEPTON_GP3VSYNC_VAL 0x0005
#define P_REG_LEPTON_TELEN        0x0218
#define P_REG_LEPTON_TELLOC       0x021c
#define P_REG_LEPTON_FFC_RUN      0x0242 // "2" in 2 lower bits mean that it is just run, no parameters set

// Actual register address ranges (probably, only even are used too?)
#define P_REG_LEPTON_DATAF8  0xf800  ///< 0xf800 to 0xf8ff
#define P_REG_LEPTON_DATAF9  0xf900  ///< 0xf900 to 0xf9ff
#define P_REG_LEPTON_DATAFA  0xfa00  ///< 0xfa00 to 0xfaff
#define P_REG_LEPTON_DATAFB  0xfb00  ///< 0xfb00 to 0xfbff
#define P_REG_LEPTON_DATAFC  0xfc00  ///< 0xfc00 to 0xfcff
#define P_REG_LEPTON_DATAFD  0xfd00  ///< 0xfd00 to 0xfdff
#define P_REG_LEPTON_DATAFE  0xfe00  ///< 0xfe00 to 0xfeff
#define P_REG_LEPTON_DATAFF  0xff00  ///< 0xff00 to 0xffff


/**
 * Just set parameter for the Lepton related register, do not write to sensor
 * @param port  sensor port number
 * @param reg   relative register to fit 256 values page
 * @param data  value to set (16 bits)
 */
#define SET_LEPTON_PAR_DRY(port,reg,data) {\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
}


/**
 * Set parameter for the Lepton related register and send related commands to hardware i2c sequencer in immediate mode
 * Only immediate mode allows waiting
 * @param port  sensor port number
 * @param sa7   7-bit i2c slave address
 * @param reg   relative register to fit 256 values page
 * @param data  value to set (16 bits)
 * @param wait_ms - maximal numer of milliseconds to wait Lepton ready
 */
#define SET_LEPTON_PAR_IMMED(port,sa7,reg,data,wait_ms) {\
	int _LEPTONREG = pSensorPortConfig[(port)].par2addr[0][(reg)];\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    if (!(_LEPTONREG&0xffff0000)) {\
    	lepton_set_reg((port), (sa7), _LEPTONREG, (wait_ms), (data));\
    }\
}
// .pare2addr[0] - 256 of32-bit register addresses, or 0xffffffff for missing ones
/**
 * Set parameter for the sensor register and send to hardware i2c sequencer
 * @param port  sensor port number
 * @param frame frame number to apply, <0 - ASAP
 * @param reg   relative register to fit 256 values page
 * @param data  value to set (16 bits)
 */
#define SET_LEPTON_PAR_NOWAIT(port, frame, reg, data) {\
	int _LEPTONREG = pSensorPortConfig[(port)].par2addr[0][(reg)];\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    if (!(_LEPTONREG&0xffff0000)) {\
    	lepton_set_reg_nowait ((port), (frame), _LEPTONREG, (data));\
    }\
}


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

#define LEPTON_MODULE(x) (((x) >> 8) & 0x3f)


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

int  lepton_wait_ready     (int sensor_port, int sa7, int num_retries);
int  lepton_get_reg        (int sensor_port, int sa7, int cmd, int wait_ms);
int  lepton_set_reg        (int sensor_port, int sa7,   int cmd, int wait_ms, int data);
void lepton_set_reg_nowait (int sensor_port, int frame, int cmd, int data);
// Maybe not needed?
int  lepton_run_cmd        (int sensor_port, int sa7, int cmd, int wait_ms);
void lepton_run_cmd_nowait (int sensor_port, int sa7, int cmd);



#endif
