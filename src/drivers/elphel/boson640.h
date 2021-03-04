/***************************************************************************//**
* @file      boson640.h
* @brief     sopport for boson640 sesnor (currently just a copy of boson640)
* @copyright Copyright 2021 (C) Elphel, Inc.
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

#ifndef _BOSON640_H
#define _BOSON640_H

#define BOSON640_I2C_ADDR      0x81 ///< Boson UART ext mode = 1 instead of the I2C slave address (7 bit)
#define BOSON640_BOOT_FRAME    75   ///< do not communicate with the sensor if frame number is less than this
// encoding number of data bytes in 2 msb of the 16-bit register address, will be encoded in sa field

#define BOSON640_0BYTES        0x000  ///< add to rah for zero data length
#define BOSON640_1BYTES        0x4000 ///< add to rah for 1-byte data
#define BOSON640_2BYTES        0x8000 ///< add to rah for 2-byte data
#define BOSON640_4BYTES        0xc000 ///< add to rah for 4-byte data

// Boson modules
#define BOSON640_GAO           0x0000
#define BOSON640_ROIC          0x0200
#define BOSON640_BPR           0x0300
#define BOSON640_TELEMETRY     0x0400
#define BOSON640_BOSON         0x0500
#define BOSON640_DVO           0x0600
#define BOSON640_SCNR          0x0800
#define BOSON640_TNR           0x0a00
#define BOSON640_SPNR          0x0c00
#define BOSON640_SYSCTL        0x0e00
#define BOSON640_TESTRAMP      0x1000
#define BOSON640_SRNR          0x2800

// Boson functions: MSB - number of bytes in 2 msb, module (6 bits), function - LSB (stars - how needed is it to implement)
#define P_BOSON640_REG_gaoSetGainState                  (BOSON640_4BYTES + BOSON640_GAO + 0x01) // **
#define P_BOSON640_REG_gaoSetFfcState                   (BOSON640_4BYTES + BOSON640_GAO + 0x03) // **
#define P_BOSON640_REG_gaoSetTempCorrectionState        (BOSON640_4BYTES + BOSON640_GAO + 0x05) // *
#define P_BOSON640_REG_gaoSetIConstL                    (BOSON640_2BYTES + BOSON640_GAO + 0x07)
#define P_BOSON640_REG_gaoSetIConstM                    (BOSON640_2BYTES + BOSON640_GAO + 0x09)
#define P_BOSON640_REG_gaoSetAveragerState              (BOSON640_4BYTES + BOSON640_GAO + 0x0b) // *
#define P_BOSON640_REG_gaoSetNumFFCFrames               (BOSON640_2BYTES + BOSON640_GAO + 0x0d) // **
#define P_BOSON640_REG_gaoSetRnsState                   (BOSON640_4BYTES + BOSON640_GAO + 0x11) // *
#define P_BOSON640_REG_gaoSetTestRampState              (BOSON640_4BYTES + BOSON640_GAO + 0x13) // **
#define P_BOSON640_REG_gaoSetSffcState                  (BOSON640_4BYTES + BOSON640_GAO + 0x17) // *
#define P_BOSON640_REG_gaoSetRpmState                   (BOSON640_4BYTES + BOSON640_GAO + 0x19) // *
#define P_BOSON640_REG_gaoSetFfcZeroMeanState           (BOSON640_4BYTES + BOSON640_GAO + 0x25) // *
#define P_BOSON640_REG_gaoSetRnsPopThreshold            (BOSON640_2BYTES + BOSON640_GAO + 0x2b) // *
#define P_BOSON640_REG_gaoSetRnsCloseThreshold          (BOSON640_2BYTES + BOSON640_GAO + 0x2d) // *
#define P_BOSON640_REG_gaoSetRnsTooFewQuit              (BOSON640_2BYTES + BOSON640_GAO + 0x2f) // *
#define P_BOSON640_REG_gaoSetRnsTooFew                  (BOSON640_2BYTES + BOSON640_GAO + 0x31) // *
#define P_BOSON640_REG_gaoSetRnsMinCorrection           (BOSON640_2BYTES + BOSON640_GAO + 0x33) // *
#define P_BOSON640_REG_gaoSetRnsDamping                 (BOSON640_2BYTES + BOSON640_GAO + 0x35) // *
#define P_BOSON640_REG_gaoSetRnsFrameHysteresis         (BOSON640_2BYTES + BOSON640_GAO + 0x37) // *
#define P_BOSON640_REG_gaoSetRnsBadDamping              (BOSON640_1BYTES + BOSON640_GAO + 0x39) // *
#define P_BOSON640_REG_gaoSetRnsNumGoodDampingThreshold (BOSON640_2BYTES + BOSON640_GAO + 0x3b) // *

#define P_BOSON640_REG_roicSetFPARampState              (BOSON640_4BYTES + BOSON640_ROIC + 0x14) // **
#define P_BOSON640_REG_roicSetFPATempOffset             (BOSON640_2BYTES + BOSON640_ROIC + 0x1b) // *
#define P_BOSON640_REG_roicSetFPATempMode               (BOSON640_4BYTES + BOSON640_ROIC + 0x1d) // *
#define P_BOSON640_REG_roicSetFPATempValue              (BOSON640_2BYTES + BOSON640_ROIC + 0x22) // *

#define P_BOSON640_REG_bprSetState                      (BOSON640_4BYTES + BOSON640_BPR +  0x02) // **

#define P_BOSON640_REG_telemetrySetState                (BOSON640_4BYTES + BOSON640_TELEMETRY +  0x01) // *****
#define P_BOSON640_REG_telemetrySetLocation             (BOSON640_4BYTES + BOSON640_TELEMETRY +  0x03) // *****
#define P_BOSON640_REG_telemetrySetPacking              (BOSON640_4BYTES + BOSON640_TELEMETRY +  0x05) // **

#define P_BOSON640_REG_bosonRunFFC                      (BOSON640_0BYTES + BOSON640_BOSON +  0x07) // *****
#define P_BOSON640_REG_bosonSetFFCTempThreshold         (BOSON640_2BYTES + BOSON640_BOSON +  0x08) // ***
#define P_BOSON640_REG_bosonSetFFCFrameThreshold        (BOSON640_4BYTES + BOSON640_BOSON +  0x0b) // ***
#define P_BOSON640_REG_bosonReboot                      (BOSON640_0BYTES + BOSON640_BOSON +  0x10) // *
#define P_BOSON640_REG_bosonSetFFCMode                  (BOSON640_4BYTES + BOSON640_BOSON +  0x12) // ***
#define P_BOSON640_REG_bosonSetGainMode                 (BOSON640_4BYTES + BOSON640_BOSON +  0x14) // ***
#define P_BOSON640_REG_bosonSetFFCWarnTimeInSecx10      (BOSON640_2BYTES + BOSON640_BOSON +  0x74) // *
#define P_BOSON640_REG_bosonSetOverTempTimerInSec       (BOSON640_2BYTES + BOSON640_BOSON +  0x77) // *
#define P_BOSON640_REG_bosonSetTimeForQuickFFCsInSecs   (BOSON640_4BYTES + BOSON640_BOSON +  0x7a) // *
#define P_BOSON640_REG_bosonSetExtSyncMode              (BOSON640_4BYTES + BOSON640_BOSON +  0x98) // *****

#define P_BOSON640_REG_dvoSetDisplayMode                (BOSON640_4BYTES + BOSON640_DVO + 0x0d) // ***** (single/continuous)
#define P_BOSON640_REG_dvoSetType                       (BOSON640_4BYTES + BOSON640_DVO + 0x0f) // (mono16/....)

#define P_BOSON640_REG_scnrSetEnableState               (BOSON640_4BYTES + BOSON640_SCNR + 0x01) // **
#define P_BOSON640_REG_scnrSetThColSum                  (BOSON640_2BYTES + BOSON640_SCNR + 0x03) // *
#define P_BOSON640_REG_scnrSetThPixel                   (BOSON640_2BYTES + BOSON640_SCNR + 0x05) // *
#define P_BOSON640_REG_scnrSetMaxCorr                   (BOSON640_2BYTES + BOSON640_SCNR + 0x07) // *
#define P_BOSON640_REG_scnrSetThPixelSafe               (BOSON640_2BYTES + BOSON640_SCNR + 0x0e) // *
#define P_BOSON640_REG_scnrSetMaxCorrSafe               (BOSON640_2BYTES + BOSON640_SCNR + 0x10) // *

#define P_BOSON640_REG_tfSetEnableState                 (BOSON640_4BYTES + BOSON640_TNR + 0x01) // **
#define P_BOSON640_REG_tfSetDelta_nf                    (BOSON640_2BYTES + BOSON640_TNR + 0x03) // *
#define P_BOSON640_REG_tfSetTHDeltaMotion               (BOSON640_2BYTES + BOSON640_TNR + 0x05) // *
#define P_BOSON640_REG_tfSetMotionThreshold             (BOSON640_4BYTES + BOSON640_TNR + 0x0e) // *

#define P_BOSON640_REG_spnrSetEnableState               (BOSON640_4BYTES + BOSON640_SPNR + 0x01) // **
#define P_BOSON640_REG_spnrSetFrameDelay                (BOSON640_4BYTES + BOSON640_SPNR + 0x05) // *
// All others in SPNR require FLOAT values that are not supported in the sequencer

#define P_BOSON640_REG_sysctrlSetFreezeState            (BOSON640_4BYTES + BOSON640_SYSCTL + 0x01) // **

// Not all test ramp commands fit
#define P_BOSON640_REG_testRampSetMotionState           (BOSON640_4BYTES + BOSON640_TESTRAMP + 0x04) // *
#define P_BOSON640_REG_testRampSetIndex                 (BOSON640_1BYTES + BOSON640_TESTRAMP + 0x06) // *

#define P_BOSON640_REG_srnrSetEnableState               (BOSON640_4BYTES + BOSON640_SRNR + 0x01) // **
#define P_BOSON640_REG_srnrSetThRowSum                  (BOSON640_2BYTES + BOSON640_SRNR + 0x03) // *
#define P_BOSON640_REG_srnrSetThPixel                   (BOSON640_2BYTES + BOSON640_SRNR + 0x05) // *
#define P_BOSON640_REG_srnrSetMaxCorr                   (BOSON640_2BYTES + BOSON640_SRNR + 0x07) // *


/**
 * SENSOR_REGSxx
 */
#define P_BOSON_GAO_GAIN                  0 ///< Enables / disables application of per-pixel gain coefficients
#define P_BOSON_GAO_FFC                   1 ///< Enables / disables application of per-pixel Flat-Field Correction (FFC) coefficients
#define P_BOSON_GAO_TEMP_CORRECTION       2 ///< Enables / disables application of per-pixel temperature corrections
#define P_BOSON_GAO_AVERAGER              3 ///< Enables / disables a smart-averager function which cuts frame rate in half
#define P_BOSON_GAO_NUM_FFC_FRAMES        4 ///< Specifies the number of frames (2, 4, 8, or 16) to be integrated during flat-field correction (FFC)
#define P_BOSON_GAO_RNS                   5 ///< Overrides the availability of the row-noise suppression (RNS) algorithm
#define P_BOSON_GAO_TEST_RAMP             6 ///< Enables / disables a test ramp generated by internal electronics (in lieu of data from the sensor array)
#define P_BOSON_GAO_SFFC                  7 ///< Enables / disables supplemental flat-field correction (SFFC)
#define P_BOSON_GAO_RPM                   8 ///< RPM (?) enable
#define P_BOSON_GAO_FFC_ZERO_MEAN         9 ///< (?)
#define P_BOSON_GAO_RNS_POP_THRESHOLD    10 ///< (?)
#define P_BOSON_GAO_RNS_CLOSE_THRESHOLD  11 ///< (?)
#define P_BOSON_GAO_RNS_TOO_FEW          12 ///< (?)
#define P_BOSON_GAO_RNS_MIN_CORRECTION   13 ///< (?)
#define P_BOSON_GAO_RNS_DAMPING          14 ///< (?)
#define P_BOSON_GAO_RNS_FRAME_HYSTERESIS 15 ///< (?)
#define P_BOSON_GAO_RNS_BAD_DAMPING      16 ///< (?)
#define P_BOSON_GAO_RNS_NUM_GOOD_DAMPING 17 ///< (?)
#define P_BOSON_ROIC_FPA_RAMP            18 ///< Enables / disables a test ramp generated by the sensor array
#define P_BOSON_ROIC_FPA_TEMP_OFFSET     19 ///< Specifies an override of or an offset applied to the camera's internal temperature sensor
#define P_BOSON_ROIC_FPA_TEMP_MODE       20 ///< Specifies the FPA temp mode (normal, fixed/override, or offset)
#define P_BOSON_ROIC_FPA_TEMP_VALUE      21 ///< Sets the value of the FPA temp when the FPA temp mode is set to fixed
#define P_BOSON_BPR                      22 ///< Enables / disables the bad-pixel replace (BPR) algorithm
#define P_BOSON_TELEMETRY                23 ///< Set the telemetry state to Enabled or Disabled
#define P_BOSON_TELEMETRY_LOCATION       24 ///< Set the telemetry to before(top) or after(bottom) the image
#define P_BOSON_TELEMETRY_PACKING        25 ///< Sets the type of packing that the telemetry data is preseneted - 16 Bit, Color or 8 -Bit
#define P_BOSON_BOSON_RUN_FFC            26 ///< Performs an FFC operation *** No matching read !! ***
#define P_BOSON_BOSON_SET_FFC_TEMP       27 ///< Sets the temperature threshold (in degC*10) for the FFC desired flag
#define P_BOSON_BOSON_SET_FFC_FRAME      28 ///< Sets the time threshold (in seconds) for the FFC desired flag
#define P_BOSON_BOSON_FFC_MODE           29 ///< Sets the mode of the camera's FFC operation (0 - manual, 1 - auto, 2 - external, 3- shutter test...)
#define P_BOSON_BOSON_GAIN_MODE          30 ///< Sets the mode of the camera's temperature compensation operation (0 - high, 1 - low, 2 - auto, ...)
#define P_BOSON_BOSON_FFC_WARN_TIME      31 ///< Sets the amount of time in 10ths of a second before the occurrence of FFC that the warn time symbol ...
#define P_BOSON_BOSON_OVER_TEMP_TIME     32 ///< Sets the time is seconds that we want to wait before setting the camera in low power state after an overTemp event
#define P_BOSON_BOSON_FFC_QUICK_TIME     33 ///< Sets the number of seconds after startup that FFC trigger params are 'reduced' to produce FFC events more often
#define P_BOSON_BOSON_EXT_SYNC_MODE      34 ///< Set the external sync mode for the camera
#define P_BOSON_DVO_DISPLAY_MODE         35 ///< Sets the display mode to continuous (0) or one shot (1)
#define P_BOSON_SCNR                     36 ///< Enable or disable Spatial Column Noise Reduction (scnr)
#define P_BOSON_SCNR_TH_COL_SUM          37 ///< Set the threshold that determines if a column should increment or decrement by 1
#define P_BOSON_SCNR_TH_PIXEL            38 ///< Set the (base) threshold that determines if a neighboring pixel is within range to affect the correction of the center
#define P_BOSON_SCNR_MAX_CORR            39 ///< Set the (base) maximum correction amount that will be applied
#define P_BOSON_SCNR_TH_PIXEL_SAFE       40 ///< Set the (base) threshold (for Safe Mode) that determines if a neighboring pixel is within range to correct the center
#define P_BOSON_SCNR_MAX_CORR_SAFE       41 ///< Set the (base) maximum correction amount (for Safe Mode) that will be applied
#define P_BOSON_TF                       42 ///< Enable or disable Temporal Noise Reduction (tnr)
#define P_BOSON_TF_DELTA_NF              43 ///< Sets the Delta NF value. The delta_nf modifies the filter behavior by scaling the index into the table of weights
#define P_BOSON_TF_TH_DELTA_MOTION       44 ///< Sets the Delta Motion threshold to determine if there was motion in the scene enough to trigger the SPNR algorithm
#define P_BOSON_TF_MOTION_THRESHOLD      45 ///< Sets the motion detection threshold - the number of pixels in a frame moved to trigger SPNR execution
#define P_BOSON_SPNR                     46 ///< Enable/disable Spatial Pattern Noise Reduction (SPNR) correction
#define P_BOSON_SPNR_FRAME_DELAY         47 ///< Sets the frame delay parameter. This determines how many frames it takes between SPNR iterations
#define P_BOSON_SYSCTRL_FREEZE           48 ///< Sets the state of the pipeline freeze parameter (enable/disable)
#define P_BOSON_TEST_RAMP_MOTION         49 ///< Enable or disable looping through the test ramp buffers
#define P_BOSON_TEST_RAMP_INDEX          50 ///< Display the selected ramp buffer on the next frame
#define P_BOSON_SRNR                     51 ///< SRNR On/off
#define P_BOSON_SRNR_TH_ROW_SUM          52 ///< (?)
#define P_BOSON_SRNR_TH_PIXEL            53 ///< (?)
#define P_BOSON_SRNR_MAX_CORR            54 ///< (?)


/* Boson640 "i2c" (uart actually) registers will be defined here */
/**
#define P_BOSON640_POWER       0x00  ///< Power On/Off register
#define P_BOSON640_STATUS      0x02  ///< Status register
#define P_BOSON640_COMMAND_ID  0x04  ///< Command ID register
#define P_BOSON640_DATA_LENGTH 0x06  ///< Data length register
#define P_BOSON640_DATA00      0x08  ///< Data0
#define P_BOSON640_DATA01      0x0a  ///< Data1
#define P_BOSON640_DATA02      0x0c  ///< Data2
#define P_BOSON640_DATA03      0x0e  ///< Data3
#define P_BOSON640_DATA04      0x10  ///< Data4
#define P_BOSON640_DATA05      0x12  ///< Data5
#define P_BOSON640_DATA06      0x14  ///< Data6
#define P_BOSON640_DATA07      0x16  ///< Data7
#define P_BOSON640_DATA08      0x18  ///< Data8
#define P_BOSON640_DATA09      0x1a  ///< Data9
#define P_BOSON640_DATA10      0x1c  ///< Data10
#define P_BOSON640_DATA11      0x1e  ///< Data11
#define P_BOSON640_DATA12      0x20  ///< Data12
#define P_BOSON640_DATA13      0x22  ///< Data13
#define P_BOSON640_DATA14      0x24  ///< Data14
#define P_BOSON640_DATA15      0x26  ///< Data15
*/
// Data model for Lepton does not match 393 convention (1 page for "important" registers,
// so just 1 register of each of the 256-words is registered
/*
#define P_BOSON640_DATAF8      0x28  ///< 0xf800 to 0xf8ff
#define P_BOSON640_DATAF9      0x29  ///< 0xf900 to 0xf9ff
#define P_BOSON640_DATAFA      0x2a  ///< 0xfa00 to 0xfaff
#define P_BOSON640_DATAFB      0x2b  ///< 0xfb00 to 0xfbff
#define P_BOSON640_DATAFC      0x2c  ///< 0xfc00 to 0xfcff
#define P_BOSON640_DATAFD      0x2d  ///< 0xfd00 to 0xfdff
#define P_BOSON640_DATAFE      0x2e  ///< 0xfe00 to 0xfeff
#define P_BOSON640_DATAFF      0x2f  ///< 0xff00 to 0xffff

// actual values for registers (when reading)
#define P_REG_BOSON640_GP3VSYNC     0x0854
#define P_REG_BOSON640_GP3VSYNC_VAL 0x0005
#define P_REG_BOSON640_TELEN        0x0218
#define P_REG_BOSON640_TELLOC       0x021c
#define P_REG_BOSON640_FFC_RUN      0x0242 // "2" in 2 lower bits mean that it is just run, no parameters set

// Actual register address ranges (probably, only even are used too?)
#define P_REG_BOSON640_DATAF8  0xf800  ///< 0xf800 to 0xf8ff
#define P_REG_BOSON640_DATAF9  0xf900  ///< 0xf900 to 0xf9ff
#define P_REG_BOSON640_DATAFA  0xfa00  ///< 0xfa00 to 0xfaff
#define P_REG_BOSON640_DATAFB  0xfb00  ///< 0xfb00 to 0xfbff
#define P_REG_BOSON640_DATAFC  0xfc00  ///< 0xfc00 to 0xfcff
#define P_REG_BOSON640_DATAFD  0xfd00  ///< 0xfd00 to 0xfdff
#define P_REG_BOSON640_DATAFE  0xfe00  ///< 0xfe00 to 0xfeff
#define P_REG_BOSON640_DATAFF  0xff00  ///< 0xff00 to 0xffff
*/

#define BOSON_PACKET_MAX_DATA 756 ///< Maximal length of Boson packet data length

#define BOSON_ERR_NO_PACKET    1  ///< UART read packet not available
#define BOSON_ERR_FBP_FORMAT   2  ///< UART read packet ended prematurely
#define BOSON_ERR_NO_EOP       3  ///< UART read packet > 765 bytes

/**
 * Just set parameter for the Boson640 related register, do not write to sensor
 * @param port  sensor port number
 * @param reg   relative register to fit 256 values page
 * @param data  value to set (16 bits)
 */
#define SET_BOSON640_PAR_DRY(port,reg,data) {\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
}


/**
 * Set parameter for the Boson640 related register and send related commands to hardware i2c sequencer in immediate mode
 * Only immediate mode allows waiting
 * @param port  sensor port number
 * @param sa7   7-bit i2c slave address
 * @param reg   relative register to fit 256 values page
 * @param data  value to set (16 bits)
 * @param wait_ms - maximal numer of milliseconds to wait Lepton ready
 */
#define SET_BOSON640_PAR_IMMED(port,sa7,reg,data,wait_ms) {\
	int _BOSON640REG = pSensorPortConfig[(port)].par2addr[0][(reg)];\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    if (!(_BOSON640REG&0xffff0000)) {\
    	boson640_set_reg((port), (sa7), _BOSON640REG, (wait_ms), (data));\
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
#define SET_BOSON640_PAR_NOWAIT(port, frame, reg, data) {\
	int _BOSON640REG = pSensorPortConfig[(port)].par2addr[0][(reg)];\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    if (!(_BOSON640REG&0xffff0000)) {\
    	boson640_set_reg_nowait ((port), (frame), _BOSON640REG, (data));\
    }\
}

/*
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
} boson640_status_t;

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
} boson640_command_t;
*/

/*

#define BOSON640_MODULE(x) (((x) >> 8) & 0x3f)


typedef enum BOSON640_MODULES {
	BOSON640_AGC = 1,
	BOSON640_SYS = 2,
	BOSON640_VID = 3,
	BOSON640_OEM = 8,
	BOSON640_RAD = 14
} boson640_modules_t;

typedef enum BOSON640_TYPES {
	BOSON640_GET = 0,
	BOSON640_SET = 1,
	BOSON640_RUN = 2
} boson640_types_t;
*/

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
extern const unsigned short boson640_par2addr[];
extern const unsigned short boson640_pages[];
extern const unsigned short boson640_ahead_tab[];

int boson640_pgm_detectsensor    (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
;
void boson640_set_device(struct device *dev);

/*
int  boson640_wait_ready     (int sensor_port, int sa7, int num_retries);
int  boson640_get_reg        (int sensor_port, int sa7, int cmd, int wait_ms);
int  boson640_set_reg        (int sensor_port, int sa7,   int cmd, int wait_ms, int data);
void boson640_set_reg_nowait (int sensor_port, int frame, int cmd, int data);
// Maybe not needed?
int  boson640_run_cmd        (int sensor_port, int sa7, int cmd, int wait_ms);
void boson640_run_cmd_nowait (int sensor_port, int sa7, int cmd);
*/


#endif
