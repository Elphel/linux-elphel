/***************************************************************************//**
* @file      c313a.h
* @brief     Various parameters and structures accessible from both kernel
* and userland.
* @copyright Copyright 2002-2016 (C) Elphel, Inc.
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
 * 05.03.2002 changing for revA
 * 03.19.2002 Started support for different sensors
 */

#ifndef _ASM_CMOSCAM_H
#define _ASM_CMOSCAM_H
#define SAFE_CHECK  1 // perform more verification on the paremeters
//#define ELPHEL_DEBUG 0 //global debug on/off in multiple files
//#define ELPHEL_DEBUG_STARTUP 000a4c00 ; 
//#define ELPHEL_DEBUG_STARTUP 0 ; // removed - add write to fpga init script
//#define ELPHEL_DEBUG 0 //global debug on/off in multiple files
#define ELPHEL_DEBUG 0 //global debug on/off in multiple files
#define ELPHEL_DEBUG_DELAY 100000 //delay after some printk-s
#define ELP_KERR(x) printk("%s:%d:%s: ERROR ",__FILE__,__LINE__,__FUNCTION__);x
#define ELP_FERR(x) fprintf(stderr,"%s:%d:%s: ERROR ",__FILE__,__LINE__,__FUNCTION__);x

#define USELONGLONG 1
#define ETRAXFS_MMAP_CACHE_BUG y


#if ELPHEL_DEBUG
 #define EDBG(x) x
#else
 #define EDBG(x)
#endif

#define ROUND_UP(N, S) ((((N) + (S) - 1) / (S)) * (S))

// _IOC_TYPE, bits 8 to 15 in ioctl cmd

#define CMOSCAM_IOCTYPE  124

/* new i2c devices */
// minors (add more later - maybe different minors for different speed - set speed when opening)
#ifdef NC353
#define X3X3_I2C_CTRL        0  // control/reset i2c
#define X3X3_I2C_8_AINC      1  // 8bit  registers, autoincement while read/write
#define X3X3_I2C_16_AINC     2  // 16bit registers, autoincement while read/write
#define X3X3_I2C1_8_AINC     3  // 8bit  registers, autoincement while read/write (bus 1)
#define X3X3_I2C1_16_AINC    4  // 16bit registers, autoincement while read/write (bus 1)
#define X3X3_I2C_RAW         5  // 8bit  registers, no address byte (just slave, then read/write byte(s)
#define X3X3_I2C1_RAW        6  // 8bit  registers, no address byte (just slave, then read/write byte(s)
#define X3X3_I2C_ENABLE      7  // enable(/protect) different I2C devices for different types of I2C accesses
#define X3X3_I2C_ENABLE_RD   0 // bit 0 - enable i2c read
#define X3X3_I2C_ENABLE_WR   1 // bit 1 - enable i2c write
#define X3X3_I2C_ENABLE_RAW  2 // bit 2 - enable i2c raw (no address byte)
#define X3X3_I2C_ENABLE_8    3 // bit 3 - enable i2c 8-bit registers access
#define X3X3_I2C_ENABLE_16   4 // bit 4 - enable i2c 16-bit registers access
#define X3X3_I2C_MAXMINOR  7  //
#define X3X3_I2C_CHANNELS  2  // number of i2c channels
//xi2craw            c 134   5
//xi2craw_aux        c 134   6
//xi2cenable         c 134   7

/* camera sequencer states */
//Obsolete
#define CAMSEQ_OFF       0 // off, not programmed (Video mode off on Zoran sensors)
#define CAMSEQ_READY     1 // sensor programmed may acquire at will (programSensor sets number of frames to skip (if any)
#define CAMSEQ_SKIP      2 // skipping specified number of frames, interrupt service routine counts and will start acquisition
#define CAMSEQ_WAIT_F 	 3 // set by "start exposure" or interrupt service routine. WAIT_F/WAIT_T/acquire/done differs by hardware register
#define CAMSEQ_WAIT_T 	 4 // set by "start exposure" or interrupt service routine. Wait/acquire/done differs by hardware register
#define CAMSEQ_ACQUIRE   5 // acquisition in progress (camSeqState is still CAMSEQ_WAIT)
#define CAMSEQ_DONE      6 // acquisition over  (camSeqState is still CAMSEQ_WAIT)
#define CAMSEQ_JPEG      7 // waiting for JPEG done interrupt, acquiring/compressing some frames

#define CAMSEQ_RUN       8 // compressor is constantly running (but if camSeqCount>0 - just skipping "bad" frames)
#define CAMSEQ_STOP      9 // compressor is constantly running but will stop after next "compressor ready"
#define CAMSEQ_SINGLE   10 // compressor is constantly running to fill one full buffer

// For KAI11000 sensor board
#define	sensorcom_W_size	1024
#define	sensorcom_R_size	256

/* MCP definitions (only used in 313 with MCP image intensifier */

#define	MCP_W_size	1024
#define	MCP_R_size	256
#define	MCPOtherBits	0xffa7a7ff
#define MCPOffReset	0x00101800
#define MCPReset	0x00001800
#define MCPNoReset	0x00105800
#define MCPToggleA	0x00080000
#define MCPToggleB	0x00001000
#define MCPctlseq	0x00
#define MCPsofttg	0x02
#define MCPeackn	0x03
#define MCPctlgate	0x04
#define MCPwstdly	0x06
#define MCPwrsmsk	0x07
#define MCPwrsup	0x08
#define MCPwrmons	0x09
#define MCPwnom		0x0a
#define MCPwdenom	0x0b
#define MCPwoutw	0x0c
#define MCPwinvctl	0x0d
#define MCPctlsync	0x0e
#define MCPwrdlys	0x10
#define MCPwinv		0x40
#define MCPwshared	0x80
#define MCPwrsynctb	0x100
#define MCPwrseq	0x200
#else
    // Temporarily porting, to use only bus = 1 (GPS, IMU)
    // Moved to include/uapi/elphel/x393_devices.h
#if 0
    #define X3X3_I2C_CTRL        0  ///< control/reset i2c
    #define X3X3_I2C_8_AINC      1  ///< 8bit  registers, autoincement while read/write
    #define X3X3_I2C_16_AINC     2  ///< 16bit registers, autoincement while read/write
    #define X3X3_I2C1_8_AINC     3  ///< 8bit  registers, autoincement while read/write (bus 1)
    #define X3X3_I2C1_16_AINC    4  ///< 16bit registers, autoincement while read/write (bus 1)
    #define X3X3_I2C_RAW         5  ///< 8bit  registers, no address byte (just slave, then read/write byte(s)
    #define X3X3_I2C1_RAW        6  ///< 8bit  registers, no address byte (just slave, then read/write byte(s)
    #define X3X3_I2C_ENABLE      7  ///< enable(/protect) different I2C devices for different types of I2C accesses
#endif
    #define X3X3_I2C_ENABLE_RD   0  ///< bit 0 - enable i2c read
    #define X3X3_I2C_ENABLE_WR   1  ///< bit 1 - enable i2c write
    #define X3X3_I2C_ENABLE_RAW  2  ///< bit 2 - enable i2c raw (no address byte)
    #define X3X3_I2C_ENABLE_8    3  ///< bit 3 - enable i2c 8-bit registers access
    #define X3X3_I2C_ENABLE_16   4  ///< bit 4 - enable i2c 16-bit registers access
    #define X3X3_I2C_MAXMINOR    7  ///<
    #define X3X3_I2C_CHANNELS    2  ///< number of i2c channels (0 is not used in NC393)
#endif


#define FPGA_DCM_STEP   22 // ps/step
#define FPGA_DCM_RANGE 250 // maximal phase correction (+/-) ?


/* supported ioctl _IOC_NR's */
#ifndef I2C_WRITEARG
 #define I2C_WRITEARG(bus, slave, reg, value) (((bus) << 24) | ((slave) << 16) | ((reg) << 8) | (value))
 #define I2C_READARG(bus, slave, reg) (((bus) << 24) | ((slave) << 16) | ((reg) << 8))

 #define I2C_ARGBUS(arg) (((arg) >> 24)  & 0x1)
 #define I2C_ARGSLAVE(arg) (((arg) >> 16)  & 0xff)
 #define I2C_ARGREG(arg) (((arg) >> 8) & 0xff)
 #define I2C_ARGVALUE(arg) ((arg) & 0xff)

 #define I2C_DELAYS   0x0   // read/write bit deleys for I2C
// return  delays, if data==0 - don't change, just read
// lower (0) byte - SCL high,
// byte 1 - SCL low
// byte 2 - slave -> master (from slave driving SDA line to master driving SDA)
// byte 3 - master -> slave (from master driving SDA line to slave driving SDA)

 #define I2C_WRITEREG 0x1   // write to an i2c register
 #define I2C_READREG  0x2   // read from an i2c register
#endif

// new for Micron sensors  - 16bit data, always bus 0
#ifndef I2C_16_WRITEARG
 #define I2C_16_WRITEREG 0x3   // write 2 bytes to an i2c register
 #define I2C_16_READREG  0x4   // read 2 bytes from an i2c register
  
 #define I2C_16_WRITEARG(slave, reg, value) (((slave) << 24) | ((reg) << 16) | (value))
 #define I2C_16_READARG(slave, reg) (((slave) << 24) | ((reg) << 16))

 #define I2C_16_ARGSLAVE(arg)   (((arg) >> 24) & 0xff)
 #define I2C_16_ARGREG(arg)     (((arg) >> 16) & 0xff)
 #define I2C_16_ARGVALUE(arg)   ( (arg)      & 0xffff)
 #define I2C_16_ARGVALUE_H(arg) (((arg) >>  8) & 0xff)
 #define I2C_16_ARGVALUE_L(arg) ( (arg)        & 0xff)
#endif



#ifdef NC353
// otherParamsRO[16]
#define _CCCMD(x,y)  (_IO(CMOSCAM_IOCTYPE, (x << 6) | (y & 0x3f)))
#define CCAM_CTRL(x)  ((_IOC_NR(x) >> 6) & 0x03)
#define CCAM_ADDR(x)  (_IOC_NR(x) & 0x3f)
//#define CCAM_RWSENSOR   1 /* direct read/write first 32 sensor registers */  // will not use at all
#define CCAM_RPARS        2 /* read  parameters      0..0x3f */
#define CCAM_WPARS        3 /* write parameters      0..0x3f */
#endif
/* New parameters and update logic
 * Separate read and write set of 64 registers
 * User may specify:
 *   0 - do not update
 *   1 - update at once
 *   2 - update when appropriate
 * and read update status:
 *   0 - will not be updated
 *   1 - in sync
 *   2 - waiting to be updated
 *   3 - update in progress (TODO: - support async)
 * 	When updating (validating) parameters and copying them to the read "registers" the I2C registers will be written
 *  only if they are different from the shadows
 */
// parameter indexes will be updated to group-related ones into the same groups of 32
// NOTE: P_* and G_* should not end with numbers - numbers will be used in PHP constants to add to the constant value (ELPHEL_AAA3 will be treated as ELPHEL_AAA+3)
#define P_NUMBER         1024   //number of registers (was 64) - NOTE: obsolete?
#define P_SENSOR            1   /**< if set to 0 - will (re)detect sensor, if set to None - won't bother <ul>
<li>                        4  - ZR32112MLC - now there is no way to see color/mono
<li>                        5  - ZR32112PLC
<li>                        8  - ZR32212MLC
<li>                        9  - ZR32112PLC
<li>                       32 - KAC1310-mono
<li>                       33 - KAC1310-RGB
<li>                       34 - KAC1310-CMY
<li>                       36 - KAC5000
<li>                       48 - MI1300
<li>                       49 - MT9M001 (1280x1024,same as MI1300)
<li>                       50 - MT9M001 (1600x1200)
<li>                       51 - MT9T001 (2048*1536)
<li>                       52 - MT9P001 (2592*1944)
<li>                       64 - IBIS5-1300 </ul>*/

// leave it here - may be used in user applications
#define SENSOR_MASK      0xfc ///< Mask to apply for distinguish between different sensor groups (of similar devices)
#define SENSOR_DETECT    0x00 ///< Unknown sensor, try to autodetect
#define SENSOR_ZR32112   0x04 ///< Zoran ZR32112
#define SENSOR_ZR32212   0x08 ///< Zoran ZR32212
#define SENSOR_KAC1310   0x20 ///< Kodak KAC1310
#define SENSOR_KAC5000   0x24 ///< Kodak KAC5000
#define SENSOR_MI1300    0x30 ///< Micron MI1300
#define SENSOR_MT9X001   0x30 ///< Micron: MT9M001 - 31, MT9D001 - 32, MT9T001 - 33, MT9P001 - 34
#define SENSOR_MT9M001   0x31 ///< MT9M001
#define SENSOR_MT9D001   0x32 ///< MT9D001
#define SENSOR_MT9T001   0x33 ///< MT9T001
#define SENSOR_MT9P006   0x34 ///< MT9P006
#define SENSOR_MT9F002   0x38 ///< MT9F002
//#define SENSOR_MT9Y001   0x34 ///< Micron/Aptina/Onsemi MT9P001 - 34
#define SENSOR_IBIS51300 0x40 ///< FillFactory IBIS51300
#define SENSOR_KAI11000  0x80 ///< Kodak KAI11002
#define SENSOR_MUX_10359 0xe0 ///< Sensor multiplexer 10359
#define SENSOR_NONE      0xfc ///< No sensor present

// sensor sizes:
#define SENSORWIDTH_ZR32112    1280 ///< Zoran ZR32112 width
#define SENSORHEIGHT_ZR32112   1024 ///< Zoran ZR32112 height
#define SENSORWIDTH_ZR32212    1280 ///< Zoran ZR32212 width
#define SENSORHEIGHT_ZR32212    968 ///< Zoran ZR32212 height
#define SENSORWIDTH_KAC1310    1280 ///< Kodak KAC1310 width
#define SENSORHEIGHT_KAC1310   1024 ///< Kodak KAC1310 height
#define SENSORWIDTH_MI1300     1280 ///< Micron MI1300 width
#define SENSORHEIGHT_MI1300    1024 ///< Micron MI1300 height
#define SENSORWIDTH_MT9M001    1280 ///< Micron MT9M001 width
#define SENSORHEIGHT_MT9M001   1024 ///< Micron MT9M001 height
#define SENSORWIDTH_MT9D001    1600 ///< Micron MT9D001 width
#define SENSORHEIGHT_MT9D001   1200 ///< Micron MT9D001 height
#define SENSORWIDTH_MT9T001    2048 ///< Micron MT9T001 width
#define SENSORHEIGHT_MT9T001   1536 ///< Micron MT9T001 height
#define SENSORWIDTH_MT9P001    2592 ///< Micron MT9P001 width
#define SENSORHEIGHT_MT9P001   1944 ///< Micron MT9P001 height
#define SENSORWIDTH_KAC5000    2592 ///< Kodak KAC5000 width
#define SENSORHEIGHT_KAC5000   1944 ///< Kodak KAC5000 height
#define SENSORWIDTH_IBIS51300  1280 ///< FillFactory IBIS51300 width
#define SENSORHEIGHT_IBIS51300 1024 ///< FillFactory IBIS51300 height

//TODO: Update CONST_NAME_ENTRY_* to new P_vars
/// Parameters related to multi-sensor (10359A) setup
//#define MAX_SENSORS 3 // maximal number of sensor attached (modify some hard-wired constants below if this to be changed)
/* Modified for 393 - using up to 4 sub-sensors (even as 10359 only supports 3 */
#define MAX_SENSORS  4        ///< maximal number of sensor attached (modify some hard-wired constants below if this to be changed)
#define SENSOR_PORTS 4        ///< Number of sensor ports (each has individual framepars_all_t
/* Notes for NC353:
 * Parameters below are accessed through mmap, because of cache coherence problem it make sense to keep them compact (maybe group by 8 - cache line of 32 bytes)
 */
#define P_SENSOR_RUN     4          ///< Sensor acquisition mode 0 - stop, 1 - single, 2 - run
  #define SENSOR_RUN_STOP   0       ///< Sensor acquisition mode: STOP
  #define SENSOR_RUN_SINGLE 1       ///< Sensor acquisition mode: SINGLE FRAME
  #define SENSOR_RUN_CONT   2       ///< Sensor acquisition mode: RUN continuously
  #define SENSOR_RUN_RESET  3       ///< Sensor acquisition mode: RESET

#define P_COMPRESSOR_RUN 5          ///< Compressor mode 0 - stop, 1 - single, 2 - run
  #define COMPRESSOR_RUN_STOP   0   ///< Compressor mode: STOP
  #define COMPRESSOR_RUN_SINGLE 1   ///< Compressor mode: SINGLE
  #define COMPRESSOR_RUN_CONT   2   ///< Compressor mode: RUN
  #define COMPRESSOR_RUN_RESET  2   ///< Compressor mode: RESET

#define P_BAYER          6 ///< filter number at (0,0) 0-R, 1-G(R), 2-G(B), 3 - B. Write enabled at first, move to WindowSize later
#define P_TRIGGERED	     7 ///< when trigger occured - 4 LSBs - pixel in DMA word, higher bits - number of DMA word OBSOLETE
#define P_PERIOD         8 ///< Frame period in pixel clocks (read only)
#define P_FP1000SLIM     9 ///< FPS limit, frames per 1000 sec
#define P_FPSFLAGS      10 //v FPS limit mode - bit 0 - limit fps (not higher than), bit 1 - maintain fps (not lower than)

#define P_JPEG_WP       11 ///< Last reported JPEG write pointer in the circular buffer. ** OBSOLETE
                            // In new code use G_CIRCBUFWP instead!
#define P_CLK_FPGA      12 ///< FPGA clock in MHz
#define P_CLK_SENSOR    13 ///< Sensor clock in MHz
#define P_FPGA_XTRA     14 ///< Extra cycles needed to compressor (probably constant...)
#define P_TRIG          15 /**< External trigger mode<ul>
<li>                     bit 0  - "old" external mode (0 - internal, 1 - external )
<li>                     bit 1 - enable(1) or disable(0) external trigger to stop clip
<li>                     bit 2 - async (snapshot, ext trigger) mode, 0 - continuous NOTE: Only this bit is used now
<li>                     bit 3 - no overlap,  single frames: program - acquire/compress same frame
<li>                     bit 4 - global reset release (GRR) mode - (only when combined with bit 2)
</ul>*/
#define P_BGFRAME       16 ///< Background measurement mode - will use 16-bit mode and no FPN correction
//#define P_IMGSZMEM      17 ///< image size in video memory (calculated when channel 0 is programmed) NC393: Not used ???
//#define P_COMP_BAYER    17 ///< -> 119 derivative, readonly - calculated from P_BAYER and COMPMOD_BYRSH to separate sensor and compressor channels
// image page numbers depend on image size/pixel depth, so changing any of them will invalidate all pages
#define P_PAGE_ACQ      18 ///< Number of image page buffer to acquire to (0.1?)  NC393: Not used ???
#define P_PAGE_READ     19 ///< Number of image page buffer to read from to (0.1?)  NC393: Not used ???
#define P_OVERLAP       20 ///< number of EXRA lines to be acquired - probably dead,
#define P_VIRT_KEEP     21 ///< 0 - recalculate P_VIRT_WIDTH, P_VIRT_HEIGHT when window is changed, 1 - keep those parameters
#define P_VIRT_WIDTH    22 ///< Virtual window width
#define P_VIRT_HEIGHT   23 ///< Virtual window height
#define P_WOI_LEFT      24 ///< WOI left corner (before applying decimation)
#define P_WOI_TOP       25 ///< WOI top corner
#define P_WOI_WIDTH     26 ///< WOI width
#define P_WOI_HEIGHT    27 ///< WOI height
#define P_FLIPH         28 ///< bit 0 horizontal flip
#define P_FLIPV         29 ///< bit 0 vertical flip
#define P_DCM_HOR       30 ///<  Horizontal decimation (1/2/4/8)
#define P_DCM_VERT      31 ///<  Vertical   decimation (1/2/4/8) copied from horizontal for Zoran chips
#define P_BIN_HOR       32 ///<  binning 1/2 - KAC1310 only - now for mt9t001
#define P_BIN_VERT      33 ///<  not used yet binning 1/2 - KAC1310 only  - now for mt9t001
#define P_FPGATEST      34 ///<  FPGA test modes (now - just one)
#define P_FRAMESYNC_DLY 35 ///<  maybe - temporary - delay of frame sync (vacts) by number of scan lines - for photofinish mode // not used anywhere?
                           ///<  Lower bits 16 will be used to delay frame sync, bit 16 - use internal HACT duration (0 - use from sensor) [*]
#define P_PF_HEIGHT     36 ///<  height of each strip in photofinish mode - normally 2 lines
                           ///<  also now includes timestamping mode +0x10000 - for normal frames, 0x20000 - for photo-finish
#define P_BITS          37 ///<  pixel depth - bits 10/8/4
#define P_SHIFTL        38 ///<  "digital gain" - shift left by 0/1/2 bits  (3 ->-1)
#define P_FPNS          39 ///<  FPN correction mode (subtract) 0..3
                           ///<  0-none, 1 - fine(25%), 2 - 50%, 3 - coarse(100%)
#define P_FPNM          40 ///<  FPN correction mode (multiply) 0..3
                           ///<  0-none, 1 - fine(+/-12.5%), 2 - medium (+/-25%), +3 - coarse(+/-50%)
#define P_TESTSENSOR    41 ///<  sensor test mode(s) 0x10000 - enable, lower bits - test mode
#define P_VIRTTRIG      42 ///<  Sum of pixels in a line greater than this value - trigger acquisition

#define P_PERIOD_MIN    43 ///<  (readonly)  minimal frame period (in pixel clocks) limited by user or compressor
#define P_PERIOD_MAX    44 ///<  (readonly) frame period (in pixel clocks) limited by user
#define P_SENSOR_PIXH   45 ///<  (readonly) pixels to be read from the sensor, horizontal (including margins, excluding embedded timestamps).
                           ///<  In multisensor (per channel) applies to the whole frame.
#define P_SENSOR_PIXV   46 ///<  (readonly) pixels to be read from the sensor, vertical (including margins)
#define P_FATZERO       47 ///<  subtract while adding data from to consecutive frames (async trigger)

#define P_COMPMOD_TILSH 48 ///<  Horizontal tiles - obsolete in 393? YES
#define P_COMPMOD_DCSUB 49 ///<  Subtract DC in compressor before DCT
#define P_COMPMOD_QTAB  50 ///<  to be written not directly, but by  pgm_quality ?

#define P_FP1000S       51 ///<  Frames per 1000 sec (fps * 1000)
#define P_SENSOR_WIDTH  52 ///<  Sensor width
#define P_SENSOR_HEIGHT 53 ///<  Sensor height
#define P_COLOR_SATURATION_BLUE 54 ///<  100*relative saturation blue - preserve?
#define P_COLOR_SATURATION_RED  55 ///<  100*relative saturation red

/// Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C *** 393: These will  need to be split for each subchannel
#define VIGNET_SUBCHN_OFFSET 0 ///< for individual per-subchannel vignetting parameters (add num_sub_chn * VIGNET_SUBCHN_OFFSET)
                               ///< set for NC393 when define, meanwhile will use the same for all sub-channels
#define P_VIGNET_AX      56 ///< AX in Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C. 393: These will  need to be split for each subchannel
#define P_VIGNET_AY      57 ///< AY in Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C. 393: These will  need to be split for each subchannel
#define P_VIGNET_BX      58 ///< BX in Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C. 393: These will  need to be split for each subchannel
#define P_VIGNET_BY      59 ///< BY in Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C. 393: These will  need to be split for each subchannel
#define P_VIGNET_C       60 ///< C (nominal 0x8000) in Vignetting control, AX*X^2+BX*X+AY*Y^2+BY*Y+C. 393: These will  need to be split for each subchannel
#define P_VIGNET_SHL     61 ///< shift left color_coeff*vign_correction. 0..7, default=1 (up to 4x color correction* vignetting correction)
#define P_SCALE_ZERO_IN  62 ///< signed 16 bit - subtract from pixel 16-bit data before multiplication
#define P_SCALE_ZERO_OUT 63 ///< signed 16 bit - add after correction


/// "digital gains" for color correction, 17-bit unsigned data (default 0x8000).
#define P_DGAINR        64 ///< R-channel of scale sensor data (fine steps) 17-bit (2.15)  unsigned data, default 0x8000 == 1.0
#define P_DGAING        65 ///< G-channel (R-row) of scale sensor data (fine steps) 17-bit (2.15)  unsigned data, default 0x8000 == 1.0
#define P_DGAINGB       66 ///< G-channel (B-row) of scale sensor data (fine steps) 17-bit (2.15)  unsigned data, default 0x8000 == 1.0
#define P_DGAINB        67 ///< B-channel of scale sensor data (fine steps) 17-bit (2.15)  unsigned data, default 0x8000 == 1.0
#define P_RSCALE_ALL    68 ///< bits 0..29: Ratio of [P_GAINR]/[P_GAING] or one of special, bit 30 - recalculate(self cleaning), bit 31 - ignore
#define P_GSCALE_ALL    69 ///< bits 0..29: Ratio of [P_GAINGB]/[P_GAING] or one of special, bit 30 - recalculate(self cleaning), bit 31 - ignore
#define P_BSCALE_ALL    70 ///< bits 0..29: Ratio of [P_GAINB]/[P_GAING] or one of special, bit 30 - recalculate(self cleaning), bit 31 - ignore
	#define CSCALES_WIDTH     28 ///< color scales ratio number of bits
	#define CSCALES_CTL_BIT   28 ///< color scales ratio control bit number
	#define CSCALES_CTL_WIDTH  2 ///< color scales ratio number of control bits
	/// commands to be written to P_*SCALE_CTL
	#define CSCALES_CTL_NORMAL  0 ///< P_*SCALE_CTL command: USE P_*SCALE as is
	#define CSCALES_CTL_RECALC  1 ///< P_*SCALE_CTL command: Recalculate P_*SCALE from P_GAIN*, P_GAING, then use it
	                              ///< (will change to CSCALES_CTL_NORMAL when applied)
	#define CSCALES_CTL_FOLLOW  2 ///< P_*SCALE_CTL command: Don't apply P_*SCALE to P_GAIN*, but update it
	                              ///< from the current P_*SCALE from P_GAIN*/P_GAING
	#define CSCALES_CTL_DISABLE 3 ///< P_*SCALE_CTL command: Disable P_*SCALE - don't apply P_*SCALE to P_GAIN*,
	                              ///< don't update P_*SCALE from P_GAIN*, P_GAING
#define P_CORING_PAGE   71       ///< zero bin + ((rounding add) << 8) 8-bit JPEG quantizer zero bin size, fractional addition to absolute value before truncating
#define P_HISTRQ        72       ///< per-frame enabling of histogram calculation - bit 0 - Y (G), bit 2 - C (R,G2,B)
#define P_TILES         73       ///< Number of 16x16 (20x20) tiles in a compressed frame // 393: Still needed?
#define P_SENSOR_PHASE  74       ///< packed, low 16 bit - signed fine phase, bits [18:17] - 90-degrees shift
                                 ///< NC393 parallel12:  6 LSBs mean quadrants:  90-degree shifts for data [1:0], hact [3:2] and vact [5:4]
#define P_TEMPERATURE_PERIOD  75 ///< period of temperature measurements, ms (normally - multiple seconds)
#define P_AUTOEXP_ON    76       ///< Autoexposure control (unsigned long on)
// relative histogram (autoexposure) window (changed from % to 1/0x10000)
// TODO: For 393 - split per multiplexed channel
#define HIST_SUBCHN_OFFSET 0 ///< for individual per-subchannel vignetting parameters (add num_sub_chn * VIGNET_SUBCHN_OFFSET)
                               ///< set for NC393 when define, meanwhile will use the same for all sub-channels
#define P_HISTWND_RWIDTH  77 ///< Histogram window relative width 16.16 (0x10000 - 1.0);
#define P_HISTWND_RHEIGHT 78 ///< Histogram window relative height 16.16 (0x10000 - 1.0);
#define P_HISTWND_RLEFT   79 ///< Histogram window relative left margin 16.16 (0x10000 - 1.0);
#define P_HISTWND_RTOP    80 ///< Histogram window relative top margin 16.16 (0x10000 - 1.0);

// Are these used anywhere now? ...P_AUTOEXP_SKIP_T
#define P_AUTOEXP_EXP_MAX     81 ///< unsigned long exp_max;		/* 100 usec == 1 etc... */
#define P_AUTOEXP_OVEREXP_MAX 82 ///< unsigned long overexp_max;	/* percentages for overexposured pixels - 1% == 100, 5% == 500, 0.02% == 2 etc... */
#define P_AUTOEXP_S_PERCENT   83 ///< unsigned long s_percent;(controlling that % of pixels that should have value greater than S_INDEX - below)
#define P_AUTOEXP_S_INDEX     84 ///< unsigned long s_index; Specified number of pixels (S_PERCENT) should have value above S_INDEX
#define P_AUTOEXP_EXP         85 ///< unsigned long exp; Current exposure time
#define P_AUTOEXP_SKIP_PMIN   86 ///< unsigned long skip_pmin;	/* percent of delta for skip changes: 1% == 100 */ - no exposure corrections if the desired change is less than that
#define P_AUTOEXP_SKIP_PMAX   87 ///< unsigned long skip_pmax;	/* percent of changes for wait one frame before apply changes: 1% == 100 */ - do not apply chnanges if they are to big - wait for the next frame
#define P_AUTOEXP_SKIP_T      88 ///< unsigned long skip_t;		/* time for skip changes: 100 usec == 1 */ Not quite sure what it is

// same as written to the FPGA for the histogram window // 393: Need to split to sub-channels (hist and focus parameters)
#define P_HISTWND_WIDTH  89 ///<  autoexposure window width  (pixels) - same as written to FPGA
#define P_HISTWND_HEIGHT 90 ///<  autoexposure window height (pixels) - same as written to FPGA
#define P_HISTWND_TOP    91 ///<  autoexposure window top    (pixels) - same as written to FPGA
#define P_HISTWND_LEFT   92 ///<  autoexposure window left   (pixels) - same as written to FPGA

#define P_FOCUS_SHOW     93 ///< show focus information instead of/combined with the image:
                            ///< 0 - regular image, 1 - block focus instead of Y DC (AC=0), 2 - image Y DC combined all frame, 3 combined in WOI
#define P_FOCUS_SHOW1    94 ///< Additional parameter that modifies visualization mode. Currently just a single bit (how much to add)

#define P_FOCUS_LEFT     96 ///< focus WOI left margin, inclusive (3 LSB will be zeroed as it should be multiple of 8x8 block width)
#define P_FOCUS_WIDTH    97 ///< focus WOI width (3 LSB will be zeroed as it should be multiple of 8x8 block width)
#define P_FOCUS_TOP      98 ///< focus WOI top margin, inclusive (3 LSB will be zeroed as it should be multiple of 8x8 block height)
#define P_FOCUS_HEIGHT   99 ///< focus WOI height (3 LSB will be zeroed as it should be multiple of 8x8 block height)
#define P_FOCUS_TOTWIDTH 100 ///< (readonly) - total width of the image frame in pixels
#define P_FOCUS_FILTER   101 ///< select 8x8 filter used for the focus calculation (same order as quantization coefficients), 0..14

//timing generator/trigger delay/external trigger control
//In NC393  P_TRIG_DELAY and P_XMIT_TIMESTAMP are per-channel, other parameters are common (Last modified takes control).
// Master channel is set to the current channels when any of the common parameters is set
#define P_TRIG_CONDITION 102 ///< trigger condition, 0 - internal, else dibits ((use<<1) | level) for each GPIO[11:0] pin
#define P_TRIG_DELAY     103 ///< trigger delay, 32 bits in pixel clocks
#define P_TRIG_OUT       104 ///< trigger output to GPIO, dibits ((use << 1) | level_when_active). Bit 24 - test mode, when GPIO[11:10] are controlled by other internal signals
#define P_TRIG_PERIOD    105 ///< output sync period (32 bits, in pixel clocks). 0- stop. 1..256 - single, >=256 repetitive with specified period.
#define P_TRIG_BITLENGTH 106 ///< bit length minus 1 (in pixel clock cycles) when transmitting/receiving timestamps. Legal values 2..255

#define P_TRIG_BITLENGTH_DEFAULT 31 ///< P_TRIG_BITLENGTH default value

#define P_SKIP_FRAMES    107 ///< number of frames to skip after restarting sensor+compressor - now zero/nonzero?
#define P_I2C_QPERIOD    108 ///< number of system clock periods in 1/4 of i2c SCL period to the sensor/sensor board // 393: Moved to DT
#define P_I2C_BYTES      109 ///< number of bytes in hardware i2c write (after slave addr) -0/1/2
#define P_IRQ_SMART      110 ///< TODO: Obsolete for 393, update"smart" IRQ modes: +1 - wait for VACT in early compressor_done,
                             ///< +2 - wait for dma fifo ready // 393: Combine IRQ? 353: bit 0 will be always 1 (needs fix in FPGA)
#define P_EXTERN_TIMESTAMP 111 ///< Use external timestamp (received with sync) when availabele, 0 - always use local timestamp
#define P_OVERSIZE       112 ///< ignore sensor dimensions, use absolute WOI_LEFT, WOI_TOP
#define P_XMIT_TIMESTAMP 113 ///< 0 - transmit just sync pulse, 1 - pulse+timestamp over the sync line

#define P_CORING_INDEX   114 ///< MSW - color coring index (if 0 - use LSW), LSW - Y coring index. Currently 1 step is 1/10 of the pre-quantized DCT
                             ///< coefficient, maximum of 10. For the same filtering effect it should be higher for higher JPEG quality

#define P_RFOCUS_LEFT    115 ///< relative (0x10000 - 1.0) focus WOI left margin, inclusive (3 LSB will be zeroed as it should be multiple of 8x8 block width)
#define P_RFOCUS_WIDTH   116 ///< relative (0x10000 - 1.0)focus WOI width (3 LSB will be zeroed as it should be multiple of 8x8 block width)
#define P_RFOCUS_TOP     117 ///< relative (0x10000 - 1.0)focus WOI top margin, inclusive (3 LSB will be zeroed as it should be multiple of 8x8 block height)
#define P_RFOCUS_HEIGHT  118 ///< relative (0x10000 - 1.0)focus WOI height (3 LSB will be zeroed as it should be multiple of 8x8 block height)

#define P_COMP_BAYER     119 ///< compressor bayer (before  applying P_COMPMOD_BYRSH)
#define P_MEMSENSOR_DLY  120 ///< sensor-to-memory channel frame sync delay in mclk cycles (5ns @200MHz)
// Obsolete in x393, may need something different
#ifdef NC353
#define P_SDRAM_CHN20    125 ///< data to be written to the SDRAM CH2 REG 0  (last moment) TODO: Obsolete in x393, may need something different
#define P_SDRAM_CHN21    126 ///< data to be written to the SDRAM CH2 REG 1 TODO: Obsolete in x393, may need something different
#define P_SDRAM_CHN22    127 ///< data to be written to the SDRAM CH2 REG 2 TODO: Obsolete in x393, may need something different
#else
#define P_SENSOR_IFACE_TIM0 124 ///< Sensor interface timing/configuration word 0 (different meaning for different interfaces)
#define P_SENSOR_IFACE_TIM1 125///< Sensor interface timing/configuration word 1 (different meaning for different interfaces)
#define P_SENSOR_IFACE_TIM2 126 ///< Sensor interface timing/configuration word 2 (different meaning for different interfaces)
#define P_SENSOR_IFACE_TIM3 127 ///< Sensor interface timing/configuration word 3 (different meaning for different interfaces)
#endif
// The following 4 parameters should have consecutive indexes
// see  FRAMEPAIR_MASK_BYTES  to modify just part of the word (i.e. scale, not hash16
//
// Will need to have them per-subchannel (4x)
#define PER_CHANNEL393    0 ///< set to 1 when ready with per-channel for 393, meanwhile will overwrite each other in histograms.c
#define P_GTAB_R         128 ///< combines (P_PIXEL_LOW<<24) | (P_GAMMA <<16) and 16-bit (6.10) scale for gamma tables, individually for each color.
                             ///<  16Msbs are also "hash16" and do not need to be black level/gamma, just uniquely identify the table for applications
#define P_GTAB_G         129 ///< same for the first green (red line)
#define P_GTAB_GB        130 ///< same for the second green (blue line)
#define P_GTAB_B         131 ///< same for the blue

#define P_QUALITY        132 ///< JPEG IMAGE QUALITY (uses 2 bytes to control color/intensity separately)
#define P_ACTUAL_WIDTH   133 ///< RD P_RO_WIDTH  1  pixels/row
#define P_ACTUAL_HEIGHT  134 ///< RD P_RO_HEIGHT 2  pixels/column

// 393: Are they the same?
#define P_COLOR          135 ///< mono - 0, color mode - 1, +0 - normal, 256 - sensor test, 512 - FPGA test
  #define COLORMODE_MONO6     1 ///< monochrome, (4:2:0), was:0
  #define COLORMODE_COLOR     0 ///< color, 4:2:0, 18x18(old) was: 1
  #define COLORMODE_JP46      2 ///< jp4, original (4:2:0)
  #define COLORMODE_JP46DC    3 ///< jp4, dc -improved (4:2:0)
  #define COLORMODE_COLOR20   4 ///<  color, 4:2:0, 20x20, middle of the tile (not yet implemented)
  #define COLORMODE_JP4       5 ///< jp4, 4 blocks, (legacy)
  #define COLORMODE_JP4DC     6 ///< jp4, 4 blocks, dc -improved
  #define COLORMODE_JP4DIFF   7 ///< jp4, 4 blocks, differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
  #define COLORMODE_JP4HDR    8 ///< jp4, 4 blocks, differential HDR: red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
  #define COLORMODE_JP4DIFF2  9 ///< jp4, 4 blocks, differential, divide differences by 2: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
  #define COLORMODE_JP4HDR2  10 ///< jp4, 4 blocks, differential HDR: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2),
  #define COLORMODE_MONO4    14 ///< monochrome, 4 blocks (but still with 2x2 macroblocks)
// the following 8 values should go in the same sequence as fields in the histogram page
// 393: per sub-channel
//// Will need to have them per-subchannel (4x)
//Next 8 copied to histograms data
#define P_FRAME          136 ///< Frame number (reset with JPEG pointers) -(read only)
#define P_GAINR          137 ///< R channel gain  8.16 (0x10000 - 1.0). Combines both analog gain and digital scaling
#define P_GAING          138 ///< G channel gain ("red line")
#define P_GAINGB         139 ///< G channel gain ("blue line")
#define P_GAINB          140 ///< B channel gain
#define P_EXPOS          141 ///< P_RW_EXPOS  1   exposure time      - now in microseconds?
#define P_VEXPOS         142 ///< video exposure (if 0 - use P_RW_EXPOS in ms)
#define P_FOCUS_VALUE    143 ///< (readonly) - sum of all blocks focus values inside focus WOI

#define P_COMPMOD_BYRSH  144 ///< Bayer shift in compressor
#define P_PORTRAIT       145 ///< Quantization coefficients optimized for vertical scan lines

// 145 (was 143 in NC353) - last to copy ============

//#define P_COMPMOD_BYRSH  160 ///< Bayer shift in compressor
//#define P_PORTRAIT       161 ///< Quantization coefficients optimized for vertical scan lines


//TODO: rearrange, combine with other AUTOEXP
// if  [G_HIST_DIM_01] is set to 0xffffffff that will force re-calibration (1 dark frame)
// The following is a daemons control so the sleeping daemons can be awaken by the drivers when the corresponding bit is set and
// one of the following events happen (depending on driver and lseek (SEEK_END) offset:
// frame interrupt (any - compressed or not) - through "/dev/frameparsall"
// frame compressed interrupt - through "/dev/circbuf"
// histogram Y (G1) is ready - through "/dev/histogram_cache"
// histogram C (R,G1,G2,B) are ready - through "/dev/histogram_cache"

#define P_DAEMON_EN      165 ///< disable all autoexp features AEXP, WB, HDR- make extra sleep for AUTOEXP_EN to become non-zero (normal frame rules)
#define P_AEXP_FRACPIX   166 ///< Fraction of all pixels that should be below P_AEXP_LEVEL (16.16 - 0x10000 - all pixels)
#define P_AEXP_LEVEL     167 ///< Target output level:  [P_AEXP_FRACPIX]/0x10000 of all pixels should have value below it (also 16.16 - 0x10000 - full output scale)

#define P_HDR_DUR        168 ///< 0 - HDR 0ff, >1 - duration of same exposure (currently 1 or 2 - for free running)
#define P_HDR_VEXPOS     169 ///< if less than 0x10000 - number of lines of exposure, >=10000 - relative to "normal" exposure

#define P_AE_PERIOD      170 ///< Autoexposure period (will be increased if below the latency)
#define P_WB_PERIOD      171 ///< White balance period (will be increased if below the latency)
#define P_WB_CTRL        172 ///< bitmask - which colors to correct (1 - correct, 0 - ignore)
#define P_WB_WHITELEV    173 ///< White balance level of white (16.16 - 0x10000 is full scale, 0xfae1 - 98%, default)
#define P_WB_WHITEFRAC   174 ///< White balance fraction (16.16) of all pixels that have level above [P_WB_WHITELEV] for the brightest color
                             ///< locally [P_WB_WHITELEV] will be decreased if needed to satisfy [P_WB_WHITELEV]. default is 1% (0x028f)
#define P_WB_MAXWHITE    175 ///< Maximal allowed "white" pixels fraction (16.16) to have level above [P_WB_WHITELEV] for the darkest color
                             ///< if this limit is exceeded there will be no correction (waiting for autoexposure to decrease overall brightness)

#define P_WB_SCALE_R     176 ///< additional correction for R from calulated by white balance (16.16)
#define P_WB_SCALE_GB    177 ///< additional correction for G2(GB) from calulated by white balance (16.16)
#define P_WB_SCALE_B     178 ///< additional correction for B from calulated by white balance (16.16)

#define P_EXP_AHEAD      179 ///< How many frames ahead of the current frame write exposure to the sensor
#define P_AE_THRESH      180 ///< AE error (logariphmic exposures) is integrated between frame and corrections are scaled when error is below thershold (500)
#define P_WB_THRESH      181 ///< same for WB

/// Used by white balancing to control analog gains in addition to gamma tables. Can be limited to narrower range than
#define P_GAIN_MIN       182 ///< minimal sensor analog gain  0x10000 ~1.0
#define P_GAIN_MAX       183 ///< maximal sensor analog gain  0x10000 ~1.0
#define P_GAIN_CTRL      184 ///< minimal correction to be applied to the analog gain (should be set larger that sensor actual gain step to prevent oscillations (0x100 - 1.0, 0x20 - 1/8)
#define GAIN_BIT_STEP      0 ///< start bit of gain step control in P_GAIN_CTRL
#define GAIN_BIT_ENABLE   16 ///< start bit of enabling analog gain controls in white balancing

/// Requests for autocampars daemon
#define P_AUTOCAMPARS_CTRL 185 ///< bits 0..24 - groups to restore, bits 24..27 - page number to save bits 28..30: 1 - restore, 2 - save, 3 - set default 4 save as default 5 - init

/// Parameters for ccamftp.php daemon
/// (server, account, password, directories,script names are sepaarte)
#define P_FTP_PERIOD     190 ///< FTP image upload period
#define P_FTP_TIMEOUT    191 ///< Maximal time allowed for image uploading
#define P_FTP_UPDATE     192 ///< Maximal time between updates (camera wil re-read remote configuration file)


#define _P_STROP_OFFSET      193 ///< streamer parameters
// multicast/unicast transport
#define P_STROP_MCAST_EN        (_P_STROP_OFFSET + 0)	///< != 0 for multicast, 0 for unicast
#define P_STROP_MCAST_IP        (_P_STROP_OFFSET + 1)	///< 0 for camera IP based multicast, other value - for custom IP
#define P_STROP_MCAST_PORT      (_P_STROP_OFFSET + 2)	///< >= 1024 && <= 65532
#define P_STROP_MCAST_TTL       (_P_STROP_OFFSET + 3)	///< >= 1 && <= 15
// audio - support only one ("default") soundcard, if present
#define P_STROP_AUDIO_EN        (_P_STROP_OFFSET + 4)	///< 0 - disable audio, else - try if soundcard is present
#define P_STROP_AUDIO_RATE      (_P_STROP_OFFSET + 5)	///< >= 11250 && <= 48000
#define P_STROP_AUDIO_CHANNEL   (_P_STROP_OFFSET + 6)	///< 1 - mono, 2 - stereo
#define P_STROP_FRAMES_SKIP     (_P_STROP_OFFSET + 7)	///< 0 - output each frame, 1 - output/skip == 1/1 fomr 2 frames, 2 - output/skip == 1/2 from 3 frames etc
#define P_AUDIO_CAPTURE_VOLUME  (_P_STROP_OFFSET + 8)	///< for streamer and camogm2: 0 == 0%; 65535 == 100% capture volume, by default 90% == 58981

///parameters related to multi-sensor operation
#define _P_MULTISENS_OFFSET      208
#define P_MULTISENS_EN         (_P_MULTISENS_OFFSET + 0)  ///< 0 - single sensor, no 10359A, otherwise - bitmask of the sensors enabled (obeys G_SENS_AVAIL that should not be modified at runtime)
#define P_MULTI_PHASE_SDRAM    (_P_MULTISENS_OFFSET + 1)  ///< similar to P_SENSOR_PHASE, but for sensor1, connected to 10359
#define P_MULTI_PHASE1         (_P_MULTISENS_OFFSET + 2)  ///< similar to P_SENSOR_PHASE, but for sensor1, connected to 10359
#define P_MULTI_PHASE2         (_P_MULTISENS_OFFSET + 3)  ///< similar to P_SENSOR_PHASE, but for sensor2, connected to 10359
#define P_MULTI_PHASE3         (_P_MULTISENS_OFFSET + 4)  ///< similar to P_SENSOR_PHASE, but for sensor3, connected to 10359
#define P_MULTI_SEQUENCE       (_P_MULTISENS_OFFSET + 5)  ///< sensor sequence (bits 0,1 - first, 2,3 - second, 4,5 - third). 0 - disable. Will obey G_SENS_AVAIL & P_MULTISENS_EN
#define P_MULTI_FLIPH          (_P_MULTISENS_OFFSET + 6)  ///< additional per-sensor horizontal flip to global P_FLIPH, same bits as in G_SENS_AVAIL
#define P_MULTI_FLIPV          (_P_MULTISENS_OFFSET + 7)  ///< additional per-sensor vertical flip to global P_FLIPV, same bits as in G_SENS_AVAIL
#define P_MULTI_MODE           (_P_MULTISENS_OFFSET + 8)  ///< Mode 0 - single sensor (first in sequence), 1 - composite (only enabled in triggered mode)
#define P_MULTI_HBLANK         (_P_MULTISENS_OFFSET + 9)  ///< Horizontal blanking for buffered frames (2,3) - not needed?

///parameters for adjusting physical sensors WOI inside the composite frame
#define P_MULTI_CWIDTH         (_P_MULTISENS_OFFSET + 10)  ///< Composite frame width  (stored while in single-sensor mode, copied to P_WOI_WIDTH)
#define P_MULTI_CHEIGHT        (_P_MULTISENS_OFFSET + 11)  ///< Composite frame height (stored while in single-sensor mode)
#define P_MULTI_CLEFT          (_P_MULTISENS_OFFSET + 12)  ///< Composite frame left margin  (stored while in single-sensor mode, copied to P_WOI_LEFT)
#define P_MULTI_CTOP           (_P_MULTISENS_OFFSET + 13)  ///< Composite frame top margin (stored while in single-sensor mode)
#define P_MULTI_CFLIPH         (_P_MULTISENS_OFFSET + 14)  ///< Horizontal flip for composite image (stored while in single-sensor mode)
#define P_MULTI_CFLIPV         (_P_MULTISENS_OFFSET + 15)  ///< Vertical flip for composite image (stored while in single-sensor mode)
#define P_MULTI_VBLANK         (_P_MULTISENS_OFFSET + 16)  ///< Vertical blanking for buffered frames (2,3) BEFORE FRAME, not after

/// should go in that sequence *1,*2,*3 (will be indexed)
#define P_MULTI_WOI            (_P_MULTISENS_OFFSET + 17) ///< Width of frame 1 (direct)  // Same as next
#define P_MULTI_WIDTH1         ( P_MULTI_WOI+          0) ///< Width of frame 1 (direct) // same as P_MULTI_WOI !!!!
#define P_MULTI_WIDTH2         ( P_MULTI_WOI+          1) ///< Width of frame 2 (first buffered)
#define P_MULTI_WIDTH3         ( P_MULTI_WOI+          2) ///< Width of frame 3 (second buffered)
#define P_MULTI_HEIGHT1        ( P_MULTI_WOI+          3) ///< Height of frame 1 (direct)
#define P_MULTI_HEIGHT2        ( P_MULTI_WOI+          4) ///< Height of frame 2 (first buffered)
#define P_MULTI_HEIGHT3        ( P_MULTI_WOI+          5) ///< Height of frame 3 (second buffered)
#define P_MULTI_LEFT1          ( P_MULTI_WOI+          6) ///< Left margin of frame 1 (direct)
#define P_MULTI_LEFT2          ( P_MULTI_WOI+          7) ///< Left margin of frame 2 (first buffered)
#define P_MULTI_LEFT3          ( P_MULTI_WOI+          8) ///< Left margin of frame 3 (second buffered)
#define P_MULTI_TOP1           ( P_MULTI_WOI+          9) ///< Top margin of frame 1 (direct)
#define P_MULTI_TOP2           ( P_MULTI_WOI+         10) ///< Top margin of frame 2 (first buffered)
#define P_MULTI_TOP3           ( P_MULTI_WOI+         11) ///< Top margin of frame 3 (second buffered)
#define P_MULTI_TOPSENSOR      (_P_MULTISENS_OFFSET + 29) ///< number of sensor channel used in first (direct) frame: 0..2, internal parameter (window->sensorin)
#define P_MULTI_SELECTED       (_P_MULTISENS_OFFSET + 30) ///< number of sensor channel (1..3, not 0..2) used when composite mode is disabled
/// parameters for MakerNote, internal
#define P_MULTI_MODE_FLIPS     (_P_MULTISENS_OFFSET + 31) ///< bit 7 - 0, bit 6 - composite mode, bits 4,5 - flips for frame 3, 2,3 - frame 2, 0,1 - frame 1 (top)
#define P_MULTI_HEIGHT_BLANK1  (_P_MULTISENS_OFFSET + 32) ///< height of the first sub-frame plus (vertical blank after the first sub-frame) << 16 (output lines, decimated)
#define P_MULTI_HEIGHT_BLANK2  (_P_MULTISENS_OFFSET + 33) ///< height of the second sub-frame plus (vertical blank after the second sub-frame) << 16 (output lines, decimated)

// up to _P_MULTISENS_OFFSET + 256-208-1=47
/// In 393 these registers go through a static lookup table, that maps them to sensor channel/register (or broadcast/common).
/// Same entries may be dynamically added (sysfs) for "active registers"
#define P_SENSOR_REGS    256 ///< shadows of the sensor registers (should be multiple of 32)
#define P_SENSOR_NUMREGS 256 ///< number of the sensor registers (should be multiple of 32)

#define P_M10359_REGS       (P_SENSOR_REGS + P_SENSOR_NUMREGS) ///< shadows of the 10359 registers (should be multiple of 32)
#define P_M10359_NUMREGS    96 ///< shadows of the 10359  registers (should be multiple of 32)
#define P_M10359_REGS32BIT  0x40 //< 10359 registers above 0x40 are 32-bit ones


#define P_MULTI_NUMREGS  32  ///< up to 32 sensor register may have individual values
#define P_MULTI_REGS          (P_M10359_REGS + P_M10359_NUMREGS) ///< 32-words aligned

#define P_MAX_PAR         (P_MULTI_REGS + (MAX_SENSORS * P_MULTI_NUMREGS )) ///< maximal # of used parameter+1
/* 393: Making P_MAX_PAR multiple of PAGE_SIZE/2, so framepars_all_t will be multiple of PAGE_SIZE*/
#ifndef PAGE_SIZE
	#define PAGE_SIZE 4096 ///< not using <asm/page.h> as this file may be used outside of kernel
#endif
#define P_MAX_PAR_ROUNDUP ROUND_UP (P_MAX_PAR , (PAGE_SIZE/8)) ///< half page in DWORDs

#ifdef NC353 // to hide old code
#ifdef SAFE_CHECK
  #define MULTIREG(x,n) ((((n)>=0) && ((n)<MAX_SENSORS) && ((x) >0) && ((x) < P_MAX_PAR) && (multiSensIndex[x] > 0))? (multiSensIndex[x]+(n)) : 0)
#else
  #define MULTIREG(x,n) ((multiSensIndex[x] > 0)? (multiSensIndex[x]+(n)) : 0) // unsafe
#endif
#define MULTIRVRSREG(x) (multiSensRvrsIndex[x])
#else
#ifdef SAFE_CHECK
  #define MULTIREG(p,x,n) ((((n)>=0) && ((n)<MAX_SENSORS) && ((x) >0) && ((x) < P_MAX_PAR) && (amultiSensIndex[p][x] > 0))? (amultiSensIndex[p][x]+(n)) : 0)
#else
  #define MULTIREG(p,x,n) ((amultiSensIndex[p][x] > 0)? (amultiSensIndex[p][x]+(n)) : 0) // unsafe
#endif
#define MULTIRVRSREG(p,x) (amultiSensRvrsIndex[p][x])
#endif
///amultiSensRvrsIndex
//#define P_MAX_PAR        511 /// maximal # of used parameter
//#define P_MAX_GPAR      1023 // maximal # of global parameter
//#define P_MAX_GPAR      2047 /// maximal # of global parameter - TODO: change name to NUM_GPAR and make it 2048
#define NUM_GPAR        2048           ///< maximal # of global parameter - TODO: change name to NUM_GPAR and make it 2048
#define P_MAX_GPAR      (NUM_GPAR - 1) ///< maximal # of global parameter - TODO: change name to NUM_GPAR and make it 2048

#define PARS_SAVE_FROM   128 ///< PARS_SAVE_NUM parameters starting from PARS_SAVE_FROM from "this" frame will be saved in circular buffer, PASTPARS_SAVE_ENTRIES entries
#define PARS_SAVE_COPY    18 ///< number of parameters copied from future (framepars) to the past (pastpars)
#define PARS_SAVE_NUM     32 ///< total size of previous parameter save page
#define PP_PROFILE_START  PARS_SAVE_COPY ///< index of the first profile timestamp in pastpars
#define P_PROFILE        (PARS_SAVE_FROM + PP_PROFILE_START) ///< index to access profiles as pastpars (i.e. from PHP ELPHEL_PROFILE1,PHP ELPHEL_PROFILE2)

#define FRAMEPAR_GLOBALS   0x01000  ///< start of global (not frame-related) parameters

#define GLOBALS_PRESERVE   0x20     /// number of parameters that are not erased during initGlobalPars

#define GLOBALPARS(p, x)   (aglobalPars[p][(x)-FRAMEPAR_GLOBALS]) ///< should work in drivers and applications, First 32 parameter values are not erased with initGlobalPars
#define GLOBALPARS_SNGL(x) (globalPars[(x)-FRAMEPAR_GLOBALS]) ///< for applications that use just one port

#define G_DEBUG         (FRAMEPAR_GLOBALS + 2) ///< Each bit turns on/off some debug outputs
#define G_TEST_CTL_BITS (FRAMEPAR_GLOBALS + 3) ///< turn some features on/off in the drivers for debugging purposes
#define G_TEST_CTL_BITS_RESET_DMA_COMPRESSOR 0 ///< reset compressor and DMA when detecting sensor, bit number in G_TEST_CTL_BITS

#define G_CABLE_TIM     (FRAMEPAR_GLOBALS + 7) ///< Extra cable delay, signed ps)
#define G_FPGA_TIM0     (FRAMEPAR_GLOBALS + 8) ///< FPGA timing parameter 0 - difference between DCLK pad and DCM input, signed (ps)
#define G_FPGA_TIM1     (FRAMEPAR_GLOBALS + 9) ///< FPGA timing parameter 1

#define G_SENS_AVAIL    (FRAMEPAR_GLOBALS + 10) ///< bitmask of the sensor attached to 10359A (0 - no 10359A brd)
#define G_DLY359_OUT    (FRAMEPAR_GLOBALS + 11) ///< output delay in 10359 board (clock to out) in ps, signed
#define G_DLY359_P1     (FRAMEPAR_GLOBALS + 12) ///< delay in 10359 board sensor port 1 (clock to sensor - clock to DCM) in ps, signed
#define G_DLY359_P2     (FRAMEPAR_GLOBALS + 13) ///< delay in 10359 board sensor port 2 (clock to sensor - clock to DCM) in ps, signed
#define G_DLY359_P3     (FRAMEPAR_GLOBALS + 14) ///< delay in 10359 board sensor port 3 (clock to sensor - clock to DCM) in ps, signed
#define G_DLY359_C1     (FRAMEPAR_GLOBALS + 15) ///< cable delay in sensor port 1 in ps, signed
#define G_DLY359_C2     (FRAMEPAR_GLOBALS + 16) ///< cable delay in sensor port 2 in ps, signed
#define G_DLY359_C3     (FRAMEPAR_GLOBALS + 17) ///< cable delay in sensor port 3 in ps, signed
//gap
#define G_MULTI_CFG     (FRAMEPAR_GLOBALS + 23) ///< Additional configuration options for 10359 board.
  #define G_MULTI_CFG_SYSCLK 0 ///< Bit 0 - use 10353 system clock, not the local one (as on 10359 rev 0)
  #define G_MULTI_CFG_DLYI2C 1 ///< Bit 1 - delay 10359 i2c commands with respect to sensor ones (in multi_pgm_window)
  #define G_MULTI_CFG_BEFORE 2 ///< Bit 2 - send 10359 i2c commands first (to be sent after frame sync, 0 - sesnor commands first)
#define G_MULTI_REGSM   (FRAMEPAR_GLOBALS + 24) ///< 8 words of bitmasks of individual sensor registers, used only at init. Later will be initialized (OR-ed?) by driver

// used up to FRAMEPAR_GLOBALS + 31
#define G_FRAME_SIZE    (FRAMEPAR_GLOBALS + 33) ///< Last compressed frame size in bytes (w/o headers). First 32 entries will not be erased when sensor init, Move some here

#define G_MULTI_NUM     (FRAMEPAR_GLOBALS + 34) ///< Actual number of parameters that are individual for different channels (limited by P_MULTI_NUMREGS)

#define G_MAXAHEAD      (FRAMEPAR_GLOBALS + 35) ///< Maximal number of frames ahead of current to be programmed to hardware
#define G_THIS_FRAME    (FRAMEPAR_GLOBALS + 36) ///< Current frame number (may lag from the hardwaere)
#define G_CIRCBUFSIZE   (FRAMEPAR_GLOBALS + 37) ///< Size of the circular buffer (in bytes)
// the following 3 locations should be in the same 32-byte (8 long) cache line
#define G_FREECIRCBUF   (FRAMEPAR_GLOBALS + 38) ///< Free space in circbuf (uses global read pointer, in bytes)
#define G_CIRCBUFWP     (FRAMEPAR_GLOBALS + 39) ///< circbuf write pointer (in bytes - similar P_JPEG_WP is in long words)
#define G_CIRCBUFRP     (FRAMEPAR_GLOBALS + 40) ///< circbuf global read pointer (in bytes )

#define G_SECONDS       (FRAMEPAR_GLOBALS + 41) ///< seconds (R/W to FPGA timer)
#define G_MICROSECONDS  (FRAMEPAR_GLOBALS + 42) ///< microseconds (R/W to FPGA timer)

/// Next 5 should go in that sequence
#define G_CALLNASAP     (FRAMEPAR_GLOBALS + 43) ///< bitmask - what functions can be used not only in the current frame (ASAP) mode
#define G_CALLNEXT      (FRAMEPAR_GLOBALS + 43) ///< (same as G_CALLNASAP) bitmask of actions to be one   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT2..G_CALLNEXT4)
//Leave 4 locations after for (G_CALLNEXT+1)...(G_CALLNEXT+4)
//#define G_CALLNEXT1     (FRAMEPAR_GLOBALS + 44) /// bitmask of actions to be one   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT2..G_CALLNEXT4)
//#define G_CALLNEXT2     (FRAMEPAR_GLOBALS + 45) /// bitmask of actions to be two   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT3..G_CALLNEXT4)
//#define G_CALLNEXT3     (FRAMEPAR_GLOBALS + 46) // bitmask of actions to be three or more frames ahead of the programmed one (OR-ed with G_CALLNEXT4)
//#define G_CALLNEXT4     (FRAMEPAR_GLOBALS + 47) // bitmask of actions to be four  or more frames ahead of the programmed one

#define G_NEXT_AE_FRAME  (FRAMEPAR_GLOBALS + 48) ///< Next frame to be processed by autoexposure - written directly from daemon through mmap
#define G_NEXT_WB_FRAME  (FRAMEPAR_GLOBALS + 49) ///< Next frame to be processed by white balance - written directly from daemon through mmap
#define G_HIST_DIM_01    (FRAMEPAR_GLOBALS + 50) ///< Percentile measured for colors 0 (lower 16 bits) and 1 (high 16 bits) for  VEXPOS=1
#define G_HIST_DIM_23    (FRAMEPAR_GLOBALS + 51) ///< Percentile measured for colors 2 (lower 16 bits) and 3 (high 16 bits) for  VEXPOS=1
/// if  [G_HIST_DIM_01] is set to 0xffffffff that will force re-calibration (1 dark frame)
//#define G_EW_HYSTCNTR    (FRAMEPAR_GLOBALS + 52) // autoexposure/white balance hysteresis counters , 1 byte each, (sign and count, Y,R,G2,B)
#define G_AE_INTEGERR    (FRAMEPAR_GLOBALS + 52) ///< current integrated error in the AE loop
#define G_WB_INTEGERR    (FRAMEPAR_GLOBALS + 53) ///< current integrated error in WB loop

#define G_TASKLET_CTL    (FRAMEPAR_GLOBALS + 54) ///< disable individual tasklets  (TODO:rename to DEBUG_CTL?)
#define G_GFOCUS_VALUE   (FRAMEPAR_GLOBALS + 55) //< (readonly) - sum of all blocks focus values inside focus WOI (global)

  #define TASKLET_CTL_PGM      0 ///< disable programming parameters (should not be)
  #define TASKLET_CTL_IGNPAST  1 ///< "ignore overdue" control bit
  #define TASKLET_CTL_NOSAME   2 ///< don't try to process parameters immediately after written. If 0, only non-ASAP will be processed to prevent
  #define TASKLET_CTL_ENPROF   3 ///< enable profiling (saving timing of the interrupts/tasklets in pastpars)
                                 ///< effects of uncertainty of when was it called relative to frame sync
  #define TASKLET_HIST_ALL     0   ///< calculate each frame
  #define TASKLET_HIST_HALF    1   ///< calculate each even (0,2,4,6 frme of 8)
  #define TASKLET_HIST_QUATER  2   ///< calculate twice per 8 (0, 4)
// NOTE: It is safer to allow all histograms at least once in 8 frames so applications will not be locked up waiting for the missed histogram
// TODO: detect missing histograms and wakeup queue
  #define TASKLET_HIST_ONCE    3   ///< calculate once  per 8 (0)
  #define TASKLET_HIST_RQONLY  4   ///< calculate only when specifically requested
  #define TASKLET_HIST_NEVER   7   ///< never calculate

  #define TASKLET_CTL_HISTY_BIT  4 ///< shift of histogram calculation for Y in G_TASKLET_CTL (bits 4,5,6)
  #define TASKLET_CTL_HISTC_BIT  8 ///< shift of histogram calculation for C in G_TASKLET_CTL (bits 8,9,10)

//#define G_HIST_LAST_INDEX (FRAMEPAR_GLOBALS + 56) // /< last used index in histogram cache
//#define G_HIST_Y_FRAME    (FRAMEPAR_GLOBALS + 57) // /< last frame for which Y histogram was calculated
//#define G_HIST_C_FRAME    (FRAMEPAR_GLOBALS + 58) // /< last frame for which C histograms were calculated
#define G_SUBCHANNELS     (FRAMEPAR_GLOBALS + 56) ///< subchannels used on this sensor port (bitmask)

#define G_COMPRESSOR_FRAME (FRAMEPAR_GLOBALS + 57) ///< last compressed frame number
//Gap 2 DWORDS

#define G_SKIP_DIFF_FRAME (FRAMEPAR_GLOBALS + 59) ///< number of frames with different size to tolerate before producing POLLHUP in poll(circbuf)
#define G_FTP_NEXT_TIME   (FRAMEPAR_GLOBALS + 60) ///< time of the next FTP upload (seconds from epoch)

#define G_DAEMON_ERR      (FRAMEPAR_GLOBALS + 63) ///< 1 bit per daemon error that needs attention (daemon would put itself to sleep through P_DAEMON_EN)
#define G_DAEMON_RETCODE  (FRAMEPAR_GLOBALS + 64) ///< return codes (32 32-bit words) for daemons, provided with a corresponding bit in G_DAEMON_ERR set
//next will be (FRAMEPAR_GLOBALS + 96)
// temperature is provided by the daemon, embedded in MakerNote by the driver
#define G_TEMPERATURE01   (FRAMEPAR_GLOBALS + 96) ///< temperature on the sensors 0 and 1 (0x1000 - sign, rest 8.4 C) no 10359 - only chn0, with 10359 - only 1,2,3 (no 0)
#define G_TEMPERATURE23   (FRAMEPAR_GLOBALS + 97) ///< temperature on the sensors 2 and 3

// Moving here as there can be multiple histograms per port
#define G_HIST_LAST_INDEX (FRAMEPAR_GLOBALS +  98) ///< last used index in histogram cache (uses 4 locations), set to !=0 to activate, 0 will skip
#define G_HIST_Y_FRAME    (FRAMEPAR_GLOBALS + 102) ///< last frame for which Y histogram was calculated (uses 4 locations)
#define G_HIST_C_FRAME    (FRAMEPAR_GLOBALS + 106) ///< last frame for which C histograms were calculated (uses 4 locations)

#define G_HIST_SHIFT     (FRAMEPAR_GLOBALS + 107) ///< debug feature - add 0..15 to the histogram frame to read DMA memory

#define G_SENSOR_CALIB   (FRAMEPAR_GLOBALS + 1024) ///< 1024 Array of sensor calibration data, sensor dependent.For Micron it is 256*4 actual gains in 8.16 format
                                                   ///< Only first 96 for each color are used



///Modifiers to the parameter numbers/addresses
#define FRAMEPAIR_FORCE_NEW     0x040000000 ///< will mark parameter as "modified" even the new==old
#define FRAMEPAIR_FORCE_PROC    0x080000000 ///< will mark schedule functions for that parameter even if called from setFramePars/setFramePar (normally it is not)
#define FRAMEPAIR_FORCE_NEWPROC 0x0c0000000 ///< combines both
#define FRAMEPAIR_JUST_THIS     0x10000000  ///< write only to this frame, don't propagate
                                            ///< (like "single frame" - compressor, sensor) first write "stop", then - "single" with FRAMEPAIR_JUST_THIS
#define FRAMEPAIR_FRAME_FUNC    0x20000000  //v write to func2call instead of the frame parameters
/// compose bit fields to be OR-ed to the parameter number bit 16..25: b - start bit (0..31), w - width 1..32
#define FRAMEPAIR_FRAME_BITS(w,b) ((((w) & 0x1f)<<21) | (((b) & 0x1f)<<16))
/// Shift new data (nd), apply mask and combine with old data (od), taking shift/width information from bits 16..25 of (a)
#define FRAMEPAIR_FRAME_MASK_NEW(a,od,nd) ((((od) ^ ((nd) << (((a)>>16) & 0x1f))) & (((1 << (((a)>>21) & 0x1f))-1) << (((a)>>16) & 0x1f))) ^ (od))

/// read bit field from the data (d), use bit information encoded in bits 16..25 of the parameter address (a)
#define FRAMEPAIR_FRAME_FIELD(a,d) (((d) >> (((a)>>16) & 0x1f)) &  ((1 << (((a) >> 21) & 0x1f))-1))

#define FRAMEPAIR_MASK_BYTES   (FRAMEPAIR_FRAME_BITS(31,31)) ///< 0x03ff0000 - if parameter address & FRAMEPAIR_FRAME_MASK is nonzero, change only some bits

//NOTE: byte/word masks not to be OR-ed!
#define FRAMEPAIR_BYTE0        (FRAMEPAIR_FRAME_BITS( 8,  0)) ///< overwrite only byte0 (LSB) in the parameter
#define FRAMEPAIR_BYTE1        (FRAMEPAIR_FRAME_BITS( 8,  8)) ///< overwrite only byte1 in the parameter
#define FRAMEPAIR_BYTE2        (FRAMEPAIR_FRAME_BITS( 8, 16)) ///< overwrite only byte2 in the parameter
#define FRAMEPAIR_BYTE3        (FRAMEPAIR_FRAME_BITS( 8, 24)) ///< overwrite only byte3 (MSB) in the parameter
#define FRAMEPAIR_WORD0        (FRAMEPAIR_FRAME_BITS(16,  0)) ///< overwrite only word0 (LSW) in the parameter (i.e. gamma scale)
#define FRAMEPAIR_WORD1        (FRAMEPAIR_FRAME_BITS(16, 16)) ///< overwrite only word1 (MSW) in the parameter (i.e. gamma hash16)

//#define P_DAEMON_EN      165 // disable all autoexp features AEXP, WB, HDR- make extra sleep for AUTOEXP_EN to become non-zero (normal frame rules)
#define DAEMON_BIT_AUTOEXPOSURE 0 ///< Daemon bit to control autoexposure
#define DAEMON_BIT_STREAMER     1 ///< Daemon bit to control streamer
#define DAEMON_BIT_CCAMFTP      2 ///< Daemon bit to control ftp
#define DAEMON_BIT_CAMOGM       3 ///< Daemon bit to control camogm
#define DAEMON_BIT_AUTOCAMPARS  4 ///< Daemon bit to control autocampars
#define DAEMON_BIT_TEMPERATURE  5 ///< Daemon bit to control temperature
// add up to 31

#define P_DAEMON_EN_AUTOEXPOSURE (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_AUTOEXPOSURE)) ///< Enable autoexposure
#define P_DAEMON_EN_STREAMER     (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_STREAMER))     ///< Enable streamer
#define P_DAEMON_EN_CCAMFTP      (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_CCAMFTP))      ///< Enable ftp
#define P_DAEMON_EN_CAMOGM       (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_CAMOGM))       ///< Enable camogm
#define P_DAEMON_EN_AUTOCAMPARS  (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_AUTOCAMPARS))  ///< Enable autocampars
#define P_DAEMON_EN_TEMPERATURE  (P_DAEMON_EN | FRAMEPAIR_FRAME_BITS(1, DAEMON_BIT_TEMPERATURE))  ///< Enable temperature monitgoring

//#define P_GAIN_CTRL       184 // combines GAIN_STEP and ANA_GAIN_ENABLE
#define P_GAIN_STEP      (P_GAIN_CTRL |  FRAMEPAIR_FRAME_BITS(16, GAIN_BIT_STEP))   ///< minimal correction to be applied to the analog gain
                                                                                    ///< (should be set larger that sensor actual gain step to prevent oscillations (0x100 - 1.0, 0x20 - 1/8)
#define P_ANA_GAIN_ENABLE    (P_GAIN_CTRL |  FRAMEPAIR_FRAME_BITS(1,  GAIN_BIT_ENABLE)) ///< enable analog gain adjustment in white balance procedure

#define WB_CTRL_BIT_EN   4 ///< White balance control bit
#define P_WB_MASK        (P_WB_CTRL |  FRAMEPAIR_FRAME_BITS(4,               0)) ///< Colors adjusted in automatic white balance
#define P_WB_EN          (P_WB_CTRL |  FRAMEPAIR_FRAME_BITS(1,  WB_CTRL_BIT_EN)) ///< Enabling/disabling automatic white balance adjustment

#define P_RSCALE         (P_RSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_WIDTH, 0)) ///< Red-to-Green ratio, 0x10000 ~ 1.0
#define P_GSCALE         (P_GSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_WIDTH, 0)) ///<Green2-to-Green ratio, 0x10000 ~ 1.0
#define P_BSCALE         (P_BSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_WIDTH, 0)) ///< Blue-to-Green ratio, 0x10000 ~ 1.0

#define P_RSCALE_CTL     (P_RSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_CTL_WIDTH, CSCALES_CTL_BIT)) ///< Red-to-Green ratio control
#define P_GSCALE_CTL     (P_GSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_CTL_WIDTH, CSCALES_CTL_BIT)) ///< Green2-to-Green ratio control
#define P_BSCALE_CTL     (P_BSCALE_ALL |  FRAMEPAIR_FRAME_BITS(CSCALES_CTL_WIDTH, CSCALES_CTL_BIT)) ///< Blue-to-Green ratio control

#define P_SENSOR_SINGLE      (P_SENSOR_RUN | FRAMEPAIR_JUST_THIS)       ///< is only good to write SENSOR_RUN_SINGLE there!
#define P_COMPRESSOR_SINGLE  (P_COMPRESSOR_RUN | FRAMEPAIR_JUST_THIS)   ///< is only good to write COMPRESSOR_RUN_SINGLE there!


//#define P_HISTRQ        67 // per-frame enabling of histogram calculation - bit 0 - Y (G), bit 2 - C (R,G2,B)
//#define   HISTRQ_BITY    0
//#define   HISTRQ_BITC    1 
#define   HISTRQ_BIT_Y    0
#define   HISTRQ_BIT_C    1 

#define P_HISTRQ_Y  (P_HISTRQ | FRAMEPAIR_FRAME_BITS(1, HISTRQ_BIT_Y) | FRAMEPAIR_JUST_THIS) ///< request calculation of the Y-histogram for just this frame
#define P_HISTRQ_C  (P_HISTRQ | FRAMEPAIR_FRAME_BITS(1, HISTRQ_BIT_C) | FRAMEPAIR_JUST_THIS) ///< request calculation of the C-histogram for just this frame
#define P_HISTRQ_YC (P_HISTRQ | FRAMEPAIR_FRAME_BITS(2, HISTRQ_BIT_Y) | FRAMEPAIR_JUST_THIS) ///< request calculation of both the Y-histogram and C-histogrames for just this frame

#define G_HISTMODE_Y   (G_TASKLET_CTL | FRAMEPAIR_FRAME_BITS(3, TASKLET_CTL_HISTY_BIT)) ///< control for histogram calculation Y (see TASKLET_HIST_*)
#define G_HISTMODE_C   (G_TASKLET_CTL | FRAMEPAIR_FRAME_BITS(3, TASKLET_CTL_HISTC_BIT)) ///< control for histogram calculation Y (see TASKLET_HIST_*)

#define G_PROFILING_EN (G_TASKLET_CTL | FRAMEPAIR_FRAME_BITS(1, TASKLET_CTL_ENPROF)) ///< enable profiling (saving timing of the interrupts/tasklets in pastpars)


//#define P_AUTOCAMPARS_CTRL 184 // bits 0..24 - groups to restore, bits 24..27 - page number to save bits 28..30: 1 - restore, 2 - save, 3 - set default 4 save as default 5 - init
#define P_AUTOCAMPARS_GROUPS (P_AUTOCAMPARS_CTRL | FRAMEPAIR_FRAME_BITS(24, 0)) ///< Autocampars groups (24 bits)
#define P_AUTOCAMPARS_PAGE   (P_AUTOCAMPARS_CTRL | FRAMEPAIR_FRAME_BITS(4, 24)) ///< Autocampars page number (4 bits)
#define P_AUTOCAMPARS_CMD    (P_AUTOCAMPARS_CTRL | FRAMEPAIR_FRAME_BITS(3, 28)) ///< Autocampars command (3 bits)

///NOTE page 0 is write protected, page 15 (0x0f) is "default" page
#define AUTOCAMPARS_CMD_RESTORE  1  ///< Autocampars command: restore specified groups of parameters from the specified page
#define AUTOCAMPARS_CMD_SAVE     2  ///< Autocampars command: save all current parameters to the specified group (page 0 is write-protected)
#define AUTOCAMPARS_CMD_DFLT     3  ///< Autocampars command: make selected page the default one (used at startup), page 0 OK
#define AUTOCAMPARS_CMD_SAVEDFLT 4  ///< Autocampars command: save all current parameters to the specified group (page 0 is write-protected) and make it default (used at startup)
#define AUTOCAMPARS_CMD_INIT     5  ///< Autocampars command: reset sensor/sequencers, restore all parameters from the specified page



/// if defined 1 - will wakeup each frame, regardless of the availability of the histograms
//#define HISTOGRAMS_WAKEUP_ALWAYS 0
#define HISTOGRAMS_WAKEUP_ALWAYS 1

/// Number of frames handled in the buffer (increased to 16 for NC393).
///
/// Making use of FPGA queues of i2c and sequencer commands (up to 14 frames ahead).<br/>
/// The P_* parameters will now be stored in 16 pages (previous frame, this frame (+0), next(+1),...(+14)<br/>
/// Top index (0..15) corresponds to hardware frame counter, parameters are copied after the frame sync/compressor done interrupts.<br/>
/// when the 4-bit counter is combined with the software variable to get the full 32-bit frame number.<br/>
/// Each parameter page includes 927 parameter registers, as well as 97 bit mask ones to speed up updates between frames.<br/>
/// So if no parameters are changed - nothing to be copied from page to page.
#ifndef PARS_FRAMES
    #define PARS_FRAMES                                  16
    #define PARS_FRAMES_MASK     (PARS_FRAMES-1)         ///< Maximal frame number (15 for NC393)
#endif
/// Keeping the same size of past frames storage as in 353:<br/>
///#define PASTPARS_SAVE_ENTRIES       (PARS_FRAMES << 8)     // 2048<br/>
///#define PASTPARS_SAVE_ENTRIES_MASK ((PARS_FRAMES << 8)-1)  // 0x7ff
#define PASTPARS_SAVE_ENTRIES       (PARS_FRAMES << 7)
#define PASTPARS_SAVE_ENTRIES_MASK ((PARS_FRAMES << 7)-1)  ///< Mask for save entry numbers 0x7ff

/** Parameters block, maintained for each frame (0..15 in NC393) of each sensor channel */
struct framepars_t {
        unsigned long pars[927];      ///< parameter values (indexed by P_* constants)
        unsigned long functions;      ///< each bit specifies function to be executed (triggered by some parameters change)
        unsigned long modsince[31];   ///< parameters modified after this frame - each bit corresponds to one element in in par[960] (bit 31 is not used)
        unsigned long modsince32;     ///< parameters modified after this frame super index - non-zero elements in in mod[31]  (bit 31 is not used)
        unsigned long mod[31];        ///< modified parameters - each bit corresponds to one element in in par[960] (bit 31 is not used)
        unsigned long mod32;          ///< super index - non-zero elements in in mod[31]  (bit 31 is not used)
        unsigned long needproc[31];   ///< FIXME: unused ( but total size should be 1024, increase pars[927] if removed)
        unsigned long needproc32;     ///< FIXME: unused ( but total size should be 1024, increase pars[927] if removed)
};




//TODO: rearrange onchage_* - functions will be executed in this sequence (from 0 to 31)
// pgm_memcompressor             uses pgm_memsensor
// pgm_compctl,pgm_comprestart   use pgm_memcompressor
// pgm_limitfps                  uses pgm_memcompressor
// pgm_memcompressor             uses pgm_memsensor
// pgm_compctl,pgm_comprestart   use  pgm_memcompressor
// pgm_limitfps                  uses pgm_memcompressor
// pgm_limitfps                  uses pgm_compmode
// pgm_trigseq                   uses pgm_limitfps
// pgm_sensorin (bayer)          uses pgm_window (flip)
//                               onchange_i2c should be the first after init sensor (even before onchange_sensorphase)
// phase (common part conditionally triggers init) -> init -> i2c -> sensorregs -> afterinit
// reset i2c - only once, when the sensor is reset, it is stopped, by frame numbers go on (and parameters are not erased)
//TODO: onchange_exposure should be after onchange_limitfps

/** @brief Image acquisition functions to be executed on parameter changes, should fit in 32.
 *  Functions are executed starting from LSB.
 *  TODO: Try to make room for some new ones.
 */
enum onchange_functions_t {
  onchange_recalcseq=0,    ///<  0 recalculate sequences/latencies, according to P_SKIP, P_TRIG
  onchange_detectsensor,   ///<  1 detect sensor type, sets sensor structure (capabilities), function pointers
  onchange_sensorphase,    ///<  2  program sensor clock/phase (needs to know maximal clock frequency)
  onchange_i2c,            ///<  3  program i2c
  onchange_sensorregs,     ///<  4  write sensor registers (only changed from outside the driver as they may have different latencies)?
  onchange_initsensor,     ///<  5  resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
  onchange_afterinit,      ///<  6  restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
  onchange_multisens,      ///<  7  chnages related to multiplexed sensors
  onchange_window,         ///<  8  program sensor WOI and mirroring (flipping)
  onchange_window_safe,    ///<  9  program sensor WOI and mirroring (flipping) - lower latency, no bad frames
//  onchange_exposure,       ///<  program exposure
  onchange_gains,          ///< 10 program analog gains
  onchange_triggermode,    ///< 11 program sensor trigger mode
  onchange_sensorin,       ///< 12 program sensor input in FPGA (Bayer, 8/16 bits, ??)
  onchange_sensorstop,     ///< 13 Stop acquisition from the sensor to the FPGA (start has latency of 2)
  onchange_sensorrun,      ///< 14 Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
  onchange_gamma,          ///< 15 program gamma table
  onchange_hist,           ///< 16 program histogram window
  onchange_aexp,           ///< 17 program autoexposure mode
  onchange_quality,        ///< 18 program quantization table(s)
  onchange_memsensor,      ///< 19 program memory channels 0 (sensor->memory) and 1 (memory->FPN)
  onchange_memcompressor,  ///< 20 program memory channel 2 (memory->compressor)
  onchange_limitfps,       ///< 21 check compressor will keep up, limit sensor FPS if needed
  onchange_exposure,       ///< 22 program exposure - NOTE: was just after onchange_window

  onchange_compmode,       ///< 23 program compressor modes (excluding start/stop/single)
  onchange_focusmode,      ///< 24 program focus modes
  onchange_trigseq,        ///< 25 program sequencer (int/ext)
  onchange_irq,            ///< 26 program smart IRQ mode (needs to be on)
  onchange_comprestart,    ///< 27 restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
  onchange_compstop,       ///< 28 stop compressor when changing geometry
  onchange_compctl,        ///< 29 only start/stop/single (after explicitly changed, not when geometry was changed)
  onchange_gammaload,      ///< 30 write gamma tables (should be prepared). Maybe - just last byte, to activate?
  onchange_prescal         ///< 31 change scales for per-color digital gains, apply vignetting correction
//  onchange_sensorregs      /// write sensor registers (only changed from outside the driver as they may have different latencies)?
// add others  - none left, all 32 bits used
};
/// Parameters of the previously acquired frame (subset of)
struct framepars_past_t {
    unsigned long past_pars[PARS_SAVE_NUM]; ///< Array of frame parameters preserved for the future
};

/// All parameter data for a sensor port, including future ones and past. Size should be PAGE_SIZE aligned
struct framepars_all_t {
    struct framepars_t      framePars[PARS_FRAMES];                ///< Future frame parameters
    struct framepars_t      func2call;                             ///< func2call.pars[] - each parameter has a 32-bit mask of what pgm_function to call - other fields not used
     unsigned long          globalPars[NUM_GPAR];                  ///< parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both R/W
    struct framepars_past_t pastPars [PASTPARS_SAVE_ENTRIES];      ///< parameters of previously acquired frames
     unsigned long          multiSensIndex[P_MAX_PAR_ROUNDUP];     ///< indexes of individual sensor register shadows (first of 3) - now for all parameters, not just sensor ones
     unsigned long          multiSensRvrsIndex[P_MAX_PAR_ROUNDUP]; ///< reverse index (to parent) for the multiSensIndex in lower 16 bits, high 16 bits - sensor number
};

/// Key/value pair of parameters
struct frameparspair_t {
        unsigned long num;           ///< parameter index ( as defined by P_* constants) ORed with "force new" (0x10000) - parameter value will be considered a new one
        unsigned long val;           ///< parameter value
};


//framePars errors - change to avoid defined in errno.h?
#define ERR_FRAMEPARS_TOOEARLY 100
#define ERR_FRAMEPARS_TOOLATE  101
#define ERR_FRAMEPARS_BADINDEX 102
#define ERR_PGM_TRYAGAINLATER  103 /// tried to program too early (gamma, ?)


#define FRAMEPARS_SETFRAME     0xff01
#define FRAMEPARS_SETFRAMEREL  0xff02
#define FRAMEPARS_SETLATENCY   0xff03
#define FRAMEPARS_SETFPGATIME  0xff04
#define FRAMEPARS_GETFPGATIME  0xff05

#define FRAME_DEAFAULT_AHEAD   3  // program current frame+3 if not specified


//!for structure mapping:
#define P_AUTOEXP P_AUTOEXP_ON
#define P_AEXPWND P_HISTWND_WIDTH


struct autoexp_t {
	unsigned long on;
	/*
	 * in percents: 1 == 1, 100 == 100
	 */
	unsigned long width;
	unsigned long height;
	unsigned long left;
	unsigned long top;
	/*
	 * start exposure time really not needed...
	 */
	unsigned long exp_max;		/* 100 usec == 1 etc... */
	unsigned long overexp_max;	/* percentages for overexposured pixels - 1% == 100, 5% == 500, 0.02% == 2 etc... */
	/*
	 * changed chema - balance exposition for set percent of pixels in needed index
	 */
	unsigned long s_percent;
	unsigned long s_index;
	/*
	 * return current state
	 */
	unsigned long exp;
	/*
	 * "sleep" settings
	 */
	unsigned long skip_pmin;	/* percent of delta for skip changes: 1% == 100 */
	unsigned long skip_pmax;	/* percent of changes for wait one frame before apply changes: 1% == 100 */
	unsigned long skip_t;		/* time for skip changes: 100 usec == 1 */
};
struct aexp_window_t {
	unsigned long width;
	unsigned long height;
	unsigned long top;
	unsigned long left;
};


struct p_names_t {
  int   value;
  char* name;
};

#define P_NAME_ENTRY(y) { P_##y, #y }
#define G_NAME_ENTRY(y) { G_##y, #y }
#define DEFINE_P_NAMES(x) struct p_names_t x[]= { \
          P_NAME_ENTRY(NUMBER), \
          P_NAME_ENTRY(SENSOR), \
          P_NAME_ENTRY(SENSOR_RUN), \
          P_NAME_ENTRY(SENSOR_SINGLE), \
          P_NAME_ENTRY(ACTUAL_WIDTH), \
          P_NAME_ENTRY(ACTUAL_HEIGHT), \
          P_NAME_ENTRY(BAYER), \
          P_NAME_ENTRY(PERIOD), \
          P_NAME_ENTRY(FP1000SLIM), \
          P_NAME_ENTRY(FRAME), \
          P_NAME_ENTRY(CLK_FPGA), \
          P_NAME_ENTRY(CLK_SENSOR), \
          P_NAME_ENTRY(FPGA_XTRA), \
          P_NAME_ENTRY(TRIG), \
          P_NAME_ENTRY(EXPOS), \
          P_NAME_ENTRY(BGFRAME), \
          P_NAME_ENTRY(PAGE_ACQ), \
          P_NAME_ENTRY(PAGE_READ), \
          P_NAME_ENTRY(OVERLAP), \
          P_NAME_ENTRY(VIRT_KEEP), \
          P_NAME_ENTRY(VIRT_WIDTH), \
          P_NAME_ENTRY(VIRT_HEIGHT), \
          P_NAME_ENTRY(WOI_LEFT), \
          P_NAME_ENTRY(WOI_TOP), \
          P_NAME_ENTRY(WOI_WIDTH), \
          P_NAME_ENTRY(WOI_HEIGHT), \
          P_NAME_ENTRY(FLIPH), \
          P_NAME_ENTRY(FLIPV), \
          P_NAME_ENTRY(FPSFLAGS), \
          P_NAME_ENTRY(DCM_HOR), \
          P_NAME_ENTRY(DCM_VERT), \
          P_NAME_ENTRY(BIN_HOR), \
          P_NAME_ENTRY(BIN_VERT), \
          P_NAME_ENTRY(FPGATEST), \
          P_NAME_ENTRY(TESTSENSOR), \
          P_NAME_ENTRY(COLOR), \
          P_NAME_ENTRY(FRAMESYNC_DLY), \
          P_NAME_ENTRY(PF_HEIGHT), \
          P_NAME_ENTRY(BITS), \
          P_NAME_ENTRY(SHIFTL), \
          P_NAME_ENTRY(FPNS), \
          P_NAME_ENTRY(FPNM), \
          P_NAME_ENTRY(VEXPOS), \
          P_NAME_ENTRY(VIRTTRIG), \
          P_NAME_ENTRY(PERIOD_MIN), \
          P_NAME_ENTRY(PERIOD_MAX), \
          P_NAME_ENTRY(SENSOR_PIXH), \
          P_NAME_ENTRY(SENSOR_PIXV), \
          P_NAME_ENTRY(GAINR), \
          P_NAME_ENTRY(GAING), \
          P_NAME_ENTRY(GAINB), \
          P_NAME_ENTRY(GAINGB), \
          P_NAME_ENTRY(RSCALE_ALL), \
          P_NAME_ENTRY(GSCALE_ALL), \
          P_NAME_ENTRY(BSCALE_ALL), \
          P_NAME_ENTRY(RSCALE), \
          P_NAME_ENTRY(GSCALE), \
          P_NAME_ENTRY(BSCALE), \
          P_NAME_ENTRY(RSCALE_CTL), \
          P_NAME_ENTRY(GSCALE_CTL), \
          P_NAME_ENTRY(BSCALE_CTL), \
          P_NAME_ENTRY(FATZERO), \
          P_NAME_ENTRY(QUALITY), \
          P_NAME_ENTRY(PORTRAIT), \
          P_NAME_ENTRY(FP1000S), \
          P_NAME_ENTRY(SENSOR_WIDTH), \
          P_NAME_ENTRY(SENSOR_HEIGHT), \
          P_NAME_ENTRY(COLOR_SATURATION_BLUE), \
          P_NAME_ENTRY(COLOR_SATURATION_RED), \
          P_NAME_ENTRY(VIGNET_AX), \
          P_NAME_ENTRY(VIGNET_AY), \
          P_NAME_ENTRY(VIGNET_BX), \
          P_NAME_ENTRY(VIGNET_BY), \
          P_NAME_ENTRY(VIGNET_C), \
          P_NAME_ENTRY(VIGNET_SHL), \
          P_NAME_ENTRY(SCALE_ZERO_IN), \
          P_NAME_ENTRY(SCALE_ZERO_OUT), \
          P_NAME_ENTRY(DGAINR), \
          P_NAME_ENTRY(DGAING), \
          P_NAME_ENTRY(DGAINGB), \
          P_NAME_ENTRY(DGAINB), \
          P_NAME_ENTRY(CORING_PAGE), \
          P_NAME_ENTRY(TILES), \
          P_NAME_ENTRY(SENSOR_PHASE), \
          P_NAME_ENTRY(TEMPERATURE_PERIOD), \
          P_NAME_ENTRY(AUTOEXP_ON), \
          P_NAME_ENTRY(HISTWND_RWIDTH), \
          P_NAME_ENTRY(HISTWND_RHEIGHT), \
          P_NAME_ENTRY(HISTWND_RLEFT), \
          P_NAME_ENTRY(HISTWND_RTOP), \
          P_NAME_ENTRY(AUTOEXP_EXP_MAX), \
          P_NAME_ENTRY(AUTOEXP_OVEREXP_MAX), \
          P_NAME_ENTRY(AUTOEXP_S_PERCENT), \
          P_NAME_ENTRY(AUTOEXP_S_INDEX), \
          P_NAME_ENTRY(AUTOEXP_EXP), \
          P_NAME_ENTRY(AUTOEXP_SKIP_PMIN), \
          P_NAME_ENTRY(AUTOEXP_SKIP_PMAX), \
          P_NAME_ENTRY(AUTOEXP_SKIP_T), \
          P_NAME_ENTRY(HISTWND_WIDTH), \
          P_NAME_ENTRY(HISTWND_HEIGHT), \
          P_NAME_ENTRY(HISTWND_TOP), \
          P_NAME_ENTRY(HISTWND_LEFT), \
          P_NAME_ENTRY(FOCUS_SHOW), \
          P_NAME_ENTRY(FOCUS_SHOW1), \
          P_NAME_ENTRY(FOCUS_LEFT), \
          P_NAME_ENTRY(FOCUS_WIDTH), \
          P_NAME_ENTRY(FOCUS_TOP), \
          P_NAME_ENTRY(FOCUS_HEIGHT), \
          P_NAME_ENTRY(FOCUS_TOTWIDTH), \
          P_NAME_ENTRY(FOCUS_FILTER), \
          P_NAME_ENTRY(TRIG_CONDITION), \
          P_NAME_ENTRY(TRIG_DELAY), \
          P_NAME_ENTRY(TRIG_OUT), \
          P_NAME_ENTRY(TRIG_PERIOD), \
          P_NAME_ENTRY(TRIG_BITLENGTH), \
          P_NAME_ENTRY(SKIP_FRAMES), \
          P_NAME_ENTRY(I2C_QPERIOD), \
          P_NAME_ENTRY(I2C_BYTES), \
          P_NAME_ENTRY(IRQ_SMART), \
          P_NAME_ENTRY(EXTERN_TIMESTAMP), \
          P_NAME_ENTRY(OVERSIZE), \
          P_NAME_ENTRY(XMIT_TIMESTAMP), \
          P_NAME_ENTRY(GTAB_R), \
          P_NAME_ENTRY(GTAB_G), \
          P_NAME_ENTRY(GTAB_GB), \
          P_NAME_ENTRY(GTAB_B), \
          P_NAME_ENTRY(CORING_INDEX), \
          P_NAME_ENTRY(RFOCUS_LEFT), \
          P_NAME_ENTRY(RFOCUS_WIDTH), \
          P_NAME_ENTRY(RFOCUS_TOP), \
          P_NAME_ENTRY(RFOCUS_HEIGHT), \
          P_NAME_ENTRY(COMP_BAYER), \
          P_NAME_ENTRY(MEMSENSOR_DLY), \
          P_NAME_ENTRY(SENSOR_IFACE_TIM0), \
          P_NAME_ENTRY(SENSOR_IFACE_TIM1), \
          P_NAME_ENTRY(SENSOR_IFACE_TIM2), \
          P_NAME_ENTRY(SENSOR_IFACE_TIM3), \
          P_NAME_ENTRY(COMPRESSOR_RUN), \
          P_NAME_ENTRY(COMPRESSOR_SINGLE), \
          P_NAME_ENTRY(COMPMOD_BYRSH), \
          P_NAME_ENTRY(COMPMOD_TILSH), \
          P_NAME_ENTRY(COMPMOD_DCSUB), \
          P_NAME_ENTRY(COMPMOD_QTAB), \
          P_NAME_ENTRY(SENSOR_REGS), \
          P_NAME_ENTRY(SENSOR_NUMREGS), \
          P_NAME_ENTRY(M10359_REGS), \
          P_NAME_ENTRY(M10359_NUMREGS), \
          P_NAME_ENTRY(DAEMON_EN), \
          P_NAME_ENTRY(DAEMON_EN_AUTOEXPOSURE), \
          P_NAME_ENTRY(DAEMON_EN_STREAMER), \
          P_NAME_ENTRY(DAEMON_EN_CCAMFTP), \
          P_NAME_ENTRY(DAEMON_EN_CAMOGM), \
          P_NAME_ENTRY(DAEMON_EN_AUTOCAMPARS), \
          P_NAME_ENTRY(DAEMON_EN_TEMPERATURE), \
          P_NAME_ENTRY(AEXP_FRACPIX), \
          P_NAME_ENTRY(AEXP_LEVEL), \
          P_NAME_ENTRY(HDR_DUR), \
          P_NAME_ENTRY(HDR_VEXPOS), \
          P_NAME_ENTRY(EXP_AHEAD), \
          P_NAME_ENTRY(AE_THRESH), \
          P_NAME_ENTRY(WB_THRESH), \
          P_NAME_ENTRY(AE_PERIOD), \
          P_NAME_ENTRY(WB_PERIOD), \
          P_NAME_ENTRY(WB_CTRL), \
          P_NAME_ENTRY(WB_MASK), \
          P_NAME_ENTRY(WB_EN), \
          P_NAME_ENTRY(WB_WHITELEV), \
          P_NAME_ENTRY(WB_WHITEFRAC), \
          P_NAME_ENTRY(WB_MAXWHITE), \
          P_NAME_ENTRY(WB_SCALE_R), \
          P_NAME_ENTRY(WB_SCALE_GB), \
          P_NAME_ENTRY(WB_SCALE_B), \
          P_NAME_ENTRY(HISTRQ), \
          P_NAME_ENTRY(HISTRQ_Y), \
          P_NAME_ENTRY(HISTRQ_C), \
          P_NAME_ENTRY(HISTRQ_YC), \
          P_NAME_ENTRY(PROFILE), \
          P_NAME_ENTRY(GAIN_MIN), \
          P_NAME_ENTRY(GAIN_MAX), \
          P_NAME_ENTRY(GAIN_CTRL), \
          P_NAME_ENTRY(GAIN_STEP), \
          P_NAME_ENTRY(ANA_GAIN_ENABLE), \
          P_NAME_ENTRY(AUTOCAMPARS_CTRL), \
          P_NAME_ENTRY(AUTOCAMPARS_GROUPS), \
          P_NAME_ENTRY(AUTOCAMPARS_PAGE), \
          P_NAME_ENTRY(AUTOCAMPARS_CMD), \
          P_NAME_ENTRY(FTP_PERIOD), \
          P_NAME_ENTRY(FTP_TIMEOUT), \
          P_NAME_ENTRY(FTP_UPDATE), \
          P_NAME_ENTRY(STROP_MCAST_EN), \
          P_NAME_ENTRY(STROP_MCAST_IP), \
          P_NAME_ENTRY(STROP_MCAST_PORT), \
          P_NAME_ENTRY(STROP_MCAST_TTL), \
          P_NAME_ENTRY(STROP_AUDIO_EN), \
          P_NAME_ENTRY(STROP_AUDIO_RATE), \
          P_NAME_ENTRY(STROP_AUDIO_CHANNEL), \
          P_NAME_ENTRY(STROP_FRAMES_SKIP), \
          P_NAME_ENTRY(AUDIO_CAPTURE_VOLUME), \
          P_NAME_ENTRY(MULTISENS_EN), \
          P_NAME_ENTRY(MULTI_PHASE_SDRAM), \
          P_NAME_ENTRY(MULTI_PHASE1), \
          P_NAME_ENTRY(MULTI_PHASE2), \
          P_NAME_ENTRY(MULTI_PHASE3), \
          P_NAME_ENTRY(MULTI_SEQUENCE), \
          P_NAME_ENTRY(MULTI_FLIPH), \
          P_NAME_ENTRY(MULTI_FLIPV), \
          P_NAME_ENTRY(MULTI_MODE), \
          P_NAME_ENTRY(MULTI_HBLANK), \
          P_NAME_ENTRY(MULTI_VBLANK), \
          P_NAME_ENTRY(MULTI_CWIDTH), \
          P_NAME_ENTRY(MULTI_CHEIGHT), \
          P_NAME_ENTRY(MULTI_CLEFT), \
          P_NAME_ENTRY(MULTI_CTOP), \
          P_NAME_ENTRY(MULTI_CFLIPH), \
          P_NAME_ENTRY(MULTI_CFLIPV), \
          P_NAME_ENTRY(MULTI_VBLANK), \
          P_NAME_ENTRY(MULTI_WOI), \
          P_NAME_ENTRY(MULTI_WIDTH1), \
          P_NAME_ENTRY(MULTI_WIDTH2), \
          P_NAME_ENTRY(MULTI_WIDTH3), \
          P_NAME_ENTRY(MULTI_HEIGHT1), \
          P_NAME_ENTRY(MULTI_HEIGHT2), \
          P_NAME_ENTRY(MULTI_HEIGHT3), \
          P_NAME_ENTRY(MULTI_LEFT1), \
          P_NAME_ENTRY(MULTI_LEFT2), \
          P_NAME_ENTRY(MULTI_LEFT3), \
          P_NAME_ENTRY(MULTI_TOP1), \
          P_NAME_ENTRY(MULTI_TOP2), \
          P_NAME_ENTRY(MULTI_TOP3), \
          P_NAME_ENTRY(MULTI_TOPSENSOR), \
          P_NAME_ENTRY(MULTI_SELECTED), \
          P_NAME_ENTRY(MULTI_MODE_FLIPS), \
          P_NAME_ENTRY(MULTI_HEIGHT_BLANK1), \
          P_NAME_ENTRY(MULTI_HEIGHT_BLANK2), \
          G_NAME_ENTRY(DEBUG), \
          G_NAME_ENTRY(TEST_CTL_BITS), \
          G_NAME_ENTRY(CABLE_TIM), \
          G_NAME_ENTRY(FPGA_TIM0), \
          G_NAME_ENTRY(FPGA_TIM1), \
          G_NAME_ENTRY(SENS_AVAIL), \
          G_NAME_ENTRY(DLY359_OUT), \
          G_NAME_ENTRY(DLY359_P1), \
          G_NAME_ENTRY(DLY359_P2), \
          G_NAME_ENTRY(DLY359_P3), \
          G_NAME_ENTRY(DLY359_C1), \
          G_NAME_ENTRY(DLY359_C2), \
          G_NAME_ENTRY(DLY359_C3), \
          G_NAME_ENTRY(MULTI_CFG), \
          G_NAME_ENTRY(MULTI_REGSM), \
          G_NAME_ENTRY(FRAME_SIZE), \
          G_NAME_ENTRY(MULTI_NUM), \
          G_NAME_ENTRY(MAXAHEAD), \
          G_NAME_ENTRY(THIS_FRAME), \
          G_NAME_ENTRY(CIRCBUFSIZE), \
          G_NAME_ENTRY(FREECIRCBUF), \
          G_NAME_ENTRY(CIRCBUFWP), \
          G_NAME_ENTRY(CIRCBUFRP), \
          G_NAME_ENTRY(SECONDS), \
          G_NAME_ENTRY(MICROSECONDS), \
          G_NAME_ENTRY(CALLNASAP), \
          G_NAME_ENTRY(CALLNEXT), \
          G_NAME_ENTRY(NEXT_AE_FRAME), \
          G_NAME_ENTRY(NEXT_WB_FRAME), \
          G_NAME_ENTRY(HIST_DIM_01), \
          G_NAME_ENTRY(HIST_DIM_23), \
          G_NAME_ENTRY(AE_INTEGERR), \
          G_NAME_ENTRY(WB_INTEGERR), \
          G_NAME_ENTRY(TASKLET_CTL), \
          G_NAME_ENTRY(GFOCUS_VALUE), \
          G_NAME_ENTRY(HISTMODE_Y), \
          G_NAME_ENTRY(HISTMODE_C), \
          G_NAME_ENTRY(HIST_LAST_INDEX), \
          G_NAME_ENTRY(HIST_Y_FRAME), \
          G_NAME_ENTRY(HIST_C_FRAME), \
          G_NAME_ENTRY(SUBCHANNELS), \
          G_NAME_ENTRY(COMPRESSOR_FRAME), \
          G_NAME_ENTRY(SKIP_DIFF_FRAME), \
          G_NAME_ENTRY(FTP_NEXT_TIME), \
          G_NAME_ENTRY(DAEMON_ERR), \
          G_NAME_ENTRY(DAEMON_RETCODE), \
          G_NAME_ENTRY(PROFILING_EN), \
          G_NAME_ENTRY(TEMPERATURE01), \
          G_NAME_ENTRY(TEMPERATURE23), \
          G_NAME_ENTRY(SENSOR_CALIB), \
          G_NAME_ENTRY(HIST_SHIFT) \
};

#define ONCHANGE_NAME_ENTRY(y) { onchange_##y, #y }
#define DEFINE_ONCHANGE_NAMES(x) struct p_names_t x[]= { \
          ONCHANGE_NAME_ENTRY(recalcseq), \
          ONCHANGE_NAME_ENTRY(detectsensor), \
          ONCHANGE_NAME_ENTRY(sensorphase), \
          ONCHANGE_NAME_ENTRY(i2c), \
          ONCHANGE_NAME_ENTRY(sensorregs), \
          ONCHANGE_NAME_ENTRY(initsensor), \
          ONCHANGE_NAME_ENTRY(afterinit), \
          ONCHANGE_NAME_ENTRY(window), \
          ONCHANGE_NAME_ENTRY(window_safe), \
          ONCHANGE_NAME_ENTRY(gains), \
          ONCHANGE_NAME_ENTRY(triggermode), \
          ONCHANGE_NAME_ENTRY(sensorin), \
          ONCHANGE_NAME_ENTRY(sensorstop), \
          ONCHANGE_NAME_ENTRY(sensorrun), \
          ONCHANGE_NAME_ENTRY(gamma), \
          ONCHANGE_NAME_ENTRY(hist), \
          ONCHANGE_NAME_ENTRY(aexp), \
          ONCHANGE_NAME_ENTRY(quality), \
          ONCHANGE_NAME_ENTRY(memsensor), \
          ONCHANGE_NAME_ENTRY(memcompressor), \
          ONCHANGE_NAME_ENTRY(limitfps), \
          ONCHANGE_NAME_ENTRY(exposure), \
          ONCHANGE_NAME_ENTRY(compmode), \
          ONCHANGE_NAME_ENTRY(focusmode), \
          ONCHANGE_NAME_ENTRY(trigseq), \
          ONCHANGE_NAME_ENTRY(irq), \
          ONCHANGE_NAME_ENTRY(comprestart), \
          ONCHANGE_NAME_ENTRY(compstop), \
          ONCHANGE_NAME_ENTRY(compctl), \
          ONCHANGE_NAME_ENTRY(gammaload), \
          ONCHANGE_NAME_ENTRY(prescal), \
          ONCHANGE_NAME_ENTRY(multisens) \
};



/* i2c errors */
#ifndef ERR_I2C_SCL_ST0
 #define	ERR_I2C_SCL_ST0		 1
 #define	ERR_I2C_SDA_ST0		 2
 #define	ERR_I2C_SCL_ST1		 4
 #define	ERR_I2C_SDA_ST1		 8
 #define	ERR_I2C_SCL_NOPULLUP 16
 #define	ERR_I2C_SDA_NOPULLUP 32

/* i2c_diagnose called by i2c_start (?) could not find any problems. Try again start */
 #define    ERR_I2C_NOTDETECTED  64
 #define	ERR_I2C_SHORT		 128
 #define	ERR_I2C_BSY		     256
 #define	ERR_I2C_NACK		 512
#endif


/* supported ioctl _IOC_NR's */
#define IO_CCAM_SET_EXT_EXPOSURE  0x06
#define IO_CCAM_MONITOR_SEQ       0x07

//#define IO_CCAM_STOP_DMA	0x08
//#define IO_CCAM_START_DMA	0x09 // just starts DMA - descriptor list should be set eatlier
//#define IO_CCAM_START_RAW	0x0a // Programs DMA descriptor list according to current frame size, FPGA registers and starts DMA

/// MOST ARE OBSOLETE - WILL REMOVE WHEN UPDATING STREAMERS
#define IO_CCAM_JPEG		0x08 ///< JPEG-compressor related commands

#define      JPEG_CMD_RESET       0x00 ///<	Resets pointers - both acquisition and readout
//#define      JPEG_CMD_ARM       0x01 ///< Prepare compressor to read next frame acquired
#define      JPEG_CMD_GET         0x02 ///< Read current page (will return empty (and length==0) if not ready
#define      JPEG_CMD_FORGET      0x03 ///< increment read frame pointer
#define      JPEG_CMD_CATCHUP     0x04 ///< set read pointer to the last acquired (or acquiring if none is acquired yet)
#define      JPEG_CMD_ACQUIRE     0x05 ///< acquire and compress one frame
#define      JPEG_CMD_SAVE_RP     0x06 ///< save read pointer
#define      JPEG_CMD_RESTORE_RP  0x07 ///< restore read pointer
#define      JPEG_CMD_N_DONE      0x08 ///< return 1 if no more frames to be acquired (frame number)
#define      JPEG_CMD_L_DONE      0x09 ///< return 1 if no more frames to be acquired (total length)
#define      JPEG_CMD_START       0x0a ///< start constant compression mode
#define      JPEG_CMD_STOP        0x0b ///< stop constant compression mode (may want to wait for CAMSEQ_DONE)
#define      JPEG_CMD_FRAMES      0x0c ///< returns number of frames in buffer, (re)uilds frames chain
#define      JPEG_CMD_JUST_STOP   0x0d ///< just stop - don't start cycle if was allready off!
#define      JPEG_CMD_DUMP        0x0f ///< printk all static data/tables
#define      JPEG_CMD_RESET0      0x10 ///< same as JPEG_CMD_RESET, but non-zero, to be used from lseek (SEEK_END)

//#define      PROGRAM_SENSOR_0     0x11 ///< programSensor(0) - to be used from lseek (SEEK_END)
//#define      PROGRAM_SENSOR_1     0x12 ///< programSensor(1) - to be used from lseek (SEEK_END)
/// Compressor state now applies only to particular frame
//#define      LSEEK_CAMSEQSTATE    0x13 ///< return camSeqState - to be used from lseek (SEEK_END)
#define      LSEEK_GAMMA_INIT        1 // SEEK_END LSEEK_GAMMA_INIT to initialize all the gamma data structures
#define      LSEEK_GAMMA_ISCURRENT   2 // SEEK_END to check if the selected node(pointed by file pointer) is current - returns 0 if not, otherwise - node index

// parameters for lseek circbuf
#define      LSEEK_CIRC_TORP         1
#define      LSEEK_CIRC_TOWP         2
#define      LSEEK_CIRC_PREV         3
#define      LSEEK_CIRC_NEXT         4
#define      LSEEK_CIRC_LAST         5
#define      LSEEK_CIRC_FIRST        6
#define      LSEEK_CIRC_SCND         7
#define      LSEEK_CIRC_SETP         8
#define      LSEEK_CIRC_VALID        9
#define      LSEEK_CIRC_READY       10
#define      LSEEK_CIRC_WAIT        11
#define      LSEEK_CIRC_FREE        12
#define      LSEEK_CIRC_USED        13
#define      LSEEK_CIRC_STOP_COMPRESSOR 14
#define      LSEEK_CIRC_UTIME       15
#define      LSEEK_CIRC_GETFRAME    16
#define      LSEEK_HUFFMAN_DC0      1
#define      LSEEK_HUFFMAN_AC0      2
#define      LSEEK_HUFFMAN_DC1      3
#define      LSEEK_HUFFMAN_AC1      4
#define      LSEEK_HUFFMAN_FPGATAB  5
#define      LSEEK_HUFFMAN_DEFAULT  6
#define      LSEEK_HUFFMAN_FPGACALC 7
#define      LSEEK_HUFFMAN_FPGAPGM  8

//#define      LSEEK_RESET_SENSOR   0x14 ///< reset sensor and FPGA - next time will reprogram it
//#define      LSEEK_INIT_SENSOR    0x15 ///< initialise SDRAM and sensor if it is not programmed yet (or reset)

#define      LSEEK_GET_FPGA_TIME  0x16 ///< get FPGA timer to G_SECONDS, G_MICROSECONDS
#define      LSEEK_SET_FPGA_TIME  0x17 ///< set FPGA timer to G_SECONDS, G_MICROSECONDS

//#define      LSEEK_FLUSH_CACHE    0x18 // workaround for Axis mmap cache coherency problems - flush all cache (8KB)
#define      LSEEK_AUTOEXP_SET    0x19 ///< set autoexposure parameters
#define      LSEEK_AUTOEXP_GET    0x1a ///< copy window and exposure parameters to autoexp_state
#define      LSEEK_TRIGGER_PGM    0x1b ///< program trigger parameters
#define      LSEEK_I2C_PGM        0x1c ///< program hardware i2c speed/bytes
#define      LSEEK_IRQ_SMART_PGM  0x1d ///< program "smart" irq modes (+1 - wait VACT, +2 - wait dma fifo)
#define      LSEEK_EXTERN_TIMESTAMP_PGM 0x1e ///< 1 - use external timestamps if available
#define      LSEEK_DMA_INIT       0x1f ///< (re-) initialize ETRAX DMA for compressor
#define      LSEEK_DMA_STOP       0x20 ///< STOP ETRAX DMA
#define      LSEEK_DMA_START      0x21 ///< STARTETRAX DMA
#define      LSEEK_COMPRESSOR_RESET 0x22 ///< reset compressor and pointers
#define      LSEEK_INTERRUPT_OFF  0x23 ///< disable camera interrupts
#define      LSEEK_INTERRUPT_ON   0x24 ///< enable camera interrupts

#define      LSEEK_FRAMEPARS_INIT 0x25 ///< reset hardware sequencers, init framepars structure
#define      LSEEK_SENSORPROC     0x26 ///< process modified parameters in frame 0 (to start sensor detection)

#define      LSEEK_FRAME_RESET    0x27 ///< reset absolute frame number to avoid integer overflow

//Histograms related commands
#define      LSEEK_HIST_WAIT_Y    0x28 ///< set histogram waiting for the Y (actually G1) histogram (default after open)
#define      LSEEK_HIST_WAIT_C    0x29 ///< set histogram waiting for the C (actually R, G2, B) histograms to become available - implies G1 too
#define      LSEEK_HIST_REQ_EN    0x2a ///< enable histogram request when reading histogram (safer, but may be not desirable in HDR mode) - default after opening
#define      LSEEK_HIST_REQ_DIS   0x2b ///< disable histogram request when reading histogram - will read latest available relying it is available
#define      LSEEK_HIST_SET_CHN   0x30 ///< ..3F Select channel to wait for (4*port+subchannel)
#define      LSEEK_HIST_NEEDED    0x10000 ///< set histogram "needed" mask - 0x10000..0x1ffff
//#define      LSEEK_HIST_WAIT_AE   0x2a ///< wait for autoexposure enabled

#define      LSEEK_DAEMON_FRAME   0x80 ///<  LSEEK_DAEMON_FRAME+B wait for frame interrupt and corresponding bit (B) in P_DAEMON_EN is set
#define      LSEEK_DAEMON_CIRCBUF 0xa0 ///<  LSEEK_DAEMON_FRAME+B wait for frame compressed interrupt and corresponding bit (B) in P_DAEMON_EN is set
#define      LSEEK_DAEMON_HIST_Y  0xc0 ///<  LSEEK_DAEMON_FRAME+B wait for histogram Y ready and corresponding bit (B) in P_DAEMON_EN is set
#define      LSEEK_DAEMON_HIST_C  0xe0 ///<  LSEEK_DAEMON_FRAME+B wait for all histograms ready and corresponding bit (B) in P_DAEMON_EN is set


#define      LSEEK_FRAME_WAIT_REL 0x100 ///< LSEEK_WAIT_FRAME_REL+N - skip N frames (0<N<256)
#define      LSEEK_FRAME_WAIT_ABS 0x200 ///< LSEEK_WAIT_FRAME_ABS+N - wait absolute frame N

#define      LSEEK_FSDRAM_RESET   0x01 // re-program FSDRAM (to be programmed again when accessed)

#define      JPEG_CTRL_MONOCHROME    0x400
#define      JPEG_CTRL_MONOCHROME_BLOCKED    0x1000
#define      JPEG_CTRL_NOMOSAIC    0x1000

#define IO_CCAM_JPEG_QUALITY	0x09 // Set P_QUALITY

#define IO_CCAM_JPEG_GET_N	0x0a	// get specified number of frames (will add to already asked for if any)
#define IO_CCAM_JPEG_GET_L	0x0b	// get specified length (will stop after frame if got more)

#define IO_CCAM_JPEG_CTRL	0x0c // Write JPEG control word (0x10000 - use header, LSW - other settings)
#define IO_CCAM_DMA		0x0d
  #define CCAM_DMA_CMD_STOP	  0x00
  #define CCAM_DMA_CMD_START  0x01 // just starts DMA - descriptor list should be set eatlier

#define IO_CCAM_CR_MODIFY 0x0e	//(bit number)<<2 | op; op= 0 - nop, 1 - set, 2 - reset, 3 - toggle)
#define IO_CCAM_CR_SHADOW 0x0f

#define IO_CCAM_PINS_WRITE	0x20
#define IO_CCAM_PINS_READ	0x21


#define LSEEK_NAME_ENTRY(y) { LSEEK_##y, #y }
#define DEFINE_LSEEK_NAMES(x) struct p_names_t x[]= { \
          LSEEK_NAME_ENTRY(GAMMA_INIT), \
          LSEEK_NAME_ENTRY(GAMMA_ISCURRENT), \
          LSEEK_NAME_ENTRY(CIRC_TORP), \
          LSEEK_NAME_ENTRY(CIRC_TOWP), \
          LSEEK_NAME_ENTRY(CIRC_PREV), \
          LSEEK_NAME_ENTRY(CIRC_NEXT), \
          LSEEK_NAME_ENTRY(CIRC_LAST), \
          LSEEK_NAME_ENTRY(CIRC_FIRST), \
          LSEEK_NAME_ENTRY(CIRC_SCND), \
          LSEEK_NAME_ENTRY(CIRC_SETP), \
          LSEEK_NAME_ENTRY(CIRC_VALID), \
          LSEEK_NAME_ENTRY(CIRC_READY), \
          LSEEK_NAME_ENTRY(CIRC_WAIT), \
          LSEEK_NAME_ENTRY(CIRC_FREE), \
          LSEEK_NAME_ENTRY(CIRC_USED), \
          LSEEK_NAME_ENTRY(CIRC_GETFRAME), \
          LSEEK_NAME_ENTRY(HUFFMAN_DC0), \
          LSEEK_NAME_ENTRY(HUFFMAN_AC0), \
          LSEEK_NAME_ENTRY(HUFFMAN_DC1), \
          LSEEK_NAME_ENTRY(HUFFMAN_AC1), \
          LSEEK_NAME_ENTRY(HUFFMAN_FPGATAB), \
          LSEEK_NAME_ENTRY(HUFFMAN_DEFAULT), \
          LSEEK_NAME_ENTRY(HUFFMAN_FPGACALC), \
          LSEEK_NAME_ENTRY(HUFFMAN_FPGAPGM), \
          LSEEK_NAME_ENTRY(GET_FPGA_TIME), \
          LSEEK_NAME_ENTRY(SET_FPGA_TIME), \
          LSEEK_NAME_ENTRY(AUTOEXP_SET), \
          LSEEK_NAME_ENTRY(AUTOEXP_GET), \
          LSEEK_NAME_ENTRY(TRIGGER_PGM), \
          LSEEK_NAME_ENTRY(I2C_PGM), \
          LSEEK_NAME_ENTRY(IRQ_SMART_PGM), \
          LSEEK_NAME_ENTRY(EXTERN_TIMESTAMP_PGM), \
          LSEEK_NAME_ENTRY(DMA_INIT), \
          LSEEK_NAME_ENTRY(DMA_STOP), \
          LSEEK_NAME_ENTRY(DMA_START), \
          LSEEK_NAME_ENTRY(COMPRESSOR_RESET), \
          LSEEK_NAME_ENTRY(INTERRUPT_OFF), \
          LSEEK_NAME_ENTRY(INTERRUPT_ON), \
          LSEEK_NAME_ENTRY(FRAMEPARS_INIT), \
          LSEEK_NAME_ENTRY(SENSORPROC), \
          LSEEK_NAME_ENTRY(FRAME_RESET), \
          LSEEK_NAME_ENTRY(HIST_WAIT_Y), \
          LSEEK_NAME_ENTRY(HIST_WAIT_C), \
          LSEEK_NAME_ENTRY(HIST_REQ_EN), \
          LSEEK_NAME_ENTRY(HIST_REQ_DIS), \
          LSEEK_NAME_ENTRY(HIST_NEEDED), \
          LSEEK_NAME_ENTRY(DAEMON_FRAME), \
          LSEEK_NAME_ENTRY(DAEMON_CIRCBUF), \
          LSEEK_NAME_ENTRY(DAEMON_HIST_Y), \
          LSEEK_NAME_ENTRY(DAEMON_HIST_C), \
          LSEEK_NAME_ENTRY(FRAME_WAIT_REL), \
          LSEEK_NAME_ENTRY(FRAME_WAIT_ABS), \
          LSEEK_NAME_ENTRY(FSDRAM_RESET) \
};

#define DMA_CHUNK 0x4000 // 32-bit words - may increase??
#define CCAM_BYTES_PER_CHUNK  (1<<16)  /* dma buffer bytes per descriptor */
#define CCAM_DESCR_PER_CHUNK  1
/* If CCAM_CHUNK_PER_DMABUF is 100 then buffer is about 6.5 million bytes
*       which is probably bigger than any single image we'll generate,
*       and is also bigger than raw data (which is 5.5MB).
*       However, images bigger than 6.5/2 == 3.25 million bytes will
*       not be able to be double-buffered and thus will slow things down.
*/
// increasing 3 times - about 20MB
//#define CCAM_CHUNK_PER_DMABUF 102  /* no. of 64Kbyte chunks per buffer */
#define CCAM_CHUNK_PER_DMABUF 302  /* no. of 64Kbyte chunks per buffer */
#define CCAM_WORDS_PER_DMABUF (CCAM_CHUNK_PER_DMABUF<<14) /*32bit words...*/
#define CCAM_BYTES_PER_DMABUF (CCAM_CHUNK_PER_DMABUF<<16)
/*  For past compatibility, CCMA_DMA_SIZE...
*/
//#define CCAM_DMA_SIZE  CCAM_WORDS_PER_DMABUF
#define CCAM_DMA_SIZE        0x4000000     ///< Each channel buffer size  in BYTES (was in DWORDS in NC353) TODO NC393: use only for initial allocation, move to DT
#define CIRCBUF_START_OFFSET 0x100000      ///< Offset for the first bufer TODO NC393: use only for initial allocation, move to DT

/*
*       CCAM_MMAP_OFFSET... -- offsets in bytes in memory mapped region.
*       CCAM_MMAP_SIZE -- no. of bytes to mmap.
*/
// CCAM_DMA1_SIZE should be 2^N 
#ifdef NC353
#define CCAM_CHUNK_PER_DMA1BUF 16  /* no. of 64Kbyte chunks per buffer */
#define CCAM_WORDS_PER_DMA1BUF (CCAM_CHUNK_PER_DMA1BUF<<14) /*32bit words...*/
#define CCAM_BYTES_PER_DMA1BUF (CCAM_CHUNK_PER_DMA1BUF<<16)
#define CCAM_DMA1_SIZE CCAM_WORDS_PER_DMA1BUF
#endif


#define CCAM_MMAP_OFFSET_MMAP_HEADER 0
#define CCAM_MMAP_OFFSET_JPEG_HEADER (PAGE_SIZE)
#define CCAM_MMAP_OFFSET_DMABUF (4*PAGE_SIZE)
#define CCAM_MMAP_SIZE (PAGE_SIZE*sizeof(long)+CCAM_BYTES_PER_DMABUF)

#define CCAM_MMAP_META 12 // extra bytes included at the end of each frame (last aligned to 32 bytes)
#define CCAM_MMAP_META_LENGTH 4 // displacement to length frame length data from the end of the 32-byte aligned frame slot
#define CCAM_MMAP_META_USEC 8 // (negative) displacement to USEC data - 20 bits (frame timestamp)
#define CCAM_MMAP_META_SEC 12 // (negative) displacement to SEC data - 32 bits (frame timestamp)

//!Moved from cc353.c
//#define CX313_FPGA_TABLES_SIZE 0xC00 //=3000 bytes, 32-bit wide, LSB first, some tables use less
#define CX313_FPGA_TABLES_SIZE 0xE00 //=3800 bytes, 32-bit wide, LSB first, some tables use less
#define CX313_FPGA_TABLES_QUANT 0x0   // 0x200 words quantization (8*64*2/2)
#define CX313_FPGA_TABLES_HUFF  0x200   // 0x200 words Huffman table
#define CX313_FPGA_TABLES_GAMMA 0x400 // 0x400 words gamma-correction (or arbitrary table)
#define CX313_FPGA_TABLES_FOCUS 0x800 // 15*64 16-bit words (now high 16 bits are unused)
#define CX313_FPGA_TABLES_FOCUSPARS 0xbc0 //block of focus-related parameters: left[11:0],right[11:0],top[11:0],bottom[11:0],full_width[11:0],filter_sel[3:0]
#define CX313_FPGA_TABLES_CORING 0xC00 // 8*2*256*4 bits - 8 table pairs (Y/C), each having256 entries for 4.4 DCT coeficients, 4 bit output


// make this structure common for sensors, add fields as needed
struct sensor_t {
// sensor constants
   unsigned long  imageWidth;     ///< nominal image width for final images
   unsigned long  imageHeight;    ///< nominal image height for final images
   unsigned long  clearWidth;     ///< maximal clear (useful) image width
   unsigned long  clearHeight;    ///< maximal clear (useful) image height;
   unsigned long  clearTop;       ///< top margin to the first clear pixel
   unsigned long  clearLeft;      ///< left margin to the first clear pixel
   unsigned long  arrayWidth;     ///< total image array width (including black and boundary)
   unsigned long  arrayHeight;    ///< total image array height (including black and boundary)
   unsigned long  minWidth;       ///< minimal WOI width
   unsigned long  minHeight;      ///< minimal WOI height
   unsigned long  minHorBlank;    ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
   unsigned long  minLineDur;     ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
   unsigned long  maxHorBlank;    ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
   unsigned long  minVertBlank;   ///< minimal vertical blanking
   unsigned long  maxVertBlank;   ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
   unsigned long  maxShutter;     ///< Maximal shutter duration (in lines)
   unsigned long  flips;          ///< capabilities: bit mask bit 0 - flipX, 1 - flipY
   unsigned long  init_flips;     ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY. will be XOR-ed with [P_FLIP] to get sensor flip
   unsigned long  bayer;          ///< bayer shift for flips==0
   unsigned long  dcmHor;         ///< bit mask bit 0 - 1:1, bit 31 - by 32
   unsigned long  dcmVert;        ///< bit mask bit 0 - 1:1, bit 31 - by 32
   unsigned long  binHor;         ///< bit mask bit 0 - 1:1, bit 31 - by 32
   unsigned long  binVert;        ///< bit mask bit 0 - 1:1, bit 31 - by 32
   unsigned long  maxGain256;     ///< maximal analog gain times 0x100
   unsigned long  minGain256;     ///< minimal analog gain (that allows saturation of all but defective pixels) times 0x100
   unsigned long  minClockFreq;   ///< Minimal clock frequency
   unsigned long  maxClockFreq;   ///< Maximal clock frequency
   unsigned long  nomClockFreq;   ///<nominal clock frequency
   unsigned long  sensorType;     ///< sensor type (for Elphel cameras)
   unsigned long  i2c_addr;       ///< i2c address
   unsigned long  i2c_period;     ///< SCL period in ns, (standard i2c - 2500)
   unsigned long  i2c_bytes;      ///< number of bytes/ register
            short hact_delay;     ///< hact delay (in ps) from data
            short sensorDelay;    ///< Delay from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
   unsigned long  needReset;      ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};
#define SENSOR_NEED_RESET_CLK   1
#define SENSOR_NEED_RESET_PHASE 2

struct sensorproc_t {
     struct sensor_t sensor;
// functions return <0 on error (and do nothing)
// first 32 functions are called directly when appropriate bit is set, next 32 - sensor specific that are called
// by corresponding one with (number-32) if not NULL. Sensor initilaization should set up those functions
     int (*pgm_func[64]) (int                  sensor_port, ///< sensor port number (0..3)
    		              struct sensor_t    * sensor,      ///< pointer to sensor parameters
                          struct framepars_t * framepars,   ///< pointer to structure with array of current frame parameters
                          struct framepars_t * prevpars,    ///< pointer to structure with array of previous frame parameters
                          int                  frame16);    ///< frame to program (latency applied), -1 - ASAP
};


/****************************************************************************************************
*! This is essential data related to the last frame aquired to be stored in the circular buffer before
*! each frame received from the FPGA - place where FPGA data is padded by 32 bytes of 0.
*! 6 bytes are already used by next frame pointer signature, so only 26 bytes are left
*! Structure also includes 8 bytes of timestamp (after 2 bytes skipped) - they will be obtained from the circbuf data
*! that goes after the encoded frame, so total is 36 bytes (26+2+8)
!****************************************************************************************************/
// move fram x353.h 
#define DEFAULT_COLOR_SATURATION_BLUE 0x90 ///< 100*relative saturation blue
#define DEFAULT_COLOR_SATURATION_RED  0xb6 ///< 100*relative saturation red

//#define EXPOSURE_UNIT 100 // to move to finer exposure settings - current unit in microseconds. TODO: Propagate it to drivers...
#define EXPOSURE_UNIT 1 ///< to move to finer exposure settings - current unit in microseconds. TODO: Propagate it to drivers...

typedef struct{
    unsigned long usec;
    unsigned long sec;
}  sec_usec_t;

/// width,height, quality are still needed even with new Exif - it is used to rebuild JPEG header

// most parameters are moved out, but width, height, quality are needed for JPEG header, so currently the following are used:
/// width
/// height
/// quality
/// meta_index
/// signffff
/// timestamp_sec (camogm only, exif is separate)
/// timestamp_usec (camogm only, exif is separate)
struct interframe_params_t {
/// This data will survive as long as the frame itself in the circular buffer. Some other fields (like exposure) are stored in Exif
/// dont move - should partially match P_* area
   union{unsigned long hash32_r; struct{unsigned short scale_r; union {unsigned short hash16_r; struct{unsigned char gamma_r; unsigned char black_r; };};};}; ///00-03
   union{unsigned long hash32_g; struct{unsigned short scale_g; union {unsigned short hash16_g; struct{unsigned char gamma_g; unsigned char black_g; };};};}; ///04-07
   union{unsigned long hash32_gb;struct{unsigned short scale_gb;union {unsigned short hash16_gb;struct{unsigned char gamma_gb;unsigned char black_gb;};};};}; ///08-11
   union{unsigned long hash32_b; struct{unsigned short scale_b; union {unsigned short hash16_b; struct{unsigned char gamma_b; unsigned char black_b; };};};}; ///12-15
   unsigned short quality2;       /// Quality is represented by 2-byte value. Each byte uses Y table if the value is Q<128,// 16-17
                                  /// and C table with (Q-128) if it is Q>=128.
                                  /// If the High byte is zero, it is treated as Q^0x80  (Q|=(Q^0x80)<<8) for compatibility
                                  /// with a standard single-byte Q value
/// updated in 8.0.8.37 - bit 7 in quality2 means "portrait mode"
   unsigned char  color;          /// color mode //18
   unsigned char  byrshift;       /// bayer shift in compressor //19
   unsigned short width;          /// frame width, pixels   20-21 - NOTE: should be 20-21
   unsigned short height;         /// frame height, pixels  22-23

/*24   *//// unsigned char  bindec_hor;     //! ((bh-1) << 4) | (dh-1) & 0xf (binning/decimation horizontal, 1..16 for each)
/*25   *//// unsigned char  bindec_vert;    //! ((bv-1) << 4) | (dv-1) & 0xf (binning/decimation vertical  , 1..16 for each)
/*24-25*/ unsigned short meta_index;     //! index of the linked meta page

/*26-27*/ unsigned short signffff;       //! should be 0xffff - it will be a signature that JPEG data was not overwritten,
                                         //! JPEG bitstream can not have two 0xff after each other
    union {
/*28-31*/ unsigned long  timestamp_sec ; //! number of seconds since 1970 till the start of the frame exposure
/*28-31*/ unsigned long  frame_length ;  //! JPEG frame length in circular buffer, bytes
          };
/*32-35*/ unsigned long  timestamp_usec; //! number of microseconds to add
};

struct i2c_timing_t {
        unsigned char scl_high; //0x02, //! SCL high:
        unsigned char scl_low;  //0x02, //! SCL low:
        unsigned char slave2master; //0x01, //! slave -> master
        unsigned char master2slave; //0x01, //! master -> slave
        unsigned char filter_sda;   //0x07, //! filter SDA read data by testing multiple times - currently just zero/non zero
        unsigned char filter_scl;  //0x07};//! filter SCL read data by testing multiple times - currently just zero/non zero 
};

/// Gamma data for one component, including direct and reverse tables, hash (i.e. black level+gamma) and links for caching
#define GAMMA_CACHE_NUMBER 256 // number of gamma-tables cached
#define GAMMA_VALID_MASK    1  // table is calculated, matches given hash/scale
#define GAMMA_VALID_REVERSE 2  // reverse table is calculated for the given hash/scale
#define GAMMA_FPGA_MASK     4  // gamma-table encoded for the FPGA is valid
//#define GAMMA_LOCK_MASK     8  // table is locked until programmed to FPGA - now locked is a separate member

// bits passed in int mode
#define GAMMA_MODE_NOT_NICE     1  // if set, no interrupts will be enabled between steps, whole operation is atomic
#define GAMMA_MODE_NEED_REVERSE 2  // reverse gamma table is needed
#define GAMMA_MODE_HARDWARE     4  // the table is needed to program FPGA: fpga-encoded table will be calculated (if not yet),
#define GAMMA_MODE_LOCK         8  // Lock the table for the specified color (used from irq/tasklet - it is needed because all 4 tables in FPGA have to be overwritten at once)

#define GAMMA_SCALE_SHIFT     10  // when scaling - shift right by GAMMA_SCALE_SHIFT (treat scale as 6.10)
#define GAMMA_SCLALE_1 ( 1 << GAMMA_SCALE_SHIFT )      // gamma scale 1.0 - 0x400
struct gamma_stuct_t {
    union {
        unsigned long hash32; /// fully identifies current table
        struct {
            unsigned short scale; /// 6.10: 0x400 is 1.0 scale (>=1.0) is applied in the driver, saturating result
            union {
                unsigned short hash16; /// scale-independent part of the table (tables themselves are calculated outside of the driver)
                struct {
                    unsigned char gamma; /// "gamma" in the range 0.0 ... 2.55
                    unsigned char black; /// black level to subtract (scaled to full scale) from the input data
                };
            };
        };
    };
    unsigned long long valid;      /// 0 - table invalid, 1 - table valid +2 for table locked (until sent to FPGA)
    //          int locked;     /// bit frame+ (color<<3) locked for color/frame
    unsigned long long locked;     /// NOTE: Changed to just color locked for color
    int this_non_scaled;      // 0 for non-scaled, others - (for scaled) - pointer to the corresponding non-scaled
    union { /// used in head (element 0) and non-scaled chain (not used in scaled)
        struct { /// element 0 - heads of the chains
            int oldest_non_scaled; //
            int newest_non_scaled; //
        };
        struct { /// non-scaled (gamma data is full 16-bit)
            int newer_non_scaled; // table type (non-scaled prototype) used later than this one
            int older_non_scaled; // table type (non-scaled prototype) used before this one
        };
    };
    union {  /// used in head (element 0) and scaled chain (not used in non-scaled) (or maybe it will be used?)
        struct { /// element 0 - heads of the chains
            int oldest_all;    //
            int newest_all;    //
        };
        struct { /// scaled (gamma data is hardware defined 10 bit)
            int newer_all;    /// newer in a single  chain of all scaled tables, regardless of the prototype
            int older_all;    /// older in a single  chain of all scaled tables, regardless of the prototype
            /// *_all also includes yet unused nodes (after init)
        };
    };
    union { /// used in non-scaled and scaled, not in the head (element 0)
        struct { /// non-scaled
            int oldest_scaled;    // oldest derivative of this prototype (scaled)
            int newest_scaled;    // newest derivative of this prototype (scaled)
        };
        struct { /// scaled (gamma data is hardware defined 10 bit)
            int newer_scaled; // table type (non-scaled prototype) used later than this one
            int older_scaled; // table type (non-scaled prototype) used before this one
        };
        struct { /// reuse in the head (element 0) - to make this variable visible through mmap to PHP (for debugging)
            int non_scaled_length; // current number of different hash values
            int num_locked;        // number of nodes locked (until table sent to FPGA)
        };
    };
    union {
        struct {
            unsigned short direct[257];   // "Gamma" table, 16-bit for both non-scaled prototypes and scaled, 0..0xffff range (hardware will use less)
            unsigned short dummy1;        // to have it 32-bit aligned
            unsigned char reverse[256];  /// reverse table to speed-up reversing. No division, but needs interpolation by the application
            unsigned long fpga[256]; // data encoded for FPGA "gamma" table (18 bits, "floating point")
        };
        struct {
            int locked_chn_color[4*MAX_SENSORS*SENSOR_PORTS]; /// NOTE: Changed to just color (locked last written to FPGA - maybe needed again, as the FPGA needs all table to be overwritten - two pages)
            // For NC393 - using 64 entries - individual for each channel/subchannel, color is in 2 lower bits
            // int other [129+64+256 -(4 * PARS_FRAMES)];
            int other [129+64+256 -4*MAX_SENSORS*SENSOR_PORTS];
        };
    };
};

///histograms related structure
#define HISTOGRAM_CACHE_NUMBER 8 // 16   // Was 8 // number of frames histograms are kept after acquisition (should be 2^n)
#define COLOR_RED            0
#define COLOR_GREEN1         1
#define COLOR_GREEN2         2 
#define COLOR_BLUE           3


#define COLOR_Y_NUMBER      COLOR_GREEN1 // green1 (index=1) is used as Y color (i.e. for auto exposure). Histogram for is calculated first (or only)
/*
/// the following 8 values should go in the same sequence as fields in the histogram page
#define P_FRAME          136 // Frame number (reset with JPEG pointers) -(read only)
#define P_GAINR          137 // R channel gain  8.16 (0x10000 - 1.0). Combines both analog gain and digital scaling
#define P_GAING          138 // G channel gain ("red line")
#define P_GAINGB         139 // G channel gain ("blue line")
#define P_GAINB          140 // B channel gain
#define P_EXPOS          141 //P_RW_EXPOS  1   exposure time      - now in microseconds?
#define P_VEXPOS         142 // video exposure (if 0 - use P_RW_EXPOS in ms)
#define P_FOCUS_VALUE    143 // (readonly) - sum of all blocks focus values inside focus WOI

*/

#define HISTOGRAM_TABLE_OFFSET 52 /// Histogram tables data starts 44 bytes from the histogram page structure (for PHP raw histogram)
///TODO: Update when histogram_stuct_t is changed
struct histogram_stuct_t {
   unsigned long frame;                /// frame number corresponding to the current histogram
/// Color gains for the frame of the histogram
   union {
       unsigned long gains[4];
     struct {
       unsigned long gain_r;
       unsigned long gain_g;
       unsigned long gain_gb;
       unsigned long gain_b;
     };
   };
   unsigned long expos;                /// Exposure time (usec) for the frame of the histogram
   unsigned long vexpos;               /// number of exposure lines for the frame of the histogram
   unsigned long focus;                /// sum of all blocks focus values inside focus WOI
   unsigned long valid;                /// bit mask of valid arrays (0 - hist_r, ... ,4-cumul_hist_r, ...,  11 - percentile_b)

/// Gamma tables hash values for the frame of the histogram
   union {
       unsigned long gtab[4];
     struct {
       unsigned long gtab_r;
       unsigned long gtab_g;
       unsigned long gtab_gb;
       unsigned long gtab_b;
     };
   };
/// Direct histograms, loaded from the FPGA
   union {
     unsigned long hist[1024] ;          /// All 4 histograms
     struct {
       unsigned long hist_r [256] ;        /// Histogram for the red component
       unsigned long hist_g [256] ;        /// Histogram for the first green component (in the "red" line)
       unsigned long hist_gb[256] ;        /// Histogram for the second green component (in the "blue" line)
       unsigned long hist_b [256] ;        /// Histogram for blue component
     };
   };
/// Direct cumulative histograms, calculated from the loaded from the FPGA
   union {
     unsigned long cumul_hist[1024] ;          /// All 4 cumulative histograms
     struct {
       unsigned long cumul_hist_r [256] ;    /// Cumulative histogram for the red component
       unsigned long cumul_hist_g [256] ;    /// Cumulative histogram for the first green component (in the "red" line)
       unsigned long cumul_hist_gb[256] ;    /// Cumulative histogram for the second green component (in the "blue" line)
       unsigned long cumul_hist_b [256] ;    /// Cumulative histogram for blue component
     };
   };
/// Calculated reverse cumulative histograms (~percentiles) - for the given 1 byte input X (0 - 1/256 of all pixels, ..., 255 - all pixels)
/// returns threshold value P (0..255), so that number of pixels with value less than x is less or equal to (P/256)*total_number_of_pixels,
/// and number of pixels with value less than (x+1) is greater than (P/256)*total_number_of_pixels,
/// P(0)=0, P(256)=256 /not included in the table/
/// percentiles arrays are calculated without division for each element, interpolation (with division) will be done only for the value of interest
/// on demand, in the user space.
/// NOTE: - argument is _output_ value (after gamma-correction), reverse gamma table is needed to relate percentiles to amount of light (proportional to exposure)
   union {
     unsigned char percentile[1024] ;     /// All 4 percentiles
     struct {
       unsigned char percentile_r [256] ; /// percentile for the red component
       unsigned char percentile_g [256] ; /// percentile for the first green component (in the "red" line)
       unsigned char percentile_gb[256] ; /// percentile for the second green component (in the "blue" line)
       unsigned char percentile_b [256] ; /// percentile for the blue component
     };
   };

};

/// Used to provide encoded huffman tables. Those tables (4 of them) will be used in the output JPEG/JP4 files
/// and whill be programmed to the FPGA
struct huffman_encoded_t {
  unsigned char bits[16];     /// number of symbols with length k+1
  unsigned char huffval[256]; /// The symbols, in order of incr code length
};

/// All other integer constants exported to PHP space (C:"CONSTANT" -> PHP:"ELPHEL_CONST_CONSTANT)
#define CONST_NAME_ENTRY(y) { y, #y }
#define DEFINE_CONST_NAMES(x) struct p_names_t x[]= { \
    CONST_NAME_ENTRY(SENSOR_RUN_STOP), \
    CONST_NAME_ENTRY(SENSOR_RUN_SINGLE), \
    CONST_NAME_ENTRY(SENSOR_RUN_CONT), \
    CONST_NAME_ENTRY(COMPRESSOR_RUN_STOP), \
    CONST_NAME_ENTRY(COMPRESSOR_RUN_SINGLE), \
    CONST_NAME_ENTRY(COMPRESSOR_RUN_CONT), \
    CONST_NAME_ENTRY(TASKLET_CTL_PGM), \
    CONST_NAME_ENTRY(TASKLET_CTL_IGNPAST), \
    CONST_NAME_ENTRY(TASKLET_CTL_NOSAME), \
    CONST_NAME_ENTRY(COLORMODE_MONO6), \
    CONST_NAME_ENTRY(COLORMODE_COLOR), \
    CONST_NAME_ENTRY(COLORMODE_JP46), \
    CONST_NAME_ENTRY(COLORMODE_JP46DC), \
    CONST_NAME_ENTRY(COLORMODE_COLOR20), \
    CONST_NAME_ENTRY(COLORMODE_JP4), \
    CONST_NAME_ENTRY(COLORMODE_JP4DC), \
    CONST_NAME_ENTRY(COLORMODE_JP4DIFF), \
    CONST_NAME_ENTRY(COLORMODE_JP4HDR), \
    CONST_NAME_ENTRY(COLORMODE_JP4DIFF2), \
    CONST_NAME_ENTRY(COLORMODE_JP4HDR2), \
    CONST_NAME_ENTRY(COLORMODE_MONO4), \
    CONST_NAME_ENTRY(PARS_FRAMES), \
    CONST_NAME_ENTRY(PARS_FRAMES_MASK), \
    CONST_NAME_ENTRY(PASTPARS_SAVE_ENTRIES), \
    CONST_NAME_ENTRY(PASTPARS_SAVE_ENTRIES_MASK), \
    CONST_NAME_ENTRY(FRAMEPAIR_FORCE_NEW), \
    CONST_NAME_ENTRY(FRAMEPAIR_FORCE_PROC), \
    CONST_NAME_ENTRY(FRAMEPAIR_FORCE_NEWPROC), \
    CONST_NAME_ENTRY(FRAMEPAIR_JUST_THIS), \
    CONST_NAME_ENTRY(FRAMEPAR_GLOBALS), \
    CONST_NAME_ENTRY(FRAMEPAIR_FRAME_FUNC), \
    CONST_NAME_ENTRY(FRAMEPAIR_MASK_BYTES), \
    CONST_NAME_ENTRY(FRAMEPAIR_BYTE0), \
    CONST_NAME_ENTRY(FRAMEPAIR_BYTE1), \
    CONST_NAME_ENTRY(FRAMEPAIR_BYTE2), \
    CONST_NAME_ENTRY(FRAMEPAIR_BYTE3), \
    CONST_NAME_ENTRY(FRAMEPAIR_WORD0), \
    CONST_NAME_ENTRY(FRAMEPAIR_WORD1), \
    CONST_NAME_ENTRY(ERR_FRAMEPARS_TOOEARLY), \
    CONST_NAME_ENTRY(ERR_FRAMEPARS_TOOLATE), \
    CONST_NAME_ENTRY(ERR_FRAMEPARS_BADINDEX), \
    CONST_NAME_ENTRY(DAEMON_BIT_AUTOEXPOSURE), \
    CONST_NAME_ENTRY(DAEMON_BIT_STREAMER), \
    CONST_NAME_ENTRY(DAEMON_BIT_CCAMFTP), \
    CONST_NAME_ENTRY(DAEMON_BIT_CAMOGM), \
    CONST_NAME_ENTRY(DAEMON_BIT_AUTOCAMPARS), \
    CONST_NAME_ENTRY(ERR_PGM_TRYAGAINLATER), \
    CONST_NAME_ENTRY(FRAMEPARS_SETFRAME), \
    CONST_NAME_ENTRY(FRAMEPARS_SETFRAMEREL), \
    CONST_NAME_ENTRY(FRAMEPARS_SETLATENCY), \
    CONST_NAME_ENTRY(FRAMEPARS_SETFPGATIME), \
    CONST_NAME_ENTRY(FRAMEPARS_GETFPGATIME), \
    CONST_NAME_ENTRY(GAMMA_CACHE_NUMBER), \
    CONST_NAME_ENTRY(GAMMA_VALID_MASK), \
    CONST_NAME_ENTRY(GAMMA_VALID_REVERSE), \
    CONST_NAME_ENTRY(GAMMA_FPGA_MASK), \
    CONST_NAME_ENTRY(GAMMA_MODE_NOT_NICE), \
    CONST_NAME_ENTRY(GAMMA_MODE_NEED_REVERSE), \
    CONST_NAME_ENTRY(GAMMA_MODE_HARDWARE), \
    CONST_NAME_ENTRY(GAMMA_MODE_LOCK), \
    CONST_NAME_ENTRY(GAMMA_SCALE_SHIFT), \
    CONST_NAME_ENTRY(GAMMA_SCLALE_1), \
    CONST_NAME_ENTRY(HISTOGRAM_CACHE_NUMBER), \
    CONST_NAME_ENTRY(COLOR_Y_NUMBER), \
    CONST_NAME_ENTRY(FRAME_DEAFAULT_AHEAD), \
    CONST_NAME_ENTRY(AUTOCAMPARS_CMD_RESTORE), \
    CONST_NAME_ENTRY(AUTOCAMPARS_CMD_SAVE), \
    CONST_NAME_ENTRY(AUTOCAMPARS_CMD_DFLT), \
    CONST_NAME_ENTRY(AUTOCAMPARS_CMD_SAVEDFLT), \
    CONST_NAME_ENTRY(AUTOCAMPARS_CMD_INIT), \
    CONST_NAME_ENTRY(COLOR_RED), \
    CONST_NAME_ENTRY(COLOR_GREEN1), \
    CONST_NAME_ENTRY(COLOR_GREEN2), \
    CONST_NAME_ENTRY(COLOR_BLUE), \
    CONST_NAME_ENTRY(CSCALES_WIDTH), \
    CONST_NAME_ENTRY(CSCALES_CTL_BIT), \
    CONST_NAME_ENTRY(CSCALES_CTL_WIDTH), \
    CONST_NAME_ENTRY(CSCALES_CTL_NORMAL), \
    CONST_NAME_ENTRY(CSCALES_CTL_RECALC), \
    CONST_NAME_ENTRY(CSCALES_CTL_FOLLOW), \
    CONST_NAME_ENTRY(CSCALES_CTL_DISABLE), \
    CONST_NAME_ENTRY(HISTOGRAM_TABLE_OFFSET) \
};
/*
///NOTE page 0 is write protected, page 15 (0x0f) is "default" page
#define AUTOCAMPARS_CMD_RESTORE  1  /// restore specified groups of parameters from the specified page
#define AUTOCAMPARS_CMD_SAVE     2  /// save all current parameters to the specified group (page 0 is write-protected)
#define AUTOCAMPARS_CMD_DFLT     3  /// make selected page the default one (used at startup), page 0 OK 
#define AUTOCAMPARS_CMD_SAVEDFLT 4  /// save all current parameters to the specified group (page 0 is write-protected) and make it default (used at startup)
#define AUTOCAMPARS_CMD_INIT     5  /// reset sensor/sequencers, restore all parameters from the specified page

*/

#endif /* _ASM_CMOSCAM_H */
