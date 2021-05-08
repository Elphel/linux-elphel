/***************************************************************************//**
 * @file      boson640.c
 * @brief
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

//#define DEBUG // should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)
/****************** INCLUDE FILES SECTION ***********************************/
#include <linux/types.h> // for div 64
#include <asm/div64.h>   // for div 64
#include <asm/byteorder.h> // endians

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/platform_device.h>
#include <linux/device.h> // for dev_dbg, platform_device.h is OK too
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/c313a.h>
#include <uapi/elphel/x393_devices.h> // For sysfs
#include <linux/platform_device.h> // For sysfs
#include <linux/errno.h> //ETIMEDOUT
#include <linux/of.h>
#include <linux/jiffies.h>

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h" // detect sensor
//#include "cci2c.h"
#include "boson640.h"
//#include "multi10359.h"
#include "framepars.h"      // parameters manipulation
#include "sensor_common.h"
#include "pgm_functions.h"
#include "x393.h"
#include "sensor_i2c.h"

#undef LOCK_BH_SENSORIO



/**
 * \def D(x) optional debug output
 */

#if ELPHEL_DEBUG
#define MDF(x) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;}
#define MDF4(x) { if (GLOBALPARS(G_DEBUG) & (1 <<4)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
#define ELPHEL_DEBUG_THIS 0
// #define ELPHEL_DEBUG_THIS 1
#else
#define MDF(x)
#define MDF4(x)
#define ELPHEL_DEBUG_THIS 0
#endif

#if ELPHEL_DEBUG_THIS
#define MDF1(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x
#define MDD1(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__); x ; udelay (ELPHEL_DEBUG_DELAY)
#define D(x) printk("%s:%d:",__FILE__,__LINE__);x
#define D1(x) x
#define MD7(x) printk("%s:%d:",__FILE__,__LINE__);x
#define MD9(x) printk("%s:%d:",__FILE__,__LINE__);x
#else
#define MDF1(x)
#define MDD1(x)
#define D(x)
#define D1(x)
#define MD7(x)
#define MD9(x)
#endif

/**
LUT to map SENSOR_REGSxxx to internal sensor register addresses
  * needed for any sensor
  * For better manual mapping:
      - even elements are SENSOR_REGSxxx,
      - odd elements are sensor's register addresses.
  * has to be at least 16-bit/entry for 16 bit addresses
  * (for MT9X001 it's a 1-to-1 mapping)
*/
const unsigned short boson640_par2addr[] = {
          P_BOSON_GAO_GAIN,                 P_BOSON640_REG_gaoSetGainState,
          P_BOSON_GAO_FFC,                  P_BOSON640_REG_gaoSetFfcState,
          P_BOSON_GAO_TEMP_CORRECTION,      P_BOSON640_REG_gaoSetTempCorrectionState,
          P_BOSON_GAO_AVERAGER,             P_BOSON640_REG_gaoSetAveragerState,
          P_BOSON_GAO_NUM_FFC_FRAMES,       P_BOSON640_REG_gaoSetNumFFCFrames,
          P_BOSON_GAO_RNS,                  P_BOSON640_REG_gaoSetRnsState,
          P_BOSON_GAO_TEST_RAMP,            P_BOSON640_REG_gaoSetTestRampState,
          P_BOSON_GAO_SFFC,                 P_BOSON640_REG_gaoSetSffcState,
          P_BOSON_GAO_RPM,                  P_BOSON640_REG_gaoSetRpmState,
          P_BOSON_GAO_FFC_ZERO_MEAN,        P_BOSON640_REG_gaoSetFfcZeroMeanState,
          P_BOSON_GAO_RNS_POP_THRESHOLD,    P_BOSON640_REG_gaoSetRnsPopThreshold,
          P_BOSON_GAO_RNS_CLOSE_THRESHOLD,  P_BOSON640_REG_gaoSetRnsCloseThreshold,
          P_BOSON_GAO_RNS_TOO_FEW,          P_BOSON640_REG_gaoSetRnsTooFew,
          P_BOSON_GAO_RNS_MIN_CORRECTION,   P_BOSON640_REG_gaoSetRnsMinCorrection,
          P_BOSON_GAO_RNS_DAMPING,          P_BOSON640_REG_gaoSetRnsDamping,
          P_BOSON_GAO_RNS_FRAME_HYSTERESIS, P_BOSON640_REG_gaoSetRnsFrameHysteresis,
          P_BOSON_GAO_RNS_BAD_DAMPING,      P_BOSON640_REG_gaoSetRnsBadDamping,
          P_BOSON_GAO_RNS_NUM_GOOD_DAMPING, P_BOSON640_REG_gaoSetRnsNumGoodDampingThreshold,
          P_BOSON_ROIC_FPA_RAMP,            P_BOSON640_REG_roicSetFPARampState,
          P_BOSON_ROIC_FPA_TEMP_OFFSET,     P_BOSON640_REG_roicSetFPATempOffset,
          P_BOSON_ROIC_FPA_TEMP_MODE,       P_BOSON640_REG_roicSetFPATempMode,
          P_BOSON_ROIC_FPA_TEMP_VALUE,      P_BOSON640_REG_roicSetFPATempValue,
          P_BOSON_BPR,                      P_BOSON640_REG_bprSetState,
          P_BOSON_TELEMETRY,                P_BOSON640_REG_telemetrySetState,
          P_BOSON_TELEMETRY_LOCATION,       P_BOSON640_REG_telemetrySetLocation,
          P_BOSON_TELEMETRY_PACKING,        P_BOSON640_REG_telemetrySetPacking,
          P_BOSON_BOSON_RUN_FFC,            P_BOSON640_REG_bosonRunFFC, // No matchin read!
          P_BOSON_BOSON_SET_FFC_TEMP,       P_BOSON640_REG_bosonSetFFCTempThreshold,
          P_BOSON_BOSON_SET_FFC_FRAME,      P_BOSON640_REG_bosonSetFFCFrameThreshold,
          P_BOSON_BOSON_FFC_MODE,           P_BOSON640_REG_bosonSetFFCMode,
          P_BOSON_BOSON_GAIN_MODE,          P_BOSON640_REG_bosonSetGainMode,
          P_BOSON_BOSON_FFC_WARN_TIME,      P_BOSON640_REG_bosonSetFFCWarnTimeInSecx10,
          P_BOSON_BOSON_OVER_TEMP_TIME,     P_BOSON640_REG_bosonSetOverTempTimerInSec,
          P_BOSON_BOSON_FFC_QUICK_TIME,     P_BOSON640_REG_bosonSetTimeForQuickFFCsInSecs,
          P_BOSON_BOSON_EXT_SYNC_MODE,      P_BOSON640_REG_bosonSetExtSyncMode,
          P_BOSON_DVO_DISPLAY_MODE,         P_BOSON640_REG_dvoSetDisplayMode,
          P_BOSON_SCNR,                     P_BOSON640_REG_scnrSetEnableState,
          P_BOSON_SCNR_TH_COL_SUM,          P_BOSON640_REG_scnrSetThColSum,
          P_BOSON_SCNR_TH_PIXEL,            P_BOSON640_REG_scnrSetThPixel,
          P_BOSON_SCNR_MAX_CORR,            P_BOSON640_REG_scnrSetMaxCorr,
          P_BOSON_SCNR_TH_PIXEL_SAFE,       P_BOSON640_REG_scnrSetThPixelSafe,
          P_BOSON_SCNR_MAX_CORR_SAFE,       P_BOSON640_REG_scnrSetMaxCorrSafe,
          P_BOSON_TF,                       P_BOSON640_REG_tfSetEnableState,
          P_BOSON_TF_DELTA_NF,              P_BOSON640_REG_tfSetDelta_nf,
          P_BOSON_TF_TH_DELTA_MOTION,       P_BOSON640_REG_tfSetTHDeltaMotion,
          P_BOSON_TF_MOTION_THRESHOLD,      P_BOSON640_REG_tfSetMotionThreshold,
          P_BOSON_SPNR,                     P_BOSON640_REG_spnrSetEnableState,
          P_BOSON_SPNR_FRAME_DELAY,         P_BOSON640_REG_spnrSetFrameDelay,
          P_BOSON_SYSCTRL_FREEZE,           P_BOSON640_REG_sysctrlSetFreezeState,
		  P_BOSON_TEST_RAMP_MOTION,         P_BOSON640_REG_testRampSetMotionState,
          P_BOSON_TEST_RAMP_INDEX,          P_BOSON640_REG_testRampSetIndex,
          P_BOSON_SRNR,                     P_BOSON640_REG_srnrSetEnableState,
          P_BOSON_SRNR_TH_ROW_SUM,          P_BOSON640_REG_srnrSetThRowSum,
          P_BOSON_SRNR_TH_PIXEL,            P_BOSON640_REG_srnrSetThPixel,
          P_BOSON_SRNR_MAX_CORR,            P_BOSON640_REG_srnrSetMaxCorr,
// Registers that are not i2c (see if it will register pages)
//Next (P_BOSON640_GP3VSYNC) is the first non-i2c register
//        P_BOSON640_GP3VSYNC,    P_REG_BOSON640_GP3VSYNC, //  0x0854
//        P_BOSON640_TELEN,       P_REG_BOSON640_TELEN,    //  0x0218
//        P_BOSON640_TELLOC,      P_REG_BOSON640_TELLOC,   //  0x021c
//		P_BOSON640_FFC_RUN,     P_REG_BOSON640_FFC_RUN,
		0xffff // END indicator
};

//#define FIRST_BOSON640_INT P_BOSON640_GP3VSYNC


/**
 * get at least one parameter for a page
 */
const unsigned short boson640_pages[] = {
		P_BOSON640_REG_gaoSetRnsBadDamping,      // 1 byte
		P_BOSON640_REG_gaoSetNumFFCFrames,       // 2 bytes
		P_BOSON640_REG_gaoSetGainState,          // 4 bytes
		P_BOSON640_REG_roicSetFPATempOffset,     // 2 bytes
		P_BOSON640_REG_roicSetFPARampState,      // 4 bytes
		P_BOSON640_REG_bprSetState,              // 4 bytes
		P_BOSON640_REG_telemetrySetState,        // 4 bytes
		P_BOSON640_REG_bosonRunFFC,              // 0 bytes (no matching read)
		P_BOSON640_REG_bosonSetFFCTempThreshold, // 2 bytes
		P_BOSON640_REG_bosonSetFFCMode,          // 4 bytes
		P_BOSON640_REG_dvoSetDisplayMode,        // 4 bytes
		P_BOSON640_REG_scnrSetThColSum,          // 2 bytes
		P_BOSON640_REG_scnrSetEnableState,       // 4 bytes
		P_BOSON640_REG_tfSetDelta_nf,            // 2 bytes
		P_BOSON640_REG_spnrSetEnableState,       // 4 bytes
		P_BOSON640_REG_sysctrlSetFreezeState,    // 4 bytes
		P_BOSON640_REG_testRampSetIndex,         // 1 byte
		P_BOSON640_REG_testRampSetMotionState,   // 4 bytes
		P_BOSON640_REG_srnrSetThRowSum,          // 2 bytes
		P_BOSON640_REG_srnrSetEnableState,       // 4 bytes
		0xffff         // END indicator
};



/**
 * pgm_functions latencies table
 */
// ASAP - ASAP
// C,S  - continuous/safe    TRIG&4=0 SAFE=0 (visa versa?)
// C,NS - continuous/no skip TRIG&4=0 SAFE=1
// A,S  - async/safe         TRIG&4=4 SAFE=0 (visa versa?)
// A,NS - async/no skip      TRIG&4=4 SAFE=1
// NOL  - nooverlap          TRIG&8=8
// array size is AHEAD_TAB_FUNCS_COUNT
const unsigned short boson640_ahead_tab[] = // copied from mt9x001
{	/// function           ASAP  C,S   C,NS, A,S   A,NS  NOL
	onchange_recalcseq,     0,    0,    0,    0,    0,    0, /// recalculate sequences/latencies, according to P_SKIP, P_TRIG
	onchange_detectsensor,  1,    0,    0,    0,    0,    0, /// detect sensor type, sets sensor structure (capabilities), function pointers
	onchange_sensorphase,   1,    0,    0,    0,    0,    0, /// program sensor clock/phase (do immediately)
	onchange_i2c,           0,    0,    0,    0,    0,    0, /// program i2c
	onchange_initsensor,    1,    0,    0,    0,    0,    0, /// resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
	onchange_afterinit,     0,    0,    0,    0,    0,    0, /// restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
	onchange_multisens,     0,    2,    1,    1,    1,    0, /// chnages related to multiplexed sensors
	onchange_window,        0,    2,    1,    1,    1,    0, /// program sensor WOI and mirroring (flipping) - NOTE: 1 bad frame to skip
	onchange_window_safe,   0,    1,    1,    1,    1,    0, /// program sensor WOI and mirroring (flipping) - NOTE: no bad frames
	onchange_exposure,      0,    2,    1,    1,    1,    0, /// program exposure
	onchange_gains,         0,    1,    1,    1,    1,    0, /// program analog gains
	onchange_triggermode,   0,    2,    1,    1,    1,    0, /// program sensor trigger mode TODO: does it have any sense here?
	onchange_sensorin,      0,    0,    0,    0,    0,    0, /// program sensor input in FPGA (Bayer, 8/16 bits, ??), stop sensor (if needed)
	onchange_sensorstop,    0,    0,    0,    0,    0,    0, /// Stop acquisition from the sensor to the FPGA (start has latency of 2)
	onchange_sensorrun,     0,    1,    1,    1,    1,    0, /// Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
	onchange_gamma,         0,    1,    1,    1,    1,    0, /// program gamma table - make sure table is calculated, maybe send all but last word)
	onchange_hist,          0,    0,    0,    0,    0,    0, /// program histogram window TODO: fix FPGA - (now pos_top will be read too early - will use previous ) and add latency
	onchange_aexp,          0,    0,    0,    0,    0,    0, /// program autoexposure mode TODO: look what exactly is changed here
	onchange_quality,       0,    0,    0,    0,    0,    0, /// program quantization table(s)
	onchange_memsensor,     0,    0,    0,    0,    0,    0, /// program memory channels 0 (sensor->memory) and 1 (memory->FPN)
	onchange_memcompressor, 0,    0,    0,    0,    0,    0, /// program memory channel 2 (memory->compressor) (delays programming until onchange_comprestart if needed)
	/// For Micron sensors limitfps should have the same latency as changing window height, otherwise when WOI_HEIGHT 0x3c0->0x790 and next frame VBLANK 0x13e->0x284
	/// sensor waits till the counter overflows (>10 seconds) without any frame sync pulses
	onchange_limitfps,      0,    2,    1,    1,    1,    0, /// check compressor will keep up, limit sensor FPS if needed
	onchange_compmode,      0,    0,    0,    0,    0,    0, /// program compressor modes
	onchange_focusmode,     1,    0,    0,    0,    0,    0, /// program focus modes (through writing the tables, so no sequencer)
	onchange_trigseq,       0,    2,    1,    1,    1,    0, /// NC393: OK to program through the sequencer (full 32 bits)
	onchange_irq,           0,    0,    0,    0,    0,    0, /// program smart IRQ mode
	onchange_comprestart,   0,    0,    0,    0,    0,    0, /// restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
	// onchange_compstop should have the same latency as onchange_window
	// NC393 - triggered mode wants   onchange_compstop==2, while onchange_window == 1?
	// TODO: NC393 now comstop is used when changing color modes does not need two cycles - just 1
	onchange_compstop,      0,    2,    2,    2,    2,    0, /// stop compressor when changing geometry
	onchange_compctl,       0,    0,    1,    1,    1,    0, /// only start/stop/single (after explicitly changed, not when geometry was changed)
	onchange_gammaload,     1,    1,    1,    1,    1,    0, /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
	onchange_sensorregs,    0,    1,    1,    1,    1,    0, /// write sensor registers (only changed from outside the driver as they may have different latencies)?
	onchange_prescal,       0,    0,    0,    0,    0,    0  /// change scales for per-color digital gains, apply vignetting correction
};

static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
void boson640_set_device(struct device *dev) // do nothing, now it has it's own device
{
    //g_dev_ptr = dev;
}


/** Capabilities of MT9M001 1.3 MPix */
struct sensor_t boson640={
        // sensor constants
        .imageWidth  = 640,              ///< nominal image width for final images
        .imageHeight = 512,              ///< nominal image height for final images
        .clearWidth  = 640,              ///< maximal clear image width
        .clearHeight = 512,              ///< maximal clear image height;
        .clearTop    = 0,                ///< top margin to the first clear pixel
        .clearLeft   = 0,                ///< left margin to the first clear pixel
        .arrayWidth  = 640,              ///< total image array width (including black and boundary)
        .arrayHeight = 513,              ///< total image array height (including black and boundary)
        .minWidth    = 640,              ///< minimal WOI width
        .minHeight   = 512,              ///< minimal WOI height
        .minHorBlank = 0,                ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 640,              ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 640,              ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 0,                ///< minimal vertical blanking
        .maxVertBlank= 513,              ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 513,              ///< Maximal shutter duration (in lines)
        .flips       = 0,                ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 0,                ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 0,                ///< bayer shift for flips==0
        .dcmHor      = 0x0,              ///< available horizontal decimation values 1,2,4,8
        .dcmVert     = 0x0,              ///< available vertical decimation values 1,2,4,8
        .binHor      = 0x00,             ///< available horizontal binning values 1
        .binVert     = 0x00,             ///< vailable vertical binning values 1
        .maxGain256  = 0x100,            ///< (15.75) maximal analog gain times 0x100
        .minGain256  = 0x100,            ///< 1.5 times 0x100
        .minClockFreq= 27000000,         ///< Minimal clock frequency (remove - not needed?)
        .maxClockFreq= 27000000,         ///< Maximal clock frequency
        .nomClockFreq= 27000000,         ///< nominal clock frequency
        .sensorType  = SENSOR_BOSON640,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = BOSON640_I2C_ADDR,///< sensor i2c slave address (7 bits)
        .i2c_period  = 1000,             ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,                ///< number of bytes/ register
        .hact_delay  = 0,                ///< delay in ps, TBD
        .sensorDelay = 0,                ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE  ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};

// Sysfs Interface for debugging the driver
//static int first_sensor_sa7 [SENSOR_PORTS] = {0,0,0,0};
/*
static unsigned int debug_delays = 0x0; // 0x6464; // udelay() values for mrst (low 8 - mrst on), [15:8] - after mrst
static unsigned int debug_modes =  3;
*/
///static unsigned short sensor_reg_copy[SENSOR_PORTS][256]; ///< Read all 256 sensor registers here - during initialization and on demand
                                                 ///< Later may increase to include multiple subchannels on 10359

// a place to add some general purpose register writes to sensors during init

/** Register initial writes for BOSON640 */
/*
static  unsigned short boson640_inits[]=
{
};
*/
/*
		P_BOSON640_GP3VSYNC, P_REG_BOSON640_GP3VSYNC_VAL, // Enable VSYNC to GPIO3
		P_BOSON640_TELEN,    0,
		P_BOSON640_TELLOC,   1 // 0 // default is 1, 0 - to check that no wait is needed
		// TODO: if needed - add dummy writes?

};
*/

static DEFINE_SPINLOCK(sensorio_lock_0); ///<
static DEFINE_SPINLOCK(sensorio_lock_1); ///<
static DEFINE_SPINLOCK(sensorio_lock_2); ///<
static DEFINE_SPINLOCK(sensorio_lock_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
spinlock_t * sensorio_locks[4] = {&sensorio_lock_0, &sensorio_lock_1, &sensorio_lock_2, &sensorio_lock_3};
//static struct device *sdev = NULL;   ///< store this device here -> g_dev_ptr
static u32 uart_seq_number[4];                  ///< 0x0000xxxx are used by extif (sequencer module)
static u32 uart_rec_seq_number[4];              ///< last received sequence number
static u32 uart_status[4];                      ///< last received status
static u16 uart_cs16[4];                        ///< last received checksum
//static int uart_extif_en[4];                    ///< extif enabled (set initially to -1,-1,-1,-1
static int uart_extif_en[] = {-1,-1,-1,-1};     ///< extif enabled (set initially to -1,-1,-1,-1
static int uart_length[4];                      ///< received/transmitted packet length
static u8  uart_data[4][BOSON_PACKET_MAX_DATA]; ///< received/transmitted packet data
static u16 drp_read_data[4];                    /// last received DRP register data
static u8  drp_read_phase[8];                   /// last received DRP clock phase data
static u8 DRP_CLK_REG_ADDR [][2] = {
		{0x14, 0x15},
		{0x08, 0x09},
		{0x0a, 0x0b},
		{0x0c, 0x0d},
		{0x0e, 0x0f},
		{0x10, 0x11},
		{0x06, 0x07},
		{0x12, 0x13}};

void boson640_reset_release      (int sensor_port); // should be called during boson640_pgm_detectsensor
//int  boson640_is_booted          (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_is_booted          (int sensor_port, struct framepars_t * thispars);
int  boson640_pgm_detectsensor   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_initsensor     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_sensorin       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_window         (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_window_safe    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_window_common  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_limitfps       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_exposure       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_gains          (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_triggermode    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int  boson640_pgm_sensorregs     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);

void io_autoupdate_status       (int chn);
x393_status_sens_io_t io_new_status(int chn);

void set_sensor_uart_ctl_boson  (int chn, int uart_extif_en, int uart_xmit_rst, int uart_recv_rst, int uart_xmit_start, int uart_recv_next);
void set_sensor_uart_start_send (int chn);
void set_sensor_uart_recv_next  (int chn);
void set_sensor_uart_set_extif  (int chn, int en);


void uart_send_packet           (int chn, u32 command, int len, u8 * data);
int  uart_receive_packet        (int chn, u8* channel, u32* sequence, u32* command, u32* status, int max_bytes, u8* data, u16* cs);
int  uart_receive_byte          (int chn);
int  uart_skip_byte             (int chn);

int  uart_wait_receive          (int chn);
int  uart_wait_transmit         (int chn);

void drp_open                   (int sensor_port);
void drp_close                  (int sensor_port);
int drp_read_reg                (int sensor_port, u8 daddr);
int drp_write_reg               (int sensor_port, int daddr, u16 data,u16 mask);
u8* drp_phase_addr              (int clk_out);
int drp_read_clock_phase        (int sensor_port, int clk_out);
int drp_write_clock_phase       (int sensor_port, int clk_out, u16 phase); // 0..1ff

// SysFS interface functions
static int     get_channel_from_name(struct device_attribute *attr);
static ssize_t store_uart_seq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_uart_seq(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_uart_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_uart_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_uart_cs16(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_extif(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_extif(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_uart_packet(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_uart_packet(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t get_uart_help(struct device *dev,  struct device_attribute *attr, char *buf);
static ssize_t store_drp_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_drp_register(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_drp_phase(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_drp_phase(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t get_drp_help(struct device *dev, struct device_attribute *attr, char *buf);
/* Sysfs functionality */
//======== UART interface ==========
// reset and enable UART communications
/**'
 * Control UART
 * @param chn             sensor port
 * @param uart_extif_en   1 - enable sequencer commands, 0 - disable sequencer commands, < 0 - no change
 * @param uart_xmit_rst   1 - persistent reset software packet transmission, 0 - normal operation, <0 - no change
 * @param uart_recv_rst   1 - persistent reset packet receive, 0 - normal operation, <0 - no change
 * @param uart_xmit_start 1 - start transmiting prepared packet (self-clearing)
 * @param uart_recv_next  1 - advance receive FIFO to next byte (self-clearing)
 */

void set_sensor_uart_ctl_boson (int chn,
								int uart_extif_en,
								int uart_xmit_rst,
								int uart_recv_rst,
								int uart_xmit_start,
								int uart_recv_next)
{
	x393_sensio_tim1_t uart_ctrl = {.d32=0};
	if (uart_extif_en >= 0) {
		uart_ctrl.uart_extif_en = uart_extif_en;
		uart_ctrl.uart_extif_en_set = 1;
	}
	if (uart_xmit_rst >= 0) {
		uart_ctrl.uart_xmit_rst = uart_xmit_rst;
		uart_ctrl.uart_xmit_rst_set = 1;
	}
	if (uart_recv_rst >= 0) {
		uart_ctrl.uart_recv_rst = uart_recv_rst;
		uart_ctrl.uart_recv_rst_set = 1;
	}
	uart_ctrl.uart_xmit_start = uart_xmit_start;
	uart_ctrl.uart_recv_next =  uart_recv_next;
	set_x393_sensio_tim1  (uart_ctrl, chn);
    dev_dbg(g_dev_ptr,"set_sensor_uart_ctl_boson(): {%d} uart_ctrl = 0x%x\n", chn, uart_ctrl.d32);
}
void set_sensor_uart_start_send (int chn){ set_sensor_uart_ctl_boson (chn, -1, -1, -1, 1, 0);}
void set_sensor_uart_recv_next  (int chn){ set_sensor_uart_ctl_boson (chn, -1, -1, -1, 0, 1);}
void set_sensor_uart_set_extif  (int chn,
		                         int en)
{
	if (uart_extif_en[chn] != en) {
		uart_extif_en[chn] = en;
		set_sensor_uart_ctl_boson (chn, en, -1, -1, 0, 0);
	}
}


// No control bits but uart_xmit_start, no waiting for completion
void uart_send_packet (int    chn,     ///< Sensor port
		               u32    command, ///< module/funcion to send
					   int    len,     ///< number of data bytes to send
					   u8 *   data)    ///< data to send
{
	x393_sensio_tim0_t uart_data;
	int i;
	uart_data.uart_txd = 0; // channel number == 0
	set_x393_sensio_tim0  (uart_data, chn);
    dev_dbg(g_dev_ptr,"uart_send_packet(){%d}:channel byte = 0x%x\n", chn, uart_data.d32);
	uart_seq_number[chn] += 1;
	if (!(uart_seq_number[chn] & 0xffff000)) uart_seq_number[chn] = 0x10000; // skip 16-bit sequnece numbers reserved for extif (sequencer) module
	for (i = 24; i >=0; i-=8){ // send sequence number, big endian
		uart_data.uart_txd = uart_seq_number[chn] >> i;
		set_x393_sensio_tim0  (uart_data, chn);
	    dev_dbg(g_dev_ptr,"uart_send_packet(){%d}:sequence byte = 0x%x\n", chn, uart_data.d32);
	}
	for (i = 24; i >=0; i-=8){ // send command, big endian
		uart_data.uart_txd = command >> i;
		set_x393_sensio_tim0  (uart_data, chn);
	    dev_dbg(g_dev_ptr,"uart_send_packet(){%d}:command byte = 0x%x\n", chn, uart_data.d32);
	}
	uart_data.uart_txd = 0xff;
	for (i = 0; i < 4; i++){ // send 0xffffffff
		set_x393_sensio_tim0  (uart_data, chn);
	    dev_dbg(g_dev_ptr,"uart_send_packet(){%d}:status byte = 0x%x\n", chn, uart_data.d32);
	}
	for (i = 0; i < len; i++){
		uart_data.uart_txd = data[i];
		set_x393_sensio_tim0  (uart_data, chn);
	    dev_dbg(g_dev_ptr,"uart_send_packet(){%d}:data byte = 0x%x\n", chn, uart_data.d32);
	}
	set_sensor_uart_start_send (chn);
}

/**
 * Read one byte from the UART read FIFO, advance pointer if it was data byte, not EOP
 * EOP will need to advance FIFO with an additional call to uart_skip_byte()
 * @param chn sensor channel
 * @return data byte, -1 for the EOP and -ETIMEDOUT for timeout
 */

int uart_receive_byte (int chn)      ///< Sensor port
                                          ///< @return byte in 8 LSBs or -1 if EOP (FIFO with EOP will not be advanced!)
{
    x393_status_sens_io_t io_status;
    int rslt, data;
    io_autoupdate_status(chn);
    io_status =  x393_sensio_status (chn);
    if (io_status.recv_eop){
    	return -1; // does not advance to the next byte !
    }
    data = io_status.recv_data;
    if ((rslt = uart_skip_byte(chn))) return rslt;
	return data;
}

/**
 * Advance UART read FIFO pointer, check for timeout. Called from uart_receive_byte() and from uart_receive_packet()
 * to advance pointer after end of packet
 * @param chn sensor channel
 * @return 0 - OK, -ETIMEDOUT for timeout
 */

int uart_skip_byte (int chn)      ///< Sensor port
                                  ///< @return 0 - OK, <0 - error (-ETIMEDOUT)
{
    x393_status_sens_io_t io_status, io_status0;
	int timeout_duration = 2; // == 1/50 sec
	unsigned long       timeout_end;
    io_autoupdate_status(chn);
    io_status0 =  x393_sensio_status (chn);
    set_sensor_uart_recv_next(chn);
    // now wait for either .recv_odd_even toggles or recv_pav is zero
    timeout_end = jiffies + timeout_duration;
    while (jiffies < timeout_end){
        io_status =  x393_sensio_status (chn);
        if ((!io_status.recv_pav) || (io_status.recv_odd_even != io_status0.recv_odd_even)) return 0;
    }
	return -ETIMEDOUT;
}


/**
 * TODO: add length limit for data (for u32 reasout)
 * @param chn
 * @param channel
 * @param sequence
 * @param command
 * @param status
 * @param max_bytes
 * @param data
 * @return
 */
int uart_receive_packet (int    chn,      ///< Sensor port
                         u8 *   channel,  ///< if not null, will return packet channel
                         u32 *  sequence, ///< if not null, will return sequence number
						 u32 *  command,  ///< if not null, will return command number
						 u32 *  status,   ///< if not null, will return status
						 int    max_bytes, ///< maximal number of payload bytes, <0 - use BOSON_PACKET_MAX_DATA
                         u8 *   data,     ///< if not null, will return data (up to 756 bytes)
                         u16*   cs)       ///<  if not null, will return 16-bit checksum
                          /// < @return number of received data bytes, -1 - packet not available, -2 - packet > 756 (needs reset)
{
    x393_status_sens_io_t io_status;
    int i;
    int recv_data;
    int data_len;
    if (max_bytes <0){
    	max_bytes = BOSON_PACKET_MAX_DATA + 2;  // 2-byte checksum
    }
    io_autoupdate_status(chn);
    io_status =  x393_sensio_status (chn);
    dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} max_bytes = %d, io_status.d32=0x%x\n", chn, max_bytes,io_status.d32);
    if (io_status.recv_pav == 0) return -BOSON_ERR_NO_PACKET;
    // get packet 1-byte channel
    recv_data = uart_receive_byte(chn);
    dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} byte 1 recv_data=0x%x\n", chn, recv_data);
    if (channel)  	*channel = recv_data;
    if (recv_data < 0) 	{
    	uart_skip_byte (chn); // skip EOP
    	return -BOSON_ERR_FBP_FORMAT;
    }
    // get packet 4-byte channel (big endian)
	if (sequence) *sequence = 0;
    for (i = 24; i >= 0; i-=8){
    	recv_data = uart_receive_byte(chn);
        dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} bytes seq recv_data=0x%x\n", chn, recv_data);
        if (recv_data < 0) 	{
        	uart_skip_byte (chn); // skip EOP
        	return -BOSON_ERR_FBP_FORMAT;
        }
    	if (sequence) *sequence |=  (recv_data << i);
    }
    // get packet 4-byte command (big endian)
	if (command) *command = 0;
    for (i = 24; i >= 0; i-=8){
    	recv_data = uart_receive_byte(chn);
        dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} bytes command recv_data=0x%x\n", chn, recv_data);
        if (recv_data < 0) 	{
        	uart_skip_byte (chn); // skip EOP
        	return -BOSON_ERR_FBP_FORMAT;
        }
    	if (command) *command |= (recv_data << i);
    }
    // get packet 4-byte status (big endian)
	if (status) *status = 0;
    for (i = 24; i >= 0; i-=8){
    	recv_data = uart_receive_byte(chn);
        dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} bytes status recv_data=0x%x\n", chn, recv_data);
        if (recv_data < 0) 	{
        	uart_skip_byte (chn); // skip EOP
        	return -BOSON_ERR_FBP_FORMAT;
        }
    	if (status) *status |= (recv_data << i);
    }
    // get variable-length (0...756( +2) bytes, optionally limited to provided max_bytes parameter) packet data
    for (data_len = 0; data_len < max_bytes; data_len++){
    	recv_data = uart_receive_byte(chn);
        dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} bytes data recv_data=0x%x\n", chn, recv_data);
    	if (recv_data < 0) break; // EOP - prematurely, should only happen with "unlimited" data length
    	if (data) data [data_len] = recv_data;
    	if (cs) *cs = (*cs << 8) + recv_data;
    }
    if (recv_data < 0) {
    	if (max_bytes < (BOSON_PACKET_MAX_DATA + 2)){
        	uart_skip_byte (chn); // skip EOP
        	return -BOSON_ERR_FBP_FORMAT;
    	}
    	data_len -= 2;
    } else { // read 2 bytes to *cs
        for (i = 8; i >= 0; i-=8){
        	recv_data = uart_receive_byte(chn);
            dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} cs recv_data=0x%x\n", chn, recv_data);
            if (recv_data < 0) 	{
            	uart_skip_byte (chn); // skip EOP
            	return -BOSON_ERR_FBP_FORMAT;
            }
        	if (cs) *cs |= (*cs << i);
        }
    }

    // Try reading 1 byte - it should be EOP (error if not)
	recv_data = uart_receive_byte(chn);
    dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} bytes EOP recv_data=0x%x\n", chn, recv_data);
    if (recv_data != -1){
    	return -BOSON_ERR_NO_EOP;
    }
	uart_skip_byte (chn); // skip EOP
    dev_dbg(g_dev_ptr,"uart_receive_packet(): {%d} data_len = %d\n", chn, data_len);
    return data_len;
}

/**
 *  Wait received packet (as a response to transmitted) with timeout (1/50 sec)
 *
 * @param chn sensor port
 * @return 0 - OK, -ETIMEDOUT - timeout
 */
int uart_wait_receive (int chn)
{
	int timeout_duration = 2; // == 1/50 sec
	unsigned long       timeout_end;
    x393_status_sens_io_t io_status;
    io_autoupdate_status(chn);
    timeout_end = jiffies + timeout_duration;
    while (jiffies < timeout_end){
        io_autoupdate_status(chn);
        io_status =  x393_sensio_status (chn);
        if (io_status.recv_pav) break;
    }
    if (io_status.recv_pav == 0) {
        dev_dbg(g_dev_ptr, "Timeout waiting for uart recieved packet for channel %d\n",chn);
    	return -ETIMEDOUT;
    }
    return 0; // OK
}

int uart_wait_transmit (int chn)
{
	int timeout_duration = 2; // == 1/50 sec
	unsigned long       timeout_end;
    x393_status_sens_io_t io_status;
    io_autoupdate_status(chn);
    timeout_end = jiffies + timeout_duration;
    while (jiffies < timeout_end){
        io_autoupdate_status(chn);
        io_status =  x393_sensio_status (chn);
        if (io_status.xmit_busy == 0) break;
    }
    if (io_status.xmit_busy != 0) {
        dev_dbg(g_dev_ptr, "Timeout waiting for uart to transmit a packet for channel %d\n",chn);
    	return -ETIMEDOUT;
    }
    return 0; // OK
}

/**
 * Enable DRP communication by resetting MMCM/PLL
 * @param sensor_port sensor port number (0..3)
 */
void drp_open (int sensor_port){
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    sensio_ctl.mmcm_rst =        1;
    sensio_ctl.mmcm_rst_set =    1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
}

/**
 * Complete DRP communication by releasing reset to MMCM/PLL
 * @param sensor_port sensor port number (0..3)
 */
void drp_close (int sensor_port){
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    sensio_ctl.mmcm_rst =        0;
    sensio_ctl.mmcm_rst_set =    1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
}

/**
 * Read specified DRP register. DRP should be open (MMCM/PLL in reset)
 * @param sensor_port sensor port number (0..3)
 * @param daddr DRP 7-bit address
 * @return
 */
int drp_read_reg (int sensor_port, u8 daddr){
	int data, i;
	u32 odd_bit, odd_bit1;
    x393_status_sens_io_t io_status;
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};

    for (i = DRP_ADDRESS_LENGTH - 1; i >= 0; i--){
    	sensio_ctl.drp_cmd = ((daddr >> i) & 1) + 1;
        x393_sensio_ctrl(sensio_ctl, sensor_port);
    }
    io_status =  io_new_status(sensor_port);
    odd_bit = io_status.drp_odd_bit;
    odd_bit1 = odd_bit;
	sensio_ctl.drp_cmd = 3; // execute
    x393_sensio_ctrl(sensio_ctl, sensor_port);
    for (i = 0; i < 10; i++){ // wait DREADY
        io_status =  io_new_status(sensor_port);
        odd_bit1 = io_status.drp_odd_bit;
        if (odd_bit1 != odd_bit) break;
    }
    if (odd_bit1 == odd_bit) return -ETIMEDOUT;
    sensio_ctl.drp_cmd = 1;
    for (i = 0; i < DRP_DATA_LENGTH; i++){
        io_status =  io_new_status(sensor_port);
        data = (data << 1) + io_status.drp_bit;
        x393_sensio_ctrl(sensio_ctl, sensor_port);
    }
    sensio_ctl.drp_cmd = 3; // nop execute, finish command to reset state machine
    x393_sensio_ctrl(sensio_ctl, sensor_port);
	return data; // u16 or negative error
}

/**
 * Write specified DRP register. DRP should be open (MMCM/PLL in reset)
 * @param sensor_port sensor port number (0..3)
 * @param daddr DRP 7-bit address
 * @param data 16-bit data to write
 * @param mask if mask != 0xffff, first read register, then write updating only those bits where mask bit ==1
 * @return 0 or negative error (-ETIMEOUT)
 */
int drp_write_reg (
		int sensor_port,
		int daddr,
		u16 data,
		u16 mask){ // bit == 1 - use new value, 0 - use old one
	int old_data, i;
	u32 odd_bit, odd_bit1;
    x393_status_sens_io_t io_status;
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
	if (mask != 0xffff){
		if ((old_data = drp_read_reg (sensor_port, daddr))<0){
			return old_data; // negaqtive error (ETIMEOUT)
		}
		data = ((old_data ^ data) & mask) ^ old_data;
	}
	// shift address
    for (i = DRP_ADDRESS_LENGTH - 1; i >= 0; i--){
    	sensio_ctl.drp_cmd = ((daddr >> i) & 1) + 1;
        x393_sensio_ctrl(sensio_ctl, sensor_port);
    }
    // shift data to write
    for (i = DRP_DATA_LENGTH - 1; i >= 0; i--){
    	sensio_ctl.drp_cmd = ((data >> i) & 1) + 1;
        x393_sensio_ctrl(sensio_ctl, sensor_port);
    }
    // get odd/even before "execute"
    io_status =  io_new_status(sensor_port);
    odd_bit = io_status.drp_odd_bit;
    odd_bit1 = odd_bit;
	sensio_ctl.drp_cmd = 3; // execute
    x393_sensio_ctrl(sensio_ctl, sensor_port);
    for (i = 0; i < 10; i++){ // wait DREADY
        io_status =  io_new_status(sensor_port);
        odd_bit1 = io_status.drp_odd_bit;
        if (odd_bit1 != odd_bit) break;
    }
    if (odd_bit1 == odd_bit) return -ETIMEDOUT;
	return 0; // OK
}

/**
 * Get DRP register pair for phase control (LSB - ClkReg1, MSB - ClkReg2
 * as defined in xapp888)
 * @param clk_out -1 - CLKFBOUT, 0..6 - CLKOUT0...CLKOUT6
 * @return address pair ClkReg1 + (ClkReg2 << 8)
 */
u8* drp_phase_addr(int clk_out){
	return DRP_CLK_REG_ADDR[clk_out + 1];
}

/**
 * Read phase shift for the selected clock output of the MMCM/PLL. Phase 3 LSBs
 * are 1/8ths of the VCO period, 5 MSBs full VCO periods
 * @param sensor_port sensor port (0..3)
 * @param clk_out clock output index: -1 - CLKFBOUT, 0..6 - CLKOUT0...CLKOUT6
 * @return 8-bit phase shift in 1/8 VCO periods or -ETIMEOUT
 */
int drp_read_clock_phase (int sensor_port, int clk_out){
	u8* addr_pair = drp_phase_addr(clk_out);
	int data1, data2;
	if ((data1 = drp_read_reg (sensor_port, addr_pair[0])) < 0) return data1;
	if ((data2 = drp_read_reg (sensor_port, addr_pair[1])) < 0) return data2;
    return ((data1 >> 13) & 7) | ((data2 & 0x3f) << 3);
}

/**
 * Read phase shift for the selected clock output of the MMCM/PLL. Phase 3 LSBs
 * are 1/8ths of the VCO period, 5 MSBs full VCO periods
 * @param sensor_port sensor port (0..3)
 * @param clk_out clock output index: -1 - CLKFBOUT, 0..6 - CLKOUT0...CLKOUT6
 * @param phase 8-bit phase shift in 1/8 VCO periods or -ETIMEOUT
 * @return 0-OK, <0 - error (-ETIMEOUT)
 */
int drp_write_clock_phase (int sensor_port, int clk_out, u16 phase)
{
	u8* addr_pair = drp_phase_addr(clk_out);
	int data1, data2, rslt;
    data1 = (phase & 7) << 13;
    data2 = (phase >> 3) & 0x3f;
    if ((rslt = drp_write_reg(sensor_port, addr_pair[0], data1, 0xe000)) < 0) return rslt;
    if ((rslt = drp_write_reg(sensor_port, addr_pair[1], data2, 0x003f)) < 0) return rslt;
    return 0;
}

void io_autoupdate_status(int chn)  ///< Sensor port
{
#ifndef LOCK_BH_SENSORIO
    unsigned long flags;
#endif
    x393_status_sens_io_t  status;
    x393_status_ctrl_t status_ctrl = get_x393_sensio_status_cntrl(chn); // last written data to status_cntrl
    if (status_ctrl.mode != 3){
#ifdef LOCK_BH_SENSORIO
        spin_lock_bh(sensorio_locks[chn]);
#else
        spin_lock_irqsave(sensorio_locks[chn],flags);
#endif
        status =  x393_sensio_status (chn);
        status_ctrl.mode = 3;
        status_ctrl.seq_num = status.seq_num ^ 0x20;
        set_x393_sensio_status_cntrl(status_ctrl, chn);
        {
            int i;
            for (i = 0; i < 10; i++) {
                status = x393_sensio_status(chn);
                if (likely(status.seq_num = status_ctrl.seq_num)) break;
            }
        }
#ifdef LOCK_BH_SENSORIO
        spin_unlock_bh(sensorio_locks[chn]);
#else
        spin_unlock_irqrestore(sensorio_locks[chn],flags);
#endif
        dev_dbg(g_dev_ptr, "io_autoupdate_status(%d): mode set to 3, status updated to 0x%08x\n",chn, (int) status.d32);
    }
}

/**
 * Get new status
 * @param chn sensor port
 * @return status - current
 */
x393_status_sens_io_t io_new_status(int chn)  ///< Sensor port
{
#ifndef LOCK_BH_SENSORIO
    unsigned long flags;
#endif
    x393_status_sens_io_t  status;
    x393_status_ctrl_t status_ctrl = get_x393_sensio_status_cntrl(chn); // last written data to status_cntrl
#ifdef LOCK_BH_SENSORIO
	spin_lock_bh(sensorio_locks[chn]);
#else
	spin_lock_irqsave(sensorio_locks[chn],flags);
#endif
	status =  x393_sensio_status (chn);
	status_ctrl.mode = 3;
	status_ctrl.seq_num = status.seq_num ^ 0x20;
	set_x393_sensio_status_cntrl(status_ctrl, chn);
	{
		int i;
		for (i = 0; i < 10; i++) {
			status = x393_sensio_status(chn);
			if (likely(status.seq_num = status_ctrl.seq_num)) break;
		}
	}
#ifdef LOCK_BH_SENSORIO
	spin_unlock_bh(sensorio_locks[chn]);
#else
	spin_unlock_irqrestore(sensorio_locks[chn],flags);
#endif
	dev_dbg(g_dev_ptr, "io_new_status(%d): mode set to 3, status updated to 0x%08x\n",chn, (int) status.d32);
	return status;
}




/**
 * Get single internal Lepton register (data length = 1), wait no longer than specified (in ms)
 */

int boson640_read_reg     (int             sensor_port,      ///< sensor port number (0..3)
                          u8               reg, ///< sensor register index (0..255)
						  u32 *            data ///< will return data
						  ){
//                          int               wait_ms){ ///< milliseconds to wait for ready (if >0)
    // assume read register command is next one after set (with the exception of zero-length), same data length

    struct sensor_port_config_t *pcfg;
    const char *name;
    int i;
    int  sensor_num = 0; // no mux, always 0 sub-channel
    int bytes_code, num_bytes, num_received;
    u32 command;
    u32 function_code; // boson640 combined bytes,module and function
    ///x393_i2c_device_t * dc;
    int extif_en_old, rslt;
    u8 rec_bytes[4];
//    psensor= &boson640;
    pcfg = &pSensorPortConfig[sensor_port];
    name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR); // "boson640"
    function_code = pcfg->par2addr[sensor_num][reg];
    if (function_code & 0xffff0000){
    	return -ENOENT; // no such register defined
    }
    bytes_code = (function_code >> 14) & 3;
    if (!bytes_code){
    	return -EPERM; // Zero bytes in data - nothing to read
    }
    num_bytes = (bytes_code==3)? 4 :  bytes_code;
    command =   (function_code & 0x3fff) + 1; // here assuming read to be next form swrite!
    // disable extif (save if it was enabled)
    extif_en_old = uart_extif_en[sensor_port];
    set_sensor_uart_set_extif  (sensor_port, 0); // disable
    // wait uart ready (what about response form Boson???)
    if ((rslt=uart_wait_transmit (sensor_port))){
        set_sensor_uart_set_extif  (sensor_port, extif_en_old); // optionally re-enable
    	return rslt; // timeout
    }
    // reset uart transmit and receive
    set_sensor_uart_ctl_boson (sensor_port, -1, 1, 1, 0, 0);
    set_sensor_uart_ctl_boson (sensor_port, -1, 0, 0, 0, 0);
// update uart_send_packet() to limit number of bytes received
    uart_send_packet          (sensor_port, command, 0, uart_data[sensor_port]);
    if ((rslt = uart_wait_receive (sensor_port)) < 0){ // wait receive packet done, return on timeout
        set_sensor_uart_set_extif  (sensor_port, extif_en_old); // optionally re-enable
    	return rslt;
    }
    num_received = uart_receive_packet (sensor_port,
            0, // u8 *   channel,              // in not null, will return packet channel
            uart_rec_seq_number + sensor_port, // u32 *  sequence, ///< in not null, will return sequence number
            &command,                           // u32 *  command,  ///< in not null, will return command number
            uart_status + sensor_port,         // if not null, will status
			-1, // int    max_bytes, ///< maximal number of payload bytes, <0 - use BOSON_PACKET_MAX_DATA
			rec_bytes,                        // if not null, will return data (up to 756
			uart_cs16 + sensor_port);
    if (num_received < 0) {
        set_sensor_uart_set_extif  (sensor_port, extif_en_old); // optionally re-enable
    	return num_received; // error
    }
    if (num_received != num_bytes){
        set_sensor_uart_set_extif  (sensor_port, extif_en_old); // optionally re-enable
    	return -EBADF; // different received data length than expected
    }
    if (data){ // not a null pointer
    	*data = 0;
    	for (i = 0; i < num_bytes; i++){
    		*data = (*data << 8) + rec_bytes[i];
    	}
    }
    set_sensor_uart_set_extif  (sensor_port, extif_en_old); // optionally re-enable
	return 0;
}



/**
 * Check is sensor is fully booted and initiate initialization if it first access after frames passed
 * Verify before any access to sensor registers
 * @param sensor_port
 * @param thispars
 * @return 0 - too early, 1 - just booted, 2 - booted
 */
int  boson640_is_booted   ( int sensor_port,
							struct framepars_t * thispars) ///< sensor current parameters
{
	int cframe;
	if (thispars->pars[P_BOOTED]){
		return 2; // already booted
	}
	cframe = getThisFrameNumber(sensor_port); // was cframe16?
	if (cframe < BOSON640_BOOT_FRAME){
//        dev_warn(g_dev_ptr,"boson640_is_booted(): Not yet fully booted{%d}, frame %d < %d\n",sensor_port, cframe, BOSON640_BOOT_FRAME);
        dev_dbg(g_dev_ptr,"boson640_is_booted(): Not yet fully booted{%d}, frame %d < %d, P_COLOR =%ld\n",sensor_port, cframe, BOSON640_BOOT_FRAME,thispars->pars[P_COLOR]);
		return 0; // too early
	}
    setFramePar(sensor_port, thispars, P_BOOTED | FRAMEPAIR_FORCE_PROC,  1); // should initiate pgm_initsensor and others?
//    setFramePar(sensor_port, thispars, P_BOOTED,  1); // should initiate pgm_initsensor and others?
//    dev_warn(g_dev_ptr,"boson640_is_booted(): Just fully booted{%d}\n",sensor_port);
    dev_warn(g_dev_ptr,"boson640_is_booted(): Just fully booted{%d}, P_COLOR =%ld\n",sensor_port,thispars->pars[P_COLOR]);
    return 1; // first time
}

/**
 * Set initial clock and lanes delays needed for 103993 rev 0 that requires precise settings. Will be removed for rev A
 * @param sensor_port
 * @return
 */

int set_initial_phase(int sensor_port,
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                       ///< be applied to,  negative - ASAP
                                       ///< @return 0 - OK, negative - error
{
    // just setting clock and 3 lanes delays (5 MSBs 3 LSB are not used)
	x393_sensio_tim2_t tim2 ={.d32=0};
//	x393_sensio_tim3_t tim3 ={.d32=0};
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    u8 initial_delays[] = {0x00, 0x80, 0x80, 0x80, 0x80,  // clock, lane0, lanel, lane2
                           0x00, 0x80, 0x80, 0x80, 0x80,
                           0x00, 0x80, 0x80, 0x80, 0x80,
	                       0x00, 0x80, 0x80, 0x88, 0x80};
    tim2.dly_lane0 = initial_delays[5*sensor_port + 1];
    tim2.dly_lane1 = initial_delays[5*sensor_port + 2];
    tim2.dly_lane2 = initial_delays[5*sensor_port + 3];
    tim2.dly_lane3 = initial_delays[5*sensor_port + 4]; // not available in rev 0, added for revA
//    tim3.phase_h =   initial_delays[4*sensor_port + 0]; // may be restored for revA - it is 9 bits for MMCM phase
//  Will use DRP (immediate mode only, not through tghe sequencer)

    set_x393_sensio_tim2(tim2, sensor_port);
//    set_x393_sensio_tim3(tim3, sensor_port);
    sensio_ctl.set_dly = 1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
//    dev_info(g_dev_ptr,"**set_initial_phase**: {%d}  setting delays tim2 = 0x%x tim3 = 0x%x \n",sensor_port, tim2.d32, tim3.d32);
    dev_info(g_dev_ptr,"**set_initial_phase**: {%d}  setting delays tim2 = 0x%x \n",sensor_port, tim2.d32);

//P_SENSOR_IFACE_TIM2
    setFramePar(sensor_port, thispars, P_SENSOR_IFACE_TIM2,  tim2.d32);
//    setFramePar(sensor_port, thispars, P_SENSOR_IFACE_TIM3,  tim3.d32);
	return 0;
}

/**
 * Detect and initialize sensor and related data structures
 * - detect sensor type.
 * - if successful, proceed to:,
 *   -- copy sensor static structure
 *   -- setup appropriate pgm_* functions
 *   -- read sensor registers to shadows
 *   -- initialize appropriate P_* registers (including sensor register shadows) - that initialization will schedule related pgm_* functions
 *
 * TODO: when is P_CLK_FPGA initialized? Needs to be done before this
 * hardware i2c is expected to be reset and initialized - no wrong, it will be programmed in
 * onchange_i2c should be the first after init sensor (even before onchange_sensorphase)
 * onchange_sensorphase will be triggered after this
 * hardware i2c after this function will be disabled, will need onchange_sensorphase to initialize/start it.
 */
int boson640_pgm_detectsensor  (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
    struct sensor_t * psensor; // current sensor
    // temporary
    struct sensor_port_config_t *pcfg;
    const char *name;
    x393_i2c_device_t * dc;

    dev_info(g_dev_ptr,"**boson640_pgm_detectsensor**: {%d}  frame16=%d, thispars->pars[P_SENSOR]= 0x%lx\n",sensor_port,frame16, thispars->pars[P_SENSOR]);
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    //  MDD1(printk("sensor=0x%x\n", (int)sensor));
    if (thispars->pars[P_SENSOR]!=0) { ///already initialized - called second time after common pgm_detectsensor(), first time is inside pgm_detectsensor()
        dev_info(g_dev_ptr,"{%d}  sensor 0x%x already detected, exiting\n",sensor_port,(int) thispars->pars[P_SENSOR]);
        return sensor->sensorType;
    }

    psensor= &boson640;

    // temporary solution
    pcfg = &pSensorPortConfig[sensor_port];
    name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR);

    dc = xi2c_dev_get(name);
    if (dc){
    	dev_info(g_dev_ptr,"{%d} setting i2c_addr to 0x%02x\n",sensor_port,dc->slave7);
    	//pr_info("{%d} Setting i2c_addr to 0x%02x\n",sensor_port,dc->slave7);
        psensor->i2c_addr = dc->slave7;
    }

    // temporarily - setting exact delays for clock and phases
    set_initial_phase(sensor_port, sensor, thispars, prevpars, frame16);
    // activate + deactivate resets, Boson starts to boot (some 12 seconds). Should be no "i2c" sequencer commands
    // or register reads until fully booted
    boson640_reset_release      (sensor_port);
    // temporarily - set delays

    // Sensor recognized, go on
    //  memcpy(&sensor, psensor, sizeof(mt9p001)); // copy sensor definitions
    memcpy(sensor, psensor, sizeof(boson640)); // copy sensor definitions
    //  MDD1(dev_dbg(g_dev_ptr,"sensor=0x%x\n", (int)sensor));
    dev_dbg(g_dev_ptr,"{%d} copied %d bytes of sensor static parameters\n",sensor_port,sizeof(boson640));
    add_sensor_proc(sensor_port,onchange_detectsensor,&boson640_pgm_detectsensor); // detect sensor type, sets sensor structure (capabilities), function pointers NOTE: will be called directly, not through pointers
    add_sensor_proc(sensor_port,onchange_initsensor,  &boson640_pgm_initsensor);   // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    add_sensor_proc(sensor_port,onchange_sensorin,    &boson640_pgm_sensorin);     // see mt9f002
    add_sensor_proc(sensor_port,onchange_exposure,    &boson640_pgm_exposure);     // program exposure
    add_sensor_proc(sensor_port,onchange_window,      &boson640_pgm_window);       // program sensor WOI and mirroring (flipping)
    add_sensor_proc(sensor_port,onchange_window_safe, &boson640_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    add_sensor_proc(sensor_port,onchange_limitfps,    &boson640_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    add_sensor_proc(sensor_port,onchange_gains,       &boson640_pgm_gains);        // program analog gains
    add_sensor_proc(sensor_port,onchange_triggermode, &boson640_pgm_triggermode);  // program sensor trigger mode
    add_sensor_proc(sensor_port,onchange_sensorregs,  &boson640_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?

    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType); // should cause other actions - delay for Boson?
    setFramePar(sensor_port, thispars, P_BOOTED,  0);

    setFramePar(sensor_port, thispars, P_COLOR, COLORMODE_RAW);
    setFramePar(sensor_port, thispars, P_BITS,  16);

    common_pars->sensors[sensor_port] =  sensor->sensorType;
    // reset is inactive, acquisition is still off
    dev_info(g_dev_ptr,"boson640_pgm_detectsensor()#%d ->  = %ld\n",sensor_port, sensor->sensorType);
    return sensor->sensorType;
}

/** Reset and initialize sensor
 * resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c - 353 only)
 * i2c is supposed to be already programmed.
 */
int boson640_pgm_initsensor     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    struct frameparspair_t pars_to_update[10];  // for all the sensor registers. Other P_* values will reuse the same ones
#if 0
    int reg;
    ///x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    ///x393_sensio_tim1_t uart_ctl =   {.d32=0};
    u32                data32;
    int i; // ,color;
    int rslt; // ,color;
#endif
    int nupdate=0;

    dev_info(g_dev_ptr,"boson640_pgm_initsensor(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    if (frame16 >= 0) return -1; // should be ASAP
    // Was disabled
  #if 0
    // reset both sequencers to frame 0
    sequencer_stop_run_reset(sensor_port, SEQ_CMD_RESET);
    sequencer_stop_run_reset(sensor_port, SEQ_CMD_RUN);  // also programs status update
    i2c_stop_run_reset      (sensor_port, I2C_CMD_RUN);  // also programs status update
  #endif

    // Enable extif (Boson UART CCI from the sequencer)
    uart_extif_en[sensor_port] = 1;
	set_sensor_uart_ctl_boson (sensor_port, uart_extif_en[sensor_port], -1, -1, 0, 0);
    dev_info(g_dev_ptr,"boson640_pgm_initsensor(): {%d}  frame16=%d frame=%ld: Enable EXTIF (Boson UART control from the sequencer)\n",sensor_port,frame16,getThisFrameNumber(sensor_port));

    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
        dev_info(g_dev_ptr,"boson640_pgm_initsensor(): {%d}  frame16=%d frame=%ld ABORTED as Boson is not yet booted\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    	return -1; // Not yet fully booted.
    }


//    boson640_par2addr

// read shadows from the sensor, ignore errors - kernel panic in boson640_read_reg()
#if 0
    for (reg = 0; reg < (sizeof(boson640_par2addr)-1)/2; reg++) { // unsigned short data!
    	data32=0;
    	rslt = boson640_read_reg(sensor_port, i, &data32); // ignore errors
        dev_dbg(g_dev_ptr,"{%d} Reading Boson register (index= %d), data = 0x%x, rslt = %d\n",
        		sensor_port, reg, data32, rslt);
        sensor_reg_copy[sensor_port][reg] = data32; // discarding bits above u16
    }
#endif

    // apply required register overwrites

  #if 0
    // Can not read shadows in the future, so write only. If needed more registers - manually read defaults
    // consider only updating shadows (no actual i2c commands) if the defaults are OK
    dev_dbg(g_dev_ptr,"{%d} Programming Lepton registers to take effect during next frame (frame 1 = 0x%x), now 0x%x, frame16=0x%x\n",
    		sensor_port, first_frame,  GLOBALPARS(sensor_port,G_THIS_FRAME), frame16  );
// GLOBALPARS(p,G_THIS_FRAME)
// thisFrameNumber(sensor_port)
    // set to empty, nothing will be done for registers (Boson should wait some 70 frames) before communicating

    for (i=0; i< sizeof(boson640_inits)/ 4;i++ ) { // unconditionally set those registers NOTE: Should be < 63 of them!
    	// set in immediate mode (with waits), update shadows
//    	SET_BOSON640_PAR_IMMED(sensor_port, sensor->i2c_addr, boson640_inits[2*i], boson640_inits[2*i + 1],wait_ms);
    	// first wtritten, 3rd - not. ANd no images - trying 1 command/frame
   	    SET_BOSON640_PAR_NOWAIT(sensor_port, first_frame, boson640_inits[2*i], boson640_inits[2*i + 1]); // assuming frame reset, so next frame == 1
        dev_dbg(g_dev_ptr,"{%d}   ****SET_BOSON640_PAR_NOWAIT(0x%x,0x%x,0x%x,0x%x)\n",sensor_port,sensor_port, first_frame, P_SENSOR_REGS+boson640_inits[2*i],boson640_inits[2*i+1]);
        sensor_reg_copy[sensor_port][boson640_inits[2*i]] = boson640_inits[2*i + 1];
    }
  #endif


    // next was for lepton
#if 0
	pars_to_update[nupdate  ].num= P_OVERSIZE; // does not work? - comes from autocampars
    pars_to_update[nupdate++].val= 1;
	pars_to_update[nupdate  ].num= P_WOI_HEIGHT;
#endif


/*
    if (sensor_reg_copy[sensor_port][P_BOSON640_TELEN]) {
    	sensio_ctl.vsync_use =    0;
        dev_dbg(g_dev_ptr,"Using %ld rows frame to include telemetry data on port=%d\n",boson640.arrayHeight, sensor_port);
        pars_to_update[nupdate++].val= boson640.arrayHeight;
        sensio_ctl.telemetry =    1;
    } else {
        dev_dbg(g_dev_ptr,"Using just %ld rows frame, do not include telemetry data on port=%d\n",boson640.clearHeight, sensor_port);
        pars_to_update[nupdate++].val= boson640.clearHeight;
        sensio_ctl.telemetry =    0;
    }
    x393_sensio_ctrl(sensio_ctl,sensor_port);
*/



    /*
     * boson640.arrayHeight
boson640.clearHeight
     *
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    // NC393: both sequencers started in pgm_detectsensor
    if (debug_delays & 0xff00) {
        udelay ((debug_delays >> 8) & 0xff); // 100);
    }
    first_sensor_i2c=sensor->i2c_addr;
    // TODO: get rid of first_sensor_i2c, leave only mux index
    if (GLOBALPARS(sensor_port, G_SENS_AVAIL)) {
        first_sensor_i2c+= I2C359_INC * ((GLOBALPARS(sensor_port, G_SENS_AVAIL) & 1)?1:((GLOBALPARS(sensor_port, G_SENS_AVAIL) & 2)?2:3));
    }
    dev_dbg(g_dev_ptr,"Reading sensor (port=%d) registers to the shadows, sa7=0x%x:\n",sensor_port,first_sensor_i2c);
    first_sensor_sa7[sensor_port] = first_sensor_i2c;
    for (i=0; i<256; i++) { // read all registers, one at a time (slower than in 353)
        X3X3_I2C_RCV2(sensor_port, first_sensor_i2c, i, &(i2c_read_data_dw[i]));
    }
    dev_dbg(g_dev_ptr,"Read 256 registers (port=%d) ID=0x%x:\n",sensor_port,i2c_read_data_dw[0]);
    for (i=0; i<256; i++) { // possible to modify register range to save (that is why nupdate is separate from i)
        regval=i2c_read_data_dw[i];
        regnum=P_SENSOR_REGS+i;
        SETFRAMEPARS_SET(regnum,regval);
        if ((mreg=MULTIREG(sensor_port,regnum,0))) for (j=0;j<MAX_SENSORS; j++) {
            SETFRAMEPARS_SET(mreg+j,regval);
        }
    }
    for (i=0;i<256;i++) {
        sensor_reg_copy[sensor_port][i] = i2c_read_data_dw[i];
    }

//    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    if (nupdate)  setFrameParsStatic(sensor_port, nupdate, pars_to_update);  // save changes to sensor register shadows for all frames



    dev_dbg(g_dev_ptr,"Initializing BOSON640 registers with default values:\n");
	sensor_register_overwrites=         (unsigned short *) &boson640_inits; // why casting is needed?
	sensor_register_overwrites_number=  sizeof(boson640_inits)/4;
    //  enable hardware i2c - NOTE: the only place where the i2c controller is enabled.
//    dev_dbg(g_dev_ptr,"Starting hardware sequencers\n");
    nupdate=0;  // Second pass over the registers to set
//#define SET_SENSOR_MBPAR(p,f,s,r,v)



    for (i=0; i<sensor_register_overwrites_number;i++ ) { // unconditionally set those registers NOTE: Should be < 63 of them!
        SET_SENSOR_MBPAR_LUT(sensor_port,
                         frame16, // == -1 (immediate)
                         sensor_register_overwrites[2*i],
                         sensor_register_overwrites[2*i+1]);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x,0x%x,0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) sensor_register_overwrites[2*i], (int) sensor_register_overwrites[2*i+1]);

    }
    SETFRAMEPARS_SET(P_GAIN_MIN, (sensor->minGain256)<<8); // less than that may not saturate sensor and confuse autoexposure/white balancing
    SETFRAMEPARS_SET(P_GAIN_MAX, (sensor->maxGain256)<<8);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows

    // G_* parameters - can write directly
    for (color=0;color<4;color++){
        for (i=0;i<81;i++) {
            GLOBALPARS(sensor_port, G_SENSOR_CALIB+(color<<8)+i)=(i>32)?((i -16)<<14): (i<<13); // one extra
        }
    }
    MDF4(for (i=0; i<1023; i++) {if ((i & 0x1f)==0) dev_dbg(g_dev_ptr,"\n"); dev_dbg(g_dev_ptr," 0x%06lx",GLOBALPARS (sensor_port, G_SENSOR_CALIB+i));});
    */

   if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}



void boson640_reset_release     (int sensor_port)               ///< sensor port number (0..3)
{
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    x393_sensio_tim1_t uart_ctl =   {.d32=0};
//    int i; // ,color;

    sensio_ctl.mrst =            0; // active
    sensio_ctl.mrst_set =        1;
    sensio_ctl.mmcm_rst =        1;
    sensio_ctl.mmcm_rst_set =    1;
    sensio_ctl.gp0 =             0; // input (tristate)
    sensio_ctl.gp0_set =         1;
    sensio_ctl.gp1 =             0; // input (tristate)
    sensio_ctl.gp1_set =         1;
    sensio_ctl.gp2 =             0; // input (tristate)
    sensio_ctl.gp2_set =         1;
    sensio_ctl.gp3 =             0; // input (tristate)
    sensio_ctl.gp3_set =         1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    uart_ctl.uart_extif_en =     0;
    uart_ctl.uart_extif_en_set = 1;
    uart_ctl.uart_xmit_rst =     1;
    uart_ctl.uart_xmit_rst_set = 1;
    uart_ctl.uart_recv_rst =     1;
    uart_ctl.uart_recv_rst_set = 1;
    set_x393_sensio_tim1(uart_ctl, sensor_port);

    udelay1000(1); // wait 1 ms

    // release master reset and all other resets
    sensio_ctl.mrst =            1; // deassert
    sensio_ctl.mrst_set =        1;
    sensio_ctl.mmcm_rst =        0;
    sensio_ctl.mmcm_rst_set =    1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    uart_ctl.uart_xmit_rst =     0;
    uart_ctl.uart_xmit_rst_set = 1;
    uart_ctl.uart_recv_rst =     0;
    uart_ctl.uart_recv_rst_set = 1;
    uart_ctl.uart_extif_en =     0; // enable extif
    uart_ctl.uart_extif_en_set = 1;
    set_x393_sensio_tim1(uart_ctl, sensor_port);
}




/**
 * Program sensor input in FPGA (Bayer, 8/16 bits, FPN, test mode, number of lines).
 */
int boson640_pgm_sensorin (int sensor_port,               ///< sensor port number (0..3)
		struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
		struct framepars_t * thispars, ///< sensor current parameters
		struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
		int frame16)                   ///< 4-bit (hardware) frame number parameters should
									   ///< be applied to,  negative - ASAP
									   ///< @return OK - 0, <0 - error
{
//    dev_dbg(g_dev_ptr,"boson640_pgm_sensorin(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
    dev_dbg(g_dev_ptr,"boson640_pgm_sensorin(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
	#if 0
	x393_sensio_width_t   sensio_width = {.d32=0};

    int cframe16 = getThisFrameNumber(sensor_port);

    dev_dbg(g_dev_ptr,"{%d} s_i cframe16=0x%08x frame16=%d\n",sensor_port,cframe16,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame

	// mt9f002 needed MT9F002_VACT_DELAY here, but mt9x001 needs 0 - has no real effect probably
	if (!(thispars->pars[P_FRAMESYNC_DLY] & 0x10000)){
		sensio_width.sensor_width = 0;
		X393_SEQ_SEND1(sensor_port, frame16, x393_sensio_width, sensio_width);
	}
#endif
	return 0;
}


/** Program sensor WOI and mirroring
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands
 * As different sensors may produce "bad frames" for different WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped)*/
int boson640_pgm_window     (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"boson640_pgm_window(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    return boson640_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Program sensor WOI and mirroring in safe mode (now does the same as boson640_pgm_window)
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands  */
int boson640_pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                       ///< be applied to,  negative - ASAP
                                       ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"boson640_pgm_window_safe(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    return boson640_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** PCommon part of programming sensor WOI */
int boson640_pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    int ww,wh; // ,wl,wt;
    int need_telemetry;
//    x393_sensio_ctl_t sensio_ctl = {.d32=0};

    struct frameparspair_t  pars_to_update[8];
    int nupdate=0;
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(): {%d} thispars->pars[P_COLOR]=%ld thispars->pars[P_BITS]=%ld\n", sensor_port,thispars->pars[P_COLOR], thispars->pars[P_BITS]);
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(entry): {%d} thispars->pars[P_WOI_HEIGHT]=%ld thispars->pars[P_WOI_WIDTH]=%ld\n", sensor_port,thispars->pars[P_WOI_HEIGHT], thispars->pars[P_WOI_WIDTH]);
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(entry): {%d} thispars->pars[P_SENSOR_PIXV]=%ld thispars->pars[P_SENSOR_PIXH]=%ld\n", sensor_port,thispars->pars[P_SENSOR_PIXV], thispars->pars[P_SENSOR_PIXH]);

    ww =sensor->imageWidth;
    wh = thispars->pars[P_SENSOR_PIXV];
    if (wh > sensor-> imageHeight){ // 512
    	wh = sensor -> arrayHeight; // with telemetry, 513
        need_telemetry = 1;
    } else {
    	wh = sensor -> imageHeight; // without telemetry
        need_telemetry = 0;
    }
    if (unlikely(thispars->pars[P_SENSOR_PIXH] != ww)) { // correct window width if needed
        SETFRAMEPARS_SET(P_SENSOR_PIXH, ww);
        SETFRAMEPARS_SET(P_WOI_WIDTH, ww);
    }
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common() {%d}   wh=0x%x thispars->pars[P_SENSOR_PIXV]=0x%lx\n",sensor_port, wh, thispars->pars[P_SENSOR_PIXV]);
    // if window hight = 513 - enable telemetry, if 512 - disable
    if (unlikely(thispars->pars[P_SENSOR_REGS + P_BOSON_TELEMETRY] != need_telemetry)) {
        SETFRAMEPARS_SET(P_SENSOR_PIXV, wh); // probably already set by pgm_window_common
        SETFRAMEPARS_SET(P_WOI_HEIGHT, wh); // probably already set by pgm_window_common
//        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_BOSON_TELEMETRY, need_telemetry); // set sensor register and parameter
        SET_SENSOR_PAR_LUT  (sensor_port, frame16, P_BOSON_TELEMETRY, need_telemetry); // set sensor register and parameter
        dev_dbg(g_dev_ptr,"boson640_pgm_window_common() SET_SENSOR_PAR_LUT(%d, %d, P_BOSON_TELEMETRY, %d)\n",sensor_port, frame16, need_telemetry);
    }
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(exit): {%d} thispars->pars[P_WOI_HEIGHT]=%lx thispars->pars[P_WOI_WIDTH]=%lx\n", sensor_port,thispars->pars[P_WOI_HEIGHT], thispars->pars[P_WOI_WIDTH]);
    dev_dbg(g_dev_ptr,"boson640_pgm_window_common(exit): {%d} thispars->pars[P_SENSOR_PIXV]=%lx thispars->pars[P_SENSOR_PIXH]=%lx\n", sensor_port,thispars->pars[P_SENSOR_PIXV], thispars->pars[P_SENSOR_PIXH]);



/*// from Lepton
    if (unlikely(thispars->pars[P_SENSOR_REGS + P_BOSON640_TELEN] != need_telemetry)) {
//    if (unlikely(thispars->pars[P_SENSOR_PIXV] != wh)) {
        SETFRAMEPARS_SET(P_SENSOR_PIXV, wh); // probably already set by pgm_window_common
        SETFRAMEPARS_SET(P_WOI_HEIGHT, wh); // probably already set by pgm_window_common

        SET_BOSON640_PAR_NOWAIT(sensor_port, frame16, P_BOSON640_TELEN, need_telemetry);
        sensio_ctl.telemetry =     (wh > sensor -> imageHeight);
        sensio_ctl.telemetry_set = 1; // change it later by WINDOEW_HEIGHT?
        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_ctrl, sensio_ctl);
        dev_dbg(g_dev_ptr," X393_SEQ_SEND1 (%d, 0x%x, x393_sensio_ctrl,  0x%x)\n",sensor_port, frame16, (int) sensio_ctl.d32);
    }
*/
    // Margins - set 0
    // Program sensor left margin
    if (unlikely(thispars->pars[P_WOI_LEFT] != 0)) { // correct window width if needed
        SETFRAMEPARS_SET(P_WOI_LEFT, 0);
    }
    // Program sensor top margin
    if (unlikely(thispars->pars[P_WOI_TOP] != 0)) { // correct window width if needed
        SETFRAMEPARS_SET(P_WOI_TOP, 0);
    }
//    dev_dbg(g_dev_ptr,"{%d}   wl =0x%x, wt=0x%x, ww=0x%x, wh=0x%x, compressor_margin=0x%x\n",sensor_port, wl, wt, ww, wh, compressor_margin);
    dev_dbg(g_dev_ptr,"{%d}   ww =0x%x, wh=0x%x\n",sensor_port, ww, wh);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return 0;
}





/** Check if compressor can keep up, limit sensor FPS if needed
 * FPS is limited by increasing verical blanking, it can not be be made too big, so this method does not work to make time lapse rate. horisontal blanking
 * is kept at minimum (to reduce ERS effect) if not specified. If it is specified (>minimal) - it will be used instead.
 * calculate line period.
 *
 * FIXME - uses P_VIRTUAL_WIDTH w/o decreasing it when changing image size? Replace VIRT_WIDTH with HOR_BANK?
 * Or require always set them to zero when chnaging WOI?
 * FIXME for multisensor - needs per-sensor individual parameters. This uses sensor registers, not the general parameters (i.e. height) */
int boson640_pgm_limitfps   (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
	// nothing to do here?
    struct frameparspair_t pars_to_update[2]; // maximum 7 registers updated (need to recount)
    int nupdate=0;
//    dev_dbg(g_dev_ptr,"boson640_pgm_limitfps(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    dev_dbg(g_dev_ptr,"boson640_pgm_limitfps(): {%d}  frame16=%d frame=%ld, P_COLOR =%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port),thispars->pars[P_COLOR]);
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
    SETFRAMEPARS_SET(P_FP1000S, 60000);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}

/** Program sensor exposure */
int boson640_pgm_exposure       (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
//    dev_dbg(g_dev_ptr,"boson640_pgm_exposure(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    dev_dbg(g_dev_ptr,"boson640_pgm_exposure(): {%d}  frame16=%d frame=%ld, P_COLOR =%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port),thispars->pars[P_COLOR]);
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
	// nothing to do here
    return 0;
}

///#define SHIFT_DGAIN 1 // shift digital gain right, so 1.0 is 0x8000 an the full range is 4:1 - make it a parameter?


/** Program analog gains
 * program analog gains TODO: Make separate read-only P_ACTUAL_GAIN** ?
 * apply sensor-specific restrictions on the allowed gain values
 * includes sensor test mode on/off/selection */
//(FRAMEPAR_MODIFIED(P_VEXPOS)
//#define CSCALES_CTL_NORMAL  0 // USE P_*SCALE as is
//#define CSCALES_CTL_RECALC  1 // Recalculate P_*SCALE from P_GAIN*, P_GAING, then use it (will change to CSCALES_CTL_NORMAL when applied)
//#define CSCALES_CTL_FOLLOW  2 // Don't apply P_*SCALE to P_GAIN*, but update it from the current P_*SCALE from P_GAIN*/P_GAING
//#define CSCALES_CTL_DISABLE 3 // Disable P_*SCALE - don't apply P_*SCALE to P_GAIN*, don't update P_*SCALE from P_GAIN*, P_GAING
///#define MAX_DIGITAL_GAIN 0x300 //integer x256 (0x300 ~3.0)
int boson640_pgm_gains      (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error

{
    dev_dbg(g_dev_ptr,"boson640_pgm_gains(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
	// nothing to do here - add high/low gain of Boson?
    return 0;
}

/** Program trigger mode as sensor-specific  */
int boson640_pgm_triggermode        (int sensor_port,               ///< sensor port number (0..3)
                                    struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                    struct framepars_t * thispars, ///< sensor current parameters
                                    struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                    int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                                   ///< be applied to,  negative - ASAP
                                                                   ///< @return 0 - OK, negative - error
{
	///TODO: Implement for Boson
    struct frameparspair_t pars_to_update[4]; ///
    int nupdate=0;
    dev_dbg(g_dev_ptr,"boson640_pgm_triggermode(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
    if (thispars->pars[P_TRIG] & 4) { // turn external
        dev_dbg(g_dev_ptr,"boson640_pgm_triggermode(): {%d}  frame16=%d frame=%ld TURN EXTERNAL (%ld)\n",sensor_port,frame16,getThisFrameNumber(sensor_port),thispars->pars[P_TRIG]);
    	// always apply P_BOSON_DVO_DISPLAY_MODE
//    	if (thispars->pars[P_SENSOR_REGS + P_BOSON_DVO_DISPLAY_MODE] != 1){
    		SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_BOSON_DVO_DISPLAY_MODE, 1);
//    	}
    	if (thispars->pars[P_SENSOR_REGS + P_BOSON_BOSON_EXT_SYNC_MODE] != 2){
    		SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_BOSON_BOSON_EXT_SYNC_MODE, 2);
    	}
    } else { // turn internal
        dev_dbg(g_dev_ptr,"boson640_pgm_triggermode(): {%d}  frame16=%d frame=%ld TURN INTERNAL (%ld)\n",sensor_port,frame16,getThisFrameNumber(sensor_port),thispars->pars[P_TRIG]);
    	if (thispars->pars[P_SENSOR_REGS + P_BOSON_BOSON_EXT_SYNC_MODE] != 0){
    		SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_BOSON_BOSON_EXT_SYNC_MODE, 0);
    	}
    }
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
//P_BOSON_DVO_DISPLAY_MODE : 0 - continuous, 1 - one-shot (can be used always?) - try it manually
//P_BOSON_BOSON_EXT_SYNC_MODE: 0 - internal, 2 - external (treat all !=0 as 2)
// if external - require one-shot, if continuous - allow any

}

/** Program sensor registers (probably just those that are manually set)
 * NOTE: all modes but ASAP are limited to 64 registers/frame, no overflow checks are performed! */

int boson640_pgm_sensorregs     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
    unsigned long bmask32= ((thispars->mod32) >> (P_SENSOR_REGS>>5)) & (( 1 << (P_SENSOR_NUMREGS >> 5))-1) ;
    unsigned long mask;
    int index,index32, reg_index;
    int I, ADDR;
    struct frameparspair_t pars_to_update[sizeof(boson640_par2addr)/sizeof(short)/2]; ///
    int nupdate=0; // No need to update any parameter w/o multiplexer !
    dev_dbg(g_dev_ptr,"boson640_pgm_sensorregs(): {%d}  frame16=%d frame=%ld\n",sensor_port,frame16,getThisFrameNumber(sensor_port));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (boson640_is_booted (sensor_port, thispars) < 2){ // not yet booted
    	return -1; // Not yet fully booted.
    }
    dev_dbg(g_dev_ptr,"{%d}  bmask32=0x%lx, thispars->mod32=0x%lx, P_SENSOR_REGS=0x%x, P_SENSOR_NUMREGS=0x%x\n",sensor_port,bmask32,thispars->mod32,P_SENSOR_REGS,P_SENSOR_NUMREGS);
//    dev_info(g_dev_ptr,"{%d}  bmask32=0x%lx, thispars->mod32=0x%lx, P_SENSOR_REGS=0x%x, P_SENSOR_NUMREGS=0x%x\n",sensor_port,bmask32,thispars->mod32,P_SENSOR_REGS,P_SENSOR_NUMREGS);
	if (bmask32) {
		for (index32=(P_SENSOR_REGS>>5); bmask32; index32++, bmask32 >>= 1) {
			dev_dbg(g_dev_ptr,"{%d}  index32=0x%x, bmask32=0x%lx\n",sensor_port,index32,bmask32);
			if (bmask32 & 1) {
				mask=thispars->mod[index32];
				dev_dbg(g_dev_ptr,"{%d}  mask=0x%lx\n",sensor_port,mask);
				for (index=(index32<<5); mask; index++, mask >>= 1) {
					reg_index = (index-P_SENSOR_REGS);
					if (mask & 1) {
                        dev_dbg(g_dev_ptr,"Boson640 {%d} X3X3_I2C_SEND2_LUT(0x%x, 0x%x, 0, 0x%lx,0x%x,0x%lx)\n",sensor_port,sensor_port, frame16,sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]);
//                        dev_info(g_dev_ptr,"Boson640 {%d} X3X3_I2C_SEND2_LUT(0x%x, 0x%x, 0, 0x%lx,0x%x,0x%lx)\n",sensor_port,sensor_port, frame16,sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]);
//                        X3X3_I2C_SEND2(sensor_port, frame16, sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]);
//						X3X3_I2C_SEND2_LUT(sensor_port,frame16,0,(index-P_SENSOR_REGS),thispars->pars[index]);
						I = pSensorPortConfig[sensor_port].broadcast_addr;
						ADDR = pSensorPortConfig[sensor_port].par2addr[I][(index-P_SENSOR_REGS)];\
                        dev_dbg(g_dev_ptr,"Boson640 {%d} pSensorPortConfig[sensor_port].broadcast_addr=0x%x \n",sensor_port, I);
                        dev_dbg(g_dev_ptr,"Boson640 {%d} pSensorPortConfig[(p)].par2addr[_I][(r)] = 0x%x \n",sensor_port,ADDR);

                        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, (index-P_SENSOR_REGS), thispars->pars[index]); // set sensor register and parameter
					}
				}
				thispars->mod[index32]=0;
			}
		}
		thispars->mod32=0;
	}
//    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // No need to update anything w/o multiplexer and broadcast
    //  send all parameters marked as "needed to be processed" to the sensor, clear those flags
    // mask out all non sensor pars
    //  unsigned long bmask32= ((thispars->mod32) >> (P_SENSOR_REGS>>5)) & (P_SENSOR_NUMREGS-1) ;
    // It will be the first for the frame (before automatic sensor changes).
    // Add testing for programmed sensor and move values to later frames (not here butin the pgm_functions)
	// nothing to do here now, use similar approach to program non-i2c Lepton registers
    return 0;
}

//static short sensor_reg_copy[SENSOR_PORTS][256]; // Read all 256 sensor registers here - during initialization and on demand
//                                                 // Later may increase to include multiple subchannels on 10359

// SysFS interface to Boson640
#define SYSFS_PERMISSIONS           0644 /* default permissions for sysfs files */
#define SYSFS_READONLY              0444
#define SYSFS_WRITEONLY             0222

/** Sysfs helper function - get channel number from the last character of the attribute name*/
static int get_channel_from_name(struct device_attribute *attr) ///< Linux kernel interface for exporting device attributes
                                                                ///< @return channel number
{
    int reg = 0;
    sscanf(attr->attr.name + (strlen(attr->attr.name)-1), "%du", &reg);
    return reg;
}
/**
 * Sets transmit sequence number
 */
static ssize_t store_uart_seq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int ni;
    ni = sscanf(buf,"%i", uart_seq_number+chn);
    if (ni < 0) {
    	return ni; // error
    }
    return count;
}
/**
 * Prints received sequence number
 */
static ssize_t show_uart_seq(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn =  get_channel_from_name(attr);
    return sprintf(buf,"%d\n", uart_rec_seq_number[chn]);
}

/**
 * Enables/disables uart commands from the sequencer
 */
static ssize_t store_uart_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int ni;
    ni = sscanf(buf,"%i", uart_status+chn);
    if (ni < 0) {
    	return ni; // error
    }
    return count;
}

/**
 * Prints received status
 */
static ssize_t show_uart_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn =  get_channel_from_name(attr);
    return sprintf(buf,"%d\n", uart_status[chn]);
}

/**
 * Prints received status
 */
static ssize_t show_uart_cs16(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn =  get_channel_from_name(attr);
    return sprintf(buf,"%d\n", uart_cs16[chn]);
}


/**
 * Enables/disables uart commands from the sequencer
 */
static ssize_t store_extif(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int ni;
    int en;
    ni = sscanf(buf,"%i", &en);
    if (ni < 0) {
    	return ni; // error
    } else if (ni > 0) {
    	set_sensor_uart_set_extif(chn, en); // Will set shadow too
    }
    return count;
}

/**
 * Prints extif enabled status
 */
static ssize_t show_extif(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn =  get_channel_from_name(attr);
    return sprintf(buf,"%d\n", uart_extif_en[chn]);
}


/**
 * Enables/disables uart commands from the sequencer
 */
static ssize_t store_uart_packet(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int i, ni, np, nb;
    u32 command;
    u32 data32;
    int d;
    char char_nb; // 0,1,2,4 bytes or 'b' for byte sequence
    char pair[2]; // a pair of characters for next byte (or '0x')
    int rslt;
///    char * cp;
//    dev_dbg(g_dev_ptr,"store_uart_packet(): {%d}\n", chn);
    ni = sscanf(buf,"%i %c %n", &command, &char_nb, &np); // last is a pointer to number of consumed characters
    dev_dbg(g_dev_ptr,"store_uart_packet(): {%d} command = 0x%x result = %d, consumed_bytes = %d\n", chn, command, ni, np);
    if (ni < 0) {
    	return ni; // error
    } else if (ni <2) {
    	return -EILSEQ;
    }
    switch (char_nb){
    case '0':
    case '1':
    case '2':
    case '4':
    	nb = char_nb - '0';
    	break;
    case 'b':
    case 'B':
    	nb = -1;
    	break;
    default:
    	return -EINVAL;
    }
    if (nb >= 0){
        ni = sscanf(buf+np,"%i", &data32);
        for (i = 0; i < nb; i++){
        	uart_data[chn][i] = (data32 >> (8 * (nb - i -1))) & 0xff;
        }
    } else {
    	nb = 0;
    	for (; nb < BOSON_PACKET_MAX_DATA; nb++){
    		ni = sscanf(buf+np,"%2c", pair);
    		if (ni < 1){
    			break;
    		}
    		if ((nb == 0) && (pair[0] == '0') && ((pair[1] == 'x') || (pair[1] == 'X'))){
    			nb --; // skip optional leading 0x
    		} else {
        		ni = sscanf(pair,"%2x", &d);
        		if (ni < 1){
        			return -EILSEQ; // not a hex byte
        		}
        		uart_data[chn][nb] = d;
    		}
    		np+=2;
    	}
    }
    // reset uart transmit and receive
    set_sensor_uart_ctl_boson (chn, -1, 1, 1, 0, 0);
    set_sensor_uart_ctl_boson (chn, -1, 0, 0, 0, 0);

    dev_dbg(g_dev_ptr,"store_uart_packet(): {%d} command = 0x%x nb = %d\n", chn, command, nb);
    uart_send_packet          (chn, command, nb, uart_data[chn]);
//    if ((rslt = uart_wait_transmit (chn)) < 0){ // wait transmit done, return on timeout
//    	return rslt;
//    }
    if ((rslt = uart_wait_receive (chn)) < 0){ // wait transmit done, return on timeout
    	return rslt;
    }
    dev_dbg(g_dev_ptr,"store_uart_packet(): {%d} returned %d\n", chn, rslt);

    uart_length[chn]= uart_receive_packet (chn,
                                           0, // u8 *   channel,      // in not null, will return packet channel
							               uart_rec_seq_number + chn, // u32 *  sequence, ///< in not null, will return sequence number
							               &command,                   // u32 *  command,  ///< in not null, will return command number
							               uart_status + chn,         // if not null, will status
										   -1,                        // limited by  BOSON_PACKET_MAX_DATA only
							               uart_data[chn],            // if not null, will return data (up to 756 bytes)
										   uart_cs16 +chn);

    dev_dbg(g_dev_ptr,"store_uart_packet(): uart_receive_packet() {%d} returned %d\n", chn, uart_length[chn]);

    if (uart_length[chn] < 0) {
    	switch (uart_length[chn]){
    	case -BOSON_ERR_NO_PACKET:  return -EPIPE;
    	case -BOSON_ERR_FBP_FORMAT: return -EINTR;
    	case -BOSON_ERR_NO_EOP:     return -EMSGSIZE;
    	default: return -EINVAL; // not yet possible
    	}
    	return uart_length[chn] ; // error
    }
    if (uart_status[chn]){
    	return -EPROTO; // non-zero status
    }
    return count;
}

/**
 * Prints received data (or status with 'ERR ' prefix if status !=0)
 */
static ssize_t show_uart_packet(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 d;
	int i;
    char * cp;
    int chn =  get_channel_from_name(attr);
    if (uart_status[chn]){
    	return sprintf(buf,"ERR %d (0x%x)\n", uart_status[chn], uart_status[chn]);
    } else if (uart_length[chn] < 1){
    	return sprintf(buf,"None\n");
    } else if (uart_length[chn] <= 4){
    	d = 0;
    	for (i = 0; i < uart_length[chn]; i++){
    		d <<= 8;
    		d+=uart_data[chn][i];
    	}
    	return sprintf(buf,"%d (0x%x)\n", d, d);
    }
    cp = buf;
    cp += sprintf(cp,"0x");
    for (i = 0; i < uart_length[chn]; i++){
    	cp += sprintf(cp,"%02x", uart_data[chn][i]);
    }
    cp += sprintf(cp,"\n");
    return cp - buf;
}



/**
 * Sysfs function - output instructions text of how to communicate with i2c devices attached to the sensor ports.
 * Output buffer *buf will contain multi-line instructions text
 * @param dev Linux kernel basic device structure
 * @param attr Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @return offset to the end of the output data in the *buf buffer
 */
static ssize_t get_uart_help(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Numeric suffix in the file names selects sensor port\n"
            "uart_seq:    read - last received sequence number, write - next transmit sequence number (will skip lower 0x10000 for extif)\n"
            "uart_status: read only - last received status\n"
            "uart_cs:     read only - last received checksum (16 bits)\n"
            "extif_en:    enable/disable extif (sequencer-generated uart commands)\n"
            "uart:        write command+data, read - last received data\n"
    );
}
//============================================== DRP-related functions (TODO: make common for other sensors) =================
/**
 * Read/write DRP register data
 * Format:
 * drp<i> address - read register, store result in  drp_read_data[i] for subsecuent show_drp<i> command
 * drp<i> address data - write data to register[address]
 * drp<i> address data mask - write data to register[address] using mask (1 - new, 0 - old)
 * @param dev Linux kernel basic device structure
 * @param attr  Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @param count length of data in the buffer buf
 * @return offset to the end of processed buffer or negative error
 */
static ssize_t store_drp_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int ni, daddr, data, mask, rslt;
    ni = sscanf(buf, "%i %i %i", &daddr, &data, &mask);
    if (ni < 1) {
        dev_err(dev, "Requires at least 1 parameter: register address (for read operation), 2 or 3 parameters (addr, data, mask) for write.\n");
        return -EINVAL;
    }
    if (ni == 1){
    	drp_open(chn);
    	data = drp_read_reg (chn, daddr);
    	drp_close(chn);
    	if (data <0) return data; // -ETIMEOUT
    	drp_read_data[chn] = data;
    } else {
    	if (ni <3) {
    		mask = 0xffff;
    	}
    	drp_open(chn);
    	rslt = drp_write_reg(chn, daddr, data, mask);
    	drp_close(chn);
    	if (rslt < 0) return rslt;
    }
    return count;
}

/**
 * Show value of the previously (by store_drp_register()) read DRP register data
 * @param dev Linux kernel basic device structure
 * @param attr  Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @return offset to the end of the output data in the *buf buffer
 */
static ssize_t show_drp_register(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn = get_channel_from_name(attr) ;
    return sprintf(buf,"%d\n",drp_read_data[chn]);
}

/**
 * Read/write DRP register data
 * Format:
 * drp_phase<i> clock - read clock phase, store result in  drp_read_phase[i] for subsecuent show_drp_phase<i> command
 * drp<i> clock data - write data to phase[clock]
 * @param dev Linux kernel basic device structure
 * @param attr  Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @param count length of data in the buffer buf
 * @return offset to the end of processed buffer or negative error
 */
static ssize_t store_drp_phase(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    int ni, clock_index, phase, rslt;
    char c;
    ni = sscanf(buf, "%c %i %i", &c, &clock_index, &phase);
    if (ni < 1) {
        dev_err(dev, "Requires at least 1 parameter: clock identifier (f,0,1,2,3,4,5,6) for read operation, 2 parameters (clock, phase) for write.\n");
        return -EINVAL;
    }
    switch (c){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    	clock_index = c - '0';
    	break;
    case 'f':
    case 'F':
    	clock_index = -1;
    	break;
    default:
        dev_err(dev, "Invalid clock identifier: %c. Valid are f,0,1,2,3,4,5,6.\n",c);
    	return -EINVAL; // invalid clock index
    }
    if (ni == 1){
    	drp_open(chn);
    	phase = drp_read_clock_phase (chn, clock_index);
    	drp_close(chn);
    	if (phase <0) return phase; // -ETIMEOUT
    	drp_read_phase[chn] = phase;
    } else {
    	drp_open(chn);
    	rslt = drp_write_clock_phase(chn, clock_index, phase);
    	drp_close(chn);
    	if (rslt < 0) return rslt;
    }
    return count;
}

/**
 * Show value of the previously (by store_drp_phase()) read DRP register data
 * @param dev Linux kernel basic device structure
 * @param attr  Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @return offset to the end of the output data in the *buf buffer
 */
static ssize_t show_drp_phase(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn = get_channel_from_name(attr) ;
    return sprintf(buf,"%d\n",drp_read_phase[chn]);
}

/**
 * Sysfs function - output instructions text of how to communicate with i2c devices attached to the sensor ports.
 * Output buffer *buf will contain multi-line instructions text
 * @param dev Linux kernel basic device structure
 * @param attr Linux kernel interface for exporting device attributes
 * @param buf 4K buffer to receive response
 * @return offset to the end of the output data in the *buf buffer
 */
static ssize_t get_drp_help(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Control MMCM/PLL clock phases and other register using DRP\n"
    		"Numeric suffix in the file names selects sensor port\n"
    		"drp_reg<n>:     read - last read DRP register\n"
    		"             write [[[drp_reg_addr] drp_reg_data] write_mask]\n"
    		"             address only - read DRP register (to be shown with drp_reg<n> read)\n"
    		"             address and data - write 16-bit DRP register\n"
    		"             address, data, and mask  - write only selected bits (ones) of the 16-bit DRP register\n"
    		"drp_phase<n> - contol clock phase in 1/8 of the VCO period, total 8 bits (max 31-7/8 of the VCO period)\n"
            "             clocks are identified as 'f' - CLKFBOUT, '0'..'6' - CLKOUT0...CLKOUT6"
    		"             read - last read DRP phaser\n"
    		"             write [[clock_id] phase]\n"
    		"             clock_id only - read clock phase (to be shown with drp_phase<n> read)\n"
    		"             clock_id and phase - write 8-bit clock phase\n"
    );
}



//============================================== end of sysfs functions =================

static DEVICE_ATTR(uart_seq0,               SYSFS_PERMISSIONS,     show_uart_seq,                store_uart_seq);
static DEVICE_ATTR(uart_seq1,               SYSFS_PERMISSIONS,     show_uart_seq,                store_uart_seq);
static DEVICE_ATTR(uart_seq2,               SYSFS_PERMISSIONS,     show_uart_seq,                store_uart_seq);
static DEVICE_ATTR(uart_seq3,               SYSFS_PERMISSIONS,     show_uart_seq,                store_uart_seq);
static DEVICE_ATTR(uart_status0,            SYSFS_PERMISSIONS,     show_uart_status,             store_uart_status);
static DEVICE_ATTR(uart_status1,            SYSFS_PERMISSIONS,     show_uart_status,             store_uart_status);
static DEVICE_ATTR(uart_status2,            SYSFS_PERMISSIONS,     show_uart_status,             store_uart_status);
static DEVICE_ATTR(uart_status3,            SYSFS_PERMISSIONS,     show_uart_status,             store_uart_status);
static DEVICE_ATTR(uart_cs0,SYSFS_PERMISSIONS & SYSFS_READONLY,    show_uart_cs16,               store_uart_status);
static DEVICE_ATTR(uart_cs1,SYSFS_PERMISSIONS & SYSFS_READONLY,    show_uart_cs16,               store_uart_status);
static DEVICE_ATTR(uart_cs2,SYSFS_PERMISSIONS & SYSFS_READONLY,    show_uart_cs16,               store_uart_status);
static DEVICE_ATTR(uart_cs3,SYSFS_PERMISSIONS & SYSFS_READONLY,    show_uart_cs16,               store_uart_status);
static DEVICE_ATTR(extif_en0,               SYSFS_PERMISSIONS,     show_extif,                   store_extif);
static DEVICE_ATTR(extif_en1,               SYSFS_PERMISSIONS,     show_extif,                   store_extif);
static DEVICE_ATTR(extif_en2,               SYSFS_PERMISSIONS,     show_extif,                   store_extif);
static DEVICE_ATTR(extif_en3,               SYSFS_PERMISSIONS,     show_extif,                   store_extif);
static DEVICE_ATTR(uart0,                   SYSFS_PERMISSIONS,     show_uart_packet,             store_uart_packet);
static DEVICE_ATTR(uart1,                   SYSFS_PERMISSIONS,     show_uart_packet,             store_uart_packet);
static DEVICE_ATTR(uart2,                   SYSFS_PERMISSIONS,     show_uart_packet,             store_uart_packet);
static DEVICE_ATTR(uart3,                   SYSFS_PERMISSIONS,     show_uart_packet,             store_uart_packet);
static DEVICE_ATTR(uart_help, SYSFS_PERMISSIONS & SYSFS_READONLY,  get_uart_help,                NULL);

static DEVICE_ATTR(drp_reg0,                SYSFS_PERMISSIONS,     show_drp_register,            store_drp_register);
static DEVICE_ATTR(drp_reg1,                SYSFS_PERMISSIONS,     show_drp_register,            store_drp_register);
static DEVICE_ATTR(drp_reg2,                SYSFS_PERMISSIONS,     show_drp_register,            store_drp_register);
static DEVICE_ATTR(drp_reg3,                SYSFS_PERMISSIONS,     show_drp_register,            store_drp_register);
static DEVICE_ATTR(drp_phase0,              SYSFS_PERMISSIONS,     show_drp_phase,               store_drp_phase);
static DEVICE_ATTR(drp_phase1,              SYSFS_PERMISSIONS,     show_drp_phase,               store_drp_phase);
static DEVICE_ATTR(drp_phase2,              SYSFS_PERMISSIONS,     show_drp_phase,               store_drp_phase);
static DEVICE_ATTR(drp_phase3,              SYSFS_PERMISSIONS,     show_drp_phase,               store_drp_phase);
static DEVICE_ATTR(drp_help, SYSFS_PERMISSIONS & SYSFS_READONLY,   get_drp_help,                 NULL);


static struct attribute *root_dev_attrs[] = {
        &dev_attr_uart_seq0.attr,
        &dev_attr_uart_seq1.attr,
        &dev_attr_uart_seq2.attr,
        &dev_attr_uart_seq3.attr,
        &dev_attr_uart_status0.attr,
        &dev_attr_uart_status1.attr,
        &dev_attr_uart_status2.attr,
        &dev_attr_uart_status3.attr,
        &dev_attr_uart_cs0.attr,
        &dev_attr_uart_cs1.attr,
        &dev_attr_uart_cs2.attr,
        &dev_attr_uart_cs3.attr,
        &dev_attr_extif_en0.attr,
        &dev_attr_extif_en1.attr,
        &dev_attr_extif_en2.attr,
        &dev_attr_extif_en3.attr,
        &dev_attr_uart0.attr,
        &dev_attr_uart1.attr,
        &dev_attr_uart2.attr,
        &dev_attr_uart3.attr,
        &dev_attr_uart_help.attr,
        &dev_attr_drp_reg0.attr,
        &dev_attr_drp_reg1.attr,
        &dev_attr_drp_reg2.attr,
        &dev_attr_drp_reg3.attr,
        &dev_attr_drp_phase0.attr,
        &dev_attr_drp_phase1.attr,
        &dev_attr_drp_phase2.attr,
        &dev_attr_drp_phase3.attr,
        &dev_attr_drp_help.attr,
        NULL
	};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_boson640_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

int boson640_init(struct platform_device *pdev)
{
//    int res;
    struct device *dev = &pdev->dev;
//    const struct of_device_id *match;
///    int sensor_port;
///    for (sensor_port = 0; sensor_port < SENSOR_PORTS; sensor_port++) {
///        first_sensor_sa7[sensor_port] = 0;
///    }
    elphel393_boson640_sysfs_register(pdev);
    dev_info(dev, DEV393_NAME(DEV393_BOSON640)": registered sysfs\n");
    g_dev_ptr = dev;
    return 0;
}

int boson640_remove(struct platform_device *pdev)
{
    unregister_chrdev(DEV393_MAJOR(DEV393_BOSON640), DEV393_NAME(DEV393_BOSON640));
    return 0;
}

static const struct of_device_id elphel393_boson640_of_match[] = {
    { .compatible = "elphel,elphel393-boson640-1.00" },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_boson640_of_match);

static struct platform_driver elphel393_boson640 = {
    .probe          = boson640_init,
    .remove         = boson640_remove,
    .driver         = {
        .name       = DEV393_NAME(DEV393_BOSON640),
        .of_match_table = elphel393_boson640_of_match,
    },
};

module_platform_driver(elphel393_boson640);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION("Driver for BOSON640 interface (Boson640)* in Elphel cameras");
