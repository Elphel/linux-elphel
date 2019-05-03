/***************************************************************************//**
 * @file      lepton.c
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

#define DEBUG // should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)
/****************** INCLUDE FILES SECTION ***********************************/
#include <linux/types.h> // for div 64
#include <asm/div64.h>   // for div 64

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

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h" // detect sensor
//#include "cci2c.h"
#include "lepton.h"
//#include "multi10359.h"
#include "framepars.h"      // parameters manipulation
#include "sensor_common.h"
#include "pgm_functions.h"
#include "x393.h"
#include "sensor_i2c.h"


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
const unsigned short lepton_par2addr[] = {
		P_LEPTON_POWER,       P_LEPTON_POWER,
		P_LEPTON_STATUS,      P_LEPTON_STATUS,
		P_LEPTON_COMMAND_ID,  P_LEPTON_COMMAND_ID,
		P_LEPTON_DATA_LENGTH, P_LEPTON_DATA_LENGTH,
		P_LEPTON_DATA00,      P_LEPTON_DATA00,
		P_LEPTON_DATA01,      P_LEPTON_DATA01,
		P_LEPTON_DATA02,      P_LEPTON_DATA02,
		P_LEPTON_DATA03,      P_LEPTON_DATA03,
		P_LEPTON_DATA04,      P_LEPTON_DATA04,
		P_LEPTON_DATA05,      P_LEPTON_DATA05,
		P_LEPTON_DATA06,      P_LEPTON_DATA06,
		P_LEPTON_DATA07,      P_LEPTON_DATA07,
		P_LEPTON_DATA08,      P_LEPTON_DATA08,
		P_LEPTON_DATA09,      P_LEPTON_DATA09,
		P_LEPTON_DATA10,      P_LEPTON_DATA10,
		P_LEPTON_DATA11,      P_LEPTON_DATA11,
		P_LEPTON_DATA12,      P_LEPTON_DATA12,
		P_LEPTON_DATA13,      P_LEPTON_DATA13,
		P_LEPTON_DATA14,      P_LEPTON_DATA14,
		P_LEPTON_DATA15,      P_LEPTON_DATA15,
		P_LEPTON_DATAF8,      P_REG_LEPTON_DATAF8,
		P_LEPTON_DATAF9,      P_REG_LEPTON_DATAF9,
		P_LEPTON_DATAFA,      P_REG_LEPTON_DATAFA,
		P_LEPTON_DATAFB,      P_REG_LEPTON_DATAFB,
		P_LEPTON_DATAFC,      P_REG_LEPTON_DATAFC,
		P_LEPTON_DATAFD,      P_REG_LEPTON_DATAFD,
		P_LEPTON_DATAFE,      P_REG_LEPTON_DATAFE,
		P_LEPTON_DATAFF,      P_REG_LEPTON_DATAFF,
// Registers that are not i2c (see if it will register pages)

        P_LEPTON_GP3VSYNC,    P_REG_LEPTON_GP3VSYNC, //  0x0854
        P_LEPTON_TELEN,       P_REG_LEPTON_TELEN,    //  0x0218
        P_LEPTON_TELLOC,      P_REG_LEPTON_TELLOC,   //  0x021c

		0xffff // END indicator
};



/**
 * get at least one parameter for a page
 */
const unsigned short lepton_pages[] = {
		P_LEPTON_POWER,
		P_REG_LEPTON_DATAF8,
		P_REG_LEPTON_DATAF9,
		P_REG_LEPTON_DATAFA,
		P_REG_LEPTON_DATAFB,
		P_REG_LEPTON_DATAFC,
		P_REG_LEPTON_DATAFD,
		P_REG_LEPTON_DATAFE,
		P_REG_LEPTON_DATAFF,
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
const unsigned short lepton_ahead_tab[] = // copied from mt9x001
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
void lepton_set_device(struct device *dev) // do nothing, now it has it's own device
{
    //g_dev_ptr = dev;
}


/** Capabilities of MT9M001 1.3 MPix */
struct sensor_t lepton35={
        // sensor constants
        .imageWidth  = 160,              ///< nominal image width for final images
        .imageHeight = 120,              ///< nominal image height for final images
        .clearWidth  = 160,              ///< maximal clear image width
        .clearHeight = 120,              ///< maximal clear image height;
        .clearTop    = 0,                ///< top margin to the first clear pixel
        .clearLeft   = 0,                ///< left margin to the first clear pixel
        .arrayWidth  = 160,              ///< total image array width (including black and boundary)
        .arrayHeight = 122,              ///< total image array height (including black and boundary)
        .minWidth    = 160,              ///< minimal WOI width
        .minHeight   = 120,              ///< minimal WOI height
        .minHorBlank = 0,                ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 160,              ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 160,              ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 0,                ///< minimal vertical blanking
        .maxVertBlank= 122,              ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 122,              ///< Maximal shutter duration (in lines)
        .flips       = 0,                ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 0,                ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 0,                ///< bayer shift for flips==0
        .dcmHor      = 0x0,              ///< available horizontal decimation values 1,2,4,8
        .dcmVert     = 0x0,              ///< available vertical decimation values 1,2,4,8
        .binHor      = 0x00,             ///< available horizontal binning values 1
        .binVert     = 0x00,             ///< vailable vertical binning values 1
        .maxGain256  = 0x100,            ///< (15.75) maximal analog gain times 0x100
        .minGain256  = 0x100,            ///< 1.5 times 0x100
        .minClockFreq= 25000000,         ///< Minimal clock frequency
        .maxClockFreq= 15000000,         ///< Maximal clock frequency
        .nomClockFreq= 25000000,         ///< nominal clock frequency
        .sensorType  = SENSOR_LEPTON35,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = LEPTON35_I2C_ADDR,///< sensor i2c slave address (7 bits)
        .i2c_period  = 1000,             ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,                ///< number of bytes/ register
        .hact_delay  = 0,                ///< delay in ps, TBD
        .sensorDelay = 0,                ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE  ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};

// Sysfs Interface for debugging the driver
static int first_sensor_sa7 [SENSOR_PORTS] = {0,0,0,0};
static unsigned int debug_delays = 0x0; // 0x6464; // udelay() values for mrst (low 8 - mrst on), [15:8] - after mrst
static unsigned int debug_modes =  3;
static unsigned short sensor_reg_copy[SENSOR_PORTS][256]; ///< Read all 256 sensor registers here - during initialization and on demand
                                                 ///< Later may increase to include multiple subchannels on 10359


// a place to add some general purpose register writes to sensors during init

/** Register initial writes for MT9M001 */
static  unsigned short lepton35_inits[]=
{
		P_LEPTON_GP3VSYNC, P_REG_LEPTON_GP3VSYNC_VAL // Enable VSYNC to GPIO3
};



int lepton_pgm_detectsensor (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_initsensor   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_sensorin     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_window       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_window_safe  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_window_common(int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_limitfps     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_exposure     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_gains        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_triggermode  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int lepton_pgm_sensorregs   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);

/**
 * Wait sensor is booted and ready: TODO - use timer interrupts to avoid blocking CPU by multiple channels waiting for i2c (add interrupts?)
 */
int lepton_wait_ready(int sensor_port,       ///< sensor port number (0..3)
                      int sa7,               ///< I2C slave address
                      int num_retries ){  ///< number of retries, 0 - forever
                                             ///< @return > 0 number of retries0 - OK, negative - error
	    int ntry;
	    u32  i2c_read_dataw;
	    lepton_status_t * status = (lepton_status_t *) &i2c_read_dataw;
		dev_dbg(g_dev_ptr,"lepton_wait_ready(%d), sa7 =0x%x,  P_LEPTON_STATUS= 0x%x\n",sensor_port, sa7, P_LEPTON_STATUS);
// If Lepton is not booted, reading status each ~600ms or sooner delays boot status indefinitely, so if not booted - shut up for 2 seconds
// See if that influences frame times, if yes - delay silently.
    	X3X3_I2C_RCV2(sensor_port, sa7, P_LEPTON_STATUS, &i2c_read_dataw);
    	if ((status->boot_mode == 0) || (status->boot_status == 0)) {
    		dev_dbg(g_dev_ptr,"Lepton on port %d is not booted, wait 1.5 s silently\n",sensor_port);
    		udelay1000(1500);
    	} else if (status->rsv5 != 0){
    		dev_dbg(g_dev_ptr,"Lepton on port %d returned invalid status 0x%x, probably it does not exist - giving up\n",sensor_port, i2c_read_dataw);
    		return -ENODEV;
    	}

	    for (ntry = num_retries; (ntry > 0) || (num_retries == 0); ntry--){
	    	X3X3_I2C_RCV2(sensor_port, sa7, P_LEPTON_STATUS, &i2c_read_dataw);
	    	if ((status->busy == 0) && (status->boot_mode == 1) && (status->boot_status == 1)){
	    		dev_dbg(g_dev_ptr,"lepton_wait_ready(%d) = 0x%x, ntry = %d (of %d)\n",sensor_port, i2c_read_dataw, (num_retries - ntry), num_retries);
	    		return (num_retries - ntry) & 0x7ffffff;
	    	}
	    	udelay1000(1); // wait 1 ms
	    }
		dev_dbg(g_dev_ptr,"lepton_wait_ready(%d) = 0x%x, timeout (%d tries)\n",sensor_port, i2c_read_dataw, num_retries);
	    return -ETIMEDOUT;
}

/**
 * Get single internal Lepton register (data length = 1), wait no longer than specified (in ms)
 */

int lepton_get_reg       (int               sensor_port,      ///< sensor port number (0..3)
						  int               sa7,              ///< I2C slave address
                          int               cmd,           ///< Lepton command id
                          int               wait_ms){ ///< milliseconds to wait for ready (if >0)
     int i2c_read_dataw;
     int ierr = 0;
	 dev_dbg(g_dev_ptr,"lepton_get_reg(%d, 0x%x,  0x%x, %d)\n", sensor_port, sa7, (int) cmd, wait_ms);
	 cmd |= ((LEPTON_MODULE(cmd) == LEPTON_OEM) || (LEPTON_MODULE(cmd) == LEPTON_RAD)) ? 0x4000: 0;
	 cmd &= 0xfffc;    // remove possible stray bits
	 cmd |= LEPTON_GET;
	 dev_dbg(g_dev_ptr,"X3X3_I2C_SEND2_LUT(%d, -1, 0, 0x%x, 0x%x)\n", sensor_port, P_LEPTON_DATA_LENGTH, 1);
	 X3X3_I2C_SEND2_LUT(sensor_port, -1, 0, P_LEPTON_DATA_LENGTH, 1);
	 dev_dbg(g_dev_ptr,"X3X3_I2C_SEND2_LUT(%d, -1, 0, 0x%x, 0x%x)\n", sensor_port, P_LEPTON_COMMAND_ID, (int) cmd);
	 X3X3_I2C_SEND2_LUT(sensor_port, -1, 0, P_LEPTON_COMMAND_ID,  cmd);
	 if (wait_ms > 0){
		 ierr = lepton_wait_ready(sensor_port,  sa7, wait_ms );
		 if (ierr <0) return ierr;
	 }
 	 X3X3_I2C_RCV2(sensor_port, sa7, P_LEPTON_DATA00, &i2c_read_dataw);
 	 dev_dbg(g_dev_ptr,"X3X3_I2C_RCV2(%d, 0x%x, 0x%x, &i2c_read_data) -> 0x%x\n", sensor_port, sa7, P_LEPTON_DATA00, i2c_read_dataw);
 	 return i2c_read_dataw;
}

/**
 * Set single internal Lepton register (data length = 1) in immediate mode, wait no longer than specified (in ms)
 */

int lepton_set_reg       (int               sensor_port,      ///< sensor port number (0..3)
						  int               sa7,              ///< I2C slave address
                          int               cmd,              ///< Lepton command id
                          int               wait_ms,          ///< milliseconds to wait for ready (if >0)
						  int               data) {           ///< data to write (16 bit)
     int ierr = 0;
	 dev_dbg(g_dev_ptr,"lepton_set_reg(%d, 0x%x, 0x%x, %d)\n", sensor_port, sa7, cmd, wait_ms);

	 if (wait_ms > 0){
		 ierr = lepton_wait_ready(sensor_port,  sa7, wait_ms );
		 if (ierr <0) return ierr;
	 }
	 lepton_set_reg_nowait(sensor_port, -1, cmd,  data); ///< data to write
 	 return 0;
}

/**
 * Set single internal Lepton register (data length = 1). No wait for not busy,or boot
 */

void lepton_set_reg_nowait(int              sensor_port,    ///< sensor port number (0..3)
                          int               frame,          ///< frame number to apply, <0 - ASAP
                          int               cmd,         ///< Lepton command id
                          int               data         ){ ///< data to write
	 cmd |= ((LEPTON_MODULE(cmd) == LEPTON_OEM) || (LEPTON_MODULE(cmd) == LEPTON_RAD)) ? 0x4000: 0;
	 cmd &= 0xfffc;    // remove possible stray bits
	 cmd |= LEPTON_SET;
	 dev_dbg(g_dev_ptr,"lepton_set_reg_nowait(%d, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame, cmd, data);
	 dev_dbg(g_dev_ptr,"X3X3_I2C_SEND2_LUT(%d, 0x%x, 0x%x, 0x%x\n", sensor_port, frame, P_LEPTON_DATA00, (int) data);

	 X3X3_I2C_SEND2_LUT(sensor_port,frame, 0, P_LEPTON_DATA00,      data);

	 dev_dbg(g_dev_ptr,"X3X3_I2C_SEND2_LUT(%d, 0x%x, 0x%x, 0x%x\n", sensor_port, frame, P_LEPTON_DATA_LENGTH, 1);

	 X3X3_I2C_SEND2_LUT(sensor_port,frame, 0, P_LEPTON_DATA_LENGTH, 1);

	 dev_dbg(g_dev_ptr,"X3X3_I2C_SEND2_LUT(%d, 0x%x, 0x%x, 0x%x\n", sensor_port, frame, P_LEPTON_COMMAND_ID, (int) cmd);

	 X3X3_I2C_SEND2_LUT(sensor_port,frame, 0, P_LEPTON_COMMAND_ID,  cmd);
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
int lepton_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
//    u32  i2c_read_dataw;
    int i;
    int i2c_data,i2c_rslt;
//    int sensor_multi_regs_number;
    struct sensor_t * psensor; // current sensor
    x393_sensio_ctl_t sensio_ctl = {.d32=0};
//    unsigned short * sensor_multi_regs;

    // temporary
    struct sensor_port_config_t *pcfg;
    const char *name;
    x393_i2c_device_t * dc;

    dev_info(g_dev_ptr,"**lepton_pgm_detectsensor**: {%d}  frame16=%d, thispars->pars[P_SENSOR]= 0x%lx\n",sensor_port,frame16, thispars->pars[P_SENSOR]);
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    //  MDD1(printk("sensor=0x%x\n", (int)sensor));
    if (thispars->pars[P_SENSOR]!=0) { ///already initialized - called second time after common pgm_detectsensor(), first time is inside pgm_detectsensor()
        dev_info(g_dev_ptr,"{%d}  sensor 0x%x already detected, exiting\n",sensor_port,(int) thispars->pars[P_SENSOR]);
        return sensor->sensorType;
    }

    psensor= &lepton35;

    // temporary solution
    pcfg = &pSensorPortConfig[sensor_port];
    name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR);

    dc = xi2c_dev_get(name);
    if (dc){
    	dev_info(g_dev_ptr,"{%d} setting i2c_addr to 0x%02x\n",sensor_port,dc->slave7);
    	//pr_info("{%d} Setting i2c_addr to 0x%02x\n",sensor_port,dc->slave7);
        psensor->i2c_addr = dc->slave7;
    }

// **** Was no setting MRST active? Will work just once after loading bitsream ***?
    // turn off power down, set clock, activate reset
    sensio_ctl.reset =        2; // no power down, reset active
    sensio_ctl.mclk =         1;  sensio_ctl.mclk_set =      1;
    sensio_ctl.spi_en =       1; // reset
    sensio_ctl.out_en =       0;  sensio_ctl.out_en_set =    1;
    sensio_ctl.reset_err =    1;
    sensio_ctl.spi_clk =      0;  sensio_ctl.spi_clk_set =   1;
    sensio_ctl.segm_zero =    0;  sensio_ctl.segm_zero_set = 1;
    sensio_ctl.vsync_use =    0;  sensio_ctl.vsync_use_set = 1;
    sensio_ctl.noresync =     0;  sensio_ctl.noresync_set =  1;
    sensio_ctl.telemetry =    0;  sensio_ctl.telemetry_set = 1; // change it later by WINDOEW_HEIGHT?
    sensio_ctl.gpio0 =        3; // input
    sensio_ctl.gpio1 =        3; // input
    sensio_ctl.gpio2 =        3; // input
    // Hardware debug source:0-running,1-will_sync,2-vsync_rdy[1],3-discard_segment,4-in_busy,5-out_busy,6-hact,7-sof
    sensio_ctl.dbg_src =      0;  sensio_ctl.dbg_src_set =  1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
    // Wait 50000 MASTER_CLK (flir-lepton-engineering-datasheet.pdf, page 18) - 2ms
    udelay1000(3);
    // reset mode bits to keep bit field values
    sensio_ctl.d32 = 0;
    // deassert MRST to enable i2c (or is it active always?)
    sensio_ctl.reset =        3; // no power down, no reset
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    // wait for sensor to boot and verify it is alive
    // optimize - do not wait, just get reasonable i2c response (busy, booting)?
    udelay1000(10);
	i = lepton_wait_ready(sensor_port, psensor->i2c_addr, 2000 );

	i2c_rslt = x393_xi2c_read_reg( "lepton35", // const char * cname,    ///< device class name
			sensor_port, // int          chn,      ///< sensor port number
			0, // int          sa7_offs, ///< slave address (7-bit) offset from the class defined slave address.
			2, // int          reg_addr, ///< register address (width is defined by class)
			&i2c_data); //int *        datap)    ///< pointer to a data receiver (read data width is defined by class)
	dev_info(g_dev_ptr,"rslt = %d, d = 0x%x\n",i2c_rslt, i2c_data);
	dev_info(g_dev_ptr,"Waited for Lepton on port = %d for %d tries\n",sensor_port, i);

    if (i <0) {
        dev_info(g_dev_ptr,"No Lepton sensors on port = %d\n",sensor_port);
    	return 0;  // no sensor found
    }

    // Sensor recognized, go on
    //  memcpy(&sensor, psensor, sizeof(mt9p001)); // copy sensor definitions
    memcpy(sensor, psensor, sizeof(lepton35)); // copy sensor definitions
    //  MDD1(dev_dbg(g_dev_ptr,"sensor=0x%x\n", (int)sensor));
    dev_dbg(g_dev_ptr,"{%d} copied %d bytes of sensor static parameters\n",sensor_port,sizeof(lepton35));
    add_sensor_proc(sensor_port,onchange_detectsensor,&lepton_pgm_detectsensor); // detect sensor type, sets sensor structure (capabilities), function pointers NOTE: will be called directly, not through pointers
    add_sensor_proc(sensor_port,onchange_initsensor,  &lepton_pgm_initsensor);   // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    add_sensor_proc(sensor_port,onchange_sensorin,    &lepton_pgm_sensorin);     // see mt9f002
    add_sensor_proc(sensor_port,onchange_exposure,    &lepton_pgm_exposure);     // program exposure
    add_sensor_proc(sensor_port,onchange_window,      &lepton_pgm_window);       // program sensor WOI and mirroring (flipping)
    add_sensor_proc(sensor_port,onchange_window_safe, &lepton_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    add_sensor_proc(sensor_port,onchange_limitfps,    &lepton_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    add_sensor_proc(sensor_port,onchange_gains,       &lepton_pgm_gains);        // program analog gains
    add_sensor_proc(sensor_port,onchange_triggermode, &lepton_pgm_triggermode);  // program sensor trigger mode
    add_sensor_proc(sensor_port,onchange_sensorregs,  &lepton_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?

    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType); // should cause other actions

    setFramePar(sensor_port, thispars, P_COLOR, COLORMODE_RAW);
    setFramePar(sensor_port, thispars, P_BITS,  16);

    common_pars->sensors[sensor_port] =  sensor->sensorType;
    // reset is inactive, acquisition is still off
    dev_info(g_dev_ptr,"lepton_pgm_detectsensor()#%d ->  = %ld\n",sensor_port, sensor->sensorType);
    return sensor->sensorType;
}

/** Reset and initialize sensor
 * resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c - 353 only)
 * i2c is supposed to be already programmed */
int lepton_pgm_initsensor     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    struct frameparspair_t pars_to_update[10];  // for all the sensor registers. Other P_* values will reuse the same ones
    int nupdate=0;
//    int first_sensor_i2c;
//    unsigned short * sensor_register_overwrites;
	int wait_ms= 1;
    x393_sensio_ctl_t sensio_ctl = {.d32=0};
//    u32 i2c_read_data_dw[256];
//    struct frameparspair_t  pars_to_update[8];
//    int nupdate=0;
    int i; // ,color;
//    int regval, regnum, mreg, j;
//    int sensor_register_overwrites_number;
//    int sensor_subtype;

    dev_dbg(g_dev_ptr,"lepton_pgm_initsensor(): {%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= 0) return -1; // should be ASAP

    // turn off power down, set clock, activate reset
    sensio_ctl.reset =        2; // no power down, reset active
    sensio_ctl.mclk =         1;  sensio_ctl.mclk_set =      1;
    sensio_ctl.spi_en =       1; // reset
    sensio_ctl.out_en =       0;  sensio_ctl.out_en_set =    1;
    sensio_ctl.reset_err =    1;
    sensio_ctl.spi_clk =      0;  sensio_ctl.spi_clk_set =   1;
    sensio_ctl.segm_zero =    0;  sensio_ctl.segm_zero_set = 1;
    sensio_ctl.vsync_use =    0;  sensio_ctl.vsync_use_set = 1;
    sensio_ctl.noresync =     0;  sensio_ctl.noresync_set =  1;
    sensio_ctl.telemetry =    0;  sensio_ctl.telemetry_set = 1; // change it later by WINDOEW_HEIGHT?
    sensio_ctl.gpio0 =        3; // input
    sensio_ctl.gpio1 =        3; // input
    sensio_ctl.gpio2 =        3; // input
    // Hardware debug source:0-running,1-will_sync,2-vsync_rdy[1],3-discard_segment,4-in_busy,5-out_busy,6-hact,7-sof
    sensio_ctl.dbg_src =      0;  sensio_ctl.dbg_src_set =  1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
    // Wait 50000 MASTER_CLK (flir-lepton-engineering-datasheet.pdf, page 18) - 2ms
    udelay1000(3);
    // reset mode bits to keep bit field values
    sensio_ctl.d32 = 0;
    // deassert MRST to enable i2c (or is it active always?)
    sensio_ctl.reset =        3; // no power down, no reset
    x393_sensio_ctrl(sensio_ctl,sensor_port);
    // wait for sensor to boot and verify it is alive
//    i = lepton_wait_ready(sensor_port, sensor->i2c_addr, 1000 );
    i = lepton_wait_ready(sensor_port, sensor->i2c_addr, 5000 );

    dev_dbg(g_dev_ptr,"lepton_pgm_initsensor(): Waited for Lepton on port = %d for %d tries\n",sensor_port, i);
    if (i <0) {
        dev_dbg(g_dev_ptr,"Lepton sensor is lost!on port = %d\n",sensor_port);
    	return 0;  // no sensor found
    }

    dev_dbg(g_dev_ptr,"Reading Lepton (port=%d) essential registers to the shadows\n",sensor_port);

    for (i = 0; lepton_par2addr[2*i] < 256; i+=1){
    	if (lepton_par2addr[2*i] > P_LEPTON_DATAFF) { // first registers are i2c registers, skip them
    		sensor_reg_copy[sensor_port][lepton_par2addr[2*i]] = lepton_get_reg (sensor_port, sensor->i2c_addr, lepton_par2addr[2*i + 1], wait_ms);
    		SET_LEPTON_PAR_DRY(sensor_port, lepton_par2addr[2*i],sensor_reg_copy[sensor_port][lepton_par2addr[2*i]]);
    	}
    }

    for (i=0; i< sizeof(lepton35_inits)/ 4;i++ ) { // unconditionally set those registers NOTE: Should be < 63 of them!
    	// set in immediate mode (with waits), update shadows
    	SET_LEPTON_PAR_IMMED(sensor_port, sensor->i2c_addr, lepton35_inits[2*i], lepton35_inits[2*i + 1],wait_ms);
        dev_dbg(g_dev_ptr,"{%d}   SET_LEPTON_PAR_IMMED(0x%x,0x%x)\n",sensor_port,P_SENSOR_REGS+lepton35_inits[2*i],lepton35_inits[2*i+1]);
        sensor_reg_copy[sensor_port][lepton35_inits[2*i]] = lepton35_inits[2*i + 1];
    }



    // Setup VOSPI to generate frame sync  for teh sequencers
    sensio_ctl.d32 = 0;
    sensio_ctl.spi_en =       3; // not reset, enabled
    sensio_ctl.out_en =       1;  sensio_ctl.out_en_set =    1;
    sensio_ctl.reset_err =    1;
//    sensio_ctl.spi_clk =      0;  sensio_ctl.spi_clk_set =   1;
    sensio_ctl.segm_zero =    0;  sensio_ctl.segm_zero_set = 1;
    sensio_ctl.vsync_use =    0;  sensio_ctl.vsync_use_set = 1;
    sensio_ctl.noresync =     0;  sensio_ctl.noresync_set =  1;
    sensio_ctl.telemetry =    0;  sensio_ctl.telemetry_set = 1; // change it later by WINDOEW_HEIGHT?
    sensio_ctl.gpio0 =        3; // input
    sensio_ctl.gpio1 =        3; // input
    sensio_ctl.gpio2 =        3; // input
    // Hardware debug source:0-running,1-will_sync,2-vsync_rdy[1],3-discard_segment,4-in_busy,5-out_busy,6-hact,7-sof
    sensio_ctl.dbg_src =      0;  sensio_ctl.dbg_src_set =  1;

    if (sensor_reg_copy[sensor_port][P_LEPTON_GP3VSYNC] == P_REG_LEPTON_GP3VSYNC_VAL) {
    	sensio_ctl.vsync_use =    0;
        dev_dbg(g_dev_ptr,"Using VSYNC on port=%d\n",sensor_port);
    } else {
    	sensio_ctl.vsync_use =    0;
        dev_dbg(g_dev_ptr,"Not using VSYNC on port=%d\n",sensor_port);
    }

	pars_to_update[nupdate  ].num= P_OVERSIZE; // does not work? - comes form autocampars
    pars_to_update[nupdate++].val= 1;
	pars_to_update[nupdate  ].num= P_WOI_HEIGHT;

    if (sensor_reg_copy[sensor_port][P_LEPTON_TELEN]) {
    	sensio_ctl.vsync_use =    0;
        dev_dbg(g_dev_ptr,"Using %ld rows frame to include telemetry data on port=%d\n",lepton35.arrayHeight, sensor_port);
        pars_to_update[nupdate++].val= lepton35.arrayHeight;
        sensio_ctl.telemetry =    1;
    } else {
        dev_dbg(g_dev_ptr,"Using just %ld rows frame, do not include telemetry data on port=%d\n",lepton35.clearHeight, sensor_port);
        pars_to_update[nupdate++].val= lepton35.clearHeight;
        sensio_ctl.telemetry =    0;
    }
    x393_sensio_ctrl(sensio_ctl,sensor_port);




    /*
     * lepton35.arrayHeight
lepton35.clearHeight
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



    dev_dbg(g_dev_ptr,"Initializing LEPTON35 registers with default values:\n");
	sensor_register_overwrites=         (unsigned short *) &lepton35_inits; // why casting is needed?
	sensor_register_overwrites_number=  sizeof(lepton35_inits)/4;
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

/**
 * Program sensor input in FPGA (Bayer, 8/16 bits, FPN, test mode, number of lines).
 */
int lepton_pgm_sensorin (int sensor_port,               ///< sensor port number (0..3)
		struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
		struct framepars_t * thispars, ///< sensor current parameters
		struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
		int frame16)                   ///< 4-bit (hardware) frame number parameters should
									   ///< be applied to,  negative - ASAP
									   ///< @return OK - 0, <0 - error
{
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
int lepton_pgm_window     (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    return lepton_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Program sensor WOI and mirroring in safe mode (now does the same as lepton_pgm_window)
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands  */
int lepton_pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                       ///< be applied to,  negative - ASAP
                                       ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    return lepton_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** PCommon part of programming sensor WOI */
int lepton_pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
//    int i,dv,dh,bv,bh,ww,wh,wl,wt,flip,flipX,flipY,d, v;
    int ww,wh; // ,wl,wt;
    int need_telemetry;
    x393_sensio_ctl_t sensio_ctl = {.d32=0};

//    int compressor_margin; // 0 for JP4, 2 for JPEG
    struct frameparspair_t  pars_to_update[8];
    int nupdate=0;
//    int styp = sensor->sensorType & 7;
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    ww =sensor->imageWidth;
    wh = thispars->pars[P_SENSOR_PIXV];
    if (wh > sensor-> imageHeight){
    	wh = sensor -> arrayHeight; // with telemetry
        need_telemetry = 1;
    } else {
    	wh = sensor -> imageHeight; // without telemetry
        need_telemetry = 0;
    }
    if (unlikely(thispars->pars[P_SENSOR_PIXH] != ww)) { // correct window width if needed
        SETFRAMEPARS_SET(P_SENSOR_PIXH, ww);
        SETFRAMEPARS_SET(P_WOI_WIDTH, ww);
    }
    dev_dbg(g_dev_ptr,"{%d}   wh=0x%x thispars->pars[P_SENSOR_PIXV]=0x%lx\n",sensor_port, wh, thispars->pars[P_SENSOR_PIXV]);
    if (unlikely(thispars->pars[P_SENSOR_REGS + P_LEPTON_TELEN] != need_telemetry)) {
//    if (unlikely(thispars->pars[P_SENSOR_PIXV] != wh)) {
        SETFRAMEPARS_SET(P_SENSOR_PIXV, wh); // probably already set by pgm_window_common
        SETFRAMEPARS_SET(P_WOI_HEIGHT, wh); // probably already set by pgm_window_common

        SET_LEPTON_PAR_NOWAIT(sensor_port, frame16, P_LEPTON_TELEN, need_telemetry);
        sensio_ctl.telemetry =     (wh > sensor -> imageHeight);
        sensio_ctl.telemetry_set = 1; // change it later by WINDOEW_HEIGHT?
        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_ctrl, sensio_ctl);
        dev_dbg(g_dev_ptr," X393_SEQ_SEND1 (%d, 0x%x, x393_sensio_ctrl,  0x%x)\n",sensor_port, frame16, (int) sensio_ctl.d32);
    }

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
int lepton_pgm_limitfps   (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
	// nothing to do here?
    return 0;
}

/** Program sensor exposure */
int lepton_pgm_exposure       (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
	// nothing to do here
    return 0;
}

#define SHIFT_DGAIN 1 // shift digital gain right, so 1.0 is 0x8000 an the full range is 4:1 - make it a parameter?


/** Program analog gains
 * program analog gains TODO: Make separate read-only P_ACTUAL_GAIN** ?
 * apply sensor-specific restrictions on the allowed gain values
 * includes sensor test mode on/off/selection */
//(FRAMEPAR_MODIFIED(P_VEXPOS)
//#define CSCALES_CTL_NORMAL  0 // USE P_*SCALE as is
//#define CSCALES_CTL_RECALC  1 // Recalculate P_*SCALE from P_GAIN*, P_GAING, then use it (will change to CSCALES_CTL_NORMAL when applied)
//#define CSCALES_CTL_FOLLOW  2 // Don't apply P_*SCALE to P_GAIN*, but update it from the current P_*SCALE from P_GAIN*/P_GAING
//#define CSCALES_CTL_DISABLE 3 // Disable P_*SCALE - don't apply P_*SCALE to P_GAIN*, don't update P_*SCALE from P_GAIN*, P_GAING
#define MAX_DIGITAL_GAIN 0x300 //integer x256 (0x300 ~3.0)
int lepton_pgm_gains      (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error

{
	// nothing to do here
    return 0;
}

/** Program trigger mode as sensor-specific  */
int lepton_pgm_triggermode        (int sensor_port,               ///< sensor port number (0..3)
                                    struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                    struct framepars_t * thispars, ///< sensor current parameters
                                    struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                    int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                                   ///< be applied to,  negative - ASAP
                                                                   ///< @return 0 - OK, negative - error
{
    struct frameparspair_t pars_to_update[4]; ///
    int nupdate=0;

    if (unlikely(thispars->pars[P_TRIG] != 0)) { // only [P_TRIG]==0 is possible with Lepton
        SETFRAMEPARS_SET(P_TRIG, 0);
    }

    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}

/** Program sensor registers (probably just those that are manually set)
 * NOTE: all modes but ASAP are limited to 64 registers/frame, no overflow checks are performed! */

int lepton_pgm_sensorregs     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
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

// SysFS interface to Lepton
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
// Dump 256 16-bit sensor registers */
static ssize_t show_sensor_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn =  get_channel_from_name(attr);
    char * cp = buf;
    int i,j, ij=0;
    for (i=0; i < 16; i++){
        cp += sprintf(cp,"%02x:",i*16);
        for (j=0;j<16;j++) cp += sprintf(cp," %04x", sensor_reg_copy[chn][ij++]);
        cp += sprintf(cp,"\n");
    }
    return cp - buf;
}

/** Ignore data, re-read 256 sensor registers */
static ssize_t store_sensor_regs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int chn =  get_channel_from_name(attr);
    u32 datai2c;
    int i;
    if (first_sensor_sa7[chn]) for (i = 0; i< 256;i++) {
        X3X3_I2C_RCV2(chn, first_sensor_sa7[chn], i, &datai2c);
        sensor_reg_copy[chn][i] = datai2c;
    }
    return count;
}

static ssize_t show_debug_delays(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%08x\n", debug_delays);
}


static ssize_t store_debug_delays(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if (!sscanf(buf, "%x", &debug_delays)) {
        return - EINVAL;
    }
    return count;
}
static ssize_t show_debug_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%08x\n", debug_modes);
}


static ssize_t store_debug_modes(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if (!sscanf(buf, "%x", &debug_modes)) {
        return - EINVAL;
    }
    return count;
}

static DEVICE_ATTR(sensor_regs0,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs1,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs2,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs3,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(debug_delays,               SYSFS_PERMISSIONS,     show_debug_delays,               store_debug_delays);
static DEVICE_ATTR(debug_modes,                SYSFS_PERMISSIONS,     show_debug_modes,                store_debug_modes);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_sensor_regs0.attr,
        &dev_attr_sensor_regs1.attr,
        &dev_attr_sensor_regs2.attr,
        &dev_attr_sensor_regs3.attr,
        &dev_attr_debug_delays.attr,
        &dev_attr_debug_modes.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_lepton_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

int lepton_init(struct platform_device *pdev)
{
//    int res;
    struct device *dev = &pdev->dev;
//    const struct of_device_id *match;
    int sensor_port;

    for (sensor_port = 0; sensor_port < SENSOR_PORTS; sensor_port++) {
        first_sensor_sa7[sensor_port] = 0;
    }
    elphel393_lepton_sysfs_register(pdev);
    dev_info(dev, DEV393_NAME(DEV393_LEPTON)": registered sysfs\n");
    g_dev_ptr = dev;
    return 0;
}

int lepton_remove(struct platform_device *pdev)
{
    unregister_chrdev(DEV393_MAJOR(DEV393_LEPTON), DEV393_NAME(DEV393_LEPTON));
    return 0;
}

static const struct of_device_id elphel393_lepton_of_match[] = {
    { .compatible = "elphel,elphel393-lepton-1.00" },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_lepton_of_match);

static struct platform_driver elphel393_lepton = {
    .probe          = lepton_init,
    .remove         = lepton_remove,
    .driver         = {
        .name       = DEV393_NAME(DEV393_LEPTON),
        .of_match_table = elphel393_lepton_of_match,
    },
};

module_platform_driver(elphel393_lepton);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION("Driver for VOSPI interface (Lepton3.5)* in Elphel cameras");
