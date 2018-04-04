/***************************************************************************//**
 * @file      mt9f002.c
 * @brief     Handles MT9F002 On Semiconductor 14MPx sensor
 *
 * @copyright Copyright 2018 (C) Elphel, Inc.
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

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h" // detect sensor
//#include "cci2c.h"
#include "mt9f002.h"
//#include "multi10359.h"
#include "framepars.h"      // parameters manipulation
#include "sensor_common.h"
#include "pgm_functions.h"
#include "x393.h"
#include "sensor_i2c.h"
//#include "detect_sensors.h"

// hact timeout in udelay(100)s - for data lanes' phase adjustment
#define HACT_TIMEOUT 8192

/**
LUT to map SENSOR_REGSxxx to internal sensor register addresses
  * needed for any sensor
  * For better manual mapping:
      - even elements are SENSOR_REGSxxx,
      - odd elements are sensor's register addresses.
  * has to be at least 16-bit/entry for 16 bit addresses
  * (for MT9X001 it's a 1-to-1 mapping)
*/
const unsigned short mt9f002_par2addr[] = {
		P_MT9F002_MODEL_ID,     P_REG_MT9F002_MODEL_ID,
		P_MT9F002_RESET,        P_REG_MT9F002_RESET_REGISTER,
		P_MT9F002_EXPOS,        P_REG_MT9F002_COARSE_INTEGRATION_TIME,
		P_MT9F002_TEST_PATTERN, P_REG_MT9F002_TEST_PATTERN_MODE,
		P_MT9F002_GAIN,         P_REG_MT9F002_GLOBAL_GAIN,
		P_MT9F002_GAINGR,       P_REG_MT9F002_GREEN1_GAIN,
		P_MT9F002_GAINR,        P_REG_MT9F002_RED_GAIN,
		P_MT9F002_GAINB,        P_REG_MT9F002_BLUE_GAIN,
		P_MT9F002_GAINGB,       P_REG_MT9F002_GREEN2_GAIN,
		P_MT9F002_HISPI_TIMING,             P_REG_MT9F002_HISPI_TIMING,
		P_MT9F002_SMIA_PLL_MULTIPLIER,      P_REG_MT9F002_SMIA_PLL_MULTIPLIER,
		P_MT9F002_HISPI_CONTROL_STATUS,     P_REG_MT9F002_HISPI_CONTROL_STATUS,
		P_MT9F002_DATAPATH_SELECT,          P_REG_MT9F002_DATAPATH_SELECT,
		P_MT9F002_RESET_REGISTER,           P_REG_MT9F002_RESET_REGISTER,
		P_MT9F002_COARSE_INTEGRATION_TIME,  P_REG_MT9F002_COARSE_INTEGRATION_TIME,
		P_MT9F002_FINE_INTEGRATION_TIME,    P_REG_MT9F002_FINE_INTEGRATION_TIME,
		P_MT9F002_Y_ADDR_START,             P_REG_MT9F002_Y_ADDR_START,
		P_MT9F002_Y_ADDR_END,               P_REG_MT9F002_Y_ADDR_END,
		P_MT9F002_X_ADDR_START,             P_REG_MT9F002_X_ADDR_START,
		P_MT9F002_X_ADDR_END,               P_REG_MT9F002_X_ADDR_END,
		P_MT9F002_Y_OUTPUT_SIZE,            P_REG_MT9F002_SMIA_Y_OUTPUT_SIZE,
		P_MT9F002_X_OUTPUT_SIZE,            P_REG_MT9F002_SMIA_X_OUTPUT_SIZE,
		P_MT9F002_LINE_LENGTH_PCK,          P_REG_MT9F002_LINE_LENGTH_PCK,
        P_MT9F002_X_ODD_INC,                P_REG_MT9F002_SMIA_X_ODD_INC,
        P_MT9F002_MIN_LINE_BLANKING_PCK,    P_REG_MT9F002_SMIA_MIN_LINE_BLANKING_PCK,
        P_MT9F002_MIN_LINE_LENGTH_PCK,      P_REG_MT9F002_SMIA_MIN_LINE_LENGTH_PCK,
		P_MT9F002_FRAME_LENGTH_LINES,       P_REG_MT9F002_FRAME_LENGTH_LINES,
		P_MT9F002_MIN_FRAME_BLANKING_LINES, P_REG_MT9F002_SMIA_MIN_FRAME_BLANKING_LINES,
		P_MT9F002_READ_MODE,                P_REG_MT9F002_READ_MODE,
		0xffff // END indicator
};

/**
 * get at least one parameter for a page
 */
const unsigned short mt9f002_pages[] = {
		P_REG_MT9F002_SMIA_MODEL_ID,                    // page 0x00
		P_REG_MT9F002_SMIA_MODE_SELECT,                 // page 0x01
		P_REG_MT9F002_SMIA_FINE_INTEGRATION_TIME,       // page 0x02
		P_REG_MT9F002_SMIA_VT_PIX_CLK_DIV,              // page 0x03
		P_REG_MT9F002_SMIA_SCALING_MODE,                // page 0x04
		P_REG_MT9F002_SMIA_COMPRESSION_MODE,            // page 0x05
		P_REG_MT9F002_SMIA_TEST_PATTERN_MODE,           // page 0x06
		P_REG_MT9F002_SMIA_INTEGRATION_TIME_CAPABILITY, // page 0x10
		P_REG_MT9F002_SMIA_MIN_EXT_CLK_FREQ_MHZ,        // page 0x11
		P_REG_MT9F002_SMIA_SCALING_CAPABILITY,          // page 0x12
		P_REG_MT9F002_SMIA_COMPRESSION_CAPABILITY,      // page 0x13
		P_REG_MT9F002_SMIA_MATRIX_ELEMENT_REDINRED,     // page 0x14
		P_REG_MT9F002_MODEL_ID,                         // page 0x30
		P_REG_MT9F002_OTPM_CFG,                         // page 0x31
		P_REG_MT9F002_P_GR_P0Q0,                        // page 0x36
		P_REG_MT9F002_P_GR_P4Q0,                        // page 0x37
		P_REG_MT9F002_DAC_LD_FBIAS,                     // page 0x3E
		0xffff // END indicator
};

static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
void mt9f002_set_device(struct device *dev) // do nothing, now it has it's own device
{
    //g_dev_ptr = dev;
}

// this struct is for pgm_functions.c
/** Capabilities of MT9F002 - 14MPix */
struct sensor_t mt9f002 = {
        // sensor constants
        .imageWidth  = 4384,     ///< nominal image width for final images
        .imageHeight = 3280,     ///< nominal image height for final images
        .clearWidth  = 4608,     ///< maximal clear image width
        .clearHeight = 3288,     ///< maximal clear image height;
        .clearTop    = 32,       ///< top margin to the first clear pixel
        .clearLeft   = 144,      ///< left margin to the first clear pixel
        .arrayWidth  = 4640,     ///< total image array width (including black and boundary)
        .arrayHeight = 3320,     ///< total image array height (including black and boundary)
        .minWidth    = 2,        ///< minimal WOI width
        .minHeight   = 2,        ///< minimal WOI height

        .minHorBlank = 0,        ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 0,        ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 0,        ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 0,        ///< minimal vertical blanking
        .maxVertBlank= 0,        ///< maximal vertical blanking/Virtual frame height (depends on sensor type)

        .maxShutter  = 0xfffff,  ///< Maximal shutter duration (in lines)

        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 0,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 2,        ///< bayer shift for flips==0

        .dcmHor      = 0xff,     ///< 1,2,3,4,5,6,7,8 (doc show [0,6] - change to 0x7f
        .dcmVert     = 0xff,     ///< 1,2,3,4,5,6,7,8
        .binHor      = 0xff,     ///< 1,2,4 0xb{0,1,3}
        .binVert     = 0xff,     ///< 1,2,3,4 0xf [0,3]
        .maxGain256  = 4064,     ///< (15.875) maximal analog gain times 0x100
        .minGain256  = 384,      ///< 1.5 times 0x100

        .minClockFreq= 200000000, ///< Minimal clock frequency
        .maxClockFreq= 240000000, ///< Maximal clock frequency
        .nomClockFreq= 220000000, ///< nominal clock frequency

        .sensorType  = SENSOR_MT9F002,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9F002_I2C_ADDR,           ///< sensor i2c slave address (7 bits)

        .hact_delay  = -2500,    ///< -2.5ns delay in ps
        .sensorDelay = 2460,     ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE   ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};

// Sysfs Interface for debugging the driver
static int first_sensor_sa7 [SENSOR_PORTS] = {0,0,0,0};
static unsigned int debug_delays = 0x0; // 0x6464; // udelay() values for mrst (low 8 - mrst on), [15:8] - after mrst
static unsigned int debug_modes =  3;
static unsigned short sensor_reg_copy[SENSOR_PORTS][256]; ///< Read all 256 sensor registers here - during initialization and on demand
                                                          ///< Later may increase to include multiple subchannels on 10359
static bool init_done[4] = {false,false,false,false};

// a place to add some general purpose register writes to sensors during init


/** Initial register writes for MT9F002 */
static unsigned short mt9f002_inits[]=
{
		P_REG_MT9F002_HISPI_TIMING,            0x8000, //
		P_REG_MT9F002_SMIA_PLL_MULTIPLIER,     MT9F002_PLL_MULTIPLIER_VALUE, //
		P_REG_MT9F002_HISPI_CONTROL_STATUS,    0x8400, //
		P_REG_MT9F002_DATAPATH_SELECT,         0x9280, //
		//P_REG_MT9F002_RESET_REGISTER,          0x001c,
		//P_REG_MT9F002_TEST_PATTERN,            0x0002, // color bars
		P_REG_MT9F002_ANALOG_GAIN_CODE_GLOBAL, 0x000a,
		P_REG_MT9F002_ANALOG_GAIN_CODE_RED,    0x000d,
		P_REG_MT9F002_ANALOG_GAIN_CODE_BLUE,   0x0010,
		P_REG_MT9F002_COARSE_INTEGRATION_TIME, 0x0100
};

/** Specifying sensor registers to be controlled individually in multi-sensor applications, MT9P006 */
//static  unsigned short mt9p001_multiregs[]=
//{
        //P_MT9X001_ROWSTART,
		//P_MT9X001_COLSTART,
		//P_MT9X001_HEIGHT,
		//P_MT9X001_WIDTH,
		//P_MT9X001_HORBLANK,
		//P_MT9X001_VERTBLANK,
		//P_MT9X001_GREEN1,
		//P_MT9X001_BLUE,
		//P_MT9X001_RED,
		//P_MT9X001_GREEN2,
		//P_MT9X001_TEST
//};

int mt9f002_pgm_detectsensor (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_initsensor   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_sensorin     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_window       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_window_safe  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_window_common(int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_limitfps     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_exposure     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_gains        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_triggermode  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_sensorregs   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);

/**
 * Detect and initialize sensor and related data structures
 */
int mt9f002_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{

    u32  i2c_read_dataw;

    //int i;
    //int sensor_multi_regs_number;
    struct sensor_t * psensor; // current sensor
    x393_sensio_ctl_t sensio_ctl = {.d32=0};
    //unsigned short * sensor_multi_regs;
    struct sensor_port_config_t *pcfg;
    const char *name;
    x393_i2c_device_t * dc;
    // disable SOF
    x393_sens_sync_mult_t dis_sof = {.d32=0};

    //struct sensor_port_config_t pcfg;
    //pcfg = pSensorPortConfig[sensor_port];

    //u16 *model_id;
    //model_id = &pcfg->par2addr[0][P_MT9F002_MODEL_ID];

    dev_dbg(g_dev_ptr,"**mt9f002_pgm_detectsensor**: {%d}  frame16=%d, thispars->pars[P_SENSOR]= 0x%lx\n",sensor_port,frame16, thispars->pars[P_SENSOR]);
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);

    // Already initialized - called second time after common pgm_detectsensor(), first time is inside pgm_detectsensor()
    if (thispars->pars[P_SENSOR]!=0) {
        dev_dbg(g_dev_ptr,"{%d}  sensor 0x%x already detected, exiting\n",sensor_port,(int)thispars->pars[P_SENSOR]);
        return sensor->sensorType;
    }

    psensor= &mt9f002;

    // temporary solution (move to the common code)
    pcfg = &pSensorPortConfig[sensor_port];
    name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR);
    dc = xi2c_dev_get(name);
    if (dc) {
    	psensor->i2c_addr = dc->slave7;
    }

    dis_sof.dis_sof = 1;
    x393_sens_sync_mult(dis_sof,sensor_port);

    // set control lines
    sensio_ctl.mrst = 1;
    sensio_ctl.mrst_set = 1;
    sensio_ctl.arst = 1;
    sensio_ctl.arst_set = 1;
    sensio_ctl.aro = 1;
    sensio_ctl.aro_set = 1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
    sensio_ctl.d32=0;
    // reset mmcm
    sensio_ctl.mmcm_rst = 1;
    sensio_ctl.mmcm_rst_set = 1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);
    sensio_ctl.mmcm_rst = 0;
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    // delay until sensor gets responsive to i2c commands
    udelay(100);

    X3X3_I2C_RCV2(sensor_port, psensor->i2c_addr, P_REG_MT9F002_MODEL_ID, &i2c_read_dataw);
    dev_dbg(g_dev_ptr,"Read i2c (port = %d, sa7=0x%lx, reg=0x%x) chip ID=%x\n",sensor_port, psensor->i2c_addr, P_REG_MT9F002_MODEL_ID, i2c_read_dataw);

    if ((i2c_read_dataw ^ MT9F002_PARTID)==0) {
        dev_dbg(g_dev_ptr,"Found MT9F002 4384x3288 sensor, chip ID=%x\n",i2c_read_dataw);
    }else{
    	return 0;  // no sensor found
    }

    // Sensor is recognized, go on

    // copy sensor definitions
    memcpy(sensor, psensor, sizeof(mt9f002));
    dev_dbg(g_dev_ptr,"{%d} copied %d bytes of sensor static parameters\n",sensor_port,sizeof(mt9f002));

    // add functions to common pgm_functions
    add_sensor_proc(sensor_port,onchange_detectsensor,&mt9f002_pgm_detectsensor); // detect sensor type, sets sensor structure (capabilities), function pointers NOTE: will be called directly, not through pointers
    add_sensor_proc(sensor_port,onchange_initsensor,  &mt9f002_pgm_initsensor);   // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    add_sensor_proc(sensor_port,onchange_sensorin,    &mt9f002_pgm_sensorin);     // currently: VACT delay hack
    add_sensor_proc(sensor_port,onchange_exposure,    &mt9f002_pgm_exposure);     // program exposure
    add_sensor_proc(sensor_port,onchange_window,      &mt9f002_pgm_window);       // program sensor WOI and mirroring (flipping)
    add_sensor_proc(sensor_port,onchange_window_safe, &mt9f002_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    add_sensor_proc(sensor_port,onchange_limitfps,    &mt9f002_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    add_sensor_proc(sensor_port,onchange_gains,       &mt9f002_pgm_gains);        // program analog gains
    //add_sensor_proc(sensor_port,onchange_triggermode, &mt9x001_pgm_triggermode);  // program sensor trigger mode
    //add_sensor_proc(sensor_port,onchange_sensorregs,  &mt9x001_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?

    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType); // should cause other actions

    common_pars->sensors[sensor_port] =  sensor->sensorType;
    //  setFramePar(thispars, P_SENSOR  | FRAMEPAIR_FORCE_NEWPROC,  sensor->sensorType); // force actions
    //  MDD1(dev_dbg(g_dev_ptr,"\n"));

    // skipped multisensor regs

    sensio_ctl.d32=0;
    sensio_ctl.aro = 1;
    sensio_ctl.aro_set = 1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    return sensor->sensorType;
    //NOTE 353:  hardware i2c is turned off (not needed in 393)
}

// part of mt9f002_pgm_initsensor
// write to sensor's i2c register, test read
int mt9f002_phases_program_phase(int sensor_port, int phase){

	int read_phase = 0;
	struct sensor_port_config_t *pcfg;
	const char *name;

	pcfg = &pSensorPortConfig[sensor_port];
	name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR);

	X3X3_I2C_SEND2_LUT_ASAP(sensor_port,0x0,P_REG_MT9F002_HISPI_TIMING,phase);

	X3X3_I2C_RCV2(sensor_port, 0x10, P_REG_MT9F002_HISPI_TIMING, &read_phase);
	if (read_phase!=phase){
		pr_err("{%d} i2c write error, target value = 0x%04x, read value = 0x%04x \n",sensor_port,phase,read_phase);
	}

	return 0;
}

// part of mt9f002_pgm_initsensor
// read hact_alive bit from x393_status_sens_io_t
int mt9f002_phases_read_flags(int sensor_port,int shift){

	int res = 0;
	int i;
	x393_status_sens_io_t status;
	x393_sensio_tim2_t reset_flags = {.d32=0};

	// reset flags
	set_x393_sensio_tim2(reset_flags,sensor_port);

	// wait for hact
	for(i=0;i<HACT_TIMEOUT;i++){
		status = x393_sensio_status(sensor_port);
		if (status.hact_alive){
			break;
		}
		if (i==(HACT_TIMEOUT-1)){
			dev_dbg(g_dev_ptr,"{%d} hact never went up after reset\n",sensor_port);
		}
		udelay(100);
	}

	// read flags
	status = x393_sensio_status(sensor_port);

	switch(shift){
		case 0: res = status.barrel_0;break;
		case 1: res = status.barrel_1;break;
		case 2: res = status.barrel_2;break;
		case 3: res = status.barrel_3;break;
	}

	return res;

}

// part of mt9f002_pgm_initsensor
// phase adjustment for a single lane
int mt9f002_phases_adjust_lane(int sensor_port, int phase, int shift){

	int i;
	int status;
	const int clk_half_phase = 0x4000;

	int value = 0;
	bool switched = false;
	// i where switched, 1 - first 8 steps (and it's the middle phase),
	//                   2 - second 8 steps
	int i1 = 0;
	int i2 = 8;

	int target_phase;

	// 16 values: 8 for CLK phase 0 position CLK, 8 - for CLK phase 4 position
	for(i=0;i<16;i++){

		// support clk phase range
		if (i==0){
			phase += clk_half_phase;
		}

		// main clk range
		if (i==8){
			phase -= clk_half_phase;
		}

		target_phase = phase + ((i&0x7)<<(3*shift));
		mt9f002_phases_program_phase(sensor_port,target_phase);

		status = mt9f002_phases_read_flags(sensor_port,shift);

		if ((i==0)||(i==8)){
			value = status;
			switched = false;
		}else{
			if (value!=status){
				if (i<8){
					if (!switched){
						switched = true;
						value = status;
						i1 = i;
					}else{
						pr_err("{%d} Lane D%d: unexpected phase shift at %d\n",sensor_port,shift,i);
					}
				}else{
					if (!switched){
						switched = true;
						value = status;
						i2 = i;
					}else{
						pr_err("{%d} Lane D%d: unexpected phase shift at %d\n",sensor_port,shift,i);
					}
				}
			}
		}
	}

	i1 = i1&0x7;
	i2 = i2&0x7;

	// just a check
	if (abs(i2-i1)<2){
		pr_err("{%d} Warning: optimal phase shift (lane D%d) is probably not correct (difference = %d)",sensor_port,shift,i2);
	}

	target_phase = phase + (i1<<(3*shift));

	dev_dbg(g_dev_ptr,"{%d} lane D%d mid phase = %d (target phase = 0x%04x)",sensor_port,shift,i1,target_phase);

	// apply center phase
	mt9f002_phases_program_phase(sensor_port,target_phase);

	return target_phase;
}

// part of mt9f002_pgm_initsensor
// phase adjustment for port (all 4 lanes)
int mt9f002_phases_adjust_port(int sensor_port){

	// insert phase adjustment here - find middle
	// status: x393_status_sens_io_t status = x393_sensio_status(port) - read barrel - bits[21:14]
	// set lane phase example: write_sensor_i2c 0 1 0 0x31c08db6 = P_REG_MT9F002_HISPI_TIMING
	//
	// for port 0:
	// reset lanes_alive bits by writing to set_x393_sensio_tim2(port) - reg:0x40e or set_x393_sensio_tim3(port) reg:0x40f
	//     data: 0x40e x393_sensio_tim2_t D3?-[31:24],D2?-[23:16],D1?-[15:8],D0?-[7:0] - higher 5 bits - which D is which?!
	//     hact: 0x40f x393_sensio_tim3_t [7:0]

	int phase = 0x8000;
	int i;
	x393_status_sens_io_t status;

	// read it first ?
	x393_status_ctrl_t status_ctrl = {.d32 = 0};

	dev_dbg(g_dev_ptr,"{%d} Phases adjust\n",sensor_port);

	// enable status updates
	status_ctrl.mode = 0x3;
	set_x393_sensio_status_cntrl(status_ctrl, sensor_port);

	// wait for hact (usually alive after 254)
	for(i=0;i<HACT_TIMEOUT;i++){
		status = x393_sensio_status(sensor_port);
		if (status.hact_alive){
			//dev_dbg(g_dev_ptr,"{%d} HACT alive at %d\n",sensor_port,i);
			break;
		}
		if (i==(HACT_TIMEOUT-1)){
			dev_dbg(g_dev_ptr,"{%d} HACT never went live\n",sensor_port);
		}
		udelay(100);
	}

	// 4 data lanes - prior knowledge
	for(i=0;i<4;i++){
		phase = mt9f002_phases_adjust_lane(sensor_port,phase,i);
	}

	dev_dbg(g_dev_ptr,"{%d} Adjusted phase is 0x%04x\n",sensor_port,phase);

	// apply final phase
	//mt9f002_phases_program_phase(sensor_port,phase,0,false);

	// disable status updates
	//status_ctrl.mode = 0x0;
	//set_x393_sensio_status_cntrl(status_ctrl, sensor_port);

	return 0;
}

/**
 * Reset and initialize sensor
 * Resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers.
 */
int mt9f002_pgm_initsensor     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
	int i;
	int n;
	int color;

	x393_sens_sync_mult_t dis_sof = {.d32=0};
	struct frameparspair_t pars_to_update[262+(MAX_SENSORS * P_MULTI_NUMREGS )]; // for all the sensor registers. Other P_* values will reuse the same ones
	int regaddr,regval,regnum;

	int nupdate=0;

	int colamp_gain, a2_gain, gain, a2_inc;

	u32 i2c_read_data_dw[256];

	//dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
	// sensor is silent before init - this check is redundant
	//if (frame16 >= 0) return -1; // should be ASAP

	if (!init_done[sensor_port]){
		init_done[sensor_port] = true;
	}else{
		dev_dbg(g_dev_ptr,"{%d} Was going to try to init sensor twice. Exiting\n",sensor_port);
		return 0;
	}

	n = sizeof(mt9f002_inits)/4; // 4 bytes per pair
	for(i=0;i<n;i++){
		// sa7 is not used
		// use broadcast address - which should be 0 for a single sensor?
		X3X3_I2C_SEND2_LUT_ASAP(sensor_port,0,mt9f002_inits[2*i],mt9f002_inits[2*i+1]);
	}

	// soft reset
	// sa7 is not used
	X3X3_I2C_SEND2_LUT_ASAP(sensor_port,0,P_REG_MT9F002_RESET_REGISTER,MT9F002_RESET_REGISTER_VALUE);
	// delay is not needed, however if bit[0]=1 then it is needed
	//udelay(100);

	// sensor is supposed to be streaming by now
	mt9f002_phases_adjust_port(sensor_port);

	// restore SOF (disabled in mt9f002_pgm_detectsensor)
	dis_sof.dis_sof = 0;
	x393_sens_sync_mult(dis_sof,sensor_port);

	// init register shadows here
	// regaddr will be 0xffffffff for not used par
    for (i=0; i<256; i++) { // read all registers, one at a time (slower than in 353)
    	regaddr = pSensorPortConfig[sensor_port].par2addr[0][i];
    	if (!(regaddr&0xffff0000)){
    		// TODO: get rid of i2c_addr
    		X3X3_I2C_RCV2(sensor_port, sensor->i2c_addr, regaddr, &i2c_read_data_dw[i]);
    	}else{
    		i2c_read_data_dw[i] = 0;
    	}
    }
    dev_dbg(g_dev_ptr,"Read 256 registers (port=%d) ID=0x%x:\n",sensor_port,i2c_read_data_dw[0]);

    for (i=0; i<256; i++) { // possible to modify register range to save (that is why nupdate is separate from i)
        regval=i2c_read_data_dw[i];
        regnum=P_SENSOR_REGS+i;
        SETFRAMEPARS_SET(regnum,regval);
    }

    for (i=0;i<256;i++) {
        sensor_reg_copy[sensor_port][i] = i2c_read_data_dw[i];
    }

    // in mt9x00x there's setFrameParsStatic-call ?!!! Parameters that never change?
    if (nupdate)  setFrameParsStatic(sensor_port,nupdate,pars_to_update);  // save changes to sensor register shadows for all frames
    //if (nupdate) setFramePars(sensor_port,thispars,nupdate,pars_to_update);  // save changes to sensor register shadows

    // next are global pars?

    // set gains ranges
    SETFRAMEPARS_SET(P_GAIN_MIN, (sensor->minGain256)<<8); // less than that may not saturate sensor and confuse autoexposure/white balancing
    SETFRAMEPARS_SET(P_GAIN_MAX, (sensor->maxGain256)<<8);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows

    // G_* parameters - can write directly
    // fill out the gain tables
    for (color=0;color<4;color++){
    	// 175 =
    	// 48 (for colamp=1) +
    	// 48 (for colamp=2) +
    	// 79 (for colamp=3)

    	colamp_gain = 2;
    	a2_gain = 0x30;
    	a2_inc = 0;

        for (i=0;i<176;i++) {
        	// for mt9x00x
            //GLOBALPARS(sensor_port, G_SENSOR_CALIB+(color<<8)+i)= (i>32)? ((i-16)<<14) : (i<<13); // one extra
        	// for mt9f002
        	if ((i==48)||(i==96)){
        		colamp_gain *= 2;
        		a2_inc = 0;
        	}
        	// actual formula
        	//gain = ((colamp_gain*(a2_gain+a2_inc))<<16)/64;
        	gain = (colamp_gain*(a2_gain+a2_inc))<<10;
        	GLOBALPARS(sensor_port, G_SENSOR_CALIB+(color<<8)+i)= gain; // one extra
        	a2_inc += 1;
        }
    }

    return 0;
}

/**
 * Program sensor input in FPGA (Bayer, 8/16 bits, FPN, test mode, number of lines).
 */
int mt9f002_pgm_sensorin (int sensor_port,               ///< sensor port number (0..3)
		struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
		struct framepars_t * thispars, ///< sensor current parameters
		struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
		int frame16)                   ///< 4-bit (hardware) frame number parameters should
									   ///< be applied to,  negative - ASAP
									   ///< @return OK - 0, <0 - error
{
	x393_sensio_width_t   sensio_width = {.d32=0};

	// this is a hack because there's no delay after SOF in HISPI proto
	sensio_width.sensor_width = MT9F002_VACT_DELAY;
	X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_width, sensio_width);

	return 0;
}

// also defines vertical blanking
int mt9f002_calc_frame_length_lines(int y_addr_start,
		                              int y_addr_end,
									  int min_frame_blanking_lines){

	int v0;

	int subsampling_factor = 1;

	v0 = (y_addr_end - y_addr_start + 1)/subsampling_factor + min_frame_blanking_lines;

	/*
	if (coarse_exposure>(v0-1)){
		v0 = coarse_exposure + 1;
		pr_info("frame_length_lines will get extended\n");
	}
	*/

	return v0;
}

// also defines horizontal blanking
int mt9f002_calc_line_length_pck(int x_addr_start,
								   int x_addr_end,
								   int x_odd_inc,
								   int x_output_size,
								   int min_line_blanking_pck,
								   int min_line_length_pck){

	int v0, v1, v2;
	// does not match with the power on default value
	v0 = min_line_length_pck;
	// incorrect formula?
	//v1 = (x_addr_end-x_addr_start+x_odd_inc)/(1+x_odd_inc)+min_line_blanking_pck;
	v1 = (x_addr_end-x_addr_start+x_odd_inc)/(1+x_odd_inc)+min_line_blanking_pck/2;
	// temporary hack
	//v0 = v1;
	//v1 = ((x_addr_end-x_addr_start+x_odd_inc)*2/(1+x_odd_inc)+min_line_blanking_pck)/2;
	v2 = MT9F002_VT_PIX_CLK/MT9F002_OP_PIX_CLK*x_output_size/4 + 0x5E;
	// what?!
	//v3 = (x_addr_end-x_addr_start+1)+min_line_blanking_pck;

	//pr_info("x_addr_end=0x%08x x_addr_start=0x%08x x_odd_inc=0x%08x min_line_blanking_pck=0x%08x\n",x_addr_end,x_addr_start,x_odd_inc,min_line_blanking_pck);
	//pr_info("v0=0x%08x v1=0x%08x v2=0x%08x\n",v0,v1,v2);

	if (v0<v1) v0 = v1+2;
	if (v0<v2) v0 = v2;
	//if (v0<v3) v0 = v3;

	//pr_info("result = 0x%08x\n",v0);

	return v0;
}

/** Program sensor WOI and mirroring
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands
 * As different sensors may produce "bad frames" for different WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped)*/
int mt9f002_pgm_window     (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    return mt9f002_pgm_window_common(sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Program sensor WOI and mirroring in safe mode (now does the same as mt9x001_pgm_window)
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands  */

int mt9f002_pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                       ///< be applied to,  negative - ASAP
                                       ///< @return 0 - OK, negative - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    return mt9f002_pgm_window_common(sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Common part of programming sensor WOI */
int mt9f002_pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    int dv,dh,bv,bh,ww,wh,wl,wt,flip,flipX,flipY,d, v;
    int wws,wwe,whs,whe;
    int compressor_margin; // 0 for JP4, 2 for JPEG
    struct frameparspair_t  pars_to_update[29];
    int nupdate=0;

    int min_frame_blanking_lines = thispars->pars[P_SENSOR_REGS+P_MT9F002_MIN_FRAME_BLANKING_LINES];
    int min_line_blanking_pck    = thispars->pars[P_SENSOR_REGS+P_MT9F002_MIN_LINE_BLANKING_PCK];
    int min_line_length_pck      = thispars->pars[P_SENSOR_REGS+P_MT9F002_MIN_LINE_LENGTH_PCK];
    //int min_line_length_pck = 0x04c8;

    int ww_odd_inc = 1;
    int fll, llp;

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame

    //pr_info("mt9f002_pgm_window_common: %d\n",thispars->pars[P_EXPOS]);

    dh=  thispars->pars[P_DCM_HOR];
    dv=  thispars->pars[P_DCM_VERT];
    bh=  thispars->pars[P_BIN_HOR];
    bv=  thispars->pars[P_BIN_VERT];
    //wws = sensor->clearLeft;
    ww  = thispars->pars[P_SENSOR_PIXH] * dh;
    //wwe = ww + wws - 1;
    //whs = sensor->clearTop;
    wh  = thispars->pars[P_SENSOR_PIXV] * dv;
    //whe = whs + wh + MT9F002_VACT_DELAY * dv - 1;

    // assuming same for horizontal(H) and vertical(V), margins are from both sides
    compressor_margin = (thispars->pars[P_SENSOR_PIXH] - thispars->pars[P_WOI_WIDTH]) >> 1;
    //pr_info("Compressor margin = %d\n",compressor_margin);

    // 10338 is _not_ flipped (as the other boards, but for legacy compatibility....)
    flip=((thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1) << 1 )) ^ sensor->init_flips;
    flipX =  flip & 1;
    flipY = (flip & 2)? 1:0;
    d = 2 * bh * (ww / (2 * bh));

    // correct window width if needed
    // is this needed?
    if (unlikely(d != ww)) {
        SETFRAMEPARS_SET(P_SENSOR_PIXH, d / dh);
        ww = d;
        //wwe = ww + wws - 1;
    }

    // height
    d = (wh+1) & 0xfffe; // round up to even
    // correct window height if needed
    // is this needed?
    if (unlikely(d != wh)) {
        SETFRAMEPARS_SET(P_SENSOR_PIXV, d / dv);
        wh = d;
        //whe = whs + wh + MT9F002_VACT_DELAY * dv - 1;
    }

    // Margins
    wl = thispars->pars[P_WOI_LEFT] - compressor_margin;
    wt = thispars->pars[P_WOI_TOP]  - compressor_margin;

    dev_dbg(g_dev_ptr,"{%d}   wl =0x%x, wt=0x%x, ww=0x%x, wh=0x%x, compressor_margin=0x%x\n",sensor_port, wl, wt, ww, wh, compressor_margin);

    // flip margins for mirrored images (except oversized, not to rely on sensor->clearWidth/sensor->clearHeight
    // get back to this
    if (!thispars->pars[P_OVERSIZE]) {

        if(flipX) {
        	//pr_info("{%d} flipX = %d\n",sensor_port, flipX);
            wl = sensor->clearWidth - ww - wl - (2 * COLOR_MARGINS) * dh;
            if(wl < 0) wl = 0;
        }
        if(flipY) {
        	//pr_info("{%d} flipY = %d\n",sensor_port, flipY);
            wt = sensor->clearHeight - wh - wt - (2 * COLOR_MARGINS) * dv;
            if(wt < 0) wt = 0;
        }
        // apply clearTop/clearLeft
        wt = (wt + sensor->clearTop) & 0xfffe;
        wl = (wl + sensor->clearLeft) & 0xfffe;
        dev_dbg(g_dev_ptr,"{%d} wl =0x%x, wt=0x%x\n",sensor_port, wl, wt);
        //pr_info("{%d} wl =0x%x, wt=0x%x\n",sensor_port, wl, wt);

        /*
        // apply binning restrictions
		d = (bh > 1) ? ((bh > 2) ? 16 : 8) : 4;
		if(flipX)
			i = d * ((wl - 2) / d) + 2;
		else
			i = d * (wl / d);
		if(i < wl)
			i += d;
		wl = i;
		*/
    }

    wws = wl;
    wwe = wws + ww - 1;

    // need sensor->clearTop-MT9F002_VACT_DELAY - beacause the 1st 2 rows are ignored
    whs = wt - MT9F002_VACT_DELAY;
    //whe = whs + wh + MT9F002_VACT_DELAY*(dv+1) - 1; // ?!
    whe = whs + wh + MT9F002_VACT_DELAY*(dv) - 1;
    // will be different if binning/skipping
    wh = whe - whs + 1;

    // program sensor width
    if (wws != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_ADDR_START]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_ADDR_START, wws);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_MT9F002_X_ADDR_START, (int) wws);
    }
    if (wwe != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_ADDR_END]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_ADDR_END, wwe);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_MT9F002_X_ADDR_END, (int) wwe);
    }
    if (ww != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_OUTPUT_SIZE]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_OUTPUT_SIZE, ww);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_MT9F002_X_OUTPUT_SIZE, (int) ww);
    }

    // program sensor height
    if (whs!= thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_ADDR_START]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16 ,P_MT9F002_Y_ADDR_START, whs);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_SENSOR_REGS+P_MT9F002_Y_ADDR_START, (int) whs);
    }
    if (whe!= thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_ADDR_END]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16 ,P_MT9F002_Y_ADDR_END, whe);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_SENSOR_REGS+P_MT9F002_Y_ADDR_END, (int) whe);
    }
    if (wh != thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_OUTPUT_SIZE]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_Y_OUTPUT_SIZE, wh);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16, (int) P_MT9F002_Y_OUTPUT_SIZE, (int) wh);
    }

    // recalc something after this one?
    //fll = mt9f002_calc_frame_length_lines(thispars);
    fll = mt9f002_calc_frame_length_lines(whs,whe,min_frame_blanking_lines);
    // this sets the vertical blanking
    if (fll!=thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES]){
    	dev_dbg(g_dev_ptr,"limit fps, old frame_length_lines=0x%08x, new frame_length_lines=0x%08x\n",
    			(int) thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES], fll);
    	SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_FRAME_LENGTH_LINES, fll);
    }

    // need this fix for max possible fps
    if (ww >= X_OUTPUT_BORDER_SIZE){
    	min_line_length_pck = MIN_LINE_LENGTH_PCK_FROM_DATASHEET;
    }
    // else, leave as thispars->pars[P_SENSOR_REGS+P_MT9F002_MIN_LINE_LENGTH_PCK]
    // which is 0x930

    // recalc exposure after this one
    //llp = mt9f002_calc_line_length_pck(thispars);

    llp = mt9f002_calc_line_length_pck(wws,wwe,ww_odd_inc,ww,min_line_blanking_pck,min_line_length_pck);
    if (llp != thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK]){
    	SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_LINE_LENGTH_PCK, llp);
    }

    // write flips and skips
    // reg 0x3040 = P_REG_MT9F002_READ_MODE

    //P_MT9F002_READ_MODE
    // bits
    //   [15] - vertical flip,   0 - normal, 1 - vertical flip, starts from y_addr_end
    //   [14] - horizontal flip, 0 - normal, 1 - horizontal flip, start from x_addr_end
    //   [13:12] - reserved
    //   [11] - X dir analog binning enable - when set:
    //                                                y_odd_inc must be set to 1
    //                                                x_odd_inc must be set to 3 for col binning
    //                                                                         7 for col skipping and binning, along with other register changes
    //   [10] - X and Y dir binning enable - when set:
    //                                                y_odd_inc & x_odd_inc must be set to 3 for binning
    //                                                                                     7 for binning and skipping
    //   [9] - reserved
    //   [8:6] - read_mode_x_odd_inc: 1 - normal,
    //                                3 - read out alternate px pairs to halve horizontal data in a frame
    //                                7 - read out 1 of 4 pixel pairs to reduce readout 1/4 the amout of pixels
    //
    //   [5:0] - read_mode_y_odd_inc: 1  - normal,
    //                                3  - read out alternate px pairs to halve vertical amount of pixels in a frame
    //                                7  - 1/4
    //                                15 - 1/8
    //                                31 - 1/16
    //                                63 - 1/32

    // the fields that should not change in non-stop will not change, so it is OK to write them always
    /*
    if((styp == MT9T_TYP) || (styp == MT9P_TYP)) { // 3MPix and 5MPix sensors
        v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM] & 0xff88) | ((bv - 1) << 4) | (dv - 1) ;
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM]) {
            SET_SENSOR_PAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RAM, v);
            dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_PAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port,  sensor_port, frame16,   (int) sensor->i2c_addr, (int) P_MT9X001_RAM, (int) v);
        }
        v=(thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM] & 0xff88) | ((bh - 1) << 4) | (dh - 1);
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM]) {
            SET_SENSOR_PAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_CAM, v);
            dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_PAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_CAM, (int) v);
        }
        v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fff) | // preserve other bits from shadows
                (flipX ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
                (flipY ? (1 << 15) : 0) ; // FLIPV
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE2, v);
            dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_RMODE2, (int) v);

        }
    } else { // 1.3 and 2 MPix sensors
        v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1] & 0xffc3) | // preserve other bits from shadows (trigger mode moved to other function)
                ((dh == 4) ? (1 << 2) : 0) | // Column skip 4
                ((dv == 4) ? (1 << 3) : 0) | // Row skip    4
                ((dh == 8) ? (1 << 4) : 0) | // Column skip 8
                ((dv == 8) ? (1 << 5) : 0) ; // Row skip    8
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE1, v);
            dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_RMODE1, (int) v);
        }
        v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fe7) | // preserve other bits from shadows
                ((dh == 2) ? (1 << 3) : 0) | // Column skip 2
                ((dv == 2) ? (1 << 4) : 0) | // Row skip    2
                (flipX ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
                (flipY ? (1 << 15) : 0) ; // FLIPV
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE2, v);
            dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_RMODE2, (int) v);
        }

    }
    */
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return 0;
}

/**
 * get TRIG_PERIOD from framepars, compare and update pix_period
 * @param pix_period
 * @param thispars
 * @return updated pix_period
 */
int compare_to_trig_period_mt9f002(int sensor_port,               ///< sensor port - only for debug
		 	 	 	 	           int pix_period,                ///< current pix_period
						           struct framepars_t * thispars) ///< sensor current parameters
{

	int trig_period;

    dev_dbg(g_dev_ptr,"{%d} thispars->pars[P_TRIG] = %d, thispars->pars[P_TRIG_PERIOD] =%d(0x%x)\n",
            sensor_port,
			(int)thispars->pars[P_TRIG],
			(int)thispars->pars[P_TRIG_PERIOD],
			(int)thispars->pars[P_TRIG_PERIOD]);

	if (thispars->pars[P_TRIG]!=0){
		trig_period = camsync_to_sensor(thispars->pars[P_TRIG_PERIOD], thispars->pars[P_CLK_SENSOR]);
		if (trig_period > pix_period) {
			pix_period=trig_period;
		}
	}

	return pix_period;
}

/** Check if compressor can keep up, limit sensor FPS if needed
 * FPS is limited by increasing verical blanking, it can not be be made too big, so this method does not work to make time lapse rate. horisontal blanking
 * is kept at minimum (to reduce ERS effect) if not specified. If it is specified (>minimal) - it will be used instead.
 * calculate line period.
 *
 * FIXME - uses P_VIRTUAL_WIDTH w/o decreasing it when changing image size? Replace VIRT_WIDTH with HOR_BANK?
 * Or require always set them to zero when chnaging WOI?
 * FIXME for multisensor - needs per-sensor individual parameters. This uses sensor registers, not the general parameters (i.e. height) */
int mt9f002_pgm_limitfps   (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{

	struct frameparspair_t pars_to_update[16]; // maximum 7 registers updated (need to recount)
    int nupdate=0;

    // decimation horizontal
    int dh=  thispars->pars[P_DCM_HOR]?thispars->pars[P_DCM_HOR]:1;
    // width = PIXELS HORIZONTAL
    int ww=  thispars->pars[P_SENSOR_PIXH] * dh;

    int binning_cost = 0;
    int width,i;
    int row_time_in_pixels=0;
    int hor_blank_min;
    int hor_blank=0;
    int p1, p2, pix_period, sclk,fp1000s, trig_period;

    int height;
    int virt_height;
    int virt_height_min;
    int vert_blank;

    uint64_t ull_fp1000s;

    int target_virt_width=(thispars->pars[P_VIRT_KEEP])?(thispars->pars[P_VIRT_WIDTH]):0;

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame

    //pr_info("mt9f002_pgm_limitfps: %d\n",thispars->pars[P_EXPOS]);


    width = 2 * ww / (2 * dh);

	if((width * dh) < ww) width++;

	/*
	switch(thispars->pars[P_BIN_VERT]) {
	case 2:
		switch(thispars->pars[P_BIN_HOR]) {
		case 1: binning_cost = 276;    break;
		case 2: binning_cost = 236;    break;
		case 4: binning_cost = 236;    break;
		break;
		}
		break;
	case 4:
		switch(thispars->pars[P_BIN_HOR]) {
		case 1: binning_cost = 826;    break;
		case 2: binning_cost = 826;    break;
		case 4: binning_cost = 826;    break;
		break;
		}
		break;
	}
	*/

	//hor_blank_min = 0x138;

	//dev_dbg(g_dev_ptr,"{%d} hor_blank_min =%d(0x%x)\n",sensor_port,hor_blank_min,hor_blank_min);
	//hor_blank = hor_blank_min;
	//dev_dbg(g_dev_ptr,"{%d} hor_blank =%d(0x%x)\n",sensor_port,hor_blank,hor_blank);
	row_time_in_pixels = 2*thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK];

	// fps correction for smaller window widths
	if (ww < X_OUTPUT_BORDER_SIZE){
		row_time_in_pixels = thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK];
	}

	//row_time_in_pixels = mt9f002_calc_line_length_pck(thispars);


	//dev_dbg(g_dev_ptr,"{%d} row_time_in_pixels =%d(0x%x)\n",sensor_port,row_time_in_pixels,row_time_in_pixels);
	//i = 2 * (41 + 208 * thispars->pars[P_BIN_VERT] + 99); //strange 41 and 99, why not 140?
	//if(i > row_time_in_pixels)  row_time_in_pixels = i;

	/*
	dev_dbg(g_dev_ptr,"{%d} row_time_in_pixels =%d(0x%x)\n",sensor_port,row_time_in_pixels,row_time_in_pixels);
	if(target_virt_width > row_time_in_pixels) { // extend line by adding horizontal blanking
		hor_blank = target_virt_width - width;
		if (hor_blank > sensor->maxHorBlank) hor_blank = sensor->maxHorBlank;
		row_time_in_pixels = width + hor_blank;
		dev_dbg(g_dev_ptr,"{%d} row_time_in_pixels =%d(0x%x)\n",sensor_port,row_time_in_pixels,row_time_in_pixels);
	}
	*/

    // schedule updating P_VIRT_WIDTH if it changed FIXME: Does not come here?

    dev_dbg(g_dev_ptr,"{%d} row_time_in_pixels = %d(0x%x), thispars->pars[P_VIRT_WIDTH] = %d(0x%x)\n",
    	sensor_port, row_time_in_pixels,
		             row_time_in_pixels,
		             (int)thispars->pars[P_VIRT_WIDTH],
					 (int)thispars->pars[P_VIRT_WIDTH]);

    if (thispars->pars[P_VIRT_WIDTH] != row_time_in_pixels) {
        SETFRAMEPARS_SET(P_VIRT_WIDTH, row_time_in_pixels);
    }

    /*
    // schedule updating P_MT9X001_HORBLANK senosr register and shadow FIXME: Seems hor_blank is too high. is the width itself subtracted?
    if (hor_blank != thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]) {
        dev_dbg(g_dev_ptr,"{%d} hor_blank =%d(0x%x), thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]=%d(0x%x)\n",sensor_port,hor_blank,hor_blank,(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK],(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]);
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9X001_HORBLANK, hor_blank);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_HORBLANK, (int)hor_blank);
    }
    */
    // Now calculate P_PERIOD (extending it as needed)
    //   int vh, vb, wh, h, dv, sclk, row_time_in_pixels, ve, e, i;
    // calculate minimal virtual heigth for current window height
    //pgm_limitfps
    /* NOTE: Was this for a long time - make sure replacement does not break anything !!!
     *
  	  int wh = thispars->pars[P_SENSOR_REGS+P_MT9X001_HEIGHT] + 1;
  	  int dv = thispars->pars[P_DCM_VERT];
  	  int height = 2 * (wh / (2 * dv));
  	  if((height * dv) < wh) height++;
    */

    height = thispars->pars[P_SENSOR_PIXV];
    // this should be based on exposure
    //virt_height = height + sensor->minVertBlank;
    //virt_height = mt9f002_calc_frame_length_lines(thispars);

    // this is allowed minimum
    virt_height = thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES];

    if (thispars->pars[P_VIRT_KEEP]) {
        if (virt_height < thispars->pars[P_VIRT_HEIGHT]) {
            virt_height = thispars->pars[P_VIRT_HEIGHT];
        }
    }

    dev_dbg(g_dev_ptr,"{%d} height =%d(0x%x), virt_height=%d(0x%x)\n",sensor_port,height,height,virt_height,virt_height);
    // limit frame rate (using minimal period), but only in sync mode - async should be programmed to observe minimal period
    // ignore trig - period
    //if ((thispars->pars[P_TRIG] & 4) == 0) {
    	// this is calculated minimum for compressor
    	virt_height_min = thispars->pars[P_PERIOD_MIN]/row_time_in_pixels; // always non-zero, calculated by pgm_limitfps (common)
        if ((row_time_in_pixels * virt_height_min) < thispars->pars[P_PERIOD_MIN]) virt_height_min++; //round up
        if (virt_height < virt_height_min) virt_height = virt_height_min;
        dev_dbg(g_dev_ptr,"{%d} height =%d(0x%x), modified virt_height=%d(0x%x)\n",sensor_port,height,height,virt_height,virt_height);
    //}

    // schedule updating P_VIRT_HEIGHT if it changed
    dev_dbg(g_dev_ptr,"{%d} thispars->pars[P_VIRT_HEIGHT] =%d(0x%x), virt_height=%d(0x%x)\n",
    		sensor_port,
			(int)thispars->pars[P_VIRT_HEIGHT],
			(int)thispars->pars[P_VIRT_HEIGHT],
			virt_height,
			virt_height);

    // no need to set it here, can update in pgm_exposure
    if (virt_height != thispars->pars[P_VIRT_HEIGHT]) {
    	//pr_info("Will set virt_height to %d\n",virt_height);
        SETFRAMEPARS_SET(P_VIRT_HEIGHT, virt_height);
    }

    // schedule updating P_PERIOD if it changed
    pix_period=row_time_in_pixels*virt_height;
    if (!pix_period ){
        dev_warn(g_dev_ptr,"** mt9f002_pgm_limitfps(%d) pix_period == 0!! (row_time_in_pixels =%d(0x%x), virt_height=%d(0x%x)\n",sensor_port,row_time_in_pixels,row_time_in_pixels,virt_height,virt_height);
        pix_period = 1000; // just non-zero
    }

    //pr_info("check 0: pix_period = 0x%08x (0x%08x*0x%08x)\n",pix_period,row_time_in_pixels,virt_height);

    // IMPORTANT: this is moved above setting P_PERIOD
    // Update period from external trigger (assuming internal/self trigger, we do not know real external trigger period)
    //pix_period = compare_to_trig_period_mt9f002(sensor_port,pix_period,thispars);

    dev_dbg(g_dev_ptr,"{%d} thispars->pars[P_PERIOD] =%d(0x%x), pix_period=%d(0x%x)\n",sensor_port,(int)thispars->pars[P_PERIOD],(int)thispars->pars[P_PERIOD],pix_period,pix_period);
    if (thispars->pars[P_PERIOD] != pix_period) {
        SETFRAMEPARS_SET(P_PERIOD, pix_period);
    }

    // switched order
    // Update period from external trigger (assuming internal/self trigger, we do not know real external trigger period)
    //pix_period = compare_to_trig_period(pix_period,thispars);

    sclk = thispars->pars[P_CLK_SENSOR];

    ull_fp1000s=((long long) 1000)* ((long long) sclk);
    __div64_32(&ull_fp1000s,pix_period);
    fp1000s= ull_fp1000s;

    //pr_info("check 1: FP1000S = 0x%08x\n",fp1000s);

    dev_dbg(g_dev_ptr,"{%d} thispars->pars[P_FP1000S] =%d(0x%x), fp1000s=%d(0x%x)\n",sensor_port,(int)thispars->pars[P_FP1000S],(int)thispars->pars[P_FP1000S],fp1000s,fp1000s);
    if (thispars->pars[P_FP1000S] != fp1000s) {
    	//pr_info("pgm_limitfps fp1000s = %d\n",fp1000s);
        SETFRAMEPARS_SET(P_FP1000S, fp1000s);
    }

    // update to compressor minimum
    if (virt_height != thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES]){
    	//pr_info("LIMIT_FPS, old frame_length_lines=0x%08x, new frame_length_lines=0x%08x\n",
    	//		thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES],virt_height);
    	SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_FRAME_LENGTH_LINES, virt_height);
    }

    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}

/** Program sensor exposure */
int mt9f002_pgm_exposure (int sensor_port,               ///< sensor port number (0..3)
                          struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                          struct framepars_t * thispars, ///< sensor current parameters
                          struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                          int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                         ///< be applied to,  negative - ASAP
                                                         ///< @return 0 - OK, negative - error
{

    uint64_t ull_fp1000s;
    uint64_t ull_exposure;
    uint64_t ull_video_exposure;

    int fp1000s;
    struct frameparspair_t pars_to_update[16]; // maximum 7? registers updated
    int nupdate=0;

    // P_VEXPOS is the coarse exposure, reg value
    int video_exposure = thispars->pars[P_VEXPOS];
    // when P_VEXPOS is used exposure will be overwritten
    // set in microseconds
    int exposure       = thispars->pars[P_EXPOS];
    // this sensor has the following 2 registers
    int coarse_exposure;
    int fine_exposure;

    int frame_length_lines;// = thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES];
    int virt_height;
    //int line_length_pck = thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK];

    // vt_pix_clk
    // TODO: Do not forget this - add those div registers?!
    //int sclk = thispars->pars[P_CLK_SENSOR]*MT9F002_PLL_MULTIPLIER_VALUE*0x2/0x6/0x6; // pixel clock in Hz before PLL, but PLL setting is ~240MHz?
    int sclk = thispars->pars[P_CLK_SENSOR];// = (thispars->pars[P_CLK_SENSOR]/0x6/0x6)*0x2*MT9F002_PLL_MULTIPLIER_VALUE;

    // NOTE: Is decimation taken care of here ??? FIXME: probably not (here and when deciding to use exposure, not number of lines)!!!
    int row_time_in_pixels=thispars->pars[P_VIRT_WIDTH];
    int pix_period;

    // wrong frame
    if (frame16 >= PARS_FRAMES) return -1;

    frame_length_lines = thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES];
    virt_height = thispars->pars[P_VIRT_HEIGHT];
    //frame_length_lines = thispars->pars[P_VIRT_HEIGHT];

    //pr_info("EXPOS: %d\n",thispars->pars[P_EXPOS]);

    exposure = thispars->pars[P_EXPOS];
    //pr_info("mt9f002_pgm_exposure: %d\n",thispars->pars[P_EXPOS]);

    //pr_info("fll = %d vs VIRT_HEIGHT = %d\n",frame_length_lines,thispars->pars[P_VIRT_HEIGHT]);

    // if yes: use number of lines == use P_VEXPOS for calculations, update P_EXPOS as well
    // else  : use P_EXPOS
    if ((video_exposure>0)&&(FRAMEPAR_MODIFIED(P_VEXPOS)||!(FRAMEPAR_MODIFIED(P_EXPOS)||FRAMEPAR_MODIFIED(P_VIRT_WIDTH)))) {
    	//pr_info("using P_VEXPOS\n");

    	coarse_exposure = video_exposure;

    	ull_exposure = ((long long)(video_exposure * row_time_in_pixels)) * ((long long) 1000000);
    	__div64_32(&ull_exposure,sclk);
    	exposure = ull_exposure;

    }else{

    	//pr_info("using P_EXPOS\n");
    	// exposure is in microseconds
    	exposure        = thispars->pars[P_EXPOS];

    	//line_length_pck = thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK];
    	// always start with min line_length_pck for the current window
    	//line_length_pck = mt9f002_calc_line_length_pck(thispars);

    	//pr_info("exposure = 0x%08x, line_length_pck = 0x%08x\n",exposure,line_length_pck);

    	// actual equation is : sclk*(exposure/10^6) - the result is in Hz*s
    	ull_video_exposure = (long long) (sclk / 1000000) * (long long) exposure;
    	// result is a reg value for coarse exposure
    	__div64_32(&ull_video_exposure,row_time_in_pixels);

    	coarse_exposure = ull_video_exposure;
    	//frame_length_lines = thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES];

    	//pr_info("coarse exposure = 0x%08x vs 0x%08x = frame_length_lines \n",coarse_exposure,frame_length_lines);

    	/*
    	if (coarse_exposure>(frame_length_lines-1)){
    		pr_info("Coarse exposure is greater than frame_length_lines\n");
    		// set coarse to max
    		coarse_exposure = frame_length_lines-1;
    		ull_video_exposure = (long long) (sclk / 1000000) * (long long) exposure;
    		__div64_32(&ull_video_exposure,coarse_exposure);
    		// extend line length
    		line_length_pck = ull_video_exposure;
    	}
    	*/


    	// at the same time the fine exposure will be:
    	ull_video_exposure = (long long) (sclk / 1000000) * (long long) exposure;
    	fine_exposure = ull_video_exposure - (long long) coarse_exposure * (long long) row_time_in_pixels;

    	//pr_info("fine exposure = %d (0x%08x)\n",fine_exposure,fine_exposure);

    }

    if (exposure <1)       exposure=1;
    if (coarse_exposure <1) coarse_exposure=1;

    // check against max shutter?
    // is exposure longer then maximal period (specified for constant fps?
    pix_period = row_time_in_pixels*frame_length_lines;

    // IMPORTANT: inserted here by myself for testing
    //pix_period = compare_to_trig_period(sensor_port,pix_period,thispars);

    // 0x2 == keep frame rate
    // 0x1 == limit frame rate
    if (thispars->pars[P_FPSFLAGS] & 0x2) {
    	if (pix_period > thispars->pars[P_PERIOD_MAX]) {
    		// already done this?!
    		coarse_exposure = virt_height-1;
    		//ull_video_exposure = (long long) (sclk / 1000000) * (long long) exposure;
    		//__div64_32(&ull_video_exposure,coarse_exposure);
    		// extend line length
    		//row_time_in_pixels = ull_video_exposure;
    	}
    }else{
    	// update fps here
    	if (coarse_exposure >= frame_length_lines-1){
    		virt_height = coarse_exposure + 1;
    	}else{
    		virt_height = frame_length_lines;
    	}

    	pix_period = row_time_in_pixels*virt_height;
    }

	if (virt_height != thispars->pars[P_VIRT_HEIGHT]) {
		//pr_info("pgm_exposure: Will set virt_height to %d\n", virt_height);
	    SETFRAMEPARS_SET(P_VIRT_HEIGHT, virt_height);
	}

	SETFRAMEPARS_SET(P_PERIOD, pix_period);
	ull_fp1000s=((long long) 1000)* ((long long) sclk);
	__div64_32(&ull_fp1000s,pix_period);
	fp1000s= ull_fp1000s;

	if (thispars->pars[P_FP1000S] != fp1000s) {
		//pr_info("pgm_exposure fp1000s = %d\n",fp1000s);
		SETFRAMEPARS_SET(P_FP1000S, fp1000s);
	}

    /*
    dev_dbg(g_dev_ptr,"{%d} sensor_port=%d,  frame16=%d, frame=0x%lx (%s)exposure=0x%lx, (%s)video_exposure=0x%lx\n",sensor_port, sensor_port, frame16, thispars->pars[P_FRAME], FRAMEPAR_MODIFIED(P_EXPOS)?"*":" ",thispars->pars[P_EXPOS],FRAMEPAR_MODIFIED(P_VEXPOS)?"*":" ",thispars->pars[P_VEXPOS] );
    dev_dbg(g_dev_ptr,"{%d}  row_time_in_pixels=0x%x\n",sensor_port, row_time_in_pixels); // 0
    //vert_blank=        thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK ];
    vert_blank=0;

    dev_dbg(g_dev_ptr,"{%d}  vert_blank=0x%x\n",sensor_port,vert_blank); // 0
    // if video exposure is non-zero, P_VEXPOS is marked as modified or P_EXPOS is not modified - use video exposure (lines), else - absolute exposure (usec)
    if ((FRAMEPAR_MODIFIED(P_VEXPOS) || ! (FRAMEPAR_MODIFIED(P_EXPOS) || FRAMEPAR_MODIFIED(P_VIRT_WIDTH)) )) { // use number of lines

    	dev_dbg(g_dev_ptr,"{%d}  exposure=%d (0x%x), video_exposure=%d (0x%x)\n",sensor_port, (int) thispars->pars[P_VEXPOS], (int) thispars->pars[P_VEXPOS], (int) video_exposure, (int) video_exposure);
#if USELONGLONG
        ull_exposure= ((long long)(video_exposure * row_time_in_pixels)) * ((long long) 1000000);
#ifdef __div64_32
        __div64_32(&ull_exposure, sclk);
#else
        do_div(ull_exposure, sclk);
//        ull_exposure /= sclk;
#endif
        exposure= ull_exposure;
#else
        exposure = (100*video_exposure * row_time_in_pixels) / (sclk/10000); // in microseconds
#endif
        use_vexp=1;
    } else { // use time in microseconds
        exposure = thispars->pars[P_EXPOS];
#if USELONGLONG
        ull_video_exposure= (long long) exposure  * (long long) sclk;
#ifdef __div64_32
        __div64_32(&ull_video_exposure, row_time_in_pixels);
        __div64_32(&ull_video_exposure, 1000000);
#else
        do_div(ull_video_exposure, row_time_in_pixels);
        do_div(ull_video_exposure, 1000000);
//        ull_video_exposure /= row_time_in_pixels;
//        ull_video_exposure /= 1000000;
#endif
        video_exposure= ull_video_exposure;
#else
        ///TODO - use shifts, not division where possible?
        if (exposure<10000) { // <0.01 sec
            video_exposure = ( exposure        * (sclk/1000))/ (row_time_in_pixels*1000);
        } else if (exposure<100000) { // 0.1 sec
            video_exposure = ( (exposure/10)   * (sclk/1000))/ (row_time_in_pixels * 100);
        } else if (exposure<1000000) { // 1.0 sec
            video_exposure = ( (exposure/100)  * (sclk/1000))/ (row_time_in_pixels * 10);
        } else {
            video_exposure = ( (exposure/1000) * (sclk/1000))/ (row_time_in_pixels );
        }
#endif
    }
    if (exposure <1)       exposure=1;
    if (video_exposure <1) video_exposure=1;

    // is video exposure longer than maximal for the sensor?
    if (video_exposure  > sensor->maxShutter) {
        video_exposure=sensor->maxShutter;
#if USELONGLONG
        ull_exposure= ((long long)(video_exposure * row_time_in_pixels)) *((long long) 1000000);
#ifdef __div64_32
        __div64_32(&ull_exposure, sclk);
#else
        do_div(ull_exposure, sclk);
//        ull_exposure /= sclk;
#endif
        exposure= ull_exposure;
#else
        exposure = (100*video_exposure * row_time_in_pixels) / (sclk/10000); // in microseconds
#endif
    }

    // is exposure longer then maximal period (specified for constant fps?
    pix_period=video_exposure*row_time_in_pixels;

    // IMPORTANT: inserted here by myself for testing
    pix_period = compare_to_trig_period(sensor_port,pix_period,thispars);

    if (thispars->pars[P_FPSFLAGS] & 2) {
        if (pix_period > thispars->pars[P_PERIOD_MAX]) {
            video_exposure=thispars->pars[P_PERIOD_MAX]/row_time_in_pixels;
#if USELONGLONG
            ull_exposure= (((long long) thispars->pars[P_PERIOD_MAX]) *((long long)  1000000));
#ifdef __div64_32
            __div64_32(&ull_exposure, sclk);
#else
            do_div(ull_exposure, sclk);
//            ull_exposure /= sclk;
#endif
            exposure= ull_exposure;
#else
            exposure = (thispars->pars[P_PERIOD_MAX] * 100) / (sclk/10000); // in microseconds
#endif
        }
    } else { // no limit on maximal period
        // is exposure increasing period (not limited by ([P_FPSFLAGS] & 2) ? In that case P_PERIOD and P_FP1000S will need to be updated
        // schedule updating P_PERIOD if it changed
        ///TODO: Check duplicate vert_blank calculation (mt9x001_pgm_limitfps)

        if (pix_period > thispars->pars[P_PERIOD]) {
            SETFRAMEPARS_SET(P_PERIOD, pix_period);
            // schedule updating P_FP1000S if it changed
#if USELONGLONG
            ull_fp1000s=((long long) 1000)* ((long long) sclk);
#ifdef __div64_32
            __div64_32(&ull_fp1000s,pix_period);
#else
            do_div(ull_fp1000s,pix_period);
//            ull_fp1000s /= pix_period;
#endif
            fp1000s= ull_fp1000s;
#else
            fp1000s= 10*sclk/(pix_period/100);
#endif
            dev_dbg(g_dev_ptr," fp1000s=%d (0x%x)", (int) fp1000s, (int) fp1000s);

            if (thispars->pars[P_FP1000S] != fp1000s) {
                SETFRAMEPARS_SET(P_FP1000S, fp1000s);
            }
        }
    }

    */
    // is video exposure P_VEXPOS modified?
    if (thispars->pars[P_VEXPOS] != coarse_exposure) {
        SETFRAMEPARS_SET(P_VEXPOS, coarse_exposure);
    }
    // is exposure P_EXPOS modified?
    if (thispars->pars[P_EXPOS] != exposure) {
        SETFRAMEPARS_SET(P_EXPOS, exposure);
    }

    // Now sensor registers
    // schedule updating P_MT9X001_VERTBLANK sensor register and shadow

    /*
    // this sets the vertical blanking
    if (frame_length_lines!=thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES]){
    	//pr_info("limit fps, old frame_length_lines=0x%08x, new frame_length_lines=0x%08x\n",thispars->pars[P_SENSOR_REGS+P_MT9F002_FRAME_LENGTH_LINES],virt_height);
    	SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_FRAME_LENGTH_LINES, frame_length_lines);
    }
    */

    // coarse integration time
    if (coarse_exposure != thispars->pars[P_SENSOR_REGS+P_MT9F002_COARSE_INTEGRATION_TIME]) {
        SET_SENSOR_MBPAR_LUT(sensor_port,frame16,P_MT9F002_COARSE_INTEGRATION_TIME, coarse_exposure);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,(int)P_MT9F002_COARSE_INTEGRATION_TIME,(int)coarse_exposure);
    }

    // fine integration time
    /*
    if (fine_exposure != thispars->pars[P_SENSOR_REGS+P_MT9F002_FINE_INTEGRATION_TIME]) {
        SET_SENSOR_MBPAR_LUT(sensor_port,frame16,P_MT9F002_FINE_INTEGRATION_TIME, fine_exposure);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,(int) sensor->i2c_addr,(int)P_MT9F002_FINE_INTEGRATION_TIME,(int)fine_exposure);
    }
    */

    if (nupdate) setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows

    dev_dbg(g_dev_ptr,"{%d} coarse exposure = %d (0x%x),  line_length_pck = %d (0x%08x),  fine exposure = %d (0x%x)\n",
    		sensor_port,
			(int) coarse_exposure,   (int) coarse_exposure,
			(int) row_time_in_pixels,(int) row_time_in_pixels,
			(int) fine_exposure,     (int) fine_exposure);

    return 0;
}

/**
 * new: Get gain table index by register value
 * TODO: reduce hardcoding
 * What if the table is recalibrated?
 * @param gain
 * @return table index
 */
int gain_get_table_index_by_reg_mt9f002(unsigned long gain){

	int colamp_gain;
	int a2_gain;
	int index_base = 0;
	int index_inc = 0;
	int index = 0;

	colamp_gain = (gain>>10)&0x3;
	a2_gain = gain&0x7f;

	switch(colamp_gain){
	case 1:
		index_base = 0;
		break;
	case 2:
		index_base = 48;
		break;
	case 3:
		index_base = 96;
		break;
	}

	index_inc = a2_gain - 0x30;

	index = index_base + index_inc;

	return index;
}

/**
 * new: Get initial approximation of table index by parameter value
 * TODO: reduce hardcoding
 * @param g
 * @return
 */
int gain_get_table_index_by_par_mt9f002(int g){

	int index_base = 0;
	int index_inc = 0;
	int index = 0;
	int colamp_gain = 0;

	if (g>=0x60000){
		index_base = 96;
		colamp_gain = 3;
	}else if (g>=0x30000){
		index_base = 48;
		colamp_gain = 2;
	}else{
		index_base = 0;
		colamp_gain = 1;
	}
	g = g>>colamp_gain;
	index_inc = (g*64)/0x10000 - 0x30;

	if (index_inc<0){
		index_inc = 0;
	}

	index = index_base + index_inc;

	return index;
}

/**
 * new: Calculate reg value from par value
 * TODO: reduce hardcoding
 * @param parvalue
 * @return regvalue (+added sensor's digital gain)
 */
int gain_get_reg_by_par(int parvalue){

	int colamp_gain = 0;
	int a2_gain;
	int regvalue;

	if (parvalue>=0x60000){
		colamp_gain = 3;
	}else if(parvalue>=0x30000){
		colamp_gain = 2;
	}else if(parvalue>=0x18000){
		colamp_gain = 1;
	}

	a2_gain = ((parvalue>>colamp_gain)*64/0x10000)&0x7f;
	// add digital gain = 1 here
	regvalue = 0x1000|(colamp_gain<<10)|a2_gain;
	return regvalue;
}

#define SHIFT_DGAIN 1 // shift digital gain right, so 1.0 is 0x8000 an the full range is 4:1 - make it a parameter?

/** Split full gain (0x1000~1.0) into analog register gain and residual digital gain (also 0x10000~1.0)
 *  provide some hysteresis for analog gain (1/2 step) when goin to positive, but never let residual
 *  gain to be <1.0 (0x10000). Uses gain correction table.  */
unsigned long gain_ajust_mt9f002(
        unsigned long   gain,       ///< required full gain
        unsigned long * gainTab,    ///< pointer to 81 element table of actual gains for each step. Uses calculated ones after init
        unsigned long   curRegGain, ///< current value of the sensor gain register
        unsigned long * newRegGain, ///< pointer to the new value of the sensor gain register
        unsigned long   anaGainEn,  ///< enable analog gain adjustment (0 - use current)
        unsigned long   minGain,    ///< minimal allowed value of the analog gain (normally 0x10000 ~ 1.0)
        unsigned long   maxGain)    ///< maximal allowed value of the analog gain (normally 0xfc000 ~ 15.75)
                                    ///< @return residual value of the digital gain (>=1.0 0x10000) except limited by the minGain
{
    int g=gain;
    int gainIndex;     // index of the gain value in the table (each register value has index, each index has preferrable register value)
    int curGainIndex;  // current index in the gains table matching sensor register value
    uint64_t ull_gain;

    dev_dbg(g_dev_ptr,"gain=0x%lx, curRegGain=0x%lx, minGain=0x%lx, maxGain=0x%lx\n",gain,curRegGain,minGain,maxGain);
    // sensor's digital gain bits are ignored when getting table index
    // curRegGain &=0xfff;
    // find out gain index for the current value of the sensor gain register
    curGainIndex = gain_get_table_index_by_reg_mt9f002(curRegGain);

    if (anaGainEn) {
        if (g<minGain) g=minGain;
        if (g>maxGain) g=maxGain;
        // calculate theoretical gainIndex (0..175)
        gainIndex=gain_get_table_index_by_par_mt9f002(g);
        dev_dbg(g_dev_ptr,"gainIndex=0x%x\n",gainIndex);
        // adjust using gain table
        while ((gainIndex>0)  && (gainTab[gainIndex-1]>=g) && (gainTab[gainIndex-1]>=minGain)) gainIndex--; // adjust down
        while ((gainIndex<175) && (gainTab[gainIndex+1]< g) && (gainTab[gainIndex+1]<=maxGain)) gainIndex++; // adjust up
        dev_dbg(g_dev_ptr,"gainIndex=0x%x\n",gainIndex);
        // Apply hysteresis
        if ((gainIndex==(curGainIndex+1)) && (((gainTab[gainIndex]+gainTab[gainIndex+1])>>1)>g)) gainIndex--;
        dev_dbg(g_dev_ptr,"gainIndex=0x%x\n",gainIndex);
        //  did the analog gain change?
        *newRegGain=gain_get_reg_by_par(gainTab[gainIndex]);
    }else {
        gainIndex=curGainIndex;
        *newRegGain=curRegGain;
    }
    dev_dbg(g_dev_ptr,"*newRegGain=0x%lx\n",*newRegGain);
    // now divide gains
    ull_gain =((long long) gain) << 16;
    __div64_32(&ull_gain, gainTab[gainIndex]);
    dev_dbg(g_dev_ptr,"((unsigned long) ull_gain)=0x%lx\n",(unsigned long) ull_gain);
    return ((unsigned long) ull_gain) >> SHIFT_DGAIN;
}

/** Apply scale (0x10000~1.0) to data using 64-bit intermediate data */
inline unsigned long mt9f002_applyScale16 (unsigned long data,  ///< 32-bit unsigned data
                                   unsigned long scale) ///< 32 bit (0x10000 for scale 1.0)
                                                        ///< @return scaled result
{
    return  (unsigned long) ((((long long) data)  * scale) >> 16);
}
/** Calculate ratio of the two 32-bit numbers, scaling it by 16 bits, so equal numbers will result in 0x10000 (1.0) using 64 by 32 bit division */
inline unsigned long mt9f002_getScale16 (unsigned long nominator,   ///< 32-bit nominator
                                 unsigned long denominator) ///< 32-bit denominator
                                                            ///< 32 bit result scaled by 16 bits
{
    uint64_t ull_result =((long long) nominator) << 16;
    __div64_32(&ull_result, denominator);
    return (unsigned long) ull_result;
}


/**
 * Program analog gains
 * program analog gains TODO: Make separate read-only P_ACTUAL_GAIN** ?
 * apply sensor-specific restrictions on the allowed gain values
 * includes sensor test mode on/off/selection
 */
#define MAX_DIGITAL_GAIN 0x300 //integer x256 (0x300 ~3.0)
int mt9f002_pgm_gains      (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error

{
    struct frameparspair_t pars_to_update[38]; // 22+12 needed
    int nupdate=0;

    unsigned long newRegGain,digitalGain, testmode;
    unsigned long anaGainEn= (thispars->pars[P_GAIN_CTRL]>> GAIN_BIT_ENABLE) & 1;
    unsigned long minAnaGain=thispars->pars[P_GAIN_MIN];
    unsigned long maxAnaGain=thispars->pars[P_GAIN_MAX];
    unsigned long maxGain;

    int limitsModified=0;
    int gaingModified=FRAMEPAR_MODIFIED(P_GAING);

    unsigned long gainr, gaing, gaingb, gainb;
    unsigned long rscale_all, gscale_all, bscale_all;
    unsigned long rscale, gscale, bscale, rscale_ctl, gscale_ctl, bscale_ctl;
    unsigned long newval;

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);

    // wrong frame
    if (frame16 >= PARS_FRAMES) return -1;

    // NOTES:
    //     - Those gain registers include digital gain - bits [15:12] - include or not?
    //

    ///make sure limits are OK. Allow violating minimal gain here
    if (FRAMEPAR_MODIFIED(P_GAIN_MIN)) {
        limitsModified=1;
        if (minAnaGain < 0x18000) {
            minAnaGain = 0x18000;
            SETFRAMEPARS_SET(P_GAIN_MIN, minAnaGain);
        }
    }
    if (FRAMEPAR_MODIFIED(P_GAIN_MAX)) {
        limitsModified+=2;
        if (maxAnaGain > (sensor->maxGain256 << 8)) { ///sensor->maxGain256 is not calibrated, so digital gain should be able to accomodate for variations
            maxAnaGain = (sensor->maxGain256 << 8);
            SETFRAMEPARS_SET(P_GAIN_MAX, maxAnaGain);
        }
    }
    //  maxGain= maxAnaGain * (MAX_DIGITAL_GAIN >> 8);
    maxGain= (maxAnaGain * MAX_DIGITAL_GAIN)  >> 8; ///should not overflow for Micron as max digital gain <4.0 (0x400), max analog <0x100000)

    gainr= thispars->pars[P_GAINR];
    gaing= thispars->pars[P_GAING];
    gaingb=thispars->pars[P_GAINGB];
    gainb= thispars->pars[P_GAINB];
    rscale_all=thispars->pars[P_RSCALE_ALL];
    gscale_all=thispars->pars[P_GSCALE_ALL];
    bscale_all=thispars->pars[P_BSCALE_ALL];
    rscale=rscale_all & ((1<<CSCALES_WIDTH)-1);
    gscale=gscale_all & ((1<<CSCALES_WIDTH)-1);
    bscale=bscale_all & ((1<<CSCALES_WIDTH)-1);
    rscale_ctl=(rscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);
    gscale_ctl=(gscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);
    bscale_ctl=(bscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);

    ///
    // what's this fixme is about?
    // FIXME: use different gain limitation when anaGainEn==0 (from current analog channel gain to that * MAX_DIGITAL_GAIN>>8),
    //        make P_GAIN_CTRL trigger this function
    ///

    // Scales will not be modified if they make gains out of limit, but gains will be. So automatic white balance should deal with gains, not with scales.
    // Preserving application-set values for scales simplifies recovery when the green gain is adjusted so all colors fit in the limits
    dev_dbg(g_dev_ptr,"{%d}, gainr=0x%lx, gaing=0x%lx, gaingb=0x%lx, gainb=0x%lx, rscale_all=0x%lx, gscale_all=0x%lx, bscale_all=0x%lx\n",
    		sensor_port, gainr, gaing, gaingb, gainb, rscale_all, gscale_all, bscale_all);

    ///Verify that green gain itself is within limits:
    if (FRAMEPAR_MODIFIED(P_GAING) || limitsModified) {
        if (gaing < minAnaGain) {
            gaing = minAnaGain;
            gaingModified=1;
            SETFRAMEPARS_SET(P_GAING, gaing); // Update as it was set too low
        } else if (gaing > maxGain) {
            gaing = maxGain;
            gaingModified=1;
            SETFRAMEPARS_SET(P_GAING, gaing); // Update as it was set too high
        }
    }

    // Second part - combine P_*SCALE and P_GAIN* parameters
    if ((gaingModified || (FRAMEPAR_MODIFIED(P_RSCALE_ALL))) // either green color or red scale changed
            && (rscale_ctl== CSCALES_CTL_NORMAL)             // update red from rscale is enabled
            && !FRAMEPAR_MODIFIED(P_GAINR) )  {              // red gain is not specifically modified

    	// Update red gain to rscale and gaing, limit it if it is out of range
        if (((newval=mt9f002_applyScale16(gaing,rscale)))!=gainr) {
        	if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
        	// don't update if it was already limited in previous frames to the same value
            if (gainr!=newval) {
                gainr=newval;
                SETFRAMEPARS_SET(P_GAINR, gainr);
            }
        }

    }

    if ((gaingModified || (FRAMEPAR_MODIFIED(P_GSCALE_ALL)))
            && (gscale_ctl== CSCALES_CTL_NORMAL)
            && !FRAMEPAR_MODIFIED(P_GAINGB) ) {

        // Update green2 gain to gscale and gaing
        if (((newval=mt9f002_applyScale16(gaing,gscale)))!=gaingb) {
            if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
            // don't update if it was already limited in previous frames to the same value
            if (gaingb!=newval) {
                gaingb=newval;
                SETFRAMEPARS_SET(P_GAINGB, gaingb);
            }
        }

    }

    if ((gaingModified || (FRAMEPAR_MODIFIED(P_BSCALE_ALL)))
            && (bscale_ctl== CSCALES_CTL_NORMAL)
            && !FRAMEPAR_MODIFIED(P_GAINB) ) {
        // Update blue gain to bscale and gaing
        if (((newval=mt9f002_applyScale16(gaing,bscale)))!=gainb) {
            if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
            // don't update if it was already limited in previous frames to the same value
            if (gainb!=newval) {
                gainb=newval;
                SETFRAMEPARS_SET(P_GAINB, gainb);
            }
        }
    }


    // Update scales only if the corresponding color gains (not the base green one) were modified outside of the driver
    // (not as a result of being limited by minimal/maximal gains)
    if ((FRAMEPAR_MODIFIED(P_GAINR))  && (rscale_ctl!= CSCALES_CTL_DISABLE)) {
        rscale=mt9f002_getScale16(gainr, gaing);
        dev_dbg(g_dev_ptr,"{%d} gainr=0x%lx, gaing=0x%lx, rscale=0x%lx\n",sensor_port,gainr, gaing, rscale);
    }
    if ((FRAMEPAR_MODIFIED(P_GAINGB)) && (gscale_ctl!= CSCALES_CTL_DISABLE)) {
        gscale=mt9f002_getScale16(gaingb,gaing);
        dev_dbg(g_dev_ptr,"{%d} gaingb=0x%lx, gaing=0x%lx, gscale=0x%lx\n",sensor_port,gaingb, gaing, gscale);
    }
    if ((FRAMEPAR_MODIFIED(P_GAINB))  && (bscale_ctl!= CSCALES_CTL_DISABLE)) {
        bscale=mt9f002_getScale16(gainb, gaing);
        dev_dbg(g_dev_ptr,"{%d} gainb=0x%lx, gaing=0x%lx, bscale=0x%lx\n",sensor_port,gainb, gaing, bscale);
    }

    // remove recalc flag
    if (rscale_ctl == CSCALES_CTL_RECALC) rscale_ctl = CSCALES_CTL_NORMAL;
    if (gscale_ctl == CSCALES_CTL_RECALC) gscale_ctl = CSCALES_CTL_NORMAL;
    if (bscale_ctl == CSCALES_CTL_RECALC) bscale_ctl = CSCALES_CTL_NORMAL;
    // update P_*SCALE if either scale or scale control changed
    if (((newval=((rscale_ctl<<CSCALES_CTL_BIT) | (rscale & ((1<<CSCALES_WIDTH)-1)))))!=rscale_all) {
        SETFRAMEPARS_SET(P_RSCALE, newval);
        dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    }
    dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    if (((newval=((gscale_ctl<<CSCALES_CTL_BIT) | (gscale & ((1<<CSCALES_WIDTH)-1)))))!=gscale_all) {
        SETFRAMEPARS_SET(P_GSCALE, newval);
        dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    }
    dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    if (((newval=((bscale_ctl<<CSCALES_CTL_BIT) | (bscale & ((1<<CSCALES_WIDTH)-1)))))!=bscale_all) {
        SETFRAMEPARS_SET(P_BSCALE, newval);
        dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    }

    dev_dbg(g_dev_ptr,"{%d} newval=0x%lx\n",sensor_port,newval);
    dev_dbg(g_dev_ptr,"{%d} gainr=0x%lx, gaing=0x%lx, gaingb=0x%lx, gainb=0x%lx, rscale_all=0x%lx, gscale_all=0x%lx, bscale_all=0x%lx\n",
    		sensor_port,gainr, gaing, gaingb, gainb, rscale_all, gscale_all, bscale_all);


    // Third part: Split overall gains into analog and digital components

    // Split required gain for red into analog register change (with half step hysteresis) and

    digitalGain= gain_ajust_mt9f002(gainr,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_RED<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINR],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);

    //pr_info("{%d} RED thispar=0x%08x newRegGain=0x%08x digitalGain=0x%08x\n",sensor_port,thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINR],newRegGain,digitalGain);

    // apply sensor register gain red if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINR]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_GAINR, newRegGain);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16, (int) P_MT9F002_GAINR, (int) newRegGain);
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the red channel
    if (digitalGain != thispars->pars[P_DGAINR]) {
        SETFRAMEPARS_SET(P_DGAINR, digitalGain);
    }

    // Split required gain for green_red into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9f002(gaing,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_GREEN1<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINGR],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);

    // apply sensor register gain red if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINGR]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_GAINGR, newRegGain);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_GAINGR, (int) newRegGain);
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the red channel
    if (digitalGain != thispars->pars[P_DGAING]) {
        SETFRAMEPARS_SET(P_DGAING, digitalGain);
    }


    // Split required gain for blue into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9f002(gainb,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_BLUE<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINB],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);

    // apply sensor register gain red if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINB]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_GAINB, newRegGain);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16, (int) P_MT9F002_GAINB, (int) newRegGain);
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the red channel
    if (digitalGain != thispars->pars[P_DGAINB]) {
        SETFRAMEPARS_SET(P_DGAINB, digitalGain);
    }


    // Split required gain for blue into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9f002(gaingb,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_GREEN2<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINGB],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);

    // apply sensor register gain red if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9F002_GAINGB]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_GAINGB, newRegGain);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_GAINGB, (int) newRegGain);
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the red channel
    if (digitalGain != thispars->pars[P_DGAINGB]) {
        SETFRAMEPARS_SET(P_DGAINGB, digitalGain);
    }


    // test mode off/on/select
    testmode= thispars->pars[P_TESTSENSOR];
    if (testmode != thispars->pars[P_SENSOR_REGS+P_MT9F002_TEST_PATTERN]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr , P_MT9F002_TEST_PATTERN, testmode);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port,sensor_port,frame16,(int)sensor->i2c_addr,(int)P_MT9F002_TEST_PATTERN,(int)testmode);
    }

    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}







// SysFS interface to mt9f002

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

static ssize_t show_status_flags(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i,j;
	int s[16];

	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			s[4*i+j] = mt9f002_phases_read_flags(i,j);
		}
	}

    return sprintf(buf,"%d %d %d %d    %d %d %d %d    %d %d %d %d    %d %d %d %d\n",
    		            s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12],s[13],s[14],s[15]);
}

static DEVICE_ATTR(sensor_regs0,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs1,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs2,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(sensor_regs3,               SYSFS_PERMISSIONS,     show_sensor_regs,                store_sensor_regs);
static DEVICE_ATTR(debug_delays,               SYSFS_PERMISSIONS,     show_debug_delays,               store_debug_delays);
static DEVICE_ATTR(debug_modes,                SYSFS_PERMISSIONS,     show_debug_modes,                store_debug_modes);
static DEVICE_ATTR(frame_status,               SYSFS_PERMISSIONS,     show_status_flags,               NULL);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_sensor_regs0.attr,
        &dev_attr_sensor_regs1.attr,
        &dev_attr_sensor_regs2.attr,
        &dev_attr_sensor_regs3.attr,
        &dev_attr_debug_delays.attr,
        &dev_attr_debug_modes.attr,
		&dev_attr_frame_status.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_mt9f002_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

int mt9f002_init(struct platform_device *pdev)
{
//    int res;
    struct device *dev = &pdev->dev;
//    const struct of_device_id *match;
    int sensor_port;

    for (sensor_port = 0; sensor_port < SENSOR_PORTS; sensor_port++) {
        first_sensor_sa7[sensor_port] = 0;
    }
    elphel393_mt9f002_sysfs_register(pdev);
    dev_info(dev, DEV393_NAME(DEV393_MT9F002)": registered sysfs\n");
    g_dev_ptr = dev;
    return 0;
}

int mt9f002_remove(struct platform_device *pdev)
{
    unregister_chrdev(DEV393_MAJOR(DEV393_MT9F002), DEV393_NAME(DEV393_MT9F002));
    return 0;
}

// TODO: add elphel,elphel393-mt9f002-1.00 to device tree
static const struct of_device_id elphel393_mt9f002_of_match[] = {
    { .compatible = "elphel,elphel393-mt9f002-1.00" },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_mt9f002_of_match);

static struct platform_driver elphel393_mt9f002 = {
    .probe          = mt9f002_init,
    .remove         = mt9f002_remove,
    .driver         = {
        .name       = DEV393_NAME(DEV393_MT9F002),
        .of_match_table = elphel393_mt9f002_of_match,
    },
};

module_platform_driver(elphel393_mt9f002);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION("Driver for MT9F002 in Elphel cameras");
