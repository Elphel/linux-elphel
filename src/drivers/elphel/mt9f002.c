/***************************************************************************//**
 * @file      mt9f002.c
 * @brief     Handles On Semiconductor MT9F002 14MPx sensor
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

// hact timeout in udelay(100)s
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
		P_MT9F002_GAIN,         P_REG_MT9F002_ANALOG_GAIN_CODE_GLOBAL,
		P_MT9F002_GAINGR,       P_REG_MT9F002_ANALOG_GAIN_CODE_GREENR,
		P_MT9F002_GAINR,        P_REG_MT9F002_ANALOG_GAIN_CODE_RED,
		P_MT9F002_GAINB,        P_REG_MT9F002_ANALOG_GAIN_CODE_BLUE,
		P_MT9F002_GAINGB,       P_REG_MT9F002_ANALOG_GAIN_CODE_GREENB,
		P_MT9F002_HISPI_TIMING,             P_REG_MT9F002_HISPI_TIMING,
		P_MT9F002_SMIA_PLL_MULTIPLIER,      P_REG_MT9F002_SMIA_PLL_MULTIPLIER,
		P_MT9F002_HISPI_CONTROL_STATUS,     P_REG_MT9F002_HISPI_CONTROL_STATUS,
		P_MT9F002_DATAPATH_SELECT,          P_REG_MT9F002_DATAPATH_SELECT,
		P_MT9F002_RESET_REGISTER,           P_REG_MT9F002_RESET_REGISTER,
		P_MT9F002_ANALOG_GAIN_CODE_GLOBAL,  P_REG_MT9F002_ANALOG_GAIN_CODE_GLOBAL,
		P_MT9F002_ANALOG_GAIN_CODE_RED,     P_REG_MT9F002_ANALOG_GAIN_CODE_RED,
		P_MT9F002_ANALOG_GAIN_CODE_BLUE,    P_REG_MT9F002_ANALOG_GAIN_CODE_BLUE,
		P_MT9F002_COARSE_INTEGRATION_TIME,  P_REG_MT9F002_COARSE_INTEGRATION_TIME,
		P_MT9F002_FINE_INTEGRATION_TIME,    P_REG_MT9F002_FINE_INTEGRATION_TIME,
		P_MT9F002_Y_ADDR_START,             P_REG_MT9F002_Y_ADDR_START,
		P_MT9F002_Y_ADDR_END,               P_REG_MT9F002_Y_ADDR_END,
		P_MT9F002_X_ADDR_START,             P_REG_MT9F002_X_ADDR_START,
		P_MT9F002_X_ADDR_END,               P_REG_MT9F002_X_ADDR_END,
		P_MT9F002_Y_OUTPUT_SIZE,            P_REG_MT9F002_SMIA_Y_OUTPUT_SIZE,
		P_MT9F002_X_OUTPUT_SIZE,            P_REG_MT9F002_SMIA_X_OUTPUT_SIZE,
		P_MT9F002_LINE_LENGTH_PCK,          P_REG_MT9F002_LINE_LENGTH_PCK,
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
        .imageHeight = 3288,     ///< nominal image height for final images
        .clearWidth  = 4608,     ///< maximal clear image width
        .clearHeight = 3288,     ///< maximal clear image height;
        .clearTop    = 32-MT9F002_VACT_DELAY, ///< top margin to the first clear pixel
        .clearLeft   = 144,      ///< left margin to the first clear pixel
        .arrayWidth  = 4640,     ///< total image array width (including black and boundary)
        .arrayHeight = 3320,     ///< total image array height (including black and boundary)
        .minWidth    = 2,        ///< minimal WOI width
        .minHeight   = 2,        ///< minimal WOI height

        .minHorBlank = 0,        ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 0,      ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 0,     ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 0,        ///< minimal vertical blanking
        .maxVertBlank= 0,     ///< maximal vertical blanking/Virtual frame height (depends on sensor type)

        .maxShutter  = 0xfffff,  ///< Maximal shutter duration (in lines)

        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 0,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 2,        ///< bayer shift for flips==0
        .dcmHor      = 0xff,     ///< 1,2,3,4,5,6,7,8 (doc show [0,6] - change to 0x7f
        .dcmVert     = 0xff,     ///< 1,2,3,4,5,6,7,8
        .binHor      = 0xff,     ///< 1,2,4 0xb{0,1,3}
        .binVert     = 0xff,     ///< 1,2,3,4 0xf [0,3]
        .maxGain256  = 4032,     ///< (15.75) maximal analog gain times 0x100
        .minGain256  = 384,      ///< 1.5 times 0x100

        .minClockFreq= 20000000, ///< Minimal clock frequency
        .maxClockFreq= 24444000, ///< Maximal clock frequency
        .nomClockFreq= 24444000, ///< nominal clock frequency
        .sensorType  = SENSOR_MT9F002,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9F002_I2C_ADDR,           ///< sensor i2c slave address (7 bits)
        .i2c_period  = 2500,     ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,        ///< number of bytes/ register
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
//int mt9f002_pgm_limitfps     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_exposure     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int mt9f002_pgm_gains        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_triggermode  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_sensorregs   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);

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
int mt9f002_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{

    u32  i2c_read_dataw;

    //int sensor_subtype=0;
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

    // temporary solution
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

    // Sensor recognized, go on

    // copy sensor definitions
    memcpy(sensor, psensor, sizeof(mt9f002));
    dev_dbg(g_dev_ptr,"{%d} copied %d bytes of sensor static parameters\n",sensor_port,sizeof(mt9f002));

    add_sensor_proc(sensor_port,onchange_detectsensor,&mt9f002_pgm_detectsensor); // detect sensor type, sets sensor structure (capabilities), function pointers NOTE: will be called directly, not through pointers
    add_sensor_proc(sensor_port,onchange_initsensor,  &mt9f002_pgm_initsensor);   // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    add_sensor_proc(sensor_port,onchange_sensorin,    &mt9f002_pgm_sensorin);     // currently: VACT delay hack
    add_sensor_proc(sensor_port,onchange_exposure,    &mt9f002_pgm_exposure);     // program exposure
    add_sensor_proc(sensor_port,onchange_window,      &mt9f002_pgm_window);       // program sensor WOI and mirroring (flipping)
    add_sensor_proc(sensor_port,onchange_window_safe, &mt9f002_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    //add_sensor_proc(sensor_port,onchange_limitfps,    &mt9x001_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    add_sensor_proc(sensor_port,onchange_gains,       &mt9f002_pgm_gains);        // program analog gains
    //add_sensor_proc(sensor_port,onchange_triggermode, &mt9x001_pgm_triggermode);  // program sensor trigger mode
    //add_sensor_proc(sensor_port,onchange_sensorregs,  &mt9x001_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?

    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType); // should cause other actions

    common_pars->sensors[sensor_port] =  sensor->sensorType;
    //  setFramePar(thispars, P_SENSOR  | FRAMEPAIR_FORCE_NEWPROC,  sensor->sensorType); // force actions
    //  MDD1(dev_dbg(g_dev_ptr,"\n"));


    // CCAM_ARO_ON set. Does it need to be set here? Not earlier (as it is now set for NC393)
    dev_dbg(g_dev_ptr,"{%d}  set ARO (TRIGGER) line HIGH\n",sensor_port);

    sensio_ctl.d32=0;
    sensio_ctl.aro = 1;
    sensio_ctl.aro_set = 1;
    x393_sensio_ctrl(sensio_ctl,sensor_port);

    return sensor->sensorType;
    //NOTE 353:  hardware i2c is turned off (not needed in 393)
}

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

/** Reset and initialize sensor
 * resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c - 353 only)
 * i2c is supposed to be already programmed */
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
	x393_sens_sync_mult_t dis_sof = {.d32=0};
	struct frameparspair_t pars_to_update[262+(MAX_SENSORS * P_MULTI_NUMREGS )]; // for all the sensor registers. Other P_* values will reuse the same ones
	int regaddr,regval,regnum,mreg,j;
	int nupdate=0;
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
		X3X3_I2C_SEND2_LUT_ASAP(sensor_port,0x0,mt9f002_inits[2*i],mt9f002_inits[2*i+1]);
	}

	// soft reset
	// sa7 is not used
	X3X3_I2C_SEND2_LUT_ASAP(sensor_port,0x0,P_REG_MT9F002_RESET_REGISTER,MT9F002_RESET_REGISTER_VALUE);
	// delay (needed?)
	udelay(100);

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
        // update multisensor regs
        if ((mreg=MULTIREG(sensor_port,regnum,0))) {
        	for (j=0;j<MAX_SENSORS; j++) {
                SETFRAMEPARS_SET(mreg+j,regval);
        	}
        }
    }
    for (i=0;i<256;i++) {
        sensor_reg_copy[sensor_port][i] = i2c_read_data_dw[i];
    }

    // in mt9x00x there's setFrameParsStatic-call ?!!! Parameters that never change?
    if (nupdate)  setFrameParsStatic(sensor_port,nupdate,pars_to_update);  // save changes to sensor register shadows for all frames
    //if (nupdate) setFramePars(sensor_port,thispars,nupdate,pars_to_update);  // save changes to sensor register shadows
    // next are global pars?
    //

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
    int i,dv,dh,bv,bh,ww,wh,wl,wt,flip,flipX,flipY,d, v;
    int wws,wwe,whs,whe;
    int compressor_margin; // 0 for JP4, 2 for JPEG
    struct frameparspair_t  pars_to_update[29];
    int nupdate=0;
    int styp = sensor->sensorType & 7;

    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);

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

    compressor_margin = (thispars->pars[P_SENSOR_PIXH] - thispars->pars[P_WOI_WIDTH]) >> 1; // assuming same for H and V

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
            wl = sensor->clearWidth - ww - wl - (2 * COLOR_MARGINS) * dh;
            if(wl < 0) wl = 0;
        }
        if(flipY) {
            wt = sensor->clearHeight - wh - wt - (2 * COLOR_MARGINS) * dv;
            if(wt < 0) wt = 0;
        }
        // apply clearTop/clearLeft
        wt = (wt + sensor->clearTop) & 0xfffe;
        wl = (wl + sensor->clearLeft) & 0xfffe;
        dev_dbg(g_dev_ptr,"{%d}   wl =0x%x, wt=0x%x\n",sensor_port, wl, wt);

        // apply binning restrictions
		d = (bh > 1) ? ((bh > 2) ? 16 : 8) : 4;
		if(flipX)
			i = d * ((wl - 2) / d) + 2;
		else
			i = d * (wl / d);
		if(i < wl)
			i += d;
		wl = i;
    }

    wws = wl;
    wwe = wws + ww - 1;

    whs = wt; // wt already got sensor->clearTop-MT9F002_VACT_DELAY
    whe = whs + wh + MT9F002_VACT_DELAY*(dv+1) - 1;
    // will be different if binning/skipping
    wh = whe - whs + 1;

    // program sensor width
    if (wws != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_ADDR_START]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_ADDR_START, wws);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_X_ADDR_START, (int) wws);
    }
    if (wwe != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_ADDR_END]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_ADDR_END, wwe);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_X_ADDR_END, (int) wwe);
    }
    if (ww != thispars->pars[P_SENSOR_REGS+P_MT9F002_X_OUTPUT_SIZE]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_X_OUTPUT_SIZE, ww);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_X_OUTPUT_SIZE, (int) ww);
    }

    // program sensor height
    if (whs!= thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_ADDR_START]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16 ,P_MT9F002_Y_ADDR_START, whs);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_SENSOR_REGS+P_MT9F002_Y_ADDR_START, (int) whs);
    }
    if (whe!= thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_ADDR_END]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16 ,P_MT9F002_Y_ADDR_END, whe);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_SENSOR_REGS+P_MT9F002_Y_ADDR_END, (int) whe);
    }
    if (wh != thispars->pars[P_SENSOR_REGS+P_MT9F002_Y_OUTPUT_SIZE]) {
        SET_SENSOR_MBPAR_LUT(sensor_port, frame16, P_MT9F002_Y_OUTPUT_SIZE, wh);
        dev_dbg(g_dev_ptr,"{%d}   SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9F002_Y_OUTPUT_SIZE, (int) wh);
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

    int video_exposure = thispars->pars[P_VEXPOS];
    // when P_VEXPOS is used exposure will be overwritten
    int exposure       = thispars->pars[P_EXPOS];
    // this sensor has the following 2 registers
    int coarse_exposure;
    int fine_exposure;

    // vt_pix_clk
    // TODO: Do not forget this - add those div registers?!
    //int sclk = thispars->pars[P_CLK_SENSOR]*MT9F002_PLL_MULTIPLIER_VALUE*0x2/0x6/0x6; // pixel clock in Hz before PLL, but PLL setting is ~240MHz?
    int sclk;// = (thispars->pars[P_CLK_SENSOR]/0x6/0x6)*0x2*MT9F002_PLL_MULTIPLIER_VALUE;

    int vert_blank;

    // NOTE: Is decimation taken care of here ??? FIXME: probably not (here and when deciding to use exposure, not number of lines)!!!
    int row_time_in_pixels=thispars->pars[P_VIRT_WIDTH];
    int pix_period;

    int line_length_pck;

    // wrong frame
    if (frame16 >= PARS_FRAMES) return -1;

    sclk = MT9F002_VT_PIX_CLK;
    pr_info("SCLK = %d (%08x)\n",sclk,sclk);

    pr_info("EXPOS: %d\n",thispars->pars[P_EXPOS]);

    exposure = thispars->pars[P_EXPOS];

    // if yes: use number of lines == use P_VEXPOS for calculations, update P_EXPOS as well
    // else  : use P_EXPOS
    if ((video_exposure>0)&&(FRAMEPAR_MODIFIED(P_VEXPOS)||!(FRAMEPAR_MODIFIED(P_EXPOS)||FRAMEPAR_MODIFIED(P_VIRT_WIDTH)))) {
    	pr_info("using P_VEXPOS\n");
    }else{

    	pr_info("using P_EXPOS\n");
    	// exposure is in microseconds
    	exposure = thispars->pars[P_EXPOS];

    	ull_video_exposure = (long long) (MT9F002_VT_PIX_CLK / 1000000) * (long long) exposure;
    	line_length_pck = thispars->pars[P_SENSOR_REGS+P_MT9F002_LINE_LENGTH_PCK];
    	pr_info("ull_video_exposure = %d (%ull)\n",ull_video_exposure,ull_video_exposure);
    	__div64_32(&ull_video_exposure,line_length_pck);
    	pr_info("ull_video_exposure = %d (%ull)\n",ull_video_exposure,ull_video_exposure);

    	coarse_exposure = ull_video_exposure;
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

    // is video exposure P_VEXPOS modified?
    if (thispars->pars[P_VEXPOS] != fine_exposure) {
        SETFRAMEPARS_SET(P_VEXPOS, fine_exposure);
    }
    // is exposure P_EXPOS modified?
    if (thispars->pars[P_EXPOS] != exposure) {
        SETFRAMEPARS_SET(P_EXPOS, exposure);
    }

    // Now sensor registers
    // schedule updating P_MT9X001_VERTBLANK sensor register and shadow
    */
    // coarse integration time
    if (coarse_exposure != thispars->pars[P_SENSOR_REGS+P_MT9F002_COARSE_INTEGRATION_TIME]) {
        SET_SENSOR_MBPAR_LUT(sensor_port,frame16,P_MT9F002_COARSE_INTEGRATION_TIME, coarse_exposure);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,(int) sensor->i2c_addr,(int)P_MT9F002_COARSE_INTEGRATION_TIME,(int)coarse_exposure);
    }
    /*
    // fine integration time
    if (fine_exposure != thispars->pars[P_SENSOR_REGS+P_MT9F002_FINE_INTEGRATION_TIME]) {
        SET_SENSOR_MBPAR_LUT(sensor_port,frame16,sensor->i2c_addr,P_MT9F002_FINE_INTEGRATION_TIME, fine_exposure);
        dev_dbg(g_dev_ptr,"{%d} SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
        		sensor_port, sensor_port, frame16,(int) sensor->i2c_addr,(int)P_MT9F002_FINE_INTEGRATION_TIME,(int)fine_exposure);
    }
    */

    if (nupdate) setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows

    dev_dbg(g_dev_ptr,"{%d} coarse exposure = %d (0x%x),  fine exposure = %d (0x%x) OK!\n",
    		sensor_port, (int) coarse_exposure, (int) coarse_exposure, (int) fine_exposure, (int) fine_exposure);

    return 0;
}

int mt9f002_calculate_gain(int parvalue){

	typedef union {
	    struct {
	          u32            analog2: 7; // [6:0]   analog gain 2
	          u32            analog3: 3; // [9:7]   analog gain 3
	          u32            colamp : 2; // [11:10] colamp
	          u32            digital: 4; // [15:12] digital
	    };
	    u32 d32; // [31: 0] cast to u32
	} gain;

	gain reg = {.d32=0};

	if (parvalue > 0x200000){
		reg.analog2 = 0;
		reg.analog3 = 1;
		reg.colamp = 3;
		reg.digital = 4;
	}else if (parvalue > 0x100000){
		reg.analog2 = 0;
		reg.analog3 = 1;
		reg.colamp = 3;
		reg.digital = 2;
	}else if (parvalue > 0x60000){
		reg.analog2 = 0;
		reg.analog3 = 1;
		reg.colamp = 3;
		reg.digital = 1;
	}else if (parvalue > 0x30000){
		reg.analog2 = 0;
		reg.analog3 = 1;
		reg.colamp = 2;
		reg.digital = 1;
	}else if (parvalue > 0x18000){
		reg.analog2 = 0;
		reg.analog3 = 1;
		reg.colamp = 1;
		reg.digital = 1;
	}

	reg.analog2 = (((parvalue<<6)/reg.digital)>>(reg.colamp+reg.analog3))&0x7f;

	return reg.d32;
}
/**
 * Program analog gains
 * program analog gains TODO: Make separate read-only P_ACTUAL_GAIN** ?
 * apply sensor-specific restrictions on the allowed gain values
 * includes sensor test mode on/off/selection
 */
int mt9f002_pgm_gains      (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error

{
	int reg;

    struct frameparspair_t pars_to_update[38]; // 22+12 needed
    int nupdate=0;

    unsigned long newRegGain,digitalGain, testmode;
    unsigned long anaGainEn= (thispars->pars[P_GAIN_CTRL]>> GAIN_BIT_ENABLE) & 1;
    unsigned long minAnaGain=thispars->pars[P_GAIN_MIN];
    unsigned long maxAnaGain=thispars->pars[P_GAIN_MAX];
    unsigned long maxGain;

    int limitsModified=0;
    //int gaingModified=FRAMEPAR_MODIFIED(P_GAING);

    unsigned long gainr, gaing, gaingb, gainb;
    //unsigned long rscale_all, gscale_all, bscale_all;
    //unsigned long rscale, gscale, bscale, rscale_ctl, gscale_ctl, bscale_ctl;
    //unsigned long newval;


    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame

    //make sure limits are OK. Allow violating minimal gain here

    gainr= thispars->pars[P_GAINR];
    gaing= thispars->pars[P_GAING];
    gaingb=thispars->pars[P_GAINGB];
    gainb= thispars->pars[P_GAINB];

    // scales will not be modified if they make gains out of limit, but gains will be. So automatic white balance should deal with gains, not with scales.
    // Preserving application-set values for scales simplifies recovery when the green gain is adjusted so all colors fit in the limits


    // Second part - combine P_*SCALE and P_GAIN* parameters

    // Update scales only if the corresponding color gains (not the base green one) were modified outside of the driver
    // (not as a result of being limited by minimal/maximal gains)
    if (FRAMEPAR_MODIFIED(P_GAING)) {
    	reg = mt9f002_calculate_gain(gaing);
    }

    if (FRAMEPAR_MODIFIED(P_GAINR)) {
    	reg = mt9f002_calculate_gain(gainr);
    }

    if (FRAMEPAR_MODIFIED(P_GAINGB)) {
    	reg = mt9f002_calculate_gain(gaingb);
    }

    if (FRAMEPAR_MODIFIED(P_GAINB)) {
    	reg = mt9f002_calculate_gain(gainb);
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
