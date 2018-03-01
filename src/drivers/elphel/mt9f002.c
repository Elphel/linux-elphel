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
        .clearTop    = 32,       ///< top margin to the first clear pixel
        .clearLeft   = 144,       ///< left margin to the first clear pixel
        //.clearTop    = 106,       ///< top margin to the first clear pixel
        //.clearLeft   = 114,       ///< left margin to the first clear pixel
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
		P_MT9F002_HISPI_TIMING,            0x8000, //
		P_MT9F002_SMIA_PLL_MULTIPLIER,     0x00b4, //
		P_MT9F002_HISPI_CONTROL_STATUS,    0x8400, //
		P_MT9F002_DATAPATH_SELECT,         0x9280, //
		//P_MT9F002_RESET_REGISTER,          0x001c,
		//P_MT9F002_TEST_PATTERN,            0x0002, // color bars
		P_MT9F002_ANALOG_GAIN_CODE_GLOBAL, 0x000a,
		P_MT9F002_ANALOG_GAIN_CODE_RED,    0x000d,
		P_MT9F002_ANALOG_GAIN_CODE_BLUE,   0x0010,
		P_MT9F002_COARSE_INTEGRATION_TIME, 0x0100
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
//int mt9f002_pgm_window       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_window_safe  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_window_common(int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_limitfps     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_exposure     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int mt9f002_pgm_gains        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
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
    //add_sensor_proc(sensor_port,onchange_exposure,    &mt9x001_pgm_exposure);     // program exposure
    //add_sensor_proc(sensor_port,onchange_window,      &mt9x001_pgm_window);       // program sensor WOI and mirroring (flipping)
    //add_sensor_proc(sensor_port,onchange_window_safe, &mt9x001_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    //add_sensor_proc(sensor_port,onchange_limitfps,    &mt9x001_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    //add_sensor_proc(sensor_port,onchange_gains,       &mt9x001_pgm_gains);        // program analog gains
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

int mt9f002_phases_program_phase(int sensor_port, int phase){

	int i;
	int read_phase = 0;
	struct sensor_port_config_t *pcfg;
	const char *name;
	int wr_full;

	pcfg = &pSensorPortConfig[sensor_port];
	name = get_name_by_code(pcfg->sensor[0],DETECT_SENSOR);

	X3X3_I2C_SEND2_ASAP(sensor_port,0x0,P_MT9F002_HISPI_TIMING,phase);

	X3X3_I2C_RCV2(sensor_port, 0x10, P_REG_MT9F002_HISPI_TIMING, &read_phase);
	if (read_phase!=phase){
		pr_err("{%d} i2c write error, target value = 0x%04x, read value = 0x%04x \n",sensor_port,phase,read_phase);
	}

	return 0;
}

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
			//if (i>1) dev_dbg(g_dev_ptr,"{%d} hact recovered after %d\n",sensor_port,i);
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

int mt9f002_phases_scan_lane(int sensor_port, int phase, int shift){

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

int mt9f002_phases_adjust(int sensor_port){

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
		phase = mt9f002_phases_scan_lane(sensor_port,phase,i);
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
		X3X3_I2C_SEND2_ASAP(sensor_port,0x0,mt9f002_inits[2*i],mt9f002_inits[2*i+1]);
	}

	// soft reset
	// sa7 is not used
	X3X3_I2C_SEND2_ASAP(sensor_port,0x0,P_MT9F002_RESET_REGISTER,MT9F002_RESET_REGISTER_VALUE);
	// delay (needed?)
	udelay(100);

	// sensor is supposed to be streaming by now
	mt9f002_phases_adjust(sensor_port);

	// restore SOF (disabled in pgm_detect)
	dis_sof.dis_sof = 0;
	x393_sens_sync_mult(dis_sof,sensor_port);

	// init register shadows here

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
