/***************************************************************************//**
* @file      pgm_functions.c
* @brief     Sensor/FPGA programming functions, called from  IRQ/tasklet in
*            response to the parameter changes
* @copyright Copyright 2008-2016 (C) Elphel, Inc.
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
/*  $Log: pgm_functions.c,v $
*!  Revision 1.24  2012/03/14 00:05:41  elphel
*!  8.2.1 - GPS speed, x353.bit file tested on Eyesis
*!
*!  Revision 1.23  2012/01/16 01:48:00  elphel
*!  increased initial sensor clock frequency (20MHz->40MHz)
*!
*!  Revision 1.22  2011/12/22 05:40:05  elphel
*!  bug fix
*!
*!  Revision 1.21  2010/08/10 21:10:41  elphel
*!  portrait mode now includes all 4 rotations (2 bits)
*!
*!  Revision 1.20  2010/08/03 23:37:34  elphel
*!  rev 8.0.8.37, portrait mode support
*!
*!  Revision 1.19  2010/08/01 19:29:24  elphel
*!  files updated to support coring function for noise filtering
*!
*!  Revision 1.18  2010/06/06 04:24:10  elphel
*!  updated debug message
*!
*!  Revision 1.17  2010/06/05 07:11:48  elphel
*!  modified startup options
*!
*!  Revision 1.16  2010/06/02 16:31:04  elphel
*!  Added P_MULTI_SELECTED parameter
*!
*!  Revision 1.15  2010/06/01 08:30:36  elphel
*!  support for the FPGA code 03534022  with optional master time stamp over the inter-camera sync line(s)
*!
*!  Revision 1.14  2010/05/25 00:52:23  elphel
*!  8.0.8.20, working on multi-sensor
*!
*!  Revision 1.13  2010/05/21 06:12:16  elphel
*!  continue working on multi-sensor software
*!
*!  Revision 1.12  2010/05/13 03:39:31  elphel
*!  8.0.8.12 -  drivers modified for multi-sensor operation
*!
*!  Revision 1.11  2010/04/30 23:38:49  elphel
*!  fixed WOI_TOP in linescan mode
*!
*!  Revision 1.10  2010/04/30 00:29:54  elphel
*!  bug fixes related to linescan mode
*!
*!  Revision 1.9  2010/04/28 02:34:33  elphel
*!  8.0.6.6 - added support for linescan mode (also useful for high fps small images). 2.5K full line pairs/second with 5MPix sensor
*!
*!  Revision 1.8  2010/04/24 21:40:15  elphel
*!  8.0.8.4 (FPGA 03534016), most changes related to sensor signals phase adjustments
*!
*!  Revision 1.7  2010/01/27 22:51:52  elphel
*!  turned off ELPHEL_DEBUG, fixed errors caused by that.
*!
*!  Revision 1.6  2009/03/16 02:07:41  elphel
*!  Bug fix when calculating total width for focus processing - FPGA requires data 16 less than the actual WOI width
*!
*!  Revision 1.5  2008/12/08 21:55:02  elphel
*!  making 3MPix work, minor cleanup
*!
*!  Revision 1.4  2008/12/02 19:11:08  elphel
*!  bug fix in sensor detection, removed unneeded initialization for P_GAIN*
*!
*!  Revision 1.3  2008/12/02 00:29:04  elphel
*!  Added FIXME for the known problem
*!
*!  Revision 1.2  2008/11/30 05:01:03  elphel
*!  Changing gains/scales behavior
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
*!
*!
*!  Revision 1.32  2008/11/27 09:27:31  elphel
*!  Support fro new parameters (vignetting correction related)
*!
*!  Revision 1.31  2008/11/14 01:01:40  elphel
*!  fixed new bug in pgm_gammaload
*!
*!  Revision 1.30  2008/11/13 05:40:45  elphel
*!  8.0.alpha16 - modified histogram storage, profiling
*!
*!  Revision 1.29  2008/11/03 18:43:18  elphel
*!  8.0.alpha12 with working apps/astreamer
*!
*!  Revision 1.28  2008/10/31 18:26:32  elphel
*!  Adding support for constants like SENSOR_REGS32 (defined constant plus 32 to simplify referencing sensor registers from PHP
*!
*!  Revision 1.27  2008/10/29 04:18:28  elphel
*!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*!
*!  Revision 1.26  2008/10/23 08:08:05  elphel
*!  corrected histogram window size setup (data written to FPGA is actual window size -2 for both width and height)
*!
*!  Revision 1.25  2008/10/22 05:29:03  elphel
*!  Rev. 8.0.alpha5 - working on external trigger mode - moved programming away from the sequencer that can only pass 24 data bits
*!
*!  Revision 1.24  2008/10/21 04:03:39  elphel
*!  bug fix (missing break in switch)
*!
*!  Revision 1.23  2008/10/20 18:45:07  elphel
*!  just more debug printk
*!
*!  Revision 1.22  2008/10/18 06:14:21  elphel
*!  8.0.alpha4 - removed some obsolete parameters, renumbered others, split P_FLIP into P_FLIPH and P_FLIPV (different latencies because of bad frames), pgm_window-> pgm_window, pgm_window_safe
*!
*!  Revision 1.21  2008/10/17 05:44:48  elphel
*!  fixing latencies
*!
*!  Revision 1.20  2008/10/12 06:13:10  elphel
*!  snapshot
*!
*!  Revision 1.19  2008/10/11 18:46:07  elphel
*!  snapshot
*!
*!  Revision 1.18  2008/10/08 21:26:25  elphel
*!  snapsot 7.2.0.pre4 - first images (actually - second)
*!
*!  Revision 1.17  2008/10/06 08:33:03  elphel
*!  snapshot, first images
*!
*!  Revision 1.15  2008/10/05 05:13:33  elphel
*!  snapshot003
*!
*!  Revision 1.14  2008/10/04 16:10:12  elphel
*!  snapshot
*!
*!  Revision 1.13  2008/09/28 00:31:57  elphel
*!  snapshot
*!
*!  Revision 1.12  2008/09/25 00:58:12  elphel
*!  snapshot
*!
*!  Revision 1.11  2008/09/22 22:55:49  elphel
*!  snapshot
*!
*!  Revision 1.10  2008/09/19 04:37:26  elphel
*!  snapshot
*!
*!  Revision 1.9  2008/09/16 00:49:32  elphel
*!  snapshot
*!
*!  Revision 1.8  2008/09/12 20:40:13  elphel
*!  snapshot
*!
*!  Revision 1.7  2008/09/12 00:23:59  elphel
*!  removed cc353.c, cc353.h
*!
*!  Revision 1.6  2008/09/05 23:21:32  elphel
*!  snapshot
*!
*!  Revision 1.4  2008/09/04 17:37:13  elphel
*!  documenting
*!
*!  Revision 1.3  2008/07/29 01:15:06  elphel
*!  another snapshot
*!
*!  Revision 1.2  2008/07/27 23:25:07  elphel
*!  next snapshot
*!
*!  Revision 1.1  2008/07/27 04:27:49  elphel
*!  next snapshot
*/

// TODO:remove unneeded
//#define DEBUG // should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)
#include <linux/types.h> // for div 64
#include <asm/div64.h>   // for div 64

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
//#include <linux/platform_device.h>
#include <linux/device.h> // for dev_dbg, platform_device.h is OK too

#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/c313a.h>
#include "framepars.h"
#include "sensor_common.h"
#include "multi10359.h"
#include "mt9x001.h"
#include "gamma_tables.h"
#include "quantization_tables.h"
#include "latency.h"
#include "pgm_functions.h"
#include "jpeghead.h"      // to program FPGA Huffman tables
#include "x393.h"
#include "sensor_i2c.h"
#include "x393_videomem.h"
#include "detect_sensors.h"
#include "x393_fpga_functions.h"

// NC393 debug macros
#include "debug393.h"



/**
 * @brief optional debug output macros (obsolete)
 */
#if ELPHEL_DEBUG
// #define MDF2(x) { if (GLOBALPARS(G_DEBUG) & (1 <<2)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
 #define MDF3(x) { if (GLOBALPARS(G_DEBUG) & (1 <<3)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
 #define MDF9(x) { if (GLOBALPARS(G_DEBUG) & (1 <<9)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
 #define MDF16(x) { if (GLOBALPARS(G_DEBUG) & (1 <<16)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
 #define MDF23(x) { if (GLOBALPARS(G_DEBUG) & (1 <<23)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
 #define ELPHEL_DEBUG_THIS 0
// #define ELPHEL_DEBUG_THIS 1
#else
// #define MDF2(x)
 #define MDF3(x)
 #define MDF9(x)
 #define MDF16(x)
 #define MDF23(x)
 #define ELPHEL_DEBUG_THIS 0
#endif

#if ELPHEL_DEBUG_THIS
  #define MDF1(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x
  #define MD1(x)  printk("%s:%d: ",__FILE__,__LINE__ );x
#else
  #define MDF1(x)
  #define MD1(x)
#endif

static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
void pgm_functions_set_device(struct device *dev)
{
    g_dev_ptr = dev;
    mt9x001_set_device(dev);
    multi10359_set_device(dev);
}
//dev_info(g_dev_ptr,

  int pgm_recalcseq     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_detectsensor  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_sensorphase   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_i2c           (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_initsensor    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_afterinit     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_multisens     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_window        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_window_safe   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_window_common (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_exposure      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_gains         (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_triggermode   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_sensorin      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_sensorstop    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_sensorrun     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_gamma         (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_hist          (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_aexp          (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_quality       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_memsensor     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_memcompressor (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_limitfps      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_compmode      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_focusmode     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_trigseq       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_irq           (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_comprestart   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_compstop      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_compctl       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_gammaload     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_sensorregs    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
  int pgm_prescal       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);

/** Initialize array of functions that program different acquisition parameters (some are sensor dependent)
 * @return always 0 */
int init_pgm_proc(int sensor_port)
{
    int i;
    struct sensorproc_t * sensorproc = &asensorproc[sensor_port];
    dev_dbg(g_dev_ptr,"Initializing generic parameter-triggered function, port=%d\n",sensor_port); // OK NULL device here (not yet set)
    MDP(DBGB_PSFN,sensor_port,"Initializing generic parameter-triggered function, %s\n","") // OK NULL device here (not yet set)

    for (i=0;i<64;i++) sensorproc->pgm_func[i]=NULL;
    sensorproc->pgm_func[onchange_recalcseq]=    &pgm_recalcseq;     // recalculate sequences/latencies, according to P_SKIP, P_TRIG
    sensorproc->pgm_func[onchange_detectsensor]= &pgm_detectsensor;  // detect sensor type, sets sensor structure (capabilities), function pointers
    sensorproc->pgm_func[onchange_sensorphase]=  &pgm_sensorphase;   // program sensor clock/phase
    sensorproc->pgm_func[onchange_i2c]=          &pgm_i2c;           // program i2c
    sensorproc->pgm_func[onchange_initsensor]=   &pgm_initsensor;    // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    sensorproc->pgm_func[onchange_afterinit]=    &pgm_afterinit;     // restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
    sensorproc->pgm_func[onchange_multisens]=    &pgm_multisens;     // changes related to multiplexed sensors
    sensorproc->pgm_func[onchange_window]=       &pgm_window;        // program sensor WOI and mirroring (flipping)
    sensorproc->pgm_func[onchange_window_safe]=  &pgm_window_safe;   // program sensor WOI and mirroring (flipping) - lower latency, no bad frames
    sensorproc->pgm_func[onchange_exposure]=     &pgm_exposure;      // program exposure
    sensorproc->pgm_func[onchange_gains]=        &pgm_gains;         // program analog gains
    sensorproc->pgm_func[onchange_triggermode]=  &pgm_triggermode;   // program sensor trigger mode
    sensorproc->pgm_func[onchange_sensorin]=     &pgm_sensorin;      // program sensor input in FPGA (Bayer, 8/16 bits, ??)
    sensorproc->pgm_func[onchange_sensorstop]=   &pgm_sensorstop;    // Stop acquisition from the sensor to the FPGA (start has latency of 2)
    sensorproc->pgm_func[onchange_sensorrun]=    &pgm_sensorrun;     // Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
    sensorproc->pgm_func[onchange_gamma]=        &pgm_gamma;         // program gamma table
    sensorproc->pgm_func[onchange_hist]=         &pgm_hist;          // program histogram window
    sensorproc->pgm_func[onchange_aexp]=         &pgm_aexp;          // program autoexposure mode
    sensorproc->pgm_func[onchange_quality]=      &pgm_quality;       // program quantization table(s)
    sensorproc->pgm_func[onchange_memsensor]=    &pgm_memsensor;     // program memory channels 0 (sensor->memory) and 1 (memory->FPN)
    sensorproc->pgm_func[onchange_memcompressor]=&pgm_memcompressor; // program memory channel 2 (memory->compressor)
    sensorproc->pgm_func[onchange_limitfps]=     &pgm_limitfps;      // check compressor will keep up, limit sensor FPS if needed
    sensorproc->pgm_func[onchange_compmode]=     &pgm_compmode;      // program compressor modes
    sensorproc->pgm_func[onchange_focusmode]=    &pgm_focusmode;     // program focus modes
    sensorproc->pgm_func[onchange_trigseq]=      &pgm_trigseq;       // program sequencer (int/ext)
    sensorproc->pgm_func[onchange_irq]=          &pgm_irq;           // program smart IRQ mode
    sensorproc->pgm_func[onchange_comprestart]=  &pgm_comprestart;   // restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
    sensorproc->pgm_func[onchange_compstop]=     &pgm_compstop;      // stop compressor when changing geometry
    sensorproc->pgm_func[onchange_compctl]=      &pgm_compctl;       // only start/stop/single (after explicitly changed, not when geometry was changed)
    sensorproc->pgm_func[onchange_gammaload]=    &pgm_gammaload;     // write gamma tables (should be prepared). Maybe - just last byte, to activate?
    sensorproc->pgm_func[onchange_sensorregs]=   &pgm_sensorregs;    // write sensor registers (only changed from outside the driver as they may have different latencies)?
    sensorproc->pgm_func[onchange_prescal]=      &pgm_prescal;       // change scales for per-color digital gains, apply vignetting correction

    return 0;
}
/** Add sensor-specific processing function (executes after the non-specific function with the same index) */
int add_sensor_proc(int port,   ///< sensor port
                    int index,  ///< index function index (internally 32 is added to distinguish from the common (not sensor-specific) functions
                    int (*sens_func)(int sensor_port, struct sensor_t * ,  struct framepars_t * , struct framepars_t *, int ))
                                ///< sens_func pointer to a sensor-specific function
                                ///< @return always 0
{
    dev_dbg(g_dev_ptr,"add_sensor_proc(%d, 0x%x,...)\n", port, index);
    MDP(DBGB_PSFN, port,"add_sensor_proc(%d, 0x%x,...)\n", port, index)
    asensorproc[port].pgm_func[32+(index & 0x1f)]=       sens_func;
    return 0;
}

/** Detect and initialize sensor and related data structures
 * - detect sensor type
 * - set sensor structure (capabilities),
 * - set function pointers (by directly calling sensor specific \b *_pgm_detectsensor */
int pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
///< be applied to,  negative - ASAP
///< @return OK - 0, <0 - error
{
    x393_camsync_mode_t camsync_mode = {.d32=0};
    int was_sensor_freq = 0; // 90000000; // getClockFreq(1);
    int qperiod;
    int i2cbytes;
    int phase;
    int mux,sens;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)

    if (frame16 >= 0) return -1; // can only work in ASAP mode
    if (thispars->pars[P_SENSOR]) {
        dev_dbg(g_dev_ptr,"{%d}  frame16=%d, SENSOR ALREADY DETECTED = %d\n",sensor_port,frame16, (int) thispars->pars[P_SENSOR]);
        return 0; // Sensor is already detected - do not bother (to re-detect it P_SENSOR should be set to 0)
    }
    // no other initializations, just the sensor-related stuff (starting with lowest sensor clock)    // stop hardware i2c controller, so it will not get stuck when waiting for !busy



    #ifndef NC353
    // NC393 - nothing to do here - use a separate module for sensor setup: DT, sysfs, something else (add pin pullup/down)
    // currently assign same sensor to all subchannles (first present)
//    thispars->pars[P_SENSOR] = get_detected_sensor_code(sensor_port, -1); // "-1" - first detected sensor
    sens=get_detected_sensor_code(sensor_port, -1); // "-1" - first detected sensor
//    setFramePar(sensor_port, thispars, P_SENSOR,  sens); //  FIXME: NC393
    GLOBALPARS(sensor_port, G_SUBCHANNELS) = get_subchannels(sensor_port);
    mux = get_detected_mux_code(sensor_port); // none/detect/10359
    dev_dbg(g_dev_ptr,"port = %d, mux = %d, sens= %d\n",sensor_port, mux, sens);
//    if ((mux == SENSOR_NONE) && (thispars->pars[P_SENSOR] == SENSOR_NONE))
    if ((mux == SENSOR_NONE) && (sens == SENSOR_NONE))
        return 0; // no sensor/mux enabled on this port
    //TODO NC393: turn on both sequencers why MRST is active, then i2c frame will definitely match ? Or is it already done in FPGA?
    dev_dbg(g_dev_ptr,"Restarting both command and i2c sequencers for port %d\n",sensor_port);
    sequencer_stop_run_reset(sensor_port, SEQ_CMD_RESET);
    sequencer_stop_run_reset(sensor_port, SEQ_CMD_RUN);  // also programs status update
    i2c_stop_run_reset      (sensor_port, I2C_CMD_RESET);
    dev_dbg(g_dev_ptr,"Setting i2c drive mode for port %d\n",sensor_port);
    i2c_drive_mode          (sensor_port, SDA_DRIVE_HIGH, SDA_RELEASE);
    i2c_stop_run_reset      (sensor_port, I2C_CMD_RUN); // also programs status update
    legacy_i2c(1<<sensor_port);// Setup i2c pages for legacy i2c commands. TODO NC393: update for compatibility with 14MPix
    camsync_mode.trig =     0;
    camsync_mode.trig_set = 1;
    camsync_mode.ext =      1; // use external timestamp (default)
    camsync_mode.ext_set =  1;
    x393_camsync_mode (camsync_mode);

    //     dev_dbg(g_dev_ptr,"trying MT9P001\n");
    //      mt9x001_pgm_detectsensor(sensor_port, sensor,  thispars, prevpars, frame16);  // try Micron 5.0 Mpixel - should return sensor type

    if (mux != SENSOR_NONE) {
        dev_dbg(g_dev_ptr,"Mux mode for port %d is %d, tryng 10359\n",sensor_port, mux);
        MDP(DBGB_PADD, sensor_port,"Mux mode for port %d is %d, tryng 10359\n",sensor_port, mux)

        // try multisensor here (before removing MRST)
        multisensor_pgm_detectsensor (sensor_port, sensor,  thispars, prevpars, frame16);  // multisensor
    } else {
        dev_dbg(g_dev_ptr,"Mux mode for port %d SENSOR_NONE, skipping 10359 detection\n",sensor_port);
        MDP(DBGB_PADD, sensor_port,"Mux mode for port %d SENSOR_NONE, skipping 10359 detection\n",sensor_port)
    }
//    if ((thispars->pars[P_SENSOR]==0) ||  // multisensor not detected
//            ((thispars->pars[P_SENSOR] & SENSOR_MASK) == SENSOR_MT9X001)) { // or is (from DT) SENSOR_MT9X001
    if ((thispars->pars[P_SENSOR]==0) &&                    // multisensor not detected
            (((sens & SENSOR_MASK) == SENSOR_MT9X001) ||    // and from DT it is some SENSOR_MT9*001
             ((sens & SENSOR_MASK) == SENSOR_MT9P006) )) {  // or SENSOR_MT9P006 or friends
//        dev_dbg(g_dev_ptr,"removing MRST from the sensor\n");
//        dev_info(g_dev_ptr,"pgm_detectsensor(%d): TRYING MT9P001\n",sensor_port);
        dev_dbg(g_dev_ptr,"trying MT9P001, port=%d\n",sensor_port);
        MDP(DBGB_PADD, sensor_port,"trying MT9P001, port=%d\n",sensor_port)
        mt9x001_pgm_detectsensor(sensor_port, sensor,  thispars, prevpars, frame16);  // try Micron 5.0 Mpixel - should return sensor type
    }
    setFramePar(sensor_port, thispars, P_CLK_FPGA,  200000000); //  FIXME: NC393
    setFramePar(sensor_port, thispars, P_CLK_SENSOR,  48000000);
    if (thispars->pars[P_SENSOR] == SENSOR_DETECT) {
        sensor->sensorType=SENSOR_NONE;                 // to prevent from initializing again
        dev_dbg(g_dev_ptr,"No image sensor found\n");
        MDP(DBGB_PADD, sensor_port,"No image sensor found%s\n","")
    }
    setFramePar(sensor_port, thispars, P_SENSOR_WIDTH,   sensor->imageWidth);  // Maybe get rid of duplicates?
    setFramePar(sensor_port, thispars, P_SENSOR_HEIGHT,  sensor->imageHeight); // Maybe get rid of duplicates?
    if (sensor->i2c_period==0) sensor->i2c_period=2500;           // SCL period in ns, (standard i2c - 2500)
    qperiod=thispars->pars[P_I2C_QPERIOD];
    if (qperiod==0) qperiod=(sensor->i2c_period * (thispars->pars[P_CLK_FPGA]/1000))/4000000;
    setFramePar(sensor_port, thispars, P_I2C_QPERIOD | FRAMEPAIR_FORCE_NEWPROC,  qperiod); // force i2c
    i2cbytes=thispars->pars[P_I2C_BYTES];
    if (i2cbytes==0) i2cbytes=sensor->i2c_bytes;
    setFramePar(sensor_port, thispars, P_I2C_BYTES | FRAMEPAIR_FORCE_NEWPROC,  i2cbytes); // force i2c
    // restore/set sensor clock
    if ((was_sensor_freq < sensor->minClockFreq) || (was_sensor_freq > sensor->maxClockFreq)) was_sensor_freq=sensor->nomClockFreq;
    setFramePar(sensor_port, thispars, P_CLK_SENSOR | FRAMEPAIR_FORCE_NEWPROC,  was_sensor_freq); // will schedule clock/phase adjustment
    phase=thispars->pars[P_SENSOR_PHASE];
    // TODO: remove phase adjustment from here
    if (phase==0) {
        phase= 0x40000;
        setFramePar(sensor_port, thispars, P_SENSOR_PHASE | FRAMEPAIR_FORCE_NEWPROC,  phase); // will schedule clock/phase adjustment
    }
    setFramePar(sensor_port, thispars, P_IRQ_SMART | FRAMEPAIR_FORCE_NEWPROC,  3);        // smart IRQ mode programming (and enable interrupts)

    // NOTE: sensor detected - enabling camera interrupts here (actual interrupts will start later)
    // Here interrupts are disabled - with compressor_interrupts (0) earlier in this function)

    compressor_interrupts (1,sensor_port); // FIXME: Should they already be set beforfe detection? If not - remove from framepars.php
    sensor_interrupts     (1,sensor_port);
    return 0;
#else
    // NOTE: disabling interrupts here !!!
    compressor_interrupts (0);
    dev_dbg(g_dev_ptr,"%s Disabled camera interrupts\n",__func__);
    // This 3 initialization commands were not here, trying to temporarily fix problem when WP was 8/16 words higher than actual data in DMA buffer
    if (GLOBALPARS(sensor_port, G_TEST_CTL_BITS) & (1<< G_TEST_CTL_BITS_RESET_DMA_COMPRESSOR )) {
        dev_dbg(g_dev_ptr,"%s x313_dma_stop()\n",__func__);
        ///    x313_dma_stop();
        dev_dbg(g_dev_ptr,"%s x313_dma_init()\n",__func__);
        ///    x313_dma_init();
        dev_dbg(g_dev_ptr,"%s reset_compressor()\n",__func__);
        reset_compressor(sensor_port);
    }
    // TODO: Add 10347 detection here //   if (IS_KAI11000) return init_KAI11000();
    // Need to set slow clock
    //  f1=imageParamsR[P_CLK_SENSOR]=20000000; setClockFreq(1, imageParamsR[P_CLK_SENSOR]); X3X3_RSTSENSDCM;
    was_sensor_freq=getClockFreq(1); // using clock driver data, not thispars
    setFramePar(sensor_port, thispars, P_CLK_FPGA,  getClockFreq(0)); // just in case - read the actual fpga clock frequency and store it (no actions)
    setFramePar(sensor_port, thispars, P_CLK_SENSOR,  48000000);
    setClockFreq(1, thispars->pars[P_CLK_SENSOR]);
    dev_dbg(g_dev_ptr,"\nsensor clock set to %d\n",(int) thispars->pars[P_CLK_SENSOR]);
    udelay (100);// 0.0001 sec to stabilize clocks
    X3X3_RSTSENSDCM; // FPGA DCM can fail after clock change, needs to be reset
    X3X3_SENSDCM_CLK2X_RESET; // reset pclk2x DCM also
    mdelay (50);// 0.05 sec to stabilize clocks
    // setting reasonable state of the control signals
    CCAM_DCLK_ON;
    CCAM_CNVEN_OFF;
    CCAM_NEGRST;  ///set negative MRST polarity
    CCAM_MRST_ON;
    CCAM_TRIG_INT;
    CCAM_EXTERNALTS_EN; // Maybe use default as enabled - yes, it will not be active if not available
    udelay (100); // apply clock before removing MRS
    // first trying MT9P001 that does not need converter
    // try multisensor here (before removing MRST)
    if (thispars->pars[P_SENSOR]==0) multisensor_pgm_detectsensor (sensor_port, sensor,  thispars, prevpars, frame16);  // multisensor
    if (thispars->pars[P_SENSOR]==0) {
        dev_dbg(g_dev_ptr,"removing MRST from the sensor\n");
        CCAM_MRST_OFF;
    }
    if (thispars->pars[P_SENSOR]==0) {
        mt9x001_pgm_detectsensor(sensor_port, sensor,  thispars, prevpars, frame16);  // try Micron 5.0 Mpixel - should return sensor type
        dev_dbg(g_dev_ptr,"trying MT9P001\n");
    }
    // temporary - disabling old sensors
#define ENABLE_OLD_SENSORS 1
#ifdef ENABLE_OLD_SENSORS
    if (thispars->pars[P_SENSOR]==0)  { // no - it is not MT9P001)
        dev_dbg(g_dev_ptr,"trying other MT9X001\n");
        CCAM_CNVEN_ON;
        //     CCAM_ARST_ON; ///MT9P001 expects ARST==0 (STANDBY) - done inside mt9x001.c
        // enable output for power converter signals
        // This should be done first!!!
        // printk ("Will Turn DC power for the sensor after 1 sec delay...\n");  udelay (1000000);
        // turning on DC-DC converter may cause system to reboot because of a power spike, so start slow
#ifdef NC353
        port_csp0_addr[X313_WA_DCDC] = 0x44; // 48 - enough, 41 - ok - was 0x61; //
        printk ("sensor power set low\n ");
        mdelay (10); // Wait voltage to come up (~10 ms)
        printk ("will set to 0x41\n");
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x41; // 
        printk ("will set to 0x30\n");
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x30; //
        printk ("will set to 0x28\n");
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x28; //
        printk ("will set to 0x24\n");
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x24; //
        printk ("will set to 0x22\n");
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x22; //
        mdelay (10); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x10; // now - full frequency (same as 0x21). Slow that down if the sensor clock is above 20MHz (i.e.0x22 for 40MHz)
        printk (".. full\n");
        mdelay (10); // Wait voltage to stabilize
        CCAM_POSRST;  //set positive MRST polarity (default)
        udelay (100); // apply clock before removing MRST
        CCAM_MRST_OFF;
#endif
    }
#ifdef CONFIG_ETRAX_ELPHEL_KAC1310
    if (thispars->pars[P_SENSOR]==0) kac1310_pgm_detectsensor(sensor,  thispars, prevpars, frame16);  // try KAC-1310 (does not exist anymore)
#endif
#ifdef CONFIG_ETRAX_ELPHEL_IBIS51300
    if (thispars->pars[P_SENSOR]==0) ibis1300_pgm_detectsensor(sensor,  thispars, prevpars, frame16); // try IBIS5-1300 (software dead)
#endif
    // invert MRST for other sensors
    if (thispars->pars[P_SENSOR]==0) {
        CCAM_NEGRST;  //set negative MRST polarity
        printk ("Inverted MRST\n");
        udelay (100);
    }
#ifdef CONFIG_ETRAX_ELPHEL_MT9X001
    if (thispars->pars[P_SENSOR]==0)  dev_dbg(g_dev_ptr,"trying MT9X001\n");
    if (thispars->pars[P_SENSOR]==0) mt9x001_pgm_detectsensor(sensor_port, sensor,  thispars, prevpars, frame16);  // try Micron 1.3/2.0/3.0 Mpixel
#endif
#ifdef CONFIG_ETRAX_ELPHEL_KAC1310
    if (thispars->pars[P_SENSOR]==0) kac5000_pgm_detectsensor(sensorsensor_port, sensor,  thispars, prevpars, frame16);  // try KAC-5000
#endif
#ifdef CONFIG_ETRAX_ELPHEL_ZR32112
    if (thispars->pars[P_SENSOR]==0) zr32112_pgm_detectsensor(sensorsensor_port, sensor,  thispars, prevpars, frame16); // try ZR32112
#endif
#ifdef CONFIG_ETRAX_ELPHEL_ZR32212
    if (thispars->pars[P_SENSOR]==0) zr32212_pgm_detectsensor(sensorsensor_port, sensor,  thispars, prevpars, frame16); // try ZR32212
#endif
#endif // ENABLE_OLD_SENSORS *************** temporary disabling other sensors ********************
    if (thispars->pars[P_SENSOR]==0) {
        sensor->sensorType=SENSOR_NONE;                 // to prevent from initializing again
        dev_dbg(g_dev_ptr,"No image sensor found\n");
    }
    setFramePar(sensor_port, thispars, P_SENSOR_WIDTH,   sensor->imageWidth);  // Maybe get rid of duplicates?
    setFramePar(sensor_port, thispars, P_SENSOR_HEIGHT,  sensor->imageHeight); // Maybe get rid of duplicates?
    if (sensor->i2c_period==0) sensor->i2c_period=2500;           // SCL period in ns, (standard i2c - 2500)
    qperiod=thispars->pars[P_I2C_QPERIOD];
    if (qperiod==0) qperiod=(sensor->i2c_period * (thispars->pars[P_CLK_FPGA]/1000))/4000000;
    setFramePar(sensor_port, thispars, P_I2C_QPERIOD | FRAMEPAIR_FORCE_NEWPROC,  qperiod); // force i2c
    i2cbytes=thispars->pars[P_I2C_BYTES];
    if (i2cbytes==0) i2cbytes=sensor->i2c_bytes;
    setFramePar(sensor_port, thispars, P_I2C_BYTES | FRAMEPAIR_FORCE_NEWPROC,  i2cbytes); // force i2c
    // restore/set sensor clock
    if ((was_sensor_freq < sensor->minClockFreq) || (was_sensor_freq > sensor->maxClockFreq)) was_sensor_freq=sensor->nomClockFreq;
    setFramePar(sensor_port, thispars, P_CLK_SENSOR | FRAMEPAIR_FORCE_NEWPROC,  was_sensor_freq); // will schedule clock/phase adjustment
    phase=thispars->pars[P_SENSOR_PHASE];
    // TODO: remove phase adjustment from here
    if (phase==0) {
        phase= 0x40000;
        setFramePar(sensor_port, thispars, P_SENSOR_PHASE | FRAMEPAIR_FORCE_NEWPROC,  phase); // will schedule clock/phase adjustment
    }
    setFramePar(sensor_port, thispars, P_IRQ_SMART | FRAMEPAIR_FORCE_NEWPROC,  3);        // smart IRQ mode programming (and enable interrupts)
    // NOTE: sensor detected - enabling camera interrupts here (actual interrupts will start later)
    // Here interrupts are disabled - with compressor_interrupts (0) earlier in this function)
    compressor_interrupts (1);
    return 0;
#endif
}
/** Reset and initialize sensor (all is done in sensor-specific functions)
 *  - resets sensor,
 *  - reads sensor registers,
 *  - schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
 *  TODO: Make sure sequencer and hardware i2c are running when needed. */
int pgm_initsensor     (int sensor_port,               ///< sensor port number (0..3)
		struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
		struct framepars_t * thispars, ///< sensor current parameters
		struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
		int frame16)                   ///< 4-bit (hardware) frame number parameters should
									   ///< be applied to,  negative - ASAP
									   ///< @return always 0
{
   // NC393: Putting here some initialization that should later be moved away (Gamma window)
  x393_status_ctrl_t      status_ctrl =      {.d32=0};
  x393_gamma_ctl_t        gamma_ctl =        {.d32=0};
  x393_gamma_height01m1_t gamma_height01m1 = {.d32=0};
  x393_gamma_height2m1_t  gamma_height2m1 =  {.d32=0};
  x393_lens_height_m1_t   vign_hight0 =      {.d32=0};
  x393_lens_height_m1_t   vign_hight1 =      {.d32=0};
  x393_lens_height_m1_t   vign_hight2 =      {.d32=0};
  x393_sens_mode_t        sens_mode =        {.d32=0};
  x393_camsync_mode_t     camsync_mode =     {.d32=0};

  dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
  MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
  status_ctrl.mode = 3; // mode auto
  set_x393_cmprs_status(status_ctrl, sensor_port);

  //Until multiple subchannels supported, use just 0
  gamma_height01m1.height0m1= 0xffff;
  gamma_height01m1.height1m1= 0;
  gamma_height2m1.height2m1 = 0;

  vign_hight0.height_m1 = 0xffff;
  vign_hight1.height_m1 = 0;
  vign_hight2.height_m1 = 0;

  // Enable gamma module to pass through
  gamma_ctl.en =        1;
  gamma_ctl.en_set =    1;
  gamma_ctl.repet =     1;
  gamma_ctl.repet_set = 1;

  X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_gamma_ctrl, gamma_ctl);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_gamma_ctrl,  0x%x)\n",sensor_port, sensor_port, frame16, gamma_ctl.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_gamma_ctrl,  0x%x)\n",sensor_port, frame16, gamma_ctl.d32)

  X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_gamma_height01m1, gamma_height01m1);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_gamma_height01m1,  0x%x)\n",sensor_port, sensor_port, frame16, gamma_height01m1.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_gamma_height01m1,  0x%x)\n",sensor_port, frame16, gamma_height01m1.d32)
  X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_gamma_height2m1, gamma_height2m1);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_gamma_height2m1,  0x%x)\n",sensor_port, sensor_port, frame16, gamma_height2m1.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_gamma_height2m1,  0x%x)\n",sensor_port, frame16, gamma_height2m1.d32)

  X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_height0_m1, vign_hight0);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height0_m1,  0x%x)\n",sensor_port, sensor_port, frame16, vign_hight0.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height0_m1,  0x%x)\n",sensor_port, frame16, vign_hight0.d32)

  X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_height1_m1, vign_hight1);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height1_m1,  0x%x)\n",sensor_port, sensor_port, frame16, vign_hight1.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height1_m1,  0x%x)\n",sensor_port, frame16, vign_hight1.d32)

  X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_height2_m1, vign_hight2);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height2_m1,  0x%x)\n",sensor_port, sensor_port, frame16, vign_hight2.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_height2_m1,  0x%x)\n",sensor_port, frame16, vign_hight2.d32)

  // TODO NC393: Setup for all used sub-channels (or is it in multisensor?)
  sens_mode.hist_en =    1; // just first subchannel
  sens_mode.hist_nrst =  1; // just first subchannel
  sens_mode.hist_set =   1;
  sens_mode.chn_en =     1;
  sens_mode.chn_en_set = 1;

  X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_mode, sens_mode);
  dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_mode,  0x%x)\n",
          sensor_port, sensor_port, frame16, sens_mode.d32);
  MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_mode,  0x%x)\n",
          sensor_port, frame16, sens_mode.d32)

/*
        self.set_camsync_period  (0) # reset circuitry
        self.X393_gpio.set_gpio_ports (port_a = True)
        self.set_camsync_mode (
                               en = True,
                               en_snd = True,
                               en_ts_external = external_timestamp,
                               triggered_mode = trigger_mode,
                               master_chn =     0,
                               chn_en = sensor_mask)

  void                         x393_camsync_mode                   (x393_camsync_mode_t d);                         // CAMSYNC mode

// CAMSYNC mode

typedef union {
    struct {
          u32              en: 1; // [    0] (1) Enable CAMSYNC module
          u32          en_set: 1; // [    1] (1) Set 'en' bit
          u32          en_snd: 1; // [    2] (1) Enable sending timestamps (valid with 'en_snd_set')
          u32      en_snd_set: 1; // [    3] (0) Set 'en_snd'
          u32             ext: 1; // [    4] (1) Use external (received) timestamps, if available. O - use local timestamps
          u32         ext_set: 1; // [    5] (0) Set 'ext'
          u32            trig: 1; // [    6] (1) Sensor triggered mode (0 - free running sensor)
          u32        trig_set: 1; // [    7] (0) Set 'trig'
          u32      master_chn: 2; // [ 9: 8] (0) master sensor channel (zero delay in internal trigger mode, delay used for flash output)
          u32  master_chn_set: 1; // [   10] (0) Set 'master_chn'
          u32         ts_chns: 4; // [14:11] (1) Channels to generate timestmp messages (bit mask)
          u32     ts_chns_set: 4; // [18:15] (0) Sets for 'ts_chns' (each bit controls corresponding 'ts_chns' bit)
          u32                :13;
    };
    struct {
          u32             d32:32; // [31: 0] (0) cast to u32
    };
} x393_camsync_mode_t;
           self.set_camsync_mode (
                               en = True,
                               en_snd = True,
                               en_ts_external = False,
                               triggered_mode = False,
                               master_chn =     0,
                               chn_en = sensor_mask)

 */

  camsync_mode.en =       1;
  camsync_mode.en_set =   1;
  camsync_mode.ext =      1; // 0;
  camsync_mode.ext_set =  1;
  camsync_mode.trig =     0;
  camsync_mode.trig_set = 1;

  x393_camsync_mode                   (camsync_mode);  // immediate mode, bypass sequencer CAMSYNC mode
                                                       // (TODO NC393: Make it possible to synchronize writes (through sequencer)
  set_x393_camsync_trig_period        (0);  // reset circuitry (immediate mode)
  if (frame16 >= 0) return -1; // should be ASAP
//TODO: seems nothing to do here - all in the sensor-specific function:
//Adding setup compressor report status mode - maybe move elsethere? Needed for comprfessor frame reporting, has to be done just once
  return 0;
}


/** Restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
 TODO: NC393 add default P_SENSOR_IFACE_TIM0..3 for parallel/serial? */
int pgm_afterinit      (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return always 0

{
    struct frameparspair_t pars_to_update[24]; // 20 needed, increase if more entries will be added
    int nupdate=0;
    int woi_width, woi_height;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)

    // If this is a multisensor camera, update composite sensor dimensions (will trigger other related changes)
    // For single sensors sensor size is updated only after initialization, with composite it needs to be updated after vertical gap  or number of active sesnors is changed
    //  if (GLOBALPARS(G_SENS_AVAIL) ) multisensor_pgm_afterinit0 (sensor, thispars, prevpars,frame16);
    // Setup WOI. If size is zero - use maximal that sensor can, in non-zero - just refresh so appropriate actions will be scheduled on chnange
    woi_width=thispars->pars[P_WOI_WIDTH];
    woi_height=thispars->pars[P_WOI_HEIGHT];
    if ((woi_width == 0) || (woi_height == 0)) { ///were zeroes
        woi_width= thispars->pars[P_SENSOR_WIDTH];
        woi_height=thispars->pars[P_SENSOR_HEIGHT];
    }
    SETFRAMEPARS_SET(P_WOI_WIDTH | FRAMEPAIR_FORCE_NEW, woi_width);
    SETFRAMEPARS_SET(P_WOI_HEIGHT | FRAMEPAIR_FORCE_NEW, woi_height);
    SETFRAMEPARS_UPDATE(P_WOI_LEFT | FRAMEPAIR_FORCE_NEW);
    SETFRAMEPARS_UPDATE(P_WOI_TOP | FRAMEPAIR_FORCE_NEW);
    SETFRAMEPARS_UPDATE(P_FLIPH | FRAMEPAIR_FORCE_NEW); // FLIPH==0 will use sensor board default flip
    SETFRAMEPARS_UPDATE(P_FLIPV | FRAMEPAIR_FORCE_NEW); // FLIPV==0 will use sensor board default flip

    SETFRAMEPARS_UPDATE_SET(P_HISTWND_RWIDTH | FRAMEPAIR_FORCE_NEW, 0x8000);  // histogram 50% width (0x10000 - 100%)
    SETFRAMEPARS_UPDATE_SET(P_HISTWND_RHEIGHT | FRAMEPAIR_FORCE_NEW, 0x8000); // histogram 50% height
    SETFRAMEPARS_UPDATE_SET(P_HISTWND_RLEFT | FRAMEPAIR_FORCE_NEW, 0x8000);   // histogram 50% left (middle)
    SETFRAMEPARS_UPDATE_SET(P_HISTWND_RTOP | FRAMEPAIR_FORCE_NEW, 0x8000);    // histogram 50% top (middle)

    SETFRAMEPARS_UPDATE_SET(P_DCM_HOR | FRAMEPAIR_FORCE_NEW,  1);
    SETFRAMEPARS_UPDATE_SET(P_DCM_VERT | FRAMEPAIR_FORCE_NEW, 1);
    SETFRAMEPARS_UPDATE_SET(P_BIN_HOR | FRAMEPAIR_FORCE_NEW,  1);
    SETFRAMEPARS_UPDATE_SET(P_BIN_VERT | FRAMEPAIR_FORCE_NEW, 1);
    // Set analog gains to 1.0 if not set otherwise
    // FIXME: Without those 3 lines it is not initialaized  (or immediately reset) from the parameters in autocampars.xml. GAING=0x10000, all the rest - 0
    /*
 if (!(GLOBALPARS(G_DEBUG) & (1 <<28))) { // debugging here ! ***********************************
    SETFRAMEPARS_UPDATE_SET(P_GAINR | FRAMEPAIR_FORCE_NEW, 0x20000); ///gain ==1.0
    SETFRAMEPARS_UPDATE_SET(P_GAING | FRAMEPAIR_FORCE_NEW, 0x20000); ///gain ==1.0
    SETFRAMEPARS_UPDATE_SET(P_GAINB | FRAMEPAIR_FORCE_NEW, 0x20000); ///gain ==1.0
    SETFRAMEPARS_UPDATE_SET(P_GAINGB | FRAMEPAIR_FORCE_NEW,0x20000); ///gain ==1.0
  }
     */
    SETFRAMEPARS_UPDATE(P_BAYER | FRAMEPAIR_FORCE_NEW);

    // Exposure use non-zero P_EXPOS, then P_VEXPOS, then P_EXPOS=10ms
    if (thispars->pars[P_EXPOS]) {
        SETFRAMEPARS_UPDATE(P_EXPOS | FRAMEPAIR_FORCE_NEW);
    } else if (thispars->pars[P_VEXPOS]) {
        SETFRAMEPARS_UPDATE(P_VEXPOS | FRAMEPAIR_FORCE_NEW);
    } else {
        SETFRAMEPARS_SET(P_EXPOS | FRAMEPAIR_FORCE_NEW, 10000); // set exposure to 0.01 sec
    }
    SETFRAMEPARS_UPDATE(P_TRIG | FRAMEPAIR_FORCE_NEW); // set trigger mode (or should it always be internal after init?)
    // something else to add? NOTE: Only sensor parameters, erased when it is reset - other parameters should not chnage here
    // NOTE: increase pars_to_update[24] size if needed
    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    return 0;
}

/** Program sensor clock and phase */
int pgm_sensorphase    (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
///< be applied to,  negative - ASAP
///< @return always 0

{
#ifndef NC353
    x393_sensio_ctl_t  sensio_ctl = {.d32=0};
    x393_sensio_tim0_t sensio_tim0 = {.d32=0};
    x393_sensio_tim1_t sensio_tim1 = {.d32=0};
    x393_sensio_tim2_t sensio_tim2 = {.d32=0};
    x393_sensio_tim3_t sensio_tim3 = {.d32=0};
// TODO: Temporary for NC393 - just applying  SENSOR_IFACE_TIM0 - SENSOR_IFACE_TIM3 to FPGA, no calculations
// Actual functionality should be moved to separate modules as they are different for parallel and hispi (may be more in the future)
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (FRAMEPAR_MODIFIED(P_SENSOR_IFACE_TIM0)) {
        sensio_tim0.d32 = thispars->pars[P_SENSOR_IFACE_TIM0];
//        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_tim0, sensio_tim0);
        set_x393_sensio_tim0  (sensio_tim0, sensor_port); // write directly, sequencer may be not operational
        sensio_ctl.set_dly = 1;
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim0,  0x%x)\n",sensor_port, sensor_port, frame16, sensio_tim0.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim0,  0x%x)\n", sensor_port, frame16, sensio_tim0.d32)
    }
    if (FRAMEPAR_MODIFIED(P_SENSOR_IFACE_TIM1)) {
        sensio_tim1.d32 = thispars->pars[P_SENSOR_IFACE_TIM1];
//        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_tim1, sensio_tim1);
        set_x393_sensio_tim1  (sensio_tim1, sensor_port); // write directly, sequencer may be not operational
        sensio_ctl.set_dly = 1;
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim1,  0x%x)\n",sensor_port, sensor_port, frame16, sensio_tim1 .d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim1,  0x%x)\n", sensor_port, frame16, sensio_tim1 .d32)
    }
    if (FRAMEPAR_MODIFIED(P_SENSOR_IFACE_TIM2)) {
        sensio_tim2.d32 = thispars->pars[P_SENSOR_IFACE_TIM2];
//        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_tim2, sensio_tim2);
        set_x393_sensio_tim2  (sensio_tim2, sensor_port); // write directly, sequencer may be not operational
        sensio_ctl.set_dly = 1;
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim2,  0x%x)\n",sensor_port, sensor_port, frame16, sensio_tim2.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim2,  0x%x)\n",sensor_port, frame16, sensio_tim2.d32)
    }
    if (FRAMEPAR_MODIFIED(P_SENSOR_IFACE_TIM3)) {
        sensio_tim3.d32 = thispars->pars[P_SENSOR_IFACE_TIM3];
//        X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_tim3, sensio_tim3);
        set_x393_sensio_tim3  (sensio_tim3, sensor_port); // write directly, sequencer may be not operational
        sensio_ctl.set_dly = 1;
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim3,  0x%x)\n",sensor_port, sensor_port, frame16, sensio_tim3.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_tim3,  0x%x)\n", sensor_port, frame16, sensio_tim3.d32)
    }

    if (get_port_interface(sensor_port) == PARALLEL12){
        if (FRAMEPAR_MODIFIED(P_SENSOR_PHASE)) { // for parallel sensor it means quadrants:  90-degree shifts for data [1:0], hact [3:2] and vact [5:4]
            sensio_ctl.quadrants = thispars->pars[P_SENSOR_PHASE] & 0x3f;
            sensio_ctl.quadrants_set = 1;
        }
        if (sensio_ctl.d32) {
            x393_sensio_ctrl(sensio_ctl, sensor_port);
            dev_dbg(g_dev_ptr,"{%d} x393_sensio_ctrl(0x%08x, %d))\n",sensor_port, sensio_ctl.d32, sensor_port);
            MDP(DBGB_PADD, sensor_port,"x393_sensio_ctrl(0x%08x, %d))\n", sensio_ctl.d32, sensor_port)
        }

    } else {
        /* TODO 393: Add HISPI code */
    }
    return 0;

#else
    struct frameparspair_t pars_to_update[5]; // ??? needed, increase if more entries will be added
    int nupdate=0;
    int i;
    int old_sensor_phase;
    int old_sensor_phase90;
    int sensor_phase;
    int sensor_phase90;
    int diff_phase;
    int diff_phase90;
    int new_freq= thispars->pars[P_CLK_SENSOR];
    int thisPhase=thispars->pars[P_SENSOR_PHASE];
    int was_sensor_freq = new_freq; // bypassing for nc393
    int  hact_shift;
    long * cableDelay;
    long * FPGADelay;
    int clk_period;  // period in ps
    int px_delay;
    int px_delay90;
    uint64_t ull_result = 1000000000000LL;

    dev_dbg(g_dev_ptr,"%s  frame16=%d\n",__func__,frame16);
    if (frame16 >= 0) return -1; // can only work in ASAP mode
    //Can not change sensor frequency
    int was_sensor_freq=getClockFreq(1); // using clock driver data, not thispars
    if (unlikely(new_freq > sensor->maxClockFreq)) {
        new_freq= sensor->maxClockFreq;
        SETFRAMEPARS_SET(P_CLK_SENSOR,  new_freq);
    }
    if ((new_freq != was_sensor_freq) || (thisPhase & 0x40000)) { // 0x40000 reprogram clock even if it did not change
        if (unlikely(setClockFreq(1, thispars->pars[P_CLK_SENSOR])<0)) { // if failed to setup frequency - use the old one
            new_freq=was_sensor_freq;
            setClockFreq(1, was_sensor_freq);
            SETFRAMEPARS_SET(P_CLK_SENSOR,  new_freq);
        }
        X3X3_RSTSENSDCM; // Make Xilinx Spartan DCM happy (it does not like changing input clock)
        X3X3_SENSDCM_CLK2X_RESET; // reset pclk2x DCM also
        if (sensor->needReset & SENSOR_NEED_RESET_CLK) schedule_this_pgm_func(sensor_port, thispars, onchange_initsensor);
        // set HACT PHASE here - 90 deg. increment - seems like a bug in MT9P001 sensors - horisontal (and vert.) sync has different phase than data
        // Update (program) hact_shift only when the frequency is changed, not when just the phase is
        // adjustment range is 270 early to 360 late
        hact_shift= 0.004f * (((int)sensor->hact_delay)/(1000000000.0f/new_freq)) + 4.5f;
        MDF16(printk ("hact_shift=%d-4\n",hact_shift));
        hact_shift-=4;
        MDF16(printk ("hact_shift=%d\n",hact_shift));
        if (hact_shift>4) hact_shift=4;
        else if (hact_shift<-3) hact_shift=-3;
        X3X3_SENSDCM_HACT_ZERO;
        if      (hact_shift>0) for (i=0; i <  hact_shift; i++) {
            X3X3_SENSDCM_HACT_LATE90;
        }
        else if (hact_shift<0) for (i=0; i < -hact_shift; i++) {
            X3X3_SENSDCM_HACT_EARLY90;
        }
        // Calculate and set sensor phase (only initially and when clock frequency is chnaged)
        cableDelay= (long *) &GLOBALPARS(sensor_port, G_CABLE_TIM );
        FPGADelay=  (long *) &GLOBALPARS(sensor_port, G_FPGA_TIM0 );
//        clk_period= 1000000000000.0f/new_freq;  // period in ps
//        clk_period= 1.0E12f/new_freq;  // period in ps
        do_div(ull_result,new_freq);
        clk_period= ull_result;

        MDF16(printk ("cableDelay=%ld, FPGADelay=%ld, clk_period=%d\n",cableDelay[0], FPGADelay[0], clk_period));
        px_delay=-(clk_period/2 - FPGADelay[0]- cableDelay[0] - ((int) sensor->sensorDelay)) ;
        MDF16(printk ("px_delay=%d\n",px_delay));
        px_delay90=(4*px_delay+clk_period/2)/clk_period;
        px_delay -= (px_delay90*clk_period)/4; // -clk_period/8<= now px_delay <= +clk_period/8
        MDF16(printk ("px_delay=%d, px_delay90=%d\n",px_delay,px_delay90));
        px_delay/= FPGA_DCM_STEP; // in DCM steps
        thisPhase= (px_delay & 0xffff) | ((px_delay90 & 3) <<16) | 0x80000;
    }


    if (thisPhase!=prevpars->pars[P_SENSOR_PHASE]) {
        old_sensor_phase= prevpars->pars[P_SENSOR_PHASE] & 0xffff;  if (old_sensor_phase>=0x8000) old_sensor_phase-=0x10000; // make it signed
        old_sensor_phase90= (prevpars->pars[P_SENSOR_PHASE]>>16)&3;
        if (thisPhase & 0x80000) {
            X3X3_RSTSENSDCM; // start from zero phase
            old_sensor_phase=0;
            old_sensor_phase90=0;
        }
        sensor_phase90=(thisPhase>>16)&3;
        sensor_phase=   thisPhase & 0xffff;  if (sensor_phase>=0x8000) sensor_phase-=0x10000; // make it signed
        diff_phase=  sensor_phase-  old_sensor_phase;
        diff_phase90=sensor_phase90-old_sensor_phase90;
        if (diff_phase90>2)diff_phase90-=4;
        else if (diff_phase90<-1)diff_phase90+=4;
        dev_dbg(g_dev_ptr,"%s old_sensor_phase=0x%x old_sensor_phase90=0x%x\n",__func__, old_sensor_phase, old_sensor_phase90 );
        dev_dbg(g_dev_ptr,"%s sensor_phase=0x%x sensor_phase90=0x%x\n",__func__, sensor_phase, sensor_phase90 );
        dev_dbg(g_dev_ptr,"%s diff_phase=0x%x diff_phase90=0x%x\n",__func__, diff_phase, diff_phase90 );
        if      (diff_phase > 0) for (i=diff_phase; i > 0; i--) {X3X3_SENSDCM_INC ; udelay(1); }
        else if (diff_phase < 0) for (i=diff_phase; i < 0; i++) {X3X3_SENSDCM_DEC ; udelay(1); }
        if      (diff_phase90 > 0) for (i=diff_phase90; i > 0; i--) {X3X3_SENSDCM_INC90 ; udelay(1); }
        else if (diff_phase90 < 0) for (i=diff_phase90; i < 0; i++) {X3X3_SENSDCM_DEC90 ; udelay(1); }
        SETFRAMEPARS_SET(P_SENSOR_PHASE,  thisPhase & 0x3ffff);
    }

    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    return 0;
#endif
}

/** Program sensor exposure - nothing to be done here, all sensor-specific */
int pgm_exposure   (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return always 0
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return 0;
}

/** Program i2c bytes/speed (may be per-frame?) 393: remove*/
int pgm_i2c        (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return always 0
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d (do nothing here?)\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
    int fpga_addr = frame16;
    X3X3_SEQ_SEND1(fpga_addr,   X313_I2C_CMD,  X3X3_SET_I2C_BYTES(thispars->pars[P_I2C_BYTES]+1) |
            X3X3_SET_I2C_DLY  (thispars->pars[P_I2C_QPERIOD]));
#endif
//dev_dbg(g_dev_ptr,"%s   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",__func__, frame16,  (int) X313_I2C_CMD, (int) (X3X3_SET_I2C_BYTES(thispars->pars[P_I2C_BYTES]+1) | X3X3_SET_I2C_DLY  (thispars->pars[P_I2C_QPERIOD]))  );
return 0;
}

/** Program sensor WOI and mirroring
 *
 * As different sensors may produce "bad frames" for differnt WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped) */
int pgm_window     (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return always 0

{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Program sensor WOI and mirroring, safe mode. Does the same as pgm_window, at least not sensor-specific */
int pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
					 struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					 struct framepars_t * thispars, ///< sensor current parameters
					 struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					 int frame16)                   ///< 4-bit (hardware) frame number parameters should
												    ///< be applied to,  negative - ASAP
												    ///< @return always 0

{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

//** Common (not sensor-specific) part of processing WOI parameters*/
int pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return always 0
{
    int dv,dh,bv,bh,width,height,timestamp_len,oversize,pfh,pf_stripes,ah, left,top,is_color;
    int sensor_width;
    int sensor_height;
    struct frameparspair_t pars_to_update[18]; // 15 needed, increase if more entries will be added
    int nupdate=0;
    int clearHeight;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",            sensor_port,frame16);
    dev_dbg(g_dev_ptr,"{%d} thispars->pars[P_WOI_HEIGHT]=%lx thispars->pars[P_WOI_WIDTH]=%lx\n", sensor_port,thispars->pars[P_WOI_HEIGHT], thispars->pars[P_WOI_WIDTH]);
    //  if (GLOBALPARS(G_SENS_AVAIL) ) multisensor_pgm_window_common0 (sensor, thispars, prevpars, frame16);

    sensor_width= thispars->pars[P_SENSOR_WIDTH];
    sensor_height=thispars->pars[P_SENSOR_HEIGHT];
    oversize=thispars->pars[P_OVERSIZE];
    is_color=1;

    switch (thispars->pars[P_COLOR] & 0x0f){
    case COLORMODE_MONO6:
    case COLORMODE_MONO4:
        is_color=0;
    }
    // flips changed?
    if (FRAMEPAR_MODIFIED(P_FLIPH)) {
        if (unlikely((thispars->pars[P_FLIPH] & sensor->flips & 1)!=thispars->pars[P_FLIPH])) { // remove unsupoported flips
            SETFRAMEPARS_SET(P_FLIPH, (thispars->pars[P_FLIPH] & sensor->flips & 1));
        }
    }
    if (FRAMEPAR_MODIFIED(P_FLIPV)) {
        if (unlikely((thispars->pars[P_FLIPV] & (sensor->flips>>1) & 1)!=thispars->pars[P_FLIPV])) { // remove unsupoported flips
            SETFRAMEPARS_SET(P_FLIPV, (thispars->pars[P_FLIPV] & (sensor->flips >> 1 ) & 1));
        }
    }
    // dh (decimation changed)?
    dh = thispars->pars[P_DCM_HOR];
    dh = dh?dh:1;
    if (FRAMEPAR_MODIFIED(P_DCM_HOR)) {
        if (dh<1) dh=1; else if (dh>32) dh=32;
        while ((dh>1) && !(sensor->dcmHor  & (1 << (dh-1)))) dh--; // adjust decimation to maximal supported (if requested is not supported)
        if (unlikely(dh!=thispars->pars[P_DCM_HOR])) SETFRAMEPARS_SET(P_DCM_HOR, dh);
    }
    // dv (decimation changed)?
    dv = thispars->pars[P_DCM_VERT];
    dv = dv?dv:1;
    if (FRAMEPAR_MODIFIED(P_DCM_VERT)) {
        if (dv<1) dv=1; else if (dv>32) dv=32;
        while ((dv>1) && !(sensor->dcmVert  & (1 << (dv-1)))) dv--; // adjust decimation to maximal supported (if requested is not supported)
        if (unlikely(dv!=thispars->pars[P_DCM_HOR])) SETFRAMEPARS_SET(P_DCM_VERT, dv);
    }
    // bh (binning changed)?
    bh = thispars->pars[P_BIN_HOR];
    dv = dv?dv:1;
    if (FRAMEPAR_MODIFIED(P_BIN_HOR)) {
        if (bh<1) bh=1; else if (bh>dh) bh=dh;
        while ((bh>1) && !(sensor->binHor  & (1 << (bh-1)))) bh--; // adjust binning to maximal supported (if requested is not supported)
        if (unlikely(bh!=thispars->pars[P_BIN_HOR])) SETFRAMEPARS_SET(P_DCM_HOR, bh);
    }
    // bv (binning changed)?
    bv = thispars->pars[P_BIN_VERT];
    bv = bv?bv:1;
    if (FRAMEPAR_MODIFIED(P_BIN_VERT)) {
        if (bv<1) bv=1; else if (bv>dv) bv=dv;
        while ((bv>1) && !(sensor->binVert  & (1 << (bv-1)))) bv--; // adjust binning to maximal supported (if requested is not supported)
        if (unlikely(bv!=thispars->pars[P_BIN_VERT])) SETFRAMEPARS_SET(P_DCM_VERT, bv);
    }
    // any other binning adjustmens needed?
    // adjustP_WOI_WIDTH, P_ACTUAL_WIDTH, P_SENSOR_PIXH - depends on decimation, photofinish mode
    timestamp_len = (((thispars->pars[P_PF_HEIGHT] >> 16) & 3)==2)? X313_TIMESTAMPLEN*dh : 0;
    // adjust width/height first, then adjust left top
    width=thispars->pars[P_WOI_WIDTH];
    //  if ((!oversize) && (width > sensor->imageWidth)) width= sensor->imageWidth;
    //  if ((!oversize) && (width > sensor_width)) width= sensor_width;
    if ((!oversize) && (width > sensor_width)) width= sensor_width;

    // make width to be multiple of the compressor tile (before adding margins of 4 pixels)
    width= (((width/dh) + timestamp_len)/X393_TILEHOR)*X393_TILEHOR-timestamp_len; // divided by dh
    // suppose minimal width refers to decimated output
    while (width < sensor->minWidth) width+=X393_TILEHOR;
    if (unlikely(thispars->pars[P_ACTUAL_WIDTH] != (width+timestamp_len))) {
        SETFRAMEPARS_SET(P_ACTUAL_WIDTH, width+timestamp_len); ///full width for the compressor, including timestamp, but excluding margins
    }
    if (unlikely(thispars->pars[P_SENSOR_PIXH] != width+(2 * COLOR_MARGINS)))
        SETFRAMEPARS_SET(P_SENSOR_PIXH,  width+(2 * COLOR_MARGINS)); ///full width for the sensor (after decimation), including margins
    width*=dh;
    if (unlikely(thispars->pars[P_WOI_WIDTH] != width))
        SETFRAMEPARS_SET(P_WOI_WIDTH, width);  // WOI width, as specified (corrected for the sensor if needed)
    // adjustP_WOI_HEIGHT, P_ACTUAL_HEIGHT, P_SENSOR_PIXV - depends on decimation, photofinish mode
    pfh = (thispars->pars[P_PF_HEIGHT] & 0xffff);
    height=thispars->pars[P_WOI_HEIGHT];
    //  pf_stripes;
    if(pfh > 0) {
        if (pfh < sensor->minHeight)  pfh =  sensor->minHeight;
        if (pfh & 1) pfh++;
        if (height > X393_MAXHEIGHT*dv) height=X393_MAXHEIGHT*dv;
        pf_stripes = height / (pfh * dv);
        if(pf_stripes < 1) pf_stripes = 1;
        if (unlikely(thispars->pars[P_SENSOR_PIXV] != pfh)) {
            SETFRAMEPARS_SET(P_SENSOR_PIXV,  pfh);
            dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_SENSOR_PIXV,  0x%x)\n", sensor_port, pfh);
            MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_SENSOR_PIXV,  0x%x)\n", pfh)
        }
    } else {
        if ((!oversize ) && (height > sensor_height)) height=sensor_height;
        height= ((height/dv)/X393_TILEVERT) * X393_TILEVERT; // divided by dv (before multisensor options)
        // suppose minimal height refers to decimated output
        while (height < sensor->minHeight) height+=X393_TILEVERT;
        if (unlikely(thispars->pars[P_SENSOR_PIXV] != height+(2 * COLOR_MARGINS)))
            SETFRAMEPARS_SET(P_SENSOR_PIXV,  height+(2 * COLOR_MARGINS)); ///full height for the sensor (after decimation), including margins
        height*=dv;
        pf_stripes = 0;
        pfh = 0;
    }
    // update photofinish height P_PF_HEIGHT
    if (unlikely((thispars->pars[P_PF_HEIGHT] & 0xffff) != pfh)) {
        SETFRAMEPARS_SET(P_PF_HEIGHT, (thispars->pars[P_PF_HEIGHT] & 0xffff0000 ) | pfh );
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_PF_HEIGHT,  0x%x)\n", sensor_port,(int) ((thispars->pars[P_PF_HEIGHT] & 0xffff0000 ) | pfh));
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_PF_HEIGHT,  0x%x)\n", (int) ((thispars->pars[P_PF_HEIGHT] & 0xffff0000 ) | pfh))
    }
    // update WOI height [P_WOI_HEIGHT
    if (unlikely(thispars->pars[P_WOI_HEIGHT] != height)) {
        SETFRAMEPARS_SET(P_WOI_HEIGHT, height); ///full height for the compressor (excluding margins)
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_WOI_HEIGHT,  0x%x)\n", sensor_port, height);
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_WOI_HEIGHT,  0x%x)\n", height)
    }
    // update P_ACTUAL_HEIGHT
    ah=height/dv;
    if (unlikely(thispars->pars[P_ACTUAL_HEIGHT] != ah)) {
        SETFRAMEPARS_SET(P_ACTUAL_HEIGHT, ah); ///full height for the compressor (excluding margins)
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_ACTUAL_HEIGHT,  0x%x)\n", sensor_port, ah);
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_ACTUAL_HEIGHT,  0x%x)\n", ah)
    }
    // left margin
    left = thispars->pars[P_WOI_LEFT];
    if (!oversize) { // in oversize mode let user to specify any margin, including odd ones (bayer shifted)
        if (is_color)                    left &= 0xfffe;
        if ((left + width + (2 * COLOR_MARGINS)) > sensor->clearWidth) {
            left = sensor->clearWidth - width - (2 * COLOR_MARGINS);
            if (is_color)               left &= 0xfffe;
        }
        if (left & 0x8000) left = 0;
    }
    // update P_WOI_LEFT
    if (unlikely(thispars->pars[P_WOI_LEFT] != left)) {
        SETFRAMEPARS_SET(P_WOI_LEFT, left);
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_WOI_LEFT,  0x%x)\n", sensor_port, left);
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_WOI_LEFT,  0x%x)\n", left)
    }

    // top margin
    top = thispars->pars[P_WOI_TOP];
    clearHeight=(sensor->clearHeight-sensor->imageHeight) + thispars->pars[P_SENSOR_HEIGHT];
    if (!oversize) { // in oversize mode let user to specify any margin, including odd ones (bayer shifted)
        if (is_color)                    top &= 0xfffe;
        if ((top + ((pfh>0)?0:height) + (2 * COLOR_MARGINS)) > clearHeight) {
            top = clearHeight - ((pfh>0)?0:height) - (2 * COLOR_MARGINS);
            if (is_color)               top &= 0xfffe;
        }
        if (top & 0x8000) top = 0;
    }
    // update P_WOI_TOP
    if (unlikely(thispars->pars[P_WOI_TOP] != top)) {
        SETFRAMEPARS_SET(P_WOI_TOP, top);
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_WOI_TOP,  0x%x)\n", sensor_port, top);
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_WOI_TOP,  0x%x)\n", top)
    }
    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    return 0;
}

/** Convert time interval in camsync periods (10ns) to sensor pixel periods */
unsigned long camsync_to_sensor(unsigned long camsync_time,  ///< time interval in camsync periods (@100MHz)
                                unsigned long sensor_clk)    ///< sensor pixel clock in Hz
                                                          ///< @return time interval in pixel clock periods
{
    uint64_t ull_period = (((long long) sensor_clk) * ((long long) camsync_time));
    do_div(ull_period, CAMSYNC_FREQ);
    return (unsigned long) ull_period;
}

/** Convert time interval sesnor pixel periods to camsync periods (10ns)*/
unsigned long sensor_to_camsync(unsigned long pixel_time, ///< time interval in pixel clock periods
                                unsigned long sensor_clk) ///< sensor pixel clock in Hz
                                                          ///< @return time interval in camsync periods (@100MHz)
{
    uint64_t ull_period = (((long long) pixel_time) * ((long long) CAMSYNC_FREQ));
    do_div(ull_period, sensor_clk);
    return (unsigned long) ull_period;
}

/** Check if compressor can keep up, limit sensor FPS if needed.
 *  Apply both user FPS limits (both low and high), prepare data for the sensor function and sequencer.
 *
 * TODO: Program external trigger frequency to the same value!
 * FIXME:  when geometry changes, _video_ exposure needs to be adjusted to keep absolute one (at least - mark as new?) */
int pgm_limitfps   (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return always 0

{
    // Calculate minimal frame period compressor can handle, apply requested fps, limit/program the sequencer
    struct frameparspair_t pars_to_update[8]; // 4 needed, increase if more entries will be added
    int nupdate=0;
    int async=(thispars->pars[P_TRIG] & 4)?1:0;
    int cycles;  // number of FPGA clock cycles per frame;
    int min_period;  // number of pixel clock periods needed for the compressor (or user limit)
    int period=0;
    int pfh;
    int n_stripes;
    int min_period_camsync, period_camsync;
    //#define CAMSYNC_FREQ 100000000 ///< 100MHz - frequency used for external trigger periods and delays
    u32 clk_sensor = thispars->pars[P_CLK_SENSOR];
    u32 clk_fpga = thispars->pars[P_CLK_FPGA];
#if USELONGLONG
    uint64_t ull_min_period;
    uint64_t ull_period;
#endif
    if (!clk_fpga) clk_fpga = 240000000; // 240MHz
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    cycles=thispars->pars[P_TILES]; // number of tiles
    //  dev_dbg(g_dev_ptr,"{%d}  tiles=%d(0x%x)\n",sensor_port,cycles,cycles);
    switch (thispars->pars[P_COLOR] & 0x0f){
    case COLORMODE_MONO6:
    case COLORMODE_COLOR:
    case COLORMODE_JP46:
    case COLORMODE_JP46DC:
    case COLORMODE_COLOR20:
        cycles*=6; // now - blocks per frame
        break;
    default:
        cycles*=4;
    }
    //    cycles  *=64 *2; //cycles per frame (64 pixels/block, 2 clock cycles/pixel)
    cycles  *=64; //cycles per frame (64 pixels/block, 1 clock cycles/pixel in NC393)
    dev_dbg(g_dev_ptr,"{%d}  cycles=%d(0x%x)\n",sensor_port,cycles,cycles);
    MDP(DBGB_PADD, sensor_port,"cycles=%d(0x%x)\n",cycles,cycles)
    cycles += thispars->pars[P_FPGA_XTRA]; // extra cycles needed for the compressor to start/finish the frame;
    dev_dbg(g_dev_ptr,"{%d}  cycles with P_FPGA_XTRA =%d(0x%x)\n",sensor_port,cycles,cycles);
    MDP(DBGB_PADD, sensor_port,"cycles with P_FPGA_XTRA =%d(0x%x)\n",cycles,cycles)
    // #define P_CLK_FPGA, #define P_CLK_SENSOR    27-28 bits, cycles - 24-25 bits
    if (!clk_sensor) {
        clk_sensor=90000000;
        dev_dbg(g_dev_ptr,"{%d}  clk_sensor is 0, setting to %d\n",sensor_port,(int)clk_sensor);
        MDP(DBGB_PADD, sensor_port,"clk_sensor is 0, setting to %d\n",(int)clk_sensor)
    }

#if USELONGLONG
    ull_min_period=(((long long) cycles) * ((long long) clk_sensor));
#ifdef __div64_32
    __div64_32(&ull_min_period, clk_fpga);
#else
    do_div(ull_min_period, clk_fpga);
    //    ull_min_period/=thispars->pars[P_CLK_FPGA];
#endif
    min_period= ull_min_period;
    dev_dbg(g_dev_ptr,"{%d} min_period =%d(0x%x)\n", sensor_port, min_period, min_period);
    MDP(DBGB_PADD, sensor_port,"min_period =%d(0x%x)\n", min_period, min_period)
    //  min_period = (((long long) cycles) * ((long long) thispars->pars[P_CLK_SENSOR])) / ((long long) thispars->pars[P_CLK_FPGA]);
#else
    if (cycles <          (1<<16) ) {
        min_period = (cycles * (thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12);
    } else   if (cycles < (1<<18) ) {
        min_period = (((cycles>>2) * (thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12)) << 2 ;
    } else   if (cycles < (1<<20) ) {
        min_period = (((cycles>>4) * (thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12)) << 4 ;
    } else   if (cycles < (1<<22) ) {
        min_period = (((cycles>>6) * (thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12)) << 6 ;
    } else   if (cycles < (1<<24) ) {
        min_period = (((cycles>>8) * (thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12)) << 8 ;
    } else {
        min_period = (((cycles>>10) *(thispars->pars[P_CLK_SENSOR] >> 12)) / (thispars->pars[P_CLK_FPGA]>>12)) << 10 ;
    }
    ///? fp1000s= 10*sclk/(pix_period/100);
    dev_dbg(sensor_port,"%s min_period =%d(0x%x)\n",sensor_port,min_period,min_period);
#endif
    // is there limit set for the FPS?
    if (thispars->pars[P_FPSFLAGS]) {
#if USELONGLONG
        ull_period=(((long long) clk_sensor) * (long long) 1000);
#ifdef __div64_32
        __div64_32(&ull_period, thispars->pars[P_FP1000SLIM]);
#else
        do_div(ull_period, thispars->pars[P_FP1000SLIM]);
        //        ull_period /= thispars->pars[P_FP1000SLIM];
#endif
        period= ull_period;
        dev_dbg(g_dev_ptr,"{%d} period =%d(0x%x)\n", sensor_port, period, period);
        MDP(DBGB_PADD, sensor_port,"period =%d(0x%x)\n", period, period)
        //    period=(((long long) clk_sensor) * (long long) 1000)/((long long) thispars->pars[P_FP1000SLIM]);
#else
        period=125*(( thispars->pars[P_CLK_SENSOR] << 3) / thispars->pars[P_FP1000SLIM]); // 125 <<3 = 1000
        dev_dbg(g_dev_ptr,"{%d} period =%d(0x%x)\n",sensor_port,period,period);
#endif
    }
    dev_dbg(g_dev_ptr,"{%d}  period=%d\n",sensor_port,period);
    MDP(DBGB_PADD, sensor_port,"period =%d(0x%x)\n", period, period)
    if ((thispars->pars[P_FPSFLAGS] & 1) && (period>min_period)) min_period=period;
    // *********************************************************** P_PF_HEIGHT
    pfh=thispars->pars[P_PF_HEIGHT] &0xffff ;
    if (pfh>0) {
        n_stripes=thispars->pars[P_WOI_HEIGHT]/pfh;
        if (n_stripes>0) min_period/=n_stripes;
    }

    if (min_period != thispars->pars[P_PERIOD_MIN]) {
        SETFRAMEPARS_SET(P_PERIOD_MIN, min_period);  // set it (and propagate to the later frames)
        dev_dbg(g_dev_ptr,"{%d}  SETFRAMEPARS_SET(P_PERIOD_MIN,  0x%x)\n", sensor_port, min_period);
        MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_PERIOD_MIN,  0x%x)\n", min_period)
    }
    if (((thispars->pars[P_FPSFLAGS] & 2)==0) || (period < min_period)) period=0x7fffffff; // no upper limit
    if (period != thispars->pars[P_PERIOD_MAX]) SETFRAMEPARS_SET(P_PERIOD_MAX, period);         // set it (and propagate to the later frames)
    // Now see if the sequencer period needs to be adjusted
    //  if (async && (thispars->pars[P_TRIG_PERIOD] >=256)) { // <256 - single trig
    //  if (async && (thispars->pars[P_TRIG_PERIOD] !=1)) { // <256 - single trig, here only ==1 is for single
    // Update period to comply even if it is not in async mode
    if (thispars->pars[P_TRIG_PERIOD] !=1) { // <256 - single trig, here only ==1 is for single
        min_period_camsync = sensor_to_camsync(min_period, thispars->pars[P_CLK_SENSOR]);
        if (thispars->pars[P_TRIG_PERIOD] < min_period_camsync) SETFRAMEPARS_SET(P_TRIG_PERIOD, min_period_camsync);  // set it (and propagate to the later frames)
        //        if (async &&  (thispars->pars[P_FPSFLAGS] & 2) && (thispars->pars[P_TRIG_PERIOD] > period)) {
        if (async &&  (thispars->pars[P_FPSFLAGS] & 2)) {
            period_camsync = sensor_to_camsync(period, thispars->pars[P_CLK_SENSOR]);
            if (thispars->pars[P_TRIG_PERIOD] > period_camsync) {
                SETFRAMEPARS_SET(P_TRIG_PERIOD, period_camsync);  // set it (and propagate to the later frames)
                MDP(DBGB_PADD, sensor_port,"SETFRAMEPARS_SET(P_TRIG_PERIOD, 0x%x)\n", period_camsync)
            }
        }
    }
    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    //  That's all - sensor or sequencer will be programmed later using the parameters specified here
    return 0;
}

/** Program analog gains (all is done in sensor-specific function)
 * Nothing to do here - all in the sensor-specific function  */
int pgm_gains          (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return always 0
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return 0;
}

/** Program sensor trigger mode (needs sensor-specific one too), programs timestamp mode (early/normal)
 * TODO: implement for 393 */
int pgm_triggermode(int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    x393_camsync_mode_t camsync_mode = {.d32=0};
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifndef NC353
    camsync_mode.trig =     (thispars->pars[P_TRIG] & 4)?1:0;
    if (camsync_mode.trig) {  // if trigger mode, enable camsync module, if off - do nothing
        camsync_mode.en = 1;
        camsync_mode.en_set = 1;
    }
    camsync_mode.trig_set = 1;
    // set directly, bypassing sequencer as it may fail with wrong trigger
    x393_camsync_mode (camsync_mode);
    MDP(DBGB_PADD, sensor_port,"x393_camsync_mode(0x%x)\n",camsync_mode.d32)
    return 0;
#else
    //    int fpga_addr= frame16;
    //    int async=(thispars->pars[P_TRIG] & 4)?1:0;
    X3X3_SEQ_SEND1(frame16,  X313_WA_DCR0, X353_DCR0(SENSTRIGEN,async));
    return 0;
#endif
}


/** Program sensor input in FPGA (Bayer, 8/16 bits, FPN, test mode, number of lines).
 * TODO: implement for 393 (program_sensor())
 */

int pgm_sensorin   (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
#ifndef NC353
    x393_sens_mode_t      sens_mode =    {.d32=0};
    x393_sensio_width_t   sensio_width = {.d32=0};
    x393_sens_sync_mult_t sync_mult =    {.d32=0};
    x393_gamma_ctl_t      gamma_ctl =    {.d32=0};
    int bayer;

    int n_scan_lines; //, n_ph_lines;
    int flips;
    int bayer_modified;
    x393_mcntrl_frame_start_dly_t start_dly ={.d32=0};

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (FRAMEPAR_MODIFIED(P_BITS)){
        sens_mode.bit16 = thispars->pars[P_BITS];
        sens_mode.bit16_set = 1;
        X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_mode, sens_mode);
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_mode,  0x%x)\n",
                sensor_port, sensor_port, frame16, sens_mode.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_mode,  0x%x)\n",
                sensor_port, frame16, sens_mode.d32)

#if 0   //TODO: not (yet) implemented
        (thispars->pars[P_FPGATEST]), \
        (thispars->pars[P_FPNS]),  \
        (thispars->pars[P_FPNM]),     \
        thispars->pars[P_SHIFTL]));
#endif
    }
    // Writing WOI width for internally generated HACT
    if (thispars->pars[P_FRAMESYNC_DLY] & 0x10000) { /// set enforced HACT length, if 0 - use HACT from sensor
        sensio_width.sensor_width = thispars->pars[P_ACTUAL_WIDTH]+(2 * COLOR_MARGINS);
    }
    X393_SEQ_SEND1 (sensor_port, frame16, x393_sensio_width, sensio_width);
    dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_width,  0x%x)\n",
            sensor_port, sensor_port, frame16, sensio_width.d32);
    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sensio_width,  0x%x)\n",
            sensor_port, frame16, sensio_width.d32)

    // Program number of scan lines to acquire
    // Is PhotoFinish mode enabled? // **************** TODO: use ACTUAL_HEIGHT (and update it) not WOI_HEIGHT
    if (((thispars->pars[P_PF_HEIGHT] & 0xffff)>0) && (thispars->pars[P_PF_HEIGHT]<=thispars->pars[P_ACTUAL_HEIGHT])){
        sync_mult.mult_frames=  thispars->pars[P_ACTUAL_HEIGHT]/(thispars->pars[P_PF_HEIGHT] &  0xffff) -1; // was 0x3fff in NC353 code
        n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]; // no margins here
    } else {
        // temporary hack trying to disable PH mode earlier
        n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]+(2 * COLOR_MARGINS)+thispars->pars[P_OVERLAP];
    }
    X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_sync_mult, sync_mult);
    dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_sync_mult,  0x%x)\n",
            sensor_port, sensor_port, frame16, sync_mult.d32);
    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_sync_mult,  0x%x)\n",
            sensor_port, frame16, sync_mult.d32)

    // See if NC393 has n_scan_lines - no
    /*
    n_scan_lines&=0xffff; // was 0x3fff in NC353 code
    X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, n_scan_lines);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_NLINES, (int) n_scan_lines);
    */

    // Bayer phase changed?

    flips=(thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1)<<1);
    bayer_modified=FRAMEPAR_MODIFIED(P_BAYER) || FRAMEPAR_MODIFIED(P_FLIPH) || FRAMEPAR_MODIFIED(P_FLIPV) || FRAMEPAR_MODIFIED(P_MULTI_MODE);
    dev_dbg(g_dev_ptr,"{%d}  flips=%x\n",sensor_port, flips);
    MDP(DBGB_PADD, sensor_port,"flips=%x\n",flips)
    if (thispars->pars[P_MULTI_MODE]) { // Modify flips in composite mode - should match flips of the top (direct) channel
        int this_bit=(1<<thispars->pars[P_MULTI_TOPSENSOR]);
        if (thispars->pars[P_MULTI_FLIPH] & this_bit) flips^=1;
        if (thispars->pars[P_MULTI_FLIPV] & this_bit) flips^=2;
        dev_dbg(g_dev_ptr,"{%d}  composite mode - adjusted flips=%x\n",sensor_port, flips);
        MDP(DBGB_PADD, sensor_port,"composite mode - adjusted flips=%x\n",flips)
        bayer_modified=  bayer_modified || FRAMEPAR_MODIFIED(P_MULTI_FLIPH) || FRAMEPAR_MODIFIED(P_MULTI_FLIPV)  || FRAMEPAR_MODIFIED(P_MULTI_TOPSENSOR);
    }

    // Change Bayer for gamma/histograms?
    if (bayer_modified) {
        bayer = thispars->pars[P_BAYER] ^ flips ^ sensor->bayer  ^ 3; // 3 added for NC393;
        setFramePar(sensor_port, thispars, P_COMP_BAYER | FRAMEPAIR_FORCE_PROC,   bayer);
        gamma_ctl.bayer = bayer; // 3 added for NC393
        gamma_ctl.bayer_set = 1;
    }
    //NC393: Other needed bits are set in pgm_initsensor (they must be set just once)
    // set gamma_ctl if needed
    if (gamma_ctl.d32) { // anything changed
        X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_gamma_ctrl, gamma_ctl);
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_gamma_ctrl,  0x%x)\n",sensor_port, sensor_port, frame16, gamma_ctl.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_sens_gamma_ctrl,  0x%x)\n",sensor_port, frame16, gamma_ctl.d32)
    }

    if (FRAMEPAR_MODIFIED(P_MEMSENSOR_DLY) && ((start_dly.start_dly = thispars->pars[P_MEMSENSOR_DLY]))){
        X393_SEQ_SEND1 (sensor_port, frame16, x393_sens_mcntrl_scanline_start_delay, start_dly);
        dev_dbg(g_dev_ptr,"{%d}  Setting sensor-to-memory frame sync delay  to %d (0x%x)\n",sensor_port, start_dly.start_dly,start_dly.start_dly);
        MDP(DBGB_PADD, sensor_port,"Setting sensor-to-memory frame sync delay  to %d (0x%x)\n", start_dly.start_dly,start_dly.start_dly)
    }


#if 0

    typedef union {
        struct {
              u32             run: 2; // [ 1: 0] (0) Run mode
              u32         run_set: 1; // [    2] (0) Set 'run'
              u32           qbank: 3; // [ 5: 3] (0) Quantization table bank
              u32       qbank_set: 1; // [    6] (0) Set 'qbank'
              u32           dcsub: 1; // [    7] (0) Subtract DC enable
              u32       dcsub_set: 1; // [    8] (0) Set 'qbank'
              u32           cmode: 4; // [12: 9] (0) Color format
              u32       cmode_set: 1; // [   13] (0) Set 'cmode'
              u32      multiframe: 1; // [   14] (0) Multi/single frame mode
              u32  multiframe_set: 1; // [   15] (0) Set 'multiframe'
              u32                : 2;
              u32           bayer: 2; // [19:18] (0) Bayer shift
              u32       bayer_set: 1; // [   20] (0) Set 'bayer'
              u32           focus: 2; // [22:21] (0) Focus mode
              u32       focus_set: 1; // [   23] (0) Set 'focus'
              u32                : 8;
        };
        struct {
              u32             d32:32; // [31: 0] (0) cast to u32
        };
    } x393_cmprs_mode_t;

    // Control for the gamma-conversion module

    typedef union {
        struct {
              u32           bayer: 2; // [ 1: 0] (0) Bayer color shift (pixel to gamma table)
              u32       bayer_set: 1; // [    2] (0) Set 'bayer' field
              u32            page: 1; // [    3] (0) Table page (only available if SENS_GAMMA_BUFFER in Verilog)
              u32        page_set: 1; // [    4] (0) Set 'page' field
              u32              en: 1; // [    5] (1) Enable module
              u32          en_set: 1; // [    6] (1) Set 'en' field
              u32           repet: 1; // [    7] (1) Repetitive (normal) mode. Set 0 for testing of the single-frame mode
              u32       repet_set: 1; // [    8] (1) Set 'repet' field
              u32            trig: 1; // [    9] (0) Single trigger used when repetitive mode is off (self clearing bit)
              u32                :22;
        };
        struct {
              u32             d32:32; // [31: 0] (0) cast to u32
        };
    } x393_gamma_ctl_t;
#endif

    return 0;
#else
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    // Set FPN mode (P_FPGATEST - currently only LSB is processed)
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SENSFPN, X313_SENSFPN_D( \
            (thispars->pars[P_FPGATEST]), \
            (thispars->pars[P_FPNS]),  \
            (thispars->pars[P_FPNM]),     \
            ((thispars->pars[P_BITS]>8)?1:0), \
            thispars->pars[P_SHIFTL]));

    MDF3(dev_dbg(g_dev_ptr,"  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SENSFPN, (int) (X313_SENSFPN_D( \
            (thispars->pars[P_FPGATEST]), \
            (thispars->pars[P_FPNS]),  \
            (thispars->pars[P_FPNM]),     \
            ((thispars->pars[P_BITS]>8)?1:0), \
            thispars->pars[P_SHIFTL]))));

    int n_scan_lines, n_ph_lines, n_pixels;

    // New - writing WOI width for internally generated HACT
    n_pixels=((thispars->pars[P_ACTUAL_WIDTH]+(2 * COLOR_MARGINS)) & 0x3fff) | 0x4000;
    X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, n_pixels);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_NLINES, (int) n_pixels);

    // Program number of scan lines to acquire
    // Is PhotoFinish mode enabled? // **************** TODO: use ACTUAL_HEIGHT (and update it) not WOI_HEIGHT
    if (((thispars->pars[P_PF_HEIGHT] & 0xffff)>0) && (thispars->pars[P_PF_HEIGHT]<=thispars->pars[P_ACTUAL_HEIGHT])){
        n_ph_lines=  thispars->pars[P_ACTUAL_HEIGHT]/(thispars->pars[P_PF_HEIGHT] &  0x3fff);
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_NLINES, (int) (n_ph_lines-1) | 0x8000);
        X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, (n_ph_lines-1) | 0x8000);
        //    n_scan_lines= thispars->pars[P_PF_HEIGHT];
        n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]; // no margins here

    } else {
        // temporary hack trying to disable PH mode earlier

        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, X313_SEQ_ASAP, X313_WA_NLINES, 0x8000);
        X3X3_SEQ_SEND1 (X313_SEQ_ASAP,  X313_WA_NLINES,  0x8000);

        n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]+(2 * COLOR_MARGINS)+thispars->pars[P_OVERLAP];
    }
    n_scan_lines&=0x3fff;
    X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, n_scan_lines);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_NLINES, (int) n_scan_lines);
    // Bayer phase changed?

    int flips=(thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1)<<1);
    int bayer_modified=FRAMEPAR_MODIFIED(P_BAYER) || FRAMEPAR_MODIFIED(P_FLIPH) || FRAMEPAR_MODIFIED(P_FLIPV) || FRAMEPAR_MODIFIED(P_MULTI_MODE);
    dev_dbg(g_dev_ptr,"{%d}  flips=%x\n",sensor_port, flips);

    if (thispars->pars[P_MULTI_MODE]) { // Modify flips in composite mode - should match flips of the top (direct) channel
        int this_bit=(1<<thispars->pars[P_MULTI_TOPSENSOR]);
        if (thispars->pars[P_MULTI_FLIPH] & this_bit) flips^=1;
        if (thispars->pars[P_MULTI_FLIPV] & this_bit) flips^=2;
        dev_dbg(g_dev_ptr,"{%d}  composite mode - adjusted flips=%x\n",sensor_port, flips);
        bayer_modified=  bayer_modified || FRAMEPAR_MODIFIED(P_MULTI_FLIPH) || FRAMEPAR_MODIFIED(P_MULTI_FLIPV)  || FRAMEPAR_MODIFIED(P_MULTI_TOPSENSOR);
    }
    if (bayer_modified) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_DCR0, X353_DCR0(BAYER_PHASE,thispars->pars[P_BAYER] ^ flips ^ sensor->bayer)); ///NOTE: hardware bayer
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int)X313_WA_DCR0, (int)X353_DCR0(BAYER_PHASE,thispars->pars[P_BAYER] ^ ((thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1)<<1)) ^ sensor->bayer) );

    }
    return 0;
#endif
}

/** Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
 * NC393 controls writing to memory. Also supports SINGLE and RESET */

int pgm_sensorrun  (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
    int reset_frame;
#ifndef NC353
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -EINVAL; // wrong frame
    reset_frame = ((prevpars->pars[P_SENSOR_RUN]==0) && (thispars->pars[P_SENSOR_RUN]!=0))? 1:0;
    if (thispars->pars[P_SENSOR_RUN] & 3) { // do nothing if stopped, set run/single accordingly
        control_sensor_memory (sensor_port,
                               thispars->pars[P_SENSOR_RUN] & 3,
                               reset_frame,
                               (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                                        frame16);
    }
    // Is it OK to process stop here too?
    return 0;
/*
     # Enable arbitration of sensor-to-memory controller
        if exit_step == 12: return False

        self.x393_axi_tasks.enable_memcntrl_en_dis(8 + num_sensor, True);

 */
#else
    int fpga_data=0;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    switch (thispars->pars[P_SENSOR_RUN] & 3) {
    case 1: fpga_data=4; break;
    case 2:
    case 3: fpga_data=5; break;
    }

    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    // only start/single, stopping will be handled by the pgm_sensorstop
    if (fpga_data) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_TRIG, fpga_data);
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_TRIG, (int) fpga_data);
    }
    return 0;
#endif
}

/** Stop acquisition from the sensor to the FPGA (start/single have latency of 2)
 * NC393 - currently same as  pgm_sensorrun*/
int pgm_sensorstop (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
#ifndef NC353
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -EINVAL; // wrong frame
    // Do we need to filter for stop only (    if ((thispars->pars[P_SENSOR_RUN] & 3)==0){... ) ?
    if ((thispars->pars[P_SENSOR_RUN] & 3)==0){ // do nothing if not stop
        control_sensor_memory (sensor_port,
                thispars->pars[P_SENSOR_RUN] & 3,
                0,
                (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                        frame16);
    }
    return 0;
#else
    int fpga_data=0;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    switch (thispars->pars[P_SENSOR_RUN] & 3) {
    case 1: fpga_data=4; break;
    case 2:
    case 3: fpga_data=5; break;
    }
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    // only start/single, stopping will be handled by the pgm_sensorstop
    if ((thispars->pars[P_SENSOR_RUN] & 3)==0){
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_TRIG, fpga_data);
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_TRIG, (int) fpga_data);
    }
    return 0;
#endif
}

/** Program gamma table
 * Table with the same hash should be available in cache. It is very unlikely
 * but is still possible that it can be pushed out - TODO: make it guaranteed. So normally new gamma table is 
 * set through a character device driver (with FPGA bit set to get locked?) and then pgm_gamma is activated when
 * the P_GTAB_R (*_G,*_GB, *_B) are updated
 * The scale part of these parameters (lower 16 bits) may be modified by white balancing code without loading a new table
 * Should not be? It would limit the output range if it is less than default 0x400
 *
 * will be programmed not earlier than 1 frame ahead of the current to prevent condition when earlier frame
 * is programmed after the later one (not possible because of just two tables in the FPGA - current and shadow,
 *  switching when the last table word is written */


int pgm_gamma      (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    int color, rslt;
    struct frameparspair_t pars_to_update[4]; // 4 needed
    int nupdate=0;
    struct {
        unsigned short scale;
        unsigned short hash16;
    } gamma32;
    unsigned long * pgamma32= (unsigned long *) & gamma32;
    // TODO: Add for multi-subchannel, for now using 0

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, (getThisFrameNumber() & PARS_FRAMES_MASK)= %ld\n",sensor_port,frame16, getThisFrameNumber(sensor_port) & PARS_FRAMES_MASK);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
/*
    MDF3(dev_dbg(g_dev_ptr," frame16=%d, thispars->pars[P_GTAB_*]=0x%lx 0x%lx 0x%lx 0x%lx, thispars->pars[P_FRAME]=0x%lx"
            " get_locked_hash32(*)=0x%lx 0x%lx 0x%lx 0x%lx\n",
            frame16, thispars->pars[P_GTAB_R],thispars->pars[P_GTAB_R+1],thispars->pars[P_GTAB_R+2],thispars->pars[P_GTAB_R+3],thispars->pars[P_FRAME],
            get_locked_hash32(0),get_locked_hash32(1),get_locked_hash32(2),get_locked_hash32(3)));
*/

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, thispars->pars[P_GTAB_*]=0x%lx 0x%lx 0x%lx 0x%lx, thispars->pars[P_FRAME]=0x%lx\n",
            sensor_port,frame16, thispars->pars[P_GTAB_R],thispars->pars[P_GTAB_R+1],thispars->pars[P_GTAB_R+2],
            thispars->pars[P_GTAB_R+3],thispars->pars[P_FRAME]);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    // Not needed - now it can be done in advance (just prepare cache). Later it will be done again and actually programmed (1 frame ahead of time)
    // return if too early
    // TODO: still calculate FPGA table , but not load it if too early?
    for (color=0; color<4; color++) {
        if (get_locked_hash32(color,sensor_port,0)!=thispars->pars[P_GTAB_R+color]) { // modified for this color
            *pgamma32=thispars->pars[P_GTAB_R+color];
            rslt=set_gamma_table (gamma32.hash16,
                    gamma32.scale, NULL,
                    GAMMA_MODE_HARDWARE,
                    color,
                    sensor_port,
                    0); // frame16 - one ahead of the current do not lock yet TODO 393 multisensor - split gamma tables to subchannels
            if (rslt<=0) SETFRAMEPARS_SET(P_GTAB_R+color, get_locked_hash32(color,sensor_port, 0)); // increases nupdate
        }
    }
    if (nupdate) {
        setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // restore failed components
        dev_dbg(g_dev_ptr,"{%d} had to restore back %d gamma tables (color components) \n",sensor_port,nupdate);
        MDP(DBGB_PSFN, sensor_port,"had to restore back %d gamma tables (color components) \n",nupdate)
        return -1;
    }
    return 0;
}



/** Program histogram window (absolute from relative and window size)
 * TODO: 393 - implement hardware interface, change to per-sensor
 */
int pgm_hist       (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    int sub_chn, poffs;
    int nupdate=0;
    x393_hist_left_top_t        left_top = {.d32=0};
    x393_hist_width_height_m1_t width_height = {.d32=0};
    struct {
        long left;
        long width;
        long top;
        long height;
    } hist_setup_data;
    struct frameparspair_t pars_to_update[4* MAX_SENSORS];
    /*
    struct frameparspair_t pars_to_update[4]={
            {P_HISTWND_LEFT, 0},
            {P_HISTWND_WIDTH, 0},
            {P_HISTWND_TOP, 0},
            {P_HISTWND_HEIGHT, 0}
    };
    */
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    for (sub_chn =0; sub_chn < MAX_SENSORS; sub_chn++)  if (GLOBALPARS(sensor_port, G_SUBCHANNELS) & (1 << sub_chn)){
        poffs = HIST_SUBCHN_OFFSET * sub_chn;
        //  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
        // calculate absolute window from the relative, apply limits
        hist_setup_data.width=  ((thispars->pars[P_HISTWND_RWIDTH + poffs] * thispars->pars[P_ACTUAL_WIDTH])>>16) & 0xffe;
        if (hist_setup_data.width<2) hist_setup_data.width=2;
        else if (hist_setup_data.width > thispars->pars[P_ACTUAL_WIDTH]) hist_setup_data.width = thispars->pars[P_ACTUAL_WIDTH];
        hist_setup_data.left=  ((thispars->pars[P_HISTWND_RLEFT + poffs] * (thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width)) >>16) & 0xffe;
        if (hist_setup_data.left> (thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width)) hist_setup_data.left = thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width;

        hist_setup_data.height=  ((thispars->pars[P_HISTWND_RHEIGHT + poffs] * thispars->pars[P_ACTUAL_HEIGHT])>>16) & 0xffe;
        if (hist_setup_data.height<2) hist_setup_data.height=2;
        else if (hist_setup_data.height > thispars->pars[P_ACTUAL_HEIGHT]) hist_setup_data.height = thispars->pars[P_ACTUAL_HEIGHT];
        hist_setup_data.top=  ((thispars->pars[P_HISTWND_RTOP + poffs] * (thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height)) >>16) & 0xffe;
        if (hist_setup_data.top > (thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height)) hist_setup_data.top = thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height;

        if ((hist_setup_data.left   != thispars->pars[P_HISTWND_LEFT + poffs]) ||
                (hist_setup_data.width  != thispars->pars[P_HISTWND_WIDTH + poffs]) ||
                (hist_setup_data.top    != thispars->pars[P_HISTWND_TOP + poffs]) ||
                (hist_setup_data.height != thispars->pars[P_HISTWND_HEIGHT + poffs])) {
            // set these values to FPGA
            left_top.left =          hist_setup_data.left;
            left_top.top =           hist_setup_data.top;
            width_height.width_m1 =  hist_setup_data.width-1;
            width_height.height_m1 = hist_setup_data.height-1;
            X393_SEQ_SEND1S (sensor_port, frame16, x393_histogram_lt, left_top, sub_chn);
            X393_SEQ_SEND1S (sensor_port, frame16, x393_histogram_wh, width_height,sub_chn);
            dev_dbg(g_dev_ptr,"{%d}   X393_SEQ_SEND1S(0x%x, 0x%x, x393_histogram_lt, 0x%x, %d)\n",
                    sensor_port, sensor_port, frame16, left_top.d32, sub_chn);
            MDP(DBGB_PADD, sensor_port,"   X393_SEQ_SEND1S(0x%x, 0x%x, x393_histogram_lt, 0x%x, %d)\n",
                    sensor_port, frame16, left_top.d32, sub_chn)

            dev_dbg(g_dev_ptr,"{%d}   X393_SEQ_SEND1S(0x%x, 0x%x, x393_histogram_wh, 0x%x, %d)\n",
                    sensor_port, sensor_port, frame16, width_height.d32, sub_chn);
            MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1S(0x%x, 0x%x, x393_histogram_wh, 0x%x, %d)\n",
                    sensor_port, frame16, width_height.d32, sub_chn)
            if ((nupdate == 0) || (HIST_SUBCHN_OFFSET > 0)) { // Update only once until there are per-subchannle parameters
                SETFRAMEPARS_SET(P_HISTWND_LEFT + poffs,hist_setup_data.left);
                SETFRAMEPARS_SET(P_HISTWND_WIDTH + poffs,hist_setup_data.width);
                SETFRAMEPARS_SET(P_HISTWND_TOP + poffs,hist_setup_data.top);
                SETFRAMEPARS_SET(P_HISTWND_HEIGHT + poffs,hist_setup_data.height);
            }

#ifdef NC353
            X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_LEFT,   hist_setup_data.left);
            X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_WIDTH,  hist_setup_data.width-2);
            X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_TOP,    hist_setup_data.top);
            X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_HEIGHT, hist_setup_data.height-2);
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_HIST_LEFT,   (int) hist_setup_data.left);
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_HIST_WIDTH,  (int) hist_setup_data.width-2);
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_HIST_TOP,    (int) hist_setup_data.top);
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_HIST_HEIGHT, (int) hist_setup_data.height-2);
            pars_to_update[0].val=hist_setup_data.left;
            pars_to_update[1].val=hist_setup_data.width;
            pars_to_update[2].val=hist_setup_data.top;
            pars_to_update[3].val=hist_setup_data.height;
            setFramePars(sensor_port, thispars, 4, pars_to_update);  // save intermediate/readonly parameters
#endif
        }
        setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save intermediate/readonly parameters
    }
    return 0;
}


/** Program autoexposure mode
 * TODO: Do something? */
int pgm_aexp       (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    //TODO:
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return 0;
}

/** Program quantization table(s)
 * TODO: 393: implement hardware interface */
int pgm_quality    (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    x393_cmprs_mode_t        cmprs_mode =        {.d32=0};
    int y_coring_index;
    int c_coring_index;
    int qtab = 0;
    int composite_quality=(thispars->pars[P_QUALITY] & 0xff7f) | ((thispars->pars[P_PORTRAIT] & 1)<<7);
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)

    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    //  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    if (thispars->pars[P_CORING_INDEX]!= prevpars->pars[P_CORING_INDEX]) {
        y_coring_index= thispars->pars[ P_CORING_INDEX]      & 0xffff;
        c_coring_index=(thispars->pars[ P_CORING_INDEX]>>16) & 0xffff;
        if (c_coring_index==0) c_coring_index=y_coring_index;
        set_coring_fpga(y_coring_index, 0, sensor_port);
        MDP(DBGB_PADD, sensor_port,"set_coring_fpga(%d, %d, %d)\n", y_coring_index, 0, sensor_port)
        set_coring_fpga(c_coring_index, 1, sensor_port);
        MDP(DBGB_PADD, sensor_port,"set_coring_fpga(%d, %d, %d)\n", c_coring_index, 1, sensor_port)

//TODO: Set coring index (it seems to be a pair?     void  set_x393_cmprs_coring_mode (x393_cmprs_coring_mode_t d, int cmprs_chn);     // Select coring mode
// Not needed, it is always downloaded to pair 0

    }

    // calculate quality tables - find already programmed FPGA page or calculates/programms a new one
    // set_qtable_fpga returns table page (0..7) or -1 - invalid q
    if ((qtab=set_qtable_fpga(composite_quality, sensor_port))>=0) {
        setFramePar(sensor_port, thispars, P_COMPMOD_QTAB,  qtab); // single parameter, when more - use SETFRAMEPARS_SET
        cmprs_mode.qbank = qtab;
        cmprs_mode.qbank_set = 1;
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
        dev_dbg(g_dev_ptr,"{%d}   X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x), qtab = %d\n",
                sensor_port, sensor_port, frame16, cmprs_mode.d32, qtab);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x), qtab = %d\n",
                sensor_port, frame16, cmprs_mode.d32, qtab)

        return 0;
    } else return -EFAULT;
}

/** Program memory channels 0 (sensor->memory) and 1 (memory->FPN)
 * TODO: 393 - replace with equivalent (mem_sensor) */
int pgm_memsensor      (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return OK - 0, <0 - error
{
#ifndef NC353
    int width_marg, height_marg, width_bursts;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    width_marg = thispars->pars[P_ACTUAL_WIDTH];
    height_marg = thispars->pars[P_ACTUAL_HEIGHT];
    switch(thispars->pars[P_COLOR]){
    case COLORMODE_COLOR:
    case COLORMODE_COLOR20:
        width_marg += (2 * COLOR_MARGINS);
        if ((thispars->pars[P_PF_HEIGHT] & 0xffff)==0) { // not a photofinish
            height_marg += (2 * COLOR_MARGINS);
        }
        break;
    }
    width_bursts = (width_marg >> 4) + ((width_marg & 0xf) ? 1 : 0);
    setup_sensor_memory (sensor_port,       // sensor port number (0..3)
                         width_bursts,      // 13-bit - in 8*16=128 bit bursts
                         height_marg,       // 16-bit window height (in scan lines)
                         0,                 // 13-bit window left margin in 8-bursts (16 bytes)
                         0,                 // 16-bit window top margin (in scan lines
                         (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                         frame16);          // Frame number the command should be applied to (if not immediate mode)
    return 0;

#else
    int ntilex,ntiley,goodEOL,padlen, imgsz,sa;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    //NC393 - use margins of 2 pixels from each side for color JPEG (even for JPEG18) to preserve Bayer
    ///programm channel1 (FPN). Will not enable if not needed (imageParamsR[P_FPN]==0)
    ntilex=((thispars->pars[P_ACTUAL_WIDTH]+(2 * COLOR_MARGINS)+7)>>3);
    ntiley=thispars->pars[P_ACTUAL_HEIGHT]+(((thispars->pars[P_PF_HEIGHT] & 0xffff)>0)?0:(2 * COLOR_MARGINS));
    dev_dbg(g_dev_ptr,"{%d} ntilex=0x%x ntiley=0x%x\n",sensor_port,ntilex,ntiley);
    if ((thispars->pars[P_PF_HEIGHT] & 0xffff)==0) { // not a photofinish
        if(!thispars->pars[P_BGFRAME] && ((thispars->pars[P_FPNS]!=0) || (thispars->pars[P_FPNM]!=0))) {
            // program memory channel1
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL1, X313_SDCHAN_REG1(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL2, X313_SDCHAN_REG2(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL0, X313_SDCHAN_REG0(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            // enable channel1 for reading SDRAM
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(1)); // wait ready later... ???
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH1_CTL1, (int) X313_SDCHAN_REG1(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH1_CTL2, (int) X313_SDCHAN_REG2(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH1_CTL0, (int) X313_SDCHAN_REG0(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(1));

        } else  {
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_DIS_D(1));
            dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_DIS_D(1));
        }
    } else  {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_DIS_D(1));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_DIS_D(1));
    }
    // Program channel 0 (sensor->memory)
    //       goodEOL=0; // last 8/16 blocks of pixels in each scanline are bad (only 2 are actually written)
    // if 8-bit mode we'll need to update ntilex. fpga tries to write 2 bytes more (but not crossing the page boundaries
    // GoodEOL - if image width is multiple of 512 pixels 1 extra block (16 pixels) needs to be written to memory (and padlen will be more by 512 bytes (256 words)
    // due to FPGA controller implementation (it writes extra 4 pixels for the margins, but not when it crosses 512 byte boundary)
    // When reading 20x20 macroblocks to the compressor, such exception is not needed, it crosses page boundaries when needed

    if ((thispars->pars[P_BITS]==8) && (!thispars->pars[P_BGFRAME])) { // in 16-bit mode ntilex will stay the same
        ntilex=((thispars->pars[P_ACTUAL_WIDTH]+(2 * COLOR_MARGINS)+15)>>4);
        goodEOL=((thispars->pars[P_ACTUAL_WIDTH] & 0x0f) == 0) && ((thispars->pars[P_ACTUAL_WIDTH] & 0x1f0) != 0);
        if (goodEOL) ntilex--;
        dev_dbg(g_dev_ptr,"{%d} ntilex=0x%x ntiley=0x%x goodEOL=0x%x\n",sensor_port,ntilex,ntiley,goodEOL);
    }
    dev_dbg(g_dev_ptr,"{%d} ntilex=0x%x ntiley=0x%x\n",sensor_port,ntilex,ntiley);

    //       if (imageParamsR[P_OVERLAP]>=(imageParamsR[P_ACTUAL_HEIGHT]+2)) imageParamsR[P_OVERLAP]=imageParamsR[P_ACTUAL_HEIGHT]+1; rotten code, left as a comment
    if (thispars->pars[P_OVERLAP]>0) ntiley=(ntiley<<1); // ntiley will be twice bigger for synch. mode)
    padlen=((ntilex+31)>>5) << 8;
    //TODO:fix it to be able to use two (or larger) frame buffer
    //  imgsz=((padlen * (thispars->pars[P_ACTUAL_HEIGHT]+(2 * COLOR_MARGINS)) * thispars->pars[P_PAGE_ACQ]) << ((thispars->pars(P_TRIG) & 1)?1:0)); // mostly rotten too
    imgsz=padlen * ntiley;
    dev_dbg(g_dev_ptr,"{%d} imgsz=0x%x, padlen=0x%x\n",sensor_port,imgsz,padlen);
    if (thispars->pars[P_IMGSZMEM]!= imgsz)  setFramePar(sensor_port, thispars, P_IMGSZMEM, imgsz);  // set it (and propagate to the later frames)
    sa=X313_MAP_FRAME + (imgsz * thispars->pars[P_PAGE_ACQ]);  // now - always X313_MAP_FRAME
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL1, X313_SDCHAN_REG1(0,1,1, sa, (ntilex-1), (ntiley-1)));
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH0_CTL1, (int) X313_SDCHAN_REG1(0,1,1, sa, (ntilex-1), (ntiley-1))  );
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL2, X313_SDCHAN_REG2(0,1,1, sa, (ntilex-1), (ntiley-1)));
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH0_CTL2, (int) X313_SDCHAN_REG2(0,1,1, sa, (ntilex-1), (ntiley-1))  );
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL0, X313_SDCHAN_REG0(0,1,1, sa, (ntilex-1), (ntiley-1))); // dependency - source
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH0_CTL0, (int) X313_SDCHAN_REG0(0,1,1, sa, (ntilex-1), (ntiley-1))  );
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(0));
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",v, fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(0)  );
    // number of scan lines to read from sensor - program in pgm_sensorin
    return 0;
#endif

}

/** Program memory channel 2 (memory->compressor)
 * Prepare data (memory, number of tiles) - it will be sent to FPGA in \b pgm_comprestart
 * Added re-read from memory option. If sensor is off - don't program dependency
 * TODO 393 - replace with equivalent
 */
int pgm_memcompressor  (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return OK - 0, <0 - error
{
    //TODO: redo for nc393
#ifndef NC353
    int width_marg, height_marg;
    int overlap = 0; // tile overlap (total - 2 for JPEG18, 4 - for JPEG20, 0 otherwise
    int width_bursts; // width in 16-pixel bursts
    int cmprs_top = 0; // 1 for JPEG18 only, 0 for others (also is used for compressor left)
    int tile_width; // in bursts, 2 for those with overlap (height>16), 4 with heigh==16
    int tile_height; // 16/18 (20 not yet implemented)
    x393_cmprs_frame_format_t cmprs_frame_format ={.d32=0};

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    width_marg = thispars->pars[P_ACTUAL_WIDTH];
    height_marg = thispars->pars[P_ACTUAL_HEIGHT];
    // NC393: maybe add later monochrome mode with small tiles?
    cmprs_frame_format.num_macro_cols_m1 = (width_marg>> 4) - 1; // before adding margins
    cmprs_frame_format.num_macro_rows_m1 = (height_marg>> 4) - 1; // before adding margins;


    switch(thispars->pars[P_COLOR]){
    case COLORMODE_COLOR:
        overlap = 2;
        cmprs_top = 1;
        break;
    case COLORMODE_COLOR20:
        overlap = 4;
        break;
    }
    if (overlap){
        width_marg += (2 * COLOR_MARGINS);
        if ((thispars->pars[P_PF_HEIGHT] & 0xffff)==0) { // not a photofinish
            height_marg += (2 * COLOR_MARGINS);
        }
        tile_width = 2;
    } else {
        tile_width = 4;
    }
    cmprs_frame_format.left_margin = cmprs_top; // same as top - only for 18x18 tiles to keep Bayer shift (0/1)

    width_bursts = (width_marg >> 4) + ((width_marg & 0xf) ? 1 : 0);
    // Adjusting for tile width. TODO: probably not needed, handled in FPGA - verify (and remove 2 next lines)
    if (width_bursts & 1)                     width_bursts++;
    if ((tile_width>2) && (width_bursts & 2)) width_bursts += 2;

    tile_height = 16 + overlap;
    setup_compressor_memory (sensor_port,       // sensor port number (0..3)
                             width_bursts,      // 13-bit - in 8*16=128 bit bursts
                             (cmprs_frame_format.num_macro_rows_m1 + 1) << 4,
//                             height_marg,       // 16-bit window height (in scan lines)
                             0,                 // 13-bit window left margin in 8-bursts (16 bytes)
                             cmprs_top,         // 16-bit window top margin (in scan lines
                             tile_width,        // tile width in bjursts (16-pixels each)
                             tile_height,       // tile height: 18 for color JPEG, 16 for JP4 flavors // = 18
                             16,                // tile vertical step in pixel rows (JPEG18/jp4 = 16) // = 16
                             (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                             frame16);          // Frame number the command should be applied to (if not immediate mode)

    X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_format, cmprs_frame_format);
    return 0;
// TODO: Do we need to maintain P_IMGSZMEM ?
// #define P_PAGE_ACQ      18 ///< Number of image page buffer to acquire to (0.1?)
// #define P_PAGE_READ     19 ///< Number of image page buffer to read from to (0.1?)

#else

    int ntilex,ntiley,sa,pf;
    //  struct frameparspair_t * pars_to_update[4]={
    struct frameparspair_t pars_to_update[4]={
            {P_SDRAM_CHN20, 0},
            {P_SDRAM_CHN21, 0},
            {P_SDRAM_CHN22, 0},
            {P_TILES, 0}
    };
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    //  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    ntilex=((thispars->pars[P_ACTUAL_WIDTH]+(2 * COLOR_MARGINS)-1)>>4);
    ntiley=thispars->pars[P_ACTUAL_HEIGHT]; // number of lines it the whole frame
    sa=X313_MAP_FRAME + ( thispars->pars[P_IMGSZMEM] * thispars->pars[P_PAGE_READ]); // now - always X313_MAP_FRAME
    pf=((thispars->pars[P_PF_HEIGHT] & 0xffff)>0)?1:0; // when mode==1, wnr means "photofinish" in fpga
    int depend=((thispars->pars[P_SENSOR_RUN] & 3)==SENSOR_RUN_STOP) ? 0 : 1;
    dev_dbg(g_dev_ptr,"{%d} ntilex=0x%x ntiley=0x%x sa=0x%x pf=0x%x depend=%x\n",sensor_port,ntilex,ntiley,sa,pf,depend);
    // will be programmed with "depend==1", so it will not be possible to re-read from memory this way
    dev_dbg(g_dev_ptr,"{%d}  thispars->pars[P_SENSOR_RUN]=0x%x,  depend=%d)\n",sensor_port, (int)thispars->pars[P_SENSOR_RUN], depend);
    pars_to_update[1].val=X313_SDCHAN_REG1(1,pf,depend, sa, (ntilex-1), (ntiley-16));
    pars_to_update[2].val=X313_SDCHAN_REG2(1,pf,depend, sa, (ntilex-1), (ntiley-16));
    pars_to_update[0].val=X313_SDCHAN_REG0(1,pf,depend, sa, (ntilex-1), (ntiley-16));
    pars_to_update[3].val=ntilex*(ntiley>>4);

    setFramePars(sensor_port, thispars, 4, pars_to_update);
    return 0;
#endif
}


/** Program compressor modes (does not start/stop)
 * will also program Huffman table to the FPGA if it was not programmed yet */
int pgm_compmode   (int sensor_port,               ///< sensor port number (0..3)
                    struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                    struct framepars_t * thispars, ///< sensor current parameters
                    struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                    int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                   ///< be applied to,  negative - ASAP
                                                   ///< @return OK - 0, <0 - error
{
    struct frameparspair_t pars_to_update[4]; // 2 needed, increase if more entries will be added
    int nupdate=0;
    int csb;
    int csr;
#ifndef NC353
//    x393cmd_t x393cmd;
    x393_cmprs_mode_t        cmprs_mode =        {.d32=0};
    x393_cmprs_colorsat_t    cmprs_colorsat =    {.d32=0};
    x393_cmprs_coring_mode_t cmprs_coring_mode = {.d32=0};
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (!jpeg_htable_is_programmed(sensor_port)) jpeg_htable_fpga_pgm (sensor_port);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//    x393cmd = (frame16<0)? ASAP: ABSOLUTE;
    if (FRAMEPAR_MODIFIED(P_COLOR)) {
        switch (thispars->pars[P_COLOR] & 0x0f){
        case COLORMODE_MONO6:    cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_MONO6;          break;
        case COLORMODE_COLOR:    cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JPEG18;         break;
        case COLORMODE_JP46:     cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP46;           break;
        case COLORMODE_JP46DC:   cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP46DC;         break;
        case COLORMODE_COLOR20:  cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JPEG20;         break; //NOT implemented
        case COLORMODE_JP4:      cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4;            break;
        case COLORMODE_JP4DC:    cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4DC;          break;
        case COLORMODE_JP4DIFF:  cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4DIFF;        break;
        case COLORMODE_JP4HDR:   cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4DIFFHDR;     break;
        case COLORMODE_JP4DIFF2: cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4DIFFDIV2;    break;
        case COLORMODE_JP4HDR2:  cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_JP4DIFFHDRDIV2; break;
        case COLORMODE_MONO4:    cmprs_mode.cmode = X393_CMPRS_CBIT_CMODE_MONO4;          break;
        }
        cmprs_mode.cmode_set = 1;
        // TODO: Modify left margin by 1 for COLORMODE_COLOR !
    }
    // Bayer shift changed? (additional bayer shift, separate from the gamma-tables one)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_BYRSH) || FRAMEPAR_MODIFIED(P_COMP_BAYER)) {
        cmprs_mode.bayer =     thispars->pars[P_COMPMOD_BYRSH] ^ thispars->pars[P_COMP_BAYER];
        cmprs_mode.bayer_set = 1;
    }
#if 0
    // Tile shift changed? (position of the 16x16, 18x8 or 20x20 inside 20x20 overlapping tile - dx==dy (diagonal), 0..4)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_TILSH)) {
        comp_cmd |= COMPCMD_TILESHIFT(thispars->pars[P_COMPMOD_TILSH]);
    }
#endif
    // DC subtraction modse changed? (mostly FPGA debug feature, normally should be on - average block level to bypass DCT conversion)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_DCSUB)) {
        cmprs_mode.dcsub =     thispars->pars[P_COMPMOD_DCSUB];
        cmprs_mode.dcsub_set = 1;
    }

    // Did focus show mode change? (do it here, not with other focus parameters that can not be set through the sequencer (writing tables
    // could break writing gamma/quantization/whatever tables . Is it applicable to NC393?
    if (FRAMEPAR_MODIFIED(P_FOCUS_SHOW)) {
        cmprs_mode.focus =     thispars->pars[P_FOCUS_SHOW];
        cmprs_mode.focus_set = 1;
    }

    if (cmprs_mode.d32) {
        // Consider frames_in_buffer_minus_one() to be static (it can be actually changed over sysfs),
        // Set it always but do not count as modified
        cmprs_mode.multiframe = (frames_in_buffer_minus_one(sensor_port)>0)? 1:0;
        cmprs_mode.multiframe_set = 1;
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
        dev_dbg(g_dev_ptr,"{%d}   X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",
                sensor_port, sensor_port, frame16, cmprs_mode.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",
                sensor_port, frame16, cmprs_mode.d32)

    } else {
        dev_dbg(g_dev_ptr,"{%d}   cmprs_mode.d32==0, does not need to be sent\n",sensor_port);
        MDP(DBGB_PADD, sensor_port,"cmprs_mode.d32=0x%x, does not need to be sent\n",cmprs_mode.d32)
    }
    // color saturation changed?
    if (FRAMEPAR_MODIFIED(P_COLOR_SATURATION_BLUE) || FRAMEPAR_MODIFIED(P_COLOR_SATURATION_RED)) {
        csb=(thispars->pars[P_COLOR_SATURATION_BLUE]* DEFAULT_COLOR_SATURATION_BLUE)/100;
        csr=(thispars->pars[P_COLOR_SATURATION_RED] * DEFAULT_COLOR_SATURATION_RED)/100;
        if (unlikely(csb>1023)) {
            csb=102300/DEFAULT_COLOR_SATURATION_BLUE;
            SETFRAMEPARS_SET(P_COLOR_SATURATION_BLUE, csb);
        }
        if (unlikely(csr>1023)) {
            csr=102300/DEFAULT_COLOR_SATURATION_RED;
            SETFRAMEPARS_SET(P_COLOR_SATURATION_RED, csr);
        }
        cmprs_colorsat.colorsat_blue = csb;
        cmprs_colorsat.colorsat_red =  csr;
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_color_saturation, cmprs_colorsat);
        dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_cmprs_color_saturation, 0x%x)\n",
                sensor_port, sensor_port, frame16, cmprs_colorsat.d32);
        MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x,  0x%x, x393_cmprs_color_saturation, 0x%x)\n",
                sensor_port, frame16, cmprs_colorsat.d32)
    }
    // compressor quantizer zero bin mode changed?
    // Quantizer tuning - bits 0..7 - zero bin, 15:8 - quantizer bias
    if (FRAMEPAR_MODIFIED(P_CORING_PAGE)) {
        cmprs_coring_mode.coring_table = thispars->pars[P_CORING_PAGE];
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_coring_mode, cmprs_coring_mode);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x, x393_cmprs_coring_mode,  0x%x)\n",
                sensor_port, sensor_port, frame16, cmprs_coring_mode.d32);
        MDP(DBGB_PADD, sensor_port,"X3X3_SEQ_SEND1(0x%x,  0x%x, x393_cmprs_coring_mode,  0x%x)\n",
                sensor_port, frame16, cmprs_coring_mode.d32)
    }

    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions


#else
    int comp_cmd=0;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (!jpeg_htable_is_programmed(sensor_port)) jpeg_htable_fpga_pgm (sensor_port);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    // QTAB is programmed separately
    // demosaic mode - is it changed?
    if (FRAMEPAR_MODIFIED(P_COLOR)) {
        switch (thispars->pars[P_COLOR] & 0x0f){
        case COLORMODE_MONO6:    comp_cmd |= COMPCMD_DEMOS(DEMOS_MONO6);    break;
        case COLORMODE_COLOR:    comp_cmd |= COMPCMD_DEMOS(DEMOS_COLOR18);  break;
        case COLORMODE_JP46:     comp_cmd |= COMPCMD_DEMOS(DEMOS_JP46);     break;
        case COLORMODE_JP46DC:   comp_cmd |= COMPCMD_DEMOS(DEMOS_JP46DC);   break;
        case COLORMODE_COLOR20:  comp_cmd |= COMPCMD_DEMOS(DEMOS_COLOR20);  break;
        case COLORMODE_JP4:      comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4);      break;
        case COLORMODE_JP4DC:    comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4DC);    break;
        case COLORMODE_JP4DIFF:  comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4DIFF);  break;
        case COLORMODE_JP4HDR:   comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4HDR);   break;
        case COLORMODE_JP4DIFF2: comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4DIFF2); break;
        case COLORMODE_JP4HDR2:  comp_cmd |= COMPCMD_DEMOS(DEMOS_JP4HDR2);  break;
        case COLORMODE_MONO4:    comp_cmd |= COMPCMD_DEMOS(DEMOS_MONO4);    break;
        }
    }
// TODO: Redo for NC393
    dev_dbg(g_dev_ptr,"{%d} comp_cmd=0x%x\n",sensor_port,comp_cmd);
    // Bayer shift changed? (additional bayer shift, separate from the gamma-tables one)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_BYRSH)) {
        comp_cmd |= COMPCMD_BAYERSHIFT(thispars->pars[P_COMPMOD_BYRSH]);
    }

    // Tile shift changed? (position of the 16x16, 18x8 or 20x20 inside 20x20 overlapping tile - dx==dy (diagonal), 0..4)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_TILSH)) {
        comp_cmd |= COMPCMD_TILESHIFT(thispars->pars[P_COMPMOD_TILSH]);
    }

    // DC subtraction modse changed? (mostly FPGA debug feature, normally should be on - average block level to bypass DCT conversion)
    if (FRAMEPAR_MODIFIED(P_COMPMOD_DCSUB)) {
        comp_cmd |= COMPCMD_DCSUB(thispars->pars[P_COMPMOD_DCSUB]);
    }

    // Did focus show mode change? (do it here, not with other focus parameters that can not be set through the sequencer (writing tables
    // could break writing gamma/quntization/whatever tables
    if (FRAMEPAR_MODIFIED(P_FOCUS_SHOW)) {
        comp_cmd |= COMPCMD_FOCUS(thispars->pars[ P_FOCUS_SHOW]);
    }

    // enqueue it for the compressor
    if (comp_cmd) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, comp_cmd);
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_COMP_CMD, (int) comp_cmd);
    } else {
        dev_dbg(g_dev_ptr,"{%d}   comp_cmd==0, does not need to be sent\n",sensor_port);
    }
    // color saturation changed?
    if (FRAMEPAR_MODIFIED(P_COLOR_SATURATION_BLUE) || FRAMEPAR_MODIFIED(P_COLOR_SATURATION_RED)) {
        csb=(thispars->pars[P_COLOR_SATURATION_BLUE]* DEFAULT_COLOR_SATURATION_BLUE)/100;
        csr=(thispars->pars[P_COLOR_SATURATION_RED] * DEFAULT_COLOR_SATURATION_RED)/100;
        if (unlikely(csb>1023)) {
            csb=102300/DEFAULT_COLOR_SATURATION_BLUE;
            SETFRAMEPARS_SET(P_COLOR_SATURATION_BLUE, csb);
        }
        if (unlikely(csr>1023)) {
            csr=102300/DEFAULT_COLOR_SATURATION_RED;
            SETFRAMEPARS_SET(P_COLOR_SATURATION_RED, csr);
        }
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COLOR_SAT, ((DEFAULT_COLOR_SATURATION_BLUE*thispars->pars[P_COLOR_SATURATION_BLUE])/100) |
                (((DEFAULT_COLOR_SATURATION_RED *thispars->pars[P_COLOR_SATURATION_RED])/100)<<12));
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%lx)\n",sensor_port, (int)fpga_addr, (int) X313_WA_COLOR_SAT, (int)((DEFAULT_COLOR_SATURATION_BLUE*thispars->pars[P_COLOR_SATURATION_BLUE])/100) | (((DEFAULT_COLOR_SATURATION_RED *thispars->pars[P_COLOR_SATURATION_RED])/100)<<12));


    }
    // compressor quantizer zero bin mode changed?
    // Quantizer tuning - bits 0..7 - zero bin, 15:8 - quantizer bias
    if (FRAMEPAR_MODIFIED(P_CORING_PAGE)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_QUANTIZER_MODE,thispars->pars[P_CORING_PAGE]);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_QUANTIZER_MODE, (int)thispars->pars[P_CORING_PAGE]);
    }

    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
#endif
    return 0;
}

/** Program focus modes (through writing the tables, so no sequencer)
 *  TODO: update it only when focus is enabled? not to recalculate it when not needed?
 */
int pgm_focusmode  (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
//    unsigned long flags;
//    int i;
//    x393_cmprs_table_addr_t table_addr;
    struct {
        short left;
        short right;
        short top;
        short bottom;
        short totalwidth;
        short filter_no;
        short show1;
    } focus_setup_data;
    u32 * focus_setup_data32 = (u32*) &focus_setup_data;
    //  struct frameparspair_t * pars_to_update[5]={
    struct frameparspair_t pars_to_update[5]={
            {P_FOCUS_TOTWIDTH, 0},
            {P_FOCUS_LEFT, 0},
            {P_FOCUS_WIDTH, 0},
            {P_FOCUS_TOP, 0},
            {P_FOCUS_HEIGHT, 0}
    };
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= 0) return -1; // now can only programm in immediate mode by writing the table
    focus_setup_data.totalwidth=(thispars->pars[P_ACTUAL_WIDTH]& 0xfff0) -0x10; // anyway should be 16x
    focus_setup_data.show1=       thispars->pars[P_FOCUS_SHOW1];
    // calculate absolute window from the relative, apply limits
    focus_setup_data.left=  ((((thispars->pars[P_RFOCUS_LEFT] * (0x10000-thispars->pars[P_RFOCUS_WIDTH])) >>16)* thispars->pars[P_ACTUAL_WIDTH])>>16) & 0xff8;
    focus_setup_data.right=(focus_setup_data.left+((thispars->pars[P_RFOCUS_WIDTH]*thispars->pars[P_ACTUAL_WIDTH])>>16) -8);
    if (focus_setup_data.right<0) {
        focus_setup_data.right=0;
        focus_setup_data.left=8;
    } else if (focus_setup_data.right >  0xfff) focus_setup_data.right = 0xfff;
    focus_setup_data.right &=0xff8;
    focus_setup_data.top=  ((((thispars->pars[P_RFOCUS_TOP] * (0x10000-thispars->pars[P_RFOCUS_HEIGHT])) >>16)* thispars->pars[P_ACTUAL_HEIGHT])>>16) & 0xff8;
    focus_setup_data.bottom=(focus_setup_data.top+((thispars->pars[P_RFOCUS_HEIGHT]*thispars->pars[P_ACTUAL_HEIGHT])>>16) -8);
    if (focus_setup_data.bottom<0) {
        focus_setup_data.bottom=0;
        focus_setup_data.top=8;
    } else if (focus_setup_data.bottom >  0xfff) focus_setup_data.bottom = 0xfff;
    focus_setup_data.bottom &=0xff8;
    focus_setup_data.filter_no=thispars->pars[P_FOCUS_FILTER];
    if   (focus_setup_data.filter_no > 14) focus_setup_data.filter_no=14;
    if ((focus_setup_data.totalwidth!=thispars->pars[P_FOCUS_TOTWIDTH]) ||
            (focus_setup_data.left   != thispars->pars[P_FOCUS_LEFT]) ||
            (focus_setup_data.right  != (focus_setup_data.left+thispars->pars[P_FOCUS_WIDTH] -8)) ||
            (focus_setup_data.top    != thispars->pars[P_FOCUS_TOP]) ||
            (focus_setup_data.bottom != (focus_setup_data.top+thispars->pars[P_FOCUS_HEIGHT] -8)) ||
            FRAMEPAR_MODIFIED(P_FOCUS_FILTER) ||
            FRAMEPAR_MODIFIED(P_FOCUS_SHOW1) ) {
        // TODO: NC393 Focus functionality requires userland write tables (was in fpga_io.c). Other functions
        // from that file are replaced, but we still need table write with disabling IRQ

#ifndef NC353
    #if 0
        table_addr.type =   X393_TABLE_FOCUS_TYPE;
        // Each focus page has 64 of 16-bit entries, total 16 pages (2KB). Configuration uses first 8 of 16-bit words in last page,
        // And FPGA accepts 32-bit data (16-bit ones merged in pairs). So address is 32*15
        table_addr.addr32 = 32*15;

        //focus_setup_data32

        local_ irq_save(flags);
        x393_cmprs_tables_address(table_addr, sensor_port);
        for (i = 0; i < 4; i++) {
            x393_cmprs_tables_data(focus_setup_data32[i], sensor_port);
        }
        local_ irq_restore(flags);
    #endif
        write_compressor_table(sensor_port,
                               TABLE_TYPE_FOCUS,
                               8*15, // to adress short (4-dwords) instead of full(32-dwords) page #15, multiply by 8
                               4,    // dwords to write
                               focus_setup_data32);

//        print_hex_dump_bytes("", DUMP_PREFIX_NONE, &focus_setup_data32[0], sizeof (focus_setup_data));
        MDP(DBGB_PADD, sensor_port,"focus_setup_data left=%d, right=%d, top=%d, bottom=%d, total width=%d, filter_no=%d, show1=%d\n",
                focus_setup_data.left,focus_setup_data.right,focus_setup_data.top,focus_setup_data.bottom,
                focus_setup_data.totalwidth,focus_setup_data.filter_no,focus_setup_data.show1 )
#else
        fpga_table_write_nice (CX313_FPGA_TABLES_FOCUSPARS, sizeof(focus_setup_data)/sizeof(focus_setup_data.left), (unsigned long *) &focus_setup_data);
#endif
        pars_to_update[0].val=focus_setup_data.totalwidth;
        pars_to_update[1].val=focus_setup_data.left;
        pars_to_update[2].val=focus_setup_data.right-focus_setup_data.left+8;
        pars_to_update[3].val=focus_setup_data.top;
        pars_to_update[4].val=focus_setup_data.bottom-focus_setup_data.top+8;

        setFramePars(sensor_port, thispars, 5, pars_to_update);  // save intermediate/readonly parameters
    }
    return 0;
}
/** Program trigger generator/external synchronization
 * TODO: 393 reimplement
 * Was for 353: can not use sequencer as data is more than 24 bit wide
 * In NC393  P_TRIG_DELAY and P_XMIT_TIMESTAMP are per-channel, other parameters are common (Last modified takes control).
 * Master channel is set to the current channels when any of the common parameters is set
 * P_TRIG_OUT (outputs):
 * off:      0x55555
 * external: 0x56555
 * internal: 0x65555
 * both:     0x66555
 * P_TRIG_IN (inputs):
 * off:      0x55555
 * external: 0x95555
 * internal: 0x59555
 * both:     0x99555
 */
int pgm_trigseq    (int sensor_port,               ///< sensor port number (0..3)
                    struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                    struct framepars_t * thispars, ///< sensor current parameters
                    struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                    int frame16)                   ///< 4-bit (hardware) frame number parameters should be applied to,  negative - ASAP
                                       ///< @return OK - 0, <0 - error
{
    struct frameparspair_t pars_to_update[10]; // ??? needed, increase if more entries will be added - just one
    int nupdate=0;
    int d;
#ifndef NC353
    x393_camsync_io_t camsync_src =      {.d32=0};
    x393_camsync_io_t camsync_dst =      {.d32=0};
    x393_gpio_set_pins_t gpio_set_pins = {.d32=0};
    x393_camsync_mode_t camsync_mode =   {.d32=0};
    int update_master_channel = 0; // set if any of the common (not channel-specific) parameters is modified
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
//    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",sensor_port, frame16, cmprs_mode.d32);

    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (frame16 >= 0) return -1; // ASAP only mode
    // Trigger condition changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_TRIG_CONDITION)) {
        camsync_src.d32 =  thispars->pars[P_TRIG_CONDITION];
        x393_camsync_trig_src(camsync_src);
        update_master_channel=1;
        dev_dbg(g_dev_ptr,"{%d}   x393_camsync_trig_src(0x%x)\n",sensor_port,  camsync_src.d32);
        MDP(DBGB_PADD, sensor_port,"x393_camsync_trig_src(0x%x)\n", camsync_src.d32)
    }
    // Trigger delay changed?
    if (FRAMEPAR_MODIFIED(P_TRIG_DELAY)) { // individual per-channel parameters
        set_x393_camsync_trig_delay  (thispars->pars[P_TRIG_DELAY], sensor_port); // CAMSYNC trigger delay
        dev_dbg(g_dev_ptr,"{%d}   set_x393_camsync_trig_delay(0x%x, %d)\n",sensor_port,  camsync_src.d32, sensor_port);
        MDP(DBGB_PADD, sensor_port,"set_x393_camsync_trig_delay(0x%x, %d)\n", camsync_src.d32, sensor_port)
    }
    // Sequencer output word changed? (to which outputs it is sent and what polarity)
    if (FRAMEPAR_MODIFIED(P_TRIG_OUT)) {
        camsync_dst.d32 =  thispars->pars[P_TRIG_OUT];
        x393_camsync_trig_dst(camsync_dst);
        update_master_channel=1;
        dev_dbg(g_dev_ptr,"{%d}   x393_camsync_trig_dst(0x%x)\n",sensor_port,  camsync_dst.d32);
        MDP(DBGB_PADD, sensor_port,"x393_camsync_trig_dst(0x%x)\n", camsync_dst.d32)

//        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCOUT, (int) thispars->pars[P_TRIG_OUT]);
        // Enable connection from the trigger module to the FPGA GPIO pins

        if (thispars->pars[P_TRIG_OUT]!=0) {
            gpio_set_pins.chn_a = 3;  // Set dibit enable
            x393_gpio_set_pins(gpio_set_pins);
            dev_dbg(g_dev_ptr,"{%d}   x393_gpio_set_pins(0x%x)\n",sensor_port,  gpio_set_pins.d32);
            MDP(DBGB_PADD, sensor_port,"x393_gpio_set_pins(0x%x)\n",gpio_set_pins.d32)
        } else {
            //  Not needed, I think
            //      port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_DIS_TRIG_OUT;
            //      dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_IOPINS, (int) X313_WA_IOPINS_DIS_TRIG_OUT);
        }
    }
    // Sequencer period changed? (0 - stopped, 1 - single trigger, >=256 - start repetitive)
    if (FRAMEPAR_MODIFIED(P_TRIG_PERIOD)) {
        if (unlikely((thispars->pars[P_TRIG_PERIOD] > 1) && (thispars->pars[P_TRIG_PERIOD] < 256))) { // Wrong value, restore old one
            SETFRAMEPARS_SET(P_TRIG_PERIOD,prevpars->pars[P_TRIG_PERIOD]); //+1
        } else {
            set_x393_camsync_trig_period(thispars->pars[P_TRIG_PERIOD]);
            update_master_channel=1;
            dev_dbg(g_dev_ptr,"{%d}   set_x393_camsync_trig_period(0x%lx)\n",sensor_port, thispars->pars[P_TRIG_PERIOD]);
            MDP(DBGB_PADD, sensor_port,"set_x393_camsync_trig_period(0x%lx)\n", thispars->pars[P_TRIG_PERIOD])
        }
    }
    // Bit length changed or not yet initialized?
    if (FRAMEPAR_MODIFIED(P_TRIG_BITLENGTH) || (thispars->pars[P_TRIG_BITLENGTH]==0)) {
        d=thispars->pars[P_TRIG_BITLENGTH];
        if (unlikely((d<2) || (d>255))) { // Wrong value, restore old one
            d=P_TRIG_BITLENGTH_DEFAULT;
            SETFRAMEPARS_SET(P_TRIG_BITLENGTH,d);
        }
        set_x393_camsync_trig_period(d);
        update_master_channel=1;
        dev_dbg(g_dev_ptr,"{%d}   set_x393_camsync_trig_period(0x%x) (bit length)\n",sensor_port, d);
        MDP(DBGB_PADD, sensor_port,"set_x393_camsync_trig_period(0x%x) (bit length)\n",d)
    }
    // P_EXTERN_TIMESTAMP changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_EXTERN_TIMESTAMP)) {
        camsync_mode.ext = thispars->pars[P_EXTERN_TIMESTAMP]?1:0;
        camsync_mode.ext_set = 1;
        update_master_channel=1;
    }
    // P_XMIT_TIMESTAMP changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_XMIT_TIMESTAMP)) {
        camsync_mode.ts_chns =     (thispars->pars[P_EXTERN_TIMESTAMP]?1:0) << sensor_port;
        camsync_mode.ts_chns_set = 1 << sensor_port;
    }
    if (update_master_channel){
        camsync_mode.master_chn = sensor_port;
        camsync_mode.master_chn_set = 1;
    }
    if (camsync_mode.d32){ // anything set?
        x393_camsync_mode (camsync_mode);
        dev_dbg(g_dev_ptr,"{%d}   x393_camsync_mode(0x%x)\n",sensor_port,  camsync_mode.d32);
        MDP(DBGB_PADD, sensor_port,"x393_camsync_mode(0x%x)\n", camsync_mode.d32)
    }
    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    return 0;
#else
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    if (frame16 >= 0) return -1; // ASAP only mode
    // Trigger condition changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_TRIG_CONDITION)) {
        port_csp0_addr[X313_WA_CAMSYNCTRIG] = thispars->pars[P_TRIG_CONDITION];
        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCTRIG, (int)thispars->pars[P_TRIG_CONDITION]);
    }
    // Trigger delay changed?
    if (FRAMEPAR_MODIFIED(P_TRIG_DELAY)) {
        port_csp0_addr[X313_WA_CAMSYNCDLY] = thispars->pars[P_TRIG_DELAY];
        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCDLY, (int) thispars->pars[P_TRIG_DELAY]);
    }
    // Sequencer output word changed? (to which outputs it is sent and what polarity)
    if (FRAMEPAR_MODIFIED(P_TRIG_OUT)) {
        port_csp0_addr[X313_WA_CAMSYNCOUT] = thispars->pars[P_TRIG_OUT];
        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCOUT, (int) thispars->pars[P_TRIG_OUT]);
        // Enable connection from the trigger module to the FPGA GPIO pins
        if (thispars->pars[P_TRIG_OUT]!=0) {
            port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_EN_TRIG_OUT;
            dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_IOPINS, (int) X313_WA_IOPINS_EN_TRIG_OUT);
        } else {
            //  Not needed, I think
            //      port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_DIS_TRIG_OUT;
            //      dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_IOPINS, (int) X313_WA_IOPINS_DIS_TRIG_OUT);
        }
    }
    // Sequencer period changed? (0 - stopped, 1 - single trigger, >=256 - start repetitive)
    if (FRAMEPAR_MODIFIED(P_TRIG_PERIOD)) {
        if (unlikely((thispars->pars[P_TRIG_PERIOD] > 1) && (thispars->pars[P_TRIG_PERIOD] < 256))) { // Wrong value, restore old one
            SETFRAMEPARS_SET(P_TRIG_PERIOD,prevpars->pars[P_TRIG_PERIOD]);
        } else {
            port_csp0_addr[X313_WA_CAMSYNCPER] = thispars->pars[P_TRIG_PERIOD];
            dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCPER, (int)thispars->pars[P_TRIG_PERIOD]);
        }
    }
    // Bit length changed or not yet initialized?
    if (FRAMEPAR_MODIFIED(P_TRIG_BITLENGTH) || (thispars->pars[P_TRIG_BITLENGTH]==0)) {
        d=thispars->pars[P_TRIG_BITLENGTH];
        if (unlikely((d<2) || (d>255))) { // Wrong value, restore old one
            d=P_TRIG_BITLENGTH_DEFAULT;
            SETFRAMEPARS_SET(P_TRIG_BITLENGTH,d);
        }
        port_csp0_addr[X313_WA_CAMSYNCPER] = d;
        dev_dbg(g_dev_ptr,"{%d} writing bit length-1:  port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_CAMSYNCPER, d);
    }
    // P_EXTERN_TIMESTAMP changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_EXTERN_TIMESTAMP)) {
        port_csp0_addr[X313_WA_DCR1]=X353_DCR1(EXTERNALTS,thispars->pars[P_EXTERN_TIMESTAMP]?1:0);
        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_DCR1, (int)X353_DCR1(EXTERNALTS,thispars->pars[P_EXTERN_TIMESTAMP]?1:0));
    }
    // P_XMIT_TIMESTAMP changed? (0 - internal sequencer)
    if (FRAMEPAR_MODIFIED(P_XMIT_TIMESTAMP)) {
        port_csp0_addr[X313_WA_DCR1]=X353_DCR1(OUTPUTTS,thispars->pars[P_XMIT_TIMESTAMP]?1:0);
        dev_dbg(g_dev_ptr,"{%d}   port_csp0_addr[0x%x]=0x%x\n",sensor_port,  (int) X313_WA_DCR1, (int)X353_DCR1(OUTPUTTS,thispars->pars[P_XMIT_TIMESTAMP]?1:0));
    }
    if (nupdate)  setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
    return 0;
#endif
}

/** Program smart IRQ mode (needs to be on, at least bit 0)
 * "smart" IRQ modes: +1 - wait for VACT in early compressor_done, +2 - wait for dma fifo ready
 * TODO: 393 - replace with what?  */
int pgm_irq    (int sensor_port,               ///< sensor port number (0..3)
				struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
				struct framepars_t * thispars, ///< sensor current parameters
				struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
				int frame16)                   ///< 4-bit (hardware) frame number parameters should
											   ///< be applied to,  negative - ASAP
											   ///< @return OK - 0, <0 - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d, does nothing\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    /** temporary make "smart" IRQ always enabled (otherwise FPGA bug fix needed)
  X3X3_SEQ_SEND1(fpga_addr,   X313_WA_SMART_IRQ,  (2 | ((thispars->pars[P_IRQ_SMART] & 1)?1:0)) | \
                                                  (8 | ((thispars->pars[P_IRQ_SMART] & 2)?4:0)));
     */

    X3X3_SEQ_SEND1(fpga_addr,   X313_WA_SMART_IRQ,  (2 | ((thispars->pars[P_IRQ_SMART] & 1)?1:0)) | \
            (8 | ((thispars->pars[P_IRQ_SMART] & 2)?4:0)));
#endif
    /*
    MDF3(dev_dbg(g_dev_ptr,"  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SMART_IRQ, (int) ( (2 | ((thispars->pars[P_IRQ_SMART] & 1)?1:0)) | \
            (8 | ((thispars->pars[P_IRQ_SMART] & 2)?4:0)))));
            */
    return 0;
}

/** Recalculate sequences/latencies, according to P_SKIP, P_TRIG */
int pgm_recalcseq  (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    int safe= thispars->pars[P_SKIP_FRAMES]?1:0; //TODO: modify storage of async mode, safe
    int async=(thispars->pars[P_TRIG] & 4)?1:0;
    int nooverlap=(thispars->pars[P_TRIG] & 8)?1:0;
    int i,b,d;
    struct frameparspair_t pars[5]= {
            {G_CALLNEXT+1,0},
            {G_CALLNEXT+2,0},
            {G_CALLNEXT+3,0},
            {G_CALLNEXT+4,0},
            {G_CALLNASAP,0}};
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, safe=%d, async=%d, nooverlap=%d\n",sensor_port,frame16, safe, async, nooverlap);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    for (i=0; i < (sizeof(ahead_tab)/sizeof(ahead_tab[0])); i+=7) {
        b=ahead_tab[i];
        d=ahead_tab[i+1+(nooverlap?5:(1+((async?2:0)+(safe?1:0))))]; ///continuous/safe - 1, continuous/no skip - 2, async/safe - 3, async/no skip - 4, nooverlap - 5
        if ((d>0) && (d <=4 )) {
            pars[d-1].val |= (1<<b);
        }
        if (!ahead_tab[i+1]) { // not the ASAP only mode
            pars[4].val |= (1<<b);
        }

    }

    for (i=2; i>=0; i--) {
        pars[i].val |= pars[i+1].val;
    }
    // moved all these parameters to "frame zero" (static)
    GLOBALPARS(sensor_port, G_CALLNEXT+1)=pars[0].val;
    GLOBALPARS(sensor_port, G_CALLNEXT+2)=pars[1].val;
    GLOBALPARS(sensor_port, G_CALLNEXT+3)=pars[2].val;
    GLOBALPARS(sensor_port, G_CALLNEXT+4)=pars[3].val;
    GLOBALPARS(sensor_port, G_CALLNASAP)= pars[4].val;
    return 0;
}


/** Restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
 * data for the CHN2 should be available (prepared)
 * NC393 - same as  pgm_compctl? */
int pgm_comprestart(int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
#ifndef NC353
    int extra_pages;
    int disable_need =  1; // TODO: Use some G_* parameter
    int reset_frame;
    x393_cmprs_mode_t        cmprs_mode =        {.d32=0};
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    // does it need to be be started (nothing do be done to stop)
    if (thispars->pars[P_COMPRESSOR_RUN]==0) {
        MDP(DBGB_PADD, sensor_port,"thispars->pars[P_COMPRESSOR_RUN] = %d, does not need compressor to be started\n",
                (int)thispars->pars[P_COMPRESSOR_RUN])
        return 0; // does not need compressor to be started
    }
    reset_frame = ((prevpars->pars[P_COMPRESSOR_RUN]==0) && (thispars->pars[P_COMPRESSOR_RUN]!=0))? 1:0;
    // NC393: memory controller already set by pgm_memcompressor, but we'll need to setup dependent/from memory here
    switch(thispars->pars[P_COLOR]){
    case COLORMODE_COLOR:
    case COLORMODE_COLOR20:
        extra_pages = 1;
        break;
    default:
        extra_pages = 0;
    }
    // Compressor memory can be stopped, run single (next frame) or run continuously
    // Compressor itself can run in standalone mode 2 (when sensor is stopped/single) or normal mode "3" (X393_CMPRS_CBIT_RUN_ENABLE)
    // program compressor mode - normal run (3) or standalone(2)
    switch (thispars->pars[P_COMPRESSOR_RUN]) {
    case COMPRESSOR_RUN_STOP:
        cmprs_mode.run = X393_CMPRS_CBIT_RUN_DISABLE;
        break;
    case COMPRESSOR_RUN_SINGLE:
    case COMPRESSOR_RUN_CONT:
        cmprs_mode.run = ((thispars->pars[P_SENSOR_RUN] & 3)==SENSOR_RUN_CONT)? X393_CMPRS_CBIT_RUN_ENABLE : X393_CMPRS_CBIT_RUN_STANDALONE;
        break;
    }
    cmprs_mode.run_set = 1;
    if (thispars->pars[P_COMPRESSOR_RUN] == COMPRESSOR_RUN_STOP) { // turn comressor off first
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
    }
    // enable memory after the compressor, same latency
    control_compressor_memory (sensor_port,
                               thispars->pars[P_COMPRESSOR_RUN] & 3, // stop/single/run(/reset)
                               reset_frame,
                               extra_pages,
                               disable_need,
                               (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                               frame16);
    if (thispars->pars[P_COMPRESSOR_RUN] != COMPRESSOR_RUN_STOP) { // turn comressor on after memory
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
    }
    dev_dbg(g_dev_ptr,"{%d}   X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",sensor_port, sensor_port, frame16, cmprs_mode.d32);
    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",sensor_port, frame16, cmprs_mode.d32)

    return 0;
#else
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    // does it need to be be started (nothing do be done to stop)
    if (thispars->pars[P_COMPRESSOR_RUN]==0) return 0; // does not need compressor to be started
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    // reset memory controller for the channel2 to the start of the frame
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL1,   thispars->pars[P_SDRAM_CHN21]);
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL2,   thispars->pars[P_SDRAM_CHN22]);
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL0,   thispars->pars[P_SDRAM_CHN20]);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH2_CTL1,   (int) thispars->pars[P_SDRAM_CHN21]);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH2_CTL2,   (int) thispars->pars[P_SDRAM_CHN22]);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SDCH2_CTL0,   (int) thispars->pars[P_SDRAM_CHN20]);

    // enable memory channel2
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(0));
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(0));
    // set number of tiles to compressor
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_MCUNUM, thispars->pars[P_TILES]-1);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_MCUNUM, (int) thispars->pars[P_TILES]-1);
    // start the compressor
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, (thispars->pars[P_COMPRESSOR_RUN]==2) ? COMPCMD_RUN : COMPCMD_SINGLE);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_COMP_CMD, (int) ((thispars->pars[P_COMPRESSOR_RUN]==2) ? COMPCMD_RUN : COMPCMD_SINGLE));
    return 0;
#endif
}

/** Stop compressor when changing geometry
 *  NC393 - stopping compressor memory or compressor should also be stopped? Doing both
 */
int pgm_compstop   (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
#ifndef NC353
    int extra_pages;
    int disable_need =  1; // TODO: Use some G_* parameter
    x393_cmprs_mode_t        cmprs_mode =        {.d32=0};
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -EINVAL; // wrong frame
    switch(thispars->pars[P_COLOR]){
    case COLORMODE_COLOR:
    case COLORMODE_COLOR20:
        extra_pages = 1;
        break;
    default:
        extra_pages = 0;
    }
    // Stop compressor (do not propagate frame sync late, finish current frame)
    cmprs_mode.run = X393_CMPRS_CBIT_RUN_DISABLE;
    cmprs_mode.run_set = 1;
    X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode); // compressor off
    // Stop memory -> compressor. Will continue current frame until finished
    //TODO NC393: Handle safe/unsafe reprogramming memory at frame syncs - compression can be finished later

    control_compressor_memory (sensor_port, // compressor memory off
                               COMPRESSOR_RUN_STOP,
                               0, // reset_frame
                               extra_pages,
                               disable_need,
                               (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                               frame16);
    dev_dbg(g_dev_ptr,"{%d}@0x%lx: X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",sensor_port, getThisFrameNumber(sensor_port), sensor_port, frame16, cmprs_mode.d32);
    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n", sensor_port, frame16, cmprs_mode.d32)
    return 0;

#else
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    //  if (frame16 & ~PARS_FRAMES_MASK) return -1; // wrong frame (can be only -1 or 0..7)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame (can be only -1 or 0..7)
    X3X3_SEQ_SEND1(fpga_addr, X313_WA_COMP_CMD, COMPCMD_STOP);
    dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int)  X313_WA_COMP_CMD, (int) COMPCMD_STOP);
    return 0;
#endif
}

/** Compressor control: only start/stop/single (after explicitly changed, not when geometry was changed) */
int pgm_compctl    (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
#ifndef NC353
    int extra_pages;
    int disable_need =  1; // TODO: Use some G_* parameter
    x393_cmprs_mode_t        cmprs_mode =        {.d32=0};
    int reset_frame = 0;
//    int just_started = 0;
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, prevpars->pars[P_COMPRESSOR_RUN]=%d, thispars->pars[P_COMPRESSOR_RUN]=%d \n",
            sensor_port,frame16, (int) prevpars->pars[P_COMPRESSOR_RUN], (int) thispars->pars[P_COMPRESSOR_RUN]);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d, prevpars->pars[P_COMPRESSOR_RUN]=%d, thispars->pars[P_COMPRESSOR_RUN]=%d \n",
            frame16, (int) prevpars->pars[P_COMPRESSOR_RUN], (int) thispars->pars[P_COMPRESSOR_RUN])

    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    reset_frame = ((prevpars->pars[P_COMPRESSOR_RUN]==0) && (thispars->pars[P_COMPRESSOR_RUN]!=0))? 1:0;
    switch(thispars->pars[P_COLOR]){
    case COLORMODE_COLOR:
    case COLORMODE_COLOR20:
        extra_pages = 1;
        break;
    default:
        extra_pages = 0;
    }
// Compressor memory can be stopped, run single (next frame) or run continuously
// Compressor itself can run in standalone mode 2 (when sensor is stopped/single) or normal mode "3" (X393_CMPRS_CBIT_RUN_ENABLE)
    // program compressor mode first - normal run (3) or standalone(2)
    switch (thispars->pars[P_COMPRESSOR_RUN]) {
    case COMPRESSOR_RUN_STOP:
        cmprs_mode.run = X393_CMPRS_CBIT_RUN_DISABLE;
        break;
    case COMPRESSOR_RUN_SINGLE:
    case COMPRESSOR_RUN_CONT:
        // look here on the sensor - if it is stopped - run in standalone mode (2), otherwise - enable(3)
        cmprs_mode.run = ((thispars->pars[P_SENSOR_RUN] & 3)==SENSOR_RUN_CONT)? X393_CMPRS_CBIT_RUN_ENABLE : X393_CMPRS_CBIT_RUN_STANDALONE;
        break;
    }
    cmprs_mode.run_set = 1;
    if (thispars->pars[P_COMPRESSOR_RUN] == COMPRESSOR_RUN_STOP) { // turn comressor off first
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
    }
//    // enable memory after the compressor, same latency
    control_compressor_memory (sensor_port,
                               thispars->pars[P_COMPRESSOR_RUN] & 3, // stop/single/run(/reset)
                               reset_frame,
                               extra_pages,
                               disable_need,
                               (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
                                        frame16);
    if (thispars->pars[P_COMPRESSOR_RUN] != COMPRESSOR_RUN_STOP) { // turn comressor on after memory
        X393_SEQ_SEND1 (sensor_port, frame16, x393_cmprs_control_reg, cmprs_mode);
    }

    dev_dbg(g_dev_ptr,"{%d}@0x%lx: X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n",sensor_port, getThisFrameNumber(sensor_port), sensor_port, frame16, cmprs_mode.d32);

    MDP(DBGB_PADD, sensor_port,"X393_SEQ_SEND1(0x%x, 0x%x, x393_cmprs_control_reg, 0x%x)\n", sensor_port, frame16, cmprs_mode.d32)
    return 0;

#else
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, prevpars->pars[P_COMPRESSOR_RUN]=%d, thispars->pars[P_COMPRESSOR_RUN]=%d \n",sensor_port,frame16, (int) prevpars->pars[P_COMPRESSOR_RUN], (int) thispars->pars[P_COMPRESSOR_RUN]);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    if ((prevpars->pars[P_COMPRESSOR_RUN]==0) && (thispars->pars[P_COMPRESSOR_RUN]!=0)) { // just started
        // reset memory controller for the channel2 to the start of the frame
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL1,   thispars->pars[P_SDRAM_CHN21]);
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL2,   thispars->pars[P_SDRAM_CHN22]);
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL0,   thispars->pars[P_SDRAM_CHN20]);
        // enable memory channel2 (NOTE: wnen is it disabled? does it need to be disabled?)
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(2));
        // set number of tiles to compressor
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_MCUNUM, thispars->pars[P_TILES]-1);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_SDCH2_CTL1, (int)thispars->pars[P_SDRAM_CHN21]);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_SDCH2_CTL2, (int)thispars->pars[P_SDRAM_CHN22]);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_SDCH2_CTL0, (int)thispars->pars[P_SDRAM_CHN20]);
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_SD_MODE, (int) X313_CHN_EN_D(2));
        dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int)X313_WA_MCUNUM, (int)(thispars->pars[P_TILES]-1));
    }
    if ((prevpars->pars[P_COMPRESSOR_RUN] != thispars->pars[P_COMPRESSOR_RUN]) || (thispars->pars[P_COMPRESSOR_RUN]==COMPRESSOR_RUN_SINGLE))  // changed or single
        switch (thispars->pars[P_COMPRESSOR_RUN]) {
        case COMPRESSOR_RUN_STOP:
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_STOP);
            dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_STOP);
            break;
        case COMPRESSOR_RUN_SINGLE:
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_SINGLE);
            //TODO: Update for NC393
            if (!x313_is_dma_on()) x313_dma_start();
            dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_SINGLE);
            break;
        case COMPRESSOR_RUN_CONT:
            X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_RUN);
            if (!x313_is_dma_on()) x313_dma_start();
            dev_dbg(g_dev_ptr,"{%d}  X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n",sensor_port, (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_RUN);
            break;
        }
    return 0;
#endif
}


/** Load gamma tables to FPGA (prepare if not yet).
 * pgm_gamma should prepare table and set onchange_gammaload (in ASAP mode)
 * parameters are for the next after the current page, gamma tables will become active after vsync  */
int pgm_gammaload  (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    //TODO: make multi-subchannel, for now using the same gamma for all
    int sub_chn;
    struct frameparspair_t pars_to_update[4]; // 4 needed, increase if more entries will be added
    int nupdate=0;

    int color, rslt;
    struct {
        unsigned short scale;
        unsigned short hash16;
    } gamma32;
    unsigned long * pgamma32= (unsigned long *) & gamma32;
    unsigned long *gtable;
    int need_pgm=0;
    int fpga_result=0;

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d, (getThisFrameNumber() & PARS_FRAMES_MASK)= %ld, thispars->pars[P_GTAB_R]=0x%lx, thispars->pars[P_FRAME]=0x%lx\n",
            sensor_port,      frame16,      getThisFrameNumber(sensor_port) & PARS_FRAMES_MASK,      thispars->pars[P_GTAB_R],     thispars->pars[P_FRAME]);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d, (getThisFrameNumber() & PARS_FRAMES_MASK)= %ld, thispars->pars[P_GTAB_R]=0x%lx, thispars->pars[P_FRAME]=0x%lx\n",
            frame16,      getThisFrameNumber(sensor_port) & PARS_FRAMES_MASK,      thispars->pars[P_GTAB_R],     thispars->pars[P_FRAME])

    ///NOTE: Yes, ASAP, but - 1 frame ahead
    if (frame16 >= 0) return -1; // only can work in ASAP mode
#if ELPHEL_DEBUG
    struct framepars_t * nextspars=&framepars[(thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK];
    dev_dbg(g_dev_ptr,"{%d}  nextframe=%d, nextsparsd, nextspars->pars[P_GTAB_R]=0x%lx, nextspars->pars[P_FRAME]=0x%lx\n",sensor_port,(int) ((thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK), nextspars->pars[P_GTAB_R], nextspars->pars[P_FRAME]);
    dev_dbg(g_dev_ptr,"{%d}  nextframe=%d, nextspars->pars[P_GTAB_R]=0x%lx, nextspars->pars[P_FRAME]=0x%lx\n",sensor_port,(int) ((thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK), nextspars->pars[P_GTAB_R], nextspars->pars[P_FRAME]);
#endif
    for (color=0; color<4; color++) if (get_locked_hash32(color,sensor_port, 0)!=thispars->pars[P_GTAB_R+color]) need_pgm++;
    // code currently does not allow to overwrite just 1 table - only all 4
    if (need_pgm) {
        for (sub_chn = 0; sub_chn< MAX_SENSORS;sub_chn++) {
            for (color=0; color<4; color++) {
                *pgamma32=thispars->pars[P_GTAB_R+color];
                // Normally, nothing will be calculated in the next set_gamma_table() call
                rslt=set_gamma_table (gamma32.hash16,
                        gamma32.scale,
                        NULL,
                        GAMMA_MODE_HARDWARE | GAMMA_MODE_LOCK,
                        color,
                        sensor_port,
                        0); // frame16 - one ahead of the current TODO 393 multisensor - split gamma tables to subchannels
                //   now gtable will be old one if result <=0 get_gamma_fpga(color) can return 0 only if nothing yet was programmed
                //TODO: Update for NC393
#ifndef NC353
                if ((gtable= get_gamma_fpga(color,sensor_port,sub_chn))) // missing channels return NULL
                {
                    fpga_result = fpga_gamma_write_nice(color,         // Color (0..3)
                            sensor_port,   // sensor port (0..3)
                            sub_chn,       // sensor sub-channel (when several are connected through a multiplexer)
                            gtable);       // Gamma table (256 DWORDs) in encoded FPGA format

                }
#else
                if ((gtable= get_gamma_fpga(color))) fpga_table_write_nice (CX313_FPGA_TABLES_GAMMA + (color * 256), 256, gtable);
#endif
                if (rslt <= 0) SETFRAMEPARS_SET(P_GTAB_R+color, get_locked_hash32(color,sensor_port,0)); // restore to the locked table
            }
            dev_dbg(g_dev_ptr,"{%d} need_pgm=%d, get_locked_hash32(*)=0x%lx 0x%lx 0x%lx 0x%lx\n",
                    sensor_port,need_pgm,
                    get_locked_hash32(0,sensor_port,sub_chn),
                    get_locked_hash32(1,sensor_port,sub_chn),
                    get_locked_hash32(2,sensor_port,sub_chn),
                    get_locked_hash32(3,sensor_port,sub_chn));
        }
    }
    if (nupdate)  {
        setFramePars(sensor_port, thispars, nupdate, pars_to_update);  // save changes, schedule functions
        dev_dbg(g_dev_ptr,"{%d} had to restore back %d gamma tables (color components) \n",sensor_port,nupdate);
        return -1;
    }
    return fpga_result; // 0; fpga_result==0 if not used or OK, -ENODEV if FPGA is not programmed
}

/** Program sensor registers (probably just those that are manually set)
 * seems nothing to do here - all in the sensor-specific function */
int pgm_sensorregs (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    return 0;
}

/** Program multisensor parameters (10359 board)
 * All replaced by multi10359.c */
  int pgm_multisens    (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return OK - 0, <0 - error
  {
      dev_dbg(g_dev_ptr,"{%d} frame16=%d\n",sensor_port,frame16); // nothing here, all in multisensor.c
      MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
      return 0;
  }




/** Programs FPGA registers that provide pre-scaling of the sensor data and vignetting correction
 * TODO: Add P,Q,X0,Y0,E parameters and trigger recalcualtion when WOI is changed
 * TODO 393: update */
//TODO: Add P,Q,X0,Y0,E parameters and trigger recalcualtion when WOI is changed
int pgm_prescal        (int sensor_port,               ///< sensor port number (0..3)
						struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
						struct framepars_t * thispars, ///< sensor current parameters
						struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
						int frame16)                   ///< 4-bit (hardware) frame number parameters should
													   ///< be applied to,  negative - ASAP
													   ///< @return OK - 0, <0 - error
{
#ifndef NC353
    int sub_chn;
    int par_index, poffs;
    x393_lens_corr_t lens_corr = {.d32=0};

    // In 393 there can be multiple subchannels, with proportional offsets. Initially offset == 0, so the same parameters will
    // be applied to all active subchannels.

    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    MDP(DBGB_PSFN, sensor_port,"frame16=%d\n",frame16)
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    for (sub_chn = 0; sub_chn < MAX_SENSORS; sub_chn++) if (GLOBALPARS(sensor_port, G_SUBCHANNELS) & (1 << sub_chn)){
        poffs = VIGNET_SUBCHN_OFFSET * sub_chn;
        par_index = P_VIGNET_AX + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn = sub_chn;
            lens_corr.addr = X393_LENS_AX;
            lens_corr.ax = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_VIGNET_AY + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn = sub_chn;
            lens_corr.addr = X393_LENS_AY;
            lens_corr.ay = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_VIGNET_C + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn = sub_chn;
            lens_corr.addr = X393_LENS_C;
            lens_corr.c = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }

        par_index = P_VIGNET_BX + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn = sub_chn;
            lens_corr.addr = X393_LENS_BX;
            lens_corr.bx = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_VIGNET_BY + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn = sub_chn;
            lens_corr.addr = X393_LENS_BY;
            lens_corr.by = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_SCALE_ZERO_IN + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_FAT0_IN;
            lens_corr.fatzero_in = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_SCALE_ZERO_OUT + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_FAT0_OUT;
            lens_corr.fatzero_out = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_VIGNET_SHL + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_POST_SCALE;
            lens_corr.post_scale = thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }

        par_index = P_DGAINR + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_SCALE0;
            lens_corr.scale =      thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }

        par_index = P_DGAING + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_SCALE1;
            lens_corr.scale =      thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_DGAINGB + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_SCALE2;
            lens_corr.scale =      thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
        par_index = P_DGAINB + poffs;
        if (FRAMEPAR_MODIFIED(par_index)) {
            lens_corr.sub_chn =    sub_chn;
            lens_corr.addr =       X393_LENS_SCALE3;
            lens_corr.scale =      thispars->pars[par_index];
            X393_SEQ_SEND1 (sensor_port, frame16, x393_lens_corr_cnh_addr_data, lens_corr);
            dev_dbg(g_dev_ptr,"{%d}  X393_SEQ_SEND1(0x%x,  0x%x, x393_lens_corr_cnh_addr_data,  0x%x)\n",sensor_port, sensor_port, frame16, lens_corr.d32);
        }
    }

    return 0;

#else
    dev_dbg(g_dev_ptr,"{%d}  frame16=%d\n",sensor_port,frame16);
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
    if (FRAMEPAR_MODIFIED(P_VIGNET_AX)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_AX(thispars->pars[P_VIGNET_AX]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_AX(thispars->pars[P_VIGNET_AX]));
    }
    if (FRAMEPAR_MODIFIED(P_VIGNET_AY)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_AY(thispars->pars[P_VIGNET_AY]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_AY(thispars->pars[P_VIGNET_AY]));
    }
    if (FRAMEPAR_MODIFIED(P_VIGNET_C)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_C(thispars->pars[P_VIGNET_C]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_C(thispars->pars[P_VIGNET_C]));
    }
    if (FRAMEPAR_MODIFIED(P_VIGNET_BX)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_BX(thispars->pars[P_VIGNET_BX]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_BX(thispars->pars[P_VIGNET_BX]));
    }
    if (FRAMEPAR_MODIFIED(P_VIGNET_BY)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_BY(thispars->pars[P_VIGNET_BY]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_BY(thispars->pars[P_VIGNET_BY]));
    }
    if (FRAMEPAR_MODIFIED(P_SCALE_ZERO_IN)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_FATZERO_IN(thispars->pars[P_SCALE_ZERO_IN]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_FATZERO_IN(thispars->pars[P_SCALE_ZERO_IN]));
    }
    if (FRAMEPAR_MODIFIED(P_SCALE_ZERO_OUT)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_FATZERO_OUT(thispars->pars[P_SCALE_ZERO_OUT]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_FATZERO_OUT(thispars->pars[P_SCALE_ZERO_OUT]));
    }
    if (FRAMEPAR_MODIFIED(P_VIGNET_SHL)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_POSTSCALE(thispars->pars[P_VIGNET_SHL]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_POSTSCALE(thispars->pars[P_VIGNET_SHL]));
    }
    if (FRAMEPAR_MODIFIED(P_DGAINR)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(0, thispars->pars[P_DGAINR]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(0,thispars->pars[P_DGAINR]));
    }
    if (FRAMEPAR_MODIFIED(P_DGAING)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(1, thispars->pars[P_DGAING]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(1,thispars->pars[P_DGAING]));
    }
    if (FRAMEPAR_MODIFIED(P_DGAINGB)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(2, thispars->pars[P_DGAINGB]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(2,thispars->pars[P_DGAINGB]));
    }
    if (FRAMEPAR_MODIFIED(P_DGAINB)) {
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(3, thispars->pars[P_DGAINB]));
        dev_dbg(g_dev_ptr,"{%d}   X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n",sensor_port, fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(3,thispars->pars[P_DGAINB]));
    }
    return 0;
#endif
}
