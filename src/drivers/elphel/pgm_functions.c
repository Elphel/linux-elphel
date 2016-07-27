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
//#include <linux/autoconf.h>
#include <linux/vmalloc.h>

//#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <elphel/c313a.h>
//#include <asm/elphel/exifa.h>
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "fpga_sdram.h" // use a single fpga_initSDRAM(void)
//#include "fpgaconfi2c.h"  //to control clocks
//#include "cc3x3.h"
//#include "fpga_io.h"
//#include "x3x3.h"           // hardware definitions
#include "framepars.h"
#include "sensor_common.h"
#include "gamma_tables.h"
#include "quantization_tables.h"
#include "latency.h"
#include "pgm_functions.h"
//#include "cxdma.h"         // is_dma_on()
#include "jpeghead.h"      // to program FPGA Huffman tables

#include "legacy_defines.h" // temporarily
#include "sensor_i2c.h"


/**
 * @brief optional debug output macros
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
int init_pgm_proc(void) {
  int i;
  MDF1(printk("\n"));
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

/**
 * @brief add sensor-specific processing function (executes after the non-specific function with the same index)
 * @param index function index (internally 32 is added to distinguish from the common (not sensor-specific) functions
 * @param sens_func pointer to a sensor-specific function
 * @return always 0 */
int add_sensor_proc(int index, int (*sens_func)(int sensor_port, struct sensor_t * ,  struct framepars_t * , struct framepars_t *, int )) {
  MDF1(printk("index=0x%x\n",index));
  sensorproc->pgm_func[32+(index & 0x1f)]=       sens_func;
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= 0) return -1; // can only work in ASAP mode
  if (thispars->pars[P_SENSOR]) return 0; // Sensor is already detected - do not bother (to re-detect it P_SENSOR should be set to 0)
// no other initializations, just the sensor-related stuff (starting with lowest sensor clock)
// stop hardware i2c controller, so it will not get stuck when waiting for !busy


// NOTE: disabling interrupts here !!!
  camera_interrupts (0);
  MDF1(printk("Disabled camera interrupts\n"));

// This 3 initialization commands were not here, trying to temporarily fix problem when WP was 8/16 words higher than actual data in DMA buffer
  if (GLOBALPARS(sensor_port, G_TEST_CTL_BITS) & (1<< G_TEST_CTL_BITS_RESET_DMA_COMPRESSOR )) {
    MDF1(printk("x313_dma_stop()\n"));
    x313_dma_stop();
    MDF1(printk("x313_dma_init()\n"));
    x313_dma_init();
    MDF1(printk("reset_compressor()\n"));
    reset_compressor();
  }

// TODO: Add 10347 detection here //   if (IS_KAI11000) return init_KAI11000();
// Need to set slow clock
//  f1=imageParamsR[P_CLK_SENSOR]=20000000; setClockFreq(1, imageParamsR[P_CLK_SENSOR]); X3X3_RSTSENSDCM;
   int was_sensor_freq=getClockFreq(1); // using clock driver data, not thispars
   setFramePar(thispars, P_CLK_FPGA,  getClockFreq(0)); // just in case - read the actual fpga clock frequency and store it (no actions)
   setFramePar(thispars, P_CLK_SENSOR,  48000000);
   setClockFreq(1, thispars->pars[P_CLK_SENSOR]);
   printk("\nsensor clock set to %d\n",(int) thispars->pars[P_CLK_SENSOR]);

   udelay (100);// 0.0001 sec to stabilize clocks
   X3X3_RSTSENSDCM; // FPGA DCM can fail after clock change, needs to be reset
   X3X3_SENSDCM_CLK2X_RESET; // reset pclk2x DCM also
   udelay (50000);// 0.05 sec to stabilize clocks
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
   if (thispars->pars[P_SENSOR]==0) multisensor_pgm_detectsensor (sensor,  thispars, prevpars, frame16);  // multisensor
   if (thispars->pars[P_SENSOR]==0) {
     printk("removing MRST from the sensor\n");
     CCAM_MRST_OFF;
   }
#ifdef CONFIG_ETRAX_ELPHEL_MT9X001
   if (thispars->pars[P_SENSOR]==0) {
      mt9x001_pgm_detectsensor(sensor,  thispars, prevpars, frame16);  // try Micron 5.0 Mpixel - should return sensor type
      printk("trying MT9P001\n");
   }
#endif
// temporary - disabling old sensors
#define ENABLE_OLD_SENSORS 1
#ifdef ENABLE_OLD_SENSORS
   if (thispars->pars[P_SENSOR]==0)  { // no - it is not MT9P001)
   printk("trying other MT9X001\n");
   CCAM_CNVEN_ON;
//     CCAM_ARST_ON; ///MT9P001 expects ARST==0 (STANDBY) - done inside mt9x001.c
// enable output for power converter signals
// This should be done first!!!
// printk ("Will Turn DC power for the sensor after 1 sec delay...\n");  udelay (1000000);
// turning on DC-DC converter may cause system to reboot because of a power spike, so start slow
#ifdef NC353
        port_csp0_addr[X313_WA_DCDC] = 0x44; // 48 - enough, 41 - ok - was 0x61; //
        printk ("sensor power set low\r\n ");
     udelay (10000); // Wait voltage to come up (~10 ms)
        printk ("will set to 0x41\r\n");
     udelay (10000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x41; // 
        printk ("will set to 0x30\r\n");
     udelay (10000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x30; //
        printk ("will set to 0x28\r\n");
     udelay (10000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x28; //
        printk ("will set to 0x24\r\n");
     udelay (10000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x24; //
        printk ("will set to 0x22\r\n");
     udelay (10000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x22; //
     udelay (100000); // to find the problem
        port_csp0_addr[X313_WA_DCDC] = 0x10; // now - full frequency (same as 0x21). Slow that down if the sensor clock is above 20MHz (i.e.0x22 for 40MHz)
        printk (".. full\r\n");
     udelay (10000); // Wait voltage to stabilize
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
   if (thispars->pars[P_SENSOR]==0)  printk("trying MT9X001\n");
   if (thispars->pars[P_SENSOR]==0) mt9x001_pgm_detectsensor(sensor,  thispars, prevpars, frame16);  // try Micron 1.3/2.0/3.0 Mpixel
#endif
#ifdef CONFIG_ETRAX_ELPHEL_KAC1310
   if (thispars->pars[P_SENSOR]==0) kac5000_pgm_detectsensor(sensor,  thispars, prevpars, frame16);  // try KAC-5000
#endif
#ifdef CONFIG_ETRAX_ELPHEL_ZR32112
   if (thispars->pars[P_SENSOR]==0) zr32112_pgm_detectsensor(sensor,  thispars, prevpars, frame16); // try ZR32112
#endif
#ifdef CONFIG_ETRAX_ELPHEL_ZR32212
   if (thispars->pars[P_SENSOR]==0) zr32212_pgm_detectsensor(sensor,  thispars, prevpars, frame16); // try ZR32212
#endif
#endif // ENABLE_OLD_SENSORS *************** temporary disabling other sensors ********************

   if (thispars->pars[P_SENSOR]==0) {
     sensor->sensorType=SENSOR_NONE;                 // to prevent from initializing again
     printk("No image sensor found\r\n");
   }
   setFramePar(thispars, P_SENSOR_WIDTH,   sensor->imageWidth);  // Maybe get rid of duplicates?
   setFramePar(thispars, P_SENSOR_HEIGHT,  sensor->imageHeight); // Maybe get rid of duplicates?
   if (sensor->i2c_period==0) sensor->i2c_period=2500;           // SCL period in ns, (standard i2c - 2500)
   int qperiod=thispars->pars[P_I2C_QPERIOD];
   if (qperiod==0) qperiod=(sensor->i2c_period * (thispars->pars[P_CLK_FPGA]/1000))/4000000;
   setFramePar(thispars, P_I2C_QPERIOD | FRAMEPAIR_FORCE_NEWPROC,  qperiod); // force i2c
   int i2cbytes=thispars->pars[P_I2C_BYTES];
   if (i2cbytes==0) i2cbytes=sensor->i2c_bytes;
   setFramePar(thispars, P_I2C_BYTES | FRAMEPAIR_FORCE_NEWPROC,  i2cbytes); // force i2c
// restore/set sensor clock
   if ((was_sensor_freq < sensor->minClockFreq) || (was_sensor_freq > sensor->maxClockFreq)) was_sensor_freq=sensor->nomClockFreq;
   setFramePar(thispars, P_CLK_SENSOR | FRAMEPAIR_FORCE_NEWPROC,  was_sensor_freq); // will schedule clock/phase adjustment
   int phase=thispars->pars[P_SENSOR_PHASE];
// TODO: remove phase adjustment from here
   if (phase==0) {
     phase= 0x40000; 
     setFramePar(thispars, P_SENSOR_PHASE | FRAMEPAIR_FORCE_NEWPROC,  phase); // will schedule clock/phase adjustment
   }
   setFramePar(thispars, P_IRQ_SMART | FRAMEPAIR_FORCE_NEWPROC,  3);        // smart IRQ mode programming (and enable interrupts)

   // NOTE: sensor detected - enabling camera interrupts here (actual interrupts will start later)
// Here interrupts are disabled - with camera_interrupts (0) earlier in this function)
   camera_interrupts (1);
   return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= 0) return -1; // should be ASAP
//TODO: seems nothing to do here - all in the sensor-specific function:
  return 0;
}


/** Restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified */
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
  MDF3(printk(" frame16=%d\n",frame16));
// If this is a multisensor camera, update composite sensor dimensions (will trigger other related changes)
// For single sensors sensor size is updated only after initialization, with composite it needs to be updated after vertical gap  or number of active sesnors is changed
//  if (GLOBALPARS(G_SENS_AVAIL) ) multisensor_pgm_afterinit0 (sensor, thispars, prevpars,frame16);
// Setup WOI. If size is zero - use maximal that sensor can, in non-zero - just refresh so appropriate actions will be scheduled on chnange
  int woi_width=thispars->pars[P_WOI_WIDTH];
  int woi_height=thispars->pars[P_WOI_HEIGHT];
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
  SETFRAMEPARS_UPDATE(P_TRIG | FRAMEPAIR_FORCE_NEW); // set trigger mode (or should it alway be internal after init?)
// something else to add? NOTE: Only sensor parameters, erased when it is reset - other parameters should not chnage here
// NOTE: increase pars_to_update[24] size if needed
  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
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
  MDF3(printk(" frame16=%d\n",frame16));
  int thisPhase=thispars->pars[P_SENSOR_PHASE];
  if (frame16 >= 0) return -1; // can only work in ASAP mode
  int was_sensor_freq=getClockFreq(1); // using clock driver data, not thispars
  if (unlikely(new_freq > sensor->maxClockFreq)) {
      new_freq= sensor->maxClockFreq;
      SETFRAMEPARS_SET(P_CLK_SENSOR,  new_freq);
  }
  if ((new_freq != was_sensor_freq) || (thisPhase & 0x40000)) { // 0x40000 reprogram clock even if it did not change
    if (unlikely(setClockFreq(1, thispars->pars[P_CLK_SENSOR])<0)) { // if it failed to setup frequency - use the old one
       new_freq=was_sensor_freq;
       setClockFreq(1, was_sensor_freq);
       SETFRAMEPARS_SET(P_CLK_SENSOR,  new_freq);
    }
    X3X3_RSTSENSDCM; // Make Xilinx Spartan DCM happy (it does not like changing input clock)
    X3X3_SENSDCM_CLK2X_RESET; // reset pclk2x DCM also
    if (sensor->needReset & SENSOR_NEED_RESET_CLK) schedule_this_pgm_func(thispars, onchange_initsensor);
// set HACT PHASE here - 90 deg. increment - seems like a bug in MT9P001 sensors - horisontal (and vert.) sync has different phase than data
// Update (program) hact_shift only when the frequency is changed, not when just the phase is
// adjustment range is 270 early to 360 late
    int  hact_shift= 0.004 * (((int)sensor->hact_delay)/(1000000000.0/new_freq)) + 4.5;
    MDF16(printk ("hact_shift=%d-4\r\n",hact_shift));
    hact_shift-=4;
    MDF16(printk ("hact_shift=%d\r\n",hact_shift));
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
    long * cableDelay= (long *) &GLOBALPARS(sensor_port, G_CABLE_TIM );
    long * FPGADelay=  (long *) &GLOBALPARS(sensor_port, G_FPGA_TIM0 );
    int clk_period= 1000000000000.0/new_freq;  // period in ps
    MDF16(printk ("cableDelay=%ld, FPGADelay=%ld, clk_period=%d\r\n",cableDelay[0], FPGADelay[0], clk_period));
    int px_delay=-(clk_period/2 - FPGADelay[0]- cableDelay[0] - ((int) sensor->sensorDelay)) ;
    MDF16(printk ("px_delay=%d\r\n",px_delay));
    int px_delay90=(4*px_delay+clk_period/2)/clk_period;
    px_delay -= (px_delay90*clk_period)/4; // -clk_period/8<= now px_delay <= +clk_period/8
    MDF16(printk ("px_delay=%d, px_delay90=%d\r\n",px_delay,px_delay90));
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
   MDF16(printk("old_sensor_phase=0x%x old_sensor_phase90=0x%x\r\n", old_sensor_phase, old_sensor_phase90 ));
   MDF16(printk("sensor_phase=0x%x sensor_phase90=0x%x\r\n", sensor_phase, sensor_phase90 ));
   MDF16(printk("diff_phase=0x%x diff_phase90=0x%x\r\n", diff_phase, diff_phase90 ));
     if      (diff_phase > 0) for (i=diff_phase; i > 0; i--) {X3X3_SENSDCM_INC ; udelay(1); }
     else if (diff_phase < 0) for (i=diff_phase; i < 0; i++) {X3X3_SENSDCM_DEC ; udelay(1); }
     if      (diff_phase90 > 0) for (i=diff_phase90; i > 0; i--) {X3X3_SENSDCM_INC90 ; udelay(1); }
     else if (diff_phase90 < 0) for (i=diff_phase90; i < 0; i++) {X3X3_SENSDCM_DEC90 ; udelay(1); }
     SETFRAMEPARS_SET(P_SENSOR_PHASE,  thisPhase & 0x3ffff);
  }

  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
  int fpga_addr = frame16;
  X3X3_SEQ_SEND1(fpga_addr,   X313_I2C_CMD,  X3X3_SET_I2C_BYTES(thispars->pars[P_I2C_BYTES]+1) |
                                             X3X3_SET_I2C_DLY  (thispars->pars[P_I2C_QPERIOD]));
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_I2C_CMD, (int) (X3X3_SET_I2C_BYTES(thispars->pars[P_I2C_BYTES]+1) | X3X3_SET_I2C_DLY  (thispars->pars[P_I2C_QPERIOD]))  ));
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
  MDF3(printk(" frame16=%d\n",frame16));
  return pgm_window_common (sensor,  thispars, prevpars, frame16);
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
  MDF3(printk(" frame16=%d\n",frame16));
  return pgm_window_common (sensor,  thispars, prevpars, frame16);
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
  MDF3(printk(" frame16=%d\n",frame16));
  MDF23(printk("thispars->pars[P_WOI_HEIGHT]=%lx thispars->pars[P_WOI_WIDTH]=%lx\n",thispars->pars[P_WOI_HEIGHT], thispars->pars[P_WOI_WIDTH]));
//  if (GLOBALPARS(G_SENS_AVAIL) ) multisensor_pgm_window_common0 (sensor, thispars, prevpars, frame16);

  int sensor_width= thispars->pars[P_SENSOR_WIDTH];
  int sensor_height=thispars->pars[P_SENSOR_HEIGHT];
  oversize=thispars->pars[P_OVERSIZE];
  is_color=1;
  struct frameparspair_t pars_to_update[18]; // 15 needed, increase if more entries will be added
  int nupdate=0;

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
  if (FRAMEPAR_MODIFIED(P_DCM_HOR)) {
    if (dh<1) dh=1; else if (dh>32) dh=32;
    while ((dh>1) && !(sensor->dcmHor  & (1 << (dh-1)))) dh--; // adjust decimation to maximal supported (if requested is not supported)
    if (unlikely(dh!=thispars->pars[P_DCM_HOR])) SETFRAMEPARS_SET(P_DCM_HOR, dh);
  }
// dv (decimation changed)?
  dv = thispars->pars[P_DCM_VERT];
  if (FRAMEPAR_MODIFIED(P_DCM_VERT)) {
    if (dv<1) dv=1; else if (dv>32) dv=32;
    while ((dv>1) && !(sensor->dcmVert  & (1 << (dv-1)))) dv--; // adjust decimation to maximal supported (if requested is not supported)
    if (unlikely(dv!=thispars->pars[P_DCM_HOR])) SETFRAMEPARS_SET(P_DCM_VERT, dv); 
  }
// bh (binning changed)?
  bh = thispars->pars[P_BIN_HOR];
  if (FRAMEPAR_MODIFIED(P_BIN_HOR)) {
    if (bh<1) bh=1; else if (bh>dh) bh=dh;
    while ((bh>1) && !(sensor->binHor  & (1 << (bh-1)))) bh--; // adjust binning to maximal supported (if requested is not supported)
    if (unlikely(bh!=thispars->pars[P_BIN_HOR])) SETFRAMEPARS_SET(P_DCM_HOR, bh); 
  }
// bv (binning changed)?
  bv = thispars->pars[P_BIN_VERT];
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
  width= (((width/dh) + timestamp_len)/X313_TILEHOR)*X313_TILEHOR-timestamp_len; // divided by dh
// suppose minimal width refers to decimated output
  while (width < sensor->minWidth) width+=X313_TILEHOR;
  if (unlikely(thispars->pars[P_ACTUAL_WIDTH] != (width+timestamp_len))) {
       SETFRAMEPARS_SET(P_ACTUAL_WIDTH, width+timestamp_len); ///full width for the compressor, including timestamp, but excluding margins
  }
  if (unlikely(thispars->pars[P_SENSOR_PIXH] != width+X313_MARGINS))
       SETFRAMEPARS_SET(P_SENSOR_PIXH,  width+X313_MARGINS); ///full width for the sensor (after decimation), including margins
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
     if (height > X313_MAXHEIGHT*dv) height=X313_MAXHEIGHT*dv;
     pf_stripes = height / (pfh * dv);
     if(pf_stripes < 1) pf_stripes = 1;
     if (unlikely(thispars->pars[P_SENSOR_PIXV] != pfh)) {
        MDF23(printk(" SETFRAMEPARS_SET(P_SENSOR_PIXV,  0x%x)\n",pfh));
        SETFRAMEPARS_SET(P_SENSOR_PIXV,  pfh);
     }
  } else {
       if ((!oversize ) && (height > sensor_height)) height=sensor_height;
       height= ((height/dv)/X313_TILEVERT) * X313_TILEVERT; // divided by dv (before multisensor options)
// suppose minimal height refers to decimated output
       while (height < sensor->minHeight) height+=X313_TILEVERT; 
       if (unlikely(thispars->pars[P_SENSOR_PIXV] != height+X313_MARGINS))
           SETFRAMEPARS_SET(P_SENSOR_PIXV,  height+X313_MARGINS); ///full height for the sensor (after decimation), including margins
       height*=dv;
       pf_stripes = 0;
       pfh = 0;
  }
 // update photofinish height P_PF_HEIGHT
  if (unlikely((thispars->pars[P_PF_HEIGHT] & 0xffff) != pfh)) {
        MDF23(printk(" SETFRAMEPARS_SET(P_PF_HEIGHT,  0x%x)\n",(int) ((thispars->pars[P_PF_HEIGHT] & 0xffff0000 ) | pfh)));
        SETFRAMEPARS_SET(P_PF_HEIGHT, (thispars->pars[P_PF_HEIGHT] & 0xffff0000 ) | pfh );
  }
// update WOI height [P_WOI_HEIGHT
  if (unlikely(thispars->pars[P_WOI_HEIGHT] != height)) {
        MDF23(printk(" SETFRAMEPARS_SET(P_WOI_HEIGHT,  0x%x)\n",height));
        SETFRAMEPARS_SET(P_WOI_HEIGHT, height); ///full height for the compressor (excluding margins)
  }
// update P_ACTUAL_HEIGHT
  ah=height/dv;
  if (unlikely(thispars->pars[P_ACTUAL_HEIGHT] != ah)) {
        MDF23(printk(" SETFRAMEPARS_SET(P_ACTUAL_HEIGHT,  0x%x)\n",ah));
        SETFRAMEPARS_SET(P_ACTUAL_HEIGHT, ah); ///full height for the compressor (excluding margins)
  }
// left margin
  left = thispars->pars[P_WOI_LEFT];
  if (!oversize) { // in oversize mode let user to specify any margin, including odd ones (bayer shifted)
    if (is_color)                    left &= 0xfffe;
    if ((left + width + X313_MARGINS) > sensor->clearWidth) { 
         left = sensor->clearWidth - width - X313_MARGINS;
         if (is_color)               left &= 0xfffe;
    }
    if (left & 0x8000) left = 0;
  }
// update P_WOI_LEFT
  if (unlikely(thispars->pars[P_WOI_LEFT] != left)) {
       MDF23(printk(" SETFRAMEPARS_SET(P_WOI_LEFT,  0x%x)\n",left));
       SETFRAMEPARS_SET(P_WOI_LEFT, left);
  }

// top margin
  top = thispars->pars[P_WOI_TOP];
  int clearHeight=(sensor->clearHeight-sensor->imageHeight) + thispars->pars[P_SENSOR_HEIGHT];
  if (!oversize) { // in oversize mode let user to specify any margin, including odd ones (bayer shifted)
    if (is_color)                    top &= 0xfffe;
    if ((top + ((pfh>0)?0:height) + X313_MARGINS) > clearHeight) { 
         top = clearHeight - ((pfh>0)?0:height) - X313_MARGINS;
         if (is_color)               top &= 0xfffe;
    }
    if (top & 0x8000) top = 0;
  }
// update P_WOI_TOP
  if (unlikely(thispars->pars[P_WOI_TOP] != top)) {
     MDF23(printk(" SETFRAMEPARS_SET(P_WOI_TOP,  0x%x)\n",top));
     SETFRAMEPARS_SET(P_WOI_TOP, top);
  }
  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  struct frameparspair_t pars_to_update[8]; // 4 needed, increase if more entries will be added
  int nupdate=0;
  int async=(thispars->pars[P_TRIG] & 4)?1:0;
  int cycles;  // number of FPGA clock cycles per frame;
  int min_period;  // number of pixel clock periods needed for the compressor (or user limit)
  int period=0;
  cycles=thispars->pars[P_TILES]; // number of tiles
#if USELONGLONG
  uint64_t ull_min_period;
  uint64_t ull_period;
#endif
//  MDF9(printk(" tiles=%d(0x%x)\n",cycles,cycles));
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
  cycles  *=64 *2; //cycles per frame (64 pixels/block, 2 clock cycles/pixel)
  MDF9(printk(" cycles=%d(0x%x)\n",cycles,cycles));
  cycles += thispars->pars[P_FPGA_XTRA]; // extra cycles needed for the compressor to start/finish the frame;
  MDF9(printk(" cycles with P_FPGA_XTRA =%d(0x%x)\n",cycles,cycles));
// #define P_CLK_FPGA, #define P_CLK_SENSOR    27-28 bits, cycles - 24-25 bits
// TODO: fix long long

#if USELONGLONG
  ull_min_period=(((long long) cycles) * ((long long) thispars->pars[P_CLK_SENSOR]));
  __div64_32(&ull_min_period, thispars->pars[P_CLK_FPGA]);
  min_period= ull_min_period;
  MDF9(printk("min_period =%d(0x%x)\n",min_period,min_period));
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
  MDF9(printk("min_period =%d(0x%x)\n",min_period,min_period));
#endif
// is there limit set for the FPS?
  if (thispars->pars[P_FPSFLAGS]) {
#if USELONGLONG
    ull_period=(((long long) thispars->pars[P_CLK_SENSOR]) * (long long) 1000);
    __div64_32(&ull_period, thispars->pars[P_FP1000SLIM]);
    period= ull_period;
  MDF9(printk("period =%d(0x%x)\n",period,period));
//    period=(((long long) thispars->pars[P_CLK_SENSOR]) * (long long) 1000)/((long long) thispars->pars[P_FP1000SLIM]);
#else
    period=125*(( thispars->pars[P_CLK_SENSOR] << 3) / thispars->pars[P_FP1000SLIM]); // 125 <<3 = 1000
  MDF9(printk("period =%d(0x%x)\n",period,period));
#endif
  }
  MDF1(printk(" period=%d\n",period));
  if ((thispars->pars[P_FPSFLAGS] & 1) && (period>min_period)) min_period=period;
// *********************************************************** P_PF_HEIGHT
  int pfh=thispars->pars[P_PF_HEIGHT] &0xffff ;
  int n_stripes;
  if (pfh>0) {
    n_stripes=thispars->pars[P_WOI_HEIGHT]/pfh;
    if (n_stripes>0) min_period/=n_stripes;
  }

  if (min_period != thispars->pars[P_PERIOD_MIN]) {
    SETFRAMEPARS_SET(P_PERIOD_MIN, min_period);  // set it (and propagate to the later frames)
    MDF9(printk(" SETFRAMEPARS_SET(P_PERIOD_MIN,  0x%x)\n",min_period));
  }
  if (((thispars->pars[P_FPSFLAGS] & 2)==0) || (period < min_period)) period=0x7fffffff; // no upper limit
  if (period != thispars->pars[P_PERIOD_MAX]) SETFRAMEPARS_SET(P_PERIOD_MAX, period);         // set it (and propagate to the later frames)
// Now see if the sequencer period needs to be adjusted
//  if (async && (thispars->pars[P_TRIG_PERIOD] >=256)) { // <256 - single trig
//  if (async && (thispars->pars[P_TRIG_PERIOD] !=1)) { // <256 - single trig, here only ==1 is for single
// Update period to comply even if it is not in async mode
  if (thispars->pars[P_TRIG_PERIOD] !=1) { // <256 - single trig, here only ==1 is for single
    if (thispars->pars[P_TRIG_PERIOD] < min_period) SETFRAMEPARS_SET(P_TRIG_PERIOD, min_period);  // set it (and propagate to the later frames)
    if (async &&  (thispars->pars[P_FPSFLAGS] & 2) && (thispars->pars[P_TRIG_PERIOD] > period)) {
       SETFRAMEPARS_SET(P_TRIG_PERIOD, period);  // set it (and propagate to the later frames)
    }
  }
  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
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
  MDF3(printk(" frame16=%d\n",frame16));
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  int fpga_addr= frame16;
  int async=(thispars->pars[P_TRIG] & 4)?1:0;
//  X3X3_SEQ_SEND1(frame16,  X313_WA_DCR0, X353_DCR0(SENSTRIGEN,async));
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
#ifdef NC353
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
// Set FPN mode (P_FPGATEST - currently only LSB is processed)
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SENSFPN, X313_SENSFPN_D( \
                         (thispars->pars[P_FPGATEST]), \
                         (thispars->pars[P_FPNS]),  \
                         (thispars->pars[P_FPNM]),     \
                         ((thispars->pars[P_BITS]>8)?1:0), \
                          thispars->pars[P_SHIFTL]));

  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SENSFPN, (int) (X313_SENSFPN_D( \
                         (thispars->pars[P_FPGATEST]), \
                         (thispars->pars[P_FPNS]),  \
                         (thispars->pars[P_FPNM]),     \
                         ((thispars->pars[P_BITS]>8)?1:0), \
                          thispars->pars[P_SHIFTL]))));

  int n_scan_lines, n_ph_lines, n_pixels;

// New - writing WOI width for internally generated HACT
  n_pixels=((thispars->pars[P_ACTUAL_WIDTH]+X313_MARGINS) & 0x3fff) | 0x4000;
  X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, n_pixels);
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_NLINES, (int) n_pixels));

// Program number of scan lines to acquire
// Is PhotoFinish mode enabled? // **************** TODO: use ACTUAL_HEIGHT (and update it) not WOI_HEIGHT
  if (((thispars->pars[P_PF_HEIGHT] & 0xffff)>0) && (thispars->pars[P_PF_HEIGHT]<=thispars->pars[P_ACTUAL_HEIGHT])){
     n_ph_lines=  thispars->pars[P_ACTUAL_HEIGHT]/(thispars->pars[P_PF_HEIGHT] &  0x3fff);
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_NLINES, (int) (n_ph_lines-1) | 0x8000));
     X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, (n_ph_lines-1) | 0x8000);
 //    n_scan_lines= thispars->pars[P_PF_HEIGHT];
     n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]; // no margins here

  } else {
// temporary hack trying to disable PH mode earlier

    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", X313_SEQ_ASAP, X313_WA_NLINES, 0x8000));
    X3X3_SEQ_SEND1 (X313_SEQ_ASAP,  X313_WA_NLINES,  0x8000);

    n_scan_lines= thispars->pars[P_ACTUAL_HEIGHT]+X313_MARGINS+thispars->pars[P_OVERLAP];
  }
  n_scan_lines&=0x3fff; 
  X3X3_SEQ_SEND1 (fpga_addr,  X313_WA_NLINES, n_scan_lines);
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_NLINES, (int) n_scan_lines));
// Bayer phase changed?

  int flips=(thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1)<<1);
  int bayer_modified=FRAMEPAR_MODIFIED(P_BAYER) || FRAMEPAR_MODIFIED(P_FLIPH) || FRAMEPAR_MODIFIED(P_FLIPV) || FRAMEPAR_MODIFIED(P_MULTI_MODE);
  MDF3(printk(" flips=%x\n", flips));

  if (thispars->pars[P_MULTI_MODE]) { // Modify flips in composite mode - should match flips of the top (direct) channel
    int this_bit=(1<<thispars->pars[P_MULTI_TOPSENSOR]);
    if (thispars->pars[P_MULTI_FLIPH] & this_bit) flips^=1;
    if (thispars->pars[P_MULTI_FLIPV] & this_bit) flips^=2;
    MDF3(printk(" composite mode - adjusted flips=%x\n", flips));
    bayer_modified=  bayer_modified || FRAMEPAR_MODIFIED(P_MULTI_FLIPH) || FRAMEPAR_MODIFIED(P_MULTI_FLIPV)  || FRAMEPAR_MODIFIED(P_MULTI_TOPSENSOR);
  }
  if (bayer_modified) {
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_DCR0, X353_DCR0(BAYER_PHASE,thispars->pars[P_BAYER] ^ flips ^ sensor->bayer)); ///NOTE: hardware bayer
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int)X313_WA_DCR0, (int)X353_DCR0(BAYER_PHASE,thispars->pars[P_BAYER] ^ ((thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1)<<1)) ^ sensor->bayer) ));

  }
#endif
  return 0;
}

/** Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
 * TODO: Implement for 393 */
int pgm_sensorrun  (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  int fpga_data=0;
  switch (thispars->pars[P_SENSOR_RUN] & 3) {
    case 1: fpga_data=4; break;
    case 2:
    case 3: fpga_data=5; break;
  }
#if NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
// only start/single, stopping will be handled by the pgm_sensorstop
  if (fpga_data) {
     X3X3_SEQ_SEND1(fpga_addr,  X313_WA_TRIG, fpga_data);
     MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_TRIG, (int) fpga_data));
  }
#endif
  return 0;
}

/** Stop acquisition from the sensor to the FPGA (start/single have latency of 2)
 * TODO: Implement for 393 */
int pgm_sensorstop (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  int fpga_data=0;
  switch (thispars->pars[P_SENSOR_RUN] & 3) {
    case 1: fpga_data=4; break;
    case 2:
    case 3: fpga_data=5; break;
  }
#if NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
// only start/single, stopping will be handled by the pgm_sensorstop
  if ((thispars->pars[P_SENSOR_RUN] & 3)==0){
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_TRIG, fpga_data);
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_TRIG, (int) fpga_data));
  }
#endif
  return 0;
}

/** Program gamma table
 * Table with the same hash should be available in cache. It is very unlikely
 * but is still possible that it can be pushed out - TODO: make it guaranteed. So normally new gamma table is 
 * set through a chracter device driver (with FPGA bit set to get locked?) and then pgm_gamma is activated when
 * the P_GTAB_R (*_G,*_GB, *_B) are updated
 * The scale part of these parameters (lower 16 bits) may be modified by white balancing code without loading a new table
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
  MDF3(printk(" frame16=%d, (getThisFrameNumber() & PARS_FRAMES_MASK)= %ld\n",frame16, getThisFrameNumber() & PARS_FRAMES_MASK));
  MDF3(printk(" frame16=%d, thispars->pars[P_GTAB_*]=0x%lx 0x%lx 0x%lx 0x%lx, thispars->pars[P_FRAME]=0x%lx"
              " get_locked_hash32(*)=0x%lx 0x%lx 0x%lx 0x%lx\n",
frame16, thispars->pars[P_GTAB_R],thispars->pars[P_GTAB_R+1],thispars->pars[P_GTAB_R+2],thispars->pars[P_GTAB_R+3],thispars->pars[P_FRAME],
get_locked_hash32(0),get_locked_hash32(1),get_locked_hash32(2),get_locked_hash32(3)));


  MDF16(printk(" frame16=%d, thispars->pars[P_GTAB_*]=0x%lx 0x%lx 0x%lx 0x%lx, thispars->pars[P_FRAME]=0x%lx\n",frame16, thispars->pars[P_GTAB_R],thispars->pars[P_GTAB_R+1],thispars->pars[P_GTAB_R+2],thispars->pars[P_GTAB_R+3],thispars->pars[P_FRAME]));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
// Not needed - now it can be done in advance (just prepare cache). Later it will be done again and actually programmed (1 frame ahead of time)
// return if too early
// TODO: still calculate FPGA table , but not load it if too early?
  int color, rslt;
  struct frameparspair_t pars_to_update[4]; // 4 needed
  int nupdate=0;
  struct {
    unsigned short scale;
    unsigned short hash16;
  } gamma32;
  unsigned long * pgamma32= (unsigned long *) & gamma32;
  for (color=0; color<4; color++) {

     if (get_locked_hash32(color)!=thispars->pars[P_GTAB_R+color]) { // modified for this color
       *pgamma32=thispars->pars[P_GTAB_R+color];
       rslt=set_gamma_table (gamma32.hash16,
    		                 gamma32.scale, NULL,
							 GAMMA_MODE_HARDWARE,
							 color,
							 sensor_port,
							 0); // frame16 - one ahead of the current do not lock yet TODO 393 multisensor - split gamma tables to subchannels
       if (rslt<=0) SETFRAMEPARS_SET(P_GTAB_R+color, get_locked_hash32(color)); // increases nupdate
     }
  }
  if (nupdate) {
     setFramePars(thispars, nupdate, pars_to_update);  // restore failed components
     MDF3(printk("had to restore back %d gamma tables (color components) \n",nupdate));
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
  struct {
    long left;
    long width;
    long top;
    long height;
  } hist_setup_data;
  struct frameparspair_t pars_to_update[4]={
   {P_HISTWND_LEFT, 0},
   {P_HISTWND_WIDTH, 0},
   {P_HISTWND_TOP, 0},
   {P_HISTWND_HEIGHT, 0}
  };
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
// calculate absolute window from the relative, apply limits
  hist_setup_data.width=  ((thispars->pars[P_HISTWND_RWIDTH] * thispars->pars[P_ACTUAL_WIDTH])>>16) & 0xffe;
  if (hist_setup_data.width<2) hist_setup_data.width=2;
  else if (hist_setup_data.width > thispars->pars[P_ACTUAL_WIDTH]) hist_setup_data.width = thispars->pars[P_ACTUAL_WIDTH];
  hist_setup_data.left=  ((thispars->pars[P_HISTWND_RLEFT] * (thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width)) >>16) & 0xffe;
  if (hist_setup_data.left> (thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width)) hist_setup_data.left = thispars->pars[P_ACTUAL_WIDTH]-hist_setup_data.width;

  hist_setup_data.height=  ((thispars->pars[P_HISTWND_RHEIGHT] * thispars->pars[P_ACTUAL_HEIGHT])>>16) & 0xffe;
  if (hist_setup_data.height<2) hist_setup_data.height=2;
  else if (hist_setup_data.height > thispars->pars[P_ACTUAL_HEIGHT]) hist_setup_data.height = thispars->pars[P_ACTUAL_HEIGHT];
  hist_setup_data.top=  ((thispars->pars[P_HISTWND_RTOP] * (thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height)) >>16) & 0xffe;
  if (hist_setup_data.top > (thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height)) hist_setup_data.top = thispars->pars[P_ACTUAL_HEIGHT]-hist_setup_data.height;

  if ((hist_setup_data.left   != thispars->pars[P_HISTWND_LEFT]) ||
      (hist_setup_data.width  != thispars->pars[P_HISTWND_WIDTH]) ||
      (hist_setup_data.top    != thispars->pars[P_HISTWND_TOP]) ||
      (hist_setup_data.height != thispars->pars[P_HISTWND_HEIGHT])) {
// set these values to FPGA
#ifdef NC353
      X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_LEFT,   hist_setup_data.left);
      X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_WIDTH,  hist_setup_data.width-2);
      X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_TOP,    hist_setup_data.top);
      X3X3_SEQ_SEND1(frame16,  X313_WA_HIST_HEIGHT, hist_setup_data.height-2);
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_HIST_LEFT,   (int) hist_setup_data.left));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_HIST_WIDTH,  (int) hist_setup_data.width-2));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_HIST_TOP,    (int) hist_setup_data.top));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_HIST_HEIGHT, (int) hist_setup_data.height-2));
      pars_to_update[0].val=hist_setup_data.left;
      pars_to_update[1].val=hist_setup_data.width;
      pars_to_update[2].val=hist_setup_data.top;
      pars_to_update[3].val=hist_setup_data.height;
     setFramePars(thispars, 4, pars_to_update);  // save intermediate/readonly parameters
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
  MDF3(printk(" frame16=%d\n",frame16));
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
  MDF3(printk(" frame16=%d\n",frame16));
  int y_coring_index;
  int c_coring_index;
  int composite_quality=(thispars->pars[P_QUALITY] & 0xff7f) | ((thispars->pars[P_PORTRAIT] & 1)<<7);
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
  if (thispars->pars[P_CORING_INDEX]!= prevpars->pars[P_CORING_INDEX]) {
     y_coring_index= thispars->pars[ P_CORING_INDEX]      & 0xffff;
     c_coring_index=(thispars->pars[ P_CORING_INDEX]>>16) & 0xffff;
     if (c_coring_index==0) c_coring_index=y_coring_index;
     set_coring_fpga(y_coring_index, 0); 
     set_coring_fpga(c_coring_index, 1);
  }

// calculate quality tables - find already programmed FPGA page or calculates/programms a new one
// set_qtable_fpga returns table page (0..7) or -1 - invalid q

  if ((thispars->pars[P_COMPMOD_QTAB]=set_qtable_fpga(composite_quality))>=0) {
//    X3X3_SEQ_SEND1(frame16,  X313_WA_COMP_CMD,   COMPCMD_QTAB(thispars->pars[P_COMPMOD_QTAB]));
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_COMP_CMD, (int) COMPCMD_QTAB(thispars->pars[P_COMPMOD_QTAB])));
    return 0;
  } else return -1;
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
  int ntilex,ntiley,goodEOL,padlen, imgsz,sa;
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
///programm channel1 (FPN). Will not enable if not needed (imageParamsR[P_FPN]==0)
  ntilex=((thispars->pars[P_ACTUAL_WIDTH]+X313_MARGINS+7)>>3);
  ntiley=thispars->pars[P_ACTUAL_HEIGHT]+(((thispars->pars[P_PF_HEIGHT] & 0xffff)>0)?0:X313_MARGINS);
  MDF3(printk("ntilex=0x%x ntiley=0x%x\n",ntilex,ntiley));
  if ((thispars->pars[P_PF_HEIGHT] & 0xffff)==0) { // not a photofinish
    if(!thispars->pars[P_BGFRAME] && ((thispars->pars[P_FPNS]!=0) || (thispars->pars[P_FPNM]!=0))) {
// program memory channel1
#ifdef NC353
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL1, X313_SDCHAN_REG1(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL2, X313_SDCHAN_REG2(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH1_CTL0, X313_SDCHAN_REG0(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1)));
// enable channel1 for reading SDRAM
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(1)); // wait ready later... ???
#endif
       MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH1_CTL1, (int) X313_SDCHAN_REG1(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1))));
       MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH1_CTL2, (int) X313_SDCHAN_REG2(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1))));
       MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH1_CTL0, (int) X313_SDCHAN_REG0(0,0,0, X313_MAP_FPN, (ntilex-1), (ntiley-1))));
       MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(1)));

    } else  {
#ifdef NC353
        X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_DIS_D(1));
#endif
        MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_DIS_D(1)));
    }
  } else  {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_DIS_D(1));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_DIS_D(1)));
  }
// Program channel 0 (sensor->memory)
//       goodEOL=0; // last 8/16 blocks of pixels in each scanline are bad (only 2 are actually written)
// if 8-bit mode we'll need to update ntilex. fpga tries to write 2 bytes more (but not crossing the page boundaries
// GoodEOL - if image width is multiple of 512 pixels 1 extra block (16 pixels) needs to be written to memory (and padlen will be more by 512 bytes (256 words)
// due to FPGA controller implementation (it writes extra 4 pixels for the margins, but not when it crosses 512 byte boundary)
// When reading 20x20 macroblocks to the compressor, such exception is not needed, it crosses page boundaries when needed

  if ((thispars->pars[P_BITS]==8) && (!thispars->pars[P_BGFRAME])) { // in 16-bit mode ntilex will stay the same
    ntilex=((thispars->pars[P_ACTUAL_WIDTH]+X313_MARGINS+15)>>4);
    goodEOL=((thispars->pars[P_ACTUAL_WIDTH] & 0x0f) == 0) && ((thispars->pars[P_ACTUAL_WIDTH] & 0x1f0) != 0);
    if (goodEOL) ntilex--;
  MDF3(printk("ntilex=0x%x ntiley=0x%x goodEOL=0x%x\n",ntilex,ntiley,goodEOL));
  }
  MDF3(printk("ntilex=0x%x ntiley=0x%x\n",ntilex,ntiley));

//       if (imageParamsR[P_OVERLAP]>=(imageParamsR[P_ACTUAL_HEIGHT]+2)) imageParamsR[P_OVERLAP]=imageParamsR[P_ACTUAL_HEIGHT]+1; rotten code, left as a comment
  if (thispars->pars[P_OVERLAP]>0) ntiley=(ntiley<<1); // ntiley will be twice bigger for synch. mode)
  padlen=((ntilex+31)>>5) << 8;
//TODO:fix it to be able to use two (or larger) frame buffer
//  imgsz=((padlen * (thispars->pars[P_ACTUAL_HEIGHT]+X313_MARGINS) * thispars->pars[P_PAGE_ACQ]) << ((thispars->pars(P_TRIG) & 1)?1:0)); // mostly rotten too
  imgsz=padlen * ntiley;
  MDF3(printk("imgsz=0x%x, padlen=0x%x\n",imgsz,padlen));
  if (thispars->pars[P_IMGSZMEM]!= imgsz)  setFramePar(thispars, P_IMGSZMEM, imgsz);  // set it (and propagate to the later frames)
#ifdef NC353
  sa=X313_MAP_FRAME + (imgsz * thispars->pars[P_PAGE_ACQ]);  // now - always X313_MAP_FRAME
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL1, X313_SDCHAN_REG1(0,1,1, sa, (ntilex-1), (ntiley-1)));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH0_CTL1, (int) X313_SDCHAN_REG1(0,1,1, sa, (ntilex-1), (ntiley-1))  ));
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL2, X313_SDCHAN_REG2(0,1,1, sa, (ntilex-1), (ntiley-1)));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH0_CTL2, (int) X313_SDCHAN_REG2(0,1,1, sa, (ntilex-1), (ntiley-1))  ));
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH0_CTL0, X313_SDCHAN_REG0(0,1,1, sa, (ntilex-1), (ntiley-1))); // dependency - source
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH0_CTL0, (int) X313_SDCHAN_REG0(0,1,1, sa, (ntilex-1), (ntiley-1))  ));
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(0));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(0)  ));
#endif
// number of scan lines to read from sensor - program in pgm_sensorin
  return 0;
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
  int ntilex,ntiley,sa,pf;
//  struct frameparspair_t * pars_to_update[4]={
  struct frameparspair_t pars_to_update[4]={
   {P_SDRAM_CHN20, 0},
   {P_SDRAM_CHN21, 0},
   {P_SDRAM_CHN22, 0},
   {P_TILES, 0}
  };
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
  ntilex=((thispars->pars[P_ACTUAL_WIDTH]+X313_MARGINS-1)>>4);
  ntiley=thispars->pars[P_ACTUAL_HEIGHT]; // number of lines it the whole frame
#ifdef NC353
  sa=X313_MAP_FRAME + ( thispars->pars[P_IMGSZMEM] * thispars->pars[P_PAGE_READ]); // now - always X313_MAP_FRAME
#endif
  pf=((thispars->pars[P_PF_HEIGHT] & 0xffff)>0)?1:0; // when mode==1, wnr means "photofinish" in fpga
  int depend=((thispars->pars[P_SENSOR_RUN] & 3)==SENSOR_RUN_STOP) ? 0 : 1;
  MDF23(printk("ntilex=0x%x ntiley=0x%x sa=0x%x pf=0x%x depend=%x\n",ntilex,ntiley,sa,pf,depend));
// will be programmed with "depend==1", so it will not be possible to re-read from memory this way
  MDF9(printk(" thispars->pars[P_SENSOR_RUN]=0x%x,  depend=%d)\n", (int)thispars->pars[P_SENSOR_RUN], depend));
  pars_to_update[1].val=X313_SDCHAN_REG1(1,pf,depend, sa, (ntilex-1), (ntiley-16));
  pars_to_update[2].val=X313_SDCHAN_REG2(1,pf,depend, sa, (ntilex-1), (ntiley-16));
  pars_to_update[0].val=X313_SDCHAN_REG0(1,pf,depend, sa, (ntilex-1), (ntiley-16));
  pars_to_update[3].val=ntilex*(ntiley>>4);

  setFramePars(thispars, 4, pars_to_update);
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (!jpeg_htable_is_programmed()) jpeg_htable_fpga_pgm ();
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
  int comp_cmd=0;
  struct frameparspair_t pars_to_update[4]; // 2 needed, increase if more entries will be added
  int nupdate=0;

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
  MDF3(printk("comp_cmd=0x%x\n",comp_cmd));
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
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, comp_cmd);
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_COMP_CMD, (int) comp_cmd));
  } else {
    MDF3(printk("  comp_cmd==0, does not need to be sent\n"));
  }
// color saturation changed?
  if (FRAMEPAR_MODIFIED(P_COLOR_SATURATION_BLUE) || FRAMEPAR_MODIFIED(P_COLOR_SATURATION_RED)) {
    int csb=(thispars->pars[P_COLOR_SATURATION_BLUE]* DEFAULT_COLOR_SATURATION_BLUE)/100;
    int csr=(thispars->pars[P_COLOR_SATURATION_RED] * DEFAULT_COLOR_SATURATION_RED)/100;
    if (unlikely(csb>1023)) {
       csb=102300/DEFAULT_COLOR_SATURATION_BLUE;
       SETFRAMEPARS_SET(P_COLOR_SATURATION_BLUE, csb);
    }
    if (unlikely(csr>1023)) {
       csr=102300/DEFAULT_COLOR_SATURATION_RED;
       SETFRAMEPARS_SET(P_COLOR_SATURATION_RED, csr);
    }
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COLOR_SAT, ((DEFAULT_COLOR_SATURATION_BLUE*thispars->pars[P_COLOR_SATURATION_BLUE])/100) |
                               (((DEFAULT_COLOR_SATURATION_RED *thispars->pars[P_COLOR_SATURATION_RED])/100)<<12));
#endif
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%lx)\n", (int)fpga_addr, (int) X313_WA_COLOR_SAT, (int)((DEFAULT_COLOR_SATURATION_BLUE*thispars->pars[P_COLOR_SATURATION_BLUE])/100) | (((DEFAULT_COLOR_SATURATION_RED *thispars->pars[P_COLOR_SATURATION_RED])/100)<<12)));


  }
// compressor quantizer zero bin mode changed?
// Quantizer tuning - bits 0..7 - zero bin, 15:8 - quantizer bias
  if (FRAMEPAR_MODIFIED(P_CORING_PAGE)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_QUANTIZER_MODE,thispars->pars[P_CORING_PAGE]);
#endif
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_QUANTIZER_MODE, (int)thispars->pars[P_CORING_PAGE]));
  }

  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
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
  struct {
    long left;
    long right;
    long top;
    long bottom;
    long totalwidth;
    long filter_no;
    long show1;
  } focus_setup_data;
//  struct frameparspair_t * pars_to_update[5]={
  struct frameparspair_t pars_to_update[5]={
   {P_FOCUS_TOTWIDTH, 0},
   {P_FOCUS_LEFT, 0},
   {P_FOCUS_WIDTH, 0},
   {P_FOCUS_TOP, 0},
   {P_FOCUS_HEIGHT, 0}
  };
  MDF3(printk(" frame16=%d\n",frame16));
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
     fpga_table_write_nice (CX313_FPGA_TABLES_FOCUSPARS, sizeof(focus_setup_data)/sizeof(focus_setup_data.left), (unsigned long *) &focus_setup_data);
     pars_to_update[0].val=focus_setup_data.totalwidth;
     pars_to_update[1].val=focus_setup_data.left;
     pars_to_update[2].val=focus_setup_data.right-focus_setup_data.left+8;
     pars_to_update[3].val=focus_setup_data.top;
     pars_to_update[4].val=focus_setup_data.bottom-focus_setup_data.top+8;

     setFramePars(thispars, 5, pars_to_update);  // save intermediate/readonly parameters
  } 
  return 0;
}
/** Program trigger generator/external synchronization
 * TODO: 393 reimplement
 * Was for 353: can not use sequencer as data is more than 24 bit wide */
int pgm_trigseq    (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
  struct frameparspair_t pars_to_update[10]; // ??? needed, increase if more entries will be added
  int nupdate=0;
  int d;
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
  if (frame16 >= 0) return -1; // ASAP only mode
// Trigger condition changed? (0 - internal sequencer)
  if (FRAMEPAR_MODIFIED(P_TRIG_CONDITION)) {
#ifdef NC353
    port_csp0_addr[X313_WA_CAMSYNCTRIG] = thispars->pars[P_TRIG_CONDITION];
#endif
    MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_CAMSYNCTRIG, (int)thispars->pars[P_TRIG_CONDITION]));
  }
// Trigger delay changed?
  if (FRAMEPAR_MODIFIED(P_TRIG_DELAY)) {
#ifdef NC353
    port_csp0_addr[X313_WA_CAMSYNCDLY] = thispars->pars[P_TRIG_DELAY];
#endif
    MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_CAMSYNCDLY, (int) thispars->pars[P_TRIG_DELAY]));
  }
// Sequencer output word changed? (to which outputs it is sent and what polarity)
  if (FRAMEPAR_MODIFIED(P_TRIG_OUT)) {
#ifdef NC353
    port_csp0_addr[X313_WA_CAMSYNCOUT] = thispars->pars[P_TRIG_OUT];
#endif
    MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_CAMSYNCOUT, (int) thispars->pars[P_TRIG_OUT]));
    // Enable connection from the trigger module to the FPGA GPIO pins
    if (thispars->pars[P_TRIG_OUT]!=0) {
#ifdef NC353
      port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_EN_TRIG_OUT;
#endif
      MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_IOPINS, (int) X313_WA_IOPINS_EN_TRIG_OUT));
    } else {
//  Not needed, I think
//      port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_DIS_TRIG_OUT;
//      MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_IOPINS, (int) X313_WA_IOPINS_DIS_TRIG_OUT));
    }
  }
// Sequencer period changed? (0 - stopped, 1 - single trigger, >=256 - start repetitive)
  if (FRAMEPAR_MODIFIED(P_TRIG_PERIOD)) {
    if (unlikely((thispars->pars[P_TRIG_PERIOD] > 1) && (thispars->pars[P_TRIG_PERIOD] < 256))) { // Wrong value, restore old one
       SETFRAMEPARS_SET(P_TRIG_PERIOD,prevpars->pars[P_TRIG_PERIOD]);
    } else {
#ifdef NC353
      port_csp0_addr[X313_WA_CAMSYNCPER] = thispars->pars[P_TRIG_PERIOD];
#endif
      MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_CAMSYNCPER, (int)thispars->pars[P_TRIG_PERIOD]));
    }
  }
// Bit length changed or not yet initialized?
  if (FRAMEPAR_MODIFIED(P_TRIG_BITLENGTH) || (thispars->pars[P_TRIG_BITLENGTH]==0)) {
    d=thispars->pars[P_TRIG_BITLENGTH];
    if (unlikely((d<2) || (d>255))) { // Wrong value, restore old one
      d=P_TRIG_BITLENGTH_DEFAULT;
      SETFRAMEPARS_SET(P_TRIG_BITLENGTH,d);
    }
#ifdef NC353
    port_csp0_addr[X313_WA_CAMSYNCPER] = d;
#endif
    MDF3(printk("writing bit length-1:  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_CAMSYNCPER, d));
  }
// P_EXTERN_TIMESTAMP changed? (0 - internal sequencer)
  if (FRAMEPAR_MODIFIED(P_EXTERN_TIMESTAMP)) {
#ifdef NC353
    port_csp0_addr[X313_WA_DCR1]=X353_DCR1(EXTERNALTS,thispars->pars[P_EXTERN_TIMESTAMP]?1:0);
#endif
    MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_DCR1, (int)X353_DCR1(EXTERNALTS,thispars->pars[P_EXTERN_TIMESTAMP]?1:0)));
  }
// P_XMIT_TIMESTAMP changed? (0 - internal sequencer)
  if (FRAMEPAR_MODIFIED(P_XMIT_TIMESTAMP)) {
#ifdef NC353
    port_csp0_addr[X313_WA_DCR1]=X353_DCR1(OUTPUTTS,thispars->pars[P_XMIT_TIMESTAMP]?1:0);
#endif
    MDF3(printk("  port_csp0_addr[0x%x]=0x%x\n",  (int) X313_WA_DCR1, (int)X353_DCR1(OUTPUTTS,thispars->pars[P_XMIT_TIMESTAMP]?1:0)));
  }
  if (nupdate)  setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
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
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SMART_IRQ, (int) ( (2 | ((thispars->pars[P_IRQ_SMART] & 1)?1:0)) | \
                                                  (8 | ((thispars->pars[P_IRQ_SMART] & 2)?4:0)))));

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
  MDF3(printk(" frame16=%d, safe=%d, async=%d, nooverlap=%d\n",frame16, safe, async, nooverlap));
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
 * TODO: 393 - reimplement */
int pgm_comprestart(int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error

{
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
// does it need to be be started (nothing do be done to stop)
  if (thispars->pars[P_COMPRESSOR_RUN]==0) return 0; // does not need comporessor to be started
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
  // reset memory controller for the channel2 to the start of the frame
#ifdef NC353
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL1,   thispars->pars[P_SDRAM_CHN21]);
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL2,   thispars->pars[P_SDRAM_CHN22]);
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL0,   thispars->pars[P_SDRAM_CHN20]);
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH2_CTL1,   (int) thispars->pars[P_SDRAM_CHN21]));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH2_CTL2,   (int) thispars->pars[P_SDRAM_CHN22]));
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SDCH2_CTL0,   (int) thispars->pars[P_SDRAM_CHN20]));

// enable memory channel2
#ifdef NC353
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(0));
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_SD_MODE, (int) X313_CHN_EN_D(0)));
// set number of tiles to compressor
#ifdef NC353
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_MCUNUM, thispars->pars[P_TILES]-1);
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_MCUNUM, (int) thispars->pars[P_TILES]-1));
// start the compressor
#ifdef NC353
  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, (thispars->pars[P_COMPRESSOR_RUN]==2) ? COMPCMD_RUN : COMPCMD_SINGLE);
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_COMP_CMD, (int) ((thispars->pars[P_COMPRESSOR_RUN]==2) ? COMPCMD_RUN : COMPCMD_SINGLE)));
  return 0;
}

/**
 * @brief stop compressor when changing geometry
 * TODO: 353 - reimplement
 */
int pgm_compstop   (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
  MDF3(printk(" frame16=%d\n",frame16));
//  if (frame16 & ~PARS_FRAMES_MASK) return -1; // wrong frame (can be only -1 or 0..7)
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame (can be only -1 or 0..7)
#ifdef NC353
  X3X3_SEQ_SEND1(fpga_addr, X313_WA_COMP_CMD, COMPCMD_STOP);
#endif
  MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int)  X313_WA_COMP_CMD, (int) COMPCMD_STOP));
  return 0;
}

/** Compressor control: only start/stop/single (after explicitly changed, not when geometry was changed)
 * TODO: 393 - reimplement*/
int pgm_compctl    (int sensor_port,               ///< sensor port number (0..3)
					struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
					struct framepars_t * thispars, ///< sensor current parameters
					struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
					int frame16)                   ///< 4-bit (hardware) frame number parameters should
												   ///< be applied to,  negative - ASAP
												   ///< @return OK - 0, <0 - error
{
  MDF3(printk(" frame16=%d, prevpars->pars[P_COMPRESSOR_RUN]=%d, thispars->pars[P_COMPRESSOR_RUN]=%d \n",frame16, (int) prevpars->pars[P_COMPRESSOR_RUN], (int) thispars->pars[P_COMPRESSOR_RUN]));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
  if ((prevpars->pars[P_COMPRESSOR_RUN]==0) && (thispars->pars[P_COMPRESSOR_RUN]!=0)) { // just started
// reset memory controller for the channel2 to the start of the frame
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL1,   thispars->pars[P_SDRAM_CHN21]);
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL2,   thispars->pars[P_SDRAM_CHN22]);
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SDCH2_CTL0,   thispars->pars[P_SDRAM_CHN20]);
#endif

// enable memory channel2 (NOTE: wnen is it disabled? does it need to be disabled?)
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_SD_MODE, X313_CHN_EN_D(2));
#endif
// set number of tiles to compressor
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_MCUNUM, thispars->pars[P_TILES]-1);
#endif
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_SDCH2_CTL1, (int)thispars->pars[P_SDRAM_CHN21]));
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_SDCH2_CTL2, (int)thispars->pars[P_SDRAM_CHN22]));
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_SDCH2_CTL0, (int)thispars->pars[P_SDRAM_CHN20]));
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_SD_MODE, (int) X313_CHN_EN_D(2)));
    MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int)X313_WA_MCUNUM, (int)(thispars->pars[P_TILES]-1)));
  }
  if ((prevpars->pars[P_COMPRESSOR_RUN] != thispars->pars[P_COMPRESSOR_RUN]) || (thispars->pars[P_COMPRESSOR_RUN]==COMPRESSOR_RUN_SINGLE))  // changed or single
    switch (thispars->pars[P_COMPRESSOR_RUN]) {
     case COMPRESSOR_RUN_STOP:
#ifdef NC353
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_STOP);
#endif
       MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_STOP));
      break;
     case COMPRESSOR_RUN_SINGLE:
#ifdef NC353
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_SINGLE);
#endif
       if (!x313_is_dma_on()) x313_dma_start();
       MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_SINGLE));
       break;
     case COMPRESSOR_RUN_CONT:
#ifdef NC353
       X3X3_SEQ_SEND1(fpga_addr,  X313_WA_COMP_CMD, COMPCMD_RUN);
#endif
       if (!x313_is_dma_on()) x313_dma_start();
       MDF9(printk(" X3X3_SEQ_SEND1(0x%x,  0x%x,  0x%x)\n", (int)fpga_addr, (int) X313_WA_COMP_CMD, (int) COMPCMD_RUN));
       break;
    }
  return 0;
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
  struct frameparspair_t pars_to_update[4]; // 4 needed, increase if more entries will be added
  int nupdate=0;
  MDF3(printk(" frame16=%d, (getThisFrameNumber() & PARS_FRAMES_MASK)= %ld, thispars->pars[P_GTAB_R]=0x%lx, thispars->pars[P_FRAME]=0x%lx\n",frame16, getThisFrameNumber() & PARS_FRAMES_MASK, thispars->pars[P_GTAB_R],thispars->pars[P_FRAME]));

#if ELPHEL_DEBUG
  struct framepars_t * nextspars=&framepars[(thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK];
#endif
  MDF3(printk(" nextframe=%d, nextsparsd, nextspars->pars[P_GTAB_R]=0x%lx, nextspars->pars[P_FRAME]=0x%lx\n",(int) ((thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK), nextspars->pars[P_GTAB_R], nextspars->pars[P_FRAME]));
  MDF16(printk(" nextframe=%d, nextspars->pars[P_GTAB_R]=0x%lx, nextspars->pars[P_FRAME]=0x%lx\n",(int) ((thispars->pars[P_FRAME]+1) & PARS_FRAMES_MASK), nextspars->pars[P_GTAB_R], nextspars->pars[P_FRAME]));
///NOTE: Yes, ASAP, but - 1 frame ahead
  if (frame16 >= 0) return -1; // only can work in ASAP mode
  int color, rslt;
  struct {
    unsigned short scale;
    unsigned short hash16;
  } gamma32;
  unsigned long * pgamma32= (unsigned long *) & gamma32;
  unsigned long *gtable;
  int need_pgm=0;
  for (color=0; color<4; color++) if (get_locked_hash32(color)!=thispars->pars[P_GTAB_R+color]) need_pgm++;
// code currently does not allow to overwrite just 1 table - only all 4
  if (need_pgm) {
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
      if ((gtable= get_gamma_fpga(color))) fpga_table_write_nice (CX313_FPGA_TABLES_GAMMA + (color * 256), 256, gtable);
      if (rslt <= 0) SETFRAMEPARS_SET(P_GTAB_R+color, get_locked_hash32(color)); // restore to the locked table
    }
    MDF3(printk("need_pgm=%d, get_locked_hash32(*)=0x%lx 0x%lx 0x%lx 0x%lx\n",need_pgm,get_locked_hash32(0),get_locked_hash32(1),get_locked_hash32(2),get_locked_hash32(3)));
  }
  if (nupdate)  {
     setFramePars(thispars, nupdate, pars_to_update);  // save changes, schedule functions
     MDF3(printk("had to restore back %d gamma tables (color components) \n",nupdate));
     return -1;
  }
  return 0;
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
  MDF3(printk(" frame16=%d\n",frame16));
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
  MDF3(printk("frame16=%d\n",frame16)); // nothing here, all in multisensor.c
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
  MDF3(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
#ifdef NC353
  int fpga_addr=(frame16 <0) ? X313_SEQ_ASAP : (X313_SEQ_FRAME0+frame16);
#endif
  if (FRAMEPAR_MODIFIED(P_VIGNET_AX)) {
#ifdef NC353
	  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_AX(thispars->pars[P_VIGNET_AX]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_AX(thispars->pars[P_VIGNET_AX])));
  }
  if (FRAMEPAR_MODIFIED(P_VIGNET_AY)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_AY(thispars->pars[P_VIGNET_AY]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_AY(thispars->pars[P_VIGNET_AY])));
  }
  if (FRAMEPAR_MODIFIED(P_VIGNET_C)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_C(thispars->pars[P_VIGNET_C]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_C(thispars->pars[P_VIGNET_C])));
  }
  if (FRAMEPAR_MODIFIED(P_VIGNET_BX)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_BX(thispars->pars[P_VIGNET_BX]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_BX(thispars->pars[P_VIGNET_BX])));
  }
  if (FRAMEPAR_MODIFIED(P_VIGNET_BY)) {
#ifdef NC353
	  X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_BY(thispars->pars[P_VIGNET_BY]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_BY(thispars->pars[P_VIGNET_BY])));
  }
  if (FRAMEPAR_MODIFIED(P_SCALE_ZERO_IN)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_FATZERO_IN(thispars->pars[P_SCALE_ZERO_IN]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_FATZERO_IN(thispars->pars[P_SCALE_ZERO_IN])));
  }
  if (FRAMEPAR_MODIFIED(P_SCALE_ZERO_OUT)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_FATZERO_OUT(thispars->pars[P_SCALE_ZERO_OUT]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_FATZERO_OUT(thispars->pars[P_SCALE_ZERO_OUT])));
  }
  if (FRAMEPAR_MODIFIED(P_VIGNET_SHL)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_POSTSCALE(thispars->pars[P_VIGNET_SHL]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_POSTSCALE(thispars->pars[P_VIGNET_SHL])));
  }
  if (FRAMEPAR_MODIFIED(P_DGAINR)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(0, thispars->pars[P_DGAINR]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(0,thispars->pars[P_DGAINR])));
  }
  if (FRAMEPAR_MODIFIED(P_DGAING)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(1, thispars->pars[P_DGAING]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(1,thispars->pars[P_DGAING])));
  }
  if (FRAMEPAR_MODIFIED(P_DGAINGB)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(2, thispars->pars[P_DGAINGB]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(2,thispars->pars[P_DGAINGB])));
  }
  if (FRAMEPAR_MODIFIED(P_DGAINB)) {
#ifdef NC353
    X3X3_SEQ_SEND1(fpga_addr,  X313_WA_LENSCORR, X313_LENS_SCALES(3, thispars->pars[P_DGAINB]));
#endif
    MDF3(printk("  X3X3_SEQ_SEND1(0x%x,0x%x, 0x%x)\n", fpga_addr,  (int) X313_WA_LENSCORR, (int)X313_LENS_SCALES(3,thispars->pars[P_DGAINB])));
  }
  return 0;
}
