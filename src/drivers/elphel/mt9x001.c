/***************************************************************************//**
 * @file      mt9x001.c
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
/*----------------------------------------------------------------------------*
 *!  $Log: mt9x001.c,v $
 *!  Revision 1.19  2011/07/30 23:19:40  elphel
 *!  comment typo
 *!
 *!  Revision 1.18  2010/07/20 20:13:34  elphel
 *!  8.0.8.33 - added MakerNote info for composite images made with multisensor cameras (with 10359 board)
 *!
 *!  Revision 1.17  2010/06/02 16:31:04  elphel
 *!  Added P_MULTI_SELECTED parameter
 *!
 *!  Revision 1.16  2010/05/29 22:16:09  elphel
 *!  modified source of window height in pgm_limitfps()
 *!
 *!  Revision 1.15  2010/05/28 16:20:42  elphel
 *!  changes related to new FPGA code version
 *!
 *!  Revision 1.14  2010/05/25 00:52:23  elphel
 *!  8.0.8.20, working on multi-sensor
 *!
 *!  Revision 1.13  2010/05/17 16:03:46  elphel
 *!  8.0.8.15, working on multisensor support
 *!
 *!  Revision 1.12  2010/05/16 02:03:47  elphel
 *!  8.0.8.4 - driver working with individual/broadcast sensor registers
 *!
 *!  Revision 1.11  2010/05/13 03:39:31  elphel
 *!  8.0.8.12 -  drivers modified for multi-sensor operation
 *!
 *!  Revision 1.10  2010/04/28 02:32:33  elphel
 *!  modified processing senor phase adjustment, now there are 3 time values used - senor internal, FPGA (bitstream dependent) and cable length-dependent
 *!
 *!  Revision 1.9  2010/04/06 20:35:42  elphel
 *!  8.0.7.5 - made the fpgaclock driver program 10359 clock in addition to the system one
 *!
 *!  Revision 1.8  2010/01/27 22:51:52  elphel
 *!  turned off ELPHEL_DEBUG, fixed errors caused by that.
 *!
 *!  Revision 1.7  2009/04/07 03:05:00  elphel
 *!  Fixed Bayer shift for 1.3MPix sensor
 *!
 *!  Revision 1.6  2008/12/08 21:55:02  elphel
 *!  making 3MPix work, minor cleanup
 *!
 *!  Revision 1.5  2008/12/03 17:17:35  elphel
 *!  comment typo
 *!
 *!  Revision 1.4  2008/12/01 02:32:17  elphel
 *!  Added FIXME
 *!
 *!  Revision 1.3  2008/11/30 21:56:39  elphel
 *!  Added enforcing limit on the overall gains in the color channels, storage of exposure and gains in the histograms cache (to be used with autoexposure/white balance)
 *!
 *!  Revision 1.2  2008/11/30 05:01:03  elphel
 *!  Changing gains/scales behavior
 *!
 *!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
 *!
 *!
 *!  Revision 1.24  2008/11/17 06:43:32  elphel
 *!  bug fix (wrong parameter name)
 *!
 *!  Revision 1.23  2008/11/15 07:03:43  elphel
 *!  sensor gains now modify parameters to match sensor capabilities, changed truncating to rounding
 *!
 *!  Revision 1.22  2008/10/29 04:18:28  elphel
 *!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
 *!
 *!  Revision 1.21  2008/10/22 05:27:55  elphel
 *!  bug fix ("||" was instead of "|"), removed some unused debug output
 *!
 *!  Revision 1.20  2008/10/22 03:47:16  elphel
 *!  added P_VIRT_KEEP parameter to enable VIRT_WIDTH, VIRT_HEIGHT preservation (if !=0), otherwise these parameters are set to minimalallowed by sensor an FPS limit
 *!
 *!  Revision 1.19  2008/10/20 18:45:07  elphel
 *!  just more debug printk
 *!
 *!  Revision 1.18  2008/10/18 06:14:21  elphel
 *!  8.0.alpha4 - removed some obsolete parameters, renumbered others, split P_FLIP into P_FLIPH and P_FLIPV (different latencies because of bad frames), pgm_window-> pgm_window, pgm_window_safe
 *!
 *!  Revision 1.17  2008/10/12 06:13:10  elphel
 *!  snapshot
 *!
 *!  Revision 1.16  2008/10/11 18:46:07  elphel
 *!  snapshot
 *!
 *!  Revision 1.15  2008/10/08 21:26:25  elphel
 *!  snapsot 7.2.0.pre4 - first images (actually - second)
 *!
 *!  Revision 1.14  2008/10/06 08:31:08  elphel
 *!  snapshot, first images
 *!
 *!  Revision 1.13  2008/10/04 16:10:12  elphel
 *!  snapshot
 *!
 *!  Revision 1.12  2008/09/28 00:31:57  elphel
 *!  snapshot
 *!
 *!  Revision 1.11  2008/09/25 00:58:11  elphel
 *!  snapshot
 *!
 *!  Revision 1.10  2008/09/22 22:55:48  elphel
 *!  snapshot
 *!
 *!  Revision 1.9  2008/09/12 00:23:59  elphel
 *!  removed cc353.c, cc353.h
 *!
 *!  Revision 1.8  2008/09/11 01:05:32  elphel
 *!  snapshot
 *!
 *!  Revision 1.7  2008/09/04 17:37:13  elphel
 *!  documenting
 *!
 *!  Revision 1.6  2008/08/11 19:17:01  elphel
 *!  reduced syntax complaints by KDevelop
 *!
 *!  Revision 1.5  2008/07/27 23:25:07  elphel
 *!  next snapshot
 *!
 *!  Revision 1.4  2008/07/27 04:27:49  elphel
 *!  next snapshot
 *!
 *!  Revision 1.3  2008/06/20 03:54:19  elphel
 *!  another snapshot
 *!
 *!  Revision 1.2  2008/06/19 02:17:36  elphel
 *!  continuing work - just a snapshot
 *!
 *!  Revision 1.9  2008/05/02 15:13:35  elphel
 *!  switch to hardware i2c write,added related sensor parameters
 *!
 *!  Revision 1.8  2008/05/01 01:28:57  elphel
 *!  hardware i2c control - macros, variables
 *!
 *!  Revision 1.7  2008/03/20 22:29:30  elphel
 *!  added trigger-related code and parameters
 *!
 *!  Revision 1.6  2008/02/12 21:53:20  elphel
 *!  Modified I2c to support multiple buses, added raw access (no address registers) and per-slave protection bitmasks
 *!
 *!  Revision 1.5  2008/02/05 18:28:59  spectr_rain
 *!  correct timing for MT9P sensor with binning and skipping 1,2,4
 *!
 *!  Revision 1.4  2007/12/27 14:27:43  spectr_rain
 *!  fixed FPS, correct work with the skipping - TODO - check the binning
 *!
 *!  Revision 1.3  2007/10/16 23:17:51  elphel
 *!  cosmetic
 *!

//  Revision 1.2  2007/10/16 20:03:38  elphel
//  cosmetic
//
//  Revision 1.1.1.1  2007/09/30 03:19:56  elphel
//  This is a fresh tree based on elphel353-2.10
//
//  Revision 1.12  2007/09/30 03:19:56  elphel
//  Cleanup, fixed broken acquisition of individual JPEG images into circbuf (in mode 7)
//
//  Revision 1.11  2007/09/19 04:22:33  elphel
//  fixed debug output
//
//  Revision 1.10  2007/09/19 00:34:21  elphel
//  support for frame rate limiting, disabled extra debug  messages
//
//  Revision 1.9  2007/09/18 06:09:22  elphel
//  support for merged P_FLIP, support for P_FPSLM (fps limit mode) that allow to limit FPS (upper limit) and maintain FPS (lower limit) preventing exposure time to exceed one divided by fps limit
//
//  Revision 1.8  2007/09/12 18:05:35  spectr_rain
//  *** empty log message ***
//
//  Revision 1.7  2007/08/17 10:23:19  spectr_rain
//  switch to GPL3 license
//
//  Revision 1.6  2007/07/20 10:17:46  spectr_rain
//  *** empty log message ***
//
//  Revision 1.10  2007/07/09 22:05:43  elphel
//  minor sensor phase adjustment (constant so far)
//
//  Revision 1.9  2007/07/09 21:10:14  elphel
//  Reducing EMI from the sensor front end cable by reducing the drive strengths on the lines. Improving phase adjustment (and phase error measurements) for the sensors. Dealing with the delayed (with respect to pixel data) line/frame valid signals in MT9P001 sensors
//
//  Revision 1.8  2007/07/09 05:15:17  elphel
//  set slowest speed on the outputs to reduce EMI
//
//  Revision 1.7  2007/06/28 02:24:40  elphel
//  Increased default frequency for 5MPix sensor to 96MHz
//
//  Revision 1.6  2007/06/18 07:57:24  elphel
//  Fixed bug working with MT9P031 - added sensor reset/reinit after frequency change
//
//  Revision 1.5  2007/05/21 17:45:11  elphel
//  boundary scan support, added 359/347 detection
//
//  Revision 1.4  2007/04/23 22:48:32  spectr_rain
//  *** empty log message ***
//
//  Revision 1.3  2007/04/17 18:28:06  spectr_rain
//  *** empty log message ***
//
//  Revision 1.2  2007/04/04 03:55:22  elphel
//  Improved i2c, added i2c as character devices (to use from php)
//
//  Revision 1.1.1.1  2007/02/23 10:11:48  elphel
//  initial import into CVS
//
//  Revision 1.12  2006/12/01 00:18:12  spectr_rain
//  *** empty log message ***
//
//  Revision 1.11  2006/11/28 18:33:26  spectr_rain
//  *** empty log message ***
//
//  Revision 1.9  2006/11/01 18:49:57  spectr_rain
//  *** empty log message ***
//
//  Revision 1.8  2006/09/12 15:21:55  spectr_rain
//  use ve for integration time if e == 0
//
//  Revision 1.7  2006/09/02 22:04:17  spectr_rain
//  *** empty log message ***
//
//  Revision 1.6  2006/09/02 00:19:49  spectr_rain
//  lock sensor while readrawimage
//
//  Revision 1.5  2006/07/17 12:22:42  spectr_rain
//  enable autoexposition
//
//  Revision 1.4  2006/07/13 04:21:15  elphel
//  mt9p031 now wants the same bayer as others - maybe DLYHOR bit was set earlier?
//
//  Revision 1.3  2006/07/12 19:32:49  spectr_rain
//  fix Bayer pattern for less than 5MPx sensors
//
//  Revision 1.2  2006/07/12 06:03:16  elphel
//  bug fix
//
//  Revision 1.1.1.1  2006/07/11 19:15:00  spectr_rain
//  unwork with less than 5MPx Micron sensor, initial branch
//
//  Revision 1.25  2006/06/05 07:31:22  spectr_rain
//  set blank vertical to 0 if pfh > 0
//
//  Revision 1.24  2006/02/16 03:37:06  elphel
//  *** empty log message ***
//
//  Revision 1.23  2006/02/08 01:53:56  spectr_rain
//  fix broken mutex in check of down_interruptible
//
//  Revision 1.22  2006/01/15 22:24:03  elphel
//  bug fix, made binning a non-stop parameter
//
//  Revision 1.21  2006/01/15 13:03:27  spectr_rain
//  fix deadlock
//
//  Revision 1.19  2006/01/12 03:52:14  elphel
//  *** empty log message ***
//
//  Revision 1.18  2006/01/10 15:41:54  spectr_rain
//  use gain compensation for color canal scale
//
//  Revision 1.17  2006/01/05 05:36:13  spectr_rain
//  *** empty log message ***
//
//  Revision 1.15  2006/01/05 05:21:32  spectr_rain
//  *** empty log message ***
//
//  Revision 1.14  2006/01/05 05:15:27  spectr_rain
//  new sensitivity/scale iface
//
//  Revision 1.13  2006/01/05 00:01:21  elphel
//  fixed hang-up - removed restarting a frame that was sometimes hanging a sensor until reset
//
//  Revision 1.12  2005/11/29 09:13:31  spectr_rain
//  add autoexposure
//
//  Revision 1.11  2005/11/23 05:13:25  spectr_rain
//  use exposition and gain with autoexposition
//
//  Revision 1.10  2005/11/22 09:21:01  elphel
//  Fixed skipping of half frames, improved period calculation (not yet for binning modes)
//
//  Revision 1.9  2005/10/12 17:58:08  elphel
//  fixed wrong fps reporting in mt9x001.c
//
//  Revision 1.8  2005/10/11 08:25:41  elphel
//  fixed long exposures without slowing down sensor (especially for 3MPix sensor)
//
//  Revision 1.7  2005/09/15 22:46:51  elphel
//  Fixed bug with 1.3MPix Micron sensor (introduced in previous release)
//
//  Revision 1.6  2005/09/10 23:33:22  elphel
//  Support of realtime clock and timestamps in the images
//
//  Revision 1.5  2005/09/06 03:40:37  elphel
//  changed parameters, added support for the photo-finish mode
//
//  Revision 1.4  2005/08/27 05:16:27  elphel
//  binning
//
//  Revision 1.3  2005/08/27 00:46:39  elphel
//  bayer control with &byr=
//
//  Revision 1.2  2005/05/10 21:08:49  elphel
//  *** empty log message ***
//
 */


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
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <elphel/c313a.h>
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h" // detect sensor
//#include "cci2c.h"
#include "mt9x001.h"
#include "multi10359.h"
#include "framepars.h"      // parameters manipulation
#include "sensor_common.h"
#include "pgm_functions.h"
//#include "x3x3.h"           // hardware definitions
#include "legacy_defines.h" // temporarily
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


/** Capabilities of MT9M001 1.3 MPix */
struct sensor_t mt9m001={
        // sensor constants
        .imageWidth  = 1280,     ///< nominal image width for final images
        .imageHeight = 1024,     ///< nominal image height for final images
        .clearWidth  = 1289,     ///< maximal clear image width
        .clearHeight = 1033,     ///< maximal clear image height;
        .clearTop    = 8,        ///< top margin to the first clear pixel
        .clearLeft   = 16,       ///< left margin to the first clear pixel
        .arrayWidth  = 1312,     ///< total image array width (including black and boundary)
        .arrayHeight = 1048,     ///< total image array height (including black and boundary)
        .minWidth    = 4,        ///< minimal WOI width
        .minHeight   = 3,        ///< minimal WOI height
        .minHorBlank = 19,       ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 244,      ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 2047,     ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 16,       ///< minimal vertical blanking
        .maxVertBlank= 2047,     ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 0x3fff,   ///< Maximal shutter duration (in lines)
        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 3,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 1,        ///< bayer shift for flips==0
        .dcmHor      = 0x8b,     ///< available horizontal decimation values 1,2,4,8
        .dcmVert     = 0x8b,     ///< available vertical decimation values 1,2,4,8
        .binHor      = 0x01,     ///< available horizontal binning values 1
        .binVert     = 0x01,     ///< vailable vertical binning values 1
        .maxGain256  = 4032,     ///< (15.75) maximal analog gain times 0x100
        .minClockFreq= 20000000, ///< Minimal clock frequency
        .maxClockFreq= 48000000, ///< Maximal clock frequency
        .nomClockFreq= 48000000, ///< nominal clock frequency
        .sensorType  = SENSOR_MT9X001 + MT9M_TYP,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9X001_I2C_ADDR,           ///< sensor i2c slave address (7 bits)
        .i2c_period  = 2500,     ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,        ///< number of bytes/ register
        .hact_delay  = 0,        ///< delay in ps, TBD
        .sensorDelay = 0,        ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE  ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};

 /** Capabilities of MT9D001 2 MPix */
struct sensor_t mt9d001={
        // sensor constants
        .imageWidth  = 1600,     ///< nominal image width for final images
        .imageHeight = 1200,     ///< nominal image height for final images
        .clearWidth  = 1609,     ///< maximal clear image width
        .clearHeight = 1209,     ///< maximal clear image height;
        .clearTop    = 8,        ///< top margin to the first clear pixel
        .clearLeft   = 20,       ///< left margin to the first clear pixel
        .arrayWidth  = 1632,     ///< total image array width (including black and boundary)
        .arrayHeight = 1224,     ///< total image array height (including black and boundary)
        .minWidth    = 4,        ///< minimal WOI width
        .minHeight   = 3,        ///< minimal WOI height
        .minHorBlank = 19,       ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 617,      ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 2047,     ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 16,       ///< minimal vertical blanking
        .maxVertBlank= 2047,     ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 0x3fff,   ///< Maximal shutter duration (in lines)
        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 3,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 0,        ///< bayer shift for flips==0
        .dcmHor      = 0x8b,     ///< available horizontal decimation values 1,2,4,8
        .dcmVert     = 0x8b,     ///< available vertical decimation values 1,2,4,8
        .binHor      = 0x01,     ///< available horizontal binning values 1
        .binVert     = 0x01,     ///< vailable vertical binning values 1
        .maxGain256  = 4032,     ///< (15.75) maximal analog gain times 0x100
        .minClockFreq= 20000000, ///< Minimal clock frequency
        .maxClockFreq= 48000000, ///< Maximal clock frequency
        .nomClockFreq= 48000000, ///<nominal clock frequency
        .sensorType  = SENSOR_MT9X001 + MT9D_TYP,      ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9X001_I2C_ADDR,               ///< sensor i2c slave address (7 bits)
        .i2c_period  = 2500,     ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,        ///< number of bytes/ register
        .hact_delay  = 0,        ///< delay in ps, TBD
        .sensorDelay = 0,        ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE   ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};
/** Capabilities of MT9T031 3 MPix*/
struct sensor_t mt9t001={
        // sensor constants
        .imageWidth  = 2048,     ///< nominal image width for final images
        .imageHeight = 1536,     ///< nominal image height for final images
        .clearWidth  = 2048,     ///< maximal clear image width
        .clearHeight = 1545,     ///< maximal clear image height;
        .clearTop    = 16,       ///< top margin to the first clear pixel
        .clearLeft   = 28,       ///< left margin to the first clear pixel
        .arrayWidth  = 2112,     ///< total image array width (including black and boundary)
        .arrayHeight = 1568,     ///< total image array height (including black and boundary)
        .minWidth    = 2,        ///< minimal WOI width
        .minHeight   = 2,        ///< minimal WOI height
        .minHorBlank = 21,       ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 511,      ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 2047,     ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 4,        ///< minimal vertical blanking
        .maxVertBlank= 2047,     ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 0xfffff,  ///< Maximal shutter duration (in lines)
        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 3,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 0,        ///< bayer shift for flips==0
        .dcmHor      = 0xff,     ///< available horizontal decimation values 1,2,3,4,5,6,7,8
        .dcmVert     = 0xff,     ///< available vertical decimation values 1,2,3,4,5,6,7,8
        .binHor      = 0xff,     ///< available horizontal binning values 1,2,3,4,5,6,7,8
        .binVert     = 0xff,     ///< vailable vertical binning values 1,2,3,4,5,6,7,8
        .maxGain256  = 4032,     ///< (15.75) maximal analog gain times 0x100
        .minClockFreq= 20000000, ///< Minimal clock frequency
        .maxClockFreq= 48000000, ///< Maximal clock frequency
        .nomClockFreq= 48000000, ///<nominal clock frequency
        .sensorType  = SENSOR_MT9X001 + MT9T_TYP,      ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9X001_I2C_ADDR,
        .i2c_period  = 2500,     ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,        ///< number of bytes/ register
        .hact_delay  = 0,        ///< delay in ps, TBD
        .sensorDelay = 0,        ///< Delay from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE   ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};
/** Capabilities of MT9P006 - 5 MPix */
struct sensor_t mt9p001={
        // sensor constants
        .imageWidth  = 2592,     ///< nominal image width for final images
        .imageHeight = 1944,     ///< nominal image height for final images
        .clearWidth  = 2608,     ///< maximal clear image width
        .clearHeight = 1952,     ///< maximal clear image height;
        .clearTop    = 50,       ///< top margin to the first clear pixel
        .clearLeft   = 10,       ///< left margin to the first clear pixel
        .arrayWidth  = 2752,     ///< total image array width (including black and boundary)
        .arrayHeight = 2003,     ///< total image array height (including black and boundary)
        .minWidth    = 2,        ///< minimal WOI width
        .minHeight   = 2,        ///< minimal WOI height
        .minHorBlank = 0,        ///< minimal horizontal blanking, in pixels in no-decimation, no-binning mode.
        .minLineDur  = 647,      ///< minimal total line duration, in pixels in no-decimation, no-binning mode.
        .maxHorBlank = 4095,     ///< maximal horizontal blanking/Virtual frame width (depends on sensor type)
        .minVertBlank= 9,        ///< minimal vertical blanking
        .maxVertBlank= 2047,     ///< maximal vertical blanking/Virtual frame height (depends on sensor type)
        .maxShutter  = 0xfffff,  ///< Maximal shutter duration (in lines)
        .flips       = 3,        ///< bit mask bit 0 - flipX, 1 - flipY
        .init_flips  = 0,        ///< normal orientation flips bit mask bit 0 - flipX, 1 - flipY
        .bayer       = 3,        ///< bayer shift for flips==0
        .dcmHor      = 0xff,     ///< 1,2,3,4,5,6,7,8 (doc show [0,6] - change to 0x7f
        .dcmVert     = 0xff,     ///< 1,2,3,4,5,6,7,8
        .binHor      = 0xff,     ///< 1,2,4 0xb{0,1,3}
        .binVert     = 0xff,     ///< 1,2,3,4 0xf [0,3]
        .maxGain256  = 4032,     ///< (15.75) maximal analog gain times 0x100
        .minClockFreq= 20000000, ///< Minimal clock frequency
        .maxClockFreq= 96000000, ///< Maximal clock frequency
        .nomClockFreq= 96000000, ///< nominal clock frequency
        .sensorType  = SENSOR_MT9X001 + MT9P_TYP,  ///< sensor type (for Elphel cameras)
        .i2c_addr    = MT9P001_I2C_ADDR,           ///< sensor i2c slave address (7 bits)
        .i2c_period  = 2500,     ///< SCL period in ns, (standard i2c - 2500)
        .i2c_bytes   = 2,        ///< number of bytes/ register
        .hact_delay  = -2500,    ///< -2.5ns delay in ps
        .sensorDelay = 2460,     ///< Dealy from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
        .needReset=    SENSOR_NEED_RESET_CLK | SENSOR_NEED_RESET_PHASE   ///< bit 0 - need reset after clock frequency change, bit 1 - need reset after phase change
};

// a place to add some general purpose register writes to sensors during init

/** Register initial writes for MT9M001 */
static  unsigned short mt9m001_inits[]= 
{
};

/** Register initial writes for MT9D001 */
static  unsigned short mt9d001_inits[]= 
{
        P_MT9X001_CALTHRESH , 0xa39d,
        P_MT9X001_CALCTRL,    0x8498
};

/** Register initial writes for MT9T031 */
static  unsigned short mt9t001_inits[]= 
{
};

/** Register initial writes for MT9P006 */
static  unsigned short mt9p001_inits[]= 
{
        P_MT9X001_OUTCTRL,  0x2, // set slowest output signals (clock and non-clock) to reduce EMI (for FCC part 15)
        P_MT9X001_7F     ,  0x0  // Should be written 0 to prevent blue "blooming" columns
};
/** Specifying sensor registers to be controlled individually in multi-sensor applications, MT9M001 */
static  unsigned short mt9m001_multiregs[]= 
{
        P_MT9X001_ROWSTART,
        P_MT9X001_COLSTART,
        P_MT9X001_HEIGHT,
        P_MT9X001_WIDTH,
        P_MT9X001_HORBLANK,
        P_MT9X001_VERTBLANK,
        P_MT9X001_SHTRWDTHU,
        P_MT9X001_SHTRWDTH,
        P_MT9X001_RMODE1,
        P_MT9X001_RMODE2,
        P_MT9X001_GREEN1,
        P_MT9X001_BLUE,
        P_MT9X001_RED,
        P_MT9X001_GREEN2,
        P_MT9X001_TEST
};

/** Specifying sensor registers to be controlled individually in multi-sensor applications, MT9D001 */
static  unsigned short mt9d001_multiregs[]= 
{
        P_MT9X001_ROWSTART,
        P_MT9X001_COLSTART,
        P_MT9X001_HEIGHT,
        P_MT9X001_WIDTH,
        P_MT9X001_HORBLANK,
        P_MT9X001_VERTBLANK,
        P_MT9X001_SHTRWDTHU,
        P_MT9X001_SHTRWDTH,
        P_MT9X001_RMODE1,
        P_MT9X001_RMODE2,
        P_MT9X001_GREEN1,
        P_MT9X001_BLUE,
        P_MT9X001_RED,
        P_MT9X001_GREEN2,
        P_MT9X001_TEST
};

/** Specifying sensor registers to be controlled individually in multi-sensor applications, MTTM031 */
static  unsigned short mt9t001_multiregs[]= 
{
        P_MT9X001_ROWSTART,
        P_MT9X001_COLSTART,
        P_MT9X001_HEIGHT,
        P_MT9X001_WIDTH,
        P_MT9X001_HORBLANK,
        P_MT9X001_VERTBLANK,
        P_MT9X001_SHTRWDTHU,
        P_MT9X001_SHTRWDTH,
        P_MT9X001_RMODE1,
        P_MT9X001_RMODE2,
        P_MT9X001_GREEN1,
        P_MT9X001_BLUE,
        P_MT9X001_RED,
        P_MT9X001_GREEN2,
        P_MT9X001_TEST
};

/** Specifying sensor registers to be controlled individually in multi-sensor applications, MT9P006 */
static  unsigned short mt9p001_multiregs[]= 
{
        P_MT9X001_ROWSTART,
        P_MT9X001_COLSTART,
        P_MT9X001_HEIGHT,
        P_MT9X001_WIDTH,
        P_MT9X001_HORBLANK,
        P_MT9X001_VERTBLANK,
        P_MT9X001_SHTRWDTHU,
        P_MT9X001_SHTRWDTH,
        P_MT9X001_RMODE1,
        P_MT9X001_RMODE2,
        P_MT9X001_GREEN1,
        P_MT9X001_BLUE,
        P_MT9X001_RED,
        P_MT9X001_GREEN2,
        P_MT9X001_TEST
};


int mt9x001_pgm_detectsensor (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_initsensor   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_window       (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_window_safe  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_window_common(int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_limitfps     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_exposure     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_gains        (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_triggermode  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
int mt9x001_pgm_sensorregs   (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);
// @brief read 2 bytes from i2c
#ifdef NC353
#define I2C_READ_DATA16(x) ((i2c_read_data[(x)<<1]<<8)+i2c_read_data[((x)<<1)+1])
#endif
/**Detect and initialize sensor and related data structures
 * - detect sensor type.
 * - if successful, proceed to:,
 * - copy sensor static structure
 * - setup appropriate pgm_* functions
 * - read sensor registers to shadows
 * - initialize appropriate P_* registers (including sensor register shadows) - that initialization will schedule related pgm_* functions
 * TODO: when is P_CLK_FPGA initialized? Needs to be done before this
 * hardware i2c is expected to be reset and initialized - no wrong, it will be programmed in 
 *                                onchange_i2c should be the first after init sensor (even before onchange_sensorphase)
 * onchange_sensorphase will be triggered after this
 * hardware i2c after this function will be disabled, will need onchange_sensorphase to initialize/start it. */
int mt9x001_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
    unsigned long flags;               // this function uses software i2c operations - they need to have interrupts (and hardware i2c off)
    //    unsigned char    i2c_read_data[2]; // each two bytes - one short word, big endian
    u32  i2c_read_dataw;
    u8 * i2c_read_data =  (u8*)& i2c_read_dataw; // each two bytes - one short word, big endian
    //    unsigned char chipver_reg=P_MT9X001_CHIPVER;
    u32 chipver_reg=P_MT9X001_CHIPVER;
    int sensor_subtype=0;
    int i;
    struct sensor_t * psensor; // current sensor
    MDF4(printk(" frame8=%d\n",frame8));
    //  MDD1(printk("sensor=0x%x\n", (int)sensor));
    if (thispars->pars[P_SENSOR]!=0) { ///already initialized - called second time after common pgm_detectsensor(), first time is inside pgm_detectsensor()
        MDF1(printk(" sensor 0x%x already detected, exiting\n",(int) thispars->pars[P_SENSOR]));
        return sensor->sensorType;
    }
    // set control lines
    CCAM_NEGRST;  ///set negative MRST polarity
    CCAM_TRIG_INT;
    CCAM_MRST_OFF;
    CCAM_ARST_OFF;
    udelay (100);

    // try MT9P001 first
    psensor= &mt9p001;

#ifdef NC353
    local_irq_save(flags); // IRQ Off
    i2c_stop_wait();
    i2c_writeData(0, (psensor->i2c_addr) & 0xfe,  &chipver_reg, 1, 0); // no stop before read  (cxi2c.c)
    i2c_readData (0, (psensor->i2c_addr) | 1,     i2c_read_data, 2, 0); ///restart, not start  (cxi2c.c)
    local_irq_restore(flags); // IRQ restore
    if (((I2C_READ_DATA16(0) ^ MT9P001_PARTID) & MT9X001_PARTIDMASK)==0) {
        printk("Found MT9P001 2592x1944 sensor, chip ID=%x\n",(i2c_read_data[0]<<8)+i2c_read_data[1]);
        sensor_subtype=MT9P_TYP; //1;
    }
#else
    X3X3_I2C_RCV2(sensor_port, psensor->i2c_addr, P_MT9X001_CHIPVER, &i2c_read_dataw);
    if (((i2c_read_dataw ^ MT9P001_PARTID) & MT9X001_PARTIDMASK)==0) {
        printk("Found MT9P001 2592x1944 sensor, chip ID=%x\n",i2c_read_dataw);
        sensor_subtype=MT9P_TYP; //1;
    }
#endif
    //  printk("sensor id= 0x%x\r\n",i2c_read_data[0]);
    //  MDD1(printk("sensor=0x%x\n", (int)sensor));
    if (sensor_subtype ==0)  { // not a 5MPix chip
        CCAM_ARST_ON;
        psensor= &mt9m001; //address the same for all others
#ifdef NC353
        local_irq_save(flags); // IRQ Off
        i2c_stop_wait();
        i2c_writeData(0, (psensor->i2c_addr) & 0xfe, &chipver_reg, 1, 0); // no stop before read
        i2c_readData (0, (psensor->i2c_addr) | 1,    i2c_read_data, 2, 0); //restart, not strart
        local_irq_restore(flags); // IRQ restore
        //    printk("-sensor id= 0x%x\r\n",i2c_read_data[0]);
        if (((I2C_READ_DATA16(0)^MT9M001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9M001 1280x1024 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9m001;
            sensor_subtype=MT9M_TYP; //1;
        } else if (((I2C_READ_DATA16(0)^MT9D001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9D001 1600x1200 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9d001;
            sensor_subtype=MT9D_TYP; //2;
        } else if (((I2C_READ_DATA16(0)^MT9T001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9T001 2048x1536 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9t001;
            sensor_subtype=MT9T_TYP; //3;
            //      if(d[2] == 0x01) - MT9T001 chip rev 01 - color gains had a bug
        }
#else
        X3X3_I2C_RCV2(sensor_port, psensor->i2c_addr, P_MT9X001_CHIPVER, &i2c_read_dataw);
        if (((i2c_read_dataw ^MT9M001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9M001 1280x1024 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9m001;
            sensor_subtype=MT9M_TYP; //1;
        } else if (((i2c_read_dataw ^ MT9D001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9D001 1600x1200 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9d001;
            sensor_subtype=MT9D_TYP; //2;
        } else if (((i2c_read_dataw ^ MT9T001_PARTID) & MT9X001_PARTIDMASK)==0) {
            printk("Found MT9T001 2048x1536 sensor, chip ID=%x\r\n",I2C_READ_DATA16(0));
            psensor= &mt9t001;
            sensor_subtype=MT9T_TYP; //3;
            //      if(d[2] == 0x01) - MT9T001 chip rev 01 - color gains had a bug
        }
#endif
    }
    //  MDD1(printk("sensor=0x%x, sensor_subtype=0x%x\n", (int)sensor, (int)sensor_subtype));
    if (sensor_subtype ==0)   return 0;  // no sensor found
    // Sensor recognized, go on
    //  memcpy(&sensor, psensor, sizeof(mt9p001)); // copy sensor definitions
    memcpy(sensor, psensor, sizeof(mt9p001)); // copy sensor definitions
    //  MDD1(printk("sensor=0x%x\n", (int)sensor));
    MDF1(printk("copied %d bytes of sensor static parameters\r\n",sizeof(mt9p001)));
    add_sensor_proc(onchange_detectsensor,&mt9x001_pgm_detectsensor); // detect sensor type, sets sensor structure (capabilities), function pointers NOTE: will be called directly, not through pointers
    add_sensor_proc(onchange_initsensor,  &mt9x001_pgm_initsensor);   // resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
    add_sensor_proc(onchange_exposure,    &mt9x001_pgm_exposure);     // program exposure
    add_sensor_proc(onchange_window,      &mt9x001_pgm_window);       // program sensor WOI and mirroring (flipping)
    add_sensor_proc(onchange_window_safe, &mt9x001_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency
    add_sensor_proc(onchange_limitfps,    &mt9x001_pgm_limitfps);     // check compressor will keep up, limit sensor FPS if needed
    add_sensor_proc(onchange_gains,       &mt9x001_pgm_gains);        // program analog gains
    add_sensor_proc(onchange_triggermode, &mt9x001_pgm_triggermode);  // program sensor trigger mode
    add_sensor_proc(onchange_sensorregs,  &mt9x001_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?
    //  MDD1(printk("sensor->sensorType=0x%lx\n", sensor->sensorType));
    setFramePar(thispars, P_SENSOR,  sensor->sensorType); // was so
    //  setFramePar(thispars, P_SENSOR  | FRAMEPAIR_FORCE_NEWPROC,  sensor->sensorType); // force actions
    //  MDD1(printk("\n"));
    ///TODO: Fill G_MULTI_REGSM+i - which registers need individual values in multi-sensor applications
    unsigned short * sensor_multi_regs;
    int sensor_multi_regs_number;
    for (i=0; i<8; i++ ) GLOBALPARS(sensor_port, G_MULTI_REGSM+i)=0; // erase old (or should we keep them?)
    if (GLOBALPARS(sensor_port,G_SENS_AVAIL)!=0) {
        switch (sensor_subtype) {
        case MT9M_TYP:
            sensor_multi_regs=         (unsigned short *) &mt9m001_multiregs; // why casting is needed?
            sensor_multi_regs_number=  sizeof(mt9m001_multiregs)/sizeof(short);
            break;
        case MT9D_TYP:
            sensor_multi_regs=         (unsigned short *) &mt9d001_multiregs;
            sensor_multi_regs_number=  sizeof(mt9d001_multiregs)/sizeof(short);
            break;
        case MT9T_TYP:
            sensor_multi_regs=         (unsigned short *) &mt9t001_multiregs;
            sensor_multi_regs_number=  sizeof(mt9t001_multiregs)/sizeof(short);
            break;
        case MT9P_TYP:
        default:
            sensor_multi_regs=         (unsigned short *) &mt9p001_multiregs;
            sensor_multi_regs_number=  sizeof(mt9p001_multiregs)/sizeof(short);
            break;
        }
        for (i=0; i<sensor_multi_regs_number; i++ ) {
            GLOBALPARS(sensor_port, G_MULTI_REGSM + ((sensor_multi_regs[i]>>5) & 7)) |= 1 << (sensor_multi_regs[i] & 0x1f);
            //       MDF(printk("i=%d, m=0x%x\n",i,(int)sensor_multi_regs[i]));
            //       MDF(printk("GLOBALPARS(0x%x)=0x%x\n",(int) G_MULTI_REGSM + ((sensor_multi_regs[i]>>5) & 7) ,(int)GLOBALPARS(G_MULTI_REGSM + ((sensor_multi_regs[i]>>5) & 7))));
        }
    }
    //#define GLOBALPARS(x) globalPars[(x)-FRAMEPAR_GLOBALS] // should work in drivers and applications

    for (i=0;i<8;i++) {
        //    MDF(printk("i=%d, m=0x%lx\n",i,GLOBALPARS(G_MULTI_REGSM+i)));
    }
    MDF4(printk(" set ARO (TRIGGER) line HIGH\n"));
    CCAM_ARO_ON ; //set
    return sensor->sensorType;
    //NOTE 353:  hardware i2c is turned off (not needed in 393)
}

/** Reset and initialize sensor
 * resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c - 353 only)
 * i2c is supposed to be already programmed */
int mt9x001_pgm_initsensor     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    MDF4(printk(" frame8=%d\n",frame8));
    if (frame16 >= 0) return -1; // should be ASAP
//    int fpga_addr=(frame8 <0) ? X313_I2C_ASAP : (X313_I2C_FRAME0+frame8);
    unsigned long flags; // this function uses software i2c operations - they need to have interrupts (and hardware i2c off)
    struct frameparspair_t pars_to_update[258+(MAX_SENSORS * P_MULTI_NUMREGS )]; // for all the sensor registers. Other P_* values will reuse the same ones
//    unsigned char    i2c_read_data[512]; // each two bytes - one short word, big endian
    u32 i2c_read_data_dw[256];
    int nupdate=0;
    int i,color;
    int regval, regnum, mreg, j;
    printk("Resetting MT9X001 sensor\r\n");
    // reset sensor by applying MRST (low):
    CCAM_MRST_ON;
    udelay (100);
    CCAM_MRST_OFF;
    udelay (100);
    printk("Reading sensor registers to the shadows:\r\n");
    int first_sensor_i2c=sensor->i2c_addr;
    if (GLOBALPARS(sensor_port, G_SENS_AVAIL)) {
        first_sensor_i2c+= I2C359_INC * ((GLOBALPARS(sensor_port, G_SENS_AVAIL) & 1)?1:((GLOBALPARS(sensor_port, G_SENS_AVAIL) & 2)?2:3));
    }
#ifdef NC353
    i2c_read_data[0]=0;
    local_irq_save(flags); // IRQ Off (rather long - all 256 registers through i2c, but there is no hurry - sensor is off)
    i2c_stop_wait();
    //G_SENS_AVAIL
    i2c_writeData(0, (first_sensor_i2c) & 0xfe,  i2c_read_data, 1, 0);   // data (register #) is 0. no stop before read  (cxi2c.c)
    i2c_readData (0, (first_sensor_i2c) | 1,     i2c_read_data, 512, 0); // read all 256 registers (512 bytes) restart, not strart  (cxi2c.c)
    local_irq_restore(flags); // IRQ restore
    // save these registers as shadows and propagate

    nupdate=0;
    // For multiple sensors will use shadows from first one. Change?
    for (i=0; i<256; i++) { // possible to modify register range to save (that is why nupdate is separate from i)
        regval=I2C_READ_DATA16(i);
        regnum=P_SENSOR_REGS+i;
        SETFRAMEPARS_SET(regnum,regval);
        if ((mreg=MULTIREG(regnum,0))) for (j=0;j<MAX_SENSORS; j++) {
            SETFRAMEPARS_SET(mreg+j,regval);
        }
    }
#else
    for (i=0; i<256; i++) { // rdead all registers, one at a time (slower than in 353)
        X3X3_I2C_RCV2(sensor_port, first_sensor_i2c, i, &(i2c_read_data_dw[i]));
    }
    for (i=0; i<256; i++) { // possible to modify register range to save (that is why nupdate is separate from i)
        regval=i2c_read_data_dw[i];
        regnum=P_SENSOR_REGS+i;
        SETFRAMEPARS_SET(regnum,regval);
        if ((mreg=MULTIREG(sensor_port,regnum,0))) for (j=0;j<MAX_SENSORS; j++) {
            SETFRAMEPARS_SET(mreg+j,regval);
        }
    }
#endif
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows

    printk("Initializing MT9X001 registers with default values:\r\n");
    unsigned short * sensor_register_overwrites;
    int              sensor_register_overwrites_number;
    int              sensor_subtype=sensor->sensorType  - SENSOR_MT9X001;
    switch (sensor_subtype) {
    case MT9M_TYP:
        sensor_register_overwrites=         (unsigned short *) &mt9m001_inits; // why casting is needed?
        sensor_register_overwrites_number=  sizeof(mt9m001_inits)/4;
        break;
    case MT9D_TYP:
        sensor_register_overwrites=         (unsigned short *) &mt9d001_inits;
        sensor_register_overwrites_number=  sizeof(mt9d001_inits)/4;
        break;
    case MT9T_TYP:
        sensor_register_overwrites=         (unsigned short *) &mt9t001_inits;
        sensor_register_overwrites_number=  sizeof(mt9t001_inits)/4;
        break;
    case MT9P_TYP:
    default:
        sensor_register_overwrites=         (unsigned short *) &mt9p001_inits;
        sensor_register_overwrites_number=  sizeof(mt9p001_inits)/4;
        break;
    }
    //  enable hardware i2c - NOTE: the only place where the i2c controller is enabled.
    printk ("Starting hardware sequencers\n");
#ifdef NC353
    local_irq_save(flags); // IRQ Off, so both sequencers to be started at the same time
    i2c_run();
    X3X3_SEQ_RUN;
    local_irq_restore(flags); // IRQ restore
#else
    X3X3_SEQ_RUN;
    i2c_run();
#endif
    nupdate=0;  // Second pass over the registers to set
//#define SET_SENSOR_MBPAR(p,f,s,r,v)
    for (i=0; i<sensor_register_overwrites_number;i++ ) { // unconditionally set those registers NOTE: Should be < 63 of them!
        SET_SENSOR_MBPAR(sensor_port,frame16,sensor->i2c_addr, sensor_register_overwrites[2*i], sensor_register_overwrites[2*i+1]);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x,0x%x,0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) sensor_register_overwrites[2*i], (int) sensor_register_overwrites[2*i+1]));

    }
    SETFRAMEPARS_SET(P_GAIN_MIN, 0x10000);
    SETFRAMEPARS_SET(P_GAIN_MAX, (sensor->maxGain256)<<8);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    // G_* parameters - can write directly
    for (color=0;color<4;color++){
        for (i=0;i<81;i++) {
            GLOBALPARS(sensor_port, G_SENSOR_CALIB+(color<<8)+i)=(i>32)?((i -16)<<14): (i<<13); // one extra
        }
    }
    MDF4(for (i=0; i<1023; i++) {if ((i & 0x1f)==0) printk ("\n"); printk(" 0x%06lx",GLOBALPARS (sensor_port, G_SENSOR_CALIB+i));});

    return 0;
} 


/** Program sensor WOI and mirroring
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands
 * As different sensors may produce "bad frames" for differnt WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped)*/
int mt9x001_pgm_window     (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
    MDF4(printk(" frame16=%d\n",frame16));
    return mt9x001_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
}

/** Program sensor WOI and mirroring in safe mode (now does the same as mt9x001_pgm_window)
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands  */
int mt9x001_pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
        struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
        struct framepars_t * thispars, ///< sensor current parameters
        struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
        int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                       ///< be applied to,  negative - ASAP
                                       ///< @return 0 - OK, negative - error
{
    MDF4(printk(" frame16=%d\n",frame16));
    return mt9x001_pgm_window_common (sensor,  thispars, prevpars, frame16);
}

/** PCommon part of programming sensor WOI */
int mt9x001_pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
    struct frameparspair_t  pars_to_update[29];
    int nupdate=0;
    MDF4(printk(" frame16=%d\n",frame16));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int i,dv,dh,bv,bh,ww,wh,wl,wt,flip,flipX,flipY,d, v;
    int styp = sensor->sensorType & 7;
    dh=  thispars->pars[P_DCM_HOR];
    dv=  thispars->pars[P_DCM_VERT];
    bh=  thispars->pars[P_BIN_HOR];
    bv=  thispars->pars[P_BIN_VERT];
    ww=  thispars->pars[P_SENSOR_PIXH] * dh;
    wh=  thispars->pars[P_SENSOR_PIXV] * dv;
    flip=((thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1) << 1 )) ^ sensor->init_flips; // 10338 is _not_ flipped (as the ther boards, but for legacy compatibility....)
    flipX =  flip & 1;
    flipY = (flip & 2)? 1:0;
    d = 2 * bh * (ww / (2 * bh));
    if (unlikely(d != ww)) { // correct window width if needed
        SETFRAMEPARS_SET(P_SENSOR_PIXH, d / dh);
        ww = d;
    }
    // program sensor width
    if ((ww-1) != thispars->pars[P_SENSOR_REGS+P_MT9X001_WIDTH]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_WIDTH, ww-1);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_WIDTH, (int) ww-1));
    }
    // height
    d = (wh+1) & 0xfffe; // round up to even
    if (unlikely(d != wh)) { // correct window height if needed
        //      setFramePar(thispars, P_SENSOR_PIXV,  d / dv);
        SETFRAMEPARS_SET(P_SENSOR_PIXV, d / dv);
        wh = d;
    }
    // program sensor height
    if ((wh-1) != thispars->pars[P_SENSOR_REGS+P_MT9X001_HEIGHT]) {
        SET_SENSOR_MBPAR(sensor_port, frame16 ,sensor->i2c_addr, P_MT9X001_HEIGHT, wh-1);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_HEIGHT, (int) wh-1));
    }
    // Margins
    wl = thispars->pars[P_WOI_LEFT];
    wt = thispars->pars[P_WOI_TOP];
    // flip margins for mirrored images (except oversized, not to rely on sensor->clearWidth/sensor->clearHeight
    if (!thispars->pars[P_OVERSIZE]) {
        if(flipX) {
            wl = sensor->clearWidth - ww - wl - X313_MARGINS * dh;
            if(wl < 0) wl = 0;
        }
        if(flipY) {
            wt = sensor->clearHeight - wh - wt - X313_MARGINS * dv;
            if(wt < 0) wt = 0;
        }
        // apply clearTop/clearLeft
        wt = (wt + sensor->clearTop) & 0xfffe;
        wl = (wl + sensor->clearLeft) & 0xfffe;
        // apply binning restrictions
        switch(styp) {
        case MT9P_TYP:
            d = (bh > 1) ? ((bh > 2) ? 16 : 8) : 4;
            if(flipX)
                i = d * ((wl - 2) / d) + 2;
            else
                i = d * (wl / d);
            if(i < wl)
                i += d;
            wl = i;
            break;
        case MT9T_TYP:
            d = (bh > 1) ? ((bh > 2) ? (16) : (8)) : (4);
            if(flipX)
                i = d * ((wl - 2) / d) + 2;
            else
                i = d * (wl / d);
            if(i < wl)
                i += d;
            wl = i;
            break;
        }
    }
    // Program sensor left margin
    if (wl != thispars->pars[P_SENSOR_REGS+P_MT9X001_COLSTART]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_COLSTART, wl);
        MDF4(printk("  XSET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",  sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_COLSTART, (int) wl));

    }
    // Program sensor top margin
    if (wt != thispars->pars[P_SENSOR_REGS+P_MT9X001_ROWSTART]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_ROWSTART, wt);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%0x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_ROWSTART,(int)  wt));
    }
    ///TODO:Get rid of get_sensor_i2c_regs16 !!!
    // the fields that should not chnage in non-stop will not change, so it is OK to write them always
    if((styp == MT9T_TYP) || (styp == MT9P_TYP)) { // 3MPix and 5MPix sensors
        v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM] & 0xff88) | ((bv - 1) << 4) | (dv - 1) ;
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM]) {
            SET_SENSOR_PAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RAM, v);
            MDF4(printk("  SET_SENSOR_PAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",  sensor_port, frame16,   (int) sensor->i2c_addr, (int) P_MT9X001_RAM, (int) v));
        }
        v=(thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM] & 0xff88) | ((bh - 1) << 4) | (dh - 1);
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM]) {
            SET_SENSOR_PAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_CAM, v);
            MDF4(printk("  SET_SENSOR_PAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_CAM, (int) v));
        }
        v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fff) | // preserve other bits from shadows
                (flipX ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
                (flipY ? (1 << 15) : 0) ; // FLIPV
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE2, v);
            MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_RMODE2, (int) v));

        }
    } else { // 1.3 and 2 MPix sensors
        v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1] & 0xffc3) | // preserve other bits from shadows (trigger mode moved to other function)
                ((dh == 4) ? (1 << 2) : 0) | // Column skip 4
                ((dv == 4) ? (1 << 3) : 0) | // Row skip    4
                ((dh == 8) ? (1 << 4) : 0) | // Column skip 8
                ((dv == 8) ? (1 << 5) : 0) ; // Row skip    8
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE1, v);
            MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_RMODE1, (int) v));
        }
        v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fe7) | // preserve other bits from shadows
                ((dh == 2) ? (1 << 3) : 0) | // Column skip 2
                ((dv == 2) ? (1 << 4) : 0) | // Row skip    2
                (flipX ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
                (flipY ? (1 << 15) : 0) ; // FLIPV
        if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2]) {
            SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RMODE2, v);
            MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16, (int) sensor->i2c_addr, (int) P_MT9X001_RMODE2, (int) v));
        }

    }
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
int mt9x001_pgm_limitfps   (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error
{
    struct frameparspair_t pars_to_update[16]; // maximum 7 registers updated (need to recount)
    int nupdate=0;
    MDF4(printk(" frame8=%d\n",frame8));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int dh=  thispars->pars[P_DCM_HOR];
    int ww=  thispars->pars[P_SENSOR_PIXH] * dh;
    int binning_cost = 0;
    int width;
    int row_time_in_pixels=0;
    int hor_blank_min;
    int hor_blank=0;
    int p1,p2, pix_period, sclk,fp1000s;
    int styp = sensor->sensorType & 7;
#if USELONGLONG
    uint64_t ull_fp1000s;
#endif
    int target_virt_width=(thispars->pars[P_VIRT_KEEP])?(thispars->pars[P_VIRT_WIDTH]):0;
    switch(styp)  {
    case MT9P_TYP: //page 16
        width = 2 * ww / (2 * dh);
        if((width * dh) < ww) width++;
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
        hor_blank_min = 208 * thispars->pars[P_BIN_VERT] + 64 + (312 + 44 + binning_cost) / 2;
        MDF4(printk("hor_blank_min =%d(0x%x)\n",hor_blank_min,hor_blank_min));
        hor_blank = hor_blank_min;
        MDF4(printk("hor_blank =%d(0x%x)\n",hor_blank,hor_blank));
        row_time_in_pixels = width + hor_blank * 2;
        MDF4(printk("row_time_in_pixels =%d(0x%x)\n",row_time_in_pixels,row_time_in_pixels));
        int i = 2 * (41 + 208 * thispars->pars[P_BIN_VERT] + 99); ///strange 41 and 99, why not 140?
        if(i > row_time_in_pixels)  row_time_in_pixels = i;
        MDF4(printk("row_time_in_pixels =%d(0x%x)\n",row_time_in_pixels,row_time_in_pixels));
        if(target_virt_width > row_time_in_pixels) { // extend line by adding horizontal blanking
            hor_blank = (target_virt_width - width) / 2;
            if(hor_blank > sensor->maxHorBlank) hor_blank = sensor->maxHorBlank;
            row_time_in_pixels = width + 2 * hor_blank;
            MDF4(printk("row_time_in_pixels =%d(0x%x)\n",row_time_in_pixels,row_time_in_pixels));
        }
        break;
        case MT9T_TYP:
            width = 2 * ww / (2 * dh);
            if((width * dh) < ww) width++;
            switch(thispars->pars[P_BIN_VERT]) {
            case 1:  p1 = 331; break;
            case 2:  p1 = 673; break;
            case 3:  p1 = 999; break;
            default: p1 = 999; break; //undocumented
            }
            switch(thispars->pars[P_BIN_HOR]) {
            case 1:  p2 = 38; break;
            case 2:  p2 = 22; break;
            case 3:  p2 = 14; break;
            default: p2 = 38; break; //undocumented
            }
            hor_blank = sensor->minHorBlank;
            row_time_in_pixels = width + p1 + p2 + hor_blank;
            if(target_virt_width > row_time_in_pixels) { // extend line by adding horizontal blanking
                hor_blank = target_virt_width - width - p1 - p2;
                if(hor_blank > sensor->maxHorBlank) hor_blank = sensor->maxHorBlank;
                row_time_in_pixels = width + p1 + p2 + hor_blank;
            }
            hor_blank--; // confusing - for mt9p001 (not others which refer reg[0x05]) :"HB Horizontal Blanking Horizontal_Blank+1"
            break;
            case MT9D_TYP:
                width = 2 * ((ww - 1) / (2 * dh)) + 2; ///docs, p.6
                hor_blank = sensor->minHorBlank;
                p1 = 322;
                p2 = 2 - 19; //excluded hor_blank
                row_time_in_pixels = width + p1 + p2 + hor_blank;
                i = p1 + 295;
                if(i > row_time_in_pixels)    row_time_in_pixels = i;
                if(target_virt_width > row_time_in_pixels) { // extend line by adding horizontal blanking
                    hor_blank = target_virt_width - width - p1 - p2;
                    if (hor_blank > sensor->maxHorBlank) hor_blank = sensor->maxHorBlank;
                    row_time_in_pixels = width + p1 + p2 + hor_blank;
                }
                break;
            case MT9M_TYP:
                width = 2 * ((ww - 1) / (2 * dh)) + 2; ///docs, p.6
                hor_blank = sensor->minHorBlank;
                p1 = 242;
                p2 = 2 - 19; //excluded hor_blank
                row_time_in_pixels= width + p1 + p2 + hor_blank;
                if(target_virt_width > row_time_in_pixels) { // extend line by adding horizontal blanking
                    hor_blank = target_virt_width - width - p1 - p2;
                    if(hor_blank > sensor->maxHorBlank) hor_blank = sensor->maxHorBlank;
                    row_time_in_pixels = width + p1 + p2 + hor_blank;
                }
                break;
    }
    // schedule updating P_VIRT_WIDTH if it changed FIXME: Does not come here?
    MDF4(printk("row_time_in_pixels =%d(0x%x), thispars->pars[P_VIRT_WIDTH ]=%d(0x%x)\n",row_time_in_pixels,row_time_in_pixels,(int)thispars->pars[P_VIRT_WIDTH ],(int)thispars->pars[P_VIRT_WIDTH ]));
    if (thispars->pars[P_VIRT_WIDTH ] != row_time_in_pixels) {
        SETFRAMEPARS_SET(P_VIRT_WIDTH, row_time_in_pixels);
    }
    // schedule updating P_MT9X001_HORBLANK senosr register and shadow FIXME: Seems hor_blank is too high. is the width itself subtracted?
    if (hor_blank != thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]) {
        MDF4(printk("hor_blank =%d(0x%x), thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]=%d(0x%x)\n",hor_blank,hor_blank,(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK],(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_HORBLANK]));
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_HORBLANK, hor_blank);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_HORBLANK, (int)hor_blank));
    }
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
    int height=       thispars->pars[P_SENSOR_PIXV];
    int virt_height = height + sensor->minVertBlank;
    if (thispars->pars[P_VIRT_KEEP]) {
        if (virt_height < thispars->pars[P_VIRT_HEIGHT]) {
            virt_height = thispars->pars[P_VIRT_HEIGHT];
        }
    }

    MDF4(printk("height =%d(0x%x), virt_height=%d(0x%x)\n",height,height,virt_height,virt_height));
    // limit frame rate (using minimal period), but only in sync mode - async should be programmed to observe minimal period
    if ((thispars->pars[P_TRIG] & 4) ==0) {
        int virt_height1= thispars->pars[P_PERIOD_MIN]/row_time_in_pixels; // always non-zero, calculated by pgm_limitfps (common)
        if ((row_time_in_pixels * virt_height1) < thispars->pars[P_PERIOD_MIN]) virt_height1++; //round up
        if (virt_height < virt_height1) virt_height = virt_height1;
        MDF4(printk("height =%d(0x%x), virt_height=%d(0x%x)\n",height,height,virt_height,virt_height));
    }

    int vert_blank= virt_height - height;
    if(vert_blank > sensor->maxVertBlank) {
        vert_blank = sensor->maxVertBlank;
        virt_height = vert_blank + height;
    }
    MDF4(printk("vert_blank =%d(0x%x), virt_height=%d(0x%x)\n",vert_blank,vert_blank,virt_height,virt_height));

    // schedule updating P_VIRT_HEIGHT if it changed
    MDF4(printk("thispars->pars[P_VIRT_HEIGHT ] =%d(0x%x), virt_height=%d(0x%x)\n",(int)thispars->pars[P_VIRT_HEIGHT ],(int)thispars->pars[P_VIRT_HEIGHT ],virt_height,virt_height));
    if (thispars->pars[P_VIRT_HEIGHT ] != virt_height) {
        SETFRAMEPARS_SET(P_VIRT_HEIGHT, virt_height);
    }
    // schedule updating P_PERIOD if it changed
    pix_period=row_time_in_pixels*virt_height;
    MDF4(printk("thispars->pars[P_PERIOD] =%d(0x%x), pix_period=%d(0x%x)\n",(int)thispars->pars[P_PERIOD],(int)thispars->pars[P_PERIOD],pix_period,pix_period));
    if (thispars->pars[P_PERIOD] != pix_period) {
        SETFRAMEPARS_SET(P_PERIOD, pix_period);
    }
    // Update period from external trigger (assuming internal/self trigger, we do not know real external trigger period)
    if ((thispars->pars[P_TRIG]!=0) && (thispars->pars[P_TRIG_PERIOD] > pix_period))  pix_period=thispars->pars[P_TRIG_PERIOD];
    // schedule updating P_FP1000S if it changed
    sclk=thispars->pars[P_CLK_SENSOR] ; ///pixel clock, in Hz
#if USELONGLONG
    ull_fp1000s=((long long) 1000)* ((long long) sclk);
    __div64_32(&ull_fp1000s,pix_period);
    fp1000s= ull_fp1000s;
    //  fp1000s= ((long long) 1000)* ((long long)sclk) /pix_period;
#else
    fp1000s= 10*sclk/(pix_period/100);
#endif
    MDF4(printk("thispars->pars[P_FP1000S] =%d(0x%x), fp1000s=%d(0x%x)\n",(int)thispars->pars[P_FP1000S],(int)thispars->pars[P_FP1000S],fp1000s,fp1000s));
    if (thispars->pars[P_FP1000S] != fp1000s) {
        SETFRAMEPARS_SET(P_FP1000S, fp1000s);
    }
    // schedule updating P_MT9X001_VERTBLANK sensor register and shadow
    MDF4(printk("thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK] =%d(0x%x), vert_blank=%d(0x%x)\n",(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK],(int)thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK],vert_blank,vert_blank));
    if (vert_blank != thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_VERTBLANK, vert_blank);
        MDF4(printk("  SET_SENSOR_MPAR(0x%x, 0x%x,0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_VERTBLANK, (int) vert_blank));
    }
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}

/** Program sensor exposure */
int mt9x001_pgm_exposure       (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
{
#if USELONGLONG
    uint64_t ull_fp1000s;
    uint64_t ull_exposure;
    uint64_t ull_video_exposure;
#endif
    int fp1000s;
    struct frameparspair_t pars_to_update[16]; // maximum 7? registers updated
    int nupdate=0;
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    int video_exposure =  thispars->pars[P_VEXPOS];
    int exposure;
    MDF4(printk("sensor_port=%d,  frame16=%d, frame=0x%lx (%s)exposure=0x%lx, (%s)video_exposure=0x%lx\n", sensor_port, frame16, thispars->pars[P_FRAME], FRAMEPAR_MODIFIED(P_EXPOS)?"*":" ",thispars->pars[P_EXPOS],FRAMEPAR_MODIFIED(P_VEXPOS)?"*":" ",thispars->pars[P_VEXPOS] ));
    int use_vexp=0;
    int sclk=thispars->pars[P_CLK_SENSOR] ; // pixel clock, in Hz
    //  MDF1(printk(" sclk=0x%x\n",(int) sclk)); //  5b8d800 correct
    int styp = sensor->sensorType & 7;
    //  MDF1(printk(" styp=0x%x\n",styp)); // 0x4

    // NOTE: Is decimation taken care of here ??? FIXME: probably not (here and when deciding to use exposure, not number of lines)!!!
    int row_time_in_pixels=thispars->pars[P_VIRT_WIDTH ];
    MDF1(printk(" row_time_in_pixels=0x%x\n", row_time_in_pixels)); // 0
    //  int vert_blank=        thispars->pars[P_SENSOR_REGS+P_MT9X001_VERTBLANK ];
    MDF1(printk(" vert_blank=0x%x\n",vert_blank)); // 0
    // if video exposure is non-zero, P_VEXPOS is marked as modified or P_EXPOS is not modified - use video exposure (lines), else - absolute exposure (usec)
    if ((video_exposure>0) && (FRAMEPAR_MODIFIED(P_VEXPOS) || ! (FRAMEPAR_MODIFIED(P_EXPOS) || FRAMEPAR_MODIFIED(P_VIRT_WIDTH)) )) { // use number of lines
        MDF1(printk(" exposure=%d (0x%x), video_exposure=%d (0x%x)\n", (int) thispars->pars[P_VEXPOS], (int) thispars->pars[P_VEXPOS], (int) video_exposure, (int) video_exposure));
#if USELONGLONG
        ull_exposure= ((long long)(video_exposure * row_time_in_pixels)) * ((long long) 1000000);
        __div64_32(&ull_exposure, sclk);
        exposure= ull_exposure;
#else
        exposure = (100*video_exposure * row_time_in_pixels) / (sclk/10000); // in microseconds
#endif
        use_vexp=1;
    } else { // use time in microseconds
        exposure = thispars->pars[P_EXPOS];
#if USELONGLONG
        ull_video_exposure= (long long) exposure  * (long long) sclk;
        __div64_32(&ull_video_exposure, row_time_in_pixels);
        __div64_32(&ull_video_exposure, 1000000);
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
        __div64_32(&ull_exposure, sclk);
        exposure= ull_exposure;
#else
        exposure = (100*video_exposure * row_time_in_pixels) / (sclk/10000); // in microseconds
#endif
    }

    // is exposure longer then maximal period (specified for constant fps?
    int pix_period=video_exposure*row_time_in_pixels;
    if (thispars->pars[P_FPSFLAGS] & 2) {
        if (pix_period > thispars->pars[P_PERIOD_MAX]) {
            video_exposure=thispars->pars[P_PERIOD_MAX]/row_time_in_pixels;
#if USELONGLONG
            ull_exposure= (((long long) thispars->pars[P_PERIOD_MAX]) *((long long)  1000000));
            __div64_32(&ull_exposure, sclk);
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
            __div64_32(&ull_fp1000s,pix_period);
            fp1000s= ull_fp1000s;
#else
            fp1000s= 10*sclk/(pix_period/100);
#endif
            D1(printk(" fp1000s=%d (0x%x)", (int) fp1000s, (int) fp1000s));

            if (thispars->pars[P_FP1000S] != fp1000s) {
                SETFRAMEPARS_SET(P_FP1000S, fp1000s);
            }
        }
    }
    //  MDF1(printk(" exposure=0x%x, video_exposure\n", (int) exposure, (int) video_exposure,));
    // is video exposure P_VEXPOS modified?
    if (thispars->pars[P_VEXPOS] != video_exposure) {
        SETFRAMEPARS_SET(P_VEXPOS, video_exposure);
    }
    // is exposure P_EXPOS modified?
    if (thispars->pars[P_EXPOS] != exposure) {
        SETFRAMEPARS_SET(P_EXPOS, exposure);
    }
    // Now sensor registers
    // schedule updating P_MT9X001_VERTBLANK sensor register and shadow
    // high word of shutter width (>=3MPix)
    if (((styp == MT9T_TYP) || (styp == MT9P_TYP)) && ((video_exposure >> 16) != thispars->pars[P_SENSOR_REGS+P_MT9X001_SHTRWDTHU])) {
        SET_SENSOR_MBPAR(sensor_port, frame16,sensor->i2c_addr, P_MT9X001_SHTRWDTHU, video_exposure >> 16);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_SHTRWDTHU, (int) (video_exposure >> 16)));
    }
    // low word of shutter width (all sensors)
    if ((video_exposure & 0xffff) != thispars->pars[P_SENSOR_REGS+P_MT9X001_SHTRWDTH]) {
        SET_SENSOR_MBPAR(sensor_port, frame16,sensor->i2c_addr, P_MT9X001_SHTRWDTH, video_exposure & 0xffff);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int)  P_MT9X001_SHTRWDTH, (int) (video_exposure & 0xffff)));
    }
    //  MDF1(printk(" nupdate=0x%x\n", (int) nupdate));
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    MDF1(printk(" exposure=%d (0x%x), video_exposure=%d (0x%x) OK!\n", (int) exposure, (int) exposure, (int) video_exposure, (int) video_exposure));
    return 0;
}

#define SHIFT_DGAIN 1 // shift digital gain right, so 1.0 is 0x8000 an the full range is 4:1 - make it a parameter?

/** Split full gain (0x1000~1.0) into analog register gain and residual digital gain (also 0x10000~1.0)
 *  provide some hysteresis for analog gain (1/2 step) when goin to positive, but never let residual
 *  gain to be <1.0 (0x10000). Uses gain correction table.  */
unsigned long gain_ajust_mt9x001(
        unsigned long   gain,       ///< required full gain
        unsigned long * gainTab,    ///< pointer to 81 element table of actual gains for each step. Uses calculated ones after init
        unsigned long   curRegGain, ///< current value of the sensor gain register
        unsigned long * newRegGain, ///< pointer to the new value of the sensor gain register
        unsigned long   anaGainEn,  ///< enable analog gain adjustment (0 - use current)
        unsigned long   minGain,    ///< minimal allowed value of the analog gain (normally 0x10000 ~ 1.0)
        unsigned long   maxGain)    ///< maximal allowed value of the analog gain (normally 0xfc000 ~ 15.75)
                                    ///< @return residual value of the digital gain (>=1.0 0x10000) except limited by the minGain
{
    MDF4(printk(" gain=0x%lx, curRegGain=0x%lx, minGain=0x%lx, maxGain=0x%lx\n",gain,curRegGain,minGain,maxGain));
    int g=gain;
    int gainIndex;     // index of the gain value in the table (each register value has index, each index has preferrable register value)
    int curGainIndex;  // current index in the gains table matching sensor register value
    uint64_t ull_gain;
    curRegGain &=0x7f;    // ignoring sensor digital gains
    // find out gain index for the current value of the sensor gain register
    curGainIndex=(curRegGain<=32)?curRegGain:((curRegGain>=80)?(curRegGain-48):
            ((curRegGain>=64)? ((curRegGain-64)<<1):((curRegGain+32)>>1))); ///this line handles bad register values
    if (anaGainEn) {
        if (g<minGain) g=minGain;
        if (g>maxGain) g=maxGain;
        // calculate theoretical gainIndex (0..80)
        gainIndex=(g<=0x40000)?(g>>13):((g-0x40000)>>14);
        MDF4(printk("gainIndex=0x%x\n",gainIndex));
        // adjust using gain table
        while ((gainIndex>0)  && (gainTab[gainIndex-1]>=g) && (gainTab[gainIndex-1]>=minGain)) gainIndex--; // adjust down
        while ((gainIndex<80) && (gainTab[gainIndex+1]< g) && (gainTab[gainIndex+1]<=maxGain)) gainIndex++; // adjust up
        MDF4(printk("gainIndex=0x%x\n",gainIndex));
        // Apply hysteresis
        if ((gainIndex==(curGainIndex+1)) && (((gainTab[gainIndex]+gainTab[gainIndex+1])>>1)>g)) gainIndex--;
        MDF4(printk("gainIndex=0x%x\n",gainIndex));
        //  did the analog gain chnage?
        *newRegGain=(gainIndex==curGainIndex)?curRegGain: // no - it is the same
                ((gainIndex<=32)?gainIndex:(gainIndex+48));
    }else {
        gainIndex=curGainIndex;
        *newRegGain=curRegGain;
    }
    MDF4(printk("*newRegGain=0x%lx\n",*newRegGain));
    // now divide gains
    ull_gain =((long long) gain) << 16;
    __div64_32(&ull_gain, gainTab[gainIndex]);
    MDF4(printk("((unsigned long) ull_gain)=0x%lx\n",((unsigned long) ull_gain)));
    return ((unsigned long) ull_gain) >> SHIFT_DGAIN;
}


/** Calculates hardware specific analog gains.
 * Changed to rounding (was truncaiting)*/
inline int gain_mt9x001(int g,          ///< gain value (integer, 256 for unity gain)
                        int maxGain256) ///< maximal supported gain (integer, 256 for unity gain)
                                        ///<  @return hardware gain value
{
    if(g > maxGain256)
        g = maxGain256;
    if(g <= 0x3f0)
        g=  (g+0x10) >> 5;
    else
        g = ((g+ 0X20) >> 6) + 0x40;
    return g;
}
/* truncating
inline int gain_mt9x001(int g, int maxGain256) {
   if(g > maxGain256)
      g = maxGain256;
   if(g <= 0x400)
      g >>= 5;
   else 
      g = (g >> 6) + 0x40;
   return g;
}
 */

/** Apply scale (0x10000~1.0) to data using 64-bit intermediate data */
inline unsigned long applyScale16 (unsigned long data,  ///< 32-bit unsigned data
                                   unsigned long scale) ///< 32 bit (0x10000 for scale 1.0)
                                                        ///< @return scaled result
{
    return  (unsigned long) ((((long long) data)  * scale) >> 16);
}

/** Calculate ratio of the two 32-bit numbers, scaling it by 16 bits, so equal numbers will result in 0x10000 (1.0) using 64 by 32 bit division */
inline unsigned long getScale16 (unsigned long nominator,   ///< 32-bit nominator
                                 unsigned long denominator) ///< 32-bit denominator
                                                            ///< 32 bit result scaled by 16 bits
{
    uint64_t ull_result =((long long) nominator) << 16;
#if ELPHEL_DEBUG
    unsigned long * l_result= (unsigned long *) &ull_result;
#endif
    MDF4(printk("l_result[1]=0x%lx, l_result[0]=0x%lx\n",l_result[1],l_result[0]));
    __div64_32(&ull_result, denominator);
    MDF4(printk("l_result[1]=0x%lx, l_result[0]=0x%lx\n",l_result[1],l_result[0]));
    return (unsigned long) ull_result;
}


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
int mt9x001_pgm_gains      (int sensor_port,               ///< sensor port number (0..3)
                            struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                            struct framepars_t * thispars, ///< sensor current parameters
                            struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                            int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                           ///< be applied to,  negative - ASAP
                                                           ///< @return 0 - OK, negative - error

{
    struct frameparspair_t pars_to_update[38]; // 22+12 needed
    int nupdate=0;
    MDF4(printk(" frame16=%d\n",frame16));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    unsigned long newRegGain,digitalGain, testmode;
    unsigned long anaGainEn= (thispars->pars[P_GAIN_CTRL]>> GAIN_BIT_ENABLE) & 1;
    unsigned long minAnaGain=thispars->pars[P_GAIN_MIN];
    unsigned long maxAnaGain=thispars->pars[P_GAIN_MAX];
    unsigned long maxGain;
    int limitsModified=0;
    int gaingModified=FRAMEPAR_MODIFIED(P_GAING);
    ///make sure limits are OK
    if (FRAMEPAR_MODIFIED(P_GAIN_MIN)) {
        limitsModified=1;
        if (minAnaGain < 0x10000) {
            minAnaGain = 0x10000;
            SETFRAMEPARS_SET(P_GAIN_MIN, minAnaGain);
        }
    }
    if (FRAMEPAR_MODIFIED(P_GAIN_MAX)) {
        limitsModified+=2;
        if (maxAnaGain > (sensor->maxGain256 << 8)) { ///sensor->maxGain256 is not calibrated, so digital gain should be able to accomodate for variations
            maxAnaGain = (sensor->maxGain256 << 8);
            SETFRAMEPARS_SET(P_GAIN_MIN, maxAnaGain);
        }
    }
    //  maxGain= maxAnaGain * (MAX_DIGITAL_GAIN >> 8);
    maxGain= (maxAnaGain * MAX_DIGITAL_GAIN)  >> 8; ///should not overflow for Micron as max digital gain <4.0 (0x400), max analog <0x100000)

    unsigned long gainr= thispars->pars[P_GAINR ];
    unsigned long gaing= thispars->pars[P_GAING ];
    unsigned long gaingb=thispars->pars[P_GAINGB];
    unsigned long gainb= thispars->pars[P_GAINB ];
    unsigned long rscale_all=thispars->pars[P_RSCALE_ALL];
    unsigned long gscale_all=thispars->pars[P_GSCALE_ALL];
    unsigned long bscale_all=thispars->pars[P_BSCALE_ALL];
    unsigned long rscale=rscale_all & ((1<<CSCALES_WIDTH)-1);
    unsigned long gscale=gscale_all & ((1<<CSCALES_WIDTH)-1);
    unsigned long bscale=bscale_all & ((1<<CSCALES_WIDTH)-1);
    unsigned long rscale_ctl=(rscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);
    unsigned long gscale_ctl=(gscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);
    unsigned long bscale_ctl=(bscale_all >> CSCALES_CTL_BIT) & ((1<<CSCALES_CTL_WIDTH)-1);
    unsigned long newval;

    ///
    // FIXME: use different gain limitation when anaGainEn==0 (from current analog channel gain to that * MAX_DIGITAL_GAIN>>8),
    //        make P_GAIN_CTRL trigger this function
    ///

    // scales will not be modified if they make gains out of limit, but gains will be. So automatic white balance should deal with gains, not with scales.
    // Preserving application-set values for scales simplifies recovery when the green gain is adjusted so all colors fit in the limits

    MDF4(printk("gainr=0x%lx, gaing=0x%lx, gaingb=0x%lx, gainb=0x%lx, rscale_all=0x%lx, gscale_all=0x%lx, bscale_all=0x%lx\n",gainr, gaing, gaingb, gainb, rscale_all, gscale_all, bscale_all));
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
            && (rscale_ctl== CSCALES_CTL_NORMAL)                                // update red from rscale is enabled
            && !FRAMEPAR_MODIFIED(P_GAINR) )  {                                 // red gain is not specifically modified
        //   update red gain to rscale and gaing, limit it if it is out of range
        if (((newval=applyScale16(gaing,rscale)))!=gainr) {
            if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
            if (gainr!=newval) { // don't update if it was already limited in previous frames to the same value
                gainr=newval;
                SETFRAMEPARS_SET(P_GAINR, gainr);
            }
        }
    }

    if ((gaingModified || (FRAMEPAR_MODIFIED(P_GSCALE_ALL)))
            && (gscale_ctl== CSCALES_CTL_NORMAL)
            && !FRAMEPAR_MODIFIED(P_GAINGB) ) {
        //   update green2 gain to gscale and gaing
        if (((newval=applyScale16(gaing,gscale)))!=gaingb) {
            if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
            if (gaingb!=newval) { // don't update if it was already limited in previous frames to the same value
                gaingb=newval;
                SETFRAMEPARS_SET(P_GAINGB, gaingb);
            }
        }
    }
    if ((gaingModified || (FRAMEPAR_MODIFIED(P_BSCALE_ALL)))
            && (bscale_ctl== CSCALES_CTL_NORMAL)
            && !FRAMEPAR_MODIFIED(P_GAINB) ) {
        //   update blue gain to bscale and gaing
        if (((newval=applyScale16(gaing,bscale)))!=gainb) {
            if      (newval < minAnaGain) newval = minAnaGain;
            else if (newval > maxGain)    newval = maxGain;
            if (gainb!=newval) { // don't update if it was already limited in previous frames to the same value
                gainb=newval;
                SETFRAMEPARS_SET(P_GAINB, gainb);
            }
        }
    }
    // Update scales only if the corresponding color gains (not the base green one) were modified outside of the driver
    // (not as a result of being limited by minimal/maximal gains)
    if ((FRAMEPAR_MODIFIED(P_GAINR))  && (rscale_ctl!= CSCALES_CTL_DISABLE)) {
        rscale=getScale16(gainr, gaing);
        MDF4(printk("gainr=0x%lx, gaing=0x%lx, rscale=0x%lx\n",gainr, gaing, rscale));
    }
    if ((FRAMEPAR_MODIFIED(P_GAINGB)) && (gscale_ctl!= CSCALES_CTL_DISABLE)) {
        gscale=getScale16(gaingb,gaing);
        MDF4(printk("gaingb=0x%lx, gaing=0x%lx, gscale=0x%lx\n",gaingb, gaing, gscale));
    }
    if ((FRAMEPAR_MODIFIED(P_GAINB))  && (bscale_ctl!= CSCALES_CTL_DISABLE)) {
        bscale=getScale16(gainb, gaing);
        MDF4(printk("gainb=0x%lx, gaing=0x%lx, bscale=0x%lx\n",gainb, gaing, bscale));
    }
    // remove recalc flag
    if (rscale_ctl == CSCALES_CTL_RECALC) rscale_ctl = CSCALES_CTL_NORMAL;
    if (gscale_ctl == CSCALES_CTL_RECALC) gscale_ctl = CSCALES_CTL_NORMAL;
    if (bscale_ctl == CSCALES_CTL_RECALC) bscale_ctl = CSCALES_CTL_NORMAL;
    // update P_*SCALE if either scale or scale control changed
    if (((newval=((rscale_ctl<<CSCALES_CTL_BIT) | (rscale & ((1<<CSCALES_WIDTH)-1)))))!=rscale_all) {
        SETFRAMEPARS_SET(P_RSCALE, newval);
        MDF4(printk("newval=0x%lx\n",newval));
    }
    MDF4(printk("newval=0x%lx\n",newval));
    if (((newval=((gscale_ctl<<CSCALES_CTL_BIT) | (gscale & ((1<<CSCALES_WIDTH)-1)))))!=gscale_all) {
        SETFRAMEPARS_SET(P_GSCALE, newval);
        MDF4(printk("newval=0x%lx\n",newval));
    }
    MDF4(printk("newval=0x%lx\n",newval));
    if (((newval=((bscale_ctl<<CSCALES_CTL_BIT) | (bscale & ((1<<CSCALES_WIDTH)-1)))))!=bscale_all) {
        SETFRAMEPARS_SET(P_BSCALE, newval);
        MDF4(printk("newval=0x%lx\n",newval));
    }
    MDF4(printk("newval=0x%lx\n",newval));
    MDF4(printk("gainr=0x%lx, gaing=0x%lx, gaingb=0x%lx, gainb=0x%lx, rscale_all=0x%lx, gscale_all=0x%lx, bscale_all=0x%lx\n",gainr, gaing, gaingb, gainb, rscale_all, gscale_all, bscale_all));

    // Third part: Split overall gains into analog and digital components

    // Split required gain for red into analog register change (with half step hysteresis) and

    digitalGain= gain_ajust_mt9x001(gainr,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_RED<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9X001_RED],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);
    // apply sensor register gain red if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9X001_RED]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_RED, newRegGain);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_RED, (int) newRegGain));
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the red channel
    if (digitalGain != thispars->pars[P_DGAINR ]) {
        SETFRAMEPARS_SET(P_DGAINR, digitalGain);
    }

    // Split required gain for green into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9x001(gaing,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_GREEN1<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9X001_GREEN1],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);
    // apply sensor register gain green if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9X001_GREEN1]) {
        SET_SENSOR_MBPAR(sensor_port, frame16,sensor->i2c_addr, P_MT9X001_GREEN1, newRegGain);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_GREEN1, (int) newRegGain));
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the green channel
    if (digitalGain != thispars->pars[P_DGAING ]) {
        SETFRAMEPARS_SET(P_DGAING, digitalGain);
    }
    // Split required gain for blue into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9x001(gainb,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_BLUE<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9X001_BLUE],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);
    // apply sensor register gain blue if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9X001_BLUE]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr, P_MT9X001_BLUE, newRegGain);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%0x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_BLUE, (int) newRegGain));
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the blue channel
    if (digitalGain != thispars->pars[P_DGAINB ]) {
        SETFRAMEPARS_SET(P_DGAINB, digitalGain);
    }

    // Split required gain for second green into analog register change (with half step hysteresis) and
    digitalGain= gain_ajust_mt9x001(gaingb,
            &GLOBALPARS(sensor_port, G_SENSOR_CALIB+(COLOR_GREEN2<<8)),
            thispars->pars[P_SENSOR_REGS+P_MT9X001_GREEN2],
            &newRegGain,
            anaGainEn,
            minAnaGain,
            maxAnaGain);
    // apply sensor register gain second green if it was changed
    if (newRegGain != thispars->pars[P_SENSOR_REGS+P_MT9X001_GREEN2]) {
        SET_SENSOR_MBPAR(sensor_port, frame16,sensor->i2c_addr, P_MT9X001_GREEN2, newRegGain);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_GREEN2, (int) newRegGain));
    }
    // schedule application of (residual after analog gain adjustment) digital gain to the second green channel
    if (digitalGain != thispars->pars[P_DGAINGB ]) {
        SETFRAMEPARS_SET(P_DGAINGB, digitalGain);
    }
    // test mode off/on/select
    testmode= thispars->pars[P_TESTSENSOR ];// (on?0x10000:0) | (Micron_tests_mode) 0x0 - off, 0x10000..0x1000F
    testmode= (testmode & 0x10000)? (((testmode & 0xf) << 3) | 1) : 0;
    if (testmode != thispars->pars[P_SENSOR_REGS+P_MT9X001_TEST]) {
        SET_SENSOR_MBPAR(sensor_port, frame16, sensor->i2c_addr , P_MT9X001_TEST, testmode) ;
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_TEST, (int) testmode));

    }
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}
#if 0
#define P_TESTSENSOR    44 // sensor test mode(s) 0x10000 - enable, lower bits - test mode

#define P_MT9X001_TEST     0xa0  ///test patterns. Probably only in MT9P001?
/*! bits 6:3:
              0: color field
              1: horizontal gradient
              2: vertical gradient
              3: diagonal
              4: classic
              5: marching 1's
              6: monochrome horizontal bars
              7: monochrome vertical bars
              8: vertical color bars
              Legal values: [0, 15].
 bit    2     Reserved
 bit    1     Reserved
 bit    0     Enable_Test_Pattern.   Enables the test pattern. When set, data from the ADC will be replaced with a digitally
              generated test pattern specified by Test_Pattern_Mode.
 */

#endif




/** Program trigger mode as sensor-specific  */
int mt9x001_pgm_triggermode        (int sensor_port,               ///< sensor port number (0..3)
                                    struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                    struct framepars_t * thispars, ///< sensor current parameters
                                    struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                    int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                                   ///< be applied to,  negative - ASAP
                                                                   ///< @return 0 - OK, negative - error
{
    MDF4(printk(" frame16=%d\n",frame16));
    struct frameparspair_t pars_to_update[4]; ///
    int nupdate=0;
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    unsigned long newreg= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1] & 0xfeff) | ((thispars->pars[P_TRIG] & 4)?0x100:0);
    if (newreg != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1]) {
        SET_SENSOR_MBPAR(sensor_port, frame16,sensor->i2c_addr, P_MT9X001_RMODE1, newreg);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", sensor_port, frame16,  (int) sensor->i2c_addr, (int) P_MT9X001_RMODE1, (int) newreg));
    }
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
    return 0;
}

/** Program sensor registers (probably just those that are manually set)
 * NOTE: all modes but ASAP are limited to 64 registers/frame, no overflow checks are performed! */

int mt9x001_pgm_sensorregs     (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error

{
    MDF4(printk(" frame8=%d\n",frame8));
    if (frame16 >= PARS_FRAMES) return -1; // wrong frame
    //  send all parameters marked as "needed to be processed" to the sensor, clear those flags
    // mask out all non sensor pars
    //  unsigned long bmask32= ((thispars->mod32) >> (P_SENSOR_REGS>>5)) & (P_SENSOR_NUMREGS-1) ;
    // It will be the first for the frame (before automatic sensor changes).
    // Add testing for programmed sensor and move vbalues to later frames (not here butin the pgm_functions)

    //  unsigned long bmask32= ((thispars->mod32) >> (P_SENSOR_REGS>>5)) & (P_SENSOR_NUMREGS-1) ; // wromg!, only valid for P_SENSOR_NUMREGS==256 (that is the case, actually)
    unsigned long bmask32= ((thispars->mod32) >> (P_SENSOR_REGS>>5)) & (( 1 << (P_SENSOR_NUMREGS >> 5))-1) ;

    MDF4(printk(" bmask32=0x%lx, thispars->mod32=0x%lx, P_SENSOR_REGS=0x%x, P_SENSOR_NUMREGS=0x%x\n",bmask32,thispars->mod32,P_SENSOR_REGS,P_SENSOR_NUMREGS));
    unsigned long mask;
    int index,index32;
    struct frameparspair_t pars_to_update[2*P_MULTI_NUMREGS*MAX_SENSORS]; ///
    int nupdate=0;

    if (GLOBALPARS(sensor_port, G_MULTI_NUM)==0) { // single sensor,don't bother with individual registers, do all old way.
        if (bmask32) {
            for (index32=(P_SENSOR_REGS>>5); bmask32; index32++, bmask32 >>= 1) {
                MDF4(printk(" index32=0x%x, bmask32=0x%lx\n",index32,bmask32));
                if (bmask32 & 1) {
                    mask=thispars->mod[index32];
                    MDF4(printk(" mask=0x%lx\n",mask));
                    for (index=(index32<<5); mask; index++, mask >>= 1) {
                        if (mask & 1) {
                            X3X3_I2C_SEND2(sensor_port, frame16, sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]);
                            MDF4(printk("X3X3_I2C_SEND2(0x%x, 0x%x, 0x%lx,0x%x,0x%lx)\n",sensor_port, frame16,sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]));
                        }
                    }
                    thispars->mod[index32]=0;
                }
            }
            thispars->mod32=0;
        }
    } else {
        //   if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to gains and sensor register shadows
        ///pass 1 - process modified broadcast parameters
        if (bmask32) {
            for (index32=(P_SENSOR_REGS>>5); bmask32; index32++, bmask32 >>= 1) {
                MDF4(printk(" index32=0x%x, bmask32=0x%lx\n",index32,bmask32));
                if (bmask32 & 1) {
                    mask=thispars->mod[index32];
                    MDF4(printk(" mask=0x%lx\n",mask));
                    for (index=(index32<<5); mask; index++, mask >>= 1) {
                        if (mask & 1) {
                            // apply broadcast  register write and schedule change the individual ones (if they are not modified)
                            SET_SENSOR_MBOPAR(sensor_port, frame16, sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]);
                            MDF4(printk("SET_SENSOR_MBOPAR(0x%x, 0x%x, 0x%lx,0x%x,0x%lx)\n",sensor_port, frame16, sensor->i2c_addr,(index-P_SENSOR_REGS),thispars->pars[index]));
                        }
                    }
                    thispars->mod[index32]=0;
                }
            }
            thispars->mod32=0;
        }
        // Pass2 - process individual sensor parameters (possibly overwriting broadcast ones)
        bmask32= ((thispars->mod32) >> (P_MULTI_REGS >> 5)) & (( 1 << ((((MAX_SENSORS) * (P_MULTI_NUMREGS)) >> 5)+1))-1) ; // added one not to miss parameters with the last group of 32
        if (bmask32) {
            for (index32=(P_MULTI_REGS>>5); bmask32; index32++, bmask32 >>= 1) {
                MDF4(printk(" index32=0x%x, bmask32=0x%lx\n",index32,bmask32));
                if (bmask32 & 1) {
                    mask=thispars->mod[index32];
                    MDF4(printk(" mask=0x%lx\n",mask));
                    for (index=(index32<<5); mask && (index < ((P_MULTI_REGS) + ((MAX_SENSORS) * (P_MULTI_NUMREGS)))); index++, mask >>= 1) {
                        if (mask & 1) {
                            if ((MULTIRVRSREG(sensor_port, index)) >0 ) {
                                X3X3_I2C_SEND2(sensor_port,
                                               frame16,
                                               sensor->i2c_addr + ((MULTIRVRSREG(sensor_port,index)>> 16) * I2C359_INC),
                                               ((MULTIRVRSREG(sensor_port,index) & 0xffff)-P_SENSOR_REGS),
                                               thispars->pars[index]);
                                MDF4(printk("  X3X3_I2C_SEND2(0x%x, 0x%x, 0x%lx,0x%x,0x%lx)\n",
                                               sensor_port,
                                               frame16,
                                               sensor->i2c_addr+((MULTIRVRSREG(sensor_port,index)>> 16)*I2C359_INC),
                                               (int) ((MULTIRVRSREG(sensor_port,index) & 0xffff)-P_SENSOR_REGS),
                                               thispars->pars[index]));
                            }
                        }
                    }
                    thispars->mod[index32]=0;
                }
            }
            thispars->mod32=0;
        }
    }
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return 0;
} 

//#define MAX_SENSORS 3 // maximal number of sensor attached (modify some hard-wired constants below if this to be changed)
//#define G_MULTI_NUM     (FRAMEPAR_GLOBALS + 34) // Actual number of parameters that are individual for different channels (limited by P_MULTI_NUMREGS)
//#define P_MULTI_NUMREGS  32  // up to 32 sensor register may have individual values
//#define P_MULTI_REGS          (P_SENSOR_REGS + P_SENSOR_NUMREGS) // 32-words aligned
//I2C359_INC
//#define FRAMEPAR_MODIFIED(x) (thispars->mod[(x) >> 5] & (1<<((x) & 0x1f)))
// #define MULTIRVRSREG(x) (multiSensRvrsIndex[x])
