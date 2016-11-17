/*!********************************************************************************
* file   param_depend.h
*! DESCRIPTION: Specifies, which actions should be performed when some acquisition
*! parameters are changed
*! Copyright (C) 2008 Elphel, Inc.
*! -----------------------------------------------------------------------------**
*!
*!  This program is free software: you can redistribute it and/or modify
*!  it under the terms of the GNU General Public License as published by
*!  the Free Software Foundation, either version 3 of the License, or
*!  (at your option) any later version.
*!
*!  This program is distributed in the hope that it will be useful,
*!  but WITHOUT ANY WARRANTY; without even the implied warranty of
*!  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*!  GNU General Public License for more details.
*!
*!  You should have received a copy of the GNU General Public License
*!  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*! -----------------------------------------------------------------------------**
*!  $Log: param_depend.h,v $
*!  Revision 1.12  2010/08/03 23:37:34  elphel
*!  rev 8.0.8.37, portrait mode support
*!
*!  Revision 1.11  2010/08/01 19:29:24  elphel
*!  files updated to support coring function for noise filtering
*!
*!  Revision 1.10  2010/06/02 16:31:04  elphel
*!  Added P_MULTI_SELECTED parameter
*!
*!  Revision 1.9  2010/06/01 08:30:36  elphel
*!  support for the FPGA code 03534022  with optional master time stamp over the inter-camera sync line(s)
*!
*!  Revision 1.8  2010/05/25 00:52:23  elphel
*!  8.0.8.20, working on multi-sensor
*!
*!  Revision 1.7  2010/05/21 06:12:16  elphel
*!  continue working on multi-sensor software
*!
*!  Revision 1.6  2010/05/16 02:03:47  elphel
*!  8.0.8.4 - driver working with individual/broadcast sensor registers
*!
*!  Revision 1.5  2010/05/13 03:39:31  elphel
*!  8.0.8.12 -  drivers modified for multi-sensor operation
*!
*!  Revision 1.4  2010/04/28 16:00:09  elphel
*!  Changing PF_HEIGHT now triggers window recalculation
*!
*!  Revision 1.3  2008/12/03 17:17:23  elphel
*!  added limitfps function to exposure change - otherwise indicated period/fps was not updated when exposure time was reduced (from above minimal frame period)
*!
*!  Revision 1.2  2008/11/30 05:01:03  elphel
*!  Changing gains/scales behavior
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:01  elphel
*!
*!
*!  Revision 1.20  2008/11/27 09:27:31  elphel
*!  Support fro new parameters (vignetting correction related)
*!
*!  Revision 1.19  2008/11/13 05:40:45  elphel
*!  8.0.alpha16 - modified histogram storage, profiling
*!
*!  Revision 1.18  2008/10/29 04:18:28  elphel
*!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*!
*!  Revision 1.17  2008/10/18 06:14:21  elphel
*!  8.0.alpha4 - removed some obsolete parameters, renumbered others, split P_FLIP into P_FLIPH and P_FLIPV (different latencies because of bad frames), pgm_window-> pgm_window, pgm_window_safe
*!
*!  Revision 1.16  2008/10/10 17:06:59  elphel
*!  just a snapshot
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
*!  Revision 1.11  2008/09/25 00:58:12  elphel
*!  snapshot
*!
*!  Revision 1.10  2008/09/22 22:55:49  elphel
*!  snapshot
*!
*!  Revision 1.9  2008/09/16 00:49:32  elphel
*!  snapshot
*!
*!  Revision 1.8  2008/08/11 19:17:01  elphel
*!  reduced syntax complaints by KDevelop
*!
*!  Revision 1.7  2008/07/29 01:15:06  elphel
*!  another snapshot
*!
*!  Revision 1.6  2008/07/27 23:25:07  elphel
*!  next snapshot
*!
*!  Revision 1.5  2008/07/27 04:27:49  elphel
*!  next snapshot
*!
*!  Revision 1.4  2008/06/24 00:43:44  elphel
*!  just a snapshot
*!
*!  Revision 1.3  2008/06/20 03:54:20  elphel
*!  another snapshot
*!
*!  Revision 1.2  2008/06/19 02:17:36  elphel
*!  continuing work - just a snapshot
*!
*!  Revision 1.1  2008/06/16 06:51:21  elphel
*!  work in progress, intermediate commit
*!
*!
*/
#ifndef _PARAM_DEPEND_H
#define _PARAM_DEPEND_H

#define   ONCHANGE_MULTISENS     (1 << onchange_multisens )      /// changes related to multiplexed sensors
#define   ONCHANGE_RECALCSEQ     (1 << onchange_recalcseq )      /// recalculate sequences/latencies, according to P_SKIP, P_TRIG
#define   ONCHANGE_DETECTSENSOR  (1 << onchange_detectsensor )   /// detect sensor type, sets sensor structure (capabilities), function pointers
#define   ONCHANGE_SENSORPHASE   (1 << onchange_sensorphase )    /// program sensor clock/phase
#define   ONCHANGE_I2C           (1 << onchange_i2c )            /// program i2c speed/bytes
#define   ONCHANGE_INITSENSOR    (1 << onchange_initsensor )     /// resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
#define   ONCHANGE_AFTERINIT     (1 << onchange_afterinit )      /// restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
#define   ONCHANGE_WINDOW        (1 << onchange_window )         /// program sensor WOI - with 1 bad frame to be skipped by the compressor
#define   ONCHANGE_WINDOW_SAFE   (1 << onchange_window_safe )    /// program sensor WOI - zero bad frames to be skipped by the compressor
#define   ONCHANGE_EXPOSURE      (1 << onchange_exposure )       /// program exposure
#define   ONCHANGE_GAINS         (1 << onchange_gains )          /// program analog gains
#define   ONCHANGE_TRIGGERMODE   (1 << onchange_triggermode )    /// program sensor trigger mode
#define   ONCHANGE_SENSORIN      (1 << onchange_sensorin )       /// program sensor input in FPGA (Bayer, 8/16 bits, ??)
#define   ONCHANGE_SENSORSTOP    (1 << onchange_sensorstop)     /// Stop acquisition from the sensor to the FPGA (start has latency of 2)
#define   ONCHANGE_SENSORRUN     (1 << onchange_sensorrun)      /// Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
#define   ONCHANGE_GAMMA         (1 << onchange_gamma )          /// program gamma table
#define   ONCHANGE_HIST          (1 << onchange_hist )           /// program histogram window
#define   ONCHANGE_AEXP          (1 << onchange_aexp )           /// program autoexposure mode
#define   ONCHANGE_QUALITY       (1 << onchange_quality )        /// program quantization table(s)
#define   ONCHANGE_MEMSENSOR     (1 << onchange_memsensor )      /// program memory channels 0 (sensor->memory) and 1 (memory->FPN)
#define   ONCHANGE_MEMCOMPRESSOR (1 << onchange_memcompressor )  /// program memory channel 2 (memory->compressor)
#define   ONCHANGE_LIMITFPS      (1 << onchange_limitfps )       /// check compressor will keep up, limit sensor FPS if needed
#define   ONCHANGE_COMPMODE      (1 << onchange_compmode )       /// program compressor modes
#define   ONCHANGE_FOCUSMODE     (1 << onchange_focusmode )      /// program focus modes
#define   ONCHANGE_TRIGSEQ       (1 << onchange_trigseq )        /// program sequencer (int/ext)
#define   ONCHANGE_IRQ           (1 << onchange_irq )            /// program smart IRQ mode (needs to be on)
#define   ONCHANGE_COMPRESTART   (1 << onchange_comprestart )    /// restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
#define   ONCHANGE_COMPSTOP      (1 << onchange_compstop )       /// stop compressor when changing geometry
#define   ONCHANGE_COMPCTL       (1 << onchange_compctl )        /// only start/stop/single (after explicitly changed, not when geometry was changed)
#define   ONCHANGE_GAMMALOAD     (1 << onchange_gammaload )      /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
#define   ONCHANGE_SENSORREGS    (1 << onchange_sensorregs )     /// write sensor registers (only changed from outside the driver as they may have different latencies)?
#define   ONCHANGE_PRESCAL       (1 << onchange_prescal )        /// change scales for per-color digital gains, apply vignetting correction

const unsigned long param_depend_tab[]= 
  {
     P_SENSOR_RUN,            ONCHANGE_SENSORSTOP | ONCHANGE_SENSORRUN | ONCHANGE_MEMCOMPRESSOR,
     P_SENSOR,                ONCHANGE_DETECTSENSOR | ONCHANGE_RECALCSEQ | ONCHANGE_INITSENSOR | ONCHANGE_AFTERINIT | ONCHANGE_MULTISENS | \
                              ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                              ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_BAYER ,                ONCHANGE_SENSORIN, // | ONCHANGE_COMPMODE , // ONCHANGE_COMPMODE added for NC393
     P_CLK_FPGA,              ONCHANGE_I2C |  ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                              ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
/// not need ONCHANGE_INITSENSOR | ONCHANGE_AFTERINIT  - they will be scheduled by pgm_sensorphase if needed for the sensor
     P_CLK_SENSOR,            ONCHANGE_SENSORPHASE | \
                              ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                              ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_SENSOR_PHASE,          ONCHANGE_SENSORPHASE | ONCHANGE_EXPOSURE | ONCHANGE_LIMITFPS | \
                              ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_FPGA_XTRA,             ONCHANGE_LIMITFPS ,
     P_TRIG,                  ONCHANGE_RECALCSEQ | ONCHANGE_TRIGGERMODE | ONCHANGE_TRIGSEQ | ONCHANGE_LIMITFPS | /// Next to call with afterinit
                                   ONCHANGE_MULTISENS |
                                   ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                   ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
//     P_VIRT_WIDTH,            ONCHANGE_LIMITFPS ,
     P_VIRT_WIDTH,            ONCHANGE_LIMITFPS | ONCHANGE_EXPOSURE , ///after limitFPS
     P_VIRT_HEIGHT,           ONCHANGE_LIMITFPS ,
     P_WOI_LEFT,              ONCHANGE_WINDOW_SAFE ,
//     P_WOI_TOP,               ONCHANGE_WINDOW_SAFE ,
     P_WOI_TOP,               ONCHANGE_WINDOW | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
     P_WOI_WIDTH,             ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_WOI_HEIGHT,            ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
///P_WOI_HEIGHT-> number of lines to acquire. Or move it to memsensor so other changes (decimation, binning) will lead to recalc number of lines?
///FLIP_H is safe, FLIPV produces bad frame
     P_FLIPH,                 ONCHANGE_WINDOW_SAFE | ONCHANGE_EXPOSURE | ONCHANGE_SENSORIN , /// bit 0 - horizontal (ONCHANGE_SENSORIN for Bayer)
     P_FLIPV,                 ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_SENSORIN | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART, /// bit  1 - vertical (ONCHANGE_SENSORIN for Bayer)
     P_FPSFLAGS,              ONCHANGE_LIMITFPS ,
     P_DCM_HOR,               ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
/// Multisensor is called without any verification of [P_DCM_VERT] (it is needed to adjust gaps between frames)
     P_DCM_VERT,              ONCHANGE_MULTISENS |
                                           ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_BIN_HOR,               ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_BIN_VERT,              ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
//Oleg 20161014: trying to restart the compressor: + ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART
//   P_COLOR,                 ONCHANGE_COMPMODE | ONCHANGE_LIMITFPS | ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
//Added ONCHANGE_WINDOW and related as JPEG <-> JP4 change window by the border pixels

     P_COLOR,                 ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,

	 //P_COLOR,                 ONCHANGE_COMPMODE | ONCHANGE_LIMITFPS | ONCHANGE_MEMCOMPRESSOR,
//     P_PF_HEIGHT,             ONCHANGE_RECALCSEQ | ONCHANGE_SENSORIN | ONCHANGE_LIMITFPS | ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_PF_HEIGHT,             ONCHANGE_RECALCSEQ | ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_BITS,                  ONCHANGE_SENSORIN | ONCHANGE_LIMITFPS | ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_FPGATEST,              ONCHANGE_SENSORIN ,
     P_FPNS,                  ONCHANGE_SENSORIN ,
     P_FPNM,                  ONCHANGE_SENSORIN ,
     P_VIRTTRIG,              ONCHANGE_SENSORIN ,
     P_COMP_BAYER,            ONCHANGE_COMPMODE , // NC393 - derivative from P_BAYER, flips, ...
     P_MEMSENSOR_DLY,         ONCHANGE_SENSORIN ,
     P_COMPMOD_BYRSH,         ONCHANGE_COMPMODE,
     P_COMPMOD_TILSH,         ONCHANGE_COMPMODE,
     P_COMPMOD_DCSUB,         ONCHANGE_COMPMODE,
     P_COMPMOD_QTAB,          ONCHANGE_COMPMODE,   // to be written not directly,but by  pgm_quality ? (pgm_gamma to be used by pgm_gammaload - wrong)

     P_FP1000SLIM,            ONCHANGE_LIMITFPS ,
     P_COLOR_SATURATION_BLUE, ONCHANGE_COMPMODE ,
     P_COLOR_SATURATION_RED,  ONCHANGE_COMPMODE ,
     P_CORING_PAGE,             ONCHANGE_COMPMODE ,
     P_AUTOEXP_ON,            ONCHANGE_AEXP ,
     P_HISTWND_RWIDTH,        ONCHANGE_HIST | ONCHANGE_AEXP ,
     P_HISTWND_RHEIGHT,       ONCHANGE_HIST | ONCHANGE_AEXP ,
     P_HISTWND_RLEFT,         ONCHANGE_HIST | ONCHANGE_AEXP ,
     P_HISTWND_RTOP,          ONCHANGE_HIST | ONCHANGE_AEXP ,
     P_AUTOEXP_EXP_MAX,       ONCHANGE_AEXP ,
     P_AUTOEXP_OVEREXP_MAX,   ONCHANGE_AEXP ,
     P_AUTOEXP_S_PERCENT,     ONCHANGE_AEXP ,
     P_AUTOEXP_S_INDEX,       ONCHANGE_AEXP ,
     P_AUTOEXP_SKIP_PMIN,     ONCHANGE_AEXP ,
     P_AUTOEXP_SKIP_PMAX,     ONCHANGE_AEXP ,
     P_AUTOEXP_SKIP_T,        ONCHANGE_AEXP ,
     P_FOCUS_SHOW,            ONCHANGE_COMPMODE , /// ONCHANGE_COMPMODE, not ONCHANGE_FOCUSMODE (it only can be done w/o the sequencer)
     P_FOCUS_SHOW1,           ONCHANGE_FOCUSMODE ,
     P_RFOCUS_LEFT,           ONCHANGE_FOCUSMODE ,
     P_RFOCUS_WIDTH,          ONCHANGE_FOCUSMODE ,
     P_RFOCUS_TOP,            ONCHANGE_FOCUSMODE ,
     P_RFOCUS_HEIGHT,         ONCHANGE_FOCUSMODE ,
     P_FOCUS_FILTER,          ONCHANGE_FOCUSMODE ,
     P_TRIG_MASTER,           ONCHANGE_TRIGSEQ ,
     P_TRIG_CONDITION,        ONCHANGE_TRIGSEQ ,
     P_TRIG_DELAY,            ONCHANGE_TRIGSEQ ,
     P_TRIG_OUT,              ONCHANGE_TRIGSEQ ,
     P_TRIG_PERIOD,           ONCHANGE_TRIGSEQ | ONCHANGE_LIMITFPS,
     P_SKIP_FRAMES,           ONCHANGE_RECALCSEQ ,
     P_I2C_QPERIOD,           ONCHANGE_I2C ,
     P_I2C_BYTES,             ONCHANGE_I2C ,
     P_I2C_EOF,               ONCHANGE_IRQ ,
     P_EXTERN_TIMESTAMP,      ONCHANGE_TRIGSEQ ,
     P_TRIG_BITLENGTH,        ONCHANGE_TRIGSEQ ,
     P_XMIT_TIMESTAMP,        ONCHANGE_TRIGSEQ ,
     P_OVERSIZE,              ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP  | ONCHANGE_COMPRESTART ,
     P_QUALITY,               ONCHANGE_QUALITY ,
     P_PORTRAIT,              ONCHANGE_QUALITY ,
     P_CORING_INDEX,          ONCHANGE_QUALITY ,

     P_TESTSENSOR,            ONCHANGE_GAINS , /// sensor test mode - now processed together with sensor gains
     P_GAINR,                 ONCHANGE_GAINS | ONCHANGE_PRESCAL ,  /// Set digital gain after adjusting analog gain 
     P_GAING,                 ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GAINGB,                ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_RSCALE_ALL,            ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GSCALE_ALL,            ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_BSCALE_ALL,            ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GAINB,                 ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GAIN_CTRL,             ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GAIN_MIN,              ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_GAIN_MAX,              ONCHANGE_GAINS | ONCHANGE_PRESCAL ,
     P_EXPOS,                 ONCHANGE_EXPOSURE | ONCHANGE_LIMITFPS, /// Otherwise it only increase period/decr. indicated FP1000S, but does not chnage when exposure is decreased
     P_VEXPOS,                ONCHANGE_EXPOSURE | ONCHANGE_LIMITFPS, /// Otherwise it only increase period/decr. indicated FP1000S, but does not chnage when exposure is decreased
     P_GTAB_R,                ONCHANGE_GAMMA | ONCHANGE_GAMMALOAD,
     P_GTAB_G,                ONCHANGE_GAMMA | ONCHANGE_GAMMALOAD,
     P_GTAB_GB,               ONCHANGE_GAMMA | ONCHANGE_GAMMALOAD,
     P_GTAB_B,                ONCHANGE_GAMMA | ONCHANGE_GAMMALOAD,
     P_COMPRESSOR_RUN,        ONCHANGE_COMPCTL,
     P_VIGNET_AX,             ONCHANGE_PRESCAL,
     P_VIGNET_AY,             ONCHANGE_PRESCAL,
     P_VIGNET_BX,             ONCHANGE_PRESCAL,
     P_VIGNET_BY,             ONCHANGE_PRESCAL,
     P_VIGNET_C,              ONCHANGE_PRESCAL,
     P_VIGNET_SHL,            ONCHANGE_PRESCAL,
     P_SCALE_ZERO_IN,         ONCHANGE_PRESCAL,
     P_SCALE_ZERO_OUT,        ONCHANGE_PRESCAL,
     P_DGAINR,                ONCHANGE_PRESCAL,
     P_DGAING,                ONCHANGE_PRESCAL,
     P_DGAINGB,               ONCHANGE_PRESCAL,
     P_DGAINB,                ONCHANGE_PRESCAL,
     P_MULTISENS_EN,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_PHASE_SDRAM,     ONCHANGE_SENSORPHASE,
     P_MULTI_PHASE1,          ONCHANGE_SENSORPHASE,
     P_MULTI_PHASE2,          ONCHANGE_SENSORPHASE,
     P_MULTI_PHASE3,          ONCHANGE_SENSORPHASE,
     P_MULTI_SEQUENCE,        ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_SELECTED,        ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_FLIPH,           ONCHANGE_WINDOW_SAFE | ONCHANGE_EXPOSURE | ONCHANGE_SENSORIN , /// bit 0 - horizontal (ONCHANGE_SENSORIN for Bayer)
     P_MULTI_FLIPV,           ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_SENSORIN | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
     P_MULTI_MODE,            ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HBLANK,          ONCHANGE_MULTISENS, /// not needed ? Calculated?)
     P_MULTI_VBLANK,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HEIGHT1,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HEIGHT2,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HEIGHT3,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
/*
     P_MULTI_CWIDTH,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART ,
     P_MULTI_CHEIGHT,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_CLEFT,           ONCHANGE_MULTISENS | ONCHANGE_WINDOW_SAFE ,
     P_MULTI_CTOP,            ONCHANGE_MULTISENS | ONCHANGE_WINDOW | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
*/
     P_MULTI_VBLANK,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_WIDTH1,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_MULTI_WIDTH2,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_MULTI_WIDTH3,          ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_MULTI_HEIGHT1,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HEIGHT2,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_HEIGHT3,         ONCHANGE_MULTISENS |
                                     ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                     ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN ,
     P_MULTI_LEFT1,           ONCHANGE_MULTISENS | ONCHANGE_WINDOW_SAFE,
     P_MULTI_LEFT2,           ONCHANGE_MULTISENS | ONCHANGE_WINDOW_SAFE,
     P_MULTI_LEFT3,           ONCHANGE_MULTISENS | ONCHANGE_WINDOW_SAFE,
     P_MULTI_TOP1,            ONCHANGE_MULTISENS |ONCHANGE_WINDOW | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
     P_MULTI_TOP2,            ONCHANGE_MULTISENS |ONCHANGE_WINDOW | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
     P_MULTI_TOP3,            ONCHANGE_MULTISENS |ONCHANGE_WINDOW | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART,
/// These two below are changed in multisensor
     P_SENSOR_WIDTH,          ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN,
     P_SENSOR_HEIGHT,         ONCHANGE_WINDOW | ONCHANGE_EXPOSURE | ONCHANGE_HIST | ONCHANGE_AEXP | ONCHANGE_FOCUSMODE | ONCHANGE_LIMITFPS | ONCHANGE_HIST | \
                                           ONCHANGE_MEMSENSOR | ONCHANGE_MEMCOMPRESSOR | ONCHANGE_COMPMODE | ONCHANGE_COMPSTOP | ONCHANGE_COMPRESTART | ONCHANGE_SENSORIN 
  };
#endif
