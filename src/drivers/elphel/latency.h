/*!********************************************************************************
*! FILE NAME  : latency.h
*! Table that specifies what frame should "onchange" function be programmed to, relative to the frame for which parameters are specified
*! 1 - same frame             (programmed at least 1 frame earlier), will happen immediately after VACT start for that frame (usually before the interrupt for that frame)
*! 2 - previous frame         (programmed at least 2 frames earlier)
*! 3 - pre-previous frame     (programmed at least 3 frames earlier)
*! 4 - pre-pre-previous frame (programmed at least 4 frames earlier)
*! 0 - ASAP (programmed after that frame interrupt - when compression of the previous frame is finished)
*!  each group consists of 6 numbers:
*!  "onchange" index (0..31), then 5 values for:
*!  1 - continuous run , safe mode (skipping frame when geometry is modified)
*!  2 - continuous run , no skipping frame when geometry is modified, trying to save all frames
*!  3 - async mode (external trigger/timer) , safe mode (skipping frame when geometry is modified)
*!  4 - async mode (external trigger/timer) , no skipping frame when geometry is modified, trying to save all frames
*!  5 - async, non-overlap (program - acquire/compress - program next - acquire/compress next (i.e.10347 with CCD)
*! Used in pgm_functions.c
*!
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
*!  $Log: latency.h,v $
*!  Revision 1.4  2010/06/05 07:13:57  elphel
*!  modified trigseq latencies so to to prevent sequencer stop when changing simultaneously trigger mode and starting generator
*!
*!  Revision 1.3  2010/05/21 06:12:16  elphel
*!  continue working on multi-sensor software
*!
*!  Revision 1.2  2010/05/13 03:39:31  elphel
*!  8.0.8.12 -  drivers modified for multi-sensor operation
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:01  elphel
*!
*!
*!  Revision 1.12  2008/11/27 09:27:31  elphel
*!  Support fro new parameters (vignetting correction related)
*!
*!  Revision 1.11  2008/10/25 19:53:49  elphel
*!  increased latency for gamma/gamma_load to fix a bug
*!
*!  Revision 1.10  2008/10/23 08:06:40  elphel
*!  set latencies for async modes
*!
*!  Revision 1.9  2008/10/22 05:29:03  elphel
*!  Rev. 8.0.alpha5 - working on external trigger mode - moved programming away from the sequencer that can only pass 24 data bits
*!
*!  Revision 1.8  2008/10/22 03:44:50  elphel
*!  increased limitfps latency to the same as the one for window to prevent sensor from missing counter rollover
*!
*!  Revision 1.7  2008/10/21 04:22:26  elphel
*!  changed latency for exposure to 2 (was 3)
*!
*!  Revision 1.6  2008/10/18 06:14:21  elphel
*!  8.0.alpha4 - removed some obsolete parameters, renumbered others, split P_FLIP into P_FLIPH and P_FLIPV (different latencies because of bad frames), pgm_window-> pgm_window, pgm_window_safe
*!
*!  Revision 1.5  2008/10/17 05:44:48  elphel
*!  fixing latencies
*!
*!  Revision 1.4  2008/10/08 21:26:25  elphel
*!  snapsot 7.2.0.pre4 - first images (actually - second)
*!
*!  Revision 1.3  2008/09/25 00:58:11  elphel
*!  snapshot
*!
*!  Revision 1.2  2008/08/11 19:17:01  elphel
*!  reduced syntax complaints by KDevelop
*!
*!  Revision 1.1  2008/07/29 01:15:06  elphel
*!  another snapshot
*!
*!
*/

#ifndef _LATENCY_H
#define _LATENCY_H
///TODO: not used  yet, but it is possible to specify a function with non-zero latency that does not work with the sequencer - only in ASAP mode
const unsigned long ahead_tab[]= 
  {                   /// ASAP C,S   C,NS, A,S   A,NS  NOL
#if 0
  onchange_recalcseq,     0,    0,    0,    0,    0,   0, /// recalculate sequences/latencies, according to P_SKIP, P_TRIG
  onchange_detectsensor,  1,    0,    0,    0,    0,   0, /// detect sensor type, sets sensor structure (capabilities), function pointers
  onchange_sensorphase,   1,    0,    0,    0,    0,   0, /// program sensor clock/phase (do immediately)
  onchange_i2c,           0,    0,    0,    0,    0,   0, /// program i2c
  onchange_initsensor,    1,    0,    0,    0,    0,   0,  /// resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
  onchange_afterinit,     0,    0,    0,    0,    0,   0, /// restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
  onchange_window,        0,    3,    2,    2,    2,   0, /// program sensor WOI and mirroring (flipping)
  onchange_exposure,      0,    3,    3,    2,    2,   0, /// program exposure
  onchange_gains,         0,    2,    2,    2,    2,   0, /// program analog gains
  onchange_triggermode,   0,    3,    2,    2,    2,   0, /// program sensor trigger mode TODO: does it have any sense here?
  onchange_sensorin,      0,    1,    1,    1,    1,   0, /// program sensor input in FPGA (Bayer, 8/16 bits, ??), stop sensor (if needed)
  onchange_sensorstop,    0,    1,    1,    1,    1,   0, /// Stop acquisition from the sensor to the FPGA (start has latency of 2)
  onchange_sensorrun,     0,    2,    2,    2,    2,   0, /// Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
  onchange_gamma,         0,    1,    1,    1,    1,   0, /// program gamma table - make sure table is calculated, maybe send all but last word)
  onchange_hist,          0,    1,    1,    1,    1,   0, /// program histogram window TODO: fix FPGA - (now pos_top will be read too early - will use previous ) and add latency
  onchange_aexp,          0,    1,    1,    1,    1,   0, /// program autoexposure mode TODO: look what exactly is changed here
  onchange_quality,       0,    1,    1,    1,    1,   0, /// program quantization table(s)
  onchange_memsensor,     0,    1,    1,    1,    1,   0, /// program memory channels 0 (sensor->memory) and 1 (memory->FPN)
  onchange_memcompressor, 0,    1,    1,    1,    1,   0, /// program memory channel 2 (memory->compressor) (delays programming until onchange_comprestart if needed)
  onchange_limitfps,      0,    2,    2,    2,    2,   0, /// check compressor will keep up, limit sensor FPS if needed
  onchange_compmode,      0,    1,    1,    1,    1,   0, /// program compressor modes
  onchange_focusmode,     1,    0,    0,    0,    0,   0, /// program focus modes (through writing the tables, so no sequencer)
  onchange_trigseq,       0,    0,    0,    0,    0,   0, /// program sequencer (int/ext)
  onchange_irq,           0,    0,    0,    0,    0,   0, /// program smart IRQ mode
  onchange_comprestart,   0,    1,    0,    1,    0,   0, /// restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
/// onchange_compstop should have the same latency as onchange_window
//  onchange_compstop,      0,    2,    1,    2,    1,   0, /// stop compressor when changing geometry
  onchange_compstop,      0,    3,    2,    3,    2,   0, /// stop compressor when changing geometry
  onchange_compctl,       0,    1,    1,    1,    1,   0, /// only start/stop/single (after explicitly changed, not when geometry was changed)
//  onchange_gammaload,     1,    0,    0,    0,    0,   0, /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
  onchange_gammaload,     1,    1,    1,    1,    1,   0, /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
  onchange_sensorregs,    0,    2,    2,    2,    2,   0  /// write sensor registers (only changed from outside the driver as they may have different latencies)?
// 29 - 3 left
#endif
/// (regular - "C,S" column) latencies decreased by 1
  onchange_recalcseq,     0,    0,    0,    0,    0,   0, /// recalculate sequences/latencies, according to P_SKIP, P_TRIG
  onchange_detectsensor,  1,    0,    0,    0,    0,   0, /// detect sensor type, sets sensor structure (capabilities), function pointers
  onchange_sensorphase,   1,    0,    0,    0,    0,   0, /// program sensor clock/phase (do immediately)
  onchange_i2c,           0,    0,    0,    0,    0,   0, /// program i2c
  onchange_initsensor,    1,    0,    0,    0,    0,   0,  /// resets sensor, reads sensor registers, schedules "secret" manufacturer's corrections to the registers (stops/re-enables hardware i2c)
  onchange_afterinit,     0,    0,    0,    0,    0,   0, /// restore image size, decimation,... after sensor reset or set them according to sensor capabilities if none were specified
  onchange_multisens,     0,    2,    1,    1,    1,   0, /// chnages related to multiplexed sensors
  onchange_window,        0,    2,    1,    1,    1,   0, /// program sensor WOI and mirroring (flipping) - NOTE: 1 bad frame to skip
  onchange_window_safe,   0,    1,    1,    1,    1,   0, /// program sensor WOI and mirroring (flipping) - NOTE: no bad frames
//  onchange_exposure,      0,    3,    3,    2,    2,   0, /// program exposure
  onchange_exposure,      0,    2,    1,    1,    1,   0, /// program exposure
  onchange_gains,         0,    1,    1,    1,    1,   0, /// program analog gains
  onchange_triggermode,   0,    2,    1,    1,    1,   0, /// program sensor trigger mode TODO: does it have any sense here?
  onchange_sensorin,      0,    0,    0,    0,    0,   0, /// program sensor input in FPGA (Bayer, 8/16 bits, ??), stop sensor (if needed)
  onchange_sensorstop,    0,    0,    0,    0,    0,   0, /// Stop acquisition from the sensor to the FPGA (start has latency of 2)
  onchange_sensorrun,     0,    1,    1,    1,    1,   0, /// Start/single acquisition from the sensor to the FPGA (stop has latency of 1)
  onchange_gamma,         0,    1,    1,    1,    1,   0, /// program gamma table - make sure table is calculated, maybe send all but last word)
  onchange_hist,          0,    0,    0,    0,    0,   0, /// program histogram window TODO: fix FPGA - (now pos_top will be read too early - will use previous ) and add latency
  onchange_aexp,          0,    0,    0,    0,    0,   0, /// program autoexposure mode TODO: look what exactly is changed here
  onchange_quality,       0,    0,    0,    0,    0,   0, /// program quantization table(s)
  onchange_memsensor,     0,    0,    0,    0,    0,   0, /// program memory channels 0 (sensor->memory) and 1 (memory->FPN)
  onchange_memcompressor, 0,    0,    0,    0,    0,   0, /// program memory channel 2 (memory->compressor) (delays programming until onchange_comprestart if needed)
//  onchange_limitfps,      0,    1,    2,    2,    2,   0, /// check compressor will keep up, limit sensor FPS if needed
/// For Micron sensors limitfps should have the same latency as changing window height, otherwise when WOI_HEIGHT 0x3c0->0x790 and next frame VBLANK 0x13e->0x284
/// sensor waits till the counter overflows (>10 seconds) without any frame sync pulses
  onchange_limitfps,      0,    2,    1,    1,    1,   0, /// check compressor will keep up, limit sensor FPS if needed
  onchange_compmode,      0,    0,    1,    1,    1,   0, /// program compressor modes
  onchange_focusmode,     1,    0,    0,    0,    0,   0, /// program focus modes (through writing the tables, so no sequencer)
//  onchange_trigseq,       0,    0,    0,    0,    0,   0, /// program sequencer (int/ext)
//  onchange_trigseq,       1,    0,    0,    0,    0,   0, /// program sequencer (int/ext) NOTE:needs >24 bit data, too much for sequencer
  onchange_trigseq,       1,    2,    1,    1,    1,   0, /// program sequencer (int/ext) NOTE:needs >24 bit data, too much for sequencer. Should be not later than onchange_triggermode and limitfps

  onchange_irq,           0,    0,    0,    0,    0,   0, /// program smart IRQ mode
  onchange_comprestart,   0,    0,    0,    0,    0,   0, /// restart after changing geometry  (recognizes ASAP and programs memory channel 2 then)
/// onchange_compstop should have the same latency as onchange_window
//  onchange_compstop,      0,    2,    1,    2,    1,   0, /// stop compressor when changing geometry
  onchange_compstop,      0,    2,    1,    1,    1,   0, /// stop compressor when changing geometry
  onchange_compctl,       0,    0,    1,    1,    1,   0, /// only start/stop/single (after explicitly changed, not when geometry was changed)
//  onchange_gammaload,     1,    0,    0,    0,    0,   0, /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
  onchange_gammaload,     1,    1,    1,    1,    1,   0, /// write gamma tables (should be prepared). Maybe - just last byte, to activate?
  onchange_sensorregs,    0,    1,    1,    1,    1,   0, /// write sensor registers (only changed from outside the driver as they may have different latencies)?
  onchange_prescal,       0,    0,    0,    0,    0,   0 /// change scales for per-color digital gains, apply vignetting correction
  };
#endif
