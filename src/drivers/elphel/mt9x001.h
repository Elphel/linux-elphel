/***************************************************************************//**
* @file      mt9x001.h
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

#ifndef _MT9X001_H
#define _MT9X001_H

#define MT9M001_PARTID     0x8411 ///< MT9M00* PartID register value
#define MT9D001_PARTID     0x8511 ///< MT9D00* PartID register value
#define MT9T001_PARTID     0x1601 ///< MT9T00* PartID register value
#define MT9P001_PARTID     0x1801 ///< MT9P00* PartID register value
#define MT9X001_PARTIDMASK 0xff00 ///< Part ID mask
#define MT9X001_I2C_ADDR     0x5d ///< MT9M, MT9D, MT9T I2C slave address (7 bit)
#define MT9P001_I2C_ADDR     0x48 ///< MT9P I2C slave address (7 bit)
#define MT9M_TYP 1 ///< MT9M00* type for the driver
#define MT9D_TYP 2 ///< MT9D00* type for the driver
#define MT9T_TYP 3 ///< MT9T00* type for the driver
#define MT9P_TYP 4 ///< MT9P00* type for the driver

/* i2c Micron MI-1300 registers will be defined here */
#define P_MT9X001_CHIPVER   0x00  ///< Chip version, dflt= 0x8411 - will change??? /0x1801
#define P_MT9X001_ROWSTART  0x01  ///< First row to read out, dflt=0x0c/0x0c/0x14/0x36 [0..2004],even
#define P_MT9X001_COLSTART  0x02  ///< First column to read out, must be even! Dflt=0x14/18/20/0x10 [0..2750],even
#define P_MT9X001_HEIGHT    0x03  ///< Number of rows-1 (min value=2), dflt=0x03ff/4af/5ff/0x797 1..2005], odd
#define P_MT9X001_WIDTH     0x04  ///< Number of columns-1 (odd, >=3),dflt =0x4ff/63f/7ff/a1f [1..2751] odd
#define P_MT9X001_HORBLANK  0x05  ///< Horizontal blanking, dflt=0x09/35/8e/0 pixels [0..4095]
#define P_MT9X001_VERTBLANK 0x06  ///< Vertical blanking,  dflt=0x19/19/19/19 rows [8..2047]
///< NOTE: There seems to be a bug in P_MT9X001_VERTBLANK register in MT9P031 sensor
///< when increasing the value. I.e. changing it from default in the camera after startup 0x284  to 0x285
///< the sensor "sleeps" for ~35 seconds with to Frame Sync output . That may depend on something else, of course,
#define P_MT9X001_OUTCTRL   0x07  ///< Output format bits (dflt=2/2/2/1f82) :<ul>
                                  ///<<li>   bit 0 - 0 - normal, 1 - do not update integration, gains, blanking, flip, decimation''
                                  ///<<li>   bit 1 - 0 - stop sensor, 1 - normal (0->1 restarts from start of the frame)
                                  ///<<li>   bits 2,3 should be 0 (in mt9p bit 2 selects fifo output data)
                                  ///<<li>   bit 6 - 0 - normal, 1 - test (mt9p - reserved)</ul>
                                  ///< mt9p001:<ul>
                                  ///<<li>   9:7  pixclk slew rate (0..7, higher - faster) dflt - 7
                                  ///<<li>   12:10 output (but pixclk) slew rate (0..7, higher - faster) dflt -7 </ul>
#define P_MT9X001_SHTRWDTHU 0x08  ///< Shutter width upper - number of rows to integrate (dflt=0x?/?/0/0)
#define P_MT9X001_SHTRWDTH  0x09  ///< Shutter width - number of rows to integrate (dflt=0x419/4c9/619/797)
#define P_MT9X001_PXLCTL    0x0a  ///< MT9P: Pixel clock control (dflt=0)
                                  ///< - bits 6:0 - divide pixel clock {0,1,2,4,8,16,32,64}
                                  ///< - bits 10:8 - shift pixel clock [-2,2]
                                  ///< - bit 15 - invert pixel clock
#define P_MT9X001_RESTART   0x0b  ///< Sensor restart (autozeroed) writing 1 restarts frame /0
                                  ///< - bit 0 - 1 - restart (autoclearing bit)
                                  ///< - bit 1 - pause restart
                                  ///< - bit 2 - trigger (like trigger input)
#define P_MT9X001_SHTRDLY   0x0c  ///< Shutter delay - number of pixels before row reset (dflt=0/0/0) - not mt9p
#define P_MT9X001_RESET     0x0d  ///< 0 - normal, 1 - reset /0
#define P_MT9X001_PLL1      0x10  ///< MT9P: PLL CTL 1  (dflt=0x50)
                                  ///< - bit 0 - power PLL
                                  ///< - bit 1 - use PLL
                                  ///< - other bits - set to dflt (0x50)
#define P_MT9X001_PLL2      0x11  ///< MT9P: PLL CTL 2  (dflt=0x6404)
                                  ///< - 5:0 - PLL n divider [0,63] - dflt 4
                                  ///< - 15:8 PLL m factor [16,255] - dflt 0x64

#define P_MT9X001_PLL3      0x12  ///< MT9P: PLL CTL 3  (dflt=0x0)
                                  ///< 4:0 PLL p1 divider [0,127]

#define P_MT9X001_RMODE1    0x1e  ///< Read options 1 (+/- indicate per-device support), (dflt=0x8000/8040/8040/4006):<ul>
///< <li> ---+bits 0,1 - reserved, sould be 0 | strobe end
///< <li> ++--bit  2 - column skip 4 (1- skip, 0 - no) reg 0x20 bit 3 should be 0 to enable this bit
///< <li> ++--bit  3 - row skip 4 (1- skip, 0 - no) reg 0x20 bit 4 should be 0 to enable this bit
///< <li> ---+bits 3:2 - Strobe start
///< <li> ++-+bit  4 - column skip 8 (1- skip, 0 - no) bit 2 and reg 0x20 bit 3 should be 0 to enable this bit
///< <li> ---+bit  4 - Strobe enable
///< <li> ++--bit  5 - row skip 8 (1- skip, 0 - no) bit 3 and reg 0x20 bit 4 should be 0 to enable this bit
///< <li> ---+bit  5 - invert strobe
///< <li> -+--bit  6 - Noise suppression (1 - enabled, default=1)
///< <li> ---+bit  6 - bulb exposure
///< <li> ---+bit  7 - Global shutter reset (0 - ers)
///< <li> ++++bit  8 - snapshot mode (0 - continuous, 1 - wait trigger)
///< <li> +++-bit  9 - strobe enable (1 - enable, 0 - disable)
///< <li> ---+bit  9 - inver trigger
///< <li> +++-bit 10 - strobe width (0 - minimal, 1 - extended)
///< <li> ---+bit 10 - continuous line valid (during vert blank)
///< <li> +++-bit 11 - strobe override (strobe enable should be 0) - set strobe active if 1, 0 - normal
///< <li> ---+bit 11 - XOR line valid
///< <li> ---bits 12,13,14 - reserved, should be 0
///< <li> +++bit 15 - reserved, should be 1</ul>

#define P_MT9X001_RMODE2    0x20  ///< Read options 2 (+/- indicate per-device support), (dflt=0x1104/1104/0/40):<ul>
///< <li> +++-bit  0 - allow "bad frames". 0 (default) - output only good frames, 1 - allow bad also
///< <li> ----bits 1 - reserved, sould be 0
///< <li> ++--bit  2 - reserved, sould be 1/1/0
///< <li> ++--bit  3 - column skip (1 - read 2, skip 2; 0 - read all) If this bit is 1,  both column skip 4 and column skip 8 (0x1e) are ignored
///< <li> ++--bit  4 - row skip (1 - read 2, skip 2; 0 - read all)  Similar to the above
///< <li> ---+bit  5 -column sum in binning (0 - average)
///< <li> ++--bit  6 - reserved, should be 0
///< <li> ---+bit  6 - row BLC (dflt=`1) (use per-row black level, 0 - global)
///< <li> +??-bit  7 - flip odd/even rows (0 - normal)
///< <li> ++--bit  8 - reserved, should be 1
///< <li> +++-bit  9 - enable "line valid" during vertical blanking, 0 - normal (no lane valid during blanking)
///< <li> +++-bit 10 - XOR "line valid" with vertical blanking, 0 just mask "l.v." with bit 9
///< <li> +---bit 11 - reserved, sould be 0
///< <li> ---+bit 11 - show dark rows
///< <li> +---bit 12 - reserved, sould be 1
///< <li> ---+bit 12 - show dark columns
///< <li> +---bits 13 - reserved, sould be 0
///< <li> --++bit 14 - flip horizontal (0 - normal) *UNDOCUMENTED* documented in MT9P001 !!
///< <li> --++bit 15 - flip vertical (0 - normal)</ul>
#define P_MT9X001_RMODE3    0x21  ///< Read options 3  (+/- indicate per-device support) (MT9T only), (dflt=0x0):<ul>
///< <li> --+-bit 0 - Global Reset. If set, uses global reset in snapshot mode (dflt=0x0)
///< <li> --+-bit 1 - Use GSHT_CTL (if set uses GSHT_CTL pad signal only, if 0 - together with TRIGGER</ul>
#define P_MT9X001_RAM       0x22  ///< Row address mode (+/- indicate per-device support) (MT9T,P only), (dflt=0x0):<ul>
///< <li> --++bits 2:0 Row skip - number of rows to skip (0 - each row). Row period will be this value+1 (even in binning mode)
///< <li> --++bits 5:4 Row Bin - number of rows to bin to the first one. For full binning &lt;Row skip&gt;==&lt;row bin&gt;</ul>
#define P_MT9X001_CAM       0x23  ///< Column address mode (+/- indicate per-device support) (MT9T,P only), (dflt=0x0):<ul>
///< <li> --++bits 2:0 Column skip - number of column-pairs to skip (0 - each column-pair). Column-pair period will be this value+1 (even in binning mode)
///< <li> --++bits 5:4 Column Bin - number of columns to bin to the first one. Not clear is binning also in pairs? needs testing</ul>
///<  Column start address should be multiple of &lt;column bin&gt;+1
#define P_MT9X001_GREEN1    0x2b      ///< Green Gain 1, dflt= 0x08 (1x), for MT9T bits 14:8 - "digital gain"
#define P_MT9X001_BLUE      0x2c      ///< Green Gain 1, dflt= 0x08 (1x), for MT9T bits 14:8 - "digital gain"
#define P_MT9X001_RED       0x2d      ///< Green Gain 1, dflt= 0x08 (1x), for MT9T bits 14:8 - "digital gain"
#define P_MT9X001_GREEN2    0x2e      ///< Green Gain 1, dflt= 0x08 (1x), for MT9T bits 14:8 - "digital gain"
#define P_MT9X001_ALLGAINS  0x35      ///< write to all 4 gains (0x2b,0x2c,0x2d,0x2e), read from red (0x2b)
#define P_MT9X001_DESIRBLACK  0x49    ///< bits 11:2 - desired  black level (MT9T,P only dflt=0xa8)
#define P_MT9X001_ROWRBLACKOFFS  0x4b ///< bits 11:0 - desired  black level (MT9P only - dflt=0x28)
#define P_MT9X001_COARSETHRSH 0x5d    ///< Black level calibration coarse thersholds (+/- indicate per-device support) (MT9T only), dflt=0x2d13 <ul>
///< <li> --+bits  6:0 low coarse thershold (should be less than low thershold - see 0x5f) ,dflt=0x13
///< <li> --+bits 14:8 high coarse thershold (should be noless than high thershold - see 0x5f) ,dflt=0x2d</ul>
#define P_MT9X001_CALTHRESH 0x5f  ///< Black level calibration control fields (+/- indicate per-device support) (dflt=0x904/a39f/231d):<ul>
///< <li> +--bits 5:0 - Low threshold for black in ADC LSBs (default - 4)
///< <li> -+-bits 5:0 - Low threshold for black in ADC LSBs (default - 29)
///< <li> --+bits 6:0 - Low threshold for black in ADC LSBs (default - 0x13)
///< <li> ++-bit  7 - Override automatic bits 5:0 and 14:8, 0 - automatic. dflt= 0/1/x
///< <li> +--bits 14:8 - Maximal allowed black level in ADC LSBs (default - low theresh+5=0x09)
///< <li> -+-bits 14:8 - Maximal allowed black level in ADC LSBs (default - low theresh+5=0x23)
///< <li> --+bits 14:8 - Maximal allowed black level in ADC LSBs (default - low theresh+5=0x23)
///< <li> ++-bit 15 - no gain dependence, 0 - both thresholds set automatically, dflt=0/1/x</ul>

#define P_MT9X001_CALGREEN1 0x60  ///< analog offset for GREEN1  For MT9M, MT9D: bits 7:0 - magnitude, bit 8 - sign, MT9T - two's complement
#define P_MT9X001_CALGREEN2 0x61  ///< analog offset for GREEN2. For MT9M, MT9D: bits 7:0 - magnitude, bit 8 - sign, MT9T - two's complement
#define P_MT9X001_CALCTRL   0x62  ///< Black levels calibration control fields (+/- indicate per-device support) (dflt 0x498/8498/0):<ul>
///< <li> +++bit  0   - manual override, correct with programmed values. 0 (default) - automatically adjust offset values
///< <li> ++-bits 2,1 - force/disable black level calibration. 00 - apply calibration during ADC operation only (default),
///<              10 - apply calibration continuously, X1 - disable black level correction (set calibration voltages to 0)
///< <li> --+bit  2 0 - enable offset calibration (dflt), 1 - disable offset calibration voltage
///< <li> ++-bits 4:3 - reserved, sould be 1
///< <li> ++-bits 6:5 - reserved, sould be 0
///< <li> ++-bit  7   - reserved, sould be 1
///< <li> ++-bits 9:8 - reserved, sould be 0
///< <li> ++-bit 10   - reserved, sould be 1
///< <li> ++-bit 11   - 1 - do not reset the upper threshold after a black level recalculation sweep, 0 - reset after sweep (default)
///< <li> +++bit 12   - (autoreset bit) - start a new running average and perform a fast black level calibration (0 - normal)
///< <li> ++-bits 14:13 - reserved, sould be 0
///< <li> --+bit 13   - if set, lock red and blue channels calibration (red and blue gains should be equal)
///< <li> --+bit 14   - if set, lock green1 and green2 channels calibration (red and blue gains should be equal)
///< <li> ++-bit 15   -  1 - do not perform fast sweep after gains change, 0 - normal operation</ul>
#define P_MT9X001_CALRED   0x63  ///< analog offset for RED.    For MT9M, MT9D: bits 7:0 - magnitude, bit 8 - sign MT9T - two's complement
#define P_MT9X001_CALBLUE  0x64  ///< analog offset for BLUE.   For MT9M, MT9D: bits 7:0 - magnitude, bit 8 - sign MT9T - two's complement
#define P_MT9X001_7F       0x7f  ///< Should be written 0 to prevent blue "bluming" columns
#define P_MT9X001_TEST     0xa0  ///< test patterns. Probably only in MT9P001?<ul>
///< <li> bits 6:3: <ul>
///< <li> 0: color field
///< <li> 1: horizontal gradient
///< <li> 2: vertical gradient
///< <li> 3: diagonal
///< <li> 4: classic
///< <li> 5: marching 1's
///< <li> 6: monochrome horizontal bars
///< <li> 7: monochrome vertical bars
///< <li> 8: vertical color bars</ul>
///<  Legal values: [0, 15].</li>
///< <li> bit    2     Reserved</li>
///< <li> bit    1     Reserved
///< <li> bit    0     Enable_Test_Pattern.   Enables the test pattern. When set, data from the ADC will be replaced with a digitally
///<                   generated test pattern specified by Test_Pattern_Mode.</li></ul>

#define P_MT9X001_CHIPEN   0xF1  ///< Chip enable and i2c sync (mirrors bits in reg 0x07 (+/- indicate per-device support) (default=0x01):<ul>
///< <li> ++-bit  0   -  1 - normal operation, 0 - stop readout (same as reg 0x07, bit 1)
///< <li> ++-bit  1   -  0 - normal, appropriate changes are made at frame boudary. 1 - do not update (same as reg 7 bit 0) </ul>
#define P_MT9X001_CHIPEN1  0xF8  ///< Chip enable and i2c sync (mirrors bits in feg 0x07 (+/- indicate per-device support)(default=0x01):<ul>
///< <li> --+bit  0   -  1 - normal operation, 0 - stop readout (same as reg 0x07, bit 1)
///< <li> --+bit  1   -  0 - normal, appropriate changes are made at frame boudary. 1 - do not update (same as reg 7 bit 0)</ul>
/** Detect one of Micron/Aptina/On Semiconductor sensors MT9M*, MT9D*,MT9T*, andMT9P* with parallel interface */

/**
LUT to map SENSOR_REGSxxx to internal sensor register addresses
  * needed for any sensor
  * For better manual mapping:
      - even elements are SENSOR_REGSxxx,
      - odd elements are sensor's register addresses.
  * has to be at least 16-bit/entry for 16 bit addresses
  * (for MT9X001 it's a 1-to-1 mapping)
*/
extern const unsigned short mt9x001_par2addr[];
extern const unsigned short mt9x001_pages[];
extern const unsigned short mt9x001_ahead_tab[];

int mt9x001_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
;
void mt9x001_set_device(struct device *dev);
#if 0
int adjustBinning_mt9x001(void);
int program_woi_mt9x001(int nonstop);
int program_gains_mt9x001(void);
int program_exposure_mt9x001(void);
#endif

#endif
