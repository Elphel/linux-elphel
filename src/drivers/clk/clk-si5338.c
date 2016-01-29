/*!***************************************************************************
 *! FILE NAME  : si5338.c
 *! DESCRIPTION: control of the Silicon Laboratories SI5338 clock generator
 *! Copyright (C) 2013 Elphel, Inc.
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
 */

#define DEBUG /* should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)*/
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/math64.h>

#undef GENERATE_EXTRA

#define DRV_VERSION "1.0"
#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

#define REG5338_PAGE               255
#define REG5338_PAGE_MASK            1
#define REG5338_DEV_CONFIG2          2
#define REG5338_DEV_CONFIG2_MASK  0x3f
#define REG5338_DEV_CONFIG2_VAL    38 /* last 2 digits of part number */
#define LAST_REG                   347

#define FVCOMIN                    2200000000LL
#define FVCOMAX                    2840000000LL
#define INFREQMIN                     5000000LL
#define INFREQMAX                   710000000LL
#define INFREQDIV                    40000000LL /* divide input frequency if above */

#define SPREAD_RATE_MIN                 31500   /* 31.5 KHz */
#define SPREAD_RATE_MAX                 63000   /* 63 KHz */
#define SPREAD_AMP_MIN                     10   /* 0.1% */
#define SPREAD_AMP_MAX                    500   /* 5.0% */
#define SPREAD_AMP_DENOM                10000   /* 0.01% amplitude step */

#define SPREAD_RATE_DFLT                31500   /* 31.5 KHz */
#define SPREAD_AMP_DFLT                    50   /* 0.5% */


#define MSINT_MIN                  8 /* not considering 4,6 */
#define MSINT_MAX                  567

#define INIT_TIMEOUT               1000 /* reads of the I2C status register (1 cycle ~ 0.1 ms) */

#define AWE_IN_MUX                 0x1d18
#define AWE_IN_MUX1                0x1c1c
#define AWE_FB_MUX                 0x1e18
#define AWE_FB_MUX1                0x1c20

#define AWE_XTAL_FREQ              0x1c03
#define AWE_PFD_REF                0x1de0
#define AWE_PFD_FB                 0x1ee0
#define AWE_P1DIV                  0x1d07
#define AWE_P2DIV                  0x1e07
#define AWE_DRV0_PDN               0x1f01
#define AWE_MS0_PDN                0x1f02
#define AWE_R0DIV                  0x1f1c
#define AWE_R0DIV_IN               0x1fe0
#define AWE_DRV1_PDN               0x2001
#define AWE_MS1_PDN                0x2002
#define AWE_R1DIV                  0x201c
#define AWE_R1DIV_IN               0x20e0
#define AWE_DRV2_PDN               0x2101
#define AWE_MS2_PDN                0x2102
#define AWE_R2DIV                  0x211c
#define AWE_R2DIV_IN               0x21e0
#define AWE_DRV3_PDN               0x2201
#define AWE_MS3_PDN                0x2202
#define AWE_R3DIV                  0x221c
#define AWE_R3DIV_IN               0x22e0

#define AWE_DRV0_VDDO              0x2303
#define AWE_DRV1_VDDO              0x230c
#define AWE_DRV2_VDDO              0x2330
#define AWE_DRV3_VDDO              0x23c0
#define AWE_DRV0_FMT               0x2407
#define AWE_DRV0_INV               0x2418
#define AWE_DRV1_FMT               0x2507
#define AWE_DRV1_INV               0x2518
#define AWE_DRV2_FMT               0x2607
#define AWE_DRV2_INV               0x2618
#define AWE_DRV3_FMT               0x2707
#define AWE_DRV3_INV               0x2718

#define AWE_DRV0_TRIM              0x281f
#define AWE_DRV1_TRIM_A            0x28e0
#define AWE_DRV1_TRIM_B            0x2903
#define AWE_DRV2_TRIM              0x297c
#define AWE_DRV3_TRIM              0x2a1f

#define AWE_FCAL_OVRD_07_00        0x2dff
#define AWE_FCAL_OVRD_15_08        0x2eff
#define AWE_FCAL_OVRD_17_15        0x2f03
#define AWE_REG47_72               0x2ffc
#define AWE_PFD_EXTFB              0x3080
#define AWE_PLL_KPHI               0x307f
#define AWE_FCAL_OVRD_EN           0x3180
#define AWE_VCO_GAIN               0x3170
#define AWE_RSEL                   0x310c
#define AWE_BWSEL                  0x3103
#define AWE_VCO_GAIN_RSEL_BWSEL    0x317f

#define AWE_PLL_EN                 0x32c0
#define AWE_MSCAL                  0x323f
#define AWE_MS3_HS                 0x3380
#define AWE_MS2_HS                 0x3340
#define AWE_MS1_HS                 0x3320
#define AWE_MS0_HS                 0x3310
#define AWE_MS_PEC                 0x3307

#define AWE_MS0_FIDCT              0x3460
#define AWE_MS0_FIDDIS             0x3410
#define AWE_MS0_SSMODE             0x340C
#define AWE_MS0_PHIDCT             0x3403
#define AWE_MS0_P1_07_00           0x35ff
#define AWE_MS0_P1_15_08           0x36ff
#define AWE_MS0_P1_17_16           0x3703
#define AWE_MS0_P2_05_00           0x37fc
#define AWE_MS0_P2_13_06           0x38ff
#define AWE_MS0_P2_21_14           0x39ff
#define AWE_MS0_P2_29_22           0x3aff
#define AWE_MS0_P3_07_00           0x3bff
#define AWE_MS0_P3_15_08           0x3cff
#define AWE_MS0_P3_23_16           0x3dff
#define AWE_MS0_P3_29_24           0x3e3f


#define AWE_MS1_FIDCT              0x3f60
#define AWE_MS1_FIDDIS             0x3f10
#define AWE_MS1_SSMODE             0x3f0C
#define AWE_MS1_PHIDCT             0x3f03
#define AWE_MS1_P1_07_00           0x40ff
#define AWE_MS1_P1_15_08           0x41ff
#define AWE_MS1_P1_17_16           0x4203
#define AWE_MS1_P2_05_00           0x42fc
#define AWE_MS1_P2_13_06           0x43ff
#define AWE_MS1_P2_21_14           0x44ff
#define AWE_MS1_P2_29_22           0x45ff
#define AWE_MS1_P3_07_00           0x46ff
#define AWE_MS1_P3_15_08           0x47ff
#define AWE_MS1_P3_23_16           0x48ff
#define AWE_MS1_P3_29_24           0x493f

#define AWE_MS2_FRCTL              0x4a60 /* different name? */
#define AWE_MS2_FIDDIS             0x4a10
#define AWE_MS2_SSMODE             0x4a0C
#define AWE_MS2_PHIDCT             0x4a03
#define AWE_MS2_P1_07_00           0x4bff
#define AWE_MS2_P1_15_08           0x4cff
#define AWE_MS2_P1_17_16           0x4d03
#define AWE_MS2_P2_05_00           0x4dfc
#define AWE_MS2_P2_13_06           0x4eff
#define AWE_MS2_P2_21_14           0x4fff
#define AWE_MS2_P2_29_22           0x50ff
#define AWE_MS2_P3_07_00           0x51ff
#define AWE_MS2_P3_15_08           0x52ff
#define AWE_MS2_P3_23_16           0x53ff
#define AWE_MS2_P3_29_24           0x543f

#define AWE_MS3_FIDCT              0x5560
#define AWE_MS3_FIDDIS             0x5510
#define AWE_MS3_SSMODE             0x550C
#define AWE_MS3_PHIDCT             0x5503
#define AWE_MS3_P1_07_00           0x56ff
#define AWE_MS3_P1_15_08           0x57ff
#define AWE_MS3_P1_17_16           0x5803
#define AWE_MS3_P2_05_00           0x58fc
#define AWE_MS3_P2_13_06           0x59ff
#define AWE_MS3_P2_21_14           0x5aff
#define AWE_MS3_P2_29_22           0x5bff
#define AWE_MS3_P3_07_00           0x5cff
#define AWE_MS3_P3_15_08           0x5dff
#define AWE_MS3_P3_23_16           0x5eff
#define AWE_MS3_P3_29_24           0x5f3f

#define AWE_MSN_P1_07_00           0x61ff
#define AWE_MSN_P1_15_08           0x62ff
#define AWE_MSN_P1_17_16           0x6303
#define AWE_MSN_P2_05_00           0x63fc
#define AWE_MSN_P2_13_06           0x64ff
#define AWE_MSN_P2_21_14           0x65ff
#define AWE_MSN_P2_29_22           0x66ff
#define AWE_MSN_P3_07_00           0x67ff
#define AWE_MSN_P3_15_08           0x68ff
#define AWE_MSN_P3_23_16           0x69ff
#define AWE_MSN_P3_29_24           0x6a3f

#define AWE_OUT0_DIS_STATE         0x6ec0
#define AWE_OUT1_DIS_STATE         0x72c0
#define AWE_OUT2_DIS_STATE         0x76c0
#define AWE_OUT3_DIS_STATE         0x7ac0

#define AWE_STATUS                 0xdaff
#define AWE_STATUS_PLL_LOL         0xda10
#define AWE_STATUS_PLL_LOS_FDBK    0xda08
#define AWE_STATUS_PLL_LOS_CLKIN   0xda04
#define AWE_STATUS_PLL_SYS_CAL     0xda01

#define AWE_MS_RESET               0xe204

#define AWE_OUT0_DIS               0xe601
#define AWE_OUT1_DIS               0xe602
#define AWE_OUT2_DIS               0xe604
#define AWE_OUT3_DIS               0xe608
#define AWE_OUT_ALL_DIS            0xe610

#define AWE_FCAL_07_00             0xebff
#define AWE_FCAL_15_08             0xecff
#define AWE_FCAL_17_16             0xed03


#define AWE_DIS_LOS                0xf180
#define AWE_REG241                 0xf1ff

#define AWE_SOFT_RESET             0xf602



#define AWE_MS0_SSUPP2_07_00      0x11fff
#define AWE_MS0_SSUPP2_14_08      0x1207f
#define AWE_MS0_SSUPP3_07_00      0x121ff /* set them to 0 - default==1 */
#define AWE_MS0_SSUPP3_14_08      0x1227f
#define AWE_MS0_SSUPP1_07_00      0x123ff
#define AWE_MS0_SSUPP1_11_08      0x1240f
#define AWE_MS0_SSUDP1_03_00      0x124f0
#define AWE_MS0_SSUDP1_11_04      0x125ff
#define AWE_MS0_SSDNP2_07_00      0x126ff
#define AWE_MS0_SSDNP2_14_08      0x1277f
#define AWE_MS0_SSDNP3_07_00      0x128ff
#define AWE_MS0_SSDNP3_14_08      0x1297f
#define AWE_MS0_SSDNP1_07_00      0x12aff
#define AWE_MS0_SSDNP1_11_08      0x12b0f

#define AWE_MS1_SSUPP2_07_00      0x12fff
#define AWE_MS1_SSUPP2_14_08      0x1307f
#define AWE_MS1_SSUPP3_07_00      0x131ff
#define AWE_MS1_SSUPP3_14_08      0x1327f
#define AWE_MS1_SSUPP1_07_00      0x133ff
#define AWE_MS1_SSUPP1_11_08      0x1340f
#define AWE_MS1_SSUDP1_03_00      0x134f0
#define AWE_MS1_SSUDP1_11_04      0x135ff
#define AWE_MS1_SSDNP2_07_00      0x136ff
#define AWE_MS1_SSDNP2_14_08      0x1377f
#define AWE_MS1_SSDNP3_07_00      0x138ff
#define AWE_MS1_SSDNP3_14_08      0x1397f
#define AWE_MS1_SSDNP1_07_00      0x13aff
#define AWE_MS1_SSDNP1_11_08      0x13b0f

#define AWE_MS2_SSUPP2_07_00      0x13fff
#define AWE_MS2_SSUPP2_14_08      0x1407f
#define AWE_MS2_SSUPP3_07_00      0x141ff
#define AWE_MS2_SSUPP3_14_08      0x1427f
#define AWE_MS2_SSUPP1_07_00      0x143ff
#define AWE_MS2_SSUPP1_11_08      0x1440f
#define AWE_MS2_SSUDP1_03_00      0x144f0
#define AWE_MS2_SSUDP1_11_04      0x145ff
#define AWE_MS2_SSDNP2_07_00      0x146ff
#define AWE_MS2_SSDNP2_14_08      0x1477f
#define AWE_MS2_SSDNP3_07_00      0x148ff
#define AWE_MS2_SSDNP3_14_08      0x1497f
#define AWE_MS2_SSDNP1_07_00      0x14aff
#define AWE_MS2_SSDNP1_11_08      0x14b0f

#define AWE_MS3_SSUPP2_07_00      0x14fff
#define AWE_MS3_SSUPP2_14_08      0x1507f
#define AWE_MS3_SSUPP3_07_00      0x151ff
#define AWE_MS3_SSUPP3_14_08      0x1527f
#define AWE_MS3_SSUPP1_07_00      0x153ff
#define AWE_MS3_SSUPP1_11_08      0x1540f
#define AWE_MS3_SSUDP1_03_00      0x154f0
#define AWE_MS3_SSUDP1_11_04      0x155ff
#define AWE_MS3_SSDNP2_07_00      0x156ff
#define AWE_MS3_SSDNP2_14_08      0x1577f
#define AWE_MS3_SSDNP3_07_00      0x158ff
#define AWE_MS3_SSDNP3_14_08      0x1597f
#define AWE_MS3_SSDNP1_07_00      0x15aff
#define AWE_MS3_SSDNP1_11_08      0x15b0f



#define AWE_MISC_47                0x2ffc /* write 0x5 */
#define AWE_MISC_106               0x6a80 /* write 0x1 */
#define AWE_MISC_116               0x7480 /* write 0x1 */
#define AWE_MISC_42                0x2a20 /* write 0x1 */
#define AWE_MISC_06A               0x06e0 /* write 0x0 */
#define AWE_MISC_06B               0x0602 /* write 0x0 */
#define AWE_MISC_28                0x1cc0 /* write 0x0 */

#define CACHE_INIT                      1
#define CACHE_VOLAT                     2

struct si5338_cache_t {
	u8 flags;
	u8 data;
};
struct si5338_data_t {
	u64 input_frequency12;
	u64 input_frequency3;
	u64 input_frequency4;
	u64 input_frequency56;
	u32 ss_on_freq_change; /* 0 - disable SS when frequency is changed, 1 - update SS.  +2 reset MS after starting SS*/ 
	u32 spread_spectrum_rate[4]; /* in Hz */
	u32 spread_spectrum_amp[4]; /* in 0.01% */
//	u64 pll_frequency;
	int reg_addr;   /* used for raw register r/w */
	int last_page;  /* value of last page accessed (bit 0 of register 255) */
	struct mutex lock;
	struct si5338_cache_t cache[LAST_REG+1];
};

struct si5338_drv_t {
	const char * description;
	u8 fmt;
	u8 vdd;
	u8 trim;
	u8 invert; /* bits [1:0} data, [3:2] - don't care ([3]==1 - [1] - any, [2]==1 - [0] - any */
};

static struct i2c_device_id si5338_id[] = {
	{ "si5338", 0 },
	{ }
};

static const struct si5338_drv_t drv_configs []={
		{"3V3_CMOS_A+",  0x1,0x0,0x17,0x8}, /* bX0 */
		{"3V3_CMOS_A-",  0x1,0x0,0x17,0x9}, /* bX1 */
		{"3V3_CMOS_B+",  0x2,0x0,0x17,0x4}, /* b0X */
		{"3V3_CMOS_B-",  0x2,0x0,0x17,0x6}, /* b1X */
		{"3V3_CMOS_A+B+",0x3,0x0,0x17,0x8},
		{"3V3_CMOS_A-B+",0x3,0x0,0x17,0x9},
		{"3V3_CMOS_A+B-",0x3,0x0,0x17,0x4},
		{"3V3_CMOS_A-B-",0x3,0x0,0x17,0x6},

		{"2V5_CMOS_A+",  0x1,0x1,0x13,0x8},
		{"2V5_CMOS_A-",  0x1,0x1,0x13,0x9},
		{"2V5_CMOS_B+",  0x2,0x1,0x13,0x4},
		{"2V5_CMOS_B-",  0x2,0x1,0x13,0x6},
		{"2V5_CMOS_A+B+",0x3,0x1,0x13,0x8},
		{"2V5_CMOS_A-B+",0x3,0x1,0x13,0x9},
		{"2V5_CMOS_A+B-",0x3,0x1,0x13,0x4},
		{"2V5_CMOS_A-B-",0x3,0x1,0x13,0x6},

		{"1V8_CMOS_A+",  0x1,0x2,0x15,0x8},
		{"1V8_CMOS_A-",  0x1,0x2,0x15,0x9},
		{"1V8_CMOS_B+",  0x2,0x2,0x15,0x4},
		{"1V8_CMOS_B-",  0x2,0x2,0x15,0x6},
		{"1V8_CMOS_A+B+",0x3,0x2,0x15,0x8},
		{"1V8_CMOS_A-B+",0x3,0x2,0x15,0x9},
		{"1V8_CMOS_A+B-",0x3,0x2,0x15,0x4},
		{"1V8_CMOS_A-B-",0x3,0x2,0x15,0x6},

		{"1V5_HSTL_A+",  0x1,0x3,0x1f,0x8},
		{"1V5_HSTL_A-",  0x1,0x3,0x1f,0x9},
		{"1V5_HSTL_B+",  0x2,0x3,0x1f,0x4},
		{"1V5_HSTL_B-",  0x2,0x3,0x1f,0x6},
		{"1V5_HSTL_A+B+",0x3,0x3,0x1f,0x8},
		{"1V5_HSTL_A-B+",0x3,0x3,0x1f,0x9},
		{"1V5_HSTL_A+B-",0x3,0x3,0x1f,0x4},
		{"1V5_HSTL_A-B-",0x3,0x3,0x1f,0x6},

		{"3V3_SSTL_A+",  0x1,0x0,0x04,0x8},
		{"3V3_SSTL_A-",  0x1,0x0,0x04,0x9},
		{"3V3_SSTL_B+",  0x2,0x0,0x04,0x4},
		{"3V3_SSTL_B-",  0x2,0x0,0x04,0x6},
		{"3V3_SSTL_A+B+",0x3,0x0,0x04,0x8},
		{"3V3_SSTL_A-B+",0x3,0x0,0x04,0x9},
		{"3V3_SSTL_A+B-",0x3,0x0,0x04,0x5},
		{"3V3_SSTL_A-B-",0x3,0x0,0x04,0x6},

		{"2V5_SSTL_A+",  0x1,0x1,0x0d,0x8},
		{"2V5_SSTL_A-",  0x1,0x1,0x0d,0x9},
		{"2V5_SSTL_B+",  0x2,0x1,0x0d,0x4},
		{"2V5_SSTL_B-",  0x2,0x1,0x0d,0x6},
		{"2V5_SSTL_A+B+",0x3,0x1,0x0d,0x8},
		{"2V5_SSTL_A-B+",0x3,0x1,0x0d,0x9},
		{"2V5_SSTL_A+B-",0x3,0x1,0x0d,0x5},
		{"2V5_SSTL_A-B-",0x3,0x1,0x0d,0x6},

		{"1V8_SSTL_A+",  0x1,0x2,0x17,0x8},
		{"1V8_SSTL_A-",  0x1,0x2,0x17,0x9},
		{"1V8_SSTL_B+",  0x2,0x2,0x17,0x4},
		{"1V8_SSTL_B-",  0x2,0x2,0x17,0x6},
		{"1V8_SSTL_A+B+",0x3,0x2,0x17,0x8},
		{"1V8_SSTL_A-B+",0x3,0x2,0x17,0x9},
		{"1V8_SSTL_A+B-",0x3,0x2,0x17,0x4},
		{"1V8_SSTL_A-B-",0x3,0x2,0x17,0x6},

		{"3V3_LVPECL",   0x4,0x0,0x0f,0xc},
		{"2V5_LVPECL",   0x4,0x1,0x10,0xc},
		{"3V3_LVDS",     0x6,0x0,0x03,0xc},
		{"2V5_LVDS",     0x6,0x1,0x04,0xc},
		{"1V8_LVDS",     0x6,0x2,0x04,0xc},

		{NULL,           0x0,0x0,0x0,0x0},
};
static const char *out_dis_states[]= {"dis_hi-z","dis_low","dis_high","dis_always_on", NULL};
static const char *out_en_states[]=  {"output_en","output_dis", NULL};
static const char *out_pwr_states[]= {"output_power_up","output_power_down", NULL};
static const char *ms_pwr_states[]=  {"ms_power_up","ms_power_down", NULL};



static const int volatile_registers[]={AWE_STATUS, AWE_SOFT_RESET, AWE_FCAL_07_00, AWE_FCAL_15_08, AWE_FCAL_17_16, -1};
static const char *out_names[]={"output0","output1","output2","output3","outputs", NULL};
static const char *in_freq_names[]={"in_frequency12", "in_frequency3", "in_frequency4", "in_frequency56", "in_frequency12xo", NULL};
static const char *pll_setup_names[]={"pll_freq_fract", "pll_freq_int", "pll_by_out_fract", "pll_by_out_int", NULL};
static const char *out_freq_setup_names[]={
		"out0_freq_fract", "out1_freq_fract", "out2_freq_fract", "out3_freq_fract",
		"out0_freq_int",   "out1_freq_int",   "out2_freq_int",   "out3_freq_int", NULL};

static u32 awe_msx_ssup[4][3][3]=
	{{{AWE_MS0_SSUPP1_07_00,AWE_MS0_SSUPP1_11_08,0},
	  {AWE_MS0_SSUPP2_07_00,AWE_MS0_SSUPP2_14_08,0},
	  {AWE_MS0_SSUPP3_07_00,AWE_MS0_SSUPP3_14_08,0}},
	 {{AWE_MS1_SSUPP1_07_00,AWE_MS1_SSUPP1_11_08,0},
	  {AWE_MS1_SSUPP2_07_00,AWE_MS1_SSUPP2_14_08,0},
	  {AWE_MS1_SSUPP3_07_00,AWE_MS1_SSUPP3_14_08,0}},
	 {{AWE_MS2_SSUPP1_07_00,AWE_MS2_SSUPP1_11_08,0},
	  {AWE_MS2_SSUPP2_07_00,AWE_MS2_SSUPP2_14_08,0},
	  {AWE_MS2_SSUPP3_07_00,AWE_MS2_SSUPP3_14_08,0}},
	 {{AWE_MS3_SSUPP1_07_00,AWE_MS3_SSUPP1_11_08,0},
	  {AWE_MS3_SSUPP2_07_00,AWE_MS3_SSUPP2_14_08,0},
	  {AWE_MS3_SSUPP3_07_00,AWE_MS3_SSUPP3_14_08,0}}};
static u32 awe_msx_ssdn[4][3][3]=
	{{{AWE_MS0_SSDNP1_07_00,AWE_MS0_SSDNP1_11_08,0},
	  {AWE_MS0_SSDNP2_07_00,AWE_MS0_SSDNP2_14_08,0},
	  {AWE_MS0_SSDNP3_07_00,AWE_MS0_SSDNP3_14_08,0}},
	 {{AWE_MS1_SSDNP1_07_00,AWE_MS1_SSDNP1_11_08,0},
	  {AWE_MS1_SSDNP2_07_00,AWE_MS1_SSDNP2_14_08,0},
	  {AWE_MS1_SSDNP3_07_00,AWE_MS1_SSDNP3_14_08,0}},
	 {{AWE_MS2_SSDNP1_07_00,AWE_MS2_SSDNP1_11_08,0},
	  {AWE_MS2_SSDNP2_07_00,AWE_MS2_SSDNP2_14_08,0},
	  {AWE_MS2_SSDNP3_07_00,AWE_MS2_SSDNP3_14_08,0}},
	 {{AWE_MS3_SSDNP1_07_00,AWE_MS3_SSDNP1_11_08,0},
	  {AWE_MS3_SSDNP2_07_00,AWE_MS3_SSDNP2_14_08,0},
	  {AWE_MS3_SSDNP3_07_00,AWE_MS3_SSDNP3_14_08,0}}};
static u32 awe_msx_ssud[4][3]=
	 {{AWE_MS0_SSUDP1_03_00,AWE_MS0_SSUDP1_11_04,0},
	  {AWE_MS1_SSUDP1_03_00,AWE_MS1_SSUDP1_11_04,0},
	  {AWE_MS2_SSUDP1_03_00,AWE_MS2_SSUDP1_11_04,0},
	  {AWE_MS3_SSUDP1_03_00,AWE_MS3_SSUDP1_11_04,0}};

static const u32 awe_rdiv_in[]=      {AWE_R0DIV_IN,  AWE_R1DIV_IN,  AWE_R2DIV_IN,  AWE_R3DIV_IN};
static const u32 awe_rdiv_k[]=       {AWE_R0DIV,     AWE_R1DIV,     AWE_R2DIV,     AWE_R3DIV};
static const u32 awe_drv_fmt[]=      {AWE_DRV0_FMT,  AWE_DRV1_FMT,  AWE_DRV2_FMT,  AWE_DRV3_FMT};
static const u32 awe_drv_vddo[]=     {AWE_DRV0_VDDO, AWE_DRV1_VDDO, AWE_DRV2_VDDO, AWE_DRV3_VDDO};
static const u32 awe_drv_trim[][4]= {{AWE_DRV0_TRIM,0,0}, {AWE_DRV1_TRIM_A,AWE_DRV1_TRIM_B,0},{AWE_DRV2_TRIM,0,0},{AWE_DRV3_TRIM,0,0}};
static const u32 awe_drv_powerdown[]={AWE_DRV0_PDN,  AWE_DRV1_PDN,  AWE_DRV2_PDN,  AWE_DRV3_PDN};
static const u32 awe_drv_disable[]=  {AWE_OUT0_DIS,  AWE_OUT1_DIS,  AWE_OUT2_DIS,  AWE_OUT3_DIS, AWE_OUT_ALL_DIS};
static const u32 awe_drv_dis_state[]={AWE_OUT0_DIS_STATE, AWE_OUT1_DIS_STATE, AWE_OUT2_DIS_STATE, AWE_OUT3_DIS_STATE};
static const u32 awe_drv_invert[]=   {AWE_DRV0_INV,  AWE_DRV1_INV,  AWE_DRV2_INV,  AWE_DRV3_INV};
static const u32 awe_drv_inv[]=      {AWE_DRV0_INV,  AWE_DRV1_INV,  AWE_DRV2_INV,  AWE_DRV3_INV};

static const u32 awe_ms_hs[]=        {AWE_MS0_HS,    AWE_MS1_HS,    AWE_MS2_HS,    AWE_MS3_HS};
static const u32 awe_ms_ssmode[]=    {AWE_MS0_SSMODE,AWE_MS1_SSMODE,AWE_MS2_SSMODE,AWE_MS3_SSMODE};

/* (register_address << 8) | mask - created from SiLabs output */
static const u32 register_masks[]= {
		0x61d,0x1b80,0x1cff,0x1dff,0x1eff,0x1fff,0x20ff,0x21ff,
		0x22ff,0x23ff,0x241f,0x251f,0x261f,0x271f,0x28ff,0x297f,
		0x2a3f,0x2dff,0x2eff,0x2f3f,0x30ff,0x31ff,0x32ff,0x33ff,
		0x34ff,0x35ff,0x36ff,0x37ff,0x38ff,0x39ff,0x3aff,0x3bff,
		0x3cff,0x3dff,0x3e3f,0x3fff,0x40ff,0x41ff,0x42ff,0x43ff,
		0x44ff,0x45ff,0x46ff,0x47ff,0x48ff,0x493f,0x4aff,0x4bff,
		0x4cff,0x4dff,0x4eff,0x4fff,0x50ff,0x51ff,0x52ff,0x53ff,
		0x543f,0x55ff,0x56ff,0x57ff,0x58ff,0x59ff,0x5aff,0x5bff,
		0x5cff,0x5dff,0x5eff,0x5f3f,0x61ff,0x62ff,0x63ff,0x64ff,
		0x65ff,0x66ff,0x67ff,0x68ff,0x69ff,0x6abf,0x6bff,0x6cff,
		0x6dff,0x6eff,0x6fff,0x70ff,0x71ff,0x72ff,0x73ff,0x74ff,
		0x75ff,0x76ff,0x77ff,0x78ff,0x79ff,0x7aff,0x7bff,0x7cff,
		0x7dff,0x7eff,0x7fff,0x80ff,0x810f,0x820f,0x83ff,0x84ff,
		0x85ff,0x86ff,0x87ff,0x88ff,0x89ff,0x8aff,0x8bff,0x8cff,
		0x8dff,0x8eff,0x8fff,0x90ff,0x98ff,0x99ff,0x9aff,0x9bff,
		0x9cff,0x9dff,0x9e0f,0x9f0f,0xa0ff,0xa1ff,0xa2ff,0xa3ff,
		0xa4ff,0xa5ff,0xa6ff,0xa7ff,0xa8ff,0xa9ff,0xaaff,0xabff,
		0xacff,0xadff,0xaeff,0xafff,0xb0ff,0xb1ff,0xb2ff,0xb3ff,
		0xb4ff,0xb50f,0xb6ff,0xb7ff,0xb8ff,0xb9ff,0xbaff,0xbbff,
		0xbcff,0xbdff,0xbeff,0xbfff,0xc0ff,0xc1ff,0xc2ff,0xc3ff,
		0xc4ff,0xc5ff,0xc6ff,0xc7ff,0xc8ff,0xc9ff,0xcaff,0xcb0f,
		0xccff,0xcdff,0xceff,0xcfff,0xd0ff,0xd1ff,0xd2ff,0xd3ff,
		0xd4ff,0xd5ff,0xd6ff,0xd7ff,0xd8ff,0xd9ff,0xf202,0x11fff,
		0x120ff,0x121ff,0x122ff,0x123ff,0x124ff,0x125ff,0x126ff,0x127ff,
		0x128ff,0x129ff,0x12aff,0x12b0f,0x12fff,0x130ff,0x131ff,0x132ff,
		0x133ff,0x134ff,0x135ff,0x136ff,0x137ff,0x138ff,0x139ff,0x13aff,
		0x13b0f,0x13fff,0x140ff,0x141ff,0x142ff,0x143ff,0x144ff,0x145ff,
		0x146ff,0x147ff,0x148ff,0x149ff,0x14aff,0x14b0f,0x14fff,0x150ff,
		0x151ff,0x152ff,0x153ff,0x154ff,0x155ff,0x156ff,0x157ff,0x158ff,
		0x159ff,0x15aff,0x15b0f};

//AWE_MS0_SSMODE
static const u8 out_div_values[]={1,2,4,8,16,32};

static void si5338_init_of(struct i2c_client *client);

static int get_chn_from_name(const char * name);

static ssize_t invalidate_cache_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_address_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_data_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_address_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_data_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_all_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_help_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

//static ssize_t input_xtal_freq_show (struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t input_xtal_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_xtal_freq_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_frequency12_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_frequency3_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_frequency4_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_frequency56_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_frequency12_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_frequency12xo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_frequency3_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_frequency4_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_frequency56_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_p12_div_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_p12_div_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_mux_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_mux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_mux_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fb_mux_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fb_mux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fb_mux_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_pfd_ref_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_pfd_ref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_pfd_ref_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fb_external_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fb_external_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_pfd_fb_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t in_pfd_fb_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t in_pfd_fb_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pll_ref_frequency_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pll_fb_frequency_show (struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t ms_p123_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ms_p123_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ms_abc_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ms_abc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t ms_pwr_states_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ms_pwr_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int set_ms_pwr_states(struct device *dev, const char * name, int chn);
static int get_ms_powerup_state(struct device *dev, char * buf, int chn);
static ssize_t ms_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static ssize_t ss_change_freq_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ss_change_freq_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ss_vals_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ss_vals_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ss_regs_hex_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ss_regs_hex_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t pre_init_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t post_init_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t pll_freq_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pll_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ms_freq_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ms_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t out_source_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t out_source_txt_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_source_freq_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_div_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_div_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t out_div_by_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t out_freq_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t out_pwr_states_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_pwr_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int set_out_pwr_states(struct device *dev, const char * name, int chn);
static int get_powerup_state(struct device *dev, char * buf, int chn);
static ssize_t out_en_states_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_en_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int set_out_en_states(struct device *dev, const char * name, int chn);
static int get_enabled_state(struct device *dev, char * buf, int chn);
static ssize_t out_dis_states_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t out_dis_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int set_out_dis_states(struct device *dev, const char * name, int chn);
static int get_disabled_state(struct device *dev, char * buf, int chn);

#ifdef GENERATE_EXTRA
static ssize_t drv_powerdown_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_powerdown_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_disable_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_disabled_state_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_disabled_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_invert_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_invert_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_type_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_type_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_vdd_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_vdd_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_trim_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t drv_auto_trim_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_trim_any_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t drv_txt_show (struct device *dev, struct device_attribute *attr, char *buf);
static int update_drv_trim(struct i2c_client *client, int novtt, int chn); /* no Vtt - CMOS, no termination, where it matters */
static char * get_drv_txt(struct i2c_client *client, int chn);
#endif

static int make_config_out (struct device *dev);
static ssize_t status_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_description_show (struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t output_route_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_route_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int get_output_description (struct device *dev, char * buf, int chn);
static int get_out_frequency_txt(struct device *dev, char *buf, int chn);
static ssize_t output_config_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static int configure_output_driver(struct device *dev, const char * name, int chn);

static int remove_common_factor(u64 * num_denom);
static int _verify_output_channel(struct i2c_client *client,int chn);
static int get_ss_vals(struct device *dev, char * buf, int chn);
static int get_ss_state(struct i2c_client *client, int chn);
static int set_ss_state(struct i2c_client *client, int state, int chn);
static int get_ss_down_rate(struct i2c_client *client, int chn);
static int get_ss_down_amplitude(struct i2c_client *client, int chn);
static int store_ss_down_parameters(struct i2c_client *client, u32 rate, u32 amp, int chn);
static int set_ss_down(struct i2c_client *client, int chn);
static int ss_pre_freq_change(struct i2c_client *client, int chn);
static int ss_post_freq_change(struct i2c_client *client, int chn);

static int calc_ss_down_to_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn);
static int get_ss_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn);
static int set_ss_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn);

static int disable_spread_spectrum(struct i2c_client *client,int chn);
static int enable_spread_spectrum(struct i2c_client *client,int chn);
static int get_drv_powerdown(struct i2c_client *client, int chn);
static int set_drv_powerdown(struct i2c_client *client, int typ, int chn);
static int get_drv_disable(struct i2c_client *client, int chn);
static int set_drv_disable(struct i2c_client *client, int typ, int chn);
static int get_drv_disabled_state(struct i2c_client *client, int chn);
static int set_drv_disabled_state(struct i2c_client *client, int typ, int chn);
static int get_drv_invert(struct i2c_client *client, int chn);
static int set_drv_invert(struct i2c_client *client, int typ, int chn);
static int get_drv_type(struct i2c_client *client, int chn);
static int set_drv_type(struct i2c_client *client, int typ, int chn);
static int get_drv_vdd(struct i2c_client *client, int chn);
static int set_drv_vdd(struct i2c_client *client, int vdd, int chn);
static int get_drv_trim(struct i2c_client *client, int chn);
static int set_drv_trim_any(struct i2c_client *client, int trim, int chn);
static int set_out_div(struct i2c_client *client, int div, int chn); /*chn =0..3 */
static int get_out_div(struct i2c_client *client, int chn); /*chn =0..3 */
static int set_out_div_by_frequency(struct i2c_client *client, u64* out_freq, int chn); /*chn =0..3 */
static int get_out_frequency(struct i2c_client *client, u64* out_freq, int chn); /*chn =0..3 */
static int get_out_source(struct i2c_client *client, int chn);
static int set_out_source(struct i2c_client *client, int chn, int src);

static int get_out_ms(struct i2c_client *client, int chn);
static int get_out_route(struct i2c_client *client, char* buf, int chn);
static int set_out_route(struct i2c_client *client, const char* route, int chn);
static int set_out_frequency_and_route (struct i2c_client *client, u64 *out_freq, int chn, int int_div);
static s64 get_output_src_frequency(struct i2c_client *client, u64 *out_freq, int chn);

static int pre_init(struct i2c_client *client, int clear_all);

static int post_init(struct i2c_client *client, int timeout); /*1 in timeout ~ 0.1ms - i2c read register */
static int reset_ms(struct i2c_client *client, int wait_cycles);
static int get_status(struct i2c_client *client);
static int power_up_down_needed_ms(struct i2c_client *client);
static int disable_output(struct i2c_client *client, int chn);
static int disable_pll_in_fb_mux(struct i2c_client *client); /* to be explicitly enabled if needed */



static int set_pll_paremeters(struct i2c_client *client);
static int is_set_up(struct i2c_client *client);
static int set_misc_registers(struct i2c_client *client);
static int get_ms_powerdown(struct i2c_client *client, int chn);
static int set_ms_powerdown(struct i2c_client *client, int typ, int chn);
static int ms_to_p123(u64* ms,u32 * p123);
static int p123_to_ms(u64* ms,u32 * p123);
static int get_ms_p123(struct i2c_client *client,u32 * p123, int chn); /* chn 0,1,2,3,4 (4 - msn) */
static int set_ms_p123(struct i2c_client *client,u32 * p123, int chn); /* chn 0,1,2,3,4 (4 - msn) */
static int set_pll_freq(struct i2c_client *client, u64 *vco_freq, int int_div);
static int get_pll_freq(struct i2c_client *client,u64 * pll_freq);
static int set_pll_freq_by_out(struct i2c_client *client, u64 *out_freq, int int_msn_div);

static int get_pll_ms_freq(struct i2c_client *client, u64 *out_freq, int chn);
static int set_pll_ms_by_out(struct i2c_client *client, u64 *out_freq, int chn, int int_div);
static s64 get_pll_in_frequency(struct i2c_client *client);
static s64 get_pll_fb_frequency(struct i2c_client *client);
static s64 get_p1div_in_frequency(struct i2c_client *client);
static s64 get_p2div_in_frequency(struct i2c_client *client);


static int set_in_mux(struct i2c_client *client, int data);
static int get_in_mux(struct i2c_client *client);
static int set_fb_mux(struct i2c_client *client, int data);
static int get_fb_mux(struct i2c_client *client);
static int set_in_pdiv(struct i2c_client *client, int div, int chn); /*chn =0,1 */
static int get_in_pdiv(struct i2c_client *client, int chn); /*chn =0,1 */
static int set_in_pfd_ref_fb(struct i2c_client *client, u8 val, int chn); /*chn =0 - ref, 1 - fb*/
static int get_in_pfd_ref_fb(struct i2c_client *client, int chn); /*chn =0,1 */
static int set_fb_external(struct i2c_client *client, u8 val);
static int get_fb_external(struct i2c_client *client);
static int set_in_frequency(struct i2c_client *client, u64 frequency,int src); /* 0 - 12, 1 - 3, 2 - 4, 3 - 5,6, 4 - 12 XO */
static u64 get_in_frequency(struct i2c_client *client,int src);


static s64 read_multireg64 (struct i2c_client *client, const u32 * awe);
static int write_multireg64 (struct i2c_client *client, u64 data, const u32 * awe);
static int read_field (struct i2c_client *client, u32 awe);
static int write_field (struct i2c_client *client, u8 data, u32 awe);
static int write_adwe(struct i2c_client *client, u32 adwe);
static int write_reg(struct i2c_client *client, u16 reg, u8 val, u8 mask);
static int read_reg(struct i2c_client *client, u16 reg);
static void invalidate_cache(struct i2c_client *client);


/* raw access to i2c registers, need to set address (9 bits) first, then r/w data */

static DEVICE_ATTR(invalidate_cache, SYSFS_PERMISSIONS & SYSFS_WRITEONLY,   NULL,                invalidate_cache_store);
static DEVICE_ATTR(address,          SYSFS_PERMISSIONS,                     raw_address_show,    raw_address_store);
static DEVICE_ATTR(data,             SYSFS_PERMISSIONS,                     raw_data_show,       raw_data_store);
static DEVICE_ATTR(hex_address,      SYSFS_PERMISSIONS,                     raw_hex_address_show,raw_hex_address_store);
static DEVICE_ATTR(hex_data,         SYSFS_PERMISSIONS,                     raw_hex_data_show,   raw_hex_data_store);
static DEVICE_ATTR(hex_all,          SYSFS_PERMISSIONS & SYSFS_READONLY,    raw_hex_all_show,    NULL);
static DEVICE_ATTR(hex_adwe,         SYSFS_PERMISSIONS,                     raw_hex_adwe_show,   raw_hex_adwe_store);
static DEVICE_ATTR(hex_adwe_help,    SYSFS_PERMISSIONS & SYSFS_READONLY,    raw_hex_adwe_help_show,  NULL);


static struct attribute *raw_dev_attrs[] = {
		&dev_attr_invalidate_cache.attr,
		&dev_attr_address.attr,
		&dev_attr_data.attr,
		&dev_attr_hex_address.attr,
		&dev_attr_hex_data.attr,
		&dev_attr_hex_all.attr,
		&dev_attr_hex_adwe.attr,
		&dev_attr_hex_adwe_help.attr,
		NULL
};

static const struct attribute_group dev_attr_raw_group = {
	.attrs = raw_dev_attrs,
	.name  = "raw",
};

//static DEVICE_ATTR(xtal_freq,    SYSFS_PERMISSIONS,  input_xtal_freq_show, input_xtal_freq_store);
static DEVICE_ATTR(xtal_freq_txt, SYSFS_PERMISSIONS & SYSFS_READONLY,  input_xtal_freq_txt_show, NULL);
static DEVICE_ATTR(in_frequency12,   SYSFS_PERMISSIONS,                   in_frequency12_show, in_frequency12_store);
static DEVICE_ATTR(in_frequency12xo, SYSFS_PERMISSIONS,                   in_frequency12_show, in_frequency12xo_store);
static DEVICE_ATTR(in_frequency3,   SYSFS_PERMISSIONS,                    in_frequency3_show,  in_frequency3_store);
static DEVICE_ATTR(in_frequency4,   SYSFS_PERMISSIONS,                    in_frequency4_show,  in_frequency4_store);
static DEVICE_ATTR(in_frequency56,   SYSFS_PERMISSIONS,                   in_frequency56_show, in_frequency56_store);
static DEVICE_ATTR(in_p1_div,        SYSFS_PERMISSIONS,                   in_p12_div_show,   in_p12_div_store);
static DEVICE_ATTR(in_p2_div,        SYSFS_PERMISSIONS,                   in_p12_div_show,   in_p12_div_store);
static DEVICE_ATTR(in_mux,           SYSFS_PERMISSIONS,                   in_mux_show, in_mux_store);
static DEVICE_ATTR(in_mux_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  in_mux_txt_show, NULL);
static DEVICE_ATTR(fb_mux,           SYSFS_PERMISSIONS,                   fb_mux_show, fb_mux_store);
static DEVICE_ATTR(fb_mux_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  fb_mux_txt_show, NULL);
static DEVICE_ATTR(in_pfd_ref,       SYSFS_PERMISSIONS,                   in_pfd_ref_show, in_pfd_ref_store);
static DEVICE_ATTR(in_pfd_ref_txt,   SYSFS_PERMISSIONS & SYSFS_READONLY,  in_pfd_ref_txt_show, NULL);
static DEVICE_ATTR(in_pfd_fb,        SYSFS_PERMISSIONS,                   in_pfd_fb_show, in_pfd_fb_store);
static DEVICE_ATTR(in_pfd_fb_txt,    SYSFS_PERMISSIONS & SYSFS_READONLY,  in_pfd_fb_txt_show, NULL);
static DEVICE_ATTR(pll_ref_frequency,SYSFS_PERMISSIONS & SYSFS_READONLY,  pll_ref_frequency_show, NULL);
static DEVICE_ATTR(pll_fb_frequency, SYSFS_PERMISSIONS & SYSFS_READONLY,  pll_fb_frequency_show,  NULL);

static DEVICE_ATTR(fb_external,      SYSFS_PERMISSIONS,                   fb_external_show, fb_external_store);

static struct attribute *input_dev_attrs[] = {
//	&dev_attr_xtal_freq.attr,
	&dev_attr_xtal_freq_txt.attr,
	&dev_attr_in_frequency12.attr,
	&dev_attr_in_frequency12xo.attr,
	&dev_attr_in_frequency3.attr,
	&dev_attr_in_frequency4.attr,
	&dev_attr_in_frequency56.attr,
	&dev_attr_in_p1_div.attr,
	&dev_attr_in_p2_div.attr,
	&dev_attr_in_mux.attr,
	&dev_attr_in_mux_txt.attr,
	&dev_attr_fb_mux.attr,
	&dev_attr_fb_mux_txt.attr,
	&dev_attr_in_pfd_ref.attr,
	&dev_attr_in_pfd_ref_txt.attr,
	&dev_attr_in_pfd_fb.attr,
	&dev_attr_in_pfd_fb_txt.attr,
	&dev_attr_pll_ref_frequency.attr,
	&dev_attr_pll_fb_frequency.attr,
	&dev_attr_fb_external.attr,
	NULL
};

static const struct attribute_group dev_attr_input_group = {
	.attrs = input_dev_attrs,
	.name  = "input",
};

/* has to have/not have '_fract' in the name */
static DEVICE_ATTR(ms0_freq_fract,SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms0_freq_int,  SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms1_freq_fract,SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms1_freq_int,  SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms2_freq_fract,SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms2_freq_int,  SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms3_freq_fract,SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);
static DEVICE_ATTR(ms3_freq_int,  SYSFS_PERMISSIONS,  ms_freq_show,  ms_freq_store);

static DEVICE_ATTR(ms0_p123, SYSFS_PERMISSIONS,  ms_p123_show, ms_p123_store);
static DEVICE_ATTR(ms0_abc,  SYSFS_PERMISSIONS,  ms_abc_show,  ms_abc_store);
static DEVICE_ATTR(ms1_p123, SYSFS_PERMISSIONS,  ms_p123_show, ms_p123_store);
static DEVICE_ATTR(ms1_abc,  SYSFS_PERMISSIONS,  ms_abc_show,  ms_abc_store);
static DEVICE_ATTR(ms2_p123, SYSFS_PERMISSIONS,  ms_p123_show, ms_p123_store);
static DEVICE_ATTR(ms2_abc,  SYSFS_PERMISSIONS,  ms_abc_show,  ms_abc_store);
static DEVICE_ATTR(ms3_p123, SYSFS_PERMISSIONS,  ms_p123_show, ms_p123_store);
static DEVICE_ATTR(ms3_abc,  SYSFS_PERMISSIONS,  ms_abc_show,  ms_abc_store);
static DEVICE_ATTR(msn_p123, SYSFS_PERMISSIONS,  ms_p123_show, ms_p123_store);
static DEVICE_ATTR(msn_abc,  SYSFS_PERMISSIONS,  ms_abc_show,  ms_abc_store);

static DEVICE_ATTR(ms_power_down,  SYSFS_PERMISSIONS,  ms_pwr_states_show,  ms_pwr_states_store);
static DEVICE_ATTR(ms_power_up,    SYSFS_PERMISSIONS,  ms_pwr_states_show,  ms_pwr_states_store);
static DEVICE_ATTR(ms_reset, SYSFS_PERMISSIONS & SYSFS_WRITEONLY,  NULL,  ms_reset_store);

static struct attribute *multisynth_attrs[] = {
		&dev_attr_ms0_freq_fract.attr,
		&dev_attr_ms0_freq_int.attr,
		&dev_attr_ms1_freq_fract.attr,
		&dev_attr_ms1_freq_int.attr,
		&dev_attr_ms2_freq_fract.attr,
		&dev_attr_ms2_freq_int.attr,
		&dev_attr_ms3_freq_fract.attr,
		&dev_attr_ms3_freq_int.attr,

		&dev_attr_ms0_p123.attr,
		&dev_attr_ms0_abc.attr,
		&dev_attr_ms1_p123.attr,
		&dev_attr_ms1_abc.attr,
		&dev_attr_ms2_p123.attr,
		&dev_attr_ms2_abc.attr,
		&dev_attr_ms3_p123.attr,
		&dev_attr_ms3_abc.attr,
		&dev_attr_msn_p123.attr,
		&dev_attr_msn_abc.attr,
		&dev_attr_ms_power_down.attr,
		&dev_attr_ms_power_up.attr,
		&dev_attr_ms_reset.attr,
		NULL
};
static const struct attribute_group dev_attr_multisynth_group = {
	.attrs = multisynth_attrs,
	.name  = "multiSynth",
};

/* Spread spectrum group */
static DEVICE_ATTR(ss_change_freq_mode, SYSFS_PERMISSIONS,  ss_change_freq_mode_show, ss_change_freq_mode_store);
static DEVICE_ATTR(ss0_values,   SYSFS_PERMISSIONS,  ss_vals_show,     ss_vals_store);
static DEVICE_ATTR(ss1_values,   SYSFS_PERMISSIONS,  ss_vals_show,     ss_vals_store);
static DEVICE_ATTR(ss2_values,   SYSFS_PERMISSIONS,  ss_vals_show,     ss_vals_store);
static DEVICE_ATTR(ss3_values,   SYSFS_PERMISSIONS,  ss_vals_show,     ss_vals_store);
static DEVICE_ATTR(ss0_regs_hex, SYSFS_PERMISSIONS,  ss_regs_hex_show, ss_regs_hex_store);
static DEVICE_ATTR(ss1_regs_hex, SYSFS_PERMISSIONS,  ss_regs_hex_show, ss_regs_hex_store);
static DEVICE_ATTR(ss2_regs_hex, SYSFS_PERMISSIONS,  ss_regs_hex_show, ss_regs_hex_store);
static DEVICE_ATTR(ss3_regs_hex, SYSFS_PERMISSIONS,  ss_regs_hex_show, ss_regs_hex_store);
static struct attribute *spread_spectrum_attrs[] = {
	&dev_attr_ss_change_freq_mode.attr,
	&dev_attr_ss0_values.attr,
	&dev_attr_ss1_values.attr,
	&dev_attr_ss2_values.attr,
	&dev_attr_ss3_values.attr,
	&dev_attr_ss0_regs_hex.attr,
	&dev_attr_ss1_regs_hex.attr,
	&dev_attr_ss2_regs_hex.attr,
	&dev_attr_ss3_regs_hex.attr,
	&dev_attr_ms_reset.attr,
	NULL
};
static const struct attribute_group dev_attr_spread_spectrum_group = {
	.attrs = spread_spectrum_attrs,
	.name  = "spread_spectrum",
};




static DEVICE_ATTR(pre_init,         SYSFS_PERMISSIONS & SYSFS_WRITEONLY,  NULL,           pre_init_store);
static DEVICE_ATTR(pre_init_clear,   SYSFS_PERMISSIONS & SYSFS_WRITEONLY,  NULL,           pre_init_store);
static DEVICE_ATTR(post_init,  SYSFS_PERMISSIONS & SYSFS_WRITEONLY,  NULL,           post_init_store);
static DEVICE_ATTR(pll_freq_fract,  SYSFS_PERMISSIONS,               pll_freq_show,  pll_freq_store);
static DEVICE_ATTR(pll_freq_int,    SYSFS_PERMISSIONS,               pll_freq_show,  pll_freq_store);
static DEVICE_ATTR(pll_by_out_fract,SYSFS_PERMISSIONS,               pll_freq_show,  pll_freq_store);
static DEVICE_ATTR(pll_by_out_int,  SYSFS_PERMISSIONS,               pll_freq_show,  pll_freq_store);


static struct attribute *pll_dev_attrs[] = {
/*	&dev_attr_pre_init.attr,
	&dev_attr_pre_init_clear.attr,
	&dev_attr_post_init.attr, */
	&dev_attr_pll_ref_frequency.attr,
	&dev_attr_pll_freq_fract.attr,
	&dev_attr_pll_freq_int.attr,
	&dev_attr_pll_by_out_fract.attr,
	&dev_attr_pll_by_out_int.attr,
	NULL
};

static const struct attribute_group dev_attr_pll_group = {
	.attrs = pll_dev_attrs,
	.name  = "pll",
};

static DEVICE_ATTR(out0_source,     SYSFS_PERMISSIONS,                   out_source_show,     out_source_store);
static DEVICE_ATTR(out0_source_txt, SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_txt_show, NULL);
static DEVICE_ATTR(out1_source,     SYSFS_PERMISSIONS,                   out_source_show,     out_source_store);
static DEVICE_ATTR(out1_source_txt, SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_txt_show, NULL);
static DEVICE_ATTR(out2_source,     SYSFS_PERMISSIONS,                   out_source_show,     out_source_store);
static DEVICE_ATTR(out2_source_txt, SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_txt_show, NULL);
static DEVICE_ATTR(out3_source,     SYSFS_PERMISSIONS,                   out_source_show,     out_source_store);
static DEVICE_ATTR(out3_source_txt, SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_txt_show, NULL);
static DEVICE_ATTR(out0_source_freq,SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_freq_show, NULL);
static DEVICE_ATTR(out1_source_freq,SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_freq_show, NULL);
static DEVICE_ATTR(out2_source_freq,SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_freq_show, NULL);
static DEVICE_ATTR(out3_source_freq,SYSFS_PERMISSIONS & SYSFS_READONLY,  out_source_freq_show, NULL);

static DEVICE_ATTR(out0_div,        SYSFS_PERMISSIONS,                   out_div_show,     out_div_store);
static DEVICE_ATTR(out1_div,        SYSFS_PERMISSIONS,                   out_div_show,     out_div_store);
static DEVICE_ATTR(out2_div,        SYSFS_PERMISSIONS,                   out_div_show,     out_div_store);
static DEVICE_ATTR(out3_div,        SYSFS_PERMISSIONS,                   out_div_show,     out_div_store);

static DEVICE_ATTR(out0_div_by_freq,SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,           out_div_by_freq_store);
static DEVICE_ATTR(out0_freq_int,   SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out0_freq_fract, SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out1_div_by_freq,SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,           out_div_by_freq_store);
static DEVICE_ATTR(out1_freq_int,   SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out1_freq_fract, SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out2_div_by_freq,SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,           out_div_by_freq_store);
static DEVICE_ATTR(out2_freq_int,   SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out2_freq_fract, SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out3_div_by_freq,SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,           out_div_by_freq_store);
static DEVICE_ATTR(out3_freq_int,   SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);
static DEVICE_ATTR(out3_freq_fract, SYSFS_PERMISSIONS,                   out_freq_show,  out_freq_store);

static DEVICE_ATTR(out0_route,      SYSFS_PERMISSIONS,                   output_route_show,    output_route_store);
static DEVICE_ATTR(out1_route,      SYSFS_PERMISSIONS,                   output_route_show,    output_route_store);
static DEVICE_ATTR(out2_route,      SYSFS_PERMISSIONS,                   output_route_show,    output_route_store);
static DEVICE_ATTR(out3_route,      SYSFS_PERMISSIONS,                   output_route_show,    output_route_store);
//output_route_show

static struct attribute *output_dev_attrs[] = {
	&dev_attr_out0_source.attr,
	&dev_attr_out0_source_txt.attr,
	&dev_attr_out1_source.attr,
	&dev_attr_out1_source_txt.attr,
	&dev_attr_out2_source.attr,
	&dev_attr_out2_source_txt.attr,
	&dev_attr_out3_source.attr,
	&dev_attr_out3_source_txt.attr,
	&dev_attr_out0_source_freq.attr,
	&dev_attr_out1_source_freq.attr,
	&dev_attr_out2_source_freq.attr,
	&dev_attr_out3_source_freq.attr,
	&dev_attr_out0_div.attr,
	&dev_attr_out1_div.attr,
	&dev_attr_out2_div.attr,
	&dev_attr_out3_div.attr,
	&dev_attr_out0_div_by_freq.attr,
	&dev_attr_out1_div_by_freq.attr,
	&dev_attr_out2_div_by_freq.attr,
	&dev_attr_out3_div_by_freq.attr,
	&dev_attr_out0_freq_int.attr,
	&dev_attr_out1_freq_int.attr,
	&dev_attr_out2_freq_int.attr,
	&dev_attr_out3_freq_int.attr,
	&dev_attr_out0_freq_fract.attr,
	&dev_attr_out1_freq_fract.attr,
	&dev_attr_out2_freq_fract.attr,
	&dev_attr_out3_freq_fract.attr,
	&dev_attr_out0_route.attr,
	&dev_attr_out1_route.attr,
	&dev_attr_out2_route.attr,
	&dev_attr_out3_route.attr,
	NULL
};

static const struct attribute_group dev_attr_output_group = {
	.attrs = output_dev_attrs,
	.name  = "output_clocks",
};
/* output drivers */
/* NOTE: state of the outputs changes with clock only, changing "dis_low" to "dis_high" does not work when disabled.
 * Going through "dis_always_on" works
 */
#ifdef GENERATE_EXTRA
static DEVICE_ATTR(drv0_powerdown,     SYSFS_PERMISSIONS,                   drv_powerdown_show,      drv_powerdown_store);
static DEVICE_ATTR(drv1_powerdown,     SYSFS_PERMISSIONS,                   drv_powerdown_show,      drv_powerdown_store);
static DEVICE_ATTR(drv2_powerdown,     SYSFS_PERMISSIONS,                   drv_powerdown_show,      drv_powerdown_store);
static DEVICE_ATTR(drv3_powerdown,     SYSFS_PERMISSIONS,                   drv_powerdown_show,      drv_powerdown_store);

static DEVICE_ATTR(drv0_disable,       SYSFS_PERMISSIONS,                   drv_disable_show,        drv_disable_store);
static DEVICE_ATTR(drv1_disable,       SYSFS_PERMISSIONS,                   drv_disable_show,        drv_disable_store);
static DEVICE_ATTR(drv2_disable,       SYSFS_PERMISSIONS,                   drv_disable_show,        drv_disable_store);
static DEVICE_ATTR(drv3_disable,       SYSFS_PERMISSIONS,                   drv_disable_show,        drv_disable_store);

static DEVICE_ATTR(drv0_disabled_state,SYSFS_PERMISSIONS,                   drv_disabled_state_show, drv_disabled_state_store);
static DEVICE_ATTR(drv1_disabled_state,SYSFS_PERMISSIONS,                   drv_disabled_state_show, drv_disabled_state_store);
static DEVICE_ATTR(drv2_disabled_state,SYSFS_PERMISSIONS,                   drv_disabled_state_show, drv_disabled_state_store);
static DEVICE_ATTR(drv3_disabled_state,SYSFS_PERMISSIONS,                   drv_disabled_state_show, drv_disabled_state_store);

static DEVICE_ATTR(drv0_invert,        SYSFS_PERMISSIONS,                   drv_invert_show,         drv_invert_store);
static DEVICE_ATTR(drv1_invert,        SYSFS_PERMISSIONS,                   drv_invert_show,         drv_invert_store);
static DEVICE_ATTR(drv2_invert,        SYSFS_PERMISSIONS,                   drv_invert_show,         drv_invert_store);
static DEVICE_ATTR(drv3_invert,        SYSFS_PERMISSIONS,                   drv_invert_show,         drv_invert_store);

static DEVICE_ATTR(drv0_invert_txt,    SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_invert_txt_show, NULL);
static DEVICE_ATTR(drv1_invert_txt,    SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_invert_txt_show, NULL);
static DEVICE_ATTR(drv2_invert_txt,    SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_invert_txt_show, NULL);
static DEVICE_ATTR(drv3_invert_txt,    SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_invert_txt_show, NULL);

static DEVICE_ATTR(drv0_type,          SYSFS_PERMISSIONS,                   drv_type_show,           drv_type_store);
static DEVICE_ATTR(drv1_type,          SYSFS_PERMISSIONS,                   drv_type_show,           drv_type_store);
static DEVICE_ATTR(drv2_type,          SYSFS_PERMISSIONS,                   drv_type_show,           drv_type_store);
static DEVICE_ATTR(drv3_type,          SYSFS_PERMISSIONS,                   drv_type_show,           drv_type_store);

static DEVICE_ATTR(drv0_type_txt,      SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_type_txt_show, NULL);
static DEVICE_ATTR(drv1_type_txt,      SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_type_txt_show, NULL);
static DEVICE_ATTR(drv2_type_txt,      SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_type_txt_show, NULL);
static DEVICE_ATTR(drv3_type_txt,      SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_type_txt_show, NULL);

static DEVICE_ATTR(drv0_vdd,           SYSFS_PERMISSIONS,                   drv_vdd_show,            drv_vdd_store);
static DEVICE_ATTR(drv1_vdd,           SYSFS_PERMISSIONS,                   drv_vdd_show,            drv_vdd_store);
static DEVICE_ATTR(drv2_vdd,           SYSFS_PERMISSIONS,                   drv_vdd_show,            drv_vdd_store);
static DEVICE_ATTR(drv3_vdd,           SYSFS_PERMISSIONS,                   drv_vdd_show,            drv_vdd_store);

static DEVICE_ATTR(drv0_vdd_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_vdd_txt_show, NULL);
static DEVICE_ATTR(drv1_vdd_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_vdd_txt_show, NULL);
static DEVICE_ATTR(drv2_vdd_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_vdd_txt_show, NULL);
static DEVICE_ATTR(drv3_vdd_txt,       SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_vdd_txt_show, NULL);

static DEVICE_ATTR(drv0_trim,          SYSFS_PERMISSIONS,                   drv_trim_show,           drv_trim_any_store);
static DEVICE_ATTR(drv1_trim,          SYSFS_PERMISSIONS,                   drv_trim_show,           drv_trim_any_store);
static DEVICE_ATTR(drv2_trim,          SYSFS_PERMISSIONS,                   drv_trim_show,           drv_trim_any_store);
static DEVICE_ATTR(drv3_trim,          SYSFS_PERMISSIONS,                   drv_trim_show,           drv_trim_any_store);

static DEVICE_ATTR(drv0_auto_trim,     SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,                     drv_auto_trim_store);
static DEVICE_ATTR(drv1_auto_trim,     SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,                     drv_auto_trim_store);
static DEVICE_ATTR(drv2_auto_trim,     SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,                     drv_auto_trim_store);
static DEVICE_ATTR(drv3_auto_trim,     SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,                     drv_auto_trim_store);

static DEVICE_ATTR(drv0_description,   SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_txt_show, NULL);
static DEVICE_ATTR(drv1_description,   SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_txt_show, NULL);
static DEVICE_ATTR(drv2_description,   SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_txt_show, NULL);
static DEVICE_ATTR(drv3_description,   SYSFS_PERMISSIONS & SYSFS_READONLY,  drv_txt_show, NULL);

static struct attribute *output_extra_dev_attrs[] = {
	&dev_attr_drv0_powerdown.attr,
	&dev_attr_drv1_powerdown.attr,
	&dev_attr_drv2_powerdown.attr,
	&dev_attr_drv3_powerdown.attr,

	&dev_attr_drv0_disable.attr,
	&dev_attr_drv1_disable.attr,
	&dev_attr_drv2_disable.attr,
	&dev_attr_drv3_disable.attr,

	&dev_attr_drv0_disabled_state.attr,
	&dev_attr_drv1_disabled_state.attr,
	&dev_attr_drv2_disabled_state.attr,
	&dev_attr_drv3_disabled_state.attr,

	&dev_attr_drv0_invert.attr,
	&dev_attr_drv1_invert.attr,
	&dev_attr_drv2_invert.attr,
	&dev_attr_drv3_invert.attr,

	&dev_attr_drv0_invert_txt.attr,
	&dev_attr_drv1_invert_txt.attr,
	&dev_attr_drv2_invert_txt.attr,
	&dev_attr_drv3_invert_txt.attr,

	&dev_attr_drv0_type.attr,
	&dev_attr_drv1_type.attr,
	&dev_attr_drv2_type.attr,
	&dev_attr_drv3_type.attr,

	&dev_attr_drv0_type_txt.attr,
	&dev_attr_drv1_type_txt.attr,
	&dev_attr_drv2_type_txt.attr,
	&dev_attr_drv3_type_txt.attr,

	&dev_attr_drv0_vdd.attr,
	&dev_attr_drv1_vdd.attr,
	&dev_attr_drv2_vdd.attr,
	&dev_attr_drv3_vdd.attr,

	&dev_attr_drv0_vdd_txt.attr,
	&dev_attr_drv1_vdd_txt.attr,
	&dev_attr_drv2_vdd_txt.attr,
	&dev_attr_drv3_vdd_txt.attr,

	&dev_attr_drv0_trim.attr,
	&dev_attr_drv1_trim.attr,
	&dev_attr_drv2_trim.attr,
	&dev_attr_drv3_trim.attr,

	&dev_attr_drv0_auto_trim.attr,
	&dev_attr_drv1_auto_trim.attr,
	&dev_attr_drv2_auto_trim.attr,
	&dev_attr_drv3_auto_trim.attr,

	&dev_attr_drv0_description.attr,
	&dev_attr_drv1_description.attr,
	&dev_attr_drv2_description.attr,
	&dev_attr_drv3_description.attr,
	NULL
};

static const struct attribute_group dev_attr_output_extra_group = {
	.attrs = output_extra_dev_attrs,
	.name  = "output_extra",
};

#endif


/* root directory */
static DEVICE_ATTR(outputs,   SYSFS_PERMISSIONS & SYSFS_READONLY,  output_description_show, NULL);
static DEVICE_ATTR(status,    SYSFS_PERMISSIONS & SYSFS_READONLY,  status_show, NULL);

static struct attribute *root_dev_attrs[] = {
		&dev_attr_pre_init.attr,
		&dev_attr_pre_init_clear.attr,
		&dev_attr_post_init.attr,
		&dev_attr_outputs.attr,
		&dev_attr_status.attr,
	    NULL
};
static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};






static int get_chn_from_name(const char * name)
{
	char * cp = strpbrk(name,"0123456789");
	return (cp)?(cp[0]-'0'):-1;
}
static int make_config_out(struct device *dev)
{
	int retval=-1;
	int index,iout,num_types,num_files;
	struct attribute **pattrs; /* array of pointers to attibutes */
	struct device_attribute *dev_attrs;
	struct attribute_group *attr_group;
	for (num_types=0;drv_configs[num_types].description;num_types++);
	num_files=num_types;
	for (iout=0;out_dis_states[iout];iout++) num_files++;
	for (iout=0;out_en_states[iout];iout++) num_files++;
	for (iout=0;out_pwr_states[iout];iout++) num_files++;
	for (iout=0;out_names[iout];iout++) num_files++;
	pattrs = devm_kzalloc(dev,(num_files+1)*sizeof(pattrs[0]), GFP_KERNEL);
	if (!pattrs) return -ENOMEM;
	dev_attrs = devm_kzalloc(dev, num_files*sizeof(dev_attrs[0]), GFP_KERNEL);
	if (!dev_attrs) return -ENOMEM;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group) return -ENOMEM;
	memset(dev_attrs,  0, num_files*sizeof(dev_attrs[0]));
	memset(attr_group, 0, sizeof(*attr_group));
	for (index=0;index<num_types;index++) {
		dev_attrs[index].attr.name=drv_configs[index].description;
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		dev_attrs[index].show= output_config_show;
		dev_attrs[index].store=output_config_store;
		pattrs[index]=&(dev_attrs[index].attr);
	}
	/*  add outputs disabled states */
	for (iout=0;out_dis_states[iout];iout++) {
		dev_attrs[index].attr.name=out_dis_states[iout];
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		dev_attrs[index].show=out_dis_states_show;
		dev_attrs[index].store=out_dis_states_store;
		pattrs[index]=&(dev_attrs[index].attr);
		index++;
	}
	/*  add outputs enable */
	for (iout=0;out_en_states[iout];iout++) {
		dev_attrs[index].attr.name=out_en_states[iout];
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		dev_attrs[index].show=out_en_states_show;
		dev_attrs[index].store=out_en_states_store;
		pattrs[index]=&(dev_attrs[index].attr);
		index++;
	}
	/*  add outputs enable  */
	for (iout=0;out_pwr_states[iout];iout++) {
		dev_attrs[index].attr.name=out_pwr_states[iout];
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		dev_attrs[index].show=out_pwr_states_show;
		dev_attrs[index].store=out_pwr_states_store;
		pattrs[index]=&(dev_attrs[index].attr);
		index++;
	}
	/*  add outputs (readonly) */
	for (iout=0;out_names[iout];iout++) {
		dev_attrs[index].attr.name=out_names[iout];
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS & SYSFS_READONLY;
		dev_attrs[index].show=output_description_show;
		dev_attrs[index].store=NULL;
		pattrs[index]=&(dev_attrs[index].attr);
		index++;
	}
	pattrs[index]=NULL;
	attr_group->name  = "output_drivers";
	attr_group->attrs =pattrs;
	dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
	index=0;
	while ((*attr_group).attrs[index]){
		dev_dbg(dev,"attr=%s\n",attr_group->attrs[index]->name);
		index++;
	}
	if (&dev->kobj) {
		retval = sysfs_create_group(&dev->kobj, attr_group);
	}
	return retval;
}

static ssize_t status_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int status;
	struct i2c_client *client = to_i2c_client(dev);
	if (((status=get_status(client)))<0) return status;
	return sprintf(buf,"0x%x input clock: %s, feedback clock: %s, PLL lock: %s, calibration: %s\n",
			status,(status & 0x4)?"LOST":"OK",(status & 0x8)?"LOST":"OK",(status & 0x10)?"LOST":"OK",(status & 0x10)?"IN PROGRESS":"DONE");
}

static ssize_t output_description_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int i,i1,rc,len=0,show_number,ms;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0; out_names[i]; i++)	if (strcmp(attr->attr.name,out_names[i]) == 0) break;
	if (!out_names[i]) return -EINVAL;
	if (i==4) { /* all outputs */
		i=0;
		i1=4;
		show_number=1;
	} else {
		i1=i+1;
		show_number=0;
	}
	for (;i<i1;i++){
		if (show_number){
			rc=sprintf(buf,"%d: ",i);
			buf+=rc;
			len+=rc;
		}
		if (((rc=get_output_description(dev, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;

		rc=sprintf(buf,", output frequency: ");
		buf+=rc;
		len+=rc;
		if (((rc=get_out_frequency_txt(dev, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;

		rc=sprintf(buf,", output route: ");
		buf+=rc;
		len+=rc;
		if (((rc=get_out_route(client, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;
		/* Show MSx power state only if it is used fro the output */
		if (((ms=get_out_ms(client,i))) >= 0){
			rc=sprintf(buf,", ");
			buf+=rc;
			len+=rc;
			if (((rc=get_ms_powerup_state(dev, buf,i)))<0) return rc;
			buf+=rc;
			len+=rc;
		}
		
		rc=sprintf(buf,", disabled state: ");
		buf+=rc;
		len+=rc;
		if (((rc=get_disabled_state(dev, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;

		rc=sprintf(buf,", ");
		buf+=rc;
		len+=rc;
		if (((rc=get_powerup_state(dev, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;

		rc=sprintf(buf,", ");
		buf+=rc;
		len+=rc;
		if (((rc=get_enabled_state(dev, buf,i)))<0) return rc;
		buf+=rc;
		len+=rc;
		/* show spread spectum settings */
		rc=sprintf(buf,", ");
		buf+=rc;
		len+=rc;
		if (((rc=get_ss_vals(dev, buf, i)))<0) return rc;
		buf+=rc;
		len+=rc;
		
		rc=sprintf(buf,"\n");
		buf+=rc;
		len+=rc;
	}
	return len;
}

static ssize_t output_route_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn,rc,len=0;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name);
	if (((rc=get_out_route(client, buf,chn)))<0) return rc;
	buf+=rc;
	len+=rc;
	rc=sprintf(buf,"\n");
	buf+=rc;
	len+=rc;
	return len;
}

static ssize_t output_route_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, rc;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name);
	if (((rc=set_out_route(client, buf, chn)))<0) return rc;
    return count;
}

//static void invalidate_cache(struct i2c_client *client)


static int get_output_description (struct device *dev, char * buf, int chn)
{
	int drv_type, drv_vdd, drv_trim, drv_invert,i;
	struct i2c_client *client = to_i2c_client(dev);
	if (((i=_verify_output_channel(client,chn)))<0) return i;
	if (((drv_type=  get_drv_type    (client,  chn)))<0) return drv_type;
	if (((drv_vdd=   get_drv_vdd     (client,  chn)))<0) return drv_vdd;
	if (((drv_trim=  get_drv_trim    (client,  chn)))<0) return drv_trim;
	if (((drv_invert=get_drv_invert  (client,  chn)))<0) return drv_invert;
	for (i=0; drv_configs[i].description; i++)	{
		if ((drv_configs[i].fmt==drv_type) &&
			(drv_configs[i].vdd==drv_vdd) &&
			(drv_configs[i].trim==drv_trim) &&
			((drv_invert |(drv_configs[i].invert>>2)) == ((drv_configs[i].invert & 3) | (drv_configs[i].invert>>2)))){
			return sprintf (buf,drv_configs[i].description);
		}
	}
	return sprintf (buf,"Invalid output configuration: type = %d, vdd=%d, trim=%d, invert=%d",drv_type,drv_vdd,drv_trim,drv_invert);
}

static int get_out_frequency_txt(struct device *dev, char *buf, int chn)
{
	int rc;
	u64 out_freq[3];
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=get_out_frequency(client, out_freq, chn)))<0) return sprintf (buf,"Not set");
	if (out_freq[1]==0) return sprintf(buf, "%lld Hz",out_freq[0]);
    return sprintf(buf, "%lld-%lld/%lld Hz",out_freq[0],out_freq[1],out_freq[2]);
}

static ssize_t output_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, i, rc;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0; drv_configs[i].description; i++)	if (strcmp(attr->attr.name,drv_configs[i].description) == 0) {
			break;
	}
	if (!drv_configs[i].description) return -EINVAL; /* filename does not exist - BUG */
	for (chn=0;chn<4;chn++){
		if (((rc=get_drv_type    (client, chn)))<0) return rc;
		if (rc!=drv_configs[i].fmt) continue;
		if (((rc=get_drv_vdd     (client, chn)))<0) return rc;
		if (rc!=drv_configs[i].vdd) continue;
		if (((rc=get_drv_trim    (client, chn)))<0) return rc;
		if (rc!=drv_configs[i].trim) continue;
		if (((rc=get_drv_invert  (client, chn)))<0) return rc;
		if (rc!= (drv_configs[i].invert & 3)) continue;
		buf+=sprintf(buf," %d",chn);
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

static ssize_t output_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, num_bytes,rc;
	while ((rc=sscanf(buf, "%d%n", &chn,&num_bytes))){
		dev_dbg(dev,"buf=%s rc==%d chn=%d num_bytes=%d", buf, rc,chn,num_bytes);
		buf+=num_bytes;
		if (((rc=configure_output_driver(dev, attr->attr.name, chn)))<0) return rc;
	}
    return count;
}


static int configure_output_driver(struct device *dev, const char * name, int chn)
{
	int i,rc;
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev,"name=%s chn=%d", name,chn);

	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (i=0; drv_configs[i].description; i++)	if (strcmp(name,drv_configs[i].description) == 0) {
		if (((rc=set_drv_type    (client, drv_configs[i].fmt,      chn)))<0) return rc;
		if (((rc=set_drv_vdd     (client, drv_configs[i].vdd,      chn)))<0) return rc;
		if (((rc=set_drv_trim_any(client, drv_configs[i].trim,     chn)))<0) return rc;
		if (((rc=set_drv_invert  (client, drv_configs[i].invert&3, chn)))<0) return rc;
		return 0;
	}
	return -EINVAL;
}


static int si5338_sysfs_register(struct device *dev)
{
	int retval=0;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_raw_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_input_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_multisynth_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_pll_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_output_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_spread_spectrum_group)))<0) return retval;
#ifdef GENERATE_EXTRA
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_output_extra_group)))<0) return retval;
#endif
		if (((retval = make_config_out (dev)))<0) return retval;
	}
	return retval;
}

static ssize_t invalidate_cache_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	invalidate_cache(client);
    return count;
}

static ssize_t raw_address_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct si5338_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    return sprintf(buf, "%d\n",clientdata->reg_addr);
}
static ssize_t raw_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct si5338_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    sscanf(buf, "%du", &clientdata->reg_addr);
    return count;
}

static ssize_t raw_data_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	int data= read_reg(client, clientdata->reg_addr);
    return sprintf(buf, "%d\n",data);
}
static ssize_t raw_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	int data;
    sscanf(buf, "%du", &data);
    write_reg(client, clientdata->reg_addr, data, 0xff); /* write all register, it is up to user to do R-mod-W */
    return count;
}

static ssize_t raw_hex_address_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct si5338_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    return sprintf(buf, "0x%03x\n",clientdata->reg_addr);
}
static ssize_t raw_hex_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct si5338_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    sscanf(buf, "%x", &clientdata->reg_addr);
    return count;
}

static ssize_t raw_hex_data_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	int data= read_reg(client, clientdata->reg_addr);
    return sprintf(buf, "0x%02x\n",data);
}
static ssize_t raw_hex_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	int data;
    sscanf(buf, "%x", &data);
    write_reg(client, clientdata->reg_addr, data, 0xff); /* write all register, it is up to user to do R-mod-W */
    return count;
}

static ssize_t raw_hex_all_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int low_addr=0,reg,data,rc,len=0, count=PAGE_SIZE;
	struct i2c_client *client = to_i2c_client(dev);
//	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	for (reg=low_addr;reg<=LAST_REG;reg++) if (count>10){
		if ((reg & 0xf) ==0){
			rc=sprintf(buf, "%03x: ",reg);
		    buf+=rc;
		    len+=rc;
		    count-=rc;
		}
		data= read_reg(client, reg); //ignore errors
		if (data<0) rc=sprintf(buf, "??");
		else        rc=sprintf(buf, "%02x",data);
	    buf+=rc;
	    len+=rc;
	    count-=rc;
		if (((reg & 0xf) == 0xf) || (reg==LAST_REG)){
			rc=sprintf(buf, "\n");
		} else {
			rc=sprintf(buf, " ");
		}
	    buf+=rc;
	    len+=rc;
	    count-=rc;
	}
	return len;
}
static ssize_t raw_hex_adwe_help_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Setting one/multiple registers with masks in the form [0x]AAADDWW, where AAA is register address\n" \
			           "DD - data byte and WW - write enable bits ( 1 - write, 0 - keep old)\n" \
			           "When read, provides current register data that can be used in device tree.\n");

}

//static const u32 register_masks[]= {
static ssize_t raw_hex_adwe_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int i,data;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0;i<ARRAY_SIZE(register_masks);i++){
		if (((data=read_reg(client, register_masks[i]>>8)))<0) return data;
		buf+=sprintf(buf," 0x%x",((register_masks[i] & 0x1ff00)<<8) | (register_masks[i] & 0xff) | ((data & 0xff)<<8));
		if (((i+1) & 0x7)==0) buf+=sprintf(buf,"\n");  
	}
	buf+=sprintf(buf,"\n");
	return buf-cp;
}

/*
 *  accepts single or multiple data, each [0x]AAADDWW - AAA - register address, DD - data byte, WW - write enable mask (1 - write, 0 - keep).
 *  Ignores any other characters, so same format as in dts with hex data is OK
 */
static ssize_t raw_hex_adwe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const char hex_digits[]="0123456789abcdefABCDEF";
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata= i2c_get_clientdata(client);
	int adwe,rc=0;
	int left=count,num_bytes;
	const char * cp;
 	mutex_lock(&clientdata->lock);
	while ((left>0) && ((cp=strpbrk(buf,hex_digits))) && cp[0]){
		left -= (cp-buf);
		buf = cp;
		dev_dbg(dev,"left=%d", left);
	    sscanf(buf, "%x%n", &adwe,&num_bytes);
	    left-=num_bytes;
	    buf+=num_bytes;
	    dev_dbg(dev,"left=%d num_bytes=%d, adwe=0x%08x", left,num_bytes,adwe);
	    if (((rc=write_adwe(client, adwe)))<0) {
	    	mutex_unlock(&clientdata->lock);
	    	return rc;
	    }
	}
	mutex_unlock(&clientdata->lock);
    return count;
}

static ssize_t input_xtal_freq_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *txt[]= {"8MHz..11Mhz", "11MHz..19Mhz", "19MHz..26Mhz", "26MHz..30Mhz"};
	struct i2c_client *client = to_i2c_client(dev);
	int data= read_field (client, AWE_XTAL_FREQ);
    return sprintf(buf, "%s\n",(data>=0)?txt[data]:"error");
}


static ssize_t in_frequency12_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 freq= get_in_frequency (client,0);
	if (freq<0) return -EINVAL;
    return sprintf(buf, "%lld\n",freq);
}
static ssize_t in_frequency3_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 freq= get_in_frequency (client,1);
	if (freq<0) return -EINVAL;
    return sprintf(buf, "%lld\n",freq);
}
static ssize_t in_frequency4_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 freq= get_in_frequency (client,2);
	if (freq<0) return -EINVAL;
    return sprintf(buf, "%lld\n",freq);
}
static ssize_t in_frequency56_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 freq= get_in_frequency (client,3);
	if (freq<0) return -EINVAL;
    return sprintf(buf, "%lld\n",freq);
}
static ssize_t in_frequency12_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	u64 freq;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%lld", &freq);
    if (((rc=set_in_frequency (client, freq,0)))<0) return rc;
    return count;
}
static ssize_t in_frequency12xo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	u64 freq;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%lld", &freq);
    if (((rc=set_in_frequency (client, freq,4)))<0) return rc;
    return count;
}
static ssize_t in_frequency3_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	u64 freq;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%lld", &freq);
    if (((rc=set_in_frequency (client, freq,1)))<0) return rc;
    return count;
}
static ssize_t in_frequency4_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	u64 freq;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%lld", &freq);
    if (((rc=set_in_frequency (client, freq,2)))<0) return rc;
    return count;
}
static ssize_t in_frequency56_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	u64 freq;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%lld", &freq);
    if (((rc=set_in_frequency (client, freq,3)))<0) return rc;
    return count;
}

static ssize_t in_p12_div_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int div, chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name)+1;
	if (((div=get_in_pdiv(client,chn)))<0) return div;
    return sprintf(buf, "%d\n",div);
}
static ssize_t in_p12_div_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int div,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name)+1;
    sscanf(buf, "%d", &div);
    if (((rc=set_in_pdiv(client, div,chn)))<0) return rc;
    return count;
}

static ssize_t in_mux_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_mux(client)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t in_mux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data,rc;
	sscanf(buf, "%d", &data);
    if (((rc=set_in_mux(client, data)))<0) return rc;
    return count;
}
static ssize_t in_mux_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *mux_txt[]={"IN1/IN2(diff)","IN3(single ended)","IN1/IN2(xtal)"};
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_mux(client)))<0) return data;
    return sprintf(buf, "%s\n",mux_txt[data]);
}
static ssize_t fb_mux_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_fb_mux(client)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t fb_mux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data,rc;
	sscanf(buf, "%d", &data);
    if (((rc=set_fb_mux(client, data)))<0) return rc;
    return count;
}
static ssize_t fb_mux_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *mux_fb_txt[]={"IN5/IN6(diff)","IN4(single ended)","No clock"};
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_fb_mux(client)))<0) return data;
    return sprintf(buf, "%s\n",mux_fb_txt[data]);
}

static ssize_t in_pfd_ref_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_pfd_ref_fb(client,0)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t in_pfd_ref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data,rc;
	sscanf(buf, "%d", &data);
    if (((rc=set_in_pfd_ref_fb(client, data,0)))<0) return rc;
    return count;
}
static ssize_t in_pfd_ref_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *pfd_ref_txt[]={"p1div_in(refclk)","p2div_in(fbclk)","p1div_out(refclk)","p2div_out(fbclk)","xoclk","noclk"};
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_pfd_ref_fb(client,0)))<0) return data;
    return sprintf(buf, "%s\n",pfd_ref_txt[data]);
}

static ssize_t fb_external_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data= get_fb_external(client)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t fb_external_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data,rc;
	sscanf(buf, "%d", &data);
    if (((rc=set_fb_external(client, data)))<0) return rc;
    return count;
}

static ssize_t in_pfd_fb_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_pfd_ref_fb(client,1)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t in_pfd_fb_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data,rc;
	sscanf(buf, "%d", &data);
    if (((rc=set_in_pfd_ref_fb(client, data,1)))<0) return rc;
    return count;
}
static ssize_t in_pfd_fb_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *pfd_fb_txt[]={"p2div_in(fbclk)","p1div_in(refclk)","p2div_out(fbclk)","p1div_out(refclk)","reserved","noclk"};
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	if (((data=get_in_pfd_ref_fb(client,1)))<0) return data;
    return sprintf(buf, "%s\n",pfd_fb_txt[data]);
}

static ssize_t pll_ref_frequency_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 pll_in_freq= get_pll_in_frequency(client);
	if (pll_in_freq<0) return (int) pll_in_freq;
    return sprintf(buf, "%lld\n",pll_in_freq);
}
static ssize_t pll_fb_frequency_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s64 pll_fb_freq= get_pll_fb_frequency(client);
	if (pll_fb_freq<0) return (int) pll_fb_freq;
    return sprintf(buf, "%lld\n",pll_fb_freq);
}
static ssize_t ms_p123_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,chn;
	u32 p123[3];
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (attr->attr.name[2]=='n') chn=4; /* exception for msn */
	if (((rc=get_ms_p123(client,p123, chn)))<0) return rc;
//    return sprintf(buf, "%ld %ld %ld\n",p123[0],p123[1],p123[2]);
    return sprintf(buf, "%u %u %u\n",p123[0],p123[1],p123[2]);
}
static ssize_t ms_p123_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,chn;
	u32 p123[3];
	int num_items;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (attr->attr.name[2]=='n') chn=4; /* exception for msn */
	num_items=sscanf(buf, "%u %u %u", &p123[0], &p123[1], &p123[2]);
	if (num_items<3){
		p123[1]=0;
		p123[2]=1;
	}
	if (((rc=set_ms_p123(client,p123, chn)))<0) return rc;
	if (chn<4){
		if (((rc=disable_spread_spectrum(client,chn)))<0) return rc;
	}
    return count;
}
static ssize_t ms_abc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,chn;
	u32 p123[3];
	u64 ms[3];
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (chn<0) chn=4; /* exception for msn - should have no digits*/
	if (((rc=get_ms_p123(client,p123, chn)))<0) return rc;
	p123_to_ms(ms,p123);
    return sprintf(buf, "%lld %lld %lld\n",ms[0],ms[1],ms[2]);
}
static ssize_t ms_abc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,chn;
	u32 p123[3];
	u64 ms[3];
	int num_items;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (chn<0) chn=4; /* exception for msn - should have no digits*/
	num_items=sscanf(buf, "%lld %lld %lld", &ms[0], &ms[1], &ms[2]);
	if (num_items<3){
		ms[1]=0;
		ms[2]=1;
	} else {
		remove_common_factor(&ms[1]);
	}
	ms_to_p123(ms,p123);
	if (((rc=set_ms_p123(client,p123, chn)))<0) return rc;
	if (chn<4){
		if (((rc=disable_spread_spectrum(client,chn)))<0) return rc;
	}
    return count;
}

static ssize_t ms_pwr_states_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, i;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (chn=0;chn<4;chn++){
		for (i=0; ms_pwr_states[i]; i++)	if (strcmp(attr->attr.name,ms_pwr_states[i]) == 0) {
			if (i== get_ms_powerdown(client, chn)){
				buf+=sprintf(buf," %d",chn);
				break;
			}
		}
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

static ssize_t ms_pwr_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, num_bytes,rc;
	while ((rc=sscanf(buf, "%d%n", &chn,&num_bytes))){
		dev_dbg(dev,"buf=%s rc==%d chn=%d num_bytes=%d", buf, rc,chn,num_bytes);
		buf+=num_bytes;
		if (((rc=set_ms_pwr_states(dev, attr->attr.name, chn)))<0) return rc;
	}
    return count;
}

static int set_ms_pwr_states(struct device *dev, const char * name, int chn)
{
	int i,rc;
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev,"name=%s chn=%d", name,chn);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (i=0; ms_pwr_states[i]; i++)	if (strcmp(name,ms_pwr_states[i]) == 0) {
	    if (((rc=set_ms_powerdown(client, i, chn)))<0) return rc;
		return 0;
	}
	return -EINVAL;
}

static int get_ms_powerup_state(struct device *dev, char * buf, int chn)
{
	int index;
	struct i2c_client *client = to_i2c_client(dev);
	if (((index=get_ms_powerdown(client,chn)))<0) return index;
	return sprintf (buf,ms_pwr_states[index]);
}

static ssize_t ms_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	reset_ms(client, 10);
	return count;
}

static ssize_t ss_change_freq_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	mode=clientdata->ss_on_freq_change;
	switch (mode) {
	case 0: return sprintf(buf, "%d - turn spread spectrum off on frequency change\n",mode);
	case 1: return sprintf(buf, "%d - recalculate spread spectrum on frequency change, do not reset MS\n",mode);
	case 2: return sprintf(buf, "%d - turn spread spectrum off on frequency change, reset MS when SS is turned on\n",mode);
	case 3: return sprintf(buf, "%d - recalculate spread spectrum on frequency change, do not reset MS\n",mode);
	default: return sprintf(buf, "%d - invalid mode\n",mode);
	}
}

static ssize_t ss_change_freq_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int num_items, mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	num_items=sscanf(buf, "%d", &mode);
	if (num_items && (mode>=0) && (mode<=3)){
		clientdata->ss_on_freq_change=mode;
		return count;
	}
	return -EINVAL;
}

static ssize_t ss_vals_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn,len;
	if (((chn=get_chn_from_name(attr->attr.name)))<0) return chn;
	if (((len= get_ss_vals(dev, buf, chn)))<0) return len;
	sprintf (buf+len,"\n");
	return len+1;
}

static ssize_t ss_vals_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, rc, state, num_items;
	u32 rate,amp;
	struct i2c_client *client = to_i2c_client(dev);
	if (((chn=get_chn_from_name(attr->attr.name)))<0) return chn;
	/* get current values */
	if (((state= get_ss_state(client, chn)))<0) return state;
	if (((rate=  get_ss_down_rate(client, chn)))<0) return rate;
	if (((amp=   get_ss_down_amplitude(client, chn)))<0) return amp;
	num_items=sscanf(buf, "%d %d %d", &state, &amp, &rate);
	if (num_items>1){
		if (((rc= store_ss_down_parameters(client, rate, amp, chn)))<0) return rc;
	}
	if (num_items>0){
		if (state) {
			/* calculate and set SS registers */
			if (((rc=set_ss_down(client, chn)))<0) return rc;
			/* enable SS, optionally reset MS */
			if (((rc=enable_spread_spectrum(client, chn)))<0) return rc;
		} else {
			if (((rc=disable_spread_spectrum(client, chn)))<0) return rc;
		}
	}
    return count;
}

static ssize_t ss_regs_hex_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, rc;
	u32 regs[7];
	u32 *updown_reg, *up_regs, *down_regs;
	struct i2c_client *client = to_i2c_client(dev);
	updown_reg=&regs[0];
	down_regs=&regs[1];
	up_regs=&regs[4];

	if (((chn=get_chn_from_name(attr->attr.name)))<0) return chn;
	if (((rc= get_ss_regs(client, up_regs, down_regs, updown_reg, chn)))<0) return rc;
	return sprintf(buf, "updown_par=0x%x down_pars=0x%x 0x%x 0x%x up_pars= 0x%x 0x%x 0x%x\n",
			regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);
}

static ssize_t ss_regs_hex_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, rc, num_items;
	u32 regs[7];
	u32 *updown_reg, *up_regs, *down_regs;
	struct i2c_client *client = to_i2c_client(dev);
	updown_reg=&regs[0];
	down_regs=&regs[1];
	up_regs=&regs[4];

	if (((chn=get_chn_from_name(attr->attr.name)))<0) return chn;
	if (((rc= get_ss_regs(client, up_regs, down_regs, updown_reg, chn)))<0) return rc;

	num_items=sscanf(buf, "%x %x %x %x %x %x %x", &regs[0], &regs[1], &regs[2], &regs[3], &regs[4], &regs[5], &regs[6]);
	if (num_items>0){
		if (num_items<5) up_regs=NULL;
		if (num_items<2) down_regs=NULL;
		if (((rc= set_ss_regs(client, up_regs, down_regs, updown_reg, chn)))<0) return rc;
	}
    return count;
}

static ssize_t pre_init_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,clear_all;
	struct i2c_client *client = to_i2c_client(dev);
	clear_all=strstr(attr->attr.name,"clear")?1:0;
	if (((rc=pre_init(client,clear_all)))<0) return rc;
    return count;
}

static ssize_t post_init_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc,timeout=0;
	sscanf(buf, "%d", &timeout);
	if (timeout <=0) timeout=INIT_TIMEOUT;
	if (((rc=post_init(client,timeout)))<0) return rc;
    return count;
}

static ssize_t pll_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	u64 pll_freq[3];
	struct i2c_client *client = to_i2c_client(dev);

	if (((rc=get_pll_freq(client,pll_freq)))<0) return rc;
    return sprintf(buf, "%lld %lld %lld\n",pll_freq[0],pll_freq[1],pll_freq[2]);
}

static ssize_t pll_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc,int_div,by_out;
	u64 freq[3];
	int num_items;
    int_div=(strstr(attr->attr.name,"_fract"))?0:1; /* if filename contains '_fract' - 0, not - 1 */
    by_out=(strstr(attr->attr.name,"_by_out"))?1:0; /* if filename contains '_by_out' - 1, not - 0 */

	num_items=sscanf(buf, "%lld %lld %lld", &freq[0], &freq[1], &freq[2]);
	if (num_items<3){
		freq[1]=0;
		freq[2]=1;
	}
	if (by_out) {
		if (((rc=set_pll_freq_by_out(client, freq, int_div)))<0) return rc;
	} else {
		if (((rc=set_pll_freq       (client, freq, int_div)))<0) return rc;
	}
	if (((rc=set_pll_paremeters(client)))<0) return rc;
/*	if (((rc=set_misc_registers(client)))<0) return rc;*/  /* moved to pre_init() */
    return count;
}


static ssize_t ms_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,chn;
	u64 ms_freq[3];
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((rc=get_pll_ms_freq(client, ms_freq, chn)))<0) return rc;
    return sprintf(buf, "%lld %lld %lld\n",ms_freq[0],ms_freq[1],ms_freq[2]);
}
static ssize_t ms_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,chn,int_div;
	u64 freq[3];
	int num_items;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
    int_div=(strstr(attr->attr.name,"_fract"))?0:1; /* if includes 'fract' - 0, not - 1 */
	num_items=sscanf(buf, "%lld %lld %lld", &freq[0], &freq[1], &freq[2]);
	if (num_items<3){
		freq[1]=0;
		freq[2]=1;
	}
	if (((rc=set_pll_ms_by_out(client, freq, chn, int_div)))<0) return rc;
    return count;
}

/* -----------Output section--------------------------- */
static ssize_t out_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int out_src,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((out_src=get_out_source(client, chn)))<0) return out_src;
    return sprintf(buf, "%d\n",out_src);
}
static ssize_t out_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc,chn;
	int out_src;
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &out_src);
	if (((rc=set_out_source(client, chn, out_src)))<0) return rc;
    return count;
}
static ssize_t out_source_txt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int out_src,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((out_src=get_out_source(client, chn)))<0) return out_src;
	switch (out_src){
	case 0: return sprintf(buf, "p2div_in\n");
	case 1: return sprintf(buf, "p1div_in\n");
	case 2: return sprintf(buf, "p2div_out\n");
	case 3: return sprintf(buf, "p1div_out\n");
	case 4: return sprintf(buf, "xoclk\n");
	case 5: return sprintf(buf, "MS0\n");
	case 6: return sprintf(buf, "MS%d\n",chn);
	case 7: return sprintf(buf, "No clock\n");
	}
    return -EINVAL;
}

static ssize_t out_source_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,chn;
	u64 out_source_freq[3];
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((rc=get_output_src_frequency(client, out_source_freq, chn)))<0) return rc;
    return sprintf(buf, "%lld %lld %lld\n",out_source_freq[0],out_source_freq[1],out_source_freq[2]);
}

static ssize_t out_div_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int div,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((div=get_out_div(client,chn)))<0) return div;
    return sprintf(buf, "%d\n",div);
}
static ssize_t out_div_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int div,rc,chn;
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
    sscanf(buf, "%d", &div);
    if (((rc=set_out_div(client, div,chn)))<0) return rc;
    return count;
}

static ssize_t out_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,chn;
	u64 out_freq[3];
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((rc=get_out_frequency(client, out_freq, chn)))<0) return rc;
    return sprintf(buf, "%lld %lld %lld\n",out_freq[0],out_freq[1],out_freq[2]);
}

static ssize_t out_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,int_div,chn;
	u64 freq[3];
	int num_items;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
    int_div=(strstr(attr->attr.name,"_fract"))?0:1; /* if filename contains '_fract' - 0, not - 1 */

	num_items=sscanf(buf, "%lld %lld %lld", &freq[0], &freq[1], &freq[2]);
	if (num_items<3){
		freq[1]=0;
		freq[2]=1;
	}
	if (((rc=set_out_frequency_and_route (client, freq, chn, int_div)))<0) return rc;
    return count;
}

static ssize_t out_div_by_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc,chn;
	u64 freq[3];
	int num_items;
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	num_items=sscanf(buf, "%lld %lld %lld", &freq[0], &freq[1], &freq[2]);
	if (num_items<3){
		freq[1]=0;
		freq[2]=1;
	}
	if (((rc=set_out_div_by_frequency(client, freq, chn)))<0) return rc;
	return count;
}

static ssize_t out_pwr_states_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, i;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (chn=0;chn<4;chn++){
		for (i=0; out_pwr_states[i]; i++)	if (strcmp(attr->attr.name,out_pwr_states[i]) == 0) {
			if (i== get_drv_powerdown(client, chn)){
				buf+=sprintf(buf," %d",chn);
				break;
			}
		}
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

static ssize_t out_pwr_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, num_bytes,rc;
	while ((rc=sscanf(buf, "%d%n", &chn,&num_bytes))){
		dev_dbg(dev,"buf=%s rc==%d chn=%d num_bytes=%d", buf, rc,chn,num_bytes);
		buf+=num_bytes;
		if (((rc=set_out_pwr_states(dev, attr->attr.name, chn)))<0) return rc;
	}
    return count;
}

static int set_out_pwr_states(struct device *dev, const char * name, int chn)
{
	int i,rc;
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev,"name=%s chn=%d", name,chn);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (i=0; out_pwr_states[i]; i++)	if (strcmp(name,out_pwr_states[i]) == 0) {
	    if (((rc=set_drv_powerdown(client, i, chn)))<0) return rc;
		return 0;
	}
	return -EINVAL;
}

static int get_powerup_state(struct device *dev, char * buf, int chn)
{
	int index;
	struct i2c_client *client = to_i2c_client(dev);
	if (((index=get_drv_powerdown(client,chn)))<0) return index;
	return sprintf (buf,out_pwr_states[index]);
}

static ssize_t out_en_states_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, i;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (chn=0;chn<4;chn++){
		for (i=0; out_en_states[i]; i++)	if (strcmp(attr->attr.name,out_en_states[i]) == 0) {
			if (i== get_drv_disable(client, chn)){
				buf+=sprintf(buf," %d",chn);
				break;
			}
		}
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

static ssize_t out_en_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, num_bytes,rc;
	while ((rc=sscanf(buf, "%d%n", &chn,&num_bytes))){
		dev_dbg(dev,"buf=%s rc==%d chn=%d num_bytes=%d", buf, rc,chn,num_bytes);
		buf+=num_bytes;
		if (((rc=set_out_en_states(dev, attr->attr.name, chn)))<0) return rc;
	}
    return count;
}

static int set_out_en_states(struct device *dev, const char * name, int chn)
{
	int i,rc;
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev,"name=%s chn=%d", name,chn);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (i=0; out_en_states[i]; i++)	if (strcmp(name,out_en_states[i]) == 0) {
	    if (((rc=set_drv_disable(client, i, chn)))<0) return rc;
		return 0;
	}
	return -EINVAL;
}

static int get_enabled_state(struct device *dev, char * buf, int chn)
{
	int index;
	struct i2c_client *client = to_i2c_client(dev);
	if (((index=get_drv_disable(client,chn)))<0) return index;
	return sprintf (buf,out_en_states[index]);
}

static ssize_t out_dis_states_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, i;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (chn=0;chn<4;chn++){
		for (i=0; out_dis_states[i]; i++)	if (strcmp(attr->attr.name,out_dis_states[i]) == 0) {
			if (i== get_drv_disabled_state(client, chn)){
				buf+=sprintf(buf," %d",chn);
				break;
			}
		}
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}
static ssize_t out_dis_states_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, num_bytes,rc;
	while ((rc=sscanf(buf, "%d%n", &chn,&num_bytes))){
		dev_dbg(dev,"buf=%s rc==%d chn=%d num_bytes=%d", buf, rc,chn,num_bytes);
		buf+=num_bytes;
		if (((rc=set_out_dis_states(dev, attr->attr.name, chn)))<0) return rc;
	}
    return count;
}

static int set_out_dis_states(struct device *dev, const char * name, int chn)
{
	int i,rc;
	struct i2c_client *client = to_i2c_client(dev);
	dev_dbg(dev,"name=%s chn=%d", name,chn);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (i=0; out_dis_states[i]; i++)	if (strcmp(name,out_dis_states[i]) == 0) {
	    if (((rc=set_drv_disabled_state(client, i, chn)))<0) return rc;
		return 0;
	}
	return -EINVAL;
}
static int get_disabled_state(struct device *dev, char * buf, int chn)
{
	int index;
	struct i2c_client *client = to_i2c_client(dev);
	if (((index=get_drv_disabled_state(client,chn)))<0) return index;
	return sprintf (buf,out_dis_states[index]);
}

#ifdef GENERATE_EXTRA
static ssize_t drv_powerdown_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_powerdown(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_powerdown_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_powerdown(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_disable_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_disable(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_disable(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_disabled_state_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_disabled_state(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_disabled_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_disabled_state(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_invert_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_invert(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_invert(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_invert_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_invert(client,chn)))<0) return data;
	switch (data) {
	case 0:return sprintf(buf, "No inversion\n");
	case 1:return sprintf(buf, "Invert A only (CMOS/SSTL,HSTL)\n");
	case 2:return sprintf(buf, "Invert B only (CMOS/SSTL,HSTL)\n");
	case 3:return sprintf(buf, "Invert both A and B (CMOS/SSTL,HSTL)\n");
	}
    return 0; /* never */
}

static ssize_t drv_type_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_type(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_type(client, data, chn)))<0) return rc;
    return count;
}

static ssize_t drv_type_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_type(client,chn)))<0) return data;
	switch (data) {
	case 0:return sprintf(buf, "reserved\n");
	case 1:return sprintf(buf, "CMOS/SSTL/HSTL A enabled, B disabled\n");
	case 2:return sprintf(buf, "CMOS/SSTL/HSTL A disabled, A enabled\n");
	case 3:return sprintf(buf, "CMOS/SSTL/HSTL A enabled, B enabled\n");
	case 4:return sprintf(buf, "LVPECL\n");
	case 5:return sprintf(buf, "LVDS\n");
	case 6:return sprintf(buf, "CML\n");
	case 7:return sprintf(buf, "HCSL\n");
	}
    return 0; /* never */
}

static ssize_t drv_vdd_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_vdd(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}
static ssize_t drv_vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_vdd(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_vdd_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_vdd(client,chn)))<0) return data;
	switch (data) {
	case 0:return sprintf(buf, "3.3V\n");
	case 1:return sprintf(buf, "2.5V\n");
	case 2:return sprintf(buf, "1.8V\n");
	case 3:return sprintf(buf, "1.5V\n");
	}
    return 0; /* never */
}
static ssize_t drv_trim_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int data,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_trim(client,chn)))<0) return data;
    return sprintf(buf, "%d\n",data);
}

static ssize_t drv_auto_trim_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=update_drv_trim(client, data, chn)))<0) return rc;
    return count;
}
static ssize_t drv_trim_any_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data,rc,chn;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	sscanf(buf, "%d", &data);
    if (((rc=set_drv_trim_any(client, data, chn)))<0) return rc;
    return count;
}

static ssize_t drv_txt_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn;
	char * data;
	struct i2c_client *client = to_i2c_client(dev);
	chn=get_chn_from_name(attr->attr.name); /* uses first digit in the name */
	if (((data=get_drv_txt(client,chn)))==NULL) return -EINVAL;
    return sprintf(buf, "%s\n",data);
}

/* uses out_type and out_vddo */
static int update_drv_trim(struct i2c_client *client, int novtt, int chn) /* no Vtt - CMOS, no termination, where it matters */
{
	int rc;
	int out_type, out_vdd,trim=-1;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((out_type=get_drv_type(client,chn)))<0) return out_type;
	if (((out_vdd=get_drv_vdd(client,chn)))<0)   return out_vdd;
	switch (out_type) {
	case 1:
	case 2:
	case 3:
		switch (out_vdd){
		case 0:trim=novtt?0x17:0x04; break;
		case 1:trim=novtt?0x13:0x0d; break;
		case 2:trim=novtt?0x17:0x15; break;
		case 3:trim=0x1f; break;
		}
		break;
	case 4:
		switch (out_vdd){
		case 0:trim=0x0f; break;
		case 1:trim=0x10; break;
		}
		break;
	case 5:
		switch (out_vdd){
		case 0:trim=0x08; break;
		case 1:trim=0x09; break;
		}
		break;
	case 6:
		switch (out_vdd){
		case 0:trim=0x03; break;
		case 1:
		case 2:trim=0x04; break;
		}
		break;
	case 7:
		switch (out_vdd){
		case 0:
		case 1:
		case 2:trim=0x07; break;
		}
		break;
	}
	if (trim<0){
		dev_err(&client->dev, "Invalid combination of output type (%d) and voltage (%d)\n",out_type,out_vdd);
		return - EINVAL;
	}
	return write_multireg64(client, trim, awe_drv_trim[chn]);
}
/* uses out_type, out_vddo and out_trim */
static char * get_drv_txt(struct i2c_client *client, int chn)
{
	int rc;
	int out_type, out_vdd,out_trim=-1;
	if (((rc=_verify_output_channel(client,chn)))<0) return NULL;
	if (((out_type=get_drv_type(client,chn)))<0) return NULL;
	if (((out_vdd=get_drv_vdd(client,chn)))<0)  return NULL;
	if (((out_vdd=get_drv_trim(client,chn)))<0)  return NULL;
	switch (out_type) {
	case 1:
		switch (out_vdd){
		case 0:return (out_trim & 0x10)?"3.3V CMOS, A & B":"3.3V SSTL, A only";
		case 1:return (out_trim & 0x10)?"2.5V CMOS, A & B":"2.5V SSTL, A only";
		case 2:return (out_trim & 0x02)?"1.8V SSTL, A & B":"1.8V CMOS, A only";
		case 3:return "1.5V HSTL, A only";
		}
		break;
	case 2:
		switch (out_vdd){
		case 0:return (out_trim & 0x10)?"3.3V CMOS, A & B":"3.3V SSTL, B only";
		case 1:return (out_trim & 0x10)?"2.5V CMOS, A & B":"2.5V SSTL, B only";
		case 2:return (out_trim & 0x02)?"1.8V SSTL, A & B":"1.8V CMOS, B only";
		case 3:return "1.5V HSTL, B only";
		}
		break;
	case 3:
		switch (out_vdd){
		case 0:return (out_trim & 0x10)?"3.3V CMOS, A & B":"3.3V SSTL, A & B";
		case 1:return (out_trim & 0x10)?"2.5V CMOS, A & B":"2.5V SSTL, A & B";
		case 2:return (out_trim & 0x02)?"1.8V SSTL, A & B":"1.8V CMOS, A & B";
		case 3:return "1.5V HSTL, A & B";
		}
		break;
	case 4:
		switch (out_vdd){
		case 0:return "3.3V LVPECL";
		case 1:return "2.5V LVPECL";
		}
		break;
	case 5:
		switch (out_vdd){
		case 0:return "3.3V CML";
		case 1:return "2.5V CML";
		}
		break;
	case 6:
		switch (out_vdd){
		case 0:return "3.3V LVDS";
		case 1:return "2.5V LVDS";
		case 2:return "1.8V LVDS";
		}
		break;
	case 7:
		switch (out_vdd){
		case 0:return "3.3V HCSL";
		case 1:return "2.5V HCSL";
		case 2:return "1.8V HCSL";
		}
		break;
	}
	dev_err(&client->dev, "Invalid combination of output type (%d) and voltage (%d)\n",out_type,out_vdd);
	switch (out_vdd){
	case 0:return "3.3V - invalid type";
	case 1:return "2.5V - invalid type";
	case 2:return "1.8V - invalid type";
	case 3:return "1.5V - invalid type";
	}
	return NULL; /* never */
}

#endif

/* -----------Output section--------------------------- */

static int remove_common_factor(u64 * num_denom)
{
	u64 a,b,r;
	if (num_denom[1]==0) return -1; /* zero denominator */
	if (num_denom[0]==0) {
		num_denom[1]=1;
		return 1;
	}
	a=max(num_denom[0],num_denom[1]);
	b=min(num_denom[0],num_denom[1]);
	r=b;
	while (r>1) {
		r=a-b*div64_u64(a,b);
		if (r==0){
			num_denom[0]=div64_u64(num_denom[0],b);
			num_denom[1]=div64_u64(num_denom[1],b);
			return 1;
		}
		a=b;
		b=r;
	}
	return 0; /* nothing done */
}
static int _verify_output_channel(struct i2c_client *client,int chn)
{
	if ((chn<0) || (chn>3)){
		dev_err(&client->dev, "Invalid output channel: %d (only 0..3 are allowed)\n",chn);
		return - EINVAL;
	}
	return 0;
}

static int get_ss_vals(struct device *dev, char * buf, int chn)
{
	int state;
	u32 rate,amp;
	struct i2c_client *client = to_i2c_client(dev);
	if (((state= get_ss_state(client, chn)))<0) return state;
	if (((amp=   get_ss_down_amplitude(client, chn)))<0) return amp;
	if (((rate=  get_ss_down_rate(client, chn)))<0) return rate;
	return sprintf(buf, "Spread spectrum is %s, down amplitude= %d ( *0.01%%), spread rate= %d Hz", state?"ON":"OFF", amp, rate);
}

static int get_ss_state(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field(client, awe_ms_ssmode[chn]);
}

static int set_ss_state(struct i2c_client *client, int state, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	 return write_field(client, state , awe_ms_ssmode[chn]);
}

static int get_ss_down_rate(struct i2c_client *client, int chn)
{
	int rc;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return clientdata->spread_spectrum_rate[chn]; /* in Hz */
}

static int get_ss_down_amplitude(struct i2c_client *client, int chn)
{
	int rc;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return clientdata->spread_spectrum_amp[chn];
}


/* store required parameters - they will be needed to recalculate SS registers after MS frequency change */
static int store_ss_down_parameters(struct i2c_client *client, u32 rate, u32 amp, int chn) /* chn 0,1,2,3 */
{
	int rc;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((rate < SPREAD_RATE_MIN) || (rate > SPREAD_RATE_MAX)){
		dev_err(&client->dev, "Invalid spread spectrum rate %u - should be in [%u,%u]Hz\n",rate,SPREAD_RATE_MIN,SPREAD_AMP_MAX);
		return - EINVAL;
	}
	if ((amp < SPREAD_AMP_MIN) || (amp > SPREAD_AMP_MAX)){
		dev_err(&client->dev, "Invalid spread spectrum amplitude %u - should be in [%u,%u] (*0.01%%)\n",amp,SPREAD_AMP_MIN,SPREAD_AMP_MAX);
		return - EINVAL;
	}
	clientdata->spread_spectrum_rate[chn]=rate; /* in Hz */
	clientdata->spread_spectrum_amp[chn]=amp; /* in 0.01% */
	return 0;
}

/* recalculate and set SS registers, disable SS if invalid */
static int set_ss_down(struct i2c_client *client, int chn) /* chn 0,1,2,3 */
{
	int rc;
	u32 ssud,ssup[3],ssdn[3];
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	/* calculate spread spectrum registers */
	if (((rc=calc_ss_down_to_regs(client, ssup, ssdn, &ssud, chn)))<0) return rc;
	if (rc!=0){
		 return disable_spread_spectrum(client,chn); /* SS parameters were never set*/
	}
	/* set spread spectrum registers */
	if (((rc=set_ss_regs(client, ssup, ssdn, &ssud, chn)))<0) return rc;
#if 0	
	/* enable spread spectrum mode */
	if (((rc=enable_spread_spectrum(client, chn)))<0) return rc;
#endif	
	return 0;
}

static int ss_pre_freq_change(struct i2c_client *client, int chn) /* chn 0,1,2,3 */
{
	int rc;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((clientdata->ss_on_freq_change & 1)==0) {
		dev_dbg(&client->dev, "Disabling spread spectrum before changing MS%d divider",chn);
		if (((rc=disable_spread_spectrum(client, chn)))<0) return rc;
	}
	return 0;
}

static int ss_post_freq_change(struct i2c_client *client, int chn) /* chn 0,1,2,3 */
{
	int rc, ss_state;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((ss_state=get_ss_state(client,chn)))<0) return ss_state;
	if (ss_state){
		/* recalculate and set SS registers */
		dev_dbg(&client->dev, "Recalculating spread spectrum after changing MS%d divider",chn);
		if (((rc=set_ss_down(client,chn)))<0) return rc;
		if (clientdata->ss_on_freq_change & 2) {
			reset_ms(client, 10);
		}
	}
	return 0;
}


static int calc_ss_down_to_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn) /* chn 0,1,2,3 */
{
	int rc;
	u32 ssud;
	u64 ms_freq, xy[2];
	u32 p123[3];
	u64 ms[3];
	u32 rate, amp;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	rate=clientdata->spread_spectrum_rate[chn]; /* in Hz */
	amp=clientdata->spread_spectrum_amp[chn]; /* in 0.01% */
	dev_dbg(&client->dev,"rate=%d, amp=%d\n",rate,amp);

	if ((rate==0) || (amp==0)) return 1; /* Should disable SS */
	
	if ((rate < SPREAD_RATE_MIN) || (rate > SPREAD_RATE_MAX)){
		dev_err(&client->dev, "*Invalid spread spectrum rate %u - should be in [%u,%u]Hz\n",rate,SPREAD_RATE_MIN,SPREAD_AMP_MAX);
		return - EINVAL;
	}
	if ((amp < SPREAD_AMP_MIN) || (amp > SPREAD_AMP_MAX)){
		dev_err(&client->dev, "*Invalid spread spectrum amplitude %u - should be in [%u,%u] (*0.01%%)\n",amp,SPREAD_AMP_MIN,SPREAD_AMP_MAX);
		return - EINVAL;
	}
	if (((rc=get_pll_ms_freq(client, &ms_freq, chn)))<0) return rc;
	if (ms_freq==0){
		dev_err(&client->dev, "MS%d frequency is not set, can not apply spread spectrum\n",chn);
		return - EINVAL;
	}
	if (((rc=get_ms_p123(client,p123, chn)))<0) return rc;
	p123_to_ms(ms,p123);
	ssud=(u32) div64_u64(ms_freq,rate << 2);
	if (updown_reg) updown_reg[0]= ssud;
	if (down_regs){
		xy[0]=6400000000LL*amp*(ms[0]*ms[2]+ms[1]);
		xy[0]=div64_u64(xy[0],ms[2]);
		xy[1]= 100000000LL*(10000-amp)*ssud;
		dev_dbg(&client->dev,"x=0x%llx, y=0x%llx,\n",xy[0],xy[1]);
		remove_common_factor(xy);
		dev_dbg(&client->dev,"x=0x%llx, y=0x%llx,\n",xy[0],xy[1]);
		down_regs[0]= (u32) div64_u64(xy[0],xy[1]);
		down_regs[2]= (u32)xy[1];
		down_regs[1]= (u32)xy[0] -  down_regs[2]*down_regs[0];
	}
	if (up_regs){
		up_regs[0]=0;
		up_regs[1]=1;
		up_regs[2]=0;
	}
	dev_dbg(&client->dev,"updown=0x%x, down[0]=0x%x, down[1]=0x%x, down[2]=0x%x, up[0]=0x%x, up[1]=0x%x, up[2]=0x%x\n",
			updown_reg[0],down_regs[0],down_regs[1],down_regs[2],up_regs[0],up_regs[1],up_regs[2]);
	return 0;
}

static int get_ss_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn) /* chn 0,1,2,3 */
{
	int i;
	s64 rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return (int) rc;
	if (up_regs) for (i=0;i<3;i++){
		if (((rc=read_multireg64 (client,  awe_msx_ssup[chn][i])))<0) return (int) rc;
		up_regs[i]= (u32) rc;
	}
	if (down_regs) for (i=0;i<3;i++){
		if (((rc=read_multireg64 (client,  awe_msx_ssdn[chn][i])))<0) return (int) rc;
		down_regs[i]= (u32) rc;
	}
	if (updown_reg) {
		if (((updown_reg[0]=read_multireg64 (client,  awe_msx_ssud[chn])))<0) return (int) updown_reg[0];
	}
	return 0;
}
static int set_ss_regs(struct i2c_client *client, u32 * up_regs, u32 * down_regs, u32 * updown_reg, int chn) /* chn 0,1,2,3, */
{
	int i,rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (up_regs){
		dev_dbg(&client->dev,"up[0]=0x%x, up[1]=0x%x, up[2]=0x%x\n",up_regs[0],up_regs[1],up_regs[2]);
		for (i=0;i<3;i++){
			if (((rc=write_multireg64 (client, (u64) up_regs[i], awe_msx_ssup[chn][i])))<0) return rc;
		}
	}
	if (down_regs) {
		dev_dbg(&client->dev,"down[0]=0x%x, down[1]=0x%x, down[2]=0x%x\n",down_regs[0],down_regs[1],down_regs[2]);
		for (i=0;i<3;i++){
			if (((rc=write_multireg64 (client, (u64) down_regs[i], awe_msx_ssdn[chn][i])))<0) return rc;
		}
	}
	if (updown_reg) {
		dev_dbg(&client->dev,"updown=0x%x\n",updown_reg[0]);
		if (((rc=write_multireg64 (client, (u64) updown_reg[0], awe_msx_ssud[chn])))<0) return rc;
	}
	return 0;
}

static int disable_spread_spectrum(struct i2c_client *client,int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	/* disable spread spectrum - only this register was changed to 0 from default 1 */
	if (((rc=write_multireg64(client, 0 , awe_msx_ssup[chn][2])))<0) return rc;
	if (((rc=set_ss_state(client, 0, chn)))<0) return rc;
	return 0;
}
static int enable_spread_spectrum(struct i2c_client *client,int chn)
{
	int rc;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((rc=set_ss_state(client, 1, chn)))<0) return rc;
	if (clientdata->ss_on_freq_change & 2) {
		if (((rc=reset_ms(client, 10)))<0) return rc; /* 10 - wait cycles */
	}
	return 0;
}

static int get_drv_powerdown(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_drv_powerdown[chn]);
}

static int set_drv_powerdown(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (typ) typ=1;
	return write_field (client, (u8) typ, awe_drv_powerdown[chn]);
}

static int get_drv_disable(struct i2c_client *client, int chn)
{
	int rc;
	if ((chn!=4) && (((rc=_verify_output_channel(client,chn)))<0)) return rc;
	return read_field (client, awe_drv_disable[chn]);
}

static int set_drv_disable(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if ((chn!=4) && (((rc=_verify_output_channel(client,chn)))<0)) return rc;
	if (typ) typ=1;
	return write_field (client, (u8) typ, awe_drv_disable[chn]);
}


static int get_drv_disabled_state(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_drv_dis_state[chn]);
}

static int set_drv_disabled_state(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((typ<0) || (typ>3)){
		dev_err(&client->dev, "Invalid disabled state %d. Only 0..3 are supported\n",typ);
		return - EINVAL;
	}
	return write_field (client, (u8) typ, awe_drv_dis_state[chn]);
}

static int get_drv_invert(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_drv_invert[chn]);
}

static int set_drv_invert(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((typ<0) || (typ>3)){
		dev_err(&client->dev, "Invalid invert drivers %d. Only 0..3 are supported\n",typ);
		return - EINVAL;
	}
	return write_field (client, (u8) typ, awe_drv_invert[chn]);
}

static int get_drv_type(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_drv_fmt[chn]);
}

static int set_drv_type(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((typ<0) || (typ>7)){
		dev_err(&client->dev, "Invalid output type %d. Only 0..7 are supported\n",typ);
		return - EINVAL;
	}
	return write_field (client, (u8) typ, awe_drv_fmt[chn]);
}
static int get_drv_vdd(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_drv_vddo[chn]);
}

static int set_drv_vdd(struct i2c_client *client, int vdd, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((vdd<0) || (vdd>7)){
		dev_err(&client->dev, "Invalid output type %d. Only 0..3 are supported\n",vdd);
		return - EINVAL;
	}
	return write_field (client, (u8) vdd, awe_drv_vddo[chn]);
}
static int get_drv_trim(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return (int) read_multireg64 (client, awe_drv_trim[chn]);
}

static int set_drv_trim_any(struct i2c_client *client, int trim, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((trim<0) || (trim>31)){
		dev_err(&client->dev, "Invalid output type %d. Only 0..31 are supported\n",trim);
		return - EINVAL;
	}
	return write_multireg64(client, trim, awe_drv_trim[chn]);
}

static int set_out_div(struct i2c_client *client, int div, int chn) /*chn =0..3 */
{
	int rc;
	u8 val;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	for (val=0;val<ARRAY_SIZE(out_div_values);val++) if (out_div_values[val]==div) {
		if (((rc=write_field (client, val,  awe_rdiv_k[chn] )))<0) return rc;
		return 0;
	}
	dev_err(&client->dev, "Invalid value for output divider: %d\n",div);
	return - EINVAL;
}

static int get_out_div(struct i2c_client *client, int chn) /*chn =0..3 */
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((rc=read_field(client, awe_rdiv_k[chn])))<0) return rc;
	if (rc>=ARRAY_SIZE(out_div_values)){
		dev_err(&client->dev, "Invalid value for output divider: %d\n",rc);
		return - EINVAL;
	}
	return out_div_values[rc];
}

static int set_out_div_by_frequency(struct i2c_client *client, u64* out_freq, int chn) /*chn =0..3 */
{
	int rc,i,idiv;
	u64 out_src_freq[3],div, div15, out_src_num,out_src_denom,out_num,out_denom;
	if (((rc=get_output_src_frequency(client, out_src_freq, chn)))<0) return rc;
	out_src_num=out_src_freq[0]*out_src_freq[2]+out_src_freq[1];
	out_src_denom=out_src_freq[2];
	out_num=out_freq[0]*out_freq[2]+out_freq[1];
	out_denom=out_freq[2];
	if (out_num==0){
		dev_err(&client->dev, "Zero output frequency for channel: %d\n",chn);
		return - EINVAL;
	}
	while ((out_src_denom>0x1000) || (((out_src_denom | out_src_num) &1)==0)){
		out_src_denom>>=1;
		out_src_num>>=  1;
	}
	while ((out_denom>0x1000) || (((out_denom | out_num) &1)==0)){
		out_denom>>=1;
		out_num>>=  1;
	}
	dev_dbg(&client->dev, "out_src_num=%lld, out_src_denom=%lld, out_num=%lld, out_denom=%lld, \n",
			out_src_num, out_src_denom, out_num, out_denom);
	out_src_num*=out_denom;
	out_src_denom*=out_num;
	div=div64_u64(out_src_num + (out_src_denom>>1), out_src_denom);
	div15=div+(div>>1);
	dev_dbg(&client->dev, "out_src_num*out_denom=%lld, out_src_denom*out_num=%lld, div=%lld, div15=%lld \n",
			out_src_num, out_src_denom, div, div15);
	if ((div15<1) || (div15>=64)) {
		dev_err(&client->dev, "Output divider (%d) out of 1..32 range for output %d \n",(int) div,chn);
		return - EINVAL;
	}
	idiv=(int) div15;
    for (i=5;i>=0;i--) if ((1<<i) & idiv){
    	return  set_out_div(client, (1<<i), chn);
    }
	return - EINVAL; /* should not happen */
}
static int get_out_frequency(struct i2c_client *client, u64* out_freq, int chn) /*chn =0..3 */
{
	int rc,div;
	u64 out_src_freq[3];
	if (((rc=get_output_src_frequency(client, out_src_freq, chn)))<0) return rc;
	if (((div=get_out_div(client, chn)))<0)return div;
	out_freq[1]=out_src_freq[0]*out_src_freq[2]+out_src_freq[1];
	out_freq[2]=out_src_freq[2]*div;
	out_freq[0]=div64_u64(out_freq[1],out_freq[2]);
	out_freq[1]-=out_freq[0]*out_freq[2];
	if (out_freq[1]==0) out_freq[2]=1;
	remove_common_factor(&out_freq[1]);
	return 0;
}
static int get_out_source(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_rdiv_in[chn]);
}

static int set_out_source(struct i2c_client *client, int chn, int src)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if ((src<0) || (src>7)){
		dev_err(&client->dev, "Invalid source %d. Only 0...7 are supported\n",src);
		return - EINVAL;
	}
	return write_field (client, (u8) src, awe_rdiv_in[chn]);
}

static int get_out_ms(struct i2c_client *client, int chn)
{
	int rc,out_src;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((out_src=get_out_source(client, chn)))<0) return out_src;
	switch (out_src){
	case 5: return 0;
	case 6: return chn;
	}
	return -1;
}

/* Examples:
 * "IN12:2:4"
 * "XO:1:1"
 * "MS0:16"
 * "NO"
 */
static int get_out_route(struct i2c_client *client, char* buf, int chn)
{
	int rc;
	int out_src,div1=-1,div2=-1,src_group, src;
	const int in_numbers[]={12,3,4,56};
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((out_src=get_out_source(client, chn)))<0) return out_src;
	if (((div2=get_out_div(client,chn)))<0) return div2; /* 1/2/4/8/16/32 */
	switch (out_src){
	case 0: /* p2div in */
	case 2: /* p2div out */
		if (out_src & 2) {
			if (((div1=get_in_pdiv(client,1)))<0) return div1; /* 1/2/4/8/16/32 */
		} else div1=1;
		if (((src=get_fb_mux(client)))<0) return src;
		src_group=0;
		src=(src)?2:3; /* mod src: 0 - IN56, 1 - IN4 */
		break;
	case 1: /* p1div in */
	case 3: /* p1div out */
		if (out_src & 2) {
			if (((div1=get_in_pdiv(client,0)))<0) return div1; /* 1/2/4/8/16/32 */
		} else div1=1;
		if (((src=get_in_mux(client)))<0) return src;
		if (src==2){
			src_group=1;
			src=0;
		} else {
			src_group=0; /* keep src: 0 - IN12, 1 - IN3 */
		}
		break;
	case 4: src_group=1; div1=1;  break;
	case 5: src_group=2; src=0;   break;
	case 6: src_group=2; src=chn; break;
	case 7: src_group=3;          break;
	}
	dev_dbg(&client->dev, "out_src=%d, src_group=%d, src=%d, div1=%d, div2=%d\n",out_src,src_group,src,div1,div2);
	switch (src_group) {
	case 0: return sprintf(buf,"IN%d:%d:%d",in_numbers[src],div1,div2);
	case 1: return sprintf(buf,"XO:%d:%d",div1,div2);
	case 2: return sprintf(buf,"MS%d:%d",src,div2);
	case 3: return sprintf(buf,"No clock");
	}
	return 0;
}

static int set_out_route(struct i2c_client *client, const char* route, int chn)
{
	int src_group, src, div1, div2, out_src, mux1=-1,mux2=-1;
	const char *cp=route;
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	/* parse string */
	if ((strncasecmp(cp,"in",2)==0)) {
		cp+=2;
		if ((strncmp(cp,"12",2)==0)) {
			cp+=2;
			src=0;
		} else if ((strncmp(cp,"3", 1)==0)) {
			cp+=1;
			src=1;
		} else if ((strncmp(cp,"4", 1)==0)) {
			cp+=1;
			src=2;
		} else if ((strncmp(cp,"56",2)==0)){
			cp+=2;
			src=3;
		} else return -EINVAL; /* invalid input number(s) */
		src_group=0;
	} else if ((strncasecmp(cp,"xo",2)==0)) {
		cp+=2;
		src_group=1;
		src=0;
	} else if ((strncasecmp(cp,"ms",2)==0)) {
		cp+=2;
		src=cp[0]-'0';
		cp++;
		if ((src!=0) && (src!=chn)){
			return -EINVAL; /* invalid MS channel */
		}
		src_group=2;
		div1=-1;
	} else if ((strncasecmp(cp,"no",2)==0)) {
		cp+=2;
		src_group=3;
	} else return -EINVAL;
	/* for IN and XO - find input divisor */
	if (src_group<2){
		if ((cp[0]=='/') || (cp[0]==':')){
			cp++;
			if      ((strncmp(cp,"32",2)==0)) {
				div1=5;
				cp++;
			} else if ((strncmp(cp,"16",2)==0)) {
				div1=4;
				cp++;
			} else if ((strncmp(cp,"8",1)==0)) div1=3;
			else if   ((strncmp(cp,"4",1)==0)) div1=2;
			else if   ((strncmp(cp,"2",1)==0)) div1=1;
			else if   ((strncmp(cp,"1",1)==0)) div1=0;
			else return -EINVAL;
			cp++;
		} else return -EINVAL; /* divisor expected */
	}
	/* get output divisor */

	if (src_group<3){ /* not 'no clock' */
		if ((cp[0]=='/') || (cp[0]==':')){
			cp++;
			if      ((strncmp(cp,"32",2)==0)) {
				div2=5;
				cp++;
			} else if ((strncmp(cp,"16",2)==0)) {
				div2=4;
				cp++;
			} else if ((strncmp(cp,"8",1)==0)) div2=3;
			else if   ((strncmp(cp,"4",1)==0)) div2=2;
			else if   ((strncmp(cp,"2",1)==0)) div2=1;
			else if   ((strncmp(cp,"1",1)==0)) div2=0;
			else return -EINVAL;
			cp++;
		} else return -EINVAL; /* divisor expected */
		/* apply output divisor*/
		if (((rc==set_out_div(client, 1<<div2, chn)))<0) return rc;
	}
	/* apply */
	if (src_group==3){
		out_src = 7; /* No clock */
	} else	if (src_group==2){
		out_src = (src==0)?5:6; /* MS0/MSn */
	} else	if ((src_group==1) && (div1==0))  { /* xo, no divisor */
		out_src = 4;
	} else {
		if (div1==0){
			out_src=(src>1)?0:1; /* p2div_in / p1div_in */
		} else {
			out_src=(src>1)?2:3; /* p2div_out / p1div_out */
			if (src_group==1){ /* xo, but with division */
				mux1=2;
			} else {
				switch (src){
				case 0: mux1=0; break; /* in1/in2 */
				case 1: mux1=1; break; /* in3 */
				case 2: mux2=1; break; /* in4 */
				case 3: mux2=0; break; /* in5/in6 */
				}
			}
		}
	}
	dev_dbg(&client->dev, "src_group=%d, src=%d, div1=%d, div2=%d, mux1=%d,mux2=%d, out_src=%d \n",src_group, src, div1, div2, mux1,mux2,out_src);
	if (((rc==set_out_source(client, chn,out_src)))<0) return rc;
	if (div1>0){ /* only set p1div/p2div if needed */
		if (((rc==set_in_pdiv(client, 1<<div1, (out_src==2)?1:0)))<0) return rc; /* set p2div or p1div - as needed */
	}
	if (mux1>=0){
		if (((rc==set_in_mux(client, mux1)))<0) return rc; /* set input mux if it is used */
	}
	if (mux2>=0){
		if (((rc==set_fb_mux(client, mux2)))<0) return rc; /* set fb mux if it is used */
	}
	return 0;
}


static int set_out_frequency_and_route (struct i2c_client *client, u64 *out_freq, int chn, int int_div)
{
	/* using MS with the same number as the output, enabling power to that MS */
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	/* setup MSn division */
	if (((rc=set_pll_ms_by_out(client, out_freq, chn, int_div)))<0) return rc;
	/* enable power for selected MS */
	if (((rc=set_ms_powerdown(client, 0, chn)))<0) return rc;
	/* route MSn to the output (6 - use 'own' MS) */
	if (((rc=set_out_source(client, chn, 6)))<0) return rc;
	/* setup output (R) division - by 1/2/4/8/16/32 */
	if (((rc=set_out_div_by_frequency(client, out_freq, chn)))<0) return rc;
	/* enable power for selected output */
	if (((rc=set_drv_powerdown(client, 0, chn)))<0) return rc;
	/* Note: output is not enabled ! Should it be not powered up too?*/
	return 0; /* all done */
}

static s64 get_output_src_frequency(struct i2c_client *client, u64 *out_freq, int chn)
{
	int mux;
	int div=1,rc=0;
	s64 freq[3]={0,0,1};
	if (((mux=get_out_source(client,chn)))<0) return mux;
	switch (mux){
	case 0:
		freq[0]= get_p2div_in_frequency(client);
		break;
	case 1:
		freq[0]= get_p1div_in_frequency(client);
		break;
	case 2:
		freq[0]= get_p2div_in_frequency(client);
		div=get_in_pdiv(client,1);
		break;
	case 3:
		freq[0]= get_p1div_in_frequency(client);
		div=get_in_pdiv(client,0);
		break;
	case 4:
		freq[0]= get_in_frequency (client,0); /* IN1/IN2, xtal */
		break;
	case 5:
		rc=get_pll_ms_freq(client, freq, 0); /* MS0 output */
		break;
	case 6:
		rc=get_pll_ms_freq(client, freq, chn); /* MS<n> output */
		break;
	case 7:
		freq[0]= 0; /* No clock */
		break;
	default:
		dev_err(&client->dev, "Invalid value for output source mux %d\n",mux);
		return - EINVAL;
	}
	if (rc<0)       return rc;
	if (freq[0]<0) return freq[0];
	if (div<0)  return div;
	out_freq[1]=freq[0]*freq[2]+freq[1];
	out_freq[2]=freq[2]*div;
	out_freq[0]=div64_u64(out_freq[1],out_freq[2]);
	out_freq[1]-=out_freq[0]*out_freq[2];
	if (out_freq[1]==0) out_freq[2]=1;
	remove_common_factor(&out_freq[1]);
	return 0;
}

/* -----------PLL section--------------------------- */
static u32 awe_fcal[]=     {AWE_FCAL_07_00,      AWE_FCAL_15_08,      AWE_FCAL_17_16,      0};
static u32 awe_fcal_ovrd[]={AWE_FCAL_OVRD_07_00, AWE_FCAL_OVRD_15_08, AWE_FCAL_OVRD_17_15, 0};
static int pre_init(struct i2c_client *client, int clear_all)
{
	int rc,chn;
	if (((rc=set_misc_registers(client)))<0) return rc; /* setup miscelalneous registers */
	if (((rc=write_field(client, 1,  AWE_OUT_ALL_DIS )))<0) return rc; /* disable all outputs */
	if (((rc=write_field(client, 1,  AWE_DIS_LOS     )))<0) return rc; /* pause LOL */
	if (clear_all){ /* clears outputs pll input/fb muxes to be set later */
		/* extra */
		for (chn=0;chn<4;chn++){
			if (((rc=disable_output(client, chn)))<0) return rc;
		}
		/* to be explicitly enabled if needed */
		if (((rc=disable_pll_in_fb_mux(client)))<0) return rc;
	}
	return 0;
}
static int post_init(struct i2c_client *client, int timeout) /*1 in timeout ~ 0.1ms - i2c read register */
{
	int rc,i,in_src, fb_src,ext_fb,check_los=0;
	s64 fcal;
	/* validate input clock status */
	if (((in_src=get_in_pfd_ref_fb(client,0)))<0) return in_src;
	switch (in_src){
	case 0:
	case 2:
	case 4:
		check_los |= AWE_STATUS_PLL_LOS_CLKIN; break;
	case 1:
	case 3:
		check_los |= AWE_STATUS_PLL_LOS_FDBK; break;
	}
	if (((ext_fb=read_field(client,AWE_PFD_EXTFB)))<0) return ext_fb;
	if (ext_fb){
		if (((fb_src=get_in_pfd_ref_fb(client,1)))<0) return fb_src;
		switch (in_src){
		case 1:
		case 3:
			check_los |= AWE_STATUS_PLL_LOS_CLKIN; break;
		case 0:
		case 2:
			check_los |= AWE_STATUS_PLL_LOS_FDBK; break;
		}
	}
	check_los &= 0xf;
	for (i=0;i<timeout;i++){
		if (((rc=get_status(client)))<0) return rc;
		if ((rc & check_los)==0) break; /* inputs OK */
	}
	if (i>=timeout){
		dev_err(&client->dev, "Timeout waiting for input clocks, status=0x%x, mask=0x%x\n",rc, check_los);
		return -EPIPE;
	}
	dev_dbg(&client->dev, "Validated input clocks, t=%d cycles (timeout= %d cycles), status =0x%x, mask=0x%x\n",
			i, timeout, rc, check_los);

	if (((rc=write_field(client, 0,  AWE_FCAL_OVRD_EN     )))<0) return rc; /* Configure PLL for locking, set FCAL_OVRD_EN=0 */
	write_field(client, 1,  AWE_SOFT_RESET       );      /* Configure PLL for locking, set SOFT_RESET=1 (ignore i2c error) */
	for (i=0;i<250;i++) get_status(client); /* wait 25 ms */
	if (((rc=write_field(client, 0x65,  AWE_REG241        )))<0) return rc; /* re-enable LOL, set reg 241=0x65 */

	check_los |= AWE_STATUS_PLL_LOL | AWE_STATUS_PLL_SYS_CAL;
	check_los &= 0xf;
	for (i=0;i<timeout;i++){
		if (((rc=get_status(client)))<0) return rc;
		if ((rc & check_los)==0) break; /* alarms not set OK */
	}
	if (i>=timeout){
		dev_err(&client->dev, "Timeout waiting for PLL lock, status=0x%x, mask=0x%x\n",rc, check_los);
		return -EPIPE;
	}
	dev_dbg(&client->dev, "Validated PLL locked, t=%d cycles (timeout= %d cycles), status =0x%x, mask=0x%x\n",
			i, timeout, rc, check_los);

	/* copy FCAL values to active registers */
	if (((fcal=read_multireg64 (client,       awe_fcal)))<0) return (int) fcal;
	if (((rc=  write_multireg64(client, fcal, awe_fcal_ovrd)))<0) return rc;
	dev_dbg(&client->dev, "Copied FCAL data 0x%llx\n", fcal);
	if (((rc=write_field(client, 5,  AWE_REG47_72     )))<0) return rc; /* set 47[7:2] to 000101b */
	if (((rc=write_field(client, 1,  AWE_FCAL_OVRD_EN     )))<0) return rc; /* SET PLL to use FCAL values, set FCAL_OVRD_EN=1 */
	/* only needed if using down-spread. Won't hurt to do anyway */
	if (((rc=reset_ms(client, 10)))<0) return rc;
	if (((rc=write_field(client, 0,  AWE_OUT_ALL_DIS )))<0) return rc; /* enable all (enabled individually) outputs */
	write_field(client, 0,  AWE_SOFT_RESET       );      /* Not documented - what to do with the soft reset bit - clearing */
	if (((rc=power_up_down_needed_ms(client)))<0) return rc;
	return 0;
}

static int reset_ms(struct i2c_client *client, int wait_cycles)
{
	int i,rc;
	dev_dbg(&client->dev, "Resetting MS dividers");
	if (((rc=write_field(client, 1,  AWE_MS_RESET      )))<0) return rc; /* SET MS RESET=1 */
	for (i=0;i<wait_cycles;i++) get_status(client); /* wait 1 ms */
	if (((rc=write_field(client, 0,  AWE_MS_RESET      )))<0) return rc; /* SET MS RESET=0 */
	return 0;
}

static int get_status(struct i2c_client *client)
{
	return read_field(client,AWE_STATUS);
}

static int power_up_down_needed_ms(struct i2c_client *client)
{
	int rc,chn,out_src;
	int ms_used=0;
	for (chn=0;chn<4;chn++){
		if (((out_src=get_out_source(client, chn)))<0) return out_src;
		switch (out_src){
		case 5: ms_used |= 1;   break;
		case 6: ms_used |= (1<<chn); break;
		}
	}
	for (chn=0;chn<4;chn++){
		if (((rc=set_ms_powerdown(client, (ms_used & (1<<chn))?0:1,  chn)))<0) return rc;
	}
	return 0;
}
static int disable_output(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (((rc=set_out_route(client, "no", chn)))<0) return rc;
	/* disable power for selected output */
	if (((rc=set_drv_powerdown(client, 1, chn)))<0) return rc;
	if (((rc=set_drv_disable(client, 1, chn)))<0) return rc;
	return 0;
}
/* to be explicitly enabled if needed */
static int disable_pll_in_fb_mux(struct i2c_client *client)
{
	int rc;
	if (((rc=set_in_pfd_ref_fb(client, 5, 0)))<0) return rc; /*chn =0 - ref, 1 - fb, '5' - noclk*/
	if (((rc=set_in_pfd_ref_fb(client, 5, 1)))<0) return rc; /*chn =0 - ref, 1 - fb, '5' - noclk*/
    return 0;
}


static int set_pll_paremeters(struct i2c_client *client)
{
	int rc;
	s64 pll_in_freq;
	u64 pll_out_freq[3];
	s64 K,Q,kphi_num,kphi_denom,fvco_mhz, fpfd_mhz;
	int rsel,bwsel,vco_gain,pll_kphi,mscal,ms_pec;
	if (((pll_in_freq=get_pll_in_frequency(client)))<0) return (int) pll_in_freq;
	if (((rc=get_pll_freq(client,pll_out_freq)))<0) return rc;
	fpfd_mhz = div64_u64(pll_in_freq,1000000ll);
	fvco_mhz = div64_u64(pll_out_freq[0],1000000ll);
	if (fpfd_mhz>=15){
		K=925;
		rsel=0;
		bwsel=0;
	} else if (fpfd_mhz>=8){
		K=325;
		rsel=1;
		bwsel=1;
	} else {
		K=185;
		rsel=3;
		bwsel=2;
	}
	if (fvco_mhz>2425){
		Q=3;
		vco_gain=0;
	} else {
		Q=4;
		vco_gain=1;
	}
	kphi_num=  K*2500LL*2500LL*2500LL;
	kphi_denom=533LL*Q*fpfd_mhz*fvco_mhz*fvco_mhz;
	pll_kphi=(int) div64_u64(kphi_num + (kphi_denom>>1),kphi_denom);
	if ((pll_kphi<1) || (pll_kphi>127)) {
		dev_err(&client->dev, "Calculated PLL_KPHI does not fit 1<=%d<=127\n",pll_kphi);
		if (pll_kphi<1) pll_kphi=1;
		else if (pll_kphi>127) pll_kphi=127;
	}
	mscal = (int) div64_u64(2067000-667*fvco_mhz+50000,100000ll);
	if ((mscal<0) || (mscal>63)) {
		dev_err(&client->dev, "Calculated MSCAL does not fit 0<=%d<=63\n",mscal);
		if (mscal<0) mscal=0;
		else if (mscal>63) mscal=63;
	}
	ms_pec = 7;
	dev_dbg(&client->dev, "Calculated values: PLL_KPHI=%d K=%lld RSEL=%d BWSEL=%d VCO_GAIN=%d MSCAL=%d MS_PEC=%d\n",
			pll_kphi, K, rsel, bwsel, vco_gain, mscal, ms_pec);
	/* setting actual registers */
	if (((rc=write_field(client, (u8) pll_kphi, AWE_PLL_KPHI)))<0) return rc;
	if (((rc=write_field(client, (u8) (((vco_gain & 7)<<4) | ((rsel & 3)<<2) | (bwsel & 3)),
			AWE_VCO_GAIN_RSEL_BWSEL)))<0) return rc;
	if (((rc=write_field(client, (u8) mscal,  AWE_MSCAL )))<0) return rc;
	if (((rc=write_field(client, (u8) ms_pec, AWE_MS_PEC)))<0) return rc;
	if (((rc=write_field(client, 3, AWE_PLL_EN)))<0) return rc; /* enable PLL */
	return 0;
}
/* verify the chip is initilaized - returns 0 if power-up state, 5 if initialized, -1 if i2c register can not be read */
static int is_set_up(struct i2c_client *client)
{
	return read_field(client,  AWE_MISC_47 );
}

static int set_misc_registers(struct i2c_client *client)
{
	/* ST52238 Reference Manual R1.2 p.28 */
	int rc;
	if (((rc=write_field(client, 0x5,  AWE_MISC_47 )))<0) return rc;
	if (((rc=write_field(client, 0x1,  AWE_MISC_106 )))<0) return rc;
	if (((rc=write_field(client, 0x1,  AWE_MISC_116 )))<0) return rc;
	if (((rc=write_field(client, 0x1,  AWE_MISC_42 )))<0) return rc;
	if (((rc=write_field(client, 0x0,  AWE_MISC_06A )))<0) return rc;
	if (((rc=write_field(client, 0x0,  AWE_MISC_06B )))<0) return rc;
	if (((rc=write_field(client, 0x0,  AWE_MISC_28 )))<0) return rc;
	return 0;
}



/* -----------MultiSynth section--------------------------- */
static u32 awe_msx[5][3][5]=
	{{{AWE_MS0_P1_07_00, AWE_MS0_P1_15_08, AWE_MS0_P1_17_16, 0, 0},
	  {AWE_MS0_P2_05_00, AWE_MS0_P2_13_06, AWE_MS0_P2_21_14, AWE_MS0_P2_29_22, 0},
	  {AWE_MS0_P3_07_00, AWE_MS0_P3_15_08, AWE_MS0_P3_23_16, AWE_MS0_P3_29_24, 0}},
	 {{AWE_MS1_P1_07_00, AWE_MS1_P1_15_08, AWE_MS1_P1_17_16, 0, 0},
	  {AWE_MS1_P2_05_00, AWE_MS1_P2_13_06, AWE_MS1_P2_21_14, AWE_MS1_P2_29_22, 0},
	  {AWE_MS1_P3_07_00, AWE_MS1_P3_15_08, AWE_MS1_P3_23_16, AWE_MS1_P3_29_24, 0}},
	 {{AWE_MS2_P1_07_00, AWE_MS2_P1_15_08, AWE_MS2_P1_17_16, 0, 0},
	  {AWE_MS2_P2_05_00, AWE_MS2_P2_13_06, AWE_MS2_P2_21_14, AWE_MS2_P2_29_22, 0},
	  {AWE_MS2_P3_07_00, AWE_MS2_P3_15_08, AWE_MS2_P3_23_16, AWE_MS2_P3_29_24, 0}},
	 {{AWE_MS3_P1_07_00, AWE_MS3_P1_15_08, AWE_MS3_P1_17_16, 0, 0},
	  {AWE_MS3_P2_05_00, AWE_MS3_P2_13_06, AWE_MS3_P2_21_14, AWE_MS3_P2_29_22, 0},
	  {AWE_MS3_P3_07_00, AWE_MS3_P3_15_08, AWE_MS3_P3_23_16, AWE_MS3_P3_29_24, 0}},
	 {{AWE_MSN_P1_07_00, AWE_MSN_P1_15_08, AWE_MSN_P1_17_16, 0, 0},
	  {AWE_MSN_P2_05_00, AWE_MSN_P2_13_06, AWE_MSN_P2_21_14, AWE_MSN_P2_29_22, 0},
	  {AWE_MSN_P3_07_00, AWE_MSN_P3_15_08, AWE_MSN_P3_23_16, AWE_MSN_P3_29_24, 0}}};


static const u32 awe_ms_powerdown[]={AWE_MS0_PDN, AWE_MS1_PDN, AWE_MS2_PDN, AWE_MS3_PDN};
static int get_ms_powerdown(struct i2c_client *client, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	return read_field (client, awe_ms_powerdown[chn]);
}

static int set_ms_powerdown(struct i2c_client *client, int typ, int chn)
{
	int rc;
	if (((rc=_verify_output_channel(client,chn)))<0) return rc;
	if (typ) typ=1;
	return write_field (client, (u8) typ, awe_ms_powerdown[chn]);
}

static int ms_to_p123(u64* ms,u32 * p123)
{
	/*
	 *  a=ms[0],b=ms[1],c=ms[2] ms~=a+b/c
	 * p1=floor(((a*c+b)*128)/c -512)
	 * p2=mod((b*128),c)
	 * p3=c
	 */
	u64 d;
	u64 ms_denom=ms[2], ms_num=ms[1], ms_int=ms[0];
	while ((ms_denom >= (1<<30))| (((ms_denom | ms_num) &1) == 0)) {
		ms_denom >>= 1;
		ms_num >>= 1;
	}
	if ((ms_num==0) || (ms_denom==0)){
		ms_denom = 1;
		ms_num = 0;
	}
	d= (ms_int * ms_denom + ms_num)<<7;
	p123[0]= (u32) (div64_u64(d,ms_denom) -512);
	d=div64_u64((ms_num<<7),ms_denom);
	p123[1]= (u32) ((ms_num<<7)-d*ms_denom);
	p123[2]=ms_denom;
	pr_debug("ms[]=%llu + %llu/%llu Hz, ms_int=%llu, ms_num=%llu, ms_denom=%llu p123=%u %u %u\n",
			ms[0],ms[1],ms[2],ms_int,ms_num,ms_denom,p123[0],p123[1],p123[2]);
	return 0;
}
static int p123_to_ms(u64* ms,u32 * p123)
{
	/* a=ms[0],b=ms[1],c=ms[2] ms~=a+b/c
	 * p1=floor(((a*c+b)*128)/c -512)
	 * p2=mod((b*128),c)
	 * p3=c
	 * ---
	 * b*128=k*c +p2; k<128, p2<c
	 * p1=floor(((a*c+b)*128)/c -512)=a*128 + floor((b*128)/c) -512 = a*128+ k -512
	 * k=mod (p1,128) =p1 & 0x7f
	 * c= p3
	 * b= (k*c + p2)/128= ((p1 & 0x7f)*p3 + p2) >>7
	 * a= (p1+512)>>7=(p1>>7)+4
	 *
	 */
	ms[2]=p123[2]; /* c= p3 */
	ms[1]=(ms[2]*(p123[0] & 0x7f) + p123[1]) >>7; /* b= (c*(p1 & 0x7f) + p2) >>7 */
	ms[0]=(p123[0]>>7)+4; /* a= (p1>>7)+4 */
	pr_debug("ms[]=%llu + %llu/%llu, p123=%u %u %u\n",
			ms[0],ms[1],ms[2],p123[0],p123[1],p123[2]);
	return 0;
}


static int get_ms_p123(struct i2c_client *client,u32 * p123, int chn) /* chn 0,1,2,3,4 (4 - msn) */
{
	int i;
	s64 rc;
	if ((chn<0) || (chn>4)){
		dev_err(&client->dev, "Invalid channel %d. Only 0,1,2,3 and 4 (for MSN) are supported\n",chn);
		return - EINVAL;
	}
	for (i=0;i<3;i++){
		if (((rc=read_multireg64 (client,  awe_msx[chn][i])))<0) return (int) rc;
		p123[i]= (u32) rc;
	}
	return 0;
}
static int set_ms_p123(struct i2c_client *client,u32 * p123, int chn) /* chn 0,1,2,3,4 (4 - msn) */
{
	int i,rc,hs;
	if ((chn<0) || (chn>4)){
		dev_err(&client->dev, "Invalid channel %d. Only 0,1,2,3 and 4 (for MSN) are supported\n",chn);
		return - EINVAL;
	}
	/* high speed bit programming */
	if (p123[0]<512){ /* div less than 8 */
		if (p123[0]<128) p123[0]=0;
		else p123[0]=256;
		p123[1]=0;
		p123[2]=1;
		dev_dbg(&client->dev, "Using high speed divisor option on ms%d",chn);
	} else hs=0;
	if (((rc=write_field(client, hs, awe_ms_hs[chn])))<0) return rc;
	/* optionally disable spread spectrum before changing frequency */
	if (chn<4){
		if (((rc=ss_pre_freq_change(client, chn)))<0) return rc;
	}
	for (i=0;i<3;i++){
		if (((rc=write_multireg64(client, (u64) p123[i], awe_msx[chn][i])))<0) return rc;
	}
	/* optionally enable spread spectrum after changing frequency, reset MS */
	if (chn<4){
		if (((rc=ss_post_freq_change(client, chn)))<0) return rc;
	}
	return 0;
}

/* Setting PLL frequency in 3 ways:
 * 1 - specified directly, allow fractional MSN
 * 2 - specified directly, integer MSN
 * 3 - specified by output frequency, allow fractional MSN (use PPL frequency closest to the middle)
 * 4 - specified by output frequency, integer MSN
 */
static int set_pll_freq(struct i2c_client *client, u64 *vco_freq, int int_div)
{
	s64 pll_in_freq, pll_in_freq_scaled,pll_out_freq_scaled,d;
	u32 msn_p123[3];
    u64 msn[]={0,0,1};
    s64 vco_int=vco_freq[0],vco_num=vco_freq[1],vco_denom=vco_freq[2];
    if ((vco_num==0) || (vco_denom==0)){
    	vco_num=0;
    	vco_denom=1;
    }
    if (vco_num>=vco_denom){ /* normalize */
		d=div64_u64(vco_num,vco_denom);
		vco_int+=d;
		vco_num-=d*vco_denom;
    }
	if (vco_int < FVCOMIN){
		dev_err(&client->dev, "Specified PLL frequency is too low: %llu < %llu\n",vco_int, FVCOMIN);
		return - EINVAL;
	}
	if (vco_int > FVCOMAX){
		dev_err(&client->dev, "Specified PLL frequency is too high: %llu > %llu\n",vco_int, FVCOMAX);
		return - EINVAL;
	}

	pll_in_freq=get_pll_in_frequency(client);
	if (pll_in_freq<0) return (int) pll_in_freq;
	pll_in_freq_scaled=pll_in_freq*vco_denom;
//	pll_out_freq_scaled=pll_out_freq*vco_denom;
	pll_out_freq_scaled=vco_int*vco_denom+vco_num;
	msn[0]=div64_u64(pll_out_freq_scaled,pll_in_freq_scaled);
	msn[1]=pll_out_freq_scaled-pll_in_freq_scaled*msn[0];
	msn[2]=pll_in_freq_scaled;
	while (msn[2] >= (1<<30)) { /* trim */
		msn[2] >>= 1;
		msn[1] >>= 1;
	}

	if (msn[0] < MSINT_MIN){
		dev_err(&client->dev, "Calculated MSN ratio is too low: %llu < %u\n",msn[0], MSINT_MIN);
		return - EINVAL;
	}
	if (msn[0] > MSINT_MAX){
		dev_err(&client->dev, "Calculated MSN ratio is too high: %llu > %u\n",msn[0], MSINT_MAX);
		return - EINVAL;
	}
	if (int_div){
		if (msn[1]>=(msn[2]>>1)) msn[0]++; // round
		msn[1] = 0;
		msn[2] = 1;
	} else {
		remove_common_factor(&msn[1]);
	}
	ms_to_p123(msn, msn_p123);
	return set_ms_p123(client,msn_p123, 4); // MSN
}
/* normalizes output */
static int get_pll_freq(struct i2c_client *client,u64 * pll_freq)
{
	int rc;
	s64 pll_in_freq;
	u32 p123[3];
	pll_in_freq=get_pll_in_frequency(client);
	if (pll_in_freq<=0) return (int) pll_in_freq; // return 0 if in frequency==0
	if (((rc=get_ms_p123(client,p123,4)))<0) return rc; /* channel4 - MSN */
	p123_to_ms(pll_freq,p123);
	if (pll_freq[2]<=0) return -EINVAL; /* 0 denominator */
	pll_freq[1] =(pll_freq[0]*pll_freq[2]+pll_freq[1])*pll_in_freq;
	pll_freq[0] =div64_u64(pll_freq[1],pll_freq[2]);
	pll_freq[1]-=pll_freq[0]*pll_freq[2];
	if (pll_freq[1]==0){
		pll_freq[2]=1;
	} else {
		remove_common_factor(&pll_freq[1]);
	}
	return 0;
}

/*
 *  Calculate pll output frequency to match specified output frequency
 *  out_freq as int,num,denom
 */
static int set_pll_freq_by_out(struct i2c_client *client, u64 *out_freq, int int_msn_div)
{
	/* use r-divider if the output frequency is too low (less than 5 MHz) */
	u64 out_int=out_freq[0],out_num=out_freq[1],out_denom=out_freq[2],
			pll_out_freq[3], scaled_max,scaled_min,d;
	s64 pll_freq_scaled, out_freq_scaled,err,best_err=-1,center_scaled,center_diff,best_center_diff,
			out_div,pll_in_freq,in_div,best_in_div, pll_in_freq_scaled,synth_out_scaled;
	int r_div=1;
	if (out_denom==0){
		dev_err(&client->dev, "denominator should not be 0 in %lld+%lld/%lld\n",
				out_int,out_num,out_denom);
		return -EINVAL;
	}
	out_freq_scaled=out_denom*out_int+out_num;
	scaled_max=(FVCOMAX/MSINT_MAX)*out_denom;
	while ((r_div < 32) && (out_freq_scaled<scaled_max)){
		out_freq_scaled<<=1;
		r_div<<=1;
	}
	if (out_freq_scaled<scaled_max){
		dev_err(&client->dev, "Specified output frequency is too low: %lld < %lld\n",
				out_freq[0], FVCOMAX/MSINT_MAX/32);
		return -EINVAL;

	}
	dev_dbg(&client->dev, "Output divider by %u, Output frequency before divider: %llu+%llu/%llu Hz\n",
			r_div, div64_u64(out_freq_scaled,out_denom),
			out_freq_scaled-out_denom*div64_u64(out_freq_scaled,out_denom),out_denom);
	scaled_max=FVCOMAX*out_denom;
	scaled_min=FVCOMIN*out_denom;
	if (int_msn_div==0){
		out_div=div64_u64( ((FVCOMAX+FVCOMIN)/2)*out_denom+(out_freq_scaled>>1),out_freq_scaled);
		dev_dbg(&client->dev, "out_div=%llu out_freq_scaled=%llu out_denom= %llu scaled_max=%llu scaled_min=%lld\n",
				out_div, out_freq_scaled, out_denom,scaled_max, scaled_min);
		if ((out_div==7) || (out_div==5) || (out_div==3)){
			if (out_freq_scaled*(out_div+1)<scaled_max){
				out_div++;
			} else if ((out_div>4) && (out_freq_scaled*(out_div-1)>scaled_min)){
				out_div--;
			} else {
				out_div=0;
			}
		}
		dev_dbg(&client->dev, "modified out_div=%lld\n",	out_div);
		if ((out_div<4) || (out_div > MSINT_MAX) ||
		    (out_freq_scaled*out_div < scaled_min) ||
		    (out_freq_scaled*out_div > scaled_max)){
			dev_err(&client->dev, "Can not find suitable divisor for output frequency %lld+%lld/%lld Hz\n",
					div64_u64(out_freq_scaled,out_denom),
								out_freq_scaled-out_denom*div64_u64(out_freq_scaled,out_denom),out_denom);
			return -EINVAL;
		}
		pll_out_freq[0]=div64_u64(out_freq_scaled*out_div,out_denom);
		pll_out_freq[1]=(out_freq_scaled*out_div)-pll_out_freq[0]*out_denom;
		pll_out_freq[2]=out_denom;
		dev_dbg(&client->dev, "PLL output divider by %llu, pll frequency: %llu+%llu/%llu Hz\n",
				out_div,pll_out_freq[0],pll_out_freq[1],pll_out_freq[2]);
		return set_pll_freq(client, pll_out_freq, 0);
	} else { /* if (int_msn_div==0), find the best pair of integer coefficients, try closest to the center, if possible */
		pll_in_freq=get_pll_in_frequency(client);
		pll_in_freq_scaled=pll_in_freq*out_denom;
		center_scaled=((FVCOMAX+FVCOMIN)>>1)*out_denom;
		if (pll_in_freq<0) return (int) pll_in_freq;
		best_in_div=0;
		for (out_div=4;out_div<=MSINT_MAX;out_div++) if ((out_div!=5) && (out_div!=7)){
			pll_freq_scaled=out_freq_scaled*out_div; /* here scaled by denominator */
			if ((pll_freq_scaled>=scaled_min) && (pll_freq_scaled<=scaled_max)) {
				in_div=div64_u64(pll_freq_scaled+(pll_in_freq_scaled>>1),pll_in_freq_scaled); // round
				d=pll_in_freq_scaled*in_div; /* actual pll frequency scaled by out_denom */
				synth_out_scaled=div64_u64(d + (out_div>>1),out_div);
				center_diff=d-center_scaled;
				if (center_diff<0) center_diff=-center_diff;
				err=synth_out_scaled-out_freq_scaled;
				if (err<0) err=-err;
				if ((best_in_div==0) || (err < best_err) || ((err == best_err) && (center_diff<best_center_diff))){
					dev_dbg(&client->dev, "synth_out_scaled: %lld center_scaled: %lld out_freq_scaled:%lld err: %lld (%lld) center_diff:%lld(%lld)\n",
							synth_out_scaled, center_scaled, out_freq_scaled,err,best_err,center_diff,best_center_diff);
					best_err=err;
					best_in_div=in_div;
					best_center_diff=center_diff;
				}
			}
		}
		if (best_in_div==0){
			dev_err(&client->dev, "Failed to find suitable integer coefficients for pll input %lld Hz\n",
					pll_in_freq);

		}
		pll_out_freq[0]=div64_u64(pll_in_freq_scaled*best_in_div,out_denom);
		pll_out_freq[1]=(pll_in_freq_scaled*best_in_div)-pll_out_freq[0]*out_denom;
		pll_out_freq[2]=out_denom;
		dev_dbg(&client->dev, "PLL output frequency: %llu+%llu/%llu Hz, MS input divider: %lld, MS output divider: %lld\n",
				pll_out_freq[0],pll_out_freq[1],pll_out_freq[2], best_in_div, out_div);
		return set_pll_freq(client, pll_out_freq, 1); /* integer result */
	}
}

static int get_pll_ms_freq(struct i2c_client *client, u64 *out_freq, int chn)
{
	int rc;
	u64 pll_out_freq[3], ms[3], pll_freq_scaled, ms_scaled;
	u32 p123[3];
	if (((rc=get_pll_freq(client,pll_out_freq)))<0) return rc;
	/* trim PLL frequency fraction */
	while (pll_out_freq[2]>=0x1000){
		pll_out_freq[1] >>= 1;
		pll_out_freq[2] >>= 1;
	}
	pll_freq_scaled=pll_out_freq[0]*pll_out_freq[2]+pll_out_freq[1];

	if (((rc=get_ms_p123(client,p123, chn)))<0) return rc; /* includes invalid chn */
	p123_to_ms(ms,p123);
	/* trim MS divisor fraction */
	while (ms[2]>=0x1000){
		ms[1] >>= 1;
		ms[2] >>= 1;
	}
	ms_scaled=ms[0]*ms[2]+ms[1];
	out_freq[1]=pll_freq_scaled*ms[2];
	out_freq[2]=ms_scaled*pll_out_freq[2];
	if (out_freq[2]==0){
		out_freq[0]=0;
		out_freq[1]=0;
		out_freq[2]=1;
	} else {
		out_freq[0]=div64_u64(out_freq[1],out_freq[2]);
		out_freq[1]-=out_freq[0]*out_freq[2];
		remove_common_factor(&out_freq[1]);
	}
	dev_dbg(&client->dev, "MS%d output frequency: %llu+%llu/%llu Hz\n",chn,out_freq[0],out_freq[1],out_freq[2]);
	return 0;
}

/*
 *  Adjust MultiSynth divisor (MS0..MS3) for specified output frequency
 *  MSN, input frequency should be already set
 *  out_freq as int,num,denom
 */
static int set_pll_ms_by_out(struct i2c_client *client, u64 *out_freq, int chn, int int_div)
{
	/* use r-divider if the output frequency is too low (less than 5 MHz) */
	u64 out_int=out_freq[0],out_num=out_freq[1],out_denom=out_freq[2],
			pll_out_freq[3],d;
	s64 pll_freq_scaled, out_freq_scaled;
	u64 ms[3];
	u32 p123[3];
	int r_div=1,rc;
	if (out_denom==0){
		dev_err(&client->dev, "denominator should not be 0 in %lld+%lld/%lld\n",
				out_int,out_num,out_denom);
		return -EINVAL;
	}
	if (out_num>=out_denom){ /* normalize */
		d=div64_u64(out_num,out_denom);
		out_int+=d;
		out_num-=d*out_denom;
	}
	if (out_num==0){
		out_denom=1;
	}
	if (out_int<(FVCOMAX/MSINT_MAX)){
		while ((r_div < 32) && (out_int<(FVCOMAX/MSINT_MAX))){
			out_int<<=1;
			out_num<<=1;
			r_div<<=1;
			if (out_num>out_denom) {
				out_int++;
				out_num-=out_denom;
			}

		}
		if (out_int<(FVCOMAX/MSINT_MAX)){
			dev_err(&client->dev, "Specified output frequency is too low: %lld < %lld\n",
					out_freq[0], FVCOMAX/MSINT_MAX/32);
			return -EINVAL;
		}
	}
	dev_dbg(&client->dev, "Output divider by %u, Output frequency before divider: %llu+%llu/%llu Hz\n",
			r_div,out_int, out_num,out_denom);
	/* trim output frequency fraction */
	while (out_denom>=0x1000){
		out_num >>= 1;
		out_denom >>= 1;
	}
	out_freq_scaled=out_int*out_denom+out_num;
	if (((rc=get_pll_freq(client,pll_out_freq)))<0) return rc;
	/* trim PLL frequency fraction */
	while (pll_out_freq[2]>=0x1000){
		pll_out_freq[1] >>= 1;
		pll_out_freq[2] >>= 1;
	}
	pll_freq_scaled=pll_out_freq[0]*pll_out_freq[2]+pll_out_freq[1];
	ms[1]=pll_freq_scaled*out_denom;
	ms[2]=out_freq_scaled*pll_out_freq[2];
	ms[0]=div64_u64(ms[1],ms[2]);
	ms[1]-=ms[0]*ms[2];
	if (int_div){
		if (ms[1]>(ms[2]>>1)) ms[0]++;
		ms[1]=0;
		ms[2]=1;
	} else {
		remove_common_factor(&ms[1]);
	}
	dev_dbg(&client->dev, "MS%d divider: %llu+%llu/%llu\n",chn,ms[0],ms[1],ms[2]);
	/* set up registers */
	ms_to_p123(ms,p123);
	if (((rc=set_ms_p123(client,p123, chn)))<0) return rc;
	if (((rc=disable_spread_spectrum(client,chn)))<0) return rc;
	return 0;
}
/* ----------- Input section ----------------- */

static s64 get_pll_in_frequency(struct i2c_client *client)
{
	int mux;
	int div=1;
	s64 freq;
	if (((mux=get_in_pfd_ref_fb(client,0)))<0) return mux;
	switch (mux){
	case 0:
		freq= get_p1div_in_frequency(client);
		break;
	case 1:
		freq= get_p2div_in_frequency(client);
		break;
	case 2:
		freq= get_p1div_in_frequency(client);
		div=get_in_pdiv(client,0);
		break;
	case 3:
		freq= get_p2div_in_frequency(client);
		div=get_in_pdiv(client,1);
		break;
	case 4:
		freq= get_in_frequency (client,0); /* IN1/IN2, xtal */
		break;
	case 5:
		freq= 0; /* No clock */
		break;
	default:
		dev_err(&client->dev, "Invalid value for PLL input multiplexer %d\n",mux);
		return - EINVAL;
	}
	if (freq<0) return freq;
	if (div<0)  return div;
	/* TODO - make it fractional? */
	return div64_u64(freq,div);
}
static s64 get_pll_fb_frequency(struct i2c_client *client)
{
	int mux;
	int div=1;
	s64 freq;
	if (((mux=get_in_pfd_ref_fb(client,1)))<0) return mux;
	switch (mux){
	case 0:
		freq= get_p2div_in_frequency(client);
		break;
	case 1:
		freq= get_p1div_in_frequency(client);
		break;
	case 2:
		freq= get_p2div_in_frequency(client);
		div=get_in_pdiv(client,1);
		break;
	case 3:
		freq= get_p1div_in_frequency(client);
		div=get_in_pdiv(client,0);
		break;
/*	case 4: */
	case 5:
		freq= 0; /* No clock */
		break;
	default:
		dev_err(&client->dev, "Invalid value for PLL feedback multiplexer %d\n",mux);
		return - EINVAL;
	}
	if (freq<0) return freq;
	if (div<0)  return div;
	/* TODO - make it fractional? */
	return div64_u64(freq,div);
}

static s64 get_p1div_in_frequency(struct i2c_client *client)
{
	int mux;
	if (((mux=   get_in_mux(client)))<0) return mux;
	switch (mux){
	case 0: return get_in_frequency (client,0); /* IN1/IN2 */
	case 1: return get_in_frequency (client,1); /* IN3 */
	case 2: return get_in_frequency (client,0); /* IN1/IN2 - xtal*/
	default:
		dev_err(&client->dev, "Invalid value for input multiplexer %d\n",mux);
		return - EINVAL;
	}
}
static s64 get_p2div_in_frequency(struct i2c_client *client)
{
	int mux;
	if (((mux=   get_fb_mux(client)))<0) return mux;
	switch (mux){
	case 0: return get_in_frequency (client,3); /* IN5/IN6 */
	case 1: return get_in_frequency (client,1); /* IN4 */
	case 2: return 0; /* no clock */
	default:
		dev_err(&client->dev, "Invalid value for input multiplexer %d\n",mux);
		return - EINVAL;
	}
}

static int set_in_mux(struct i2c_client *client, int data)
{
	int data1,rc;
	switch (data) {
	case 0: data1=0; break;
	case 1: data1=2; break;
	case 2: data1=5; break;
	default:
		dev_err(&client->dev, "Invalid value for input multiplexer %d\n",data);
		return - EINVAL;
	}
	if (((rc=write_field (client, data,  AWE_IN_MUX )))<0) return rc;
	if (((rc=write_field (client, data1, AWE_IN_MUX1)))<0) return rc;
	return 0;
}

static int get_in_mux(struct i2c_client *client)
{
	return read_field(client,AWE_IN_MUX );
}

static int set_fb_mux(struct i2c_client *client, int data)
{
	int data1,rc;
	switch (data) {
	case 0: data1=0; break;
	case 1: data1=1; break;
	case 2: data1=0; break;
	default:
		dev_err(&client->dev, "Invalid value for feedback multiplexer %d\n",data);
		return - EINVAL;
	}
	if (((rc=write_field (client, data,  AWE_FB_MUX )))<0) return rc;
	if (((rc=write_field (client, data1, AWE_FB_MUX1)))<0) return rc;
	return 0;
}

static int get_fb_mux(struct i2c_client *client)
{
	return read_field(client,AWE_FB_MUX );
}



static const u8 in_div_values[]={1,2,4,8,16,32};
static int set_in_pdiv(struct i2c_client *client, int div, int chn) /*chn =0,1 */
{
	int rc;
	u8 val;
	for (val=0;val<ARRAY_SIZE(in_div_values);val++) if (in_div_values[val]==div) {
		if (((rc=write_field (client, val,  chn?AWE_P2DIV:AWE_P1DIV )))<0) return rc;
		return 0;
	}
	dev_err(&client->dev, "Invalid value for input divider: %d\n",div);
	return - EINVAL;
}

static int get_in_pdiv(struct i2c_client *client, int chn) /*chn =0,1 */
{
	int rc;
	if (((rc=read_field(client, chn?AWE_P2DIV:AWE_P1DIV )))<0) return rc;
	if (rc>=ARRAY_SIZE(in_div_values)){
		dev_err(&client->dev, "Invalid value for input divider: %d\n",rc);
		return - EINVAL;
	}
	return in_div_values[rc];
}

static int set_in_pfd_ref_fb(struct i2c_client *client, u8 val, int chn) /*chn =0 - ref, 1 - fb*/
{
	int rc;
	if (val>5) {
		dev_err(&client->dev, "Invalid value for input pfd selector: %d\n", (int) val);
		return - EINVAL;
	}
	if (((rc=write_field (client, val,  chn?AWE_PFD_FB:AWE_PFD_REF )))<0) return rc;
	return 0;
}

static int get_in_pfd_ref_fb(struct i2c_client *client, int chn) /*chn =0,1 */
{
	return read_field(client, chn?AWE_PFD_FB:AWE_PFD_REF );
}

static int set_fb_external(struct i2c_client *client, u8 val)
{
	int rc;
	if (((rc=write_field (client, val?1:0,  AWE_PFD_EXTFB)))<0) return rc;
	return 0;
}
static int get_fb_external(struct i2c_client *client)
{
	return read_field(client,AWE_PFD_EXTFB);
}

static int set_in_frequency(struct i2c_client *client, u64 frequency,int src) /* 0 - 12, 1 - 3, 2 - 4, 3 - 5,6, 4 - 12XO */
{
	int xtal_mode,rc,idiv;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (frequency < INFREQMIN){
		dev_err(&client->dev, "Input frequency too low: %llu < %llu\n",frequency, INFREQMIN);
		return - EINVAL;
	}
	if (frequency > INFREQMAX){
		dev_err(&client->dev, "Input frequency too high: %llu > %llu\n",frequency, INFREQMAX);
		return - EINVAL;
	}
	for (idiv=0;idiv<5;idiv++) if ((frequency >> idiv) <= INFREQDIV) break;

	switch (src){
	case 4: /* set xtal mode */
		xtal_mode=0;
		if (frequency>11000000ll) xtal_mode=1;
		if (frequency>19000000ll) xtal_mode=2;
		if (frequency>26000000ll) xtal_mode=3;
		if (((rc=write_field (client, xtal_mode, AWE_XTAL_FREQ)))<0) return rc;
		if (((rc=set_in_mux(client, 2)))<0) return rc;  /* in mux to XO */
		if (((rc=set_in_pfd_ref_fb(client, 4, 0)))<0) return rc; /* set pfd reference to XO  - may use 0 (p1div_in also? )*/
		clientdata->input_frequency12=frequency;
		break;
	case 0:
		if (((rc=set_in_mux(client, 0)))<0) return rc;  /* in mux to IN12 */
		if (idiv==0){
			if (((rc=set_in_pfd_ref_fb(client, 0, 0)))<0) return rc; /* set pfd reference to p1div_in */
		} else {
			if (((rc=set_in_pfd_ref_fb(client, 2, 0)))<0) return rc; /* set pfd reference to p1div_out */
			if (((rc=set_in_pdiv(client, 1<<idiv, 0)))<0) return rc; /* p1div */
		}
		clientdata->input_frequency12=frequency;
		break;
	case 1:
		if (((rc=set_in_mux(client, 1)))<0) return rc;  /* in mux to IN3 */
		if (idiv==0){
			if (((rc=set_in_pfd_ref_fb(client, 0, 0)))<0) return rc; /* set pfd reference to p1div_in */
		} else {
			if (((rc=set_in_pfd_ref_fb(client, 2, 0)))<0) return rc; /* set pfd reference to p1div_out */
			if (((rc=set_in_pdiv(client, 1<<idiv, 0)))<0) return rc; /* p1div */
		}
		clientdata->input_frequency3=frequency;
		break;
	case 2:
		if (((rc=set_fb_mux(client, 1)))<0) return rc;  /* fb mux to IN4 */
		if (idiv==0){
			if (((rc=set_in_pfd_ref_fb(client, 1, 0)))<0) return rc; /* set pfd reference to p2div_in */
		} else {
			if (((rc=set_in_pfd_ref_fb(client, 3, 0)))<0) return rc; /* set pfd reference to p2div_out */
			if (((rc=set_in_pdiv(client, 1<<idiv, 1)))<0) return rc; /* p2div */
		}
		clientdata->input_frequency4=frequency;
		break;
	case 3:
		if (((rc=set_fb_mux(client, 0)))<0) return rc;  /* fb mux to IN5/6 */
		if (idiv==0){
			if (((rc=set_in_pfd_ref_fb(client, 1, 0)))<0) return rc; /* set pfd reference to p2div_in */
		} else {
			if (((rc=set_in_pfd_ref_fb(client, 3, 0)))<0) return rc; /* set pfd reference to p2div_out */
			if (((rc=set_in_pdiv(client, 1<<idiv, 1)))<0) return rc; /* p2div */
		}
		clientdata->input_frequency56=frequency;
		break;
	default:
		dev_err(&client->dev, "Invalid source %d only 0 (IN1/2), 1 (IN3), 2 (IN4), 3 (IN4/IN5) and 4 (IN1/2, XO) are supported\n",src);
		return - EINVAL;
	}
	return 0;
}

static u64 get_in_frequency(struct i2c_client *client, int src)
{
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	switch (src){
	case 0:	return clientdata->input_frequency12;
	case 1:	return clientdata->input_frequency3;
	case 2:	return clientdata->input_frequency4;
	case 3:	return clientdata->input_frequency56;
	default:
		dev_err(&client->dev, "Invalid source %d only 0 (IN1/2), 1 (IN3), 2 (IN4) and 3 (IN4/IN5) are supported\n",src);
		return - EINVAL;
	}
}

/* ----------- General ----------------- */

static s64 read_multireg64 (struct i2c_client *client, const u32 * awe)
{
	int i,nshift,nbits, full_shift=0;
	u8 mask;
	u16 reg;
	s64 data=0, rc;
	for (i=0;awe[i]!=0;i++){
		reg=awe[i]>>8;
		mask=awe[i]&0xff;
		if (mask!=0){
			nshift=0;
			nbits=1;
			while (((1<<nshift) & mask)==0) nshift++;
			while (((1<<(nshift+nbits)) & mask)!=0) nbits++;
			if (((rc=read_reg(client, reg)))<0) return rc;
			rc &=  mask;
			rc >>= nshift;
			rc <<= full_shift;
			data |= rc;
			full_shift+=nbits;
		}
	}
	return data;
}


static int write_multireg64 (struct i2c_client *client, u64 data, const u32 * awe)
{
	int i,rc,nshift,nbits;
	u8 mask,reg_data;
	u16 reg;
	for (i=0;awe[i]!=0;i++){
		reg=awe[i]>>8;
		mask=awe[i]&0xff;
		if (mask!=0){
			nshift=0;
			nbits=1;
			while (((1<<nshift) & mask)==0) nshift++;
			while (((1<<(nshift+nbits)) & mask)!=0) nbits++;
			reg_data=(data & 0xff) << nshift; /* may have some garbage in high bits, will be cut of by mask */
			data >>= nbits;
			if (((rc=write_reg(client, reg, reg_data, mask)))<0) return rc;
		}
	}
	return 0;
}

static int read_field (struct i2c_client *client, u32 awe)
{
	int rc,nshift;
	u8 mask;
	u16 reg;
	reg=awe>>8;
	mask=awe&0xff;
	if (mask!=0){
		nshift=0;
		while (((1<<nshift) & mask)==0) nshift++;
		if (((rc=read_reg(client, reg)))<0) return rc;
		return (rc & mask) >> nshift;
	}
	return 0;
}


static int write_field (struct i2c_client *client, u8 data, u32 awe)
{
	int rc,nshift;
	u8 mask,reg_data;
	u16 reg;
	reg=awe>>8;
	mask=awe&0xff;
	if (mask!=0){
		nshift=0;
		while (((1<<nshift) & mask)==0) nshift++;
		reg_data=(data & 0xff) << nshift;
		if (((rc=write_reg(client, reg, reg_data, mask)))<0) return rc;
	}
	return 0;
}


static int write_adwe(struct i2c_client *client, u32 adwe)
{
	u8 we=   adwe & 0xff;
	u8 data= (adwe>>8) & 0xff;
	u16 reg=  (adwe>>16) & (0xff | (REG5338_PAGE_MASK << 8)); /* 0x1ff */
	return write_reg(client, reg, data, we);
}

static int _write_single_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	int ireg=reg;
	dev_dbg(&client->dev,"device write: slave=0x%x, reg=0x%x, val=0x%x\n", (int) (client->addr),reg,val);
	if (clientdata) {
		if (reg==REG5338_PAGE) {
			//		dev_dbg(&client->dev,"changing page: new=0x%x, was=0x%x\n",val & REG5338_PAGE_MASK,clientdata->last_page);
			clientdata->last_page=val & REG5338_PAGE_MASK;
		} else {
			ireg |=(clientdata->last_page)<<8;
		}
		if (ireg<=LAST_REG){
			clientdata->cache[ireg].data= val;
			clientdata->cache[ireg].flags |= CACHE_INIT;
		}
	}
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int write_reg(struct i2c_client *client, u16 reg, u8 val, u8 mask)
{
	int rc,page;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (mask==0) return 0;
	dev_dbg(&client->dev,"reg=0x%x, val=0x%x, mask=0x%x\n", (int) reg, (int) val, (int) mask);
	if (mask !=0xff){
	    if (((rc=read_reg(client, reg)))<0) return rc; /* full register including page */
	    val=((val ^ rc) & mask)^ rc;
	    if ((val==rc) && !(clientdata->cache[reg].flags & CACHE_VOLAT)) {
	    	dev_dbg(&client->dev,"No change and not volatile -> no write\n");
	    	return 0;
	    }
	}
	page=(reg >> 8) & REG5338_PAGE_MASK;
	if (page != (clientdata->last_page)) { /* set page if needed */
		if (((rc=_write_single_reg(client, REG5338_PAGE, page)))<0) return rc;
	}
	return  _write_single_reg(client, reg & 0xff, val);
}

static int read_reg(struct i2c_client *client, u16 reg)
{
	int rc,page;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	if (clientdata && (reg<=LAST_REG) && (clientdata->cache[reg].flags & CACHE_INIT) && !(clientdata->cache[reg].flags & CACHE_VOLAT)){
		dev_dbg(&client->dev,"Using cached register: reg=0x%x -> 0x%x\n",reg,(int) clientdata->cache[reg].data);
		return clientdata->cache[reg].data;
	}
	page=(reg >> 8) & REG5338_PAGE_MASK;
//	dev_dbg(&client->dev,"reading i2c device : slave=0x%x, reg=0x%x page=0x%x, last_page=0x%x\n",(int) (client->addr),reg,page,clientdata->last_page);
	if (clientdata && (reg!=REG5338_PAGE) && (page != clientdata->last_page)) { /* set page if needed */
		if (((rc=_write_single_reg(client, REG5338_PAGE, page)))<0) return rc;
	}
	rc= i2c_smbus_read_byte_data(client, reg & 0xff);
	dev_dbg(&client->dev,"reading i2c device : slave=0x%x, reg=0x%x -> 0x%x\n",(int) (client->addr),reg,rc);
	if (rc<0) return rc;
	if (clientdata && (reg==REG5338_PAGE)) {
		clientdata->last_page= rc & REG5338_PAGE_MASK;
	}
	if (clientdata && (reg<=LAST_REG)){
		clientdata->cache[reg].data= (u8) rc;
		clientdata->cache[reg].flags |= CACHE_INIT;
	}
	return rc;
}

static void si5338_init_of(struct i2c_client *client)
{
	//	struct device *dev=&client->dev;
	const __be32 * config_data;
    const char *   init_type_string;
    int init_type=0; /* 0 - none, 1 - always, 2 - if not running (TODO) */
	struct device_node *node = client->dev.of_node;
	int len,i,n;
	u16 page_reg;
	char buf[40];
	u64 freq[3];
	u32 rate,amp;
	struct si5338_setup_data  {
		u8		page;
		u8		reg;
		u8		data;
		u8		mask;
	};
	struct si5338_setup_data setup_data;
	__be32 * setup_data_be32= (__be32 *) &setup_data;
	if (node) {
		init_type_string = of_get_property(client->dev.of_node, "si5338,init", &len);
		if (init_type_string){
			if      (strcmp(init_type_string,"always")==0) init_type=1;
			else if (strcmp(init_type_string,"if off")==0) init_type=2;
			else {
				dev_err(&client->dev,"Unrecognized si5338 initialization type '%s'. Only 'always' and 'if off' are permitted\n",init_type_string);
			}
		}
		switch (init_type){
		case 2:
			// static int is_set_up(struct i2c_client *client);
			i=is_set_up(client);
			if (i<0){
				dev_err(&client->dev,"Error reading i2c register, aborting initialization\n");
				return;
			} else if (i>0){
				init_type=0;
				dev_dbg(&client->dev,"Skipping conditional initialization (some driver variables will not be initialized)\n");
				return;
			}
			init_type=1;
			/* falling to initialization */
		case 1:
			pre_init(client,1); // clear outputs and muxes - they will be programmed later
			break;
		}

		config_data = of_get_property(client->dev.of_node, "si5338,configuration_data", &len);
		if (config_data){
			len /= sizeof(*config_data);
			dev_dbg(&client->dev,"Read %d values\n",len);
			dev_dbg(&client->dev,"Found %d items in 'si5338,configuration_data' in the Device Tree\n",len);
			for (i=0;i<len;i++){
				*setup_data_be32=config_data[i];
				page_reg=setup_data.reg+(setup_data.page<<8);
				dev_dbg(&client->dev,"page_reg=0x%03x, data=0x%02x, mask=0x%02x \n",
						(int) page_reg,(int)setup_data.data,(int)setup_data.mask);
				if (write_reg(client, page_reg, setup_data.data, setup_data.mask)<0) return;
			}
		}
		/* input section */
		/* setting input frequency here divides (if needed) and feeds it to the PLL reference. Other variants can use raw register writes */
		for (n=0;in_freq_names[n];n++){
			sprintf(buf,"si5338,%s",in_freq_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				dev_dbg(&client->dev,"Found '%s', value = %d (0x%x)\n",buf,(int)(be32_to_cpup(config_data)),(int)(be32_to_cpup(config_data)));
				if (set_in_frequency(client, be32_to_cpup(config_data),n)<0) return; /* 32 bits are sufficient here */
			}
		}
		/* setting PLL for the most important output frequency, sets analog parameters accordingly. Assumes input frequency set above */
		for (n=0;pll_setup_names[n];n++){
			sprintf(buf,"si5338,%s",pll_setup_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				freq[0]=be32_to_cpup(config_data);
				if (len<3){
					freq[1]=0;
					freq[2]=1;
				} else {
					freq[1]=be32_to_cpup(&config_data[1]);
					freq[2]=be32_to_cpup(&config_data[2]);
				}
				dev_dbg(&client->dev,"Found '%s', value = %lld+(%lld/%lld)\n",buf,freq[0],freq[1],freq[2]);
				if (n & 2){ /* by output */
					if (set_pll_freq_by_out(client, freq, n & 1)<0) return;
				} else { /* directly set  PLL frequency */
					if (set_pll_freq       (client, freq, n & 1)<0) return;
				}
				if (set_pll_paremeters(client)<0) return;
/*				if (set_misc_registers(client)<0) return; */ /* moved to pre_init() */ 
			}
		}
		/* setting MSn dividers (same channel as output), powering them up, setting output dividers and routing outputs */
		for (n=0;out_freq_setup_names[n];n++){
			sprintf(buf,"si5338,%s",out_freq_setup_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				freq[0]=be32_to_cpup(config_data);
				if (len<3){
					freq[1]=0;
					freq[2]=1;
				} else {
					freq[1]=be32_to_cpup(&config_data[1]);
					freq[2]=be32_to_cpup(&config_data[2]);
				}
				dev_dbg(&client->dev,"Found '%s', value = %lld+(%lld/%lld)\n",buf,freq[0],freq[1],freq[2]);
				if (set_out_frequency_and_route(client, freq, n&3, n>>2)<0) return;
			}
		}
		/* configure output driver standard */
		for (n=0;drv_configs[n].description;n++){
			sprintf(buf,"si5338,%s",drv_configs[n].description);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (configure_output_driver(&client->dev, drv_configs[n].description, setup_data.mask)<0) return;
				}
			}
		}

		/* configure disabled state of the output(s) */
		for (n=0;out_dis_states[n];n++){
			sprintf(buf,"si5338,%s",out_dis_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_disable(client, n, setup_data.mask)<0) return;
				}
			}
		}

		/* configure powerdown state of the output(s) */
		for (n=0;out_pwr_states[n];n++){
			sprintf(buf,"si5338,%s",out_pwr_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_powerdown(client, n, setup_data.mask)<0) return;
				}
			}
		}

		/* configure output enable state of the output(s) */
		for (n=0;out_en_states[n];n++){
			sprintf(buf,"si5338,%s",out_en_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_disable(client, n, setup_data.mask)<0) return;
				}
			}
		}
		/* setting spread spectrum parameters */
		for (n=0;n<4;n++){
			sprintf(buf,"si5338,spread_spectrum_%d",n);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				rate=get_ss_down_rate(client, n);
				amp= get_ss_down_amplitude(client, n);
				if (len>1) amp =  be32_to_cpup(&config_data[1]);
				if (len>2) rate = be32_to_cpup(&config_data[2]);
				if (store_ss_down_parameters(client, rate, amp, n)==0){ 
					dev_dbg(&client->dev,"Set spread spectrum parameters for MS%d, amplitude=%d (*0.01%%), rate=%d Hz, %s\n",
							n,amp,rate,config_data[0]?"ON":"OFF");
				} else {
					dev_err(&client->dev,"Failed to set spread spectrum parameters for MS%d, amplitude=%d (*0.01%%), rate=%d Hz, %s\n",
							n,amp,rate,config_data[0]?"ON":"OFF");
					continue;
				}
				if (config_data[0]){ /* enable SS */
					if ((set_ss_down(client, n)==0) && /* calculate and set SS registers */
							(set_ss_state(client, 1, n)==0)){ // enable SS. Not using enable_spread_spectrum() as we'll reset MS later anyway
						dev_dbg(&client->dev,"Spread spectrum enabled for MS%d\n",n);
					} else {
						dev_err(&client->dev,"Fail to enable spread spectrum for MS%d\n",n);
					}
				}
			}
		}
	} else {
		dev_info(&client->dev,"Device tree data not found for %s\n",client->name);
	}
	if (init_type){
		if (post_init(client,INIT_TIMEOUT)<0) dev_err(&client->dev,"SI5338 initialization failed\n");
		else dev_info(&client->dev,"SI5338 initialized\n");
	}

}

static void invalidate_cache(struct i2c_client *client)
{
	int i;
	struct si5338_data_t *clientdata = i2c_get_clientdata(client);
	for (i=0;i<=LAST_REG;i++){
		clientdata->cache[i].flags&= ~CACHE_INIT;
	}
}


static int si5338_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int i,rc=0;
	struct si5338_data_t *clientdata = NULL;
	/* initialize i2c ... */
//#define REG5338_DEV_CONFIG2          2
//#define REG5338_DEV_CONFIG2_MASK  0x3f
//#define REG5338_DEV_CONFIG2_VAL    38 /* last 2 digits of part number */
	if (((rc=_write_single_reg(client, REG5338_PAGE,0)))<0) return rc; // did not respond
	if (((rc=read_reg(client, REG5338_DEV_CONFIG2)))<0) return rc; // did not respond
	if ((rc & REG5338_DEV_CONFIG2_MASK)!= REG5338_DEV_CONFIG2_VAL){
		dev_err(&client->dev,
			 "Chip returned unexpected value from reg %d: %d, expected %d. It is not %s\n",
			 REG5338_DEV_CONFIG2,rc, REG5338_DEV_CONFIG2_VAL,id->name);
		return -EIO;
	}

	dev_info(&client->dev,
		 "Chip %s is found, driver version %s\n", id->name, DRV_VERSION);
	clientdata = devm_kzalloc(&client->dev, sizeof(*clientdata), GFP_KERNEL);
	for (i=0;i<=LAST_REG;i++){
		clientdata->cache[i].flags=0;
		clientdata->cache[i].data=0;
	}
	for (i=0;volatile_registers[i]>=0;i++){
		clientdata->cache[volatile_registers[i]>>8].flags |= CACHE_VOLAT;
	}
	//volatile_registers[]
	i2c_set_clientdata(client, clientdata);
	if (((rc=read_reg(client, REG5338_PAGE)))<0) return rc; // will set clientdata->last_page
	si5338_sysfs_register(&client->dev);
	mutex_init(&clientdata->lock);

	clientdata->input_frequency12=0;
	clientdata->input_frequency3=0;
	clientdata->input_frequency4=0;
	clientdata->input_frequency56=0;
	clientdata->ss_on_freq_change=0;  /* 0 - disable SS when frequency is changed, 1 - update SS.  +2 reset MS after starting SS*/ 

	for (i=0;i<4;i++){
		clientdata->spread_spectrum_rate[i]=SPREAD_RATE_DFLT; /* 31.5 KHz */
		clientdata->spread_spectrum_amp[i]=SPREAD_AMP_DFLT; /* 0.5% */
	}
	si5338_init_of(client);
	return 0;
}

static int si5338_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static struct i2c_driver si5338_i2c_driver = {
	.driver = {
		.name	= "si5338",
		.owner	= THIS_MODULE,
	},
	.probe		= si5338_i2c_probe,
	.remove		= si5338_i2c_remove,
	.id_table	= si5338_id,
};
module_i2c_driver(si5338_i2c_driver);
MODULE_DEVICE_TABLE(i2c, si5338_id);
MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("SI5338 I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:si5338");
