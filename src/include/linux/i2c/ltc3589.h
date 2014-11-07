/*!***************************************************************************
 *! FILE NAME  : ltc3589.c
 *! DESCRIPTION: control of the Linear Technology LTC3589 8-channel voltage regulator 
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
 
#ifndef __LINUX_LTC3589_H
#define __LINUX_LTC3589_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
int ltc3589_read_field(struct i2c_client *client, u32 awe);
int ltc3589_write_field(struct i2c_client *client, u8 data, u32 awe);
int ltc3589_write_adwe(struct i2c_client *client, u32 adwe);
void ltc3589_set_simulate(struct i2c_client *client, int simulate);

#define LTC3589_AWE_SCR1                   0x07ff
#define LTC3589_AWE_SCR1_MODE_SD1          0x0703
#define LTC3589_AWE_SCR1_MODE_SD2          0x070c
#define LTC3589_AWE_SCR1_MODE_SD3          0x0730
#define LTC3589_AWE_SCR1_MODE_BB           0x0740

#define LTC3589_AWE_OVEN                   0x10ff
#define LTC3589_AWE_OVEN_EN_SD1            0x1001
#define LTC3589_AWE_OVEN_EN_SD2            0x1002
#define LTC3589_AWE_OVEN_EN_SD3            0x1004
#define LTC3589_AWE_OVEN_EN_BB             0x1008
#define LTC3589_AWE_OVEN_EN_LDO2           0x1010
#define LTC3589_AWE_OVEN_EN_LDO3           0x1020
#define LTC3589_AWE_OVEN_EN_LDO4           0x1040
#define LTC3589_AWE_OVEN_ONLY              0x1080

#define LTC3589_AWE_SCR2                   0x12ff
#define LTC3589_AWE_SCR2_NOWAIT_SD1        0x1201
#define LTC3589_AWE_SCR2_NOWAIT_SD2        0x1202
#define LTC3589_AWE_SCR2_NOWAIT_SD3        0x1204
#define LTC3589_AWE_SCR2_NOWAIT_BB         0x1208
#define LTC3589_AWE_SCR2_NOWAIT_LDO2       0x1210
#define LTC3589_AWE_SCR2_NOWAIT_LDO3       0x1220
#define LTC3589_AWE_SCR2_NOWAIT_LDO4       0x1240
#define LTC3589_AWE_SCR2_PGOOD_SHTDN_INH   0x1280

#define LTC3589_AWE_VCCR                   0x20ff
#define LTC3589_AWE_VCCR_SLEW_SD1          0x2001 /* self clearing bit */
#define LTC3589_AWE_VCCR_REF_SEL_SD1       0x2002
#define LTC3589_AWE_VCCR_SLEW_SD2          0x2004 /* self clearing bit */
#define LTC3589_AWE_VCCR_REF_SEL_SD2       0x2008
#define LTC3589_AWE_VCCR_SLEW_SD3          0x2010 /* self clearing bit */
#define LTC3589_AWE_VCCR_REF_SEL_SD3       0x2020
#define LTC3589_AWE_VCCR_SLEW_LDO2         0x2040 /* self clearing bit */
#define LTC3589_AWE_VCCR_REF_SEL_LDO2      0x2080

#define LTC3589_AWE_CLIRQ                  0x21ff

#define LTC3589_AWE_B1DTV1                 0x23ff
#define LTC3589_AWE_B1DTV1_REF             0x231f
#define LTC3589_AWE_B1DTV1_PGMASK          0x2320
#define LTC3589_AWE_B1DTV1_DVDT            0x23c0

#define LTC3589_AWE_B1DTV2                 0x24ff
#define LTC3589_AWE_B1DTV2_REF             0x241f
#define LTC3589_AWE_B1DTV2_CLKRATE         0x2420
#define LTC3589_AWE_B1DTV2_PHASE           0x2440
#define LTC3589_AWE_B1DTV2_KEEP_ALIVE      0x2480

#define LTC3589_AWE_VRRCR                  0x25ff
#define LTC3589_AWE_VRRCR_SD1              0x2503
#define LTC3589_AWE_VRRCR_SD2              0x250c
#define LTC3589_AWE_VRRCR_SD3              0x2530
#define LTC3589_AWE_VRRCR_LDO2             0x25c0

#define LTC3589_AWE_B2DTV1                 0x26ff
#define LTC3589_AWE_B2DTV1_REF             0x261f
#define LTC3589_AWE_B2DTV1_PGMASK          0x2620

#define LTC3589_AWE_B2DTV2                 0x27ff
#define LTC3589_AWE_B2DTV2_REF             0x271f
#define LTC3589_AWE_B2DTV2_CLKRATE         0x2720
#define LTC3589_AWE_B2DTV2_PHASE           0x2740
#define LTC3589_AWE_B2DTV2_KEEP_ALIVE      0x2780

#define LTC3589_AWE_B3DTV1                 0x29ff
#define LTC3589_AWE_B3DTV1_REF             0x291f
#define LTC3589_AWE_B3DTV1_PGMASK          0x2920

#define LTC3589_AWE_B3DTV2                 0x2aff
#define LTC3589_AWE_B3DTV2_REF             0x2a1f
#define LTC3589_AWE_B3DTV2_CLKRATE         0x2a20
#define LTC3589_AWE_B3DTV2_PHASE           0x2a40
#define LTC3589_AWE_B3DTV2_KEEP_ALIVE      0x2a80

#define LTC3589_AWE_L2DTV1                 0x32ff
#define LTC3589_AWE_L2DTV1_REF             0x321f
#define LTC3589_AWE_L2DTV1_PGMASK          0x3220
#define LTC3589_AWE_L2DTV1_KEEP_ALIVE      0x3280

#define LTC3589_AWE_L2DTV2                 0x33ff
#define LTC3589_AWE_L2DTV2_REF             0x331f
#define LTC3589_AWE_L2DTV2_REF_LDO4        0x3360
#define LTC3589_AWE_L2DTV2_MODE_LDO4       0x3380

#define LTC3589_AWE_IRQSTAT                0x02ff
#define LTC3589_AWE_IRQSTAT_PGOOD_TIMOUT   0x0208
#define LTC3589_AWE_IRQSTAT_NEAR_UV        0x0210
#define LTC3589_AWE_IRQSTAT_HARD_UV        0x0220
#define LTC3589_AWE_IRQSTAT_NEAR_THERM     0x0240
#define LTC3589_AWE_IRQSTAT_HARD_THERM     0x0280

#define LTC3589_AWE_PGSTAT                 0x13ff
#define LTC3589_AWE_PGSTAT_LDO1            0x1301
#define LTC3589_AWE_PGSTAT_SD1             0x1302
#define LTC3589_AWE_PGSTAT_SD2             0x1304
#define LTC3589_AWE_PGSTAT_SD3             0x1308
#define LTC3589_AWE_PGSTAT_BB              0x1310
#define LTC3589_AWE_PGSTAT_LDO2            0x1320
#define LTC3589_AWE_PGSTAT_LDO3            0x1340
#define LTC3589_AWE_PGSTAT_LDO4            0x1380

#endif /* __LINUX_LTC3589_H */
