/***************************************************************************//**
 * @file   sensor_i2c.c
 * @brief  Interface to FPGA-based i2c sequencer for sensor ports
 * @copyright Copyright 2016 (C) Elphel, Inc.
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
//#define DEBUG /* should be before linux/module.h - enables dev_dbg at boot in this file */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <uapi/elphel/x393_devices.h> // just driver name
#include <asm/uaccess.h>
#include "x393.h"
#include "sensor_i2c.h"
#include <uapi/elphel/c313a.h> // PARS_FRAMES_MASK

//------------------

#include <asm/page.h>
#include <linux/io.h>

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_net.h>
#include <linux/sysfs.h>
#include <linux/list.h>

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

//#define DRV_NAME "elphel_sensor_i2c"
// FIXME: Warnings in softtirq.c, do not understand why. using LOCK_BH_SENSORI2C to enable spin_lock_bh, otherwise use spin_lock_irqsave
#undef LOCK_BH_SENSORI2C
struct x393_i2c_device_list {
    x393_i2c_device_t i2c_dev;
    struct list_head  list;
};
const char sensor_i2c_of_prop_name[] = "elphel393-sensor-i2c,i2c_devices";
const char group_name[] =   "i2c_classes";
const int max_buf_len =     4096-256; ///< stop output when only 256 bytes are left in the caller's buffer
const int max_i2c_classes = 256;
const int mclk_mhz =        200;      ///< mclk frequency in MHz
//const int tenth_sec =       max(2, HZ/10); // make it not less than 2 jiffies for some very slow clock
//const int tenth_sec =       (HZ >=20) ? (HZ/10) : 2; // make it not less than 2 jiffies for some very slow clock
const int tenth_sec =       2;

static LIST_HEAD(i2c_class_list);
static u32 free_i2c_groups[4];        ///< Number of channels is hard-wired to 4
static u32 free_i2c_pages [4][8];
static u32 i2c_pages_shadow[4][256]; ///< Mostly for debugging to analyze i2c pages allocation
static int sysfs_page[4];            ///< when positive - page locked for exclusive access
static struct device *sdev = NULL;   ///< store this device here
static u32    i2c_read_data[4];      ///< last data read from i2c device
static int    scale_i2c_speed = 256; // scale i2c speed for all devices, coefficint is scale_i2c_speed/256
static int    max_unbalanced_writes = 32; // debug feature used in multi10359 - maximal writes to i2c w/o read
static int unbalanced_writes [SENSOR_PORTS] = {0,0,0,0};

//static DEFINE_SPINLOCK(lock);
static DEFINE_SPINLOCK(sensori2c_lock_0); ///<
static DEFINE_SPINLOCK(sensori2c_lock_1); ///<
static DEFINE_SPINLOCK(sensori2c_lock_2); ///<
static DEFINE_SPINLOCK(sensori2c_lock_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
spinlock_t * sensori2c_locks[4] = {&sensori2c_lock_0, &sensori2c_lock_1, &sensori2c_lock_2, &sensori2c_lock_3};

/** Set status to autoupdate mode and wait for updated result (if it was not in autoupdate mode yet) */
void i2c_autoupdate_status(int chn)  ///< Sensor port
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_status_sens_i2c_t  status;
    x393_status_ctrl_t status_ctrl = get_x393_sensi2c_status_ctrl(chn); // last written data to status_cntrl
    if (status_ctrl.mode != 3){
#ifdef LOCK_BH_SENSORI2C
        spin_lock_bh(sensori2c_locks[chn]);
#else
        spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
        status =  x393_sensi2c_status (chn);
        status_ctrl.mode = 3;
        status_ctrl.seq_num = status.seq_num ^ 0x20;
        set_x393_sensi2c_status_ctrl(status_ctrl, chn);
        {
            int i;
            for (i = 0; i < 10; i++) {
                status = x393_sensi2c_status(chn);
                if (likely(status.seq_num = status_ctrl.seq_num)) break;
            }
        }
#ifdef LOCK_BH_SENSORI2C
        spin_unlock_bh(sensori2c_locks[chn]);
#else
        spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
        dev_dbg(sdev, "read_xi2c_fifo(%d): mode set to 3, status updated to 0x%08x\n",chn, (int) status.d32);
    }
}

/** I2C sequencer stop/run/reset (also programs status if run)*/
int i2c_stop_run_reset(int chn,  ///< Sensor port
        int cmd)  ///< Command: I2C_CMD_STOP=0 - stop, I2C_CMD_RUN=1 - run, I2C_CMD_RESET=2 - reset
///< @return always 0
{
    x393_i2c_ctltbl_t i2c_ctl = {.d32=0};
    x393_status_ctrl_t status_ctrl = {.d32=0};

    switch (cmd){
    case I2C_CMD_STOP:
        i2c_ctl.cmd_run = 2;
        break;
    case I2C_CMD_RUN:
        i2c_ctl.cmd_run = 3;
        status_ctrl.mode = 3; // autoupdate, is anyway set to it when reading i2c
        break;
    case I2C_CMD_RESET:
        i2c_ctl.reset =   1;
    }
    if (i2c_ctl.d32)
        x393_sensi2c_ctrl (i2c_ctl, chn);
    if (cmd == I2C_CMD_RESET)
        udelay(1);
    if (status_ctrl.mode)
        set_x393_sensi2c_status_ctrl(status_ctrl, chn);
    return 0;
}
EXPORT_SYMBOL_GPL(i2c_stop_run_reset);

/** I2C sequencer drive mode */
int i2c_drive_mode(int chn,             ///< Sensor port
        int sda_drive_high,  ///< Actively drive SDA=1 during second half of SCL=1
        int sda_release)     ///< Release SDA early if next bit is SDA=1
///< @return always 0
{
    x393_i2c_ctltbl_t i2c_ctl = {.d32=0};
    i2c_ctl.sda_drive_high =    sda_drive_high;
    i2c_ctl.sda_release =       sda_release;
    i2c_ctl.drive_ctl =         1;
    x393_sensi2c_ctrl (i2c_ctl, chn);
    return 0;
}
EXPORT_SYMBOL_GPL(i2c_drive_mode);

/** Mark all i2c pages for each channel as free */
void i2c_page_alloc_init( int chn)
{
    int i;
    free_i2c_groups[chn] = 0xff000000; // Should it be different?
    for (i = 0; i < sizeof(free_i2c_pages[chn])/ sizeof(free_i2c_pages[chn][0]);  i++) free_i2c_pages[chn][i] =  0xffffffff;
    for (i = 0; i < sizeof(i2c_pages_shadow[chn])/sizeof(i2c_pages_shadow[chn][0]); i++) i2c_pages_shadow[chn][i] = 0;
    sysfs_page[chn] = -1;
    dev_dbg(sdev, "All I2c page allocation for sensor port %d, is RESET( group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
            chn,free_i2c_groups[chn],
            free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
            free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);


}
EXPORT_SYMBOL_GPL(i2c_page_alloc_init);


/** Reserve i2c page (1 of 256) for a sensor port
 * @param chn sensor port number (0..3)
 * @return allocated page number or -ENOMEM if all pages are already used
 */
int i2c_page_alloc(int chn)
{
    int g, b;
    u32 * fb;
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
//    if (unlikely(g > 7)) {
    if (unlikely(!free_i2c_groups[chn])) {
#ifdef LOCK_BH_SENSORI2C
        spin_unlock_bh(sensori2c_locks[chn]);
#else
        spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
        dev_dbg(sdev, "-ENOMEM(chn=%d, group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
                chn,free_i2c_groups[chn],
                free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
                free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);
        return -ENOMEM; // no free i2c pages left
    }
    g = __builtin_clz(free_i2c_groups[chn]);
//    fb = free_i2c_pages + ((chn << 3) + g);
    fb = &free_i2c_pages[chn][g];

    b = __builtin_clz(*fb);
    *fb &= ~ (1 << (31-b));
    if (unlikely(*fb == 0)){
        free_i2c_groups[chn] &= ~(1 << (31 - g));
    }
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
    dev_dbg(sdev, "Allocated page= %d for port %d, (group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
            (g << 5) + b, chn,free_i2c_groups[chn],
            free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
            free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);

    return (g << 5) + b;
}
EXPORT_SYMBOL_GPL(i2c_page_alloc);

/** Register specific page, can be used with legacy software to register page equal to slave address,
 * and use 0xff for reading. Works with 1-byte addresses and 16-bit data */
int i2c_page_register(int chn,   ///< Sensor port
        int page)  ///< page to register (for legacy software, use 7-bit slave address
///< @return 0 on success, -ENOMEM if page is already registered
{
    int g =    page >> 5;
    int b =    page & 0x1f;
//    u32 * fb = free_i2c_pages + ((chn << 3) + g);
    u32 * fb = &free_i2c_pages[chn][g];
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    dev_dbg(sdev, "i2c_page_register( %d, %d): g=%d, b=%d, group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
                chn,page, g, b, free_i2c_groups[chn],
                free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
                free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    if (unlikely(!(*fb & (1 << (31-b))))) {
#ifdef LOCK_BH_SENSORI2C
        spin_unlock_bh(sensori2c_locks[chn]);
#else
        spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
        dev_dbg(sdev, "-ENOMEM(chn=%d, page=%d, group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
                chn,page,free_i2c_groups[chn],
                free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
                free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);
        return -ENOMEM; // page is already registered
    }
    *fb &= ~(1 << (31-b));
    if (unlikely(*fb == 0)){
        free_i2c_groups[chn] &= ~(1 << (31 - g));
    }
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
    dev_dbg(sdev, "DONE (chn=%d, page=%d, group= 0x%08x bits = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x )\n",
                chn,page,free_i2c_groups[chn],
                free_i2c_pages[chn][0],free_i2c_pages[chn][1],free_i2c_pages[chn][2],free_i2c_pages[chn][3],
                free_i2c_pages[chn][4],free_i2c_pages[chn][5],free_i2c_pages[chn][6],free_i2c_pages[chn][7]);
    return 0;
}
EXPORT_SYMBOL_GPL(i2c_page_register);


/** Free i2c page
 * @param chn sensor port number (0..3)
 * @param page i2c configuration page to release
 */
void i2c_page_free(int chn, int page)
{
    int g =    page >> 5;
    int b =    page & 0x1f;
//    u32 * fb = free_i2c_pages + ((chn << 3) + g);
    u32 * fb = &free_i2c_pages[chn][g];

#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    free_i2c_groups[chn] |=  1 << (31 - g);
    *fb |= 1 << (31-b);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
    dev_dbg(sdev, "Freed chn=%d, page=%d\n", chn, page);
}

EXPORT_SYMBOL_GPL(i2c_page_free);

/** Calculate value of SCLK 1/4 period delay from maximal SCL frequency
 * @param khz - maximal I2C clock frequency in KHz
 * @return bit delay for 1/4 SCL period in mclk (now 200MHz) cycles*/
int get_bit_delay(int khz)
{
    int dly;
    if (!khz)
        return 0;
// Modified FPGA - added extra division
//    dly = (1000 * mclk_mhz / 4)/khz;
//    dly = (1000 * mclk_mhz / 8 )/khz; //

    // WARNING: for 500KHz it is supposed to write 100, but it actually writes 50
    dly = (32000 * mclk_mhz / scale_i2c_speed )/khz;
    if (dly > 255)
        dly = 255;
    return dly;
}

/** Set i2c table entry to raw data (will just overwrite tbl_mode = 2)*/
void set_xi2c_raw(int chn,    ///< Sensor port number
        int page,   ///< index in lookup table
        u32 data)   ///< Raw table data (bits 27:0), bits 29:28 will be set to 2
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data, tb_addr;
    tb_addr.d32 =      0;
    tb_addr.tbl_mode = 3;
    tb_data.d32 =      data;
    tb_data.tbl_mode = 2; //
    /* Table address and data should not interleave with others */
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    x393_sensi2c_ctrl (tb_addr, chn);
    x393_sensi2c_ctrl (tb_data, chn);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
//    i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
    i2c_pages_shadow[chn][page] =tb_data.d32;
    dev_dbg(sdev, "Channel = %d, page = %d, raw data = 0x%08x\n", chn, page, tb_data.d32);

}
EXPORT_SYMBOL_GPL(set_xi2c_raw);

/** Set i2c table entry for write operation using known devices.
 * Get device with xi2c_dev_get(), copy and modify, if needed to
 * offset slave address or change number of bytes to write, SCL frequency
 */
void set_xi2c_wrc( x393_i2c_device_t * dc,   ///< device class
        int                  chn,  ///< sensor port
        int                  page, ///< index in lookup table
        int                  rah)  ///< High byte of the i2c register address
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data, tb_addr;
    tb_addr.d32 =      0;
    tb_addr.tbl_mode = 3;
    tb_addr.tbl_addr = page;
    tb_data.d32 =      0;
    tb_data.rah =      rah;
    tb_data.rnw =      0;
    tb_data.sa =       dc -> slave7;
    tb_data.nbwr =     dc -> address_bytes + dc -> data_bytes;
    tb_data.dly =      get_bit_delay(dc -> scl_khz);
    tb_data.tbl_mode = 2;
    dev_dbg(sdev, "device name %s : port= %d, page = 0x%x, rah = 0x%x\n",dc->name, chn, page,rah);
    /* Table address and data should not interleave with others */
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    x393_sensi2c_ctrl (tb_addr, chn);
    x393_sensi2c_ctrl (tb_data, chn);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
//    i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
    i2c_pages_shadow[chn][page] =tb_data.d32;
    dev_dbg(sdev, "DONE: device name %s : port= %d, page = 0x%x, rah = 0x%x\n",dc->name, chn, page,rah);
}
EXPORT_SYMBOL_GPL(set_xi2c_wrc);
/** Set i2c table entry for read operation using known devices
 * Get device with xi2c_dev_get(), copy and modify, if needed to
 * offset slave address or change number of bytes to write, SCL frequency
 */
void set_xi2c_rdc(x393_i2c_device_t * dc,  ///< device class
        int                 chn, ///< sensor port
        int                 page)///< index in lookup table
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data, tb_addr;
    tb_addr.d32 =      0;
    tb_addr.tbl_mode = 3;
    tb_addr.tbl_addr = page;
    tb_data.d32 =      0;
    tb_data.rnw =      1;
    tb_data.nabrd =    (dc -> address_bytes > 1)? 1:0;
    tb_data.nbrd =     dc -> data_bytes;
    tb_data.dly =      get_bit_delay(dc -> scl_khz);
    tb_data.tbl_mode = 2;
    /* Table address and data should not interleave with others */
    dev_dbg(sdev, "set_xi2c_rdc(*, %d, %d), tb_addr.d32=0x%08x, tb_data.d32 = 0x%08x\n",chn,page,tb_addr.d32,tb_data.d32);
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    x393_sensi2c_ctrl (tb_addr, chn);
    x393_sensi2c_ctrl (tb_data, chn);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
//    i2c_pages_shadow[(chn << 8) + page] = tb_data.d32;
    i2c_pages_shadow[chn][page] =tb_data.d32;
    dev_dbg(sdev, "DONE: set_xi2c_rdc(*, %d, %d), tb_addr.d32=0x%08x, tb_data.d32 = 0x%08x\n",chn,page,tb_addr.d32,tb_data.d32);
}
EXPORT_SYMBOL_GPL(set_xi2c_rdc);


/** Set i2c table entry for write operation */
void set_xi2c_wr(int chn,        ///< sensor port
        int page,       ///< index in lookup table
        int sa7,        ///< slave address (7 bit)
        int rah,        ///< High byte of the i2c register address
        int num_bytes,  ///< Number of bytes to write (1..10)
        int bit_delay)  ///< Bit delay - number of mclk periods in 1/4 of the SCL period
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data, tb_addr;
    tb_addr.d32 =      0;
    tb_addr.tbl_mode = 3;
    tb_addr.tbl_addr = page;
    tb_data.d32 =      0;
    tb_data.rah =      rah;
    tb_data.rnw =      0;
    tb_data.sa =       sa7;
    tb_data.nbwr =     num_bytes;
    tb_data.dly =      bit_delay;
    tb_data.tbl_mode = 2;
    /* Table address and data should not interleave with others */
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    x393_sensi2c_ctrl (tb_addr, chn);
    x393_sensi2c_ctrl (tb_data, chn);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
//    i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
    i2c_pages_shadow[chn][page] =tb_data.d32;
    dev_dbg(sdev, "chn=%d, page = %d, tb_addr.d32=0x%08x, tb_data.d32 = 0x%08x\n",chn,page,tb_addr.d32,tb_data.d32);

}
EXPORT_SYMBOL_GPL(set_xi2c_wr);


/** Set i2c table entry for read operation */
void set_xi2c_rd(int chn,           ///< sensor port
        int page,          ///< index in lookup table
        int two_byte_addr, ///< Number of address bytes (0 - one byte, 1 - two bytes)
        int num_bytes,     ///< Number of bytes to read (1..8, 0 means 8)
        int bit_delay)     ///< Bit delay - number of mclk periods in 1/4 of the SCL period
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data, tb_addr;
    tb_addr.d32 =      0;
    tb_addr.tbl_mode = 3;
    tb_addr.tbl_addr = page;
    tb_data.d32 =      0;
    tb_data.rnw =      1;
    tb_data.nabrd =    two_byte_addr;
    tb_data.nbrd =     num_bytes;
    tb_data.dly =      bit_delay;
    tb_data.tbl_mode = 2;
    /* Table address and data should not interleave with others */
#ifdef LOCK_BH_SENSORI2C
    spin_lock_bh(sensori2c_locks[chn]);
#else
    spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
    x393_sensi2c_ctrl (tb_addr, chn);
    x393_sensi2c_ctrl (tb_data, chn);
#ifdef LOCK_BH_SENSORI2C
    spin_unlock_bh(sensori2c_locks[chn]);
#else
    spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
//    i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
    i2c_pages_shadow[chn][page] =tb_data.d32;
    dev_dbg(sdev, "chn=%d, page = %d, tb_addr.d32=0x%08x, tb_data.d32 = 0x%08x\n",chn,page,tb_addr.d32,tb_data.d32);
}
EXPORT_SYMBOL_GPL(set_xi2c_rd);

/** @brief Write i2c command to the i2c command sequencer, block of 16 DWORD slots for absolute frame numbers
 *  (modulo 16) and 15 slots for relative ones: 0 - ASAP, 1 next frame, 14 -14-th next.
 * This command writes one or multiple DWORDs to i2c relative (modulo16) address. Use offs = 0 for immediate (ASAP) command.
 * Length of data is determined by the page data already preset. Data is written MSB first, first data word starts
 * with (in bits 31:24) table index.
 *
 * Data written depends on context:
 *
 * 1 - I2C register write: index page (MSB), 3 payload bytes. Payload bytes are used according to table and sent
 *     after the slave address and optional high address byte. Other bytes are sent in descending order (LSB- last).
 *     If less than 4 bytes are programmed in the table the high bytes (starting with the one from the table) are
 *     skipped.
 *     If more than 4 bytes are programmed in the table for the page (high byte), one or two next 32-bit words
 *     bypass the index table and all 4 bytes are considered payload ones. If less than 4 extra bytes are to be
 *     sent for such extra word, only the lower bytes are sent.
 *
 * 2 - I2C register read: index page, slave address (8-bit, with lower bit 0) and one or 2 address bytes (as programmed
 *     in the table. Slave address is always in byte 2 (bits 23:16), byte1 (high register address) is skipped if
 *     read address in the table is programmed to be a single-byte one. */
int write_xi2c_rel (int chn,    ///< sensor port
        int offs,   ///< relative frame number modulo 16
        u32 * data) ///< data array. Length written is defined by the pre-configured table entry.
///< MSB (data is sent MSB first) of the first entry is index in the table.
///< @return 0 on success, negative error if table shadow is not initialized to data
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data;
    int len;
    int i;
//    tb_data.d32 = i2c_pages_shadow[(chn <<8) + (data[0] >> 24)];
    tb_data.d32 = i2c_pages_shadow[chn][data[0] >> 24];
    if (tb_data.tbl_mode !=2) return -EBADR;
    len = (tb_data.rnw )? 1:((tb_data.nbwr + 5) >> 2); // read mode - always 1 DWORD, write - 1..3
    if (len > 1) {
#ifdef LOCK_BH_SENSORI2C
        spin_lock_bh(sensori2c_locks[chn]);
#else
        spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
        for (i = 0; i <len; i++){
            x393_sensi2c_rel (data[i], chn, offs);
        }
#ifdef LOCK_BH_SENSORI2C
        spin_unlock_bh(sensori2c_locks[chn]);
#else
        spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
    } else {
        x393_sensi2c_rel (data[0], chn, offs);
    }
    dev_dbg(sdev, "chn=%d, offs = %d, data[0] = 0x%08x\n",chn,offs,data[0]);
    return 0;
}
EXPORT_SYMBOL_GPL(write_xi2c_rel);

/** Write one or multiple DWORDs to i2c absolute (modulo16) address.
 * Length of data is determined by the page data already preset. Data is written MSB first, first data word starts
 * with (in bits 31:24) table index.
 * @see write_xi2c_rel */
int write_xi2c_abs (int chn,    ///< sensor port
        int offs,   ///< absolute frame number modulo 16
        u32 * data) ///< data array. Length written is defined by the pre-configured table entry.
///< MSB (data is sent MSB first) of the first entry is index in the table.
///< @return 0 on success, negative error if table shadow is not initialized to data
{
#ifndef LOCK_BH_SENSORI2C
    unsigned long flags;
#endif
    x393_i2c_ctltbl_t tb_data;
    int len;
    int i;
//    tb_data.d32 = i2c_pages_shadow[(chn <<8) + (data[0] >> 24)];
    tb_data.d32 = i2c_pages_shadow[chn][data[0] >> 24];

    if (tb_data.tbl_mode !=2) return -EBADR;
    len = (tb_data.rnw )? 1:((tb_data.nbwr + 5) >> 2); // read mode - always 1 DWORD, write - 1..3
    if (len > 1) {
#ifdef LOCK_BH_SENSORI2C
        spin_lock_bh(sensori2c_locks[chn]);
#else
        spin_lock_irqsave(sensori2c_locks[chn],flags);
#endif
        for (i = 0; i <len; i++){
            x393_sensi2c_abs (data[i], chn, offs);
        }
#ifdef LOCK_BH_SENSORI2C
        spin_unlock_bh(sensori2c_locks[chn]);
#else
        spin_unlock_irqrestore(sensori2c_locks[chn],flags);
#endif
    } else {
        x393_sensi2c_abs (data[0], chn, offs);
    }
    dev_dbg(sdev, "chn=%d, offs = %d, data[0] = 0x%08x\n",chn,offs,data[0]);
    return 0;
}
EXPORT_SYMBOL_GPL(write_xi2c_abs);

u32 i2c_combine_page_addr_data8_16( int chn,  ///< sensor port
                                    int page, ///< index in the table (8 bits)
                                    int addr, ///< register address (single byte for 8-bit data, LSB for 16 bit data)
                                    int data) ///< 8/16 bit data to write
                                              ///< @ return combined page/address/data
{
    x393_i2c_ctltbl_t i2c_ctltbl = {.d32= i2c_pages_shadow[chn][page]};
//    i2c_ctltbl.d32 = i2c_pages_shadow[chn][page];
//    if ((((x393_i2c_ctltbl_t) i2c_pages_shadow[chn][page]).nbwr > 2))
    if (i2c_ctltbl.nbwr > 2)
        return ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
    else
        return ((page & 0xff) << 24) | ((addr & 0xff) <<  8) | (data & 0xff);
}

/** Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_xi2c_reg16  (int chn,  ///< sensor port
        int page, ///< index in the table (8 bits)
        int addr, ///< low byte of the register address (high is in the table), 8 bits
        u32 data) ///< 16 or 8-bit data (LSB aligned), 16 address only for 16 bit data
{
//    u32 dw = ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
    u32 dw = i2c_combine_page_addr_data8_16(chn, page, addr, data);
    x393_sensi2c_rel (dw, chn, 0);
    dev_dbg(sdev, "i2c_pages_shadow[%d][0x%x]= 0x%08x addr=0x%x data=0x%x dw=0x%08x\n",
            chn,page,i2c_pages_shadow[chn][page], addr,data, (int) dw);
}
EXPORT_SYMBOL_GPL(write_xi2c_reg16);

/** Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_xi2c_reg16_rel (int chn,  ///< sensor port
        int page, ///< index in the table (8 bits)
        int frame, ///< relative frame number modulo PARS_FRAMES
        int addr, ///< low byte of the register address (high is in the table), 8 bits
        u32 data) ///< 16 or 8-bit data (LSB aligned), 16 address only for 16 bit data
{
//    u32 dw = ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
    u32 dw = i2c_combine_page_addr_data8_16(chn, page, addr, data);
    x393_sensi2c_rel (dw, chn, frame & PARS_FRAMES_MASK);
}
EXPORT_SYMBOL_GPL(write_xi2c_reg16_rel);

/** Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_xi2c_reg16_abs (int chn,  ///< sensor port
        int page, ///< index in the table (8 bits)
        int frame, ///< absolute frame number modulo PARS_FRAMES
        int addr, ///< low byte of the register address (high is in the table), 8 bits
        u32 data) ///< 16 or 8-bit data (LSB aligned), 16 address only for 16 bit data
{
//    u32 dw = ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
    u32 dw = i2c_combine_page_addr_data8_16(chn, page, addr, data);
    x393_sensi2c_abs (dw, chn, frame & PARS_FRAMES_MASK);
}
EXPORT_SYMBOL_GPL(write_xi2c_reg16_abs);

/** Compatibility with the legacy code: frame <0 - ASAP, >=0 - absolute
 * Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_xi2c_reg16_abs_asap (int chn,  ///< sensor port
        int page, ///< index in the table (8 bits)
        int frame, ///< absolute frame number modulo PARS_FRAMES
        int addr, ///< low byte of the register address (high is in the table), 8 bits
        u32 data) ///< 16 or 8-bit data (LSB aligned), 16 address only for 16 bit data
{
//    u32 dw = ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
    u32 dw = i2c_combine_page_addr_data8_16(chn, page, addr, data);
    if (frame<0) x393_sensi2c_rel (dw, chn, 0);
    else x393_sensi2c_abs (dw, chn, frame & PARS_FRAMES_MASK);
}
EXPORT_SYMBOL_GPL(write_xi2c_reg16_abs_asap);


/** Initiate sensor i2c read in immediate mode (data itself has to be read from FIFO with read_xi2c_fifo)
 *  Use slave address from provided class structure.
 */
void  read_xi2c (x393_i2c_device_t * dc, ///< device class
        int chn,                ///< sensor port
        int page,               ///< Table index (8 bits)
        int addr)               ///< 8/16 bit register address
{
    u32 dw = ((page & 0xff) << 24)  | (dc -> slave7 << 17) | (addr & 0xffff);
    x393_sensi2c_rel (dw, chn, 0);
    dev_dbg(sdev, "chn=%d, page = %d, addr = %d\n",chn,page, addr);

}
EXPORT_SYMBOL_GPL(read_xi2c);



/** Initiate sensor i2c read in immediate mode, specify 7-bit slave address
 *  (data itself has to be read from FIFO with read_xi2c_fifo) */
void  read_xi2c_sa7 (int chn,   ///< sensor port
        int page,  ///< Table index (8 bits)
        int sa7,   ///< 7-bit i2c slave address
        int addr)  ///< 8/16 bit register address
{
    u32 dw = ((page & 0xff) << 24)  | (sa7 << 17) | (addr & 0xffff);
    dev_dbg(sdev, "read_xi2c_sa7(%d,0x%x,0x%x,0x%x): 0x%08x\n",chn,page,sa7,addr,(int) dw);
    x393_sensi2c_rel (dw, chn, 0);
}
EXPORT_SYMBOL_GPL(read_xi2c_sa7);

/** Read i2c sequencer frame number (4 bits). Make sure it is equal to the cmdseq one */
int read_xi2c_frame(int chn) ///< sensor port number
///< @return i2c sequencer frame number (4 bits)
{
    x393_status_sens_i2c_t  status;
    i2c_autoupdate_status(chn);
    status =  x393_sensi2c_status (chn);
    return (int) status.frame_num;
}
EXPORT_SYMBOL_GPL(read_xi2c_frame);

/** Read next byte from the channel i2c read FIFO.
 * Sensor channel status will be set  to auto update mode (3) if it was in different mode */
int read_xi2c_fifo(int chn) ///< sensor port
///< @return byte or -1 if no data available, -EIO - error
{
    int fifo_lsb, rslt,i;
    x393_i2c_ctltbl_t i2c_cmd;
    x393_status_sens_i2c_t  status;
    x393_cmdseqmux_status_t  status2;

    i2c_autoupdate_status(chn);
    status =  x393_sensi2c_status (chn);
    //	dev_dbg(sdev, "read_xi2c_fifo(%d): status = 0x%08x\n",chn, (int) status.d32);
    if (!status.i2c_fifo_nempty) return -1; // No data available
    fifo_lsb = status.i2c_fifo_lsb;
    rslt = status.i2c_fifo_dout;
    /* Advance FIFO readout pointer */
    i2c_cmd.d32 = 0;
    i2c_cmd.next_fifo_rd = 1; // tbl_mode is 0 already
    x393_sensi2c_ctrl (i2c_cmd, chn);
    /* Make sure status matches next FIFO entry (or empty)*/
    for (i = 0; i < 10; i++) {
        status = x393_sensi2c_status(chn);
        if (likely(status.i2c_fifo_lsb != fifo_lsb)) break;
    }
    dev_dbg(sdev, "read_xi2c_fifo(%d): new status = 0x%08x\n",chn, (int) status.d32);
    status2 = x393_cmdseqmux_status();
    return rslt;
}
EXPORT_SYMBOL_GPL(read_xi2c_fifo);

/** Single-command i2c write register using pre-defined device classes.
 * This command reserves and configures table entry, releases it in the end */
int x393_xi2c_write_reg(const char * cname,    ///< device class name
        int          chn,      ///< sensor port number
        int          sa7_offs, ///< slave address (7-bit) offset from the class defined slave address,
        ///< used when multiple instances of the same class devices are present.
        int          reg_addr, ///< register address (width is defined by class)
        int          data)     ///< data to write (width is defined by class)
///< @return 0 on success, < 0 - error
{
    x393_i2c_device_t * dc;
    x393_i2c_device_t   ds;
    int                 page;
    /* Get i2c class */
    dc = xi2c_dev_get(cname);
    if (!dc) {
        dev_err(sdev, "I2c class name %s is not defined\n",cname);
        return -ENOENT;
    }
    /* Reserve table page */
    page = i2c_page_alloc(chn);
    if (page < 0) {
        dev_err(sdev, "Failed to reserve page, returned %d\n",page);
        return page;
    }
    /* Set table entry for writing */
    memcpy(&ds, dc, sizeof(ds));
    ds.slave7 = dc->slave7 + sa7_offs;
    set_xi2c_wrc(&ds,                // device class
            chn,                     // sensor port
            page,                    // index in lookup table
            (reg_addr >> 8) & 0xff); // High byte of the i2c register address
    /* Write i2c in immediate mode */
    write_xi2c_reg16  (chn,             // sensor port
            page,            // page (8 bits)
            reg_addr & 0xff, // low 8 bits
            (u32) data);     // 16 or 8-bit data (LSB aligned)
    /* Free table page */
    i2c_page_free(chn, page);
    return 0;
}
EXPORT_SYMBOL_GPL(x393_xi2c_write_reg);

/** Single-command i2c read register using pre-defined device classes.
 * This command reserves and configures table entry, initiates i2c read, waits and receives data, releases table entry in the end */
int x393_xi2c_read_reg( const char * cname,    ///< device class name
        int          chn,      ///< sensor port number
        int          sa7_offs, ///< slave address (7-bit) offset from the class defined slave address.
        ///< Offset is non-zero when multiple devices of the same class are present.
        int          reg_addr, ///< register address (width is defined by class)
        int *        datap)    ///< pointer to a data receiver (read data width is defined by class)
///< @return 0 on success, < 0 - error
{
    x393_i2c_device_t * dc;
    int                 page, i, db=-1;
    unsigned long       timeout_end;
    /* Get i2c class */
    dev_dbg(sdev, "x393_xi2c_read_reg(%s, %d, 0x%x, 0x%x)\n",cname,chn,sa7_offs,reg_addr);
    dc = xi2c_dev_get(cname);
    if (!dc) {
        dev_err(sdev, "I2c class name %s is not defined\n",cname);
        return -ENOENT;
    }
    dev_dbg(sdev, "got class %s:slave7=0x%x,address_bytes = %d, data_bytes = %d, scl_khz = %d)\n",
            dc->name, dc->slave7, dc->address_bytes, dc->data_bytes, dc->scl_khz);
    /* Reserve table page */
    page = i2c_page_alloc(chn);
    if (page < 0) {
        dev_err(sdev, "Failed to reserve page, returned %d\n",page);
        return page;
    }
    dev_dbg(sdev, "got page= 0x%x\n",page);

    /* Set table entry for reading */
    set_xi2c_rdc(dc,                      // device class
            chn,                     // sensor port
            page);                   // index in lookup table

    /* Flush channel i2c read FIFO to make sure it is empty, status mode will be set, if needed */
    while (read_xi2c_fifo(chn) >= 0) ; // includes waiting for status propagate
    dev_dbg(sdev, "Flushed i2c read fifo for channel, running read_xi2c_sa7(%d,0x%x,0x%x, 0x%x)\n",chn, page & 0xff, (dc->slave7 + sa7_offs) & 0x7f,  reg_addr & 0xffff);

    /* Initiate i2c read */
    read_xi2c_sa7 (chn,
            page & 0xff,                     // page (8 bits)
            (dc->slave7 + sa7_offs) & 0x7f,  // 7-bit i2c slave address
            reg_addr & 0xffff);              // 8/16 bit address

    /* Now read required number of bytes with timeout     */
    *datap = 0;
    dev_dbg(sdev, "Trying to get FIFO data for channel %d\n",chn);
    for (i = 0; i< dc->data_bytes; i++) {
        timeout_end = jiffies + tenth_sec;
        while (jiffies < timeout_end){
            db = read_xi2c_fifo(chn);
            if (db >=0)
                break;
        }
        if (db < 0) {
            /* release used page */
            dev_dbg(sdev, "Timeout waiting for i2c fifo read data for channel %d, freeing page %d\n",chn,page);
            i2c_page_free(chn, page);
            return -ETIMEDOUT;
        }
        *datap = (*datap <<8) | (db & 0xff);
    }

    /* Free table page */
    dev_dbg(sdev, "Freeing i2c page %d, got data 0x%x\n",page, datap[0]);
    i2c_page_free(chn, page);
    return 0;
}
EXPORT_SYMBOL_GPL(x393_xi2c_read_reg);

/** Test if i2c sequencer can accept more data (valid only for the same frame as last written
 * In ASAP mode ready means that at least 64/4=16 locations is empty, this can be used when sequencer
 * is stopped (no frame pulses) and data to be written is >64 items (i.e. programming 10359)
 * in non-ASAP mode can be tested only after writing if needs to write more to the same page
 * It will be ready again only if writing to a different page */
int x393_xi2c_ready_wr(int chn)  ///< sensor port number
                                 ///< @return - 1 - ready, 0 - busy
{
    x393_status_sens_i2c_t  status;
    i2c_autoupdate_status(chn);
    status =  x393_sensi2c_status (chn);
    return status.wr_full? 0 : 1;
}
EXPORT_SYMBOL_GPL(x393_xi2c_ready_wr);


/** Wait for channel sequencer ready to accept more data
 * It is valid for the same page as last written, most useful when writing ~>64 times in ASAP mode */
int x393_xi2c_wait_wr(int chn)  ///< sensor port number
                                ///< @return - OK, <0 -ETIMEDOUT
{
    int rdy;
    unsigned long       timeout_end;

    dev_dbg(sdev, "Waiting i2c sequencer ready for channel %d\n",chn);
    timeout_end = jiffies + tenth_sec;
    while (jiffies < timeout_end){
        if ((rdy=x393_xi2c_ready_wr( chn)))
            break;
    }
    if (!rdy) {
        dev_dbg(sdev, "Timeout waiting for i2c sequencer channel %d ready to accept more data\n",chn);
        return -ETIMEDOUT;
    }
    return 0;
}
EXPORT_SYMBOL_GPL(x393_xi2c_wait_wr);



/** Single-command i2c read register compatible with legacy code. Device class(es) should already be registered, len parameter should
 * match pre-programmed length */
int legacy_read_i2c_reg( int          chn,      ///< sensor port number
        int          page,     ///< i2c table page registered for read operation
        int          sa7,      ///< slave address (7-bit) of the device
        ///< Offset is non-zero when multiple devices of the same class are present.
        int          reg_addr, ///< register address (width is defined by class)
        int          len,      ///< number of bytes to read.
        int *        datap)    ///< pointer to a data receiver (read data width is defined by class)
///< @return 0 on success, < 0 - error (ETIMEDOUT)
{
    int               i, db=-1;
    unsigned long       timeout_end;
    *datap = 0;

    /* Flush channel i2c read FIFO to make sure it is empty, status mode will be set, if needed */
    // Seems after sensor re-init it was getting some stray data?
    while (read_xi2c_fifo(chn) >= 0); // includes waiting for status propagate
    dev_dbg(sdev, "Flushed i2c read fifo for channel %d\n",chn);

    /* Initiate i2c read */
    read_xi2c_sa7 (chn,
            page & 0xff,          // page (8 bits)
            sa7 & 0x7f,           // 7-bit i2c slave address
            reg_addr & 0xffff);   // 8/16 bit address

    /* Now read required number of bytes with timeout     */
    dev_dbg(sdev, "Trying to get FIFO data for channel %d\n",chn);
    for (i = 0; i< len; i++) {
        timeout_end = jiffies + tenth_sec;
        while (jiffies < timeout_end){
            db = read_xi2c_fifo(chn);
            if (db >=0)
                break;
        }
        if (db < 0) {
            dev_dbg(sdev, "Timeout waiting for i2c fifo read data for channel %d, freeing page %d\n",chn,page);
            return -ETIMEDOUT;
        }
        *datap = (*datap <<8) | (db & 0xff);
    }
    reset_unbalanced_writes(chn);
    return 0;
}
EXPORT_SYMBOL_GPL(legacy_read_i2c_reg);







/* Handling classes of i2c devices */
/** Find device list entry by device class name */
struct x393_i2c_device_list * i2c_dev_get(const char * name) ///< Device class name as string
///< @return list entry or NULL (if not found)
{
    struct list_head *p;
    struct x393_i2c_device_list * sp;
    list_for_each(p, &i2c_class_list) {

        sp = list_entry(p, struct x393_i2c_device_list, list);
        dev_dbg(sdev,"i2c_dev_get(%s): sp->i2c_dev.name=%s\n",name,sp->i2c_dev.name);
        if (!strncmp(name, sp->i2c_dev.name, sizeof(sp->i2c_dev.name)))
            return sp;
    }
    return NULL;
}
/** Find device list entry by device class name */
x393_i2c_device_t * xi2c_dev_get(const char * name) ///< Device class name as string
///< @return i2c device or NULL (if not found)
{
    struct x393_i2c_device_list * dl = i2c_dev_get(name);
    if (dl)
        return &dl->i2c_dev;
    else
        return NULL;

}
EXPORT_SYMBOL_GPL(xi2c_dev_get);
/** Add new i2c device class, OK to provide existing name */
struct x393_i2c_device_list * i2c_dev_add(const char * name) ///< Device class name (<32 characters)
///< @return pointer to a device list entry
{
    struct x393_i2c_device_list * sp = i2c_dev_get(name);
    if (sp) return sp; /* already exists */
    /*Allocate new structure */
    dev_dbg(sdev,"allocating space for %s\n",name);
    sp = (struct x393_i2c_device_list *) devm_kzalloc(sdev, sizeof(struct x393_i2c_device_list), GFP_KERNEL);
    dev_dbg(sdev,"Copying name\n");
    strncpy(sp->i2c_dev.name, name, sizeof(sp->i2c_dev.name));
    dev_dbg(sdev,"Done\n");
    INIT_LIST_HEAD(&sp->list);
    list_add(&sp->list, &i2c_class_list);
    dev_dbg(sdev,"list_add() OK\n");
    /* create sysfs entry here? */
    return sp;
}

/** Remove i2c device class by name */
int i2c_dev_remove(const char * name) ///< existing device class name
///< @return 0 if OK, -ENOENT if specified device class does not exist
{
    struct x393_i2c_device_list * sp = i2c_dev_get(name);
    if (!sp) return - ENOENT;
    /* remove sysfs entry */
    list_del(&sp->list);
    devm_kfree(sdev, sp);
    return 0;
}

//TODO: Add Write/read <name> <reg addr> <reg data>, with separate register for read data

/* =========================== sysfs functionality ============================== */

/** Sysfs helper function - get channel number from the last character of the attribute name*/
static int get_channel_from_name(struct device_attribute *attr) ///< Linux kernel interface for exporting device attributes
///< @return channel number
{
    int reg = 0;
    sscanf(attr->attr.name + (strlen(attr->attr.name)-1), "%du", &reg);
    return reg;
}
/** Sysfs function - respond to attempt of write to read only i2c device class member record */
static ssize_t i2c_member_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    dev_warn(dev,"i2c_member_store(): not implemented\n");
    return count;
}

/** Sysfs function - show i2c device class member record */
static ssize_t i2c_member_show(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    struct x393_i2c_device_list * sp;
    sp =  i2c_dev_get(attr->attr.name);
    if (!sp)
        return sprintf(buf,"i2c device class %s not found (should not get here)\n",attr->attr.name);
    return sprintf(buf,"%d %d %d %d\n",
            sp->i2c_dev.slave7,
            sp->i2c_dev.address_bytes,
            sp->i2c_dev.data_bytes,
            sp->i2c_dev.scl_khz);
}

/**Sysfs function - add new i2c device class. Buffer data should contain the following data, separated by spaces:
 * - name
 * - slave addr (7 bit)
 * - address width (bytes)
 * - data  width (bytes)
 * - max SCL frequency (kHz)
 */
static ssize_t i2c_class_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    struct x393_i2c_device_list * dl;
    char name[32];
    int ni, sa7, num_addr, num_data, khz;
    struct device_attribute *new_attr;
    int rslt;
    char * dname;
    ni = sscanf(buf,"%31s %i %i %i %i", name, &sa7, &num_addr, &num_data, &khz);
    if (ni < 5) {
        dev_err(dev, "Requires 5 parameters: name, slave addr (7 bit), address width (bytes), data  width (bytes), max SCL frequency (kHz)\n");
        return -EINVAL;
    }
    dl = i2c_dev_get(name);
    if (!dl){
        dl = i2c_dev_add (name);
        // create sysfs group member
        //group_name
        new_attr =  devm_kzalloc(dev, sizeof(new_attr[0]), GFP_KERNEL);
        if (!new_attr)
            return -ENOMEM;
        //		new_attr->attr.name =  devm_kzalloc(dev, strlen(name)+1, GFP_KERNEL);
        //		strcpy(new_attr->attr.name, (const char *) name);  // warning that it disqualifies const
        dname = devm_kzalloc(dev, strlen(name)+1, GFP_KERNEL);
        strcpy(dname, name);
        new_attr->attr.name= (const char *) dname;

        new_attr->attr.mode = SYSFS_PERMISSIONS;
        new_attr->show =      i2c_member_show;
        new_attr->store =     i2c_member_store;
        if (&dev->kobj) {
            if ((rslt = sysfs_add_file_to_group (
                    &dev->kobj,
                    &new_attr -> attr, // const struct attribute * attr,
                    group_name)))       // const char * group);
            {
                dev_err(dev,"i2c_class_store() failed to add %s to group %s\n",name, group_name);
                return rslt;
            }
        }
    }
    if (dl) {
        dl-> i2c_dev.slave7 =        sa7;
        dl-> i2c_dev.address_bytes = num_addr;
        dl-> i2c_dev.data_bytes =    num_data;
        dl-> i2c_dev.scl_khz =       khz;
    }

    return count;
}

/** Sysfs function - show all i2c device classes */
static ssize_t i2c_class_show(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    struct list_head *p;
    struct x393_i2c_device_list * sp;
    char * cp = buf;
    list_for_each(p, &i2c_class_list) {
        sp =   list_entry(p, struct x393_i2c_device_list, list);
        buf += sprintf(buf,"%s: 0x%x %d %d %d kHz\n",
                sp->i2c_dev.name,
                sp->i2c_dev.slave7,
                sp->i2c_dev.address_bytes,
                sp->i2c_dev.data_bytes,
                sp->i2c_dev.scl_khz);
        if ((buf-cp) > 	max_buf_len) {
            buf += sprintf(buf,"--- truncated ---\n");
            break;
        }
    }
    if (buf == cp){
        return sprintf(buf,"No I2C classes defined\n");
    }
    return buf-cp;
}

/** Sysfs function - allocate new page in i2c device table, output page number as a decimal value */
static ssize_t get_i2c_page_alloc(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer, or -ENOMEM
{
    int chn = get_channel_from_name(attr) ;
    int page;
    page= i2c_page_alloc(chn);
    if (page <0)
        return -ENOMEM;
    sysfs_page[chn] = page;
    return sprintf(buf,"%d\n",page);
}

/** Sysfs function - release page in i2c device table, input buffer *buf should contain page number as decimal or hex number*/
static ssize_t free_i2c_page(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    int page;
    sscanf(buf, "%i", &page);
    page &= 0xff;
    i2c_page_free(chn, page);
    return count;
}

/** Sysfs function - set page in i2c device table to use for next reading, input buffer *buf should contain page number as decimal or hex number*/
static ssize_t set_i2c_page_inuse(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    int page;
    sscanf(buf, "%i", &page);
    sysfs_page[chn] = page & 0xff;
    return count;
}

/** Sysfs function - get page number in i2c device table that was set to use for next reading.
 *  Output buffer *buf provides page number as a decimal */
static ssize_t get_i2c_page_inuse(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    int chn = get_channel_from_name(attr) ;
    return sprintf(buf,"%d\n",sysfs_page[chn]);
}



/** Sysfs function - get i2c table entry as raw value (as hex number), page number is the same as used for reading, @see set_i2c_page_inuse  */
static ssize_t get_i2c_tbl_raw(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer, or -ENXIO
{
    int chn = get_channel_from_name(attr) ;
    int page = sysfs_page[chn]; // currently selected page for sysfs reads
    if (page < 0)
        return -ENXIO;   /* No such device or address */

//    return sprintf(buf,"0x%08x\n",i2c_pages_shadow[(chn << 8) + (page &0xff)]);
    return sprintf(buf,"0x%08x\n",i2c_pages_shadow[chn][page &0xff]);
}

/** Sysfs function - set i2c table entry as raw value. Buffer string *buf should contain space-separated
 *  page number and table data as decimal or hex (starting with 0x) numbers */
static ssize_t set_i2c_tbl_raw(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    int ni, page, data;
    ni = sscanf(buf, "%i %i", &page, &data);
    if (ni < 2) {
        dev_err(dev, "Requires 2 parameters: page, data\n");
        return -EINVAL;
    }
    set_xi2c_raw(chn,
            page & 0xff,   // index in lookup table
            (u32) data); // Bit delay - number of mclk periods in 1/4 of the SCL period
    return count;
}

/** Sysfs function - read and interpret i2c table entry, page number is the same as used for reading, @see set_i2c_page_inuse  */
static ssize_t get_i2c_tbl_human(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer, or -ENXIO
{
    x393_i2c_ctltbl_t tb_data;
    int chn = get_channel_from_name(attr) ;
    int page = sysfs_page[chn]; // currently selected page for sysfs reads
    if (page < 0)
        return -ENXIO;   /* No such device or address */
//    tb_data.d32 = i2c_pages_shadow[(chn << 8) + (page & 0xff)];
    tb_data.d32 = i2c_pages_shadow[chn][page & 0xff];
    if (tb_data.rnw){
        return sprintf(buf,"Read entry: chn=%d page=%d(0x%x) two_byte_addr=%d number bytes to read=%d bit_duration=%d\n",
                chn,   page,page,            tb_data.nabrd,tb_data.nbrd,  tb_data.dly);
    } else {
        return sprintf(buf,"Write entry: chn=%d page=%d(0x%x) sa=0x%02x rah=0x%02x nbw=%d bit_duration=%d\n",
                chn,   page,page,tb_data.sa,tb_data.rah,tb_data.nbwr,  tb_data.dly);
    }
}

/** Sysfs function - set i2c table entry for i2c write commands. Buffer string *buf should contain the following
 *  space-separated items as decimal or hex (0x) values:
 *  - page,
 *  - slave address (7 bit),
 *
 *  Optional parameters (device class data will be used if not specified):
 *  - high reg address byte,
 *  - bytes to write (1..10),
 *  - 1/4 scl period in mclk (5 ns for current 200 MHz) */
static ssize_t set_i2c_tbl_wr_human(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    int ni, page, rah,sa7,nbwr,dly,khz;
    char name[32];
    x393_i2c_device_t * dc;
    x393_i2c_device_t ds;
    // check if it starts from a number
    if (sscanf(buf, "%i", &page)) {

        ni = sscanf(buf, "%i %i %i %i %i", &page, &sa7, &rah, &nbwr, &dly);
        if (ni < 2) {
            dev_err(dev, "Requires 5 parameters: page, slave address (7 bit), high reg address byte, bytes to write (1..10), 1/4 scl period in mclk\n");
            return -EINVAL;
        }
        set_xi2c_wr(chn,
                page & 0xff, // index in lookup table
                sa7 &  0x7f,  // slave address (7 bit)
                rah &  0xff,  // High byte of the i2c register address
                nbwr & 0xf, // Number of bytes to write (1..10)
                dly &  0xff); // Bit delay - number of mclk periods in 1/4 of the SCL period
    } else { // try alternative (class name based format)
        // set defaults for optional parameters
        sa7 =  0;
        rah =  0;
        nbwr = 0;
        dly =  0;
        ni = sscanf(buf, "%31s %i %i %i %i %i", name, &page, &sa7, &rah, &nbwr, &khz);
        if (ni < 2) {
            dev_err(dev, "Requires at least 2 parameters: class name, page. Optional: slave address (7 bit), high reg address byte, bytes to write (1..10), SCL freq. (kHz)\n");
            return -EINVAL;
        }
        dc = xi2c_dev_get(name);
        if (!dc) {
            dev_err(dev, "I2c class name %s is not defined\n",name);
            return -ENOENT;
        }
        memcpy(&ds, dc, sizeof(ds));
        ds.slave7 = dc->slave7 + sa7;
        if (nbwr) ds.data_bytes = nbwr;
        if (khz)  ds.scl_khz =    khz;
        set_xi2c_wrc(&ds,   // device class
                chn,  // sensor port
                page, // index in lookup table
                rah); // High byte of the i2c register address
    }
    return count;
}

/** Sysfs function - set i2c table entry for i2c read commands. Buffer string *buf should contain the following
 *  space-separated items as decimal or hex (0x) values:
 *  - page,
 *  - two byte addr (0 - 8bit,1 - 16 bit reg. addr),
 *
 *  Optional parameters (device class data will be used if not specified):
 *  - number of bytes to read (1..8),
 *  - 1/4 scl period in mclk (5 ns for current 200 MHz) */
static ssize_t set_i2c_tbl_rd_human(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    int ni, page, two_byte_addr, num_bytes, bit_delay, khz;
    char name[32];
    x393_i2c_device_t * dc;
    x393_i2c_device_t ds;
    // check if it starts from a number
    if (sscanf(buf, "%i", &page)) {
        ni = sscanf(buf, "%i %i %i %i", &page, &two_byte_addr, &num_bytes, &bit_delay);
        if (ni < 2) {
            dev_err(dev, "Requires 4 parameters: page, two byte addr (0 - 8bit,1 - 16 bit reg. addr), number of bytes to read (1..8), 1/4 scl period in mclk\n");
            return -EINVAL;
        }
        set_xi2c_rd(chn,
                page & 0xff,          // index in lookup table
                two_byte_addr & 1,    // Number of address bytes (0 - one byte, 1 - two bytes)
                num_bytes & 7,        // Number of bytes to read (1..8, 0 means 8)
                bit_delay & 0xff);    // Bit delay - number of mclk periods in 1/4 of the SCL period
    } else { // try alternative (class name based format)
        // set defaults for optional parameters
        num_bytes =  0;
        khz =        0;
        ni = sscanf(buf, "%31s %i %i %i", name, &page, &num_bytes, &khz);
        if (ni < 2) {
            dev_err(dev, "Requires at least 2 parameters: class name, page. Optional: number of bytes to read, SCL freq. (kHz)\n");
            return -EINVAL;
        }
        dc = xi2c_dev_get(name);
        if (!dc) {
            dev_err(dev, "I2c class name %s is not defined\n",name);
            return -ENOENT;
        }
        memcpy(&ds, dc, sizeof(ds));
        if (num_bytes) ds.data_bytes = num_bytes;
        if (khz)  ds.scl_khz =         khz;
        set_xi2c_rdc(&ds,    // device class name
                chn,    // sensor port
                page);  // index in lookup table
    }
    return count;
}

/** Sysfs function - i2c read/write register (read requires subsequent reading from the same node).
 * Buffer should contain the following space-separated items as decimal or hex (0x) values:
 *  - device class name,
 *  - slave offset, non-zero when multiple instances of the class are present on the bus or the address is different from the base one
 *  - device register address,
 *
 *  Additional parameter
 *  - data to write identifies write operation, if it is missing i2c read will be issued
 *  This command wait for sequencer ready after writing */
static ssize_t i2c_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int chn = get_channel_from_name(attr) ;
    char cname[32];
    int ni, sa7_offset, reg_addr, reg_data, rslt;
    dev_dbg(sdev, "i2c_store(), chn=%d\n",chn);

    ni = sscanf(buf, "%31s %i %i %i", cname, &sa7_offset, &reg_addr, &reg_data);

    dev_dbg(sdev, "i2c_store(), ni = %d, chn=%d, cname = %s, sa7_offset = 0x%x, reg_addr=0x%x, reg_data= 0x%x)\n",
            ni, chn, cname, sa7_offset, reg_addr, reg_data);

    if (ni < 3) {
        dev_err(dev, "Requires at least 3 parameters: cname, sa7_offset, reg_addr. Optional reg_data (for register write operation)\n");
        return -EINVAL;
    }
    if (ni == 3){ // register read
        dev_dbg(sdev, "i2c_store(), calling x393_xi2c_read_reg()\n");
        rslt =  x393_xi2c_read_reg(cname,      // device class name
                chn,        // sensor port number
                sa7_offset, // slave address (7-bit) offset from the class defined slave address
                reg_addr,   // register address (width is defined by class)
                &reg_data); // pointer to a data receiver (read data width is defined by class)
        if (rslt < 0) return rslt;
        i2c_read_data[chn] = (u32) reg_data;
    } else { // register write
        dev_dbg(sdev, "i2c_store(), calling x393_xi2c_write_reg()\n");
        rslt = x393_xi2c_write_reg(cname,      // device class name
                chn,        // sensor port number
                sa7_offset, // slave address (7-bit) offset from the class defined slave address
                reg_addr,   // register address (width is defined by class)
                reg_data);  // data to write (width is defined by class)
        if (rslt < 0) return rslt;
        if ((rslt = x393_xi2c_wait_wr(chn)) < 0) return rslt;
    }
    return count;
}

// Get i2c read data from fifo
/** Sysfs function - i2c read previously acquired i2c data from i2c read fifo, output buffer *buf
 * will contain decimal representation of the value.
 * This command should be preceded by the i2c command, @see i2c_store how to start an i2c read command. */
static ssize_t i2c_show(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    int chn = get_channel_from_name(attr) ;
    dev_dbg(sdev, "i2c_show(), chn=%d\n",chn);
    return sprintf(buf,"%d\n",i2c_read_data[chn]);
}

/** Sysfs function - output instructions text of how to communicate with i2c devices attached to the sensor ports.
 * Output buffer *buf will contain multi-line instructions text */
static ssize_t get_i2c_help(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    return sprintf(buf,"Numeric suffix in the file names selects sensor port\n"
            "i2c_all: read - list classes of i2c devices, write - add new (name, sa7, address bytes, data bytes, SCL kHz)\n"
            "i2c_classes - directory with register i2c classes, when read provide sa7, address bytes, data bytes, SCL kHz\n"
            "alloc*:   read - allocate and return page, write <page> (any data) - free page\n"
            "rd_page*: read/write page number used in read operations (-1 if none)\n"
            "tbl_raw*: read - raw hex table value (for current rd_page), write <page> <data32> set page\n"
            "tbl_wr*:  read - decoded table entry for current rd_page, write <page> <sa7> <high_addr_byte> <bytes_to_write> <dly>\n"
            "tbl_wr*:  alt:  <class-name> <page> <sa7-offset> <high_addr_byte> <bytes_to_write or 0 (use class)> <dly or 0(use class)>\n"
            "tbl_rd*:  read - decoded table entry for current rd_page, write <page> <2-byte addr> <bytes_to_read> <dly>\n"
            "tbl_rd*:  alt:  <class-name> <page> <bytes_to_read or 0 (use class)> <dly or 0 (use class)>\n"
            "tbl_rd* and tbl_wr* return same result when read. Delay is 8 bit, 250 - 200KHz SCL\n"
            "Read/write i2c register (for the pre-configured device classes), sa7_offset is added to class device slave address\n"
            "[<data>] is used for register write, without - register read. Reading i2c* returns result of last i2c read operation\n"
            "i2c*:    read - last read data, write: <class_name> <sa7_offset> <reg_addr> [<data>]\n"
            "i2c_frame*:    read - i2c sequencer frame number (4 bit), write: 0 - stop, 1 - run, 2 - reset, 3 - reset+run\n"
    );
}

/** Sysfs function - read i2c sequencer frame number (4 bits) */
static ssize_t i2c_frame_show(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        char *buf)                       ///< 4K buffer to receive response
///< @return offset to the end of the output data in the *buf buffer
{
    return sprintf(buf,"%d\n",read_xi2c_frame(get_channel_from_name(attr)));
}
/** Sysfs function - stop(0)/run(1)/reset(2)/reset+run(3) i2c sequencer */
static ssize_t i2c_frame_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    int cmd;
    int chn = get_channel_from_name(attr) ;
    if (sscanf(buf, "%i", &cmd)>0) {
        switch (cmd){
        case 0:
            i2c_stop_run_reset      (chn, I2C_CMD_STOP);
            break;
        case 1:
            i2c_stop_run_reset      (chn, I2C_CMD_RUN);
            break;
        case 2:
            i2c_stop_run_reset      (chn, I2C_CMD_RESET);
            break;
        case 3:
            i2c_stop_run_reset      (chn, I2C_CMD_RESET);
            i2c_stop_run_reset      (chn, I2C_CMD_RUN);
            break;
        default:
            return - EINVAL;
        }
    }
    return count;
}
//scale_i2c_speed
/** Show i2c speed scale (256 - normal speed, 128 - 1/2, 512 - 2x) */
static ssize_t i2c_speed_show(struct device *dev,            ///< Linux kernel basic device structure
                              struct device_attribute *attr, ///< Linux kernel interface for exporting device attributes
                              char *buf)                     ///< 4K buffer to receive response
                                                             ///< @return offset to the end of the output data in the *buf buffer
{
    return sprintf(buf,"%d\n",scale_i2c_speed);
}
static ssize_t i2c_speed_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    if (sscanf(buf, "%i", &scale_i2c_speed)> 0) {
        if (scale_i2c_speed<1) scale_i2c_speed = 1;
    }
    return count;
}

/** Show i2c speed scale (256 - normal speed, 128 - 1/2, 512 - 2x) */
static ssize_t i2c_max_unbalanced_writes_show(struct device *dev,            ///< Linux kernel basic device structure
                              struct device_attribute *attr, ///< Linux kernel interface for exporting device attributes
                              char *buf)                     ///< 4K buffer to receive response
                                                             ///< @return offset to the end of the output data in the *buf buffer
{
    return sprintf(buf,"%d\n",max_unbalanced_writes);
}
static ssize_t i2c_max_unbalanced_writes_store(struct device *dev,              ///< Linux kernel basic device structure
        struct device_attribute *attr,   ///< Linux kernel interface for exporting device attributes
        const char *buf,                 ///< 4K buffer with what was written
        size_t count)                    ///< offset to the end of buffer data
///< @return new offset to the end of buffer data
{
    sscanf(buf, "%i", &max_unbalanced_writes);
    return count;
}
int i2c_get_max_unbalanced_writes(void) // ///@return - maximal number of unbalanced writes (w/o reads) - debug feature
{
    return max_unbalanced_writes;
}
void reset_unbalanced_writes(int chn)
{
    unbalanced_writes[chn]=0;
}
void inc_unbalanced_writes(int chn)
{
    unbalanced_writes[chn]++;
}
int check_unbalanced_writes(int chn)
{
    return unbalanced_writes[chn] > max_unbalanced_writes;
}
int get_unbalanced_writes(int chn)
{
    return unbalanced_writes[chn];
}

//max_unbalanced_writes
// Sysfs top
/* alloc*: read - allocate and return page, write (any data) - free page */
static DEVICE_ATTR(i2c_all ,       SYSFS_PERMISSIONS                 ,    i2c_class_show ,       i2c_class_store);
static DEVICE_ATTR(alloc0  ,       SYSFS_PERMISSIONS                 ,    get_i2c_page_alloc ,   free_i2c_page);
static DEVICE_ATTR(alloc1  ,       SYSFS_PERMISSIONS                 ,    get_i2c_page_alloc ,   free_i2c_page);
static DEVICE_ATTR(alloc2  ,       SYSFS_PERMISSIONS                 ,    get_i2c_page_alloc ,   free_i2c_page);
static DEVICE_ATTR(alloc3  ,       SYSFS_PERMISSIONS                 ,    get_i2c_page_alloc ,   free_i2c_page);
/* rd_page*: read/write page number used in read operations */
static DEVICE_ATTR(rd_page0 ,      SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,    set_i2c_page_inuse);
static DEVICE_ATTR(rd_page1 ,      SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,    set_i2c_page_inuse);
static DEVICE_ATTR(rd_page2 ,      SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,    set_i2c_page_inuse);
static DEVICE_ATTR(rd_page3 ,      SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,    set_i2c_page_inuse);
static DEVICE_ATTR(tbl_raw0 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,       set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw1 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,       set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw2 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,       set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw3 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,       set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_wr0 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_wr_human);
static DEVICE_ATTR(tbl_wr1 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_wr_human);
static DEVICE_ATTR(tbl_wr2 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_wr_human);
static DEVICE_ATTR(tbl_wr3 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_wr_human);
static DEVICE_ATTR(tbl_rd0 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_rd_human);
static DEVICE_ATTR(tbl_rd1 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_rd_human);
static DEVICE_ATTR(tbl_rd2 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_rd_human);
static DEVICE_ATTR(tbl_rd3 ,       SYSFS_PERMISSIONS                 ,    get_i2c_tbl_human,     set_i2c_tbl_rd_human);
static DEVICE_ATTR(i2c0 ,          SYSFS_PERMISSIONS                 ,    i2c_show,              i2c_store);
static DEVICE_ATTR(i2c1 ,          SYSFS_PERMISSIONS                 ,    i2c_show,              i2c_store);
static DEVICE_ATTR(i2c2 ,          SYSFS_PERMISSIONS                 ,    i2c_show,              i2c_store);
static DEVICE_ATTR(i2c3 ,          SYSFS_PERMISSIONS                 ,    i2c_show,              i2c_store);
static DEVICE_ATTR(i2c_frame0 ,    SYSFS_PERMISSIONS                 ,    i2c_frame_show,        i2c_frame_store);
static DEVICE_ATTR(i2c_frame1 ,    SYSFS_PERMISSIONS                 ,    i2c_frame_show,        i2c_frame_store);
static DEVICE_ATTR(i2c_frame2 ,    SYSFS_PERMISSIONS                 ,    i2c_frame_show,        i2c_frame_store);
static DEVICE_ATTR(i2c_frame3 ,    SYSFS_PERMISSIONS                 ,    i2c_frame_show,        i2c_frame_store);
static DEVICE_ATTR(i2c_speed,      SYSFS_PERMISSIONS                 ,    i2c_speed_show,        i2c_speed_store);
static DEVICE_ATTR(max_unbalanced_writes, SYSFS_PERMISSIONS          ,  i2c_max_unbalanced_writes_show,i2c_max_unbalanced_writes_store);
static DEVICE_ATTR(help,           SYSFS_PERMISSIONS & SYSFS_READONLY,    get_i2c_help,          NULL);

/** Contains entries for the root node (directory) of the sysfs representation for sensor-port i2c devices */
static struct attribute *root_dev_attrs[] = {
        &dev_attr_i2c_all.attr,
        &dev_attr_alloc0.attr,
        &dev_attr_alloc1.attr,
        &dev_attr_alloc2.attr,
        &dev_attr_alloc3.attr,
        &dev_attr_rd_page0.attr,
        &dev_attr_rd_page1.attr,
        &dev_attr_rd_page2.attr,
        &dev_attr_rd_page3.attr,
        &dev_attr_tbl_raw0.attr,
        &dev_attr_tbl_raw1.attr,
        &dev_attr_tbl_raw2.attr,
        &dev_attr_tbl_raw3.attr,
        &dev_attr_tbl_wr0.attr,
        &dev_attr_tbl_wr1.attr,
        &dev_attr_tbl_wr2.attr,
        &dev_attr_tbl_wr3.attr,
        &dev_attr_tbl_rd0.attr,
        &dev_attr_tbl_rd1.attr,
        &dev_attr_tbl_rd2.attr,
        &dev_attr_tbl_rd3.attr,
        &dev_attr_i2c0.attr,
        &dev_attr_i2c1.attr,
        &dev_attr_i2c2.attr,
        &dev_attr_i2c3.attr,
        &dev_attr_i2c_frame0.attr,
        &dev_attr_i2c_frame1.attr,
        &dev_attr_i2c_frame2.attr,
        &dev_attr_i2c_frame3.attr,
        &dev_attr_i2c_speed.attr,
        &dev_attr_max_unbalanced_writes.attr,
        &dev_attr_help.attr,
        NULL
};

/** Attribute group for the root node (directory) of the sysfs representation for sensor-port i2c devices*/
static const struct attribute_group dev_attr_root_group = {
        .attrs = root_dev_attrs, ///< Array of the items in the root directory
        .name  = NULL,           ///< No name for the root node
};

/** Add a named group (subdirectory) with the specified name*/
static int make_group (struct device *dev, ///< Linux kernel basic device structure
        const char * name)  ///< Group name
///< @return 0 on success, or negative error.
{
    int retval=-1;
    struct attribute *pattrs[max_i2c_classes]; /* array of pointers to attibutes */
    struct attribute_group *attr_group;
    attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
    if (!attr_group) return -ENOMEM;
    pattrs[0] = NULL;
    attr_group->name  = name;
    attr_group->attrs =pattrs;
    dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
    if (&dev->kobj) {
        retval = sysfs_create_group(&dev->kobj, attr_group);
    }
    return retval;
}

/** Register this driver to implement sysfs interface */
static int elphel393_sens_i2c_sysfs_register(struct platform_device *pdev) ///< Platform device structure for this driver
///< @return 0 on success, or negative error.
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
        dev_dbg(dev,"sysfs_create_group(dev_attr_root_group) done \n");
        if (((retval = make_group (dev, group_name)))<0) return retval;
        dev_dbg(dev,"sysfs_create_group(%s) done \n",group_name);

    }
    return retval;
}

/** Initialize this driver from the Device Tree.
 * Configures some i2c device classes specified in the DT. */
static void elphel393_sensor_i2c_init_of(struct platform_device *pdev) ///< Platform device structure for this driver
///< @return 0 on success, or negative error.
{
    const   char * config_string;
    struct x393_i2c_device_list * dl;
    char name[32];
    int rslt;
    int num_devs, nd, ni, sa7, num_addr, num_data, khz;
    struct device_node *node = pdev->dev.of_node;
    struct device_attribute *new_attr;
    struct device *dev =&pdev->dev;
    if (node) {
        /*TODO: Configure some i2c devices here (slaves, formats, speeds) to be used by names*/
        num_devs = of_property_count_strings(node,sensor_i2c_of_prop_name);
        for (nd=0; nd <num_devs; nd++){
            if (of_property_read_string_index(node, sensor_i2c_of_prop_name, nd, &config_string)) {
                pr_err("%s: No data for selected i2c device\n", __func__);
                BUG();
            }
            ni = sscanf(config_string,"%31s %i %i %i %i", name, &sa7, &num_addr, &num_data, &khz);
            if (ni <5){
                pr_err("%s: wrong number of items (%d) in %s, 5 expected\n", __func__, ni, config_string);
                BUG();
            }
            dl = i2c_dev_add (name);
            // create sysfs group member
            //group_name
            new_attr =  devm_kzalloc(dev, sizeof(new_attr[0]), GFP_KERNEL);
            if (!new_attr) {
                pr_err("%s: failed to allocate memory for %s\n", __func__,name);
                return;
            }
            new_attr->attr.name =  devm_kzalloc(dev, strlen(name)+1, GFP_KERNEL);
            strcpy(new_attr->attr.name, (const char *)name);
            new_attr->attr.mode = SYSFS_PERMISSIONS;
            new_attr->show =      i2c_member_show;
            new_attr->store =     i2c_member_store;
            if (&dev->kobj) {
                if ((rslt = sysfs_add_file_to_group (
                        &dev->kobj,
                        &new_attr -> attr, // const struct attribute * attr,
                        group_name)))       // const char * group);
                {
                    dev_err(dev,"i2c_class_store() failed to add %s to group %s, returned %d\n",name, group_name, rslt);
                    return;
                }
            }
            if (dl) {
                dl-> i2c_dev.slave7 =        sa7;
                dl-> i2c_dev.address_bytes = num_addr;
                dl-> i2c_dev.data_bytes =    num_data;
                dl-> i2c_dev.scl_khz =       khz;

            }
        }
    }
    pr_info("elphel393_sensor_i2c: registered %d i2c device classes\n", num_devs);
}

/** Probe this driver, register it with sysfs and initialize from the Device Tree */
static int elphel393_sensor_i2c_probe(struct platform_device *pdev) ///< Platform device structure for this driver
///< @return 0 on success, or negative error.
{
    int chn;
    sdev =&pdev->dev;
    dev_dbg(&pdev->dev,"Probing elphel393-sensor-i2c\n");
    for (chn =0; chn<4;chn++) {
        i2c_page_alloc_init(chn);
    }
    dev_dbg(&pdev->dev,"i2c_page_alloc_init() done\n");

    elphel393_sens_i2c_sysfs_register(pdev);
    dev_dbg(&pdev->dev,"elphel393_sens_i2c_sysfs_register() done\n");

    elphel393_sensor_i2c_init_of(pdev);
    dev_dbg(&pdev->dev,"done probing elphel393-sensor-i2c\n");

    return 0;
}

/** Remove device driver - does nothing, just pretends.*/
static int elphel393_sensor_i2c_remove(struct platform_device *pdev) ///< Platform device structure for this driver
///< @return 0 on success, or negative error.
{
    dev_info(&pdev->dev,"Removing elphel393-sensor-i2c");
    return 0;
}
/** Compatible records in Device Tree for this driver */
static struct of_device_id elphel393_sensor_i2c_of_match[] = {
        { .compatible = "elphel,elphel393-sensor-i2c-1.00", }, ///< Compatibility data
        { /* end of table */}
};

MODULE_DEVICE_TABLE(of, elphel393_sensor_i2c_of_match);

/** Platform driver description of this driver */
static struct platform_driver elphel393_sensor_i2c = {
        .probe   = elphel393_sensor_i2c_probe,     ///< Function executed on probe operation
        .remove  = elphel393_sensor_i2c_remove,    ///< Function executed on remove operation
        .driver  = {                                         ///< Driver specifications
                .name  = DEV393_NAME(DEV393_I2C_SENSORS),        ///< driver name
                .owner = THIS_MODULE,                            ///< Driver owner
                .of_match_table = elphel393_sensor_i2c_of_match, ///< Device tree match data
                .pm = NULL,                                      ///< No power management supported in this driver
        },
};

module_platform_driver(elphel393_sensor_i2c);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("Elphel 10393 sensor ports i2c");
MODULE_LICENSE("GPL");
