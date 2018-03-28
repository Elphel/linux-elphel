/***************************************************************************//**
* @file      gamma_tables.c
* @brief     Handles "gamma"tables storage and scaling
*              exposes device driver to manipulate custom "gamma" tables
*              "Gamma" tables are calculated in several steps, leaving the
*              exponent calculation to the application, but handling scaling inside.
*              Scaling (with saturation if >1.0)is used for color balancing.
*
*              Gamma table calculation involves several intermediate tables:
*              - forward table
*              - reverse table (for histogram corrections)
*              - FPGA format
*              And the driver caches intermediate tables when possible, calculates
*              them when needed.
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


/*********************************************************************************
*   $Log: gamma_tables.c,v $
*   Revision 1.1.1.1  2008/11/27 20:04:01  elphel
*
*
*   Revision 1.19  2008/11/13 05:40:45  elphel
*   8.0.alpha16 - modified histogram storage, profiling
*
*   Revision 1.18  2008/10/29 04:18:28  elphel
*   v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*
*   Revision 1.17  2008/10/25 19:51:06  elphel
*   Changed word order in writes to gamma tables driver
*
*   Revision 1.16  2008/10/23 08:04:19  elphel
*   reenabling IRQ in debug mode
*
*   Revision 1.15  2008/10/12 16:46:22  elphel
*   snapshot
*
*   Revision 1.14  2008/10/06 08:31:08  elphel
*   snapshot, first images
*
*   Revision 1.13  2008/10/05 05:13:33  elphel
*   snapshot003
*
*   Revision 1.12  2008/10/04 16:10:12  elphel
*   snapshot
*
*   Revision 1.11  2008/09/22 22:55:48  elphel
*   snapshot
*
*   Revision 1.10  2008/09/20 00:29:50  elphel
*   moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
*
*   Revision 1.9  2008/09/19 04:37:25  elphel
*   snapshot
*
*   Revision 1.8  2008/09/16 00:49:32  elphel
*   snapshot
*
*   Revision 1.7  2008/09/12 00:23:59  elphel
*   removed cc353.c, cc353.h
*
*   Revision 1.6  2008/09/11 01:05:32  elphel
*   snapshot
*
*   Revision 1.5  2008/09/05 23:20:26  elphel
*   just a snapshot
*
*   Revision 1.4  2008/07/27 04:27:49  elphel
*   next snapshot
*
*   Revision 1.3  2008/06/16 06:51:21  elphel
*   work in progress, intermediate commit
*
*   Revision 1.2  2008/06/10 00:02:42  elphel
*   fast calculation of 8-bit reverse functions for "gamma" tables and histograms
*
*   Revision 1.1  2008/06/08 23:46:45  elphel
*   added drivers files for handling quantization tables, gamma tables and the histograms
*/

//copied from cxi2c.c - TODO:remove unneeded 
//#define DEBUG // should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

//#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/x393_devices.h>
#include <uapi/elphel/c313a.h>
#include <uapi/elphel/exifa.h>
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
#include "framepars.h"        // for debug mask
#include "sensor_common.h"  // for FPGA_TABLE_CHUNK - or move it?

//#include "fpga_io.h"//fpga_table_write_nice

//#include "cc3x3.h"
//#include "x3x3.h" // just for FPGA_NQTAB

//#include "framepars.h"
//#include "quantization_tables.h"
#include "gamma_tables.h"
#include "x393.h"
#include "detect_sensors.h"
#include "x393_fpga_functions.h" // to check bitstream

/**
 * @brief optional debug output 
 */
#if ELPHEL_DEBUG
#define MDF(x) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;}
#define D10(x) { if (GLOBALPARS(G_DEBUG) & (1 <<10)) { x ;} }
//  #define MDD1(x) printk("%s:%d:",__FILE__,__LINE__); x ; udelay (ELPHEL_DEBUG_DELAY)
#define MDF10(x) { if (GLOBALPARS(G_DEBUG) & (1 <<10)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
#define MDF11(x) { if (GLOBALPARS(G_DEBUG) & (1 <<11)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }

//  #define MDD1(x)
#define D1I(x) x

#else
#define MDF(x)
#define D10(x)
#define D1I(x) x
#define MDF10(x)
#define MDF11(x)


#endif
/** Combine color, sensor port and sub-channel into a single index
 * It is still possible to use "color" parameter in the range of 0..63 with port and channel set to 0  */
#define PORT_CHN_COLOR(color,port,chn) (((color) & 0x3f) | ((((port) & 3 ) << 4)) | ((((chn) & 3 ) << 2)))
#define  X3X3_GAMMAS_DRIVER_DESCRIPTION "Elphel (R) Model 353 Gamma Tables device driver"
/**
 * @brief number of different non-scaled tables in cache when it starts to overwrite non-scaled tables rather than scaled
 * \n TODO: use P_*?
 */
#define GAMMA_THRESH (GAMMA_CACHE_NUMBER/16)
/** @brief Global pointer to basic device structure. This pointer is used in debugfs output functions */
static struct device *g_dev_ptr;

static DEFINE_SPINLOCK(gamma_lock);   ///< Non port-specific lock
//static DEFINE_SPINLOCK(gamma_lock_0); ///<
//static DEFINE_SPINLOCK(gamma_lock_1); ///<
//static DEFINE_SPINLOCK(gamma_lock_2); ///<
//static DEFINE_SPINLOCK(gamma_lock_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
//spinlock_t * gamma_locks[4] = {&gamma_lock_0, &gamma_lock_1, &gamma_lock_2, &gamma_lock_3};

#ifdef USE_GAMMA_LOCK
    #define GAMMA_LOCK_BH(x)    spin_lock_bh(x)
    #define GAMMA_UNLOCK_BH(x)  spin_unlock_bh(x)
#else
    #define GAMMA_LOCK_BH(x)    {}
    #define GAMMA_UNLOCK_BH(x)  {}

#endif

static const char * const gamma_dev = DEV393_DEVNAME(DEV393_GAMMA);
static const int gamma_major = DEV393_MAJOR(DEV393_GAMMA);
static const int gamma_minor = DEV393_MINOR(DEV393_GAMMA);

/** @brief Global device class for sysfs */
static struct class *gamma_dev_class;

static struct gamma_stuct_t gammas[GAMMA_CACHE_NUMBER] __attribute__ ((aligned (PAGE_SIZE)));
struct gamma_stuct_t * gammas_p; // to use with mmap

struct gammas_pd {
    int                minor;
    unsigned short     scale;
    unsigned short     hash16;
    unsigned char      mode;
    unsigned char      color; // Does it need port/sub-channel?
    // something else to be added here?
};

int        gammas_open   (struct inode *inode, struct file *file);
int        gammas_release(struct inode *inode, struct file *file);
loff_t     gammas_lseek  (struct file * file, loff_t offset, int orig);
ssize_t    gammas_write  (struct file * file, const char * buf, size_t count, loff_t *off);
int        gammas_mmap   (struct file *file, struct vm_area_struct *vma);

//static int __init gammas_init(void);


/**
 * @brief remove item from non-scaled (top, "horizontal") chain
 * @param index item index to remove
 */
inline void remove_from_nonscaled(int index) {
    gammas[gammas[index].newer_non_scaled].older_non_scaled=gammas[index].older_non_scaled;
    gammas[gammas[index].older_non_scaled].newer_non_scaled=gammas[index].newer_non_scaled;
    gammas[0].non_scaled_length--;
}

/**
 * @brief remove item from scaled ("vertical") chain
 * @param index item index to remove
 */
inline void remove_from_scaled   (int index) {
    if (likely(gammas[index].newer_scaled)) { // will skip first, until the cache is all used
        gammas[gammas[index].newer_scaled].older_scaled=gammas[index].older_scaled;
        gammas[gammas[index].older_scaled].newer_scaled=gammas[index].newer_scaled;
    }
}

/**
 * @brief remove item from the all ("diagonal") chain
 * @param index item index to remove
 */
inline void remove_from_all      (int index) { // always - in that chain - after init
    gammas[gammas[index].newer_all].older_all=gammas[index].older_all;
    gammas[gammas[index].older_all].newer_all=gammas[index].newer_all;
}

/**
 * @brief insert item into non-scaled ("horizontal") chain as first
 * @param index item index to remove
 */
inline void insert_first_nonscaled(int index) {
    dev_dbg(g_dev_ptr,"insert_first_nonscaled(%d)\n",index);
    gammas[gammas[0].newest_non_scaled].newer_non_scaled= index; // 1 ==[0].oldest_non_scaled
    gammas[index].older_non_scaled=  gammas[0].newest_non_scaled; // 4
    gammas[0].newest_non_scaled=index; // 5
    gammas[index].newer_non_scaled=0; // 6
    gammas[0].non_scaled_length++;
    gammas[index].this_non_scaled=0; // none
    gammas[index].newest_scaled=index; // no scaled yet - point to itself
    gammas[index].oldest_scaled=index; // no scaled yet - point to itself

}

/**
 * @brief insert item into scaled ("vertical") chain as first
 * @param non_scaled index of non-scaled version of the node
 * @param index item index to remove
 */
inline void insert_first_scaled   (int non_scaled, int index) {
    dev_dbg(g_dev_ptr,"insert_first_scaled(%d, %d)\n",non_scaled, index);
    gammas[index].older_scaled=     gammas[non_scaled].newest_scaled; //4
    gammas[gammas[non_scaled].newest_scaled].newer_scaled= index; //1
    //    gammas[index].older_scaled=     gammas[non_scaled].newest_scaled; //4

    gammas[index].newer_scaled= non_scaled; //6
    gammas[non_scaled].newest_scaled=     index; //5
    //    gammas[index].newer_scaled= non_scaled; //6
    gammas[index].this_non_scaled=non_scaled;
}

/**
 * @brief insert item into "all" ("diagonal") chain as first
 * @param index item index to remove
 */
inline void insert_first_all      (int index) {
    gammas[gammas[0].newest_all].newer_all= index; //1
    gammas[index].older_all=     gammas[0].newest_all; //4
    gammas[0].newest_all=         index; //5
    gammas[index].newer_all= 0; //6
}

/**
 * @brief Initialize gamma tables data structures
 */
void init_gammas(void) {
    int i;
    gammas_p=gammas;
    // empty 2-d chain
    dev_dbg(g_dev_ptr,"init_gammas()\n");
    GAMMA_LOCK_BH(&gamma_lock);
    gammas[0].oldest_non_scaled=0;
    gammas[0].newest_non_scaled=0;
    // all entries in a same
    gammas[0].oldest_all=GAMMA_CACHE_NUMBER-1;
    gammas[0].newest_all=1;
    for (i=1; i < GAMMA_CACHE_NUMBER;i++) {
        gammas[i].this_non_scaled=-1; // no parent.FIXME: Where is it used? -1 if never used
        // something else?
        gammas[i].newer_all=i-1;
        gammas[i].older_all= (i==(GAMMA_CACHE_NUMBER-1))? 0: (i+1);
        gammas[i].locked=0;
        gammas[i].valid=0;
    }
    gammas[0].non_scaled_length=0;
    for (i=1; i < sizeof(gammas[0].locked_chn_color)/sizeof(gammas[0].locked_chn_color[0]);i++) { // 64
        gammas[0].locked_chn_color[i]=0;
    }
    GAMMA_UNLOCK_BH(&gamma_lock);
    dev_dbg(g_dev_ptr,"initialized %d port/channel/color gamma chains heads\n",i);
}

/**
 * @brief verifies that index is current (points to specified hash and scale)
 * @param hash16 - 16-bit unique hash for gamma table
 * @param scale  - 16-bit scale for gamma table (6.10, so GAMMA_SCLALE_10x400 is 1.0)
 * @param index  - gamma table index
 * @return 1 - table pointed by index is current, 0 - table is not current
 */
int is_gamma_current (unsigned short hash16, unsigned short scale, int index) {
    return ((gammas[index].hash16 == hash16) && (gammas[index].scale == scale))?1:0;
}

/**
 * @brief verifies that index is current and points to a valid table with specified hash and scale
 * @param hash16 - 16-bit unique hash for gamma table
 * @param scale  - 16-bit scale for gamma table (6.10, so GAMMA_SCLALE_1=0x400 is 1.0)
 * @param index  - gamma table index
 * @return 1 - table pointed by index is current and valid, 0 - table is not current or not valid
 */
int is_gamma_valid (unsigned short hash16, unsigned short scale, int index) {
    return ((gammas[index].hash16 == hash16) && (gammas[index].scale == scale) && (gammas[index].valid != 0))?1:0;
}


/** Looks for the hash32 last programmed to the FPGA for the particular color
 * @param color 
 * @return hash32 (combined gamma/black/scale) locked for a specified color. If none - return 0
 */
unsigned long get_locked_hash32(int color,         ///< color channel 0..3
        int sensor_port,   ///< sensor port number (0..3)
        int sensor_subchn) ///< sensor sub-channel (connected to the same port through 10359 mux) (0..3)
///< @return hash32 (combined gamma/black/scale) locked for a specified color,
///< port, sub-channel
{
    int index=gammas[0].locked_chn_color[PORT_CHN_COLOR(color,sensor_port,sensor_subchn)];
    return index?gammas[index].hash32:0;
}

/**
 * @brief Lock gamma table for the specified color/port/subchannel, save previous locks (if any) so new locks can be applied/canceled
 * NOTE: interrupts should be disabled!  */
inline void lock_gamma_node (int index,         ///< gamma table index
        int color,         ///< color channel 0..3
        int sensor_port,   ///< sensor port number (0..3)
        int sensor_subchn) ///< sensor sub-channel (connected to the same port through 10359 mux) (0..3)

{
    int tmp_p;
    int cps = PORT_CHN_COLOR(color,sensor_port,sensor_subchn);
    GAMMA_LOCK_BH(&gamma_lock);
    dev_dbg(g_dev_ptr,"color=0x%x, cps = 0x%x\n",color,cps);
    if (((tmp_p=gammas[0].locked_chn_color[cps]))!=0) { ///new gamma to the same color
        gammas[tmp_p].locked &= ~(1ULL << cps); // remove any previous lock on the same color (if any)
    }
    gammas[0].locked_chn_color[cps]= index;
    gammas[index].locked |= (1ULL << cps);
    GAMMA_UNLOCK_BH(&gamma_lock);
}
/** Unlock gamma table for the specified color/port/subchannel
 * NOTE: Not needed anymore */
int unlock_gamma_node  (int color,         ///< color channel 0..3
        int sensor_port,   ///< sensor port number (0..3)
        int sensor_subchn) ///< sensor sub-channel (connected to the same port through 10359 mux) (0..3)
///< @return wrong data -1, nothing to unlock - 0, >0 - unlocked index
{
    //  unsigned long flags;
    int index;
    int cps = PORT_CHN_COLOR(color,sensor_port,sensor_subchn);
    dev_dbg(g_dev_ptr,"color=0x%x, cps = 0x%x\n",color,cps);
    //  local_ irq_save(flags);
    GAMMA_LOCK_BH(&gamma_lock);
    index =gammas[0].locked_chn_color[cps];
    if (index) {
        gammas[index].locked &= ~(1ULL <<  color); // clear appropriate "locked" bit for this table
        gammas[0].locked_chn_color[color]=0;
    }
    GAMMA_UNLOCK_BH(&gamma_lock);
    //  local_irq_restore(flags);
    return index;
}
/** Find a gamma table in FPGA format to be programmed (table should already be locked for this color) */
unsigned long * get_gamma_fpga (int color,         ///< color channel 0..3
        int sensor_port,   ///< sensor port number (0..3)
        int sensor_subchn) ///< sensor sub-channel (connected to the same port through 10359 mux) (0..3)
///< @return pointer to a gamma table (or NULL if table does not exist)

{ // NOTE: Not needed anymore?
    int index;
    int cps = PORT_CHN_COLOR(color,sensor_port,sensor_subchn);
    //  if (unlikely((color>=4) || (color<0))) return NULL; //
    index =gammas[0].locked_chn_color[cps];
    dev_dbg(g_dev_ptr,"*** index=%d(0x%x)\n",index,index);
    if (index) return gammas[index].fpga;
    else return NULL;
}


/**
 * @brief Get a new node for gamma tables 
 * Find least recently used node (balancing between non-scaled and scaled), remove it from current chains and return
 * pointers in the returned node are not initialized, "valid" bit is cleared
 * NOTE: interrupts should be disabled ***
 * @return Node index or 0 if none (unlocked) nodes are found
 */
int gamma_new_node(void) {
    int tmp_p;
    if ((gammas[0].non_scaled_length > GAMMA_THRESH) && (gammas[gammas[0].oldest_non_scaled].newest_scaled == gammas[0].oldest_non_scaled)) { // no scaled for the oldest hash
        // sacrifice oldest hash
        tmp_p=gammas[0].oldest_non_scaled;
        remove_from_nonscaled(tmp_p);
    } else { // use oldest scaled
        tmp_p=gammas[0].oldest_all;
        // skip locked if any (should be unlikely to get any locked)
        while ((tmp_p!=0) && gammas[tmp_p].locked) tmp_p=gammas[tmp_p].newer_all;
        if (tmp_p==0) return 0; // none (unlocked) nodes are found
        // remove from "all" chain (should work for tmp_p being oldest or not
        remove_from_all   (tmp_p);
        // remove from "scaled chain"
        remove_from_scaled   (tmp_p);
    }
    gammas[tmp_p].valid=0;
    return tmp_p;
}

/**
 * @brief Hardware-dependent encoding of the FPGA "gamma" table.
 * @param gamma_in pointer to array of 257 16-bit values (only 10 msb-s are currently used)
 * @param gamma_out pointer to an array of 256 unsigned long words to be written to FPGA
 */
void gamma_encode_fpga(unsigned short * gamma_in, unsigned long * gamma_out) {
    int i,base,diff;
    dev_dbg(g_dev_ptr,"gamma_encode_fpga()\n");
    for (i=0;i<256;i++) {
        base=(gamma_in[i] >> 6);
        diff=(gamma_in[i+1] >> 6);
        diff-=base;
        if ((diff>63) || (diff < -64)) {
            diff=(diff+8)>>4;
            gamma_out[i]=(base & 0x3ff) | ((diff & 0x7f) << 10) | (1 << 17);
        } else {
            gamma_out[i]=(base & 0x3ff) | ((diff & 0x7f) << 10);
        }
    }
}

/**
 * @brief scale gamma table by (scale>>GAMMA_SCALE_SHIFT), saturate to 0..0xffff
 * @param scale scale to apply (1.0 ~ GAMMA_SCLALE_1=0x400)
 * @param gamma_in input (non-scaled) gamma table (16 bit)
 * @param gamma_out output (scaled) gamma table (16 bit) 
 */
void  gamma_calc_scaled (unsigned short scale,unsigned short * gamma_in, unsigned short * gamma_out) {
    int i;
    unsigned long d;
    unsigned long max_scaled=0xffff << GAMMA_SCALE_SHIFT;
    dev_dbg(g_dev_ptr,"gamma_calc_scaled(0x%x)\n",(int) scale);
    for (i=0; i<257; i++) {
        d= ((unsigned long) scale ) * ((unsigned long) gamma_in[i] ) + (1 <<(GAMMA_SCALE_SHIFT-1)); ///rounding, not truncating
        if (d>max_scaled) d=max_scaled;
        gamma_out[i]=d >> GAMMA_SCALE_SHIFT;
    }
}

/**
 * @brief calculate reverse gamma table (8-bit output)
 * Reverse gamma table restores 1-byte gamma-converted data (that is stored in the video DDR SDRAM by the FPGA)
 * to input data (in the 0..ffff range) TODO: should it be 0x7fff ?
 * calculates p(x), so that for any x (0<=x<=255) gamma(p(x))<=x*256, and gamma(p(x+1)) >x*256 (here gamma[256] is considered to be 0x10000
 * @param gamma_in direct gamma table (16 bit)
 * @param gamma_out reversed gamma table (8 bit)
 */
void  gamma_calc_reverse(unsigned short * gamma_in, unsigned char * gamma_out) {
    unsigned long gcurr=0; // running value to be compared against direct gamma
    int r=0; // current value of reverse gamma table
    int x=0; // current indedx of reverse gamma table
    dev_dbg(g_dev_ptr,"gamma_calc_reverse()\n");
    while ((r<256) && (x<256)) {
        gamma_out[x]=r;
        //    if ((r<255) && (( gamma_in[r]<<8) <= gcurr)) {
        if ((r<255) && ( gamma_in[r] <= gcurr)) {
            r++;
        } else {
            x++;
            gcurr+=256;
        }
    }
}

/** Calculate gamma table (and requested derivatives), insert new node if needed. */
int set_gamma_table (unsigned short hash16,        ///< 16-bit unique (non-scaled) gamma table identifier. Can be 1-byte gamma and 1-byte black level shift
        ///< TODO: make black level fine-grained?
        unsigned short scale,         ///< gamma table scale (currently 0x400 ~ 1.0)  GAMMA_SCLALE_1 = 0x400
        unsigned short * gamma_proto, ///< 16-bit gamma table prototype (or NULL)
        unsigned char mode,           ///< bits specify calculation mode:
        ///< - 1 - if set, no interrupts will be enabled between steps, whole operation will be atomic
        ///<- 2 - calculate reverse gamma table
        ///<- 4 - calculate FPGA-format gamma table
        ///<  - 8 - Lock (FPGA) table for specified color/port/subchannel
        int color,                    ///< index (0..63) combined with the next two parameters to lock
        ///< table for (if mode bit 4 is set), otherwise color, sensor_port, sensor_subchn are ignored
        int sensor_port,              ///< sensor port number (0..3)
        int sensor_subchn)            ///< sensor sub-channel (connected to the same port through 10359 mux) (0..3)
///< @return index for the specified table or 0 if none exists and prototype was not provided (gamma_proto==NULL)
{
    //  D1I(unsigned long flags);
    int tmp_p, tmp_p1; //,tmp_p0;
    int cps=PORT_CHN_COLOR(color,sensor_port,sensor_subchn);
    unsigned short gamma_linear[257]=
    {0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700,0x0800,0x0900,0x0a00,0x0b00,0x0c00,0x0d00,0x0e00,0x0f00,
            0x1000,0x1100,0x1200,0x1300,0x1400,0x1500,0x0600,0x1700,0x1800,0x1900,0x1a00,0x1b00,0x1c00,0x1d00,0x1e00,0x1f00,
            0x2000,0x2100,0x2200,0x2300,0x2400,0x2500,0x0600,0x2700,0x2800,0x2900,0x2a00,0x2b00,0x2c00,0x2d00,0x2e00,0x2f00,
            0x3000,0x3100,0x3200,0x3300,0x3400,0x3500,0x0600,0x3700,0x3800,0x3900,0x3a00,0x3b00,0x3c00,0x3d00,0x3e00,0x3f00,
            0x4000,0x4100,0x4200,0x4300,0x4400,0x4500,0x0600,0x4700,0x4800,0x4900,0x4a00,0x4b00,0x4c00,0x4d00,0x4e00,0x4f00,
            0x5000,0x5100,0x5200,0x5300,0x5400,0x5500,0x0600,0x5700,0x5800,0x5900,0x5a00,0x5b00,0x5c00,0x5d00,0x5e00,0x5f00,
            0x6000,0x6100,0x6200,0x6300,0x6400,0x6500,0x0600,0x6700,0x6800,0x6900,0x6a00,0x6b00,0x6c00,0x6d00,0x6e00,0x6f00,
            0x7000,0x7100,0x7200,0x7300,0x7400,0x7500,0x0600,0x7700,0x7800,0x7900,0x7a00,0x7b00,0x7c00,0x7d00,0x7e00,0x7f00,
            0x8000,0x8100,0x8200,0x8300,0x8400,0x8500,0x0600,0x8700,0x8800,0x8900,0x8a00,0x8b00,0x8c00,0x8d00,0x8e00,0x8f00,
            0x9000,0x9100,0x9200,0x9300,0x9400,0x9500,0x0600,0x9700,0x9800,0x9900,0x9a00,0x9b00,0x9c00,0x9d00,0x9e00,0x9f00,
            0xa000,0xa100,0xa200,0xa300,0xa400,0xa500,0x0600,0xa700,0xa800,0xa900,0xaa00,0xab00,0xac00,0xad00,0xae00,0xaf00,
            0xb000,0xb100,0xb200,0xb300,0xb400,0xb500,0x0600,0xb700,0xb800,0xb900,0xba00,0xbb00,0xbc00,0xbd00,0xbe00,0xbf00,
            0xc000,0xc100,0xc200,0xc300,0xc400,0xc500,0x0600,0xc700,0xc800,0xc900,0xca00,0xcb00,0xcc00,0xcd00,0xce00,0xcf00,
            0xf000,0xd100,0xd200,0xd300,0xd400,0xd500,0x0600,0xd700,0xd800,0xd900,0xda00,0xdb00,0xdc00,0xdd00,0xde00,0xdf00,
            0xe000,0xe100,0xe200,0xe300,0xe400,0xe500,0x0600,0xe700,0xe800,0xe900,0xea00,0xeb00,0xec00,0xed00,0xee00,0xef00,
            0xf000,0xf100,0xf200,0xf300,0xf400,0xf500,0x0600,0xf700,0xf800,0xf900,0xfa00,0xfb00,0xfc00,0xfd00,0xfe00,0xff00,
            0xffff};
    dev_dbg(g_dev_ptr, "hash16=0x%x scale=0x%x gamma_proto=0x%x mode =0x%x port/channel/color=%x\n", (int) hash16, (int) scale, (int) gamma_proto,  (int) mode,  cps);

    if (!gamma_proto & (hash16==0)) {
        gamma_proto=gamma_linear;
        dev_dbg(g_dev_ptr, "Using linear table\n");
    } else {
        dev_dbg(g_dev_ptr, "Using non-linear table\n"); ///NOTE: here
    }
    ///disable interrupts here
    //  D1I(local_ irq_save(flags));
    GAMMA_LOCK_BH(&gamma_lock);
    // look for the matching hash
    tmp_p=gammas[0].newest_non_scaled;
    dev_dbg(g_dev_ptr,"gammas[0].oldest_all=%d\n", gammas[0].oldest_all); ///NOTE: 253

    dev_dbg(g_dev_ptr,"gammas[0].newest_all=%d\n", gammas[0].newest_all);
    dev_dbg(g_dev_ptr,"tmp_p=0x%x gammas[tmp_p].hash16=0x%x hash16=0x%x\n", tmp_p, (int) gammas[tmp_p].hash16, (int) hash16 );
    while ((tmp_p!=0) && (gammas[tmp_p].hash16 != hash16)) {
        dev_dbg(g_dev_ptr," --tmp_p=0x%x\n", tmp_p); ///NOTE: never
        tmp_p=gammas[tmp_p].older_non_scaled;
    }
    dev_dbg(g_dev_ptr,"tmp_p=0x%x\n", tmp_p); ///NOTE: 0xff
    // Got right hash?
    if (tmp_p == 0) { // no luck
        dev_dbg(g_dev_ptr,"Need new table\n"); ///NOTE: never
        if (!gamma_proto) { //
            GAMMA_UNLOCK_BH(&gamma_lock);
            //          D1I(local_irq_restore(flags));
            dev_dbg(g_dev_ptr,"matching hash not found, new table is not provided\n"); ///NOTE: never
            return 0;   // matching hash not found, new table is not provided - return 0;
        }
        // Create new proto table
        tmp_p=gamma_new_node();
        dev_dbg(g_dev_ptr,"tmp_p=0x%x gamma_proto= \n0x000: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                    "0x008: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                    "...\n"
                                                    "0x0f8: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                    "0x100: 0x%04x\n"
                              , tmp_p, (int) gamma_proto[  0],(int) gamma_proto[  1], (int) gamma_proto[  2], (int) gamma_proto[  3],
                                       (int) gamma_proto[  4],(int) gamma_proto[  5], (int) gamma_proto[  6], (int) gamma_proto[  7],
                                       (int) gamma_proto[  8],(int) gamma_proto[  9], (int) gamma_proto[ 10], (int) gamma_proto[ 11],
                                       (int) gamma_proto[ 12],(int) gamma_proto[ 13], (int) gamma_proto[ 14], (int) gamma_proto[ 15],
                                       (int) gamma_proto[248],(int) gamma_proto[249], (int) gamma_proto[250], (int) gamma_proto[251],
                                       (int) gamma_proto[252],(int) gamma_proto[253], (int) gamma_proto[254], (int) gamma_proto[255],
                                       (int) gamma_proto[256]);
        if (unlikely(!tmp_p)) { // could not allocate node
            //          D1I(local_irq_restore(flags));
            GAMMA_UNLOCK_BH(&gamma_lock);
            return 0;   // failure: could not allocate node - return 0;
        }
        // fill it:
        gammas[tmp_p].hash16=hash16;
        gammas[tmp_p].scale=0;
        gammas[tmp_p].oldest_scaled=tmp_p;  // points to itself - no scaled versions yet
        gammas[tmp_p].newest_scaled=tmp_p;  // points to itself - no scaled versions yet
        if ((mode & GAMMA_MODE_NOT_NICE)==0) {
            // let interrupts to take place, and disable again - not needed with a tasklet
            //          D1I(local_irq_restore(flags));
            //          MDF10(printk("Interrupts reenabled, tmp_p=0x%x\n", tmp_p));
            //          D1I(local_ irq_save(flags));
            // check if it is still there (likely so, but allow it to fail).
            if (unlikely(!is_gamma_current (hash16, 0, tmp_p))) {
                //              D1I(local_irq_restore(flags));
                GAMMA_UNLOCK_BH(&gamma_lock);
                dev_dbg(g_dev_ptr,"failure: other code used this node - return 0; (try not_nice next time?), tmp_p = 0x%x\n",tmp_p);
                return 0;   // failure: other code used this node - return 0; (try not_nice next time?)
            }
        }
        //    memcpy ((void *)...
        memcpy (gammas[tmp_p].direct, gamma_proto, 257*2) ; ///copy the provided table (full 16 bits)
        gammas[tmp_p].valid |= GAMMA_VALID_MASK;
        // add it to the chain
        insert_first_nonscaled(tmp_p);
        dev_dbg(g_dev_ptr,"insert_first_nonscaled(0x%x)\n", tmp_p);
        // matching hash found,make it newest (remove from the chain + add to the chain)
    } else  if (gammas[tmp_p].newer_non_scaled !=0) { // if 0 - it is already the newest
        remove_from_nonscaled (tmp_p);
        insert_first_nonscaled(tmp_p);
        dev_dbg(g_dev_ptr,"remove_from_nonscaled(0x%x), insert_first_nonscaled (0x%x)\n", tmp_p, tmp_p);///NOTE: 0xff
    }
    dev_dbg(g_dev_ptr,"tmp_p= 0x%x\n", tmp_p); ///NOTE: 0xff
    // now looking for the correct scale.
    if (scale==0) {
        //      D1I(local_irq_restore(flags));
        GAMMA_UNLOCK_BH(&gamma_lock);
        dev_dbg(g_dev_ptr,"wanted non-scaled, got it: 0x%x\n",tmp_p);
        return tmp_p;   // wanted non-scaled, got it ///NOTE: returns here
    }
    tmp_p1=gammas[tmp_p].newest_scaled;
    dev_dbg(g_dev_ptr,"tmp_p1=0x%x\n", tmp_p1);  ///FIXME: 0xff
    while ((tmp_p1!=tmp_p) && (gammas[tmp_p1].scale != scale)){ ///FIXME: got stuck here
        dev_dbg(g_dev_ptr," >>tmp_p1=0x%x)\n", tmp_p1);
        tmp_p1=gammas[tmp_p1].older_scaled;
    }
    // Got right scale?
    //  if (tmp_p1 == 0) { // no luck
    if (tmp_p1 == tmp_p) { // no luck
        dev_dbg(g_dev_ptr,"create new scaled table\n");
        // create new scale
        tmp_p1=gamma_new_node();
        if (unlikely(!tmp_p1)) { // could not allocate node
            //          D1I(local_irq_restore(flags));
            GAMMA_UNLOCK_BH(&gamma_lock);
            dev_dbg(g_dev_ptr,"failure: could not allocate node - return 0\n");
            return 0;   // failure: could not allocate node - return 0;
        }
        // fill it
        gammas[tmp_p1].hash16=hash16;
        gammas[tmp_p1].scale= scale;
        // insert into 2-d
        insert_first_scaled   (tmp_p, tmp_p1);
        // insert into 1-d (all)
        insert_first_all (tmp_p1);
        if ((mode & GAMMA_MODE_NOT_NICE)==0) {
            // let interrupts to take place, and disable again - not needed with tasklets
            //          D1I(local_irq_restore(flags));
            //          D1I(local_ irq_save(flags));
            // check if it is still there (likely so, but allow it to fail).
            if (unlikely(!is_gamma_current (hash16, scale, tmp_p1))) {
                //              D1I(local_irq_restore(flags));
                GAMMA_UNLOCK_BH(&gamma_lock);
                dev_dbg(g_dev_ptr,"failure: other code used this node - return 0; (try not_nice next time?), tmp_p = 0x%x\n",tmp_p);
                return 0;   // failure: other code used this node - return 0; (try not_nice next time?)
            }
        }
    } else { // scaled table already exists, make it first in 2 chains:
        dev_dbg(g_dev_ptr,"reuse scaled table\n");
        // found right scale, make it newest in two chain (2d - hash/scale and 1-d - all scaled together, regardless of the hash
        ///2-d chain
        if (gammas[tmp_p1].newer_scaled != tmp_p) { // not already the newest of scales for the same hash
            remove_from_scaled    (tmp_p1);
            insert_first_scaled   (tmp_p, tmp_p1);
        }
        ///1-d chain
        if (gammas[tmp_p1].newer_all != 0) { // not already the newest from all scaled
            remove_from_all       (tmp_p1);
            insert_first_all      (tmp_p1);
        }
    }
    // is the scaled version already calculated?
    if ((gammas[tmp_p1].valid & GAMMA_VALID_MASK) == 0) {
        // calculate scaled version
        gamma_calc_scaled (scale, gammas[tmp_p].direct, gammas[tmp_p1].direct);
        gammas[tmp_p1].valid |= GAMMA_VALID_MASK;
        dev_dbg(g_dev_ptr,"gammas[0x%0x].direct= \n0x000: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                  "0x008: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                  "...\n"
                                                  "0x0f8: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n"
                                                  "0x100: 0x%04x\n"
                             , tmp_p1, (int) gammas[tmp_p1].direct[  0],(int) gammas[tmp_p1].direct[  1], (int) gammas[tmp_p1].direct[  2], (int) gammas[tmp_p1].direct[  3],
                                       (int) gammas[tmp_p1].direct[  4],(int) gammas[tmp_p1].direct[  5], (int) gammas[tmp_p1].direct[  6], (int) gammas[tmp_p1].direct[  7],
                                       (int) gammas[tmp_p1].direct[  8],(int) gammas[tmp_p1].direct[  9], (int) gammas[tmp_p1].direct[ 10], (int) gammas[tmp_p1].direct[ 11],
                                       (int) gammas[tmp_p1].direct[ 12],(int) gammas[tmp_p1].direct[ 13], (int) gammas[tmp_p1].direct[ 14], (int) gammas[tmp_p1].direct[ 15],
                                       (int) gammas[tmp_p1].direct[248],(int) gammas[tmp_p1].direct[249], (int) gammas[tmp_p1].direct[250], (int) gammas[tmp_p1].direct[251],
                                       (int) gammas[tmp_p1].direct[252],(int) gammas[tmp_p1].direct[253], (int) gammas[tmp_p1].direct[254], (int) gammas[tmp_p1].direct[255],
                                       (int) gammas[tmp_p1].direct[256]);
    }
    if (mode & GAMMA_MODE_HARDWARE) {
        // is hardware-encoded array already calculated (do it if not)?
        if ((gammas[tmp_p1].valid & GAMMA_FPGA_MASK)==0) {
            gamma_encode_fpga(gammas[tmp_p1].direct, gammas[tmp_p1].fpga);
            gammas[tmp_p1].valid |= GAMMA_FPGA_MASK;
            dev_dbg(g_dev_ptr,"gammas[0x%0x].fpga= \n0x000: 0x%05x 0x%05x 0x%05x 0x%05x 0x%5x 0x%05x 0x%05x 0x%05x\n"
                                                    "0x008: 0x%05x 0x%05x 0x%05x 0x%05x 0x%5x 0x%05x 0x%05x 0x%05x\n"
                                                    "...\n"
                                                    "0x0f8: 0x%05x 0x%05x 0x%05x 0x%05x 0x%5x 0x%05x 0x%05x 0x%05x\n"
                                 , tmp_p1, (int) gammas[tmp_p1].fpga[  0],(int) gammas[tmp_p1].fpga[  1], (int) gammas[tmp_p1].fpga[  2], (int) gammas[tmp_p1].fpga[  3],
                                           (int) gammas[tmp_p1].fpga[  4],(int) gammas[tmp_p1].fpga[  5], (int) gammas[tmp_p1].fpga[  6], (int) gammas[tmp_p1].fpga[  7],
                                           (int) gammas[tmp_p1].fpga[  8],(int) gammas[tmp_p1].fpga[  9], (int) gammas[tmp_p1].fpga[ 10], (int) gammas[tmp_p1].fpga[ 11],
                                           (int) gammas[tmp_p1].fpga[ 12],(int) gammas[tmp_p1].fpga[ 13], (int) gammas[tmp_p1].fpga[ 14], (int) gammas[tmp_p1].fpga[ 15],
                                           (int) gammas[tmp_p1].fpga[248],(int) gammas[tmp_p1].fpga[249], (int) gammas[tmp_p1].fpga[250], (int) gammas[tmp_p1].fpga[251],
                                           (int) gammas[tmp_p1].fpga[252],(int) gammas[tmp_p1].fpga[253], (int) gammas[tmp_p1].fpga[254], (int) gammas[tmp_p1].fpga[255]);
        }
    }
    if (mode & GAMMA_MODE_LOCK) {
        // lock the node for the color/port/channel
        lock_gamma_node (tmp_p1, color, sensor_port,sensor_subchn);
    }
    if (mode & GAMMA_MODE_NEED_REVERSE) {
        if ((gammas[tmp_p1].valid & GAMMA_VALID_REVERSE)==0) {
            if ((mode & GAMMA_MODE_NOT_NICE)==0) {
                // let interrupts to take place, and disable again // not needed with tasklets
                //              D1I(local_ irq_restore(flags));
                //              D1I(local_ irq_save(flags));
                // check if it is still there (likely so, but allow it to fail).
                if (unlikely(!is_gamma_current (hash16, 0, tmp_p))) {
                    //                  D1I(local_irq_restore(flags));
                    GAMMA_UNLOCK_BH(&gamma_lock);
                    return 0;   // failure: other code used this node - return 0; (try not_nice next time?)
                }
            }
            gamma_calc_reverse(gammas[tmp_p1].direct, gammas[tmp_p1].reverse);
            gammas[tmp_p1].valid |= GAMMA_VALID_REVERSE;
        }
    }
    //  D1I(local_irq_restore(flags));
    GAMMA_UNLOCK_BH(&gamma_lock);

    dev_dbg(g_dev_ptr,"set_gamma_table(): return %d\n",tmp_p1);
    return tmp_p1;
}

/** Writing gamma table to FPGA (1 color, 1 sub-channel) enabling IRQ after transferring each FPGA_TABLE_CHUNK DWORDs
* This function is only called from tasklet context, no extra locking is required */

int fpga_gamma_write_nice(int color,         ///< Color (0..3)
        int sensor_port,   ///< sensor port (0..3)
        int sensor_subchn, ///< sensor sub-channel (when several are connected through a multiplexer)
        unsigned long * gamma)       ///< Gamma table (256 DWORDs) in encoded FPGA format
                           ///< @return 0 OK, -ENODEV - FPGA is not programmed
{
    x393_gamma_tbl_t gamma_tbl_a = {.d32=0};
    x393_gamma_tbl_t gamma_tbl_d = {.d32=0};
    const int gamma_size=256; // 18-bit entries
    int addr32, len32, i;
    if (is_fpga_programmed()<=0){
        return -ENODEV;
    }

    gamma_tbl_a.a_n_d = 1;
    gamma_tbl_a.color = color;
    gamma_tbl_a.sub_chn = sensor_subchn;
//    GAMMA_LOCK_BH(gamma_locks[sensor_port]);
    for (addr32 = 0; addr32 < gamma_size; addr32 += FPGA_TABLE_CHUNK){
        len32 = FPGA_TABLE_CHUNK;
        if (unlikely(addr32 + len32 > gamma_size))
            len32 = gamma_size - addr32;
        gamma_tbl_a.addr= addr32;
        x393_sens_gamma_tbl(gamma_tbl_a, sensor_port);
        for (i = addr32; i < addr32 + len32; i++){
            gamma_tbl_d.d32 = gamma[i];
            x393_sens_gamma_tbl(gamma_tbl_d, sensor_port);
        }
    }
//    GAMMA_UNLOCK_BH(gamma_locks[sensor_port]);
    return 0;
}

///======================================
// File operations:
// open, release - nop
// read - none
// write should be a single call (with or without actual table), file pointer after write is result node index (0 - failure)
// returns - full length passed or 0 if failed
// write -> set_gamma_table: first 2 bytes [0.1] - table hash - (i.e. gamma | (black << 8)),
//                           next 2 bytes  [2.3] - scale (0..0xffff),
//                           next 1 byte   [4]   - mode (1 - not_nice, 2 - need reverse, 4 - hardware, 8 - lock)
//                           next byte     [5]   - color only if lock bit in mode is set
//                           next 514 bytes [6..519] - 16-bit gamma table

// can use current file pointer or special indexes (0x****ff01 - set frame number, 0x****ff02 - set latency) that should come before actual parameters
// file pointer - absolute frame number
// lseek (SEEK_SET, value) - do nothing, return 0
// lseek (SEEK_CUR, value) - ignore value, return last write result (and if it is still valid) - used by ftell
// lseek (SEEK_END, value <= 0) - do nothing?, do not modify file pointer
// lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer
// lseek (SEEK_END, 1) -          initialize all the gamma data structures
// lseek (SEEK_END, 2) -          check that current hash/scale/index are still current
// mmap (should be used read only)
//#define LSEEK_GAMMA_INIT        1 // SEEK_END LSEEK_GAMMA_INIT to initialize all the gamma data structures
//#define LSEEK_GAMMA_ISCURRENT   2 // SEEK_END to check if the selected node(pointed by file pointer) is current - returns 0 if not, otherwise - node index
/**
 * @brief File size reported by gamma device driver
 */
#define GAMMA_FILE_SIZE GAMMA_CACHE_NUMBER
static struct file_operations gammas_fops = {
        owner:    THIS_MODULE,
        llseek:   gammas_lseek,
        write:    gammas_write,
        open:     gammas_open,
        mmap:     gammas_mmap,
        release:  gammas_release
};
/**
 * @brief Gammas driver OPEN method
 * @param inode  inode
 * @param file   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int gammas_open(struct inode *inode, struct file *file) {
    int res;
    struct gammas_pd * privData;
    privData= (struct gammas_pd *) kmalloc(sizeof(struct gammas_pd),GFP_KERNEL);
    dev_dbg(g_dev_ptr,"gammas[0].oldest_all=%d\n", gammas[0].oldest_all);
    dev_dbg(g_dev_ptr,"gammas[0].newest_all=%d\n", gammas[0].newest_all);
    if (!privData) return -ENOMEM;
    file->private_data = privData;
    privData-> minor=MINOR(inode->i_rdev);
    dev_dbg(g_dev_ptr,"gammas_open, minor=0x%x\n",privData-> minor);
    switch (privData-> minor) {
    case DEV393_MINOR(DEV393_GAMMA) :
                inode->i_size = GAMMA_FILE_SIZE;
    privData-> scale= 0;
    privData-> hash16=0;
    privData-> mode= 0;
    return 0;
    default:
        kfree(file->private_data); // already allocated
        return -EINVAL;
    }
    file->f_pos = 0;
    return res;
}

/**
 * @brief Gammas driver RELEASE method
 * @param inode  inode
 * @param file   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int gammas_release(struct inode *inode, struct file *file) {
    int res=0;
    int p = MINOR(inode->i_rdev);
    MDF10(printk("gammas_release, minor=0x%x\n",p));
    switch ( p ) {
    case  DEV393_MINOR(DEV393_GAMMA) :
                break;
    default:
        return -EINVAL; //! do not need to free anything - "wrong number"
    }
    kfree(file->private_data);
    return res;
}


/**
 * @brief Gammas driver LSEEK  method (and execute commands)
 * - lseek (SEEK_SET, value) - do nothing, return 0 - replaced
 * - lseek (SEEK_SET, value) - set file position (cache page index) to offset
 * - lseek (SEEK_CUR, value) - ignore value, return last write result (and if it is still valid) - used by ftell
 * - lseek (SEEK_END, value < 0) - do nothing?, do not modify file pointer
 * - lseek (SEEK_END, value = 0) - set file pointer to GAMMA_CACHE_NUMBER
 * - lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer. Commands are:
 *   - LSEEK_GAMMA_INIT      - initialize gamma tables structures
 *   - LSEEK_GAMMA_ISCURRENT - verify that the gama table is current
 * @param file 
 * @param offset 
 * @param orig SEEK_SET, SEEK_CUR or SEEK_SET END
 * @return file position gamma cache index (0 - invalid)
 */
loff_t gammas_lseek (struct file * file, loff_t offset, int orig) {
    struct gammas_pd * privData = (struct gammas_pd *) file->private_data;
    dev_dbg(g_dev_ptr,"offset=0x%x, orig=0x%x, file->f_pos=0x%x\n",(int) offset, (int) orig, (int) file->f_pos);
    switch (privData->minor) {
    case DEV393_MINOR(DEV393_GAMMA) :
                 switch(orig) {
                 case SEEK_SET:
                     file->f_pos = offset;
                     break;
                 case SEEK_CUR:
                     //     file->f_pos = getThisFrameNumber() + offset;
                     break;
                 case SEEK_END:
                     if (offset < 0) {
                         break;
                     } else if (offset == 0) {
                         file->f_pos=GAMMA_CACHE_NUMBER;
                         break;
                     } else {//!Execute commands
                         switch (offset) {
                         case LSEEK_GAMMA_INIT:
                             init_gammas();
                             file->f_pos=0;
                             break;
                         case LSEEK_GAMMA_ISCURRENT:
                             if (file->f_pos==0) break; // wrong index
                             if (!is_gamma_current (privData->hash16, privData->scale, (int) file->f_pos)) file->f_pos=0;
                             break;
                         default: ///other commands
                             return -EINVAL;
                         }
                         break;
                     }
                 default:  // not SEEK_SET/SEEK_CUR/SEEK_END
                     return -EINVAL;
                 } // switch (orig)
    dev_dbg(g_dev_ptr,"file->f_pos=0x%x\n",(int) file->f_pos);
    return  file->f_pos ;
                 default: // other minors
                     return -EINVAL;
    }
}

/** Gammas driver WRITE method
 * write should be a single call (with or without actual table), file pointer after write is result node index (0 - failure)
 * write method receives data and uses it with \b set_gamma_table()
 * - first 2 bytes  [0.1] - scale (0..0xffff),
 * - next  2 bytes [2.3] - table hash - (i.e. gamma | (black << 8)),
 * - next 1 byte   [4]   - mode (1 - not_nice, 2 - need reverse, 4 - hardware, 8 - lock)
 * - next 1 byte   [5]   - color only if lock bit in mode is set. Note: could not find why it was >>3 - error?, seems never used. In 393 it holds {port[1:0],chn[1:0],color[1:0]}
 * - next 514 bytes [6..519] - 16-bit gamma table (if less than 514 bytes NULL will be passed to \b set_gamma_table()
 * sets file pointer to gamma cache index (0 - no table exist) */
ssize_t gammas_write(struct file * file, ///< this file structure
        const char * buf,   ///< userland buffer
        size_t count,       ///< number of bytes to write
        loff_t *off)        ///< updated offset in the buffer
///< @return full length passed or 0 if failed
{
    struct gammas_pd * privData = (struct gammas_pd *) file->private_data;
    struct {
        unsigned short scale;
        unsigned short hash16;
        unsigned char  mode;
        unsigned char  color; // 393: it is now combination of color, port and channel. Could not find what are the 3 LSBs ((privData->color >> 3) &3) below
        unsigned short gamma[257];
    } data;

    int     head,   result;
    // ************* NOTE: Never use file->f_pos in write() and read() !!!
    unsigned short * gamma= data.gamma;
    dev_dbg(g_dev_ptr," file->f_pos=0x%x, *off=0x%x\n", (int) file->f_pos, (int) *off);
    switch (privData->minor) {
    case DEV393_MINOR(DEV393_GAMMA) :
                 if (count>sizeof (data)) count = sizeof (data);
    if(count) {
        if(copy_from_user((char *) &data, buf, count))  return -EFAULT;
        head=6;
        if ((count-head) < (2 * 257)) gamma=NULL; // complete gamma table is not available
        if (head>count) head=count;
        memcpy (&(privData->scale),&(data.scale),head);
        dev_dbg(g_dev_ptr,"count=%d, head=%d, hash16=0x%x scale=0x%x mode=0x%x color=%x\n", count, head, (int) data.hash16, (int) data.scale,  (int) data.mode,  (int) data.color);
        dev_dbg(g_dev_ptr,"count=%d, head=%d, hash16=0x%x scale=0x%x mode=0x%x color=%x\n", count, head, (int) privData->hash16, (int) privData->scale,  (int) privData->mode,  (int) privData->color);
        //           result=set_gamma_table (privData->hash16, privData->scale, gamma,  privData->mode, ( privData->color >> 3) & 3);
        result=set_gamma_table (privData->hash16, privData->scale, gamma,  privData->mode, ( privData->color >> 0) & 3, ( privData->color >> 4) & 3, ( privData->color >> 2) & 3);
        *off= (result>0)?result:0;
    } else *off=0;
    dev_dbg(g_dev_ptr,"file->f_pos=0x%x\n", (int) *off);
    return (*off) ? count: 0;
    default:                           return -EINVAL;
    }
}

/**
 * @brief Gammas driver MMAP method (debug feature that gives access to gammas structures, should be used read only)
 * @param file 
 * @param vma 
 * @return OK - 0, negative - errors
 */
int gammas_mmap (struct file *file, struct vm_area_struct *vma) {
    int result;
    struct gammas_pd * privData = (struct gammas_pd *) file->private_data;
    dev_dbg(g_dev_ptr,"gammas_all_mmap, minor=0x%x\n",privData-> minor);
    switch (privData->minor) {
    case  DEV393_MINOR(DEV393_GAMMA) :
                   result=remap_pfn_range(vma,
                           vma->vm_start,
                           ((unsigned long) virt_to_phys(gammas_p)) >> PAGE_SHIFT, // Should be page-aligned
                           vma->vm_end-vma->vm_start,
                           vma->vm_page_prot);
    dev_dbg(g_dev_ptr,"remap_pfn_range returned=%x\n",result);
    if (result) return -EAGAIN;
    return 0;
    default: return -EINVAL;
    }
}

/**
 * @brief Gammas driver init
 * @return 0
 */
static int gammas_init(struct platform_device *pdev) {
    int res;
    struct device *dev = &pdev->dev;
    const struct of_device_id *match; // not yet used
    struct device *chrdev;

    dev_info(dev,"Starting "DEV393_NAME(DEV393_GAMMA)" - %d \n",DEV393_MAJOR(DEV393_GAMMA));
    init_gammas();
    //   MDF10(printk("set_gamma_table (0, GAMMA_SCLALE_1, NULL,  0, 0)\n"); udelay (ELPHEL_DEBUG_DELAY));
    set_gamma_table (0, GAMMA_SCLALE_1, NULL,  0, 0, 0, 0); // maybe not needed to put linear to cache - it can be calculated as soon FPGA will be tried to be programmed with
    // hash16==0

    res = register_chrdev(DEV393_MAJOR(DEV393_GAMMA), DEV393_NAME(DEV393_GAMMA), &gammas_fops);
    if(res < 0) {
        dev_err(dev,"\ngammas_init: couldn't get a major number %d.\n",DEV393_MAJOR(DEV393_GAMMA));
        return res;
    }
    //   init_waitqueue_head(&gammas_wait_queue);
    dev_info(dev, DEV393_NAME(DEV393_GAMMA)" - %d \n",DEV393_MAJOR(DEV393_GAMMA));

    // create device class
	gamma_dev_class = class_create(THIS_MODULE, DEV393_NAME(DEV393_GAMMA));
	if (IS_ERR(gamma_dev_class)) {
		pr_err("Cannot create \"%s\" class", DEV393_NAME(DEV393_GAMMA));
		return PTR_ERR(gamma_dev_class);
	}

	// create device
	chrdev = device_create(
			  gamma_dev_class,
			  &pdev->dev,
			  MKDEV(gamma_major, gamma_minor),
			  NULL,
			  "%s",gamma_dev);
	if(IS_ERR(chrdev)){
		pr_err("Failed to create a device (gamma). Error code: %ld\n",PTR_ERR(chrdev));
	}

    g_dev_ptr = dev; // to use for debug print

    return 0;
}
int gammas_remove(struct platform_device *pdev)
{
	device_destroy(gamma_dev_class, MKDEV(gamma_major, gamma_minor));
    unregister_chrdev(DEV393_MAJOR(DEV393_GAMMA), DEV393_NAME(DEV393_GAMMA));

    return 0;
}

static const struct of_device_id elphel393_gamma_tables_of_match[] = {
        { .compatible = "elphel,elphel393-gamma_tables-1.00" },
        { /* end of list */ }
};


MODULE_DEVICE_TABLE(of, elphel393_gamma_tables_of_match);


static struct platform_driver elphel393_gamma_tables = {
        .probe          = gammas_init,
        .remove         = gammas_remove,
        .driver         = {
                .name       = DEV393_NAME(DEV393_GAMMA),
                .of_match_table = elphel393_gamma_tables_of_match,
        },
};


module_platform_driver(elphel393_gamma_tables);
//module_init(gammas_init);
MODULE_LICENSE("GPLv3.0");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_GAMMAS_DRIVER_DESCRIPTION);
