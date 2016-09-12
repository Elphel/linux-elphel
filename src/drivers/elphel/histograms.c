/***************************************************************************//**
* @file      histograms.c
* @brief     Handles histograms storage, access and percentile calculation
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
/* -----------------------------------------------------------------------------**
*!  $Log: histograms.c,v $
*!  Revision 1.3  2009/02/18 06:25:59  elphel
*!  typo in format
*!
*!  Revision 1.2  2008/11/30 21:56:39  elphel
*!  Added enforcing limit on the overall gains in the color channels, storage of exposure and gains in the histograms cache (to be used with autoexposure/white balance)
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
*!
*!
*!  Revision 1.17  2008/11/14 07:08:11  elphel
*!  no request for the histogram if past histogram is needed and some exist in the past
*!
*!  Revision 1.16  2008/11/13 05:40:45  elphel
*!  8.0.alpha16 - modified histogram storage, profiling
*!
*!  Revision 1.15  2008/10/29 04:18:28  elphel
*!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*!
*!  Revision 1.14  2008/10/28 07:04:28  elphel
*!  driver returns histogram frame number (it lags one frame from the real frame number)
*!
*!  Revision 1.13  2008/10/25 19:59:48  elphel
*!  added lseek() calls to enable/disable daemons at events (compressed frame available, any frame available, histogram-Y and histograms-C available)
*!
*!  Revision 1.12  2008/10/23 18:26:14  elphel
*!  Fixed percentile calculations in histograms
*!
*!  Revision 1.11  2008/10/23 08:05:56  elphel
*!  added 2 wait queues for histogram data - separately for G1 (used as Y for autoexposure) and other color components (for color balancing and histogram display)
*!
*!  Revision 1.10  2008/10/12 16:46:22  elphel
*!  snapshot
*!
*!  Revision 1.9  2008/10/04 16:10:12  elphel
*!  snapshot
*!
*!  Revision 1.8  2008/09/20 00:29:50  elphel
*!  moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
*!
*!  Revision 1.7  2008/09/12 00:23:59  elphel
*!  removed cc353.c, cc353.h
*!
*!  Revision 1.6  2008/09/07 19:48:09  elphel
*!  snapshot
*!
*!  Revision 1.5  2008/09/05 23:20:26  elphel
*!  just a snapshot
*!
*!  Revision 1.4  2008/07/27 23:25:07  elphel
*!  next snapshot
*!
*!  Revision 1.3  2008/06/16 06:51:21  elphel
*!  work in progress, intermediate commit
*!
*!  Revision 1.2  2008/06/10 00:02:42  elphel
*!  fast calculation of 8-bit reverse functions for "gamma" tables and histograms
*!
*!  Revision 1.1  2008/06/08 23:46:45  elphel
*!  added drivers files for handling quantization tables, gamma tables and the histograms
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
#include <linux/platform_device.h>
#include <linux/spinlock.h>

//#include <linux/autoconf.h>
#include <linux/vmalloc.h>

//#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
// ##include <asm/dma-mapping.h>

#include <uapi/elphel/x393_devices.h>
#include <uapi/elphel/c313a.h>
#include <uapi/elphel/exifa.h>
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "fpga_io.h"//fpga_table_write_nice
#include "framepars.h"        // for debug mask
#include <elphel/elphel393-mem.h>
#include "x393.h"
#include "histograms.h"
#include "detect_sensors.h"
#include "x393_fpga_functions.h" // to check bitsteram
/**
 * \def MDF21(x) optional debug output 
 */

#if ELPHEL_DEBUG
// only when commands are issued
  #define MDF21(x) { if (GLOBALPARS(G_DEBUG) & (1 <<21)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
// includes tasklets
  #define MDF22(x) { if (GLOBALPARS(G_DEBUG) & (1 <<22)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
#else
  #define MDF21(x)
  #define MDF22(x)
#endif

//u32         (*fpga_hist_data)[SENSOR_PORTS][MAX_SENSORS][PARS_FRAMES][4][256]; ///< Array of histogram data, mapped to the memory wheer FPGA sends data
//u32        *fpga_hist_data[SENSOR_PORTS][MAX_SENSORS][PARS_FRAMES][4][256]; ///< Array of histogram data, mapped to the memory where FPGA sends data
//u32        (*fpga_hist_data)[SENSOR_PORTS][MAX_SENSORS][PARS_FRAMES][4][256]; ///< Array of histogram data, mapped to the memory where FPGA sends data
//typedef u32 fpga_hist_t [SENSOR_PORTS][MAX_SENSORS][PARS_FRAMES][4][256];
//typedef u32 fpga_hist_t [][MAX_SENSORS][PARS_FRAMES][4][256];

typedef u32 (*fpga_hist_t)[MAX_SENSORS][PARS_FRAMES][4][256];
fpga_hist_t fpga_hist_data;

dma_addr_t   fpga_hist_phys; // physical address of the start of the received histogram data

#define  X3X3_HISTOGRAMS_DRIVER_DESCRIPTION "Elphel (R) Model 353 Histograms device driver"

/** for each port and possible sensor subchannel provides index in combine histogram data */
int histograms_map[SENSOR_PORTS][MAX_SENSORS];

static DEFINE_SPINLOCK(histograms_init_lock); ///< do not start multiple threads of histogram structures initialization

/** total number of sensors (on all ports) used */
static int numHistChn = 0;

/** Variable-length array (length is the total number of active sensors <=16), each being the same as in 353:
 * consisting of SENSOR_PORTS histogram_stuct_t structures */
struct  histogram_stuct_t (*histograms)[HISTOGRAM_CACHE_NUMBER] = NULL;
//struct  histogram_stuct_t *histograms;

dma_addr_t histograms_phys; ///< likely not needed, saved during allocation

struct histogram_stuct_t * histograms_p; ///< alias of histogram_stuct_t
/** @brief Global pointer to basic device structure. This pointer is used in debugfs output functions */
static struct device *g_dev_ptr;

wait_queue_head_t hist_y_wait_queue;    ///< wait queue for the G1 histogram (used as Y)
wait_queue_head_t hist_c_wait_queue;    ///< wait queue for all the other (R,G2,B) histograms (color)
void init_histograms(int chn_mask); ///< combined subchannels and ports Save mask to global P-variable
int histograms_init_hardware(void);
static volatile int histograms_initialized = 0; ///< 0 - not initialized, 1 - structures only, 2 structures and hardware NC393: initialize when first used?
/** Check if histograms structures are initialized, initialize if not */
int histograms_check_init(void)
{
    int sensor_port, chn_mask=0;
    if (histograms_initialized > 1)
        return histograms_initialized;

    spin_lock(&histograms_init_lock);
    if (histograms_initialized > 1) {
        spin_unlock(&histograms_init_lock);
        return histograms_initialized;
    }
    if (!histograms_initialized){
        dev_dbg(g_dev_ptr, "need to initialize histograms structures");
        for (sensor_port = 0; sensor_port < SENSOR_PORTS; sensor_port++){
            chn_mask |= get_subchannels(sensor_port) << (MAX_SENSORS * sensor_port);
            dev_dbg(g_dev_ptr, "sensor_port=%d, chn_mask updated to 0x%x", sensor_port, chn_mask);
        }
        init_histograms(chn_mask);
        histograms_initialized = 1; // structures initialized
    }
    if (is_fpga_programmed() >0 ){ // do not try to access FPGA if it is not programmed
        dev_dbg(g_dev_ptr, "need to initialize histograms hardware");
        if (!histograms_init_hardware())
            histograms_initialized = 2; // fully initialized
    }
    dev_dbg(g_dev_ptr, "histograms_check_init() -> %d", histograms_initialized);
    spin_unlock(&histograms_init_lock);
    return histograms_initialized;
}

/** File operations private data */
struct histograms_pd {
    int                minor;
    unsigned long      frame;             ///< absolute frame number requested
    int                frame_index;       ///< histogram fame index (in cache and when accessing through mmap), -1 if invalid,
    int                needed;            ///< bits specify what histograms (color, type) are requested
                                          ///<  each group of 4 bits covers 4 colors of the same type:
                                          ///<  - bits 0..3 - read raw histograms from the FPGA - normally called from IRQ/tasklet (use just 1 color for autoexposure to speed up?)
                                          ///<  - bits 4..7 - calculate cumulative histograms (sum of raw ones) - normally called from applications
                                          ///<  - bits 8..11 - calculate percentiles (reverse cumulative histograms) - normally called from applications
                                          ///<  "needed" for raw histograms should be specified explicitly (can not be read from FPGA later),
                                          ///<  "needed" for cumul_hist will be added automatically if percentiles are requested
    int                wait_mode;         ///< 0 - wait just for G1 histogram, 1 - wait for all histograms
    int                request_en;        ///< enable requesting histogram for the specified frame (0 - rely on the available ones)
    int                port;              ///< selected sensor port (0..3)
    int                subchannel;        ///< selected sensor sub_channel(0..3)
    struct wait_queue *hist_y_wait_queue; ///< wait queue for the G1 histogram (used as Y)  ///NOTE: not used at all?
    struct wait_queue *hist_c_wait_queue; ///< wait queue for all the other (R,G2,B) histograms (color)  ///NOTE: not used at all?
// something else to be added here?
};


int        histograms_open   (struct inode *inode, struct file *file);
int        histograms_release(struct inode *inode, struct file *file);
loff_t     histograms_lseek  (struct file * file, loff_t offset, int orig);
int        histograms_mmap   (struct file *file, struct vm_area_struct *vma);


inline unsigned long  histogram_calc_cumul       ( unsigned long * hist,        unsigned long * cumul_hist );
inline void  histogram_calc_percentiles ( unsigned long * cumul_hist,  unsigned char * percentile);

/** Initialize FPGA DMA engine for histograms. Obviously requires bitstream to be loaded.
 * Histograms will be initialized for all possible ports/channels, only some will be enabled */
int histograms_init_hardware(void)
{
    int port, chn, hist_chn;
    x393_hist_saxi_addr_t saxi_addr = {.d32=0};
//    fpga_hist_data = (u32 *) pElphel_buf->histograms_vaddr; // d2h_vaddr; // must be page-aligned!
    fpga_hist_data = (fpga_hist_t) pElphel_buf->histograms_vaddr; // d2h_vaddr; // must be page-aligned!
    fpga_hist_phys = pElphel_buf->histograms_paddr; //d2h_paddr;
    for (port=0; port<SENSOR_PORTS; port++) for (chn=0; chn < MAX_SENSORS; chn++) {
        hist_chn = chn + MAX_SENSORS *port;
        saxi_addr.page=(fpga_hist_phys >> PAGE_SHIFT)+ PARS_FRAMES * hist_chn;// table for 4 colors is exactly 1 page;
        set_x393_hist_saxi_addr (saxi_addr, hist_chn);       // Histogram DMA addresses (in 4096 byte pages)
    }
    // Hand all data to device
    dma_sync_single_for_device(NULL, fpga_hist_phys, SENSOR_PORTS*MAX_SENSORS*PARS_FRAMES*4*256*4, DMA_FROM_DEVICE);

    histograms_dma_ctrl(2);
    return 0;
}

/** Reset/Enable/disable histograms DMA engine */
void histograms_dma_ctrl(int mode) ///< 0 - reset, 1 - disable, 2 - enable
{
    x393_hist_saxi_mode_t saxi_mode;
    saxi_mode.d32=    0;
    saxi_mode.cache=  3;
    saxi_mode.confirm=1;
    saxi_mode.nrst=   (mode)?  1:0;
    saxi_mode.en=     (mode>1)?1:0;
    set_x393_hist_saxi_mode(saxi_mode);
}


/** Initialize histograms data structures, should be called when active subchannels are known (maybe use DT)? */
//#define HISTOGRAMS_DISABLE_IRQ
void init_histograms(int chn_mask) ///< combined subchannels and ports Save mask to global P-variable
{
#ifdef HISTOGRAMS_DISABLE_IRQ
    unsigned long flags;
#endif
    int p,s,i, sz,pages;
    numHistChn = 0; //__builtin_popcount (chn_mask & 0xffff);
    for (p=0; p< SENSOR_PORTS; p++) for (s=0;s <MAX_SENSORS;s++) {
        i = p * SENSOR_PORTS + s;
        if (chn_mask & (1 << i)){
            histograms_map[p][s] = numHistChn++;
            GLOBALPARS(p, G_HIST_LAST_INDEX + s) =0; // mark as valid
            GLOBALPARS(p, G_SUBCHANNELS) |= 1 << s;
        } else {
            histograms_map[p][s] = -1;
            GLOBALPARS(p, G_HIST_LAST_INDEX + s) =0xffffffff; // mark as invalid
            GLOBALPARS(p, G_SUBCHANNELS) &= ~(1 << s);
        }
    }
    dev_dbg(g_dev_ptr, "Histograms structures, channel mask = 0x%x, numHistChn = 0x%x",chn_mask, numHistChn);

    //G_SUBCHANNELS
    sz = numHistChn * HISTOGRAM_CACHE_NUMBER * sizeof(struct histogram_stuct_t);
    pages = ((sz -1) >> PAGE_SHIFT)+1;
//    if (sz & (PAGE_SIZE-1)) pages++;
    // When device == NULL, dma_alloc_coherent just allocates notmal memory, page aligned, CMA if available
//    histograms = (struct  histogram_stuct_t* [HISTOGRAM_CACHE_NUMBER]) dma_alloc_coherent(NULL,(sz * PAGE_SIZE),&histograms_phys,GFP_KERNEL);
//    histograms = (struct  histogram_stuct_t[HISTOGRAM_CACHE_NUMBER] * ) dma_alloc_coherent(NULL,(sz * PAGE_SIZE),&histograms_phys,GFP_KERNEL);
//    histograms = (struct  histogram_stuct_t[HISTOGRAM_CACHE_NUMBER]) *  dma_alloc_coherent(NULL,(sz * PAGE_SIZE),&histograms_phys,GFP_KERNEL);


//    histograms = dma_alloc_coherent(NULL,(pages * PAGE_SIZE),&histograms_phys,GFP_KERNEL); // OK
//    dev_warn(g_dev_ptr, "dma_alloc_coherent(NULL, 0x%x, 0x%x,GFP_KERNEL)",pages * PAGE_SIZE, (int) histograms_phys);
// This code is spin-locked above to prevent simultaneous allocation from several php instances, so  GFP_KERNEL may not be used
//    histograms = dma_alloc_coherent(NULL,(pages * PAGE_SIZE),&histograms_phys,GFP_NOWAIT); // OK
//    dev_warn(g_dev_ptr, "dma_alloc_coherent(NULL, 0x%x, 0x%x,GFP_NOWAIT)",pages * PAGE_SIZE, (int) histograms_phys);

//    histograms = dma_alloc_coherent(NULL,(pages * PAGE_SIZE),&histograms_phys,GFP_ATOMIC); // OK
    dev_warn(g_dev_ptr, "dma_alloc_coherent should be done before (@probe), needed are 0x%lx bytes)",pages * PAGE_SIZE);
    BUG_ON(!histograms);
    histograms_p= (struct histogram_stuct_t *) histograms;
//    MDF21(printk("\n"));
#ifdef HISTOGRAMS_DISABLE_IRQ
    local_irq_save(flags);
#endif
    dev_dbg(g_dev_ptr, "Histograms structures, channel mask = 0x%x",chn_mask);
    for (s=0; s<numHistChn; s++) {
        for (i=0; i < HISTOGRAM_CACHE_NUMBER; i++) {
            histograms[s][i].frame=0xffffffff;
            histograms[s][i].valid=0;
        }
    }
#ifdef HISTOGRAMS_DISABLE_IRQ
    local_irq_restore(flags);
#endif
    dev_dbg(g_dev_ptr, "Histograms structures initialized");

}
/** Get histogram index for sensor port/channel, skipping unused ones */
int get_hist_index (int sensor_port,     ///< sensor port number (0..3)
                    int sensor_chn)      ///< sensor subchannel (0..3, 0 w/o multiplexer)
                                         ///< @return index of used (active) histogram set (just skipping unused ports/subchannels
{
      return histograms_map[sensor_port][sensor_chn];
}

/**
 * @brief Get histograms from the FPGA (called as tasklet - not anymore?)
 * TODO: should it be one frame behind current?
 * each group of 4 bits cover 4 colors of the same type */
int set_histograms  (int sensor_port,           ///< sensor port number (0..3)
                     int sensor_chn,            ///< sensor subchannel (0 w/o multiplexer)
                     unsigned long frame,       ///< absolute frame number (Caller should match it to the hardware frame)
                     int needed,                ///< bits specify what histograms (color, type) are requested
                                                ///<  each group of 4 bits covers 4 colors of the same type:
                                                ///<  - bits 0..3 - read raw histograms from the FPGA - normally called from IRQ/tasklet (use just 1 color for autoexposure to speed up?)
                                                ///<  - bits 4..7 - calculate cumulative histograms (sum of raw ones) - normally called from applications
                                                ///<  - bits 8..11 - calculate percentiles (reverse cumulative histograms) - normally called from applications
                                                ///<  "needed" for raw histograms should be specified explicitly (can not be read from FPGA later),
                                                ///<  "needed" for cumul_hist will be added automatically if percentiles are requested
                     unsigned long * gammaHash,///< array of 4 hash32 values to be saved with the histograms (same gamma for all sub-channels), NULL OK
                     unsigned long * framep)   ///< array of 8 values to copy  (frame, gains,expos,vexpos, focus), NULL OK
                                               ///< @return 0 OK, -EINVAL unused port/channel
{
    const int hist_frames_available = PARS_FRAMES - 3; // ?
    int i, color_start, hist_indx, hist_frame, hist_frame_index;
    unsigned long thisFrameNumber=getThisFrameNumber(sensor_port);
    dev_dbg(g_dev_ptr, "Setting histograms for frame=%ld(0x%lx), thisFrameNumber=%ld (0x%lx), needed=0x%x\n",
            frame,frame,thisFrameNumber,thisFrameNumber, needed);
    hist_indx=get_hist_index(sensor_port,sensor_chn); // channel/subchannel combination, removed unused
    if (hist_indx <0 ) return -EINVAL;
    if (frame >= thisFrameNumber) {
        dev_dbg(g_dev_ptr, "frame=%ld(0x%lx) >= thisFrameNumber=%ld (0x%lx) (not yet available)\n",frame,frame,thisFrameNumber,thisFrameNumber);
        return -EINVAL; // not yet available
    }
    // verify frame still not overwritten
    if (frame < (thisFrameNumber - hist_frames_available)) {
        dev_dbg(g_dev_ptr, "frame=%ld(0x%lx) < thisFrameNumber=%ld (0x%lx) - %d (already gone)\n",frame,frame,thisFrameNumber,thisFrameNumber,hist_frames_available);
        return -EINVAL; // not yet available
    }
    dev_dbg(g_dev_ptr, "Setting histograms for frame=%ld(0x%lx), thisFrameNumber=%ld (0x%lx), needed=0x%x, hist_indx=0x%x \n",
            frame,frame,thisFrameNumber,thisFrameNumber, needed, hist_indx);
    hist_frame_index = (int) GLOBALPARS(sensor_port,G_HIST_LAST_INDEX+sensor_chn);
    dev_dbg(g_dev_ptr, "histograms[%d][%d].frame = 0x%lx\n",
            hist_indx, hist_frame_index, histograms[hist_indx][hist_frame_index].frame);
    if (histograms[hist_indx][hist_frame_index].frame!=frame) {
        hist_frame_index = (hist_frame_index+1) & (HISTOGRAM_CACHE_NUMBER-1);
        GLOBALPARS(sensor_port, G_HIST_LAST_INDEX+sensor_chn)=hist_frame_index;
        histograms[hist_indx][hist_frame_index].valid=0;     // overwrite all
        histograms[hist_indx][hist_frame_index].frame=frame; // add to existent
        if (framep)    memcpy (&(histograms[hist_indx][hist_frame_index].frame),  framep,    32); // copy provided frame, gains,expos,vexpos, focus
        if (gammaHash) memcpy (&(histograms[hist_indx][hist_frame_index].gtab_r), gammaHash, 16); // copy provided 4 hash32 values
        dev_dbg(g_dev_ptr, "histograms[%d][%d].frame = 0x%lx\n",
                hist_indx, hist_frame_index, histograms[hist_indx][hist_frame_index].frame);
    } else {
        needed &= ~histograms[hist_indx][hist_frame_index].valid; // remove those that are already available from the request
    }
    dev_dbg(g_dev_ptr, "needed=0x%x\n", needed);

    // TODO: handle valid and needed for multichannel?
    if (!(needed & 0xf)) // nothing to do with FPGA
        return 0;
    // Copying received data to histograms structure, maybe later we can skip that step and use data in place
    //  hist_frame=(frame-1) & PARS_FRAMES_MASK; // TODO: Verify
//    hist_frame=frame & PARS_FRAMES_MASK; // TODO: Verify
    hist_frame=(frame + (GLOBALPARS(sensor_port, G_HIST_SHIFT))) & PARS_FRAMES_MASK; // TODO: Verify
    for (i=0; i<4; i++) if (needed & ( 1 << i )) {
        u32 phys_addr= fpga_hist_phys + PAGE_SIZE*(hist_frame + PARS_FRAMES * (sensor_chn + MAX_SENSORS *sensor_port)) + i*256*sizeof(u32); // start of selected color
        u32 * dma_data = &fpga_hist_data[sensor_port][sensor_chn][hist_frame][i][0];
        // invalidate both caches
        outer_inv_range(phys_addr, phys_addr + (256*sizeof(u32) - 1));
        __cpuc_flush_dcache_area(dma_data, 256*sizeof(u32));
        color_start= i<<8 ;
        memcpy(&histograms[hist_indx][hist_frame_index].hist[256*i],
                dma_data, 256*sizeof(u32));
        // old in 353:  fpga_hist_read_nice (color_start, 256, (unsigned long *) &histograms[GLOBALPARS(G_HIST_LAST_INDEX)].hist[color_start]);
        histograms[hist_indx][hist_frame_index].valid |= 1 << i;
        dev_dbg(g_dev_ptr, "histograms[%d][%d].valid=0x%lx\n",
                hist_indx, hist_frame_index,histograms[hist_indx][hist_frame_index].valid );
    }
    return 0;
/*
   for (i=0; i<4; i++) if (needed & ( 1 << i )) {
    color_start= i<<8 ;
    fpga_hist_read_nice (color_start, 256, (unsigned long *) &histograms[GLOBALPARS(G_HIST_LAST_INDEX)].hist[color_start]);
    histograms[GLOBALPARS(G_HIST_LAST_INDEX)].valid |= 1 << i;
  }

 */
}


/**
 * @brief Get derivative histograms (raw FPGA should be already there read by a tasklet needed)
 *  Will look for requested (or earlier) frame that has the "needed" raw histograms
 * TODO: should it be one frame behind current? - yes, exactly
 */
///TODO: Make color (rare) histograms survive longer? - Challenge - Y goes first, we do not know if it will be followed by color
int  get_histograms(int sensor_port,     ///< sensor port number (0..3)
                    int sensor_chn,      ///< sensor subchannel (0 w/o multiplexer)
                    unsigned long frame, ///< absolute frame number (Caller should match it to the hardware frame)
                    int needed)          ///< bits specify what histograms (color, type) are requested
                                         ///<  each group of 4 bits covers 4 colors of the same type:
                                         ///<  - bits 0..3 - read raw histograms from the FPGA - normally called from IRQ/tasklet (use just 1 color for autoexposure to speed up?)
                                         ///<  - bits 4..7 - calculate cumulative histograms (sum of raw ones) - normally called from applications
                                         ///<  - bits 8..11 - calculate percentiles (reverse cumulative histograms) - normally called from applications
                                         ///<  "needed" for raw histograms should be specified explicitly (can not be read from FPGA later),
                                         ///<  "needed" for cumul_hist will be added automatically if percentiles are requested
                                         ///< @return index of the histogram (>=0) if OK, otherwise:
                                         ///< - -EFAULT not reading FPGA and frame number stored is different from the requested (too late - histogram buffer overrun?)
                                         ///< - -EINVAL unused port/channel
{
    int i, color_start, index;
    int hist_indx=get_hist_index(sensor_port,sensor_chn);
    int raw_needed;
    unsigned long thisFrameNumber=getThisFrameNumber(sensor_port);
    unsigned long * gammaHash;
    unsigned long * framep;
    unsigned long dbg_sum;
    if (hist_indx <0 ) return -EINVAL;
    raw_needed=(needed | (needed>>4) | needed>>8) & 0xf;
    dev_dbg(g_dev_ptr, "sensor_port=%d, sensor_chn=%d, frame = %ld (0x%lx), thisFrameNumber=%ld(0x%lx), needed = 0x%x, raw_needed=0x%x\n",
            sensor_port, sensor_chn, frame, frame, thisFrameNumber, thisFrameNumber, needed, raw_needed);
    if (raw_needed){
        // get parameters - decide from current or mpastpars
        if (frame == (thisFrameNumber-1)) { // use pars
            gammaHash = get_imageParamsFramePtr(sensor_port, P_GTAB_R, frame);
            framep =    get_imageParamsFramePtr(sensor_port, P_FRAME, frame);
        } else {
            gammaHash = get_imageParamsPastPtr (sensor_port, P_GTAB_R, frame);
            framep =    get_imageParamsPastPtr (sensor_port, P_FRAME, frame);
        }
        if ((i= set_histograms (sensor_port, sensor_chn, frame, raw_needed, gammaHash, framep))){
            dev_dbg(g_dev_ptr, "Failed to set up histograms for frame= %ld(0x%lx), thisFrameNumber=%ld(0x%lx), returned %d\n",
                    frame,frame, thisFrameNumber,thisFrameNumber,i);
            return i;
        }
    }
    index=GLOBALPARS(sensor_port, G_HIST_LAST_INDEX+sensor_chn); // set_histograms may increment G_HIST_LAST_INDEX+sensor_chn
    for (i=0;i<HISTOGRAM_CACHE_NUMBER;i++) {
        dev_dbg(g_dev_ptr, "index=%d, needed=0x%x\n",index,needed);
        if ((histograms[hist_indx][index].frame <= frame) && ((histograms[hist_indx][index].valid & raw_needed)==raw_needed)) break;
        index = (index-1) & (HISTOGRAM_CACHE_NUMBER-1);
    }
    if (i>=HISTOGRAM_CACHE_NUMBER) {
        dev_err(g_dev_ptr, "no histograms exist for requested colors (0x%x), requested 0x%x\n",raw_needed,needed);
        // here we need to try to locate and copy raw histogram



        return -EFAULT; // if Y - never calculated, if C - maybe all the cache is used by Y
    }
///    needed &= ~0x0f; // mask out FPGA read requests -= they are not handled here anymore (use set_histograms())
    dev_dbg(g_dev_ptr, "needed=0x%x\n",needed);
    needed |= ((needed >>4) & 0xf0); // cumulative histograms are needed for percentile calculations
    needed &= ~histograms[hist_indx][index].valid;
    dev_dbg(g_dev_ptr, "needed=0x%x\n",needed);
    if (needed & 0xf0) { // Calculating cumulative histograms
        for (i=0; i<4; i++) if (needed & ( 0x10 << i )) {
            color_start= i<<8 ;
            dbg_sum = histogram_calc_cumul ( (unsigned long *) &histograms[hist_indx][index].hist[color_start],  (unsigned long *) &histograms[hist_indx][index].cumul_hist[color_start] );
            dev_dbg(g_dev_ptr, "frame: 0x%lx (now 0x%lx) color:%d, pixel sum=0x%08lx\n",frame, thisFrameNumber, i, dbg_sum);
            histograms[hist_indx][index].valid |= 0x10 << i;
        }
        dev_dbg(g_dev_ptr, "needed=0x%x, valid=0x%lx\n",needed,histograms[hist_indx][index].valid);
    }
    if (needed & 0xf00) { // Calculating percentiles
        for (i=0; i<4; i++) if (needed & ( 0x100 << i )) {
            color_start= i<<8 ;
            histogram_calc_percentiles ( (unsigned long *) &histograms[hist_indx][index].cumul_hist[color_start],  (unsigned char *) &histograms[hist_indx][index].percentile[color_start] );
            histograms[hist_indx][index].valid |= 0x100 << i;
        }
        dev_dbg(g_dev_ptr, "needed=0x%x, valid=0x%lx\n",needed, histograms[hist_indx][index].valid);
    }
    return index;
}

/** Calculate cumulative histogram (one color component) from the corresponding raw histogram */
inline  unsigned long  histogram_calc_cumul ( unsigned long * hist,       ///< input raw histogram array of unsigned long, single color (256)
                                              unsigned long * cumul_hist) ///< output cumulative histogram array of unsigned long, single color (256)
                                               ///< @return sum of all pixels (last value)
{
    int i;
    cumul_hist[0]=hist[0];
    for (i=1; i<256;i++) cumul_hist[i]=cumul_hist[i-1]+hist[i];
    return cumul_hist[255];
}

/**
 * Calculate reverse cumulative histograms (~percentiles)
 * The reverse cumulative histogram (~percentiles) works as the following:
 * for the given 1 byte input X (0 - 1/256 of all pixels, *  ..., 255 - all pixels)
 * it returns threshold value P (0..255), so that number of pixels with value less than x is
 * less or equal to (P/256)*total_number_of_pixels,  and number of pixels with value less than (x+1) is
 * greater than (P/256)*total_number_of_pixels, P(0)=0, P(256)=256 (not included in the table).
 *
 * Percentiles arrays are calculated without division for each element, interpolation (involving division)
 * will be done only for the value of interest  on demand, in the user space.
 *
 * NOTE: - argument is FPGA pixel output-to-videoRAM value (after gamma-correction), reverse gamma table
 *         is needed to relate percentiles to amount of light (proportional to exposure)
 *
 * Current algorithm is limited to 16 MPix/color_component (64 MPix total) */
inline void  histogram_calc_percentiles (unsigned long * cumul_hist,  ///< [IN] Pointer to the start of u32[256] cumulative histogram array
                                         unsigned char * percentile)  ///< [OUT]Pointer to the start of u32[256] calculated percentile array
{
    unsigned long v256=0; // running value to be compared against cumulative histogram (it is 256 larger than cumul_hist)
    unsigned long inc_v256=cumul_hist[255];  // step of v256 increment
    int shiftl=8;
    int p=0; // current value of percentile
    int x=0; // current percentile index
    while (inc_v256>0xffffff) { // to protect from unlikely overflow at 16MPix - in the future)
        inc_v256 >>= 1;
        shiftl--;
    }
    while ((p<256) && (x<256)) {
        percentile[x]=p;
        if ((p<255) && ( (cumul_hist[p] << shiftl) <= v256)) {
            p++;
        } else {
            x++;
            v256+=inc_v256;
        }
    }
}


//======================================
// use G_SUBCHANNELS in userspace to re-calculate full histogram index
// File operations:
// open, release - nop
// read - none
// write - none
// lseek
// mmap (should be used read only)

/** HISTOGRAMS_FILE_SIZE histograms file size in frames (total nu,ber in all channels), not bytes) */
#define HISTOGRAMS_FILE_SIZE (HISTOGRAM_CACHE_NUMBER*numHistChn)
static struct file_operations histograms_fops = {
   owner:    THIS_MODULE,
   llseek:   histograms_lseek,
   open:     histograms_open,
   mmap:     histograms_mmap,
   release:  histograms_release
};

/** Histograms driver OPEN method */
int histograms_open(struct inode *inode, ///< inode
                    struct file *file)   ///< file pointer
                                         ///<  @return OK - 0, -EINVAL for wrong minor
{
    int res;
    struct histograms_pd * privData;
    privData= (struct histograms_pd *) kmalloc(sizeof(struct histograms_pd),GFP_KERNEL);
    if (!privData) return -ENOMEM;
    file->private_data = privData;
    privData-> minor=MINOR(inode->i_rdev);
    dev_dbg(g_dev_ptr, "histograms_open: minor=0x%x\n",privData-> minor);
    switch (privData-> minor) {
    case  DEV393_MINOR(DEV393_HISTOGRAM) :
        inode->i_size = HISTOGRAMS_FILE_SIZE;
        privData->frame=0xffffffff;
        privData->frame_index=-1;
        privData->needed= 0;
        privData->wait_mode=0;  // 0 - wait just for G1 histogram, 1 - wait for all histograms
        privData->request_en=1; // enable requesting histogram for the specified frame (0 - rely on the available ones)
        privData->port=0;
        privData->subchannel=0;
        if (histograms_check_init() < 2)
            return -ENODEV; // Bitstream not loaded?
        return 0;
    default:
        kfree(file->private_data); // already allocated
        return -EINVAL;
    }
    file->f_pos = 0;
    return res;
}

/** Histograms driver RELEASE method */
int histograms_release (struct inode *inode, ///< inode
                        struct file *file)   ///< file pointer
                                             ///<  @return OK - 0, -EINVAL for wrong minor
{
    int res=0;
    int p = MINOR(inode->i_rdev);
    dev_dbg(g_dev_ptr, "histograms_release minor=0x%x\n",p);
    switch ( p ) {
    case  DEV393_MINOR(DEV393_HISTOGRAM) :
        break;
    default:
        return -EINVAL; //! do not need to free anything - "wrong number"
    }
    kfree(file->private_data);
    return res;
}

/** Histograms driver LSEEK  method (and execute commands)<ul>
 * <li>lseek <b>(SEEK_SET, value)</b> wait for histogram of the absolute frame 'value' (G1 or all depending on wait_mode
 *                           locate frame number value and set frame_index (and file pointer) to the result.
 *                           Return error if frame can not be found, otherwise - histogram index (to use with mmap)
 *                           Calculate missing tables according to "needed" variable
 * <li>lseek <b>(SEEK_CUR, value)</b>   wait for histogram of the  frame 'value' from the current one (G1 or all depending on wait_mode
 *                           locate frame number value and set frame_index (and file pointer) to the result.
 *                           Return error if frame can not be found, otherwise - histogram index (to use with mmap)
 *                           Calculate missing tables according to "needed" variable
 *                           lseek (SEEK_CUR, 0) will wait for the histogram(s) for current frame
 * <li> lseek <b>(SEEK_CUR, value)</b> - ignore value, return frame_index (may be negative if error)
 * <li> lseek <b>(SEEK_END, value < 0)</b> - do nothing?, do not modify file pointer, return error
 * <li> lseek <b>(SEEK_END, value = 0)</b> - return HISTOGRAMS_FILE_SIZE
 * <li> lseek <b>(SEEK_END, LSEEK_HIST_WAIT_Y)</b> - set histogram waiting for the Y (actually G1) histogram (default after open)
 * <li> lseek <b>(SEEK_END, LSEEK_HIST_WAIT_C)</b> - set histogram waiting for the C (actually R, G2, B) histograms to become available - implies G1 too
 * <li> lseek <b>(SEEK_END,  LSEEK_HIST_SET_CHN +4*port+ subchannel)</b> - select sensor port and subchannel. Returns -ENXIO if port/subchannel does not
 *                           match any active sensor.
 * <li> lseek <b>(SEEK_END, LSEEK_HIST_NEEDED)</b> set histogram "needed" bits
 * <li> lseek <b>(SEEK_END, LSEEK_HIST_REQ_EN)</b> - (default)enable histogram request when reading histogram (safer, but may be not desirable in HDR mode) - default after opening
 * <li> lseek <b>(SEEK_END, LSEEK_HIST_REQ_DIS</b>) - disable histogram request when reading histogram - will read latest available relying it is available </ul>
 * @param file 
 * @param offset 
 * @param orig SEEK_SET, SEEK_CUR or SEEK_SET END
 * @return file position (histogram frame index (combined frame index and channel))
 */

// TODO: NC393 - use int camSeqGetJPEG_frame(unsigned int chn); and
// get_imageParamsFrame(..., camSeqGetJPEG_frame(chn)) instead of get_imageParamsThis(...)
// TODO: add flag that will allow driver to wakeup processes before the specified frame comes ?
loff_t histograms_lseek (struct file * file,
                         loff_t offset,
                         int orig)
{
    int p,s,index;
    struct histograms_pd * privData = (struct histograms_pd *) file->private_data;
    unsigned long reqAddr,reqFrame;

    dev_dbg(g_dev_ptr, "histograms_lseek: offset=%d(0x%x), orig=0x%x, getThisFrameNumber(%d)=0x%x\n", (int) offset, (int) offset, (int) orig, privData->port, (int)getThisFrameNumber(privData->port));
    switch (privData->minor) {
    case DEV393_MINOR(DEV393_HISTOGRAM) :
        switch(orig) {
        case SEEK_CUR: // ignore offset - in NC353 it was get latest?
//            offset = -1; // for now - just make "latest"
#if 0
            offset+=(privData-> wait_mode)?
                    GLOBALPARS(privData->port,G_HIST_C_FRAME+privData->subchannel):
                    GLOBALPARS(privData->port,G_HIST_Y_FRAME+privData->subchannel);
#endif

            offset+=getThisFrameNumber(privData->port); // get relative to current frame (-1 - latest)
            //no break (CDT understands this)
        case SEEK_SET: // negative - make relative to current (-1 - latest, -2 - before latest - up to 15 before)
            if (offset <0){
                dev_dbg(g_dev_ptr, "offset= %lld (0x%llx) changing to previous frame \n",offset,offset);
                offset += getThisFrameNumber(privData->port);
            }
            privData->frame=offset;
            // Try to make some precautions to avoid waiting forever - if the past frame is requested - request histogram for the current frame,
            // if the "immediate" future (fits into the array of frames) one  - request that frame's  histogram
            // if in the far future (unsafe) do nothing -NOTE: far future should be avoided if the histograms are set request-only
            // NOTE: there could be another wrong condition - request written w/o "JUST_THIS" modifier - then it will turn to always on until cleared.
            // TODO: Save time on always enabled histograms? Don't request them additionally?
            if (privData->request_en) {
                reqAddr=(privData-> wait_mode)?P_HISTRQ_C:P_HISTRQ_Y;
                reqFrame=getThisFrameNumber(privData->port);
                dev_dbg(g_dev_ptr, "offset= %d (0x%x), reqFrame=%d (0x%x) \n",(int) offset,(int) offset,(int) reqFrame,(int) reqFrame);
                if (offset > reqFrame) {
                    if (offset > (reqFrame+5)) reqFrame+=5; // What is this 5?
                    else                       reqFrame=offset;
                    dev_dbg(g_dev_ptr, "offset= %d (0x%x), modified reqFrame for future request =%d (0x%x) \n",(int) offset,(int) offset,(int) reqFrame,(int) reqFrame);
                }
                if (offset < reqFrame) { // just debugging
                    dev_dbg(g_dev_ptr, "offset < reqFrame, will run get_histograms (%d, %d, 0x%x, 0x%x) \n",
                            privData->port, privData->subchannel, (int) offset, (int) privData->needed);
                }
                if ((offset < reqFrame) && // if the requested frame is in the past - try to get it first before requesting a new
                        (((privData->frame_index = get_histograms (privData->port, privData->subchannel, offset, privData->needed))) >=0)) {
//                    file->f_pos=privData->frame_index;
                    if (((index = get_hist_index(privData->port, privData->subchannel))) <0)
                        return -ENODEV; // requested combination of port and subchannel does not exist
                    file->f_pos=privData->frame_index + HISTOGRAM_CACHE_NUMBER * get_hist_index(privData->port, privData->subchannel);
                    dev_dbg(g_dev_ptr, "Returning %d (0x%x)\n", (int) file->f_pos, (int) file->f_pos);
                    return file->f_pos;
                }
                // request histogram(s)
                //             setFramePar(&framepars[reqFrame & PARS_FRAMES_MASK], reqAddr, 1);
                dev_dbg(g_dev_ptr, "setFrameParLocked(%d, &aframepars[%d][0x%x], 0x%lx, 1)\n",
                        privData->port, privData->port, (int) (reqFrame & PARS_FRAMES_MASK), reqAddr);
                setFrameParLocked(privData->port, &aframepars[privData->port][reqFrame & PARS_FRAMES_MASK], reqAddr, 1);
                // make sure (harmful) interrupt did not happen since getThisFrameNumber()
                if (reqFrame < getThisFrameNumber(privData->port)) {
                    dev_dbg(g_dev_ptr, "setFrameParLocked(%d, &aframepars[%d][0x%lx], 0x%lx, 1)\n",
                            privData->port, privData->port, getThisFrameNumber(privData->port) & PARS_FRAMES_MASK, reqAddr);
                    setFrameParLocked(privData->port, &aframepars[privData->port][getThisFrameNumber(privData->port) & PARS_FRAMES_MASK], reqAddr, 1);
                }
            } else { // debug
                dev_dbg(g_dev_ptr, "privData->request_en=0\n");
            }
#if 0
            if (privData-> wait_mode)  wait_event_interruptible (hist_c_wait_queue,GLOBALPARS(privData->port,G_HIST_C_FRAME + privData->subchannel)>=offset);
            else                       wait_event_interruptible (hist_y_wait_queue,GLOBALPARS(privData->port,G_HIST_Y_FRAME + privData->subchannel)>=offset);
#endif
            dev_dbg(g_dev_ptr, "Before waiting: frame = 0x%x, offset=0x%x privData-> wait_mode=%d\n",
                    (int) getThisFrameNumber(privData->port), (int) offset, privData-> wait_mode);
            // neded next frame after requested (modify php too?)
            if (privData-> wait_mode)  wait_event_interruptible (hist_c_wait_queue,getThisFrameNumber(privData->port)>offset);
            else                       wait_event_interruptible (hist_y_wait_queue,getThisFrameNumber(privData->port)>offset);
            dev_dbg(g_dev_ptr, "After waiting: frame = 0x%x\n", (int) getThisFrameNumber(privData->port));
            privData->frame_index = get_histograms (privData->port, privData->subchannel, offset, privData->needed);
            if (privData->frame_index <0) {
                return -EFAULT;
            } else {
//                file->f_pos=privData->frame_index;
                file->f_pos=privData->frame_index + HISTOGRAM_CACHE_NUMBER * get_hist_index(privData->port, privData->subchannel);
                dev_dbg(g_dev_ptr, "file->f_pos (full histogram number - cache and port/channel combined) = 0x%x\n", (int) file->f_pos);
                return file->f_pos;
            }
            break; // just in case
        case SEEK_END:
            if (offset < 0) {
                return -EINVAL;
            } else {
                if (offset < LSEEK_HIST_NEEDED) {
                    //#define      LSEEK_HIST_SET_CHN   0x30 ///< ..2F Select channel to wait for (4*port+subchannel)
                    if ((offset & ~0xf) == LSEEK_HIST_SET_CHN){
                        p = (offset >> 2)  & 3;
                        s = (offset >> 0)  & 3;
                        if (get_hist_index(p,s)<0)
                            return -ENXIO; // invalid port/channel combination
                        privData->port =       p;
                        privData->subchannel = s;
                        file->f_pos=privData->frame_index + HISTOGRAM_CACHE_NUMBER * get_hist_index(privData->port, privData->subchannel);
                    } else switch (offset) {
                    case 0:
                        break;
                    case  LSEEK_HIST_REQ_EN: // enable requesting histogram for the specified frame (0 - rely on the available ones)
                        privData->request_en=1; ///default after open
                        break;
                    case  LSEEK_HIST_REQ_DIS: // disable requesting histogram for the specified frame, rely on the available ones
                        privData->request_en=0;
                        break;
                    case  LSEEK_HIST_WAIT_Y: // set histogram waiting for the Y (actually G1) histogram (default after open)
                        privData-> wait_mode=0;
                        break;
                    case  LSEEK_HIST_WAIT_C: // set histogram waiting for the C (actually R, G2, B) histograms to become available - implies G1 too
                        privData-> wait_mode=1;
                        break;
                    default:
                        switch (offset & ~0x1f) {
                        case  LSEEK_DAEMON_HIST_Y: // wait for daemon enabled and histograms Y ready
                            dev_dbg(g_dev_ptr, "wait_event_interruptible (hist_y_wait_queue,0x%x & 0x%x)\n",(int) get_imageParamsThis(privData->port, P_DAEMON_EN), (int) (1<<(offset & 0x1f)));
                            wait_event_interruptible (hist_y_wait_queue, get_imageParamsThis(privData->port, P_DAEMON_EN) & (1<<(offset & 0x1f)));
                            break;
                        case  LSEEK_DAEMON_HIST_C: // wait for daemon enabled and histograms Y ready
                            dev_dbg(g_dev_ptr, "wait_event_interruptible (hist_c_wait_queue,0x%x & 0x%x)\n",(int) get_imageParamsThis(privData->port, P_DAEMON_EN), (int) (1<<(offset & 0x1f)));
                            wait_event_interruptible (hist_c_wait_queue, get_imageParamsThis(privData->port, P_DAEMON_EN) & (1<<(offset & 0x1f)));
                            break;
                        default:
                            return -EINVAL;
                        }
                    }
                } else if (offset < (LSEEK_HIST_NEEDED + 0x10000)) {
                    privData->needed= (offset & 0xffff);
                } else  {
                    return -EINVAL;
                }
                file->f_pos= HISTOGRAMS_FILE_SIZE;
                return file->f_pos;
            }
            break;
        default:  // not SEEK_SET/SEEK_CUR/SEEK_END
            return -EINVAL;
        } // switch (orig)
        return  file->f_pos ;
        default: // other minors
            return -EINVAL;
    }
}
/**
 * @brief Histograms driver MMAP method to read out the histogram data (raw and calculated)
 * @param file 
 * @param vma 
 * @return OK - 0, negative - errors
 */

int histograms_mmap (struct file *file, struct vm_area_struct *vma) {
    int result;
    struct histograms_pd * privData = (struct histograms_pd *) file->private_data;
    dev_dbg(g_dev_ptr, "histograms_mmap minor=0x%x\n",privData-> minor);
    switch (privData->minor) {
    case  DEV393_MINOR(DEV393_HISTOGRAM) :
                result=remap_pfn_range(vma,
                        vma->vm_start,
                        ((unsigned long) virt_to_phys(histograms_p)) >> PAGE_SHIFT, // Should be page-aligned
                        vma->vm_end-vma->vm_start,
                        vma->vm_page_prot);
    dev_dbg(g_dev_ptr, "remap_pfn_range returned=%x\n",result);
    if (result) return -EAGAIN;
    return 0;
    default: return -EINVAL;
    }
}

/**
 * @brief Histograms driver init
 * @return 0
 */
int histograms_init(struct platform_device *pdev) {
    int res;
    int sz, pages;
    struct device *dev = &pdev->dev;
//    const struct of_device_id *match; // not yet used
//    init_histograms(); // Not now??? Need to have list of channels
    // Do it later, from the user space
    res = register_chrdev(DEV393_MAJOR(DEV393_HISTOGRAM), DEV393_NAME(DEV393_HISTOGRAM), &histograms_fops);
    if(res < 0) {
        dev_err(dev, "histograms_init: couldn't get a major number %d.\n", DEV393_MAJOR(DEV393_HISTOGRAM));
        return res;
    }
    //   init_waitqueue_head(&histograms_wait_queue);
    init_waitqueue_head(&hist_y_wait_queue);    // wait queue for the G1 histogram (used as Y)
    init_waitqueue_head(&hist_c_wait_queue);    // wait queue for all the other (R,G2,B) histograms (color)
    dev_info(dev, DEV393_NAME(DEV393_HISTOGRAM)": registered MAJOR: %d\n", DEV393_MAJOR(DEV393_HISTOGRAM));
    histograms_initialized = 0;
    // NC393: Did not find a way to get memory when histograms a first needed:
    // GFP_KERNEL can nolt work inside spinlock (needed to prevent simultaneous initializations
    // from multiple php-cli instances, GFP_ATOMIC just fails.
    // So here we
    // allocate for all 16 (ports*subchannels) ~1.1MB and then use portion of it (usually 1/4)

    sz = SENSOR_PORTS * MAX_SENSORS * HISTOGRAM_CACHE_NUMBER * sizeof(struct histogram_stuct_t);
    pages = ((sz -1 ) >> PAGE_SHIFT) + 1;
    histograms = dma_alloc_coherent(NULL,(pages * PAGE_SIZE),&histograms_phys,GFP_KERNEL);
    dev_info(dev, "dma_alloc_coherent(NULL, 0x%lx, 0x%x,GFP_KERNEL)", pages * PAGE_SIZE, (int) histograms_phys);
    g_dev_ptr = dev; // to use for debug print
    return 0;
}
int histograms_remove(struct platform_device *pdev)
{
    unregister_chrdev(DEV393_MAJOR(DEV393_HISTOGRAM), DEV393_NAME(DEV393_HISTOGRAM));

    return 0;
}
static const struct of_device_id elphel393_histograms_of_match[] = {
    { .compatible = "elphel,elphel393-histograms-1.00" },
    { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_histograms_of_match);


static struct platform_driver elphel393_histograms = {
    .probe          = histograms_init,
    .remove         = histograms_remove,
    .driver         = {
        .name       = DEV393_NAME(DEV393_HISTOGRAM),
        .of_match_table = elphel393_histograms_of_match,
    },
};

//module_init(histograms_init);

module_platform_driver(elphel393_histograms);

MODULE_LICENSE("GPLv3.0");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_HISTOGRAMS_DRIVER_DESCRIPTION);
