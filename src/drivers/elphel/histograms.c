/*!********************************************************************************
*! FILE NAME  : histograms.c
*! DESCRIPTION: Handles histograms storage, access and percentile calculation
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
*!
*!
*/

//copied from cxi2c.c - TODO:remove unneeded 

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/autoconf.h>
#include <linux/vmalloc.h>

#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <asm/elphel/driver_numbers.h>
#include <asm/elphel/c313a.h>
#include <asm/elphel/exifa.h>
#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr 
#include "fpga_io.h"//fpga_table_write_nice
#include "framepars.h"        // for debug mask

//#include "cc3x3.h"
#include "histograms.h"

/**
 * \def MDF21(x) optional debug output 
 */

#if ELPHEL_DEBUG
/// only when commands are issued
  #define MDF21(x) { if (GLOBALPARS(G_DEBUG) & (1 <<21)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
/// includes tasklets
  #define MDF22(x) { if (GLOBALPARS(G_DEBUG) & (1 <<22)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
#else
  #define MDF21(x)
  #define MDF22(x)
#endif

#define  X3X3_HISTOGRAMS_DRIVER_NAME "Elphel (R) Model 353 Histograms device driver"

static struct histogram_stuct_t histograms[HISTOGRAM_CACHE_NUMBER] __attribute__ ((aligned (PAGE_SIZE)));

       struct histogram_stuct_t * histograms_p; // to use with mmap

wait_queue_head_t hist_y_wait_queue;    /// wait queue for the G1 histogram (used as Y)
wait_queue_head_t hist_c_wait_queue;    /// wait queue for all the other (R,G2,B) histograms (color)


struct histograms_pd {
    int                minor;
    unsigned long      frame;
    int                frame_index; /// -1 if invalid
    int                needed;
    int                wait_mode;  /// 0 - wait just for G1 histogram, 1 - wait for all histograms
    int                request_en; /// enable requesting histogram for the specified frame (0 - rely on the available ones)
    struct wait_queue *hist_y_wait_queue;    /// wait queue for the G1 histogram (used as Y)  ///NOTE: not used at all?
    struct wait_queue *hist_c_wait_queue;    /// wait queue for all the other (R,G2,B) histograms (color)  ///NOTE: not used at all?

/// something else to be added here?
};

int        histograms_open   (struct inode *inode, struct file *file);
int        histograms_release(struct inode *inode, struct file *file);
loff_t     histograms_lseek  (struct file * file, loff_t offset, int orig);
int        histograms_mmap   (struct file *file, struct vm_area_struct *vma);
static int __init histograms_init(void);


inline void  histogram_calc_cumul       ( unsigned long * hist,        unsigned long * cumul_hist );
inline void  histogram_calc_percentiles ( unsigned long * cumul_hist,  unsigned char * percentile);


/*
   unsigned long gtab_r;
   unsigned long gtab_g;
   unsigned long gtab_gb;
   unsigned long gtab_b;

*/

/**
 * @brief Initialize histgograms data structures (called at module init)
 */
void init_histograms(void) {
  unsigned long flags;
  int i;
  histograms_p=histograms;
  MDF21(printk("\n"));
  local_irq_save(flags);
  for (i=0; i < HISTOGRAM_CACHE_NUMBER; i++) {
    histograms[i].frame=0xffffffff;
    histograms[i].valid=0;
  }
  local_irq_restore(flags);
}

/**
 * @brief Get histograms from the FPGA (called as tasklet?)
 * @param frame absolute frame number (Caller should match it to the hardware frame)
 * TODO: should it be one frame behind current?
 * @param needed bits specify what histograms (color, type) are requested
 * @param gammaHash array of 4 hash32 values to be saved with the histograms
 * each group of 4 bits cover 4 colors of the same type:
 * - bits 0..3 - read raw histograms from the FPGA - normally called from IRQ/tasklet (use just 1 color for autoexposure to speed up?)
 * @return 0 for now
 */

int set_histograms (unsigned long frame, int needed, unsigned long * gammaHash,  unsigned long * framep) {
//  unsigned long flags;
//hist_next_index
  int i, color_start;
  if (histograms[GLOBALPARS(G_HIST_LAST_INDEX)].frame!=frame) {
       GLOBALPARS(G_HIST_LAST_INDEX)=(GLOBALPARS(G_HIST_LAST_INDEX)+1) & (HISTOGRAM_CACHE_NUMBER-1);
       histograms[GLOBALPARS(G_HIST_LAST_INDEX)].valid=0;     /// overwrite all
       histograms[GLOBALPARS(G_HIST_LAST_INDEX)].frame=frame; /// add to existent
       if (framep)    memcpy (&(histograms[GLOBALPARS(G_HIST_LAST_INDEX)].frame),  framep,    32); // copy provided frame, gains,expos,vexpos, focus
       if (gammaHash) memcpy (&(histograms[GLOBALPARS(G_HIST_LAST_INDEX)].gtab_r), gammaHash, 16); // copy provided 4 hash32 values
  } else {
      needed &= ~histograms[GLOBALPARS(G_HIST_LAST_INDEX)].valid; /// remove those that are already available from the request
  }
  for (i=0; i<4; i++) if (needed & ( 1 << i )) {
    color_start= i<<8 ;
    fpga_hist_read_nice (color_start, 256, (unsigned long *) &histograms[GLOBALPARS(G_HIST_LAST_INDEX)].hist[color_start]);
    histograms[GLOBALPARS(G_HIST_LAST_INDEX)].valid |= 1 << i;
  }
  return 0;
}


/**
 * @brief Get derivative histograms (raw FPGA should be already there read by a tasklet needed)
 *  Will look for requested (or earlier) frame that has the "needed" raw histograms
 * @param frame absolute frame number (Caller should match it to the hardware frame).
 * TODO: should it be one frame behind current? - yes, exactly
 * @param needed bits specify what histograms (color, type) are requested
 * TODO: Add P_* parameter - what to read from tasklet,  turn colors it off for high FPS/small window
 * each group of 4 bits cover 4 colors of the same type:
 * - bits 0..3 - ignored here (used for raw FPGA tables)
 * - bits 4..7 - calculate cumulative histograms (sum of raw ones) - normally called from applications
 * - bits 8..11 - calculate percentiles (reverse cumulative histograms) - normally called from applications
 * "needed" for raw histograms should be specified explicitely (can not be read from FPGA later),
 * "needed" for cumul_hist will be added automatically if percentiles are requested
 * @return index of the histogram (>=0) if OK, otherwise:
 * - -1 not reading FPGA and frame number stored is different from the requested (too late - histogram buffer overrun?)
 */
///TODO: Make color (rare) histograms survive longer? - Challenge - Y goes first, we do not know if it will be followed by color
int get_histograms (unsigned long frame, int needed) {
//  unsigned long flags;
//hist_next_index
  int i, color_start;
  int index=GLOBALPARS(G_HIST_LAST_INDEX);
  int raw_needed=(needed | (needed>>4) | needed>>8) & 0xf;
  for (i=0;i<HISTOGRAM_CACHE_NUMBER;i++) {
    MDF21(printk("index=%d, needed=0x%x\n",index,needed));
    if ((histograms[index].frame <= frame) && ((histograms[index].valid & raw_needed)==raw_needed)) break;
    index = (index-1) & (HISTOGRAM_CACHE_NUMBER-1);
  }
  if (i>=HISTOGRAM_CACHE_NUMBER) {
     ELP_KERR(printk("no histograms exist for requested colors (0x%x), requested 0x%x\n",raw_needed,needed));
     return -1; /// if Y - never calculated, if C - maybe all the cache is used by Y 
  }
  needed &= ~0x0f; /// mask out FPGA read requests -= they are not handled here anymore (use set_histograms())
  MDF22(printk("needed=0x%x\n",needed));
  needed |= ((needed >>4) & 0xf0); /// cumulative histograms are needed for percentile calculations
  needed &= ~histograms[index].valid;
///  if ((needed >> 4)  & (~histograms[index].valid) & 0x0f) return -2 ; /// some needed raw histograms are not available NOTE: now never comes here
  MDF22(printk("needed=0x%x\n",needed));
  if (needed & 0xf0) { /// Calculating cumulative histograms
    for (i=0; i<4; i++) if (needed & ( 0x10 << i )) {
       color_start= i<<8 ;
       histogram_calc_cumul ( (unsigned long *) &histograms[index].hist[color_start],  (unsigned long *) &histograms[index].cumul_hist[color_start] );
       histograms[index].valid |= 0x10 << i;
    }
    MDF21(printk("needed=0x%x, valid=0x%lx\n",needed,histograms[index].valid));
  }
  if (needed & 0xf00) { /// Calculating percentiles
    for (i=0; i<4; i++) if (needed & ( 0x100 << i )) {
       color_start= i<<8 ;
       histogram_calc_percentiles ( (unsigned long *) &histograms[index].cumul_hist[color_start],  (unsigned char *) &histograms[index].percentile[color_start] );
       histograms[index].valid |= 0x100 << i;
    }
    MDF21(printk("needed=0x%x, valid=0x%lx\n",needed, histograms[index].valid));
  }
  return index;
}

/**
 * @brief calculate cumulative histogram (one color component) from the corresponding raw histogram
 * @param hist       input raw histogram array of unsigned long, single color (256)
 * @param cumul_hist output cumulative histogram array of unsigned long, single color (256)
 */
inline void  histogram_calc_cumul       ( unsigned long * hist,        unsigned long * cumul_hist ) {
  int i;
  cumul_hist[0]=hist[0];
  for (i=1; i<256;i++) cumul_hist[i]=cumul_hist[i-1]+hist[i];
}

/**
 * @brief Calculate reverse cumulative histograms (~percentiles)
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
 * Current algorithm will work up to 16 MPix/color_component (64 MPix total)
 * @param cumul_hist 
 * @param percentile 
 */
inline void  histogram_calc_percentiles ( unsigned long * cumul_hist,  unsigned char * percentile) {
  unsigned long v256=0; /// running value to be compared against cumulative histogram (it is 256 larger than cumul_hist)
  unsigned long inc_v256=cumul_hist[255];  /// step of v256 increment
  int shiftl=8;
  while (inc_v256>0xffffff) { /// to protect from unlikely overflow at 16MPix - in the future)
    inc_v256 >>= 1;
    shiftl--;
  }
  int p=0; /// current value of percentile
  int x=0; /// current percentile index
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


///======================================
/// File operations:
/// open, release - nop
/// read - none
/// write - none
/// lseek
/// mmap (should be used read only)

/**
 * \def HISTOGRAMS_FILE_SIZE histograms file size (in frames, not bytes)
 */
#define HISTOGRAMS_FILE_SIZE HISTOGRAM_CACHE_NUMBER
static struct file_operations histograms_fops = {
   owner:    THIS_MODULE,
   llseek:   histograms_lseek,
   open:     histograms_open,
   mmap:     histograms_mmap,
   release:  histograms_release
};

/**
 * @brief Histograms driver OPEN method
 * @param inode  inode
 * @param file   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int histograms_open(struct inode *inode, struct file *file) {
   int res;
   struct histograms_pd * privData;
   privData= (struct histograms_pd *) kmalloc(sizeof(struct histograms_pd),GFP_KERNEL);
   if (!privData) return -ENOMEM;
   file->private_data = privData;
   privData-> minor=MINOR(inode->i_rdev);
   MDF21(printk("minor=0x%x\n",privData-> minor));
   switch (privData-> minor) {
     case  CMOSCAM_MINOR_HISTOGRAMS :
        inode->i_size = HISTOGRAMS_FILE_SIZE;
        privData->frame=0xffffffff;
        privData->frame_index=-1;
        privData->needed= 0;
        privData->wait_mode=0;  /// 0 - wait just for G1 histogram, 1 - wait for all histograms
        privData->request_en=1; /// enable requesting histogram for the specified frame (0 - rely on the available ones)
        return 0;
     default:
       kfree(file->private_data); // already allocated
       return -EINVAL;
   }
   file->f_pos = 0;
   return res;
}

/**
 * @brief Histograms driver RELEASE method
 * @param inode  inode
 * @param file   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int histograms_release(struct inode *inode, struct file *file) {
  int res=0;
  int p = MINOR(inode->i_rdev);
  MDF21(printk("minor=0x%x\n",p));
  switch ( p ) {
    case  CMOSCAM_MINOR_HISTOGRAMS :
        break;
    default:
        return -EINVAL; //! do not need to free anything - "wrong number"
    }
  kfree(file->private_data);
  return res;
}

/**
 * @brief Histograms driver LSEEK  method (and execute commands)
 * lseek (SEEK_SET, value)   wait for histogram of the absolute frame 'value' (G1 or all depending on wait_mode
 *                           locate frame number value and set frame_index (and file pointer) to the result.
 *                           Return error if frame can not be found, otherwise - histogram index (to use with mmap)
 *                           Calculate missing tables according to "needed" variable
 * lseek (SEEK_CUR, value)   wait for histogram of the  frame 'value' from the current one (G1 or all depending on wait_mode
 *                           locate frame number value and set frame_index (and file pointer) to the result.
 *                           Return error if frame can not be found, otherwise - histogram index (to use with mmap)
 *                           Calculate missing tables according to "needed" variable
 *                           lseek (SEEK_CUR, 0) will wait for the histogram(s) for current frame
 * lseek (SEEK_CUR, value) - ignore value, return frame_index (may be negative if error)
 * lseek (SEEK_END, value < 0) - do nothing?, do not modify file pointer, return error
 * lseek (SEEK_END, value = 0) - return HISTOGRAMS_FILE_SIZE
 * lseek (SEEK_END, LSEEK_HIST_WAIT_Y) - set histogram waiting for the Y (actually G1) histogram (default after open)
 * lseek (SEEK_END, LSEEK_HIST_WAIT_C) - set histogram waiting for the C (actually R, G2, B) histograms to become available - implies G1 too
 * lseek (SEEK_END, LSEEK_HIST_NEEDED) /// LSEEK_HIST_NEEDED | (0..0xffff) set histogram "needed" bits 
 * lseek (SEEK_END, LSEEK_HIST_REQ_EN) - (default)enable histogram request when reading histogram (safer, but may be not desirable in HDR mode) - default after opening
 * lseek (SEEK_END, LSEEK_HIST_REQ_DIS) - disable histogram request when reading histogram - will read latest available relying it is available
 * @param file 
 * @param offset 
 * @param orig SEEK_SET, SEEK_CUR or SEEK_SET END
 * @return file position (absolute frame number)
 */
/// TODO: add flag that will allow driver to wakeup processes before the specified frame comes ?
loff_t histograms_lseek (struct file * file, loff_t offset, int orig) {
   struct histograms_pd * privData = (struct histograms_pd *) file->private_data;
   unsigned long reqAddr,reqFrame;

 MDF21(printk("offset=0x%x, orig=0x%x\n",(int) offset, (int) orig));
   switch (privData->minor) {
       case CMOSCAM_MINOR_HISTOGRAMS :
         switch(orig) {
         case SEEK_CUR: /// ignore offset
           offset+=(privData-> wait_mode)?GLOBALPARS(G_HIST_C_FRAME):GLOBALPARS(G_HIST_Y_FRAME);
         case SEEK_SET:
           privData->frame=offset;
/// Try to make some precautions to avoid waiting forever - if the past frame is requested - request histogram for the current frame,
/// if the "immediate" future (fits into the array of frames) one  - request that frame's  histogram
/// if in the far future (unsafe) do nothing -NOTE: far future should be avoided if the histograms are set request-only
/// NOTE: there could be another wrong condition - request written w/o "JUST_THIS" modifier - then it will turn to always on until cleared.
/// TODO: Save time on always enabled histograms? Don't request them additionally?
           if (privData->request_en) {
             reqAddr=(privData-> wait_mode)?P_HISTRQ_C:P_HISTRQ_Y;
             reqFrame=getThisFrameNumber();
             if (offset > reqFrame) {
               if (offset > (reqFrame+5)) reqFrame+=5;
               else                       reqFrame=offset;
             }
             if ((offset < reqFrame) && /// if the requested frame is in the past - try to get it first before requesting a new
                (((privData->frame_index = get_histograms (offset, privData->needed))) >=0)) {
                 file->f_pos=privData->frame_index;
                 return file->f_pos;
             }
/// request histogram(s)
             setFramePar(&framepars[reqFrame & PARS_FRAMES_MASK], reqAddr, 1);
/// make sure (harmful) interrupt did not happen since getThisFrameNumber()
             if (reqFrame < getThisFrameNumber()) {
               setFramePar(&framepars[getThisFrameNumber() & PARS_FRAMES_MASK], reqAddr, 1);
             }
           }
           if (privData-> wait_mode)  wait_event_interruptible (hist_c_wait_queue,GLOBALPARS(G_HIST_C_FRAME)>=offset);
           else                       wait_event_interruptible (hist_y_wait_queue,GLOBALPARS(G_HIST_Y_FRAME)>=offset);
           privData->frame_index = get_histograms (offset, privData->needed);
           if (privData->frame_index <0) {
             return -EFAULT;
           } else {
             file->f_pos=privData->frame_index;
             return file->f_pos;
           }
           break; /// just in case
         case SEEK_END:
           if (offset < 0) {
             return -EINVAL;
           } else {
             if (offset < LSEEK_HIST_NEEDED) {
               switch (offset) {
                 case 0:
                   break;
                 case  LSEEK_HIST_REQ_EN: /// enable requesting histogram for the specified frame (0 - rely on the available ones)
                   privData->request_en=1; ///default after open
                   break;
                 case  LSEEK_HIST_REQ_DIS: /// disable requesting histogram for the specified frame, rely on the available ones
                   privData->request_en=0;
                   break;
                 case  LSEEK_HIST_WAIT_Y: /// set histogram waiting for the Y (actually G1) histogram (default after open)
                   privData-> wait_mode=0;
                   break;
                 case  LSEEK_HIST_WAIT_C: /// set histogram waiting for the C (actually R, G2, B) histograms to become available - implies G1 too
                   privData-> wait_mode=1;
                   break;
                 default:
                   switch (offset & ~0x1f) {
                     case  LSEEK_DAEMON_HIST_Y: /// wait for daemon enabled and histograms Y ready
                       MDF21(printk("wait_event_interruptible (hist_y_wait_queue,0x%x & 0x%x)\n",(int) get_imageParamsThis(P_DAEMON_EN), (int) (1<<(offset & 0x1f))));
                       wait_event_interruptible (hist_y_wait_queue, get_imageParamsThis(P_DAEMON_EN) & (1<<(offset & 0x1f)));
                       break;
                     case  LSEEK_DAEMON_HIST_C: /// wait for daemon enabled and histograms Y ready
                       MDF21(printk("wait_event_interruptible (hist_c_wait_queue,0x%x & 0x%x)\n",(int) get_imageParamsThis(P_DAEMON_EN), (int) (1<<(offset & 0x1f))));
                       wait_event_interruptible (hist_c_wait_queue, get_imageParamsThis(P_DAEMON_EN) & (1<<(offset & 0x1f)));
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
         default:  /// not SEEK_SET/SEEK_CUR/SEEK_END 
            return -EINVAL;
         } /// switch (orig)
         return  file->f_pos ;
       default: /// other minors
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
  MDF21(printk("minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case  CMOSCAM_MINOR_HISTOGRAMS :
           result=remap_pfn_range(vma,
                  vma->vm_start,
                 ((unsigned long) virt_to_phys(histograms_p)) >> PAGE_SHIFT, /// Should be page-aligned
                  vma->vm_end-vma->vm_start,
                  vma->vm_page_prot);
           MDF21(printk("remap_pfn_range returned=%x\r\n",result));
           if (result) return -EAGAIN;
           return 0;
       default: return -EINVAL;
  }
}

/**
 * @brief Histograms driver init
 * @return 0
 */
static int __init histograms_init(void) {
   int res;
   init_histograms();
   res = register_chrdev(HISTOGRAMS_MAJOR, "gamma_tables_operations", &histograms_fops);
   if(res < 0) {
     printk(KERN_ERR "histograms_init: couldn't get a major number %d.\n",HISTOGRAMS_MAJOR);
     return res;
   }
//   init_waitqueue_head(&histograms_wait_queue);
   init_waitqueue_head(&hist_y_wait_queue);    /// wait queue for the G1 histogram (used as Y)
   init_waitqueue_head(&hist_c_wait_queue);    /// wait queue for all the other (R,G2,B) histograms (color)
   printk(X3X3_HISTOGRAMS_DRIVER_NAME"\r\n");
   return 0;
}


module_init(histograms_init);
MODULE_LICENSE("GPLv3.0");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_HISTOGRAMS_DRIVER_NAME);






















