/** @file framepars.c */
/*!********************************************************************************
*! FILE NAME  : framepars.c
*! DESCRIPTION: Handling of frame parameters, making use of FPGA i2c
*! and command sequencer that accepts commands up to 6 frames ahead.
*! This module includes parameter storage, code called from ISR,
*! from other kernel drivers as well as from the user space
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
*!senssensor_common.hor_common.h
*!  You should have received a copy of the GNU General Public License
*!  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*! -----------------------------------------------------------------------------**
*!  $Log: framepars.c,v $
*!  Revision 1.14  2011/12/22 05:39:07  elphel
*!  catching up after some missed interrupts
*!
*!  Revision 1.13  2010/08/10 21:10:41  elphel
*!  portrait mode now includes all 4 rotations (2 bits)
*!
*!  Revision 1.12  2010/08/03 23:37:34  elphel
*!  rev 8.0.8.37, portrait mode support
*!
*!  Revision 1.11  2010/05/25 00:52:23  elphel
*!  8.0.8.20, working on multi-sensor
*!
*!  Revision 1.10  2010/05/21 06:12:16  elphel
*!  continue working on multi-sensor software
*!
*!  Revision 1.9  2010/05/16 02:03:47  elphel
*!  8.0.8.4 - driver working with individual/broadcast sensor registers
*!
*!  Revision 1.8  2010/05/13 03:39:31  elphel
*!  8.0.8.12 -  drivers modified for multi-sensor operation
*!
*!  Revision 1.7  2010/04/28 02:34:33  elphel
*!  8.0.6.6 - added support for linescan mode (also useful for high fps small images). 2.5K full line pairs/second with 5MPix sensor
*!
*!  Revision 1.6  2010/04/06 20:35:42  elphel
*!  8.0.7.5 - made the fpgaclock driver program 10359 clock in addition to the system one
*!
*!  Revision 1.5  2010/01/27 22:51:52  elphel
*!  turned off ELPHEL_DEBUG, fixed errors caused by that.
*!
*!  Revision 1.4  2008/12/02 19:08:54  elphel
*!  Bug fixin setFramePar()
*!
*!  Revision 1.3  2008/11/30 05:01:03  elphel
*!  Changing gains/scales behavior
*!
*!  Revision 1.2  2008/11/28 08:17:09  elphel
*!  keeping Doxygen a little happier
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
*!
*!
*!  Revision 1.41  2008/11/17 06:42:37  elphel
*!  added SETFRAMEREL - skipping specified number of frames from current (through lseek)
*!
*!  Revision 1.40  2008/11/14 07:09:48  elphel
*!  additional test to prevent "JUST_THIS" parameters to be written to the future-most frame (otherwise it can get stuck)
*!
*!  Revision 1.39  2008/11/13 05:40:45  elphel
*!  8.0.alpha16 - modified histogram storage, profiling
*!
*!  Revision 1.38  2008/11/05 02:01:25  elphel
*!  Added bit field manipulation in parameters
*!
*!  Revision 1.37  2008/11/02 00:31:25  elphel
*!  reduced required initialization steps
*!
*!  Revision 1.36  2008/10/29 04:18:28  elphel
*!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*!
*!  Revision 1.35  2008/10/25 19:59:48  elphel
*!  added lseek() calls to enable/disable daemons at events (compressed frame available, any frame available, histogram-Y and histograms-C available)
*!
*!  Revision 1.34  2008/10/23 18:25:40  elphel
*!  removed unneeded test condition before calling wait_event_interruptible()
*!
*!  Revision 1.33  2008/10/23 08:03:38  elphel
*!  cleanup
*!
*!  Revision 1.32  2008/10/21 21:28:26  elphel
*!  minor bug fix
*!
*!  Revision 1.31  2008/10/20 18:46:36  elphel
*!  all the functions for the same target frame are processed in the order regardless of latencies
*!
*!  Revision 1.30  2008/10/19 06:51:51  elphel
*!  rearranged initialization, added frame number reset (to avoid unlikely integer overflow)
*!
*!  Revision 1.29  2008/10/17 05:44:48  elphel
*!  fixing latencies
*!
*!  Revision 1.28  2008/10/15 22:28:56  elphel
*!  snapshot 8.0.alpha2
*!
*!  Revision 1.27  2008/10/12 06:13:10  elphel
*!  snapshot
*!
*!  Revision 1.26  2008/10/11 18:46:07  elphel
*!  snapshot
*!
*!  Revision 1.25  2008/10/10 17:06:59  elphel
*!  just a snapshot
*!
*!  Revision 1.24  2008/10/08 21:26:25  elphel
*!  snapsot 7.2.0.pre4 - first images (actually - second)
*!
*!  Revision 1.23  2008/10/06 08:31:08  elphel
*!  snapshot, first images
*!
*!  Revision 1.22  2008/10/05 05:13:33  elphel
*!  snapshot003
*!
*!  Revision 1.21  2008/10/04 16:10:12  elphel
*!  snapshot
*!
*!  Revision 1.20  2008/09/28 00:31:57  elphel
*!  snapshot
*!
*!  Revision 1.19  2008/09/25 00:58:11  elphel
*!  snapshot
*!
*!  Revision 1.18  2008/09/20 00:29:50  elphel
*!  moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
*!
*!  Revision 1.17  2008/09/19 04:37:25  elphel
*!  snapshot
*!
*!  Revision 1.16  2008/09/16 00:49:31  elphel
*!  snapshot
*!
*!  Revision 1.15  2008/09/12 20:40:11  elphel
*!  snapshot
*!
*!  Revision 1.14  2008/09/12 00:28:54  elphel
*!  typo fixed
*!
*!  Revision 1.13  2008/09/12 00:23:59  elphel
*!  removed cc353.c, cc353.h
*!
*!  Revision 1.12  2008/09/07 19:48:08  elphel
*!  snapshot
*!
*!  Revision 1.11  2008/09/05 23:20:26  elphel
*!  just a snapshot
*!
*!  Revision 1.10  2008/09/04 17:37:13  elphel
*!  documenting
*!
*!  Revision 1.9  2008/09/02 21:01:06  elphel
*!  just next...
*!
*!  Revision 1.8  2008/07/27 04:27:49  elphel
*!  next snapshot
*!
*!  Revision 1.7  2008/06/24 00:43:44  elphel
*!  just a snapshot
*!
*!  Revision 1.6  2008/06/20 03:54:19  elphel
*!  another snapshot
*!
*!  Revision 1.5  2008/06/19 02:17:36  elphel
*!  continuing work - just a snapshot
*!
*!  Revision 1.4  2008/06/16 06:51:21  elphel
*!  work in progress, intermediate commit
*!
*!  Revision 1.3  2008/06/10 00:03:14  elphel
*!  storing past frame data - subset of the frame parameters
*!
*!  Revision 1.2  2008/06/08 23:48:39  elphel
*!  minor cleanup
*!
*!  Revision 1.1  2008/05/26 23:33:00  elphel
*!  Added driver to handle multi-frame parameters
*!
*!
*/

//copied from cxi2c.c - TODO:remove unneeded


#include <linux/types.h> /// div for 64
#include <asm/div64.h>  /// div for 64


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
#include <linux/of.h>
#include <linux/of_device.h>

//#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
#include <elphel/exifa.h>
//#include "fpgactrl.h"  // defines port_csp0_adsensor_common.hdr, port_csp4_addr
//#include "cc3x3.h"
//#include "x3x3.h"           // hardware definitions


#include "sensor_common.h"
#include "framepars.h"
#include "param_depend.h" // specifies what functions should be called for different parameters changed
/// needed for lseek commands
//#include "cxdma.h" // x313_dma_init
//#include "cci2c.h" // to use void i2c_reset_wait(void), reset shadow static 'i2c_hardware_on'
#include "x393_macro.h"

/**
 * \def MDF1(x) optional debug output 
 */

#if ELPHEL_DEBUG
 #define MDF(x) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;}
 #define MDF2(x) { if (GLOBALPARS(G_DEBUG) & (1 <<2)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
/// setFrameParsAtomic
 #define MDF5(x) { if (GLOBALPARS(G_DEBUG) & (1 <<5)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define   D5(x) { if (GLOBALPARS(G_DEBUG) & (1 <<5)) {x ;} }
/// processPars
 #define MDF6(x) { if (GLOBALPARS(G_DEBUG) & (1 <<6)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define   D6(x) { if (GLOBALPARS(G_DEBUG) & (1 <<6)) {x ;} }
///update FramePars
 #define MDF7(x) { if (GLOBALPARS(G_DEBUG) & (1 <<7)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define   D7(x) { if (GLOBALPARS(G_DEBUG) & (1 <<7)) {x ;} }
/// setFramePar[s]
 #define MDF8(x) { if (GLOBALPARS(G_DEBUG) & (1 <<8)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define   D8(x) { if (GLOBALPARS(G_DEBUG) & (1 <<8)) {x ;} }
 #define ELPHEL_DEBUG_THIS 0
// #define ELPHEL_DEBUG_THIS 1
#else
 #define MDF(x)
 #define MDF2(x)
 #define MDF5(x)
 #define D5(x)
 #define MDF6(x)
 #define D6(x)
 #define MDF7(x)
 #define D7(x)
 #define MDF8(x)
 #define D8(x)
 #define ELPHEL_DEBUG_THIS 0
#endif


#if ELPHEL_DEBUG_THIS
  #define MDD1(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__); x ; udelay (ELPHEL_DEBUG_DELAY)
  #define MDF1(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__); x
  #define D1(x) x
  #define D1I(x)

#else
  #define MDD1(x)
  #define MDF1(x)
  #define D1(x)
  #define D1I(x) x
#endif
//#define ELP_KERR(x) printk("%s:%d:%s: ERROR ",__FILE__,__LINE__,__FUNCTION__); x

/**
 * \def FRAMEPARS_DRIVER_NAME driver name to display
 */
#define  FRAMEPARS_DRIVER_NAME "Elphel (R) Model 393 Frame Parameters device driver"

static const struct of_device_id elphel393_framepars_of_match[];
static struct framepars_all_t sFrameParsAll __attribute__ ((aligned (PAGE_SIZE)));   ///< Sensor Parameters, currently 8 pages all and 2048 pages some, static struct
unsigned long frameParsInitialized; /// set to 0 at startup, 1 after initialization that is triggered by setParsAtomic()
#define  thisFrameNumber GLOBALPARS(G_THIS_FRAME) // Current frame number (may lag from the hardware)
//#define  THISFRAMENUMBER GLOBALPARS(G_THIS_FRAME) // Current frame number (may lag from the hardware)

struct framepars_all_t  *frameparsall=NULL; /// - will be mmap-ed
struct framepars_t      *framepars=   NULL; ///< getting rid of static to be able to use extern
struct framepars_past_t *pastpars=    NULL; ///< getting rid of static to be able to use extern
unsigned long           *funcs2call=  NULL; ///  sFrameParsAll.func2call.pars; - each parameter has a 32-bit mask of what pgm_function to call - other fields not used
unsigned long           *globalPars=  NULL; /// parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both 
unsigned long           *multiSensIndex= NULL; /// index for per-sensor alternatives
unsigned long           *multiSensRvrsIndex=NULL; /// reverse index (to parent) for the multiSensIndex
wait_queue_head_t framepars_wait_queue; /// used to wait for the frame to be acquired

/**
 * @brief file private data
 */
struct framepars_pd {
    int                minor; ///< file minor value
    struct wait_queue *framepars_wait_queue; ///< wait queue (waiting for file number to increase)  ///NOTE: not used at all?
// something else to be added here?
};



/**
 * @brief assign non-static pointers to static data to be used as extern
 */
void init_framepars_ptr(void) {
    frameparsall=      &sFrameParsAll; /// - will be mmap-ed
    framepars =         sFrameParsAll.framePars;
    pastpars =          sFrameParsAll.pastPars;
    funcs2call =        sFrameParsAll.func2call.pars; /// each parameter has a 32-bit mask of what pgm_function to call - other fields not used
    globalPars =        sFrameParsAll.globalPars; /// parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both 
    multiSensIndex=     sFrameParsAll.multiSensIndex; /// indexes of individual sensor register shadows (first of 3) - now for all parameters, not just sensor ones
    multiSensRvrsIndex= sFrameParsAll.multiSensRvrsIndex; /// reverse index (to parent) for the multiSensIndex

}

int        framepars_open   (struct inode *inode, struct file *filp);
int        framepars_release(struct inode *inode, struct file *filp);
loff_t     framepars_lseek  (struct file * file, loff_t offset, int orig);
ssize_t    framepars_write  (struct file * file, const char * buf, size_t count, loff_t *off);
int        framepars_mmap   (struct file *file, struct vm_area_struct *vma);

/**
 * @brief Reset hardware sequencers (i2c, command) and initialize framepars structure
 */
void initSequencers(void) {
  unsigned long flags;
  MDF2(printk ("\n"));
  printk ("initSequencers:resetting both sequencers\n");
#ifdef TEST_DISABLE_CODE
  local_irq_save(flags);
  X3X3_SEQ_RESET;
  i2c_reset_wait();
  local_irq_restore(flags);
  initFramePars();
#endif
}

/**
 * @brief reset absolute frame number \b thisFrameNumber to \b frame8
 */
void resetFrameNumber(void) {
   int i;
#ifdef TEST_DISABLE_CODE
   thisFrameNumber= X3X3_I2C_FRAME;
#endif
  MDF2(printk (" thisFrameNumber=0x%lx\n",thisFrameNumber));
// write absolute frame numbers
   for (i=thisFrameNumber; i<(thisFrameNumber+PARS_FRAMES); i++) framepars[i & PARS_FRAMES_MASK].pars[P_FRAME]=i;
/// initialize frameParsDeps.pars masks:
}

/**
 * @brief initialize all parameters, set \b thisFrameNumber to \b frame number read from hardware hardware ( 0 after resetting i2c and cmd_seq)
 */
void initFramePars(void) {
   int i;
   memset(framepars, 0, sizeof(framepars));
   resetFrameNumber();
/// initialize frameParsDeps.pars masks:
   for (i=0; i < (sizeof(param_depend_tab)/8); i++) {
      funcs2call[param_depend_tab[2*i] & 0xffff]=param_depend_tab[2*i+1]; /// remove possible flags
      MDF2(printk("funcs2call[0x%lx]=0x%08lx\n",param_depend_tab[2*i] & 0xffff,param_depend_tab[2*i+1]));
   }
   for (i=0; i < P_SENSOR_NUMREGS; i++)    funcs2call[P_SENSOR_REGS+i] = ONCHANGE_SENSORREGS; /// by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function
/// Same for 10359 registers - will not change anything if there is no 10359 - these registers will not be chnaged, and if will be it wil cause no action
   for (i=0; i < P_M10359_NUMREGS; i++)    funcs2call[P_M10359_REGS+i] = ONCHANGE_SENSORREGS; /// by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function

   initMultiPars();     /// initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called after/during sensor detection
   frameParsInitialized=1;
}

/**
 * @brief reset all global parameters, set default for debug mask (if ELPHEL_DEBUG)
 */
void initGlobalPars(void) {
   memset(&globalPars[GLOBALS_PRESERVE], 0, sizeof(globalPars)-GLOBALS_PRESERVE*sizeof(globalPars[0]));
///   MDF(GLOBALPARS(G_DEBUG) = ELPHEL_DEBUG_STARTUP;// removed - add write to fpga init script
   MDF(printk("GLOBALPARS(G_DEBUG)=%lx\n",GLOBALPARS(G_DEBUG)));
}

/**
 * @brief initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called after/during sensor detection
 * @return number of multi-regs
 */
int initMultiPars(void) {
   int i,j,n;
   int ireg=P_MULTI_REGS; /// multi-reg shadows start index
   unsigned long m;
   memset(multiSensIndex, 0, sizeof(multiSensIndex));
   memset(multiSensRvrsIndex, 0, sizeof(multiSensRvrsIndex));
   GLOBALPARS(G_MULTI_NUM)=0;
   for (i=0;i<8;i++) {
     m=GLOBALPARS(G_MULTI_REGSM+i); /// 1 bit per register that need individual shadows
//     MDF(printk("i=%d, m=0x%lx\n",i,m));
     for (j= P_SENSOR_REGS +(i<<5); m && (GLOBALPARS(G_MULTI_NUM)<P_MULTI_NUMREGS) ; j++, m >>= 1) {
       if (m & 1) {
         multiSensIndex[j]=ireg;
//         MDF(printk("j=0x%x ireg=0x%x\n",j,ireg));
         for (n=0;n<MAX_SENSORS;n++) {
           funcs2call[ireg] = ONCHANGE_SENSORREGS; /// by default each "manual" write to any of these registers will trigger pgm_sensorreg function
           multiSensRvrsIndex[ireg++]=j | ((n+1)<<16);
         }
         GLOBALPARS(G_MULTI_NUM)++;
       }
     }
   }
   for (i=0; i < P_SENSOR_NUMREGS; i++)    funcs2call[P_SENSOR_REGS+i] = ONCHANGE_SENSORREGS; /// by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function
   MDF(printk("GLOBALPARS(G_MULTI_NUM)=%lx\n",GLOBALPARS(G_MULTI_NUM)));
   return GLOBALPARS(G_MULTI_NUM);
}

/**
 * @brief reads parameters from the current frame (matching hardware index)
 * @param n number of a parameter to read
 * @return parameter value (unsigned long)
 */
inline unsigned long get_imageParamsThis    (int n) {
   return framepars[thisFrameNumber & PARS_FRAMES_MASK].pars[n] ;
}
//EXPORT_SYMBOL_GPL(get_imageParamsThis);

/**
 * @brief reads parameters from the previous frame (matching hardware index) - used to determine if historam was needed
 * @param n number of a parameter to read
 * @return parameter value (unsigned long)
 */
inline unsigned long get_imageParamsPrev    (int n) {
   return framepars[(thisFrameNumber-1) & PARS_FRAMES_MASK].pars[n] ;
}

/**
 * @brief writes read-only parameter to the current frame (does not propagate to next frames as setFramePar() does)
 * In most cases you really need to use setFramePar() instead;
 * @param n number of a parameter to set
 * @param d data to write to the selected parameter
 */

inline void set_imageParamsThis    (int n,unsigned long d) {
   framepars[thisFrameNumber & PARS_FRAMES_MASK].pars[n]=d ;
}

/**
 * @brief reads global (not related to particular frames) parameters
 * @param n number of a parameter to read (numbers start from FRAMEPAR_GLOBALS)
 * @return parameter value (unsigned long)
 */
inline unsigned long get_globalParam  (int n) {
   return GLOBALPARS(n) ;
}
//EXPORT_SYMBOL_GPL(get_globalParam);
/**
 * @brief sets global (not related to particular frames) parameters
 * @param n number of a parameter to set  (numbers start from FRAMEPAR_GLOBALS)
 * @param d data to write to the selected parameter
 */

inline void          set_globalParam   (int n, unsigned long d) {
  GLOBALPARS(n)=d;
}
//EXPORT_SYMBOL_GPL(set_globalParam);

/**
 * @brief set same parameters in all frames
 * currently used only in compressor reset
 * @param n number of a parameter to set
 * @param d data to write to the selected parameter
 */
inline void          set_imageParamsR_all(int n, unsigned long d) {
  int i;
  for (i=0; i < PARS_FRAMES; i++) framepars[i].pars[n]=d;
}

///++++++++++++++++++++++++++++++++++++++++++
/*!
 * @brief called from ISR - advance thisFrameNumber to match hardware frame8, copy parameters as needed.
 * before: (thisFrameNumber mod8      pointed to current (for the software) parameters frame (now behind by at least 1, maybe 2)
 *         (thisFrameNumber-1) mod 8 - oldest with parameters preserved, also containes histograms results (+image timestamp, size?)
 *                                    subset of that frame data is copied to pastpars
 *         (thisFrameNumber-2) mod 8 - farthest in the future frame
 * after:   thisFrameNumber matches hardware pointer
 * @param interframe_pars pointer to structure (between frames in the frame buffer) to save a pointer to past parameters
 *                   pass NULL if compressor was off (or no time to copy?)
 */
void updateFramePars(int frame8, struct interframe_params_t * interframe_pars) {
  int findex_this,findex_prev,findex_future,findex_next;
  int index,index32;
  unsigned long bmask, bmask32;
  int pastParsIndex;
/// If interrupt was from compression done (circbuf advanced, interframe_pars!=null), the frame8 (hardware) maybe not yet advanced
/// We can fix it here, but it will not work if some frames were not processed in time
  if ((interframe_pars!=NULL) && (((frame8 ^ thisFrameNumber) & PARS_FRAMES_MASK)==0)){
    findex_this=  frame8  & PARS_FRAMES_MASK;
    if (framepars[findex_this].pars[P_IRQ_SMART] & 4) frame8= (frame8+1) &  PARS_FRAMES_MASK; // verify that this mode is enabled (together with bit0)
  }
  while ((frame8 ^ thisFrameNumber) & PARS_FRAMES_MASK) {
/// before update:
///   framepars[findex_prev]  holds previous frame data (oldest availble)
///   framepars[findex_future] holds fartherst in the future one
/// after update:
///   framepars[findex_prev]  holds fartherst in the future one ("this" will become "prev")
    findex_this=  thisFrameNumber  & PARS_FRAMES_MASK;
    findex_prev=  (findex_this-1)  & PARS_FRAMES_MASK;
    findex_future= (findex_this-2)  & PARS_FRAMES_MASK; // farthest in the future
    findex_next=  (findex_this+1)  & PARS_FRAMES_MASK;
/// copy subset of the parameters to the long buffer of past parameters. TODO: fill Exif also here?
/// TODO:DONE: Change - make pastpars be save for all frames, not just compressed ones
/// With PASTPARS_SAVE_ENTRIES being multiple of PARS_FRAMES - make it possible to calculate past_index from thisFrameNumber
//    pastParsIndex= thisFrameNumber & PASTPARS_SAVE_ENTRIES_MASK;
    pastParsIndex= (thisFrameNumber-1) & PASTPARS_SAVE_ENTRIES_MASK; /// copying from what was past frame that might include histogram data 
//    memcpy (pastpars[pastParsIndex].past_pars, &framepars[findex_prev].pars[PARS_SAVE_FROM], sizeof(pastpars[0].past_pars));
    memcpy (pastpars[pastParsIndex].past_pars, &framepars[findex_prev].pars[PARS_SAVE_FROM], PARS_SAVE_COPY<<2);

/// Now update interframe_pars (interframe area) used to create JPEG headers. Interframe area survives exactly as long as the frames themselves (not like pastpars)
    if (interframe_pars) { /// frame was compressed, not just vsync
///TODO: get rid of *_prev, use it for the future. 
      memcpy (interframe_pars,  &framepars[findex_this].pars[P_GTAB_R], 24); /// will leave some gaps, but copy [P_ACTUAL_WIDTH]
      interframe_pars->height=  framepars[findex_this].pars[P_ACTUAL_HEIGHT]; /// NOTE: P_ACTUAL_WIDTH,P_QUALITY copied with memcpy
      interframe_pars->color=   framepars[findex_this].pars[P_COLOR];
      interframe_pars->byrshift=framepars[findex_this].pars[P_COMPMOD_BYRSH];
      interframe_pars->quality2 |= (framepars[findex_this].pars[P_PORTRAIT] & 1) << 7;
    }
/// copy parameters from findex_future (old "fartherst in the future") to findex_prev (new "fartherst in the future") if it was changed since
    if ((bmask32=framepars[findex_prev].modsince32)) {
     MDF7(printk("framepars[%d].modsince32=0x%lx\n",findex_prev,bmask32));
      for (index32=0; bmask32; index32++, bmask32 >>= 1) {
         if (bmask32 & 1) {
           for (index=(index32<<5),bmask=framepars[findex_prev].modsince[index32]; bmask; index++, bmask >>= 1)
             if (bmask & 1) {
               framepars[findex_prev].pars[index] = framepars[findex_future].pars[index];
     MDF7(printk("hw=%d framepars[%d].pars[%d]=framepars[%d].pars[%d]=0x%lx\n",frame8, findex_prev,index,findex_future,index,framepars[findex_future].pars[index]));
             }
           framepars[findex_prev].modsince[index32]=0; ///  mark as not "modified since" (yet)
         }
      }
      framepars[findex_prev].modsince32=0;///  mark as not "modified since" super index
    }
/// clear "modified" and  flags on the brand new future frame
    if (framepars[findex_prev].mod32) memset(framepars[findex_prev].mod, 0, 32*4); /// .mod[0]-.mod[30], .mod32
    framepars[findex_prev].functions=0; /// No functions yet needed on the brand new frame
    framepars[findex_prev].pars[P_FRAME]=thisFrameNumber+7; /// that will be the full frame number
/// NOTE: Handle past due - copy functions, and mod if functions were non-zero
    if (framepars[findex_this].functions) { /// Some functions were not yet processed (past due)
      if (!(get_globalParam(G_TASKLET_CTL) & (1 << TASKLET_CTL_IGNPAST))) {
        framepars[findex_next].functions |= framepars[findex_this].functions;
        if ((bmask32=framepars[findex_this].mod32)) {
          for (index32=0; bmask32; index32++, bmask32 >>= 1) {
            if (bmask32 & 1) {
              framepars[findex_next].mod[index32] |= framepars[findex_this].mod[index32];
            }
            framepars[findex_next].mod32 |= framepars[findex_this].mod32;
          }
        }
        MDF7(printk ("resubmitting past due functions = 0x%lx for frame=%ld (0x%x)\n",framepars[findex_this].functions,thisFrameNumber,findex_this));
      } else {
        MDF(printk ("Ignored past due functions = 0x%lx for frame=%ld (0x%x)\n",framepars[findex_this].functions,thisFrameNumber,findex_this));
      }
    }
    thisFrameNumber++;
  } 
}


/**
 * @brief process parameters that are overdue or due in ASAP mode (not through the sequencer)
 * Called twice from processPars - at the beginning and at the end to finish off any derivatives (needed?)
 * @param sensorproc 
 * @param frame8 
 */
inline void processParsASAP (struct sensorproc_t * sensorproc, int frame8) {
  unsigned long todo, mask , remain;
  int pars_ahead;  /// considering parameter "pars_ahead" of the (frame8+job_ahead) mod 8
  int frame_proc;   /// current frame for which parameters are considered
  struct framepars_t * procpars;
  struct framepars_t * prevpars ; /// maybe - drop calculation for each function, move it to pgm_* where needed?
  unsigned long * p_nasap=& GLOBALPARS(G_CALLNASAP);
  int i;
  int rslt;
#if ELPHEL_DEBUG
  unsigned long allfunctions=framepars[0].functions  | framepars[1].functions | framepars[2].functions | framepars[3].functions |
                             framepars[4].functions | framepars[5].functions | framepars[6].functions | framepars[7].functions;
  if (allfunctions)       MDF6(printk("frame8=%d, functions: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n", frame8, framepars[0].functions,framepars[1].functions,framepars[2].functions,framepars[3].functions,framepars[4].functions,framepars[5].functions,framepars[6].functions,framepars[7].functions));

#endif
/// do all ASAP tasks (they should not be done ahead of the corresponding interrupt!) 
/// Now try overdue functions with latencies >=1 and try them in ASAP mode
  for (pars_ahead=0; pars_ahead <= 4;  pars_ahead++ ) {
    frame_proc=(frame8 + pars_ahead) & PARS_FRAMES_MASK;
    procpars  = &framepars[frame_proc];
    prevpars  = &framepars[(frame_proc-1) & PARS_FRAMES_MASK];
    i=0;
    mask=1;
    remain=0xffffffff;
    while ((todo=(pars_ahead)?
                   (p_nasap[pars_ahead] & (procpars->functions) & remain):
                   (procpars->functions & remain) )) { ///none, *1, *2,*3,*4

      while (!(todo & mask)) { /// skip zeros - todo will stay current (.functions will not change
        i++;
        mask   <<=1;
        remain <<=1;
      }
/// now (todo & mask) !=0
      MDF6(printk(" todo=0x%08lx (curr=0x%08lx) frame8=%d, pars_ahead=%d, frame_proc=%d i=%d, mask=0x%08lx\n", todo, procpars->functions, frame8,pars_ahead,frame_proc,i,mask));

      MDF6(printk(" %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n", framepars[0].functions,framepars[1].functions,framepars[2].functions,framepars[3].functions,framepars[4].functions,framepars[5].functions,framepars[6].functions,framepars[7].functions));

      if (sensorproc->pgm_func[i]) {
        rslt=sensorproc->pgm_func[i]    ( &(sensorproc->sensor), procpars, prevpars, -1);
      } else rslt=0; /// only sensor-specific function, nothing to do common to all sensors
      if ((rslt >= 0) && (sensorproc->pgm_func[i+32])) { /// sensor - specific functions, called after the main ones
        rslt=sensorproc->pgm_func[i+32] ( &(sensorproc->sensor), procpars, prevpars, -1);
      }
/// Nothing to do with errors here - just report?
      if (rslt<0) printk("%s:%d:%s - error=%d",__FILE__,__LINE__,__FUNCTION__, rslt);
      procpars->functions &= ~mask;
      MDF6(printk(".functions=0x%08lx)\n", procpars->functions));
      i++;
      mask   <<=1;
      remain <<=1;
    }
  }
}
/// Next 5 should go in that sequence
//#define G_CALLNASAP    119 // bitmask - what functions can be used not only in the current frame (ASAP) mode 
//#define G_CALLNEXT1      120 // bitmask of actions to be one   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT2..G_CALLNEXT4) 
//#define G_CALLNEXT2      121 // bitmask of actions to be two   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT3..G_CALLNEXT4)
//#define G_CALLNEXT3      122 // bitmask of actions to be three or more frames ahead of the programmed one (OR-ed with G_CALLNEXT4)
//#define G_CALLNEXT4      123 // bitmask of actions to be four  or more frames ahead of the programmed one


inline void processParsSeq (struct sensorproc_t * sensorproc, int frame8, int maxahead) {
  unsigned long todo, mask , remain;
  int job_ahead;   /// doing job "job_ahead" ahead of needed 
  int pars_ahead;  /// considering parameter "pars_ahead" of the (frame8+job_ahead) mod 8
  int frame_proc;   /// current frame for which parameters are considered
  struct framepars_t * procpars;
  struct framepars_t * prevpars ; /// maybe - drop calculation fpr each function, move it to pgm_* where needed?
  unsigned long * p_nasap=& GLOBALPARS(G_CALLNASAP);
  int seq_frame;  /// sequencer frame for which pgm_* function should schedule data
  int i;
  int rslt;
  int max_par_ahead;
  int this_ahead;
  if (maxahead > (PARS_FRAMES-3)) maxahead = PARS_FRAMES-3; /// use 5 if maxahead >5
/// commands that use FPGA queues for the i2c/sequencer commands, executed at frame syncs
/// Modifying - as soon as found the frame to process with non-zero masked .functions - process all functions for that
/// frame with appropriate sequencer frame.
/// For now - scan p_nasap[i] to find latency - improve that later
  for (job_ahead=0; job_ahead <= maxahead; job_ahead++ ) {
    max_par_ahead=min(5,(PARS_FRAMES-3) -job_ahead); 
    for (pars_ahead=0; pars_ahead < max_par_ahead;  pars_ahead++ ) {
      frame_proc=(frame8 + job_ahead + pars_ahead +1) & PARS_FRAMES_MASK; ///
      procpars  = &framepars[frame_proc];
/// Check if at least one function is needed for frame_proc
      if (procpars->functions &
                   p_nasap[pars_ahead] &  ///all, *1, *2,*3,*4 - for all will have G_CALLNASAP twice
                   p_nasap[0]) {
        prevpars  = &framepars[(frame_proc-1) & PARS_FRAMES_MASK];
//        seq_frame=  (frame8+job_ahead+1) & PARS_FRAMES_MASK;
        i=0;
        mask=1;
        remain=0xffffffff;
        while ((todo=procpars->functions &
///                     p_nasap[pars_ahead] &  ///all, *1, *2,*3,*4 - for all will have G_CALLNASAP twice
                     p_nasap[0] & remain)) { /// eliminate ASAP-only function
          while (!(todo & mask)) { /// skip zeros - todo will stay current (.functions will not change)
            i++;
            mask   <<=1;
            remain <<=1;
          }
/// now (todo & mask) !=0
/// find the right latency
          for (this_ahead=1; (p_nasap[this_ahead] & todo & mask) && (this_ahead <=4); this_ahead++); /// this_ahead==1..5
//          seq_frame=  (frame8 + job_ahead + this_ahead) & PARS_FRAMES_MASK;
          seq_frame=  (frame_proc+1-this_ahead) & PARS_FRAMES_MASK;

          MDF6(printk(" todo=0x%08lx (curr=0x%08lx) frame8=%d, frame_proc=%d, seq_frame=%d, i=%d, mask=0x%08lx\n", todo, procpars->functions, frame8,frame_proc,seq_frame,i,mask));
          MDF6(printk(" %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n", framepars[0].functions,framepars[1].functions,framepars[2].functions,framepars[3].functions,framepars[4].functions,framepars[5].functions,framepars[6].functions,framepars[7].functions));

          if (sensorproc->pgm_func[i]) {
            /// NOTE: Was (frame8+job_ahead +1) & PARS_FRAMES_MASK
            rslt=sensorproc->pgm_func[i]    ( &(sensorproc->sensor), procpars, prevpars, seq_frame); 
          } else rslt=0; /// only sensor-specific function, nothing to do common to all sensors
          if ((rslt >= 0) && (sensorproc->pgm_func[i+32])) { /// sensor - specific functions, called after the main ones
            rslt=sensorproc->pgm_func[i+32] ( &(sensorproc->sensor), procpars, prevpars, seq_frame);
          }
          if (rslt >= 0) {
            procpars->functions &= ~mask; /// mark it done
          } else {
            MDF6(printk("Error - function result was %d\n", rslt));
          }
          i++;
          mask   <<=1;
          remain <<=1;
        }
      }
    }
  }
}

/**
 * @brief Program image acquisition, according to the parameters changed
 * Called from ISR?
 * @param sensorproc pointer to sensor static parameters and functions
 * @param frame8     current hardware frame number
 * @param maxahead   maximal number of frames to program ahead of the current (make it  P_* parameter ?)
 * @return always 0 ?
 */
//TODO: "Do it later" should be the only reason not to erase todo bit
//#define P_CALLASAP       107 // bitmask - what functions work only in the current frame (ASAP) mode 

void processPars (struct sensorproc_t * sensorproc, int frame8, int maxahead) {
   frame8 &= PARS_FRAMES_MASK;
/// first - do all ASAP tasks (they should not be done ahead of the corresponding interrupt!)
//   MDF6(printk("before first processParsASAP\n"));
   processParsASAP (sensorproc, frame8);
/// now - the rest commands that use FPGA queues for the i2c/sequencer commands, executed at frame syncs
/// for jobahead =0 it is still possible to have some functions in ASAP mode with non-zero latency
//   MDF6(printk("before processParsSeq\n"));
   processParsSeq (sensorproc, frame8, maxahead);
/// re-test ASAP tasks - they might appear as a result of other commands executed
//   MDF6(printk("before second processParsASAP\n"));
   processParsASAP (sensorproc, frame8);
}

/**
 * @brief schedule pgm_func to be executed for selected frame (frame8)
 * @param frame8 frame number (3-bit) to schedule a function for
 * @param func_num function number to schedule
 */
void schedule_pgm_func(int frame8, int func_num) {
   MDF1(printk(" frame8=%d, func_num=%d\n", frame8, func_num));
  framepars[frame8 & PARS_FRAMES_MASK].functions |= 1 << func_num;
}

/**
 * @brief schedule pgm_func to be executed for this_framepars->pars[P_FRAME] & PARS_FRAMES_MASK
 * @param this_framepars pointer to frame parameters structure
 * @param func_num number of function to schedule
 */
void schedule_this_pgm_func(struct framepars_t * this_framepars, int func_num) {
  int frame8=this_framepars->pars[P_FRAME] & PARS_FRAMES_MASK;
   MDF1(printk(" frame8=%d, func_num=%d\n", frame8, func_num));
  framepars[frame8].functions |= 1 << func_num;
}


/**
 * @brief just return current thisFrameNumber
 * @return current value of thisFrameNumber
 */
unsigned long getThisFrameNumber(void) { 
  return thisFrameNumber;
}


/**
 * @brief Set parameters that will never change (usually after sensor discovery)
 * @param numPars number of parameters to set
 * @param pars array of parameters (number/value pairs)
 * @return always 0
 */
int setFrameParsStatic(int numPars, struct frameparspair_t * pars) { 
  int npar,nframe,index;
  for (npar=0; npar < numPars; npar++) {
    index=pars[npar].num;
    if (index > P_MAX_PAR) return -ERR_FRAMEPARS_BADINDEX;
    for (nframe=0; nframe < PARS_FRAMES; nframe++) {
      framepars[nframe].pars[index]=pars[npar].val;
    }
  }
  return 0;
}

/**
 * @brief set parameters for the specified frame  (atomic, with interrupts off). Used from applications through driver write
 * @param frameno absolute (full) frame number parameters should be applied to
 * @param maxLatency maximal command latency (parameters should be set not less than maxLatency ahead of the current frame)
 * maxLatency < 0 - don't check latency (i.e.only commands that are not releted to particular frames)
 * @param numPars number of parameters to set (0 is OK to just test if it is too early/too late)
 * @param pars array of parameters (number/value pairs). FRAMEPAIR_FORCE_NEW modifier to parameter number 
 * @return 0 - OK, -ERR_FRAMEPARS_TOOEARLY, -ERR_FRAMEPARS_TOOLATE
 */
///TODO: Check that writes never to the future or past frame (only 6 of 8 are allowed). Have seen just_this to flood all
int setFrameParsAtomic(unsigned long frameno, int maxLatency, int numPars, struct frameparspair_t * pars) {
  unsigned long flags;
  int npar,nframe;
  unsigned long val, bmask, bmask32;
  int index,bindex;
  if (!frameParsInitialized) {
     initSequencers(); /// Will call  initFramePars(); and initialize functions
  }
  int findex_this=  thisFrameNumber & PARS_FRAMES_MASK;
  int findex_prev= (findex_this-1)  & PARS_FRAMES_MASK;
  int findex_future=(findex_this-2)  & PARS_FRAMES_MASK; /// actually - fartherst in the future??
//  int frame8=       frameno         & PARS_FRAMES_MASK;
  int frame8;
  MDF2(printk (": frameno=0x%lx, findex_this=%d (0x%lx) maxLatency=%d, numPars=%d\n",frameno, findex_this, thisFrameNumber, maxLatency, numPars));
  D1I(local_irq_save(flags));
  PROFILE_NOW(6);
  if (maxLatency>=0) {
    if      (frameno <= (thisFrameNumber+maxLatency)) {
      D1I(local_irq_restore(flags));
      return -ERR_FRAMEPARS_TOOLATE;
    }
    else if (frameno >= (thisFrameNumber+ (PARS_FRAMES-1))) {
      D1I(local_irq_restore(flags));
      return -ERR_FRAMEPARS_TOOEARLY;
    }
  }
  /// not too late, not too early, go ahead
  for (npar=0; npar < numPars; npar++) {
    D5(printk(" --pars[%d].num=0x%lx, pars[%d].val=0x%lx",npar,pars[npar].num, npar, pars[npar].val));
//    frame8=      (pars[npar].num & FRAMEPAR_GLOBALS)? -1: (frameno  & PARS_FRAMES_MASK);
    frame8=      frameno  & PARS_FRAMES_MASK; 
    val=pars[npar].val;
    index= pars[npar].num & 0xffff;
    if (index> ((index >= FRAMEPAR_GLOBALS)? (P_MAX_GPAR+FRAMEPAR_GLOBALS): P_MAX_PAR)) {
      D1I(local_irq_restore(flags));
      return -ERR_FRAMEPARS_BADINDEX;
    }
    D5(printk(" index=0x%x, val=0x%lx",index, val));
    if (index >= FRAMEPAR_GLOBALS) { /// ignore frame logic, set "static" parameters to frame 0
      if (pars[npar].num & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
        val= FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, GLOBALPARS(index), val);
      }
      GLOBALPARS(index)=val;
      D5(printk(" set GLOBALPARS(0x%x)=0x%lx\n",index,val));
    } else if (pars[npar].num  & FRAMEPAIR_FRAME_FUNC) {
      funcs2call[index] = val;
      D5(printk(" set funcs2call[0x%x]=0x%lx\n",index,val));
//    } else if ((frameno !=findex_prev) && (frameno != findex_future)) { /// do not write parameters in the future otherwise 
    } else if ((frame8 != findex_future) || ((pars[npar].num & FRAMEPAIR_JUST_THIS)==0)) { /// do not write "JUST_THIS" parameters in the future otherwise they'll stick
      if (pars[npar].num & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
        val= FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, framepars[frame8].pars[index], val);
      }
//TODO: optimize to use mask several parameters together
      D5(printk(" frame8=0x%x\n",frame8));
      if ((framepars[frame8].pars[index]!= val) || (pars[npar].num & FRAMEPAIR_FORCE_NEW)){
        bmask=   1 << (index & 31);
        bindex = index >> 5;
        bmask32= 1 << bindex;
///   Set this parameter for specified frame
        framepars[frame8].pars[index]       = val;
        framepars[frame8].mod[bindex]      |= bmask;
        framepars[frame8].mod32            |= bmask32;
        framepars[frame8].functions        |= funcs2call[index]; ///Mark which functions will be needed to process the parameters 
        D5(printk(" bindex=0x%x, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",bindex, bmask, bmask32, framepars[frame8].functions));
/// Write parameter to the next frames up to the one that have the same parameter already modified (only if not FRAMEPAIR_JUST_THIS)
        if ((pars[npar].num & FRAMEPAIR_JUST_THIS)==0) {
          MDF5(printk (":        ---   setting next frames"));
          for (nframe=(frame8+1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe=(nframe+1) & PARS_FRAMES_MASK) {
            framepars[nframe].pars[index] = val;
            D5(printk (" %d",nframe));
          }
          frame8=(frame8-1) & PARS_FRAMES_MASK;/// for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
          D5(printk ("\n"));
        }
/// Mark this parameter in all previous frames as "modified since"
/// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
        for (nframe=frame8; nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) { ///NOTE: frame8 is modified here
          framepars[nframe].modsince[bindex] |= bmask;
          framepars[nframe].modsince32       |= bmask32;
        }
      }
    } else { /// error - trying to write "just this" to the "future" - that would stick if allowed
      D1I(local_irq_restore(flags));
      ELP_KERR(printk("Tried to write JUST_THIS parameter (0x%lx) too far in the future", pars[npar].num));
      return -ERR_FRAMEPARS_TOOEARLY;
    }
  }
/// Try to process parameters immediately after written. If 0, only non-ASAP will be processed to prevent
/// effects of uncertainty of when was it called relative to frame sync
/// Changed to all (don't care about uncertainty - they will trigger only if it is too late or during sensor detection/initialization)
  if (!(get_globalParam(G_TASKLET_CTL) & (1<< TASKLET_CTL_NOSAME))) {
//    processParsSeq (sensorproc, thisFrameNumber & PARS_FRAMES_MASK, 0); ///maxahead=0, the rest will be processed after frame sync, from the tasklet
    MDF5(printk ("\n"));
    processPars (sensorproc, thisFrameNumber & PARS_FRAMES_MASK, 0); ///maxahead=0, the rest will be processed after frame sync, from the tasklet
  }
  PROFILE_NOW(7);
  D1I(local_irq_restore(flags));
  return 0;
}
//#define FRAMEPAIR_JUST_THIS     0x40000 // write only to this frame, don't propagate
                                         // (like "single frame" - compressor, sensor) first write "stop", then - "single" with FRAMEPAIR_JUST_THIS
/**
 * @brief set a single output (calculated) parameter for the frame referenced by this_framepars structure.
 * Shedules action only if the FRAMEPAIR_FORCE_PROC modifier bit is set in mindex
 * @param this_framepars pointer to the current parameters structure
 * @param mindex parameter number (with optional modifiers in high bits)
 * @param val parameter value to set
 * @return 0 - OK, -ERR_FRAMEPARS_BADINDEX
 */
int setFramePar(struct framepars_t * this_framepars, unsigned long mindex, unsigned long val) {
  int frame8=  (this_framepars->pars[P_FRAME]) & PARS_FRAMES_MASK;
  unsigned long flags;
  int nframe;
  unsigned long bmask, bmask32 , bindex;
  int findex_this=  thisFrameNumber & PARS_FRAMES_MASK;
  int findex_prev= (findex_this-1)  & PARS_FRAMES_MASK;
  int findex_future=(findex_this-2)  & PARS_FRAMES_MASK;
  int index=    mindex & 0xffff;
  MDF8(printk (": thisFrameNumber=0x%lx frame8=%d index= %d (0x%lx), val=0x%lx\n", thisFrameNumber, frame8, index, mindex, val));
  D1I(local_irq_save(flags));
//  if (index > P_MAX_PAR) {
  if (index> ((index >= FRAMEPAR_GLOBALS)? (P_MAX_GPAR+FRAMEPAR_GLOBALS): P_MAX_PAR)) {
    D1I(local_irq_restore(flags));
    return -ERR_FRAMEPARS_BADINDEX;
  }
//TODO: optimize to use mask several parameters together
  if (index >= FRAMEPAR_GLOBALS) { /// ignore frame logic, set "static" parameters to frame 0
    if (mindex & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
      val= FRAMEPAIR_FRAME_MASK_NEW(mindex, GLOBALPARS(index), val);
    }
    GLOBALPARS(index)=val;
  } else if (mindex  & FRAMEPAIR_FRAME_FUNC) { /// write to func_proc[] instead
    funcs2call[index] = val;
//  } else {

  } else if ((frame8 != findex_future) || ((mindex & FRAMEPAIR_JUST_THIS)==0)) { /// do not write "JUST_THIS" parameters in the future otherwise they'll stick
    if (mindex & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
      val= FRAMEPAIR_FRAME_MASK_NEW(mindex, framepars[frame8].pars[index], val);
    }
    if ((framepars[frame8].pars[index]!= val) || (mindex & (FRAMEPAIR_FORCE_NEW | FRAMEPAIR_FORCE_PROC))){
      bmask=   1 << (index & 31);
      bindex = index >> 5;
      bmask32= 1 << bindex;
///   Set this parameter for specified frame, (for now - unconditionally mark as modified, even if the value is the same as it was - CHANGED!
      framepars[frame8].pars[index]       = val;
      framepars[frame8].mod[bindex]      |= bmask;
      framepars[frame8].mod32            |= bmask32;
      if (mindex & FRAMEPAIR_FORCE_PROC){
        framepars[frame8].functions        |= funcs2call[index]; ///Mark which functions will be needed to process the parameters 
      }
      MDF8(printk(" bindex=0x%lx, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",bindex, bmask, bmask32, framepars[frame8].functions)); 
/// Write parameter to the next frames up to the one that have the same parameter already modified
      if ((mindex & FRAMEPAIR_JUST_THIS)==0) {
        MDF8(printk (":        ---   setting next frames"));
//        for (nframe=(frame8+1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[frame8].mod[bindex] & bmask)); nframe=(nframe+1) & PARS_FRAMES_MASK) {
        for (nframe=(frame8+1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe=(nframe+1) & PARS_FRAMES_MASK) {
          framepars[nframe].pars[index] = val;
          D8(printk (" %d",nframe));
        }
        frame8=(frame8-1) & PARS_FRAMES_MASK;  /// for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
      }
//      MDF1(printk("\n"));
/// Mark this parameter in all previous frames as "modified since"
/// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
      MDF8(printk (":        >>>   setting modsince"));
//    for (nframe=(frame8-1) & PARS_FRAMES_MASK; nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) {
      for (nframe=frame8;                        nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) { ///NOTE: frame8 is modified here
        framepars[nframe].modsince[bindex] |= bmask;
        framepars[nframe].modsince32       |= bmask32;
        D8(printk (" %d",nframe));
      }
      D8(printk ("\n"));
    }
  } else { /// error - trying to write "just this" to the "future" - that would stick if allowed
    D1I(local_irq_restore(flags));
    ELP_KERR(printk("Tried to write JUST_THIS parameter (0x%lx) too far in the future", mindex));
    return -ERR_FRAMEPARS_TOOEARLY;
  }
  D1I(local_irq_restore(flags));
  return 0;
}
/**
 * @brief set multiple output (calculated) parameters for the frame referenced by this_framepars structure.
 * Shedules action only if the FRAMEPAIR_FORCE_PROC modifier bit is set in the particular parameter index
 * @param this_framepars pointer to the current parameters structure
 * @param numPars number of parameters to set
 * @param pars  array of parameters (number/value pairs). Parameter numbers accept modifiers
 * @return  0 - OK, -ERR_FRAMEPARS_BADINDEX
 */
int setFramePars(struct framepars_t * this_framepars, int numPars, struct frameparspair_t * pars) {
  int frame8;
  unsigned long flags;
  int npar,nframe;
  unsigned long val, bmask, bmask32;
  int index,bindex;
  int findex_this=  thisFrameNumber & PARS_FRAMES_MASK;
  int findex_prev= (findex_this-1)  & PARS_FRAMES_MASK;
  int findex_future=(findex_this-2)  & PARS_FRAMES_MASK;
  MDF8(printk (": this_framepars=0x%x numPars=%d\n",(int) this_framepars, numPars));
  D1I(local_irq_save(flags));
  for (npar=0; npar < numPars; npar++) {
    frame8= (this_framepars->pars[P_FRAME]) & PARS_FRAMES_MASK;
    val=pars[npar].val;
    index= pars[npar].num & 0xffff;
    MDF8(printk (":    ---   frame8=%d index=%d (0x%x) val=0x%x\n", frame8, index, (int) pars[npar].num, (int) val));
    if (index> ((index >= FRAMEPAR_GLOBALS)? (P_MAX_GPAR+FRAMEPAR_GLOBALS): P_MAX_PAR)) {
      D1I(local_irq_restore(flags));
      ELP_KERR(printk(" bad index=%d > %d\n", index, P_MAX_PAR));
      return -ERR_FRAMEPARS_BADINDEX;
    }
    if (index >= FRAMEPAR_GLOBALS) { /// ignore frame logic, set "static" parameters to frame 0
       if (pars[npar].num & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
         val= FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, GLOBALPARS(index), val);
       }
       GLOBALPARS(index)=val;
    } else if (pars[npar].num  & FRAMEPAIR_FRAME_FUNC) {
        funcs2call[index] = val;
//    } else {
    } else if ((frame8 != findex_future) || ((pars[npar].num & FRAMEPAIR_JUST_THIS)==0)) { /// do not write "JUST_THIS" parameters in the future otherwise they'll stick
       if (pars[npar].num & FRAMEPAIR_MASK_BYTES) { /// combine new value with the old one
         val= FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, framepars[frame8].pars[index], val);
       }
//TODO: optimize to use mask several parameters together
      if ((framepars[frame8].pars[index]!= val) || (pars[npar].num & (FRAMEPAIR_FORCE_NEW | FRAMEPAIR_FORCE_PROC))){
        bmask=   1 << (index & 31);
        bindex = index >> 5;
        bmask32= 1 << bindex;
///   Set this parameter for specified frame, (for now - unconditionally mark as modified, even if the value is the same as it was - CHANGED!
        framepars[frame8].pars[index]       = val;
        framepars[frame8].mod[bindex]      |= bmask;
        framepars[frame8].mod32            |= bmask32;
        if (pars[npar].num & FRAMEPAIR_FORCE_PROC){
          framepars[frame8].functions        |= funcs2call[index]; ///Mark which functions will be needed to process the parameters 
        }
/// Write parameter to the next frames up to the one that have the same parameter already modified (only if not FRAMEPAIR_JUST_THIS)
        if ((pars[npar].num & FRAMEPAIR_JUST_THIS)==0) {
        MDF8(printk (":        ---   setting next frames"));
          for (nframe=(frame8+1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe=(nframe+1) & PARS_FRAMES_MASK) {
            D8(printk (" %d",nframe));
            framepars[nframe].pars[index] = val;
          }
          frame8=(frame8-1) & PARS_FRAMES_MASK; /// for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
          D8(printk ("\n"));
        }
/// Mark this parameter in all previous frames as "modified since"
/// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
//      for (nframe=(frame8-1) & PARS_FRAMES_MASK; nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) {
        for (nframe=frame8;                        nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) { ///NOTE: frame8 is modified here
          framepars[nframe].modsince[bindex] |= bmask;
          framepars[nframe].modsince32       |= bmask32;
        }
      }
    } else { /// error - trying to write "just this" to the "future" - that would stick if allowed
      D1I(local_irq_restore(flags));
      ELP_KERR(printk("Tried to write JUST_THIS parameter (0x%lx) too far in the future", pars[npar].num));
      return -ERR_FRAMEPARS_TOOEARLY;
    }
  }
  D1I(local_irq_restore(flags));
  return 0;
}

///TODO: make some parameters readonly (prohibited from modification from the userland)
/// File operations:
/// open, release - nop
/// read - none
/// write -> setFrameParsAtomic (first 4 bytes - absolute frame number, next 4 bytes - latency, then each 8 bytes - index/value) 
/// can use current file pointer or special indexes (0x****ff01 - set frame number, 0x****ff02 - set latency) that should come before actual parameters
/// file pointer - absolute frame number
/// lseek (SEEK_SET, value) - set absolute frame number
/// lseek (SEEK_CUR, value) - set frame number relative to the current frame number (thisFrameNumber),
/// lseek (SEEK_CUR, 0)     - (used by ftell()) also modifies file pointer - set it to thisFrameNumber,
/// lseek (SEEK_END, value <= 0) - do nothing?, do not modify file pointer
/// lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer (and actually use it - frame number the command applies to)
/// mmap (should be used read only)

static struct file_operations framepars_fops = {
   owner:    THIS_MODULE,
   llseek:   framepars_lseek,
   write:    framepars_write,
   open:     framepars_open,
   mmap:     framepars_mmap,
   release:  framepars_release
};

/**
 * @brief Driver OPEN method
 * @param inode  inode
 * @param filp   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int framepars_open(struct inode *inode, struct file *filp) {
  int res;
  struct framepars_pd * privData;
  privData= (struct framepars_pd *) kmalloc(sizeof(struct framepars_pd),GFP_KERNEL);
  if (!privData) return -ENOMEM;
  filp->private_data = privData;
  privData-> minor=MINOR(inode->i_rdev);
  MDF1(printk(": minor=0x%x\n",privData-> minor));
  switch (privData-> minor) {
    case  CMOSCAM_MINOR_FRAMEPARS :
        inode->i_size = 0; //or return 8 - number of frame pages?
        return 0;
    default:
       kfree(filp->private_data); // already allocated
       return -EINVAL;
  }
  return res;
}

/**
 * @brief Driver RELEASE method
 * @param inode  inode
 * @param filp   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int framepars_release(struct inode *inode, struct file *filp) {
  int res=0;
  int p = MINOR(inode->i_rdev);
  MDF1(printk(": minor=0x%x\n",p));
  switch ( p ) {
    case  CMOSCAM_MINOR_FRAMEPARS :
        break;
    default:
        return -EINVAL; //! do not need to free anything - "wrong number"
    }
  kfree(filp->private_data);
  return res;
}

/**
 * @brief Driver LSEEK  method (and execute commands)
 * - lseek (SEEK_SET, value) - set absolute frame number
 * - lseek (SEEK_CUR, value) - set frame number relative to the current frame number (thisFrameNumber),
 * - lseek (SEEK_CUR, 0)     - (used by ftell()) DOES NOT modify file pointer, returns thisFrameNumber,
 * - lseek (SEEK_END, value <= 0) - do nothing?, do not modify file pointer
 * - lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer (and actually use it - frame number the command applies to)
 * - no commands yet
 * @param file 
 * @param offset 
 * @param orig SEEK_SET, SEEK_CUR or SEEK_SET END
 * @return file position (absolute frame number)
 */
loff_t framepars_lseek (struct file * file, loff_t offset, int orig) {
 unsigned long target_frame;
 MDF1(printk(" offset=0x%x, orig=0x%x\n",(int) offset, (int) orig));
   switch(orig) {
   case SEEK_SET:
     file->f_pos = offset;
     break;
   case SEEK_CUR:
     if (offset==0) return getThisFrameNumber(); /// do not modify frame number
     file->f_pos = getThisFrameNumber() + offset; /// modifies frame number, but it is better to use absolute SEEK SET
     break;
   case SEEK_END:
     if (offset <= 0) {
        break;
     } else if (offset >= LSEEK_FRAME_WAIT_REL) {
       if (offset >=LSEEK_FRAME_WAIT_ABS) target_frame=offset-LSEEK_FRAME_WAIT_ABS;                     /// Wait for absolute frame number
       else                               target_frame=getThisFrameNumber()+offset-LSEEK_FRAME_WAIT_REL; /// Skip 0..255 frames
       wait_event_interruptible (framepars_wait_queue,getThisFrameNumber()>=target_frame);
//       if (getThisFrameNumber()<target_frame) wait_event_interruptible (framepars_wait_queue,getThisFrameNumber()>=target_frame);
       return getThisFrameNumber(); /// Does not modify current frame pointer? lseek (,0,SEEK_CUR) anyway returns getThisFrameNumber()
     } else { //! Other lseek commands
        switch (offset & ~0x1f) {
          case  LSEEK_DAEMON_FRAME:  /// wait the daemon enabled and a new frame interrupt (sensor frame sync)
            wait_event_interruptible (framepars_wait_queue, get_imageParamsThis(P_DAEMON_EN) & (1<<(offset & 0x1f)));
            break;
          default:
            switch (offset) {
              case LSEEK_GET_FPGA_TIME:
                //X313_GET_FPGA_TIME( GLOBALPARS(G_SECONDS), GLOBALPARS(G_MICROSECONDS) );
                MDF2(printk("X313_GET_FPGA_TIME\n"));
                break;
              case LSEEK_SET_FPGA_TIME:    /// better to use write, not lseek to set FPGA time
                //X313_SET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
                MDF2(printk("X313_SET_FPGA_TIME\n"));
                break;
              case LSEEK_FRAMEPARS_INIT:   /// reset hardware sequencers, init framepars structure
                MDF2(printk("LSEEK_FRAMEPARS_INIT\n"));
                initGlobalPars();/// reset all global parameters but the first 32
                initSequencers();
                break;
              case LSEEK_FRAME_RESET:     /// reset absoulte frame number to avoid integer frame number overflow
                MDF2(printk("LSEEK_FRAME_RESET\n"));
                resetFrameNumber();
                break;
              case LSEEK_SENSORPROC:      /// process modified parameters in frame 0 (to start sensor detection)
                MDF2(printk("LSEEK_SENSORPROC: framepars[0].functions=0x%08lx\n",framepars[0].functions));
                processPars (sensorproc, 0, 8);  ///frame0, all 8 frames (maxAhead==8)
                break;
              case LSEEK_DMA_INIT:         /// initialize ETRAX DMA (normally done in sensor_common.c at driver init
                MDF2(printk("LSEEK_DMA_INIT\n"));
                //x313_dma_init();
                break;
              case LSEEK_DMA_STOP:         /// stop DMA
                MDF2(printk("LSEEK_DMA_STOP\n"));
                //x313_dma_stop();           ///
                break;
              case LSEEK_DMA_START:        /// start DMA
                MDF2(printk("LSEEK_DMA_START\n"));
                //x313_dma_start();          ///
                break;
              case LSEEK_COMPRESSOR_RESET: /// reset compressor and buffer pointers 
                MDF2(printk("LSEEK_COMPRESSOR_RESET\n"));
                reset_compressor();
                break;
              case LSEEK_INTERRUPT_OFF:    /// disable camera interrupts
                MDF2(printk ("LSEEK_INTERRUPT_OFF\n"));
                camera_interrupts (0);
                break;
              case LSEEK_INTERRUPT_ON:     /// enable camera interrupts
                MDF2(printk ("LSEEK_INTERRUPT_ON\n"));
                camera_interrupts (1);
                break;
            }
        }
        break;
     }
     break;
   default:
      return -EINVAL;
   }
   return  file->f_pos ;
}

/**
 * @brief Driver WRITE method
 * writes all at once, no provisions to continue in the next call
 * @param file 
 * @param buf 
 * @param count 
 * @param off 
 * @return OK - number of bytes written, negative - errors
 */
ssize_t framepars_write(struct file * file, const char * buf, size_t count, loff_t *off) {
   struct frameparspair_t   pars_static[256];/// will be sufficient for most calls
   struct frameparspair_t * pars = pars_static;
   struct framepars_pd * privData = (struct framepars_pd *) file->private_data;
   unsigned long frame=*off; /// ************* NOTE: Never use file->f_pos in write() and read() !!!
   int           latency=-1;
   int           first=0;
   int           last;
   int           result;
   MDF1(printk(": file->f_pos=0x%x, *off=0x%x, count=0x%x\n", (int) file->f_pos, (int) *off, (int) count));
   count &= ~7; /// sizeof (struct frameparspair_t)==8
   switch (privData->minor) {
       case CMOSCAM_MINOR_FRAMEPARS :
         if (count>sizeof(pars_static)) /// only allocate if static is not enough
            pars =     (struct frameparspair_t *) kmalloc(count, GFP_KERNEL);
         if (!pars) return -ENOMEM;
         count >>=3; /// divide by sizeof(struct frameparspair_t); // 8
         if(count) {
            if(copy_from_user((char *) pars, buf, count<<3)) {
              if (count>sizeof(pars_static)) kfree(pars);
              return -EFAULT;
           }
           while (first <count) {
             while ((first <count) && ((pars[first].num & 0xff00)==0xff00)) { // process special instructions
               switch (pars[first].num & 0xffff) {
                 case FRAMEPARS_SETFRAME:
                    frame= pars[first].val;
                    break;
                 case FRAMEPARS_SETFRAMEREL:
                    frame= pars[first].val+getThisFrameNumber();
                    break;
                 case FRAMEPARS_SETLATENCY:
                    latency= (pars[first].val & 0x80000000)? -1: pars[first].val;
                    break;
                 case FRAMEPARS_SETFPGATIME:  ///ignore value (should be already set in G_SECONDS, G_MICROSECONDS frame0)
                    //X313_SET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
                    break;
                 case FRAMEPARS_GETFPGATIME:  ///ignore value, set frame 0 G_SECONDS, G_MICROSECONDS to FPGA registers
                    //X313_GET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
                    break;
                 default:
                 printk("framepars_write: invalid special instruction: 0x%x\n", (int) pars[first].num);
               }
               first ++;
             }
             last=first+1;
             while ((last < count) && ((pars[last].num & 0xff00)!=0xff00)) last++; // skip to the end or next special instructions
             result=setFrameParsAtomic(frame, latency, last-first, &pars[first]);
             MDF1(printk("setFrameParsAtomic(%ld, %d, %d)\n", frame, latency, last-first));
             if (result <0) { 
               if (count>sizeof(pars_static)) kfree(pars);
               return -EFAULT;
             }
             first=last;
           }
         }
         if (count>sizeof(pars_static)) kfree(pars);
         return count<<3; /// *sizeof(struct frameparspair_t);
       default: return -EINVAL;
  }
}

/**
 * @brief Driver MMAP method (should be used read only)
 * provides access to both 8-frame parameters (including future ones), frame - 0 - some static ones too and
 * much longer retained pastparameters - subset of all parameters
 * @param file 
 * @param vma 
 * @return OK - 0, negative - errors
 */
int framepars_mmap (struct file *file, struct vm_area_struct *vma) {
  int result;
  struct framepars_pd * privData = (struct framepars_pd *) file->private_data;
  MDF1(printk(": minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case  CMOSCAM_MINOR_FRAMEPARS :
           result=remap_pfn_range(vma,
                    vma->vm_start,
                   ((unsigned long) virt_to_phys(frameparsall)) >> PAGE_SHIFT, // Should be page-aligned
                    vma->vm_end-vma->vm_start,
                    vma->vm_page_prot);
           MDF1(printk("remap_pfn_range returned=%x\r\n",result));
           if (result) return -EAGAIN;
           return 0;
       default: return -EINVAL;
  }
}

/**
 * @brief framepars driver probing function
 * @param[in]   pdev   pointer to \b platform_device structure
 * @return      0 on success or negative error code otherwise
 */
static int framepars_init(struct platform_device *pdev)
{
   int res;
   struct device *dev = &pdev->dev;
   const struct of_device_id *match;

   /* sanity check */
   match = of_match_device(elphel393_framepars_of_match, dev);
   if (!match)
	   return -EINVAL;

   init_framepars_ptr();
   initGlobalPars(); /// sets default debug if enabled - not anymore. Add here?
   initMultiPars(); /// just clear - needs to be called again when sensor is recognized
   frameParsInitialized=0;

   res = register_chrdev(FRAMEPARS_MAJOR, "framepars_operations", &framepars_fops);
   if(res < 0) {
     printk(KERN_ERR "\nframepars_init: couldn't get a major number %d.\n",FRAMEPARS_MAJOR);
     return res;
   }
   init_waitqueue_head(&framepars_wait_queue);
   dev_info(dev, "registered MAJOR: %d\n", FRAMEPARS_MAJOR);

   return 0;
}

static int framepars_remove(struct platform_device *pdev)
{
	unregister_chrdev(FRAMEPARS_MAJOR, "framepars_operations");

	return 0;
}

static const struct of_device_id elphel393_framepars_of_match[] = {
		{ .compatible = "elphel,elphel393-framepars-1.00" },
		{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_framepars_of_match);

static struct platform_driver elphel393_framepars = {
		.probe          = framepars_init,
		.remove         = framepars_remove,
		.driver = {
				.name = FRAMEPARS_DRIVER_NAME,
				.of_match_table = elphel393_framepars_of_match,
		},
};

module_platform_driver(elphel393_framepars);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_FRAMEPARS_DRIVER_NAME);
