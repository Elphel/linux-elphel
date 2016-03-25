/*!********************************************************************************
*! FILE NAME  : sensor_common.c
*! DESCRIPTION: Sensor discovery, initialization, programming - common
*! for all sensors
*! Now most is moved to other files, here will be
*! - system initialization
*! - compressor write (and global read) pointers
*! - populating 32-byte interframe data
*! - something for Exif?
*! - where are histograms? ->histograms.c
*! - interrupt loop
*! - tasklets
*! - device driver that includes waiting for the next frame regardless of compression
*! - anything else?
*!
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
*!  $Log: sensor_common.c,v $
*!  Revision 1.11  2012/04/08 04:09:23  elphel
*!  added temperatures to MakerNote
*!
*!  Revision 1.10  2010/08/10 21:14:31  elphel
*!  8.0.8.39 - added EEPROM support for multiplexor and sensor boards, so autocampars.php uses application-specific defaults. Exif Orientation tag support, camera Model reflects application and optional mode (i.e. camera number in Eyesis)
*!
*!  Revision 1.9  2010/08/03 23:37:34  elphel
*!  rev 8.0.8.37, portrait mode support
*!
*!  Revision 1.8  2010/08/01 19:29:24  elphel
*!  files updated to support coring function for noise filtering
*!
*!  Revision 1.7  2010/07/20 20:13:34  elphel
*!  8.0.8.33 - added MakerNote info for composite images made with multisensor cameras (with 10359 board)
*!
*!  Revision 1.6  2010/05/16 02:03:47  elphel
*!  8.0.8.4 - driver working with individual/broadcast sensor registers
*!
*!  Revision 1.5  2010/05/13 03:39:31  elphel
*!  8.0.8.12 -  drivers modified for multi-sensor operation
*!
*!  Revision 1.4  2010/03/04 06:41:40  elphel
*!  8.0.7.3 - more data to makerNote
*!
*!  Revision 1.3  2009/12/28 06:24:17  elphel
*!  8.0.6.6 - added MakerNote to Exif, it icludes channels gains and gammas/black levels
*!
*!  Revision 1.2  2008/11/30 21:56:39  elphel
*!  Added enforcing limit on the overall gains in the color channels, storage of exposure and gains in the histograms cache (to be used with autoexposure/white balance)
*!
*!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
*!
*!
*!  Revision 1.33  2008/11/20 07:04:47  elphel
*!  made silent when debug is 0
*!
*!  Revision 1.32  2008/11/14 07:06:54  elphel
*!  fixed wrong timing for servicing histogram requests
*!
*!  Revision 1.31  2008/11/13 05:40:45  elphel
*!  8.0.alpha16 - modified histogram storage, profiling
*!
*!  Revision 1.30  2008/11/10 19:47:46  elphel
*!  added TODO abut reducing CPU load by decimating histogram calculations (not each frame)
*!
*!  Revision 1.29  2008/10/29 04:18:28  elphel
*!  v.8.0.alpha10 made a separate structure for global parameters (not related to particular frames in a frame queue)
*!
*!  Revision 1.28  2008/10/26 05:54:45  elphel
*!  changed value of frame counters for histograms to compensate for 1 frame latency of histograms
*!
*!  Revision 1.27  2008/10/25 19:55:00  elphel
*!  made the histograms calculations relate to previous, not this frame.
*!
*!  Revision 1.26  2008/10/23 08:09:02  elphel
*!  support for histogram wait queues
*!
*!  Revision 1.25  2008/10/19 06:56:05  elphel
*!  elphel_wait_frame() now works only for compressed frames, new elphel_skip_frames() and elphel_wait_frame_abs() wait sequencer frames (all sensor frames, even those that are not compressed)
*!
*!  Revision 1.24  2008/10/15 22:28:56  elphel
*!  snapshot 8.0.alpha2
*!
*!  Revision 1.23  2008/10/13 16:55:53  elphel
*!  removed (some) obsolete P_* parameters, renamed CIRCLSEEK to LSEEK_CIRC constants (same as other similar)
*!
*!  Revision 1.22  2008/10/12 16:46:22  elphel
*!  snapshot
*!
*!  Revision 1.21  2008/10/11 18:46:07  elphel
*!  snapshot
*!
*!  Revision 1.20  2008/10/10 17:06:59  elphel
*!  just a snapshot
*!
*!  Revision 1.19  2008/10/08 21:26:25  elphel
*!  snapsot 7.2.0.pre4 - first images (actually - second)
*!
*!  Revision 1.18  2008/10/05 05:13:33  elphel
*!  snapshot003
*!
*!  Revision 1.17  2008/10/04 16:10:12  elphel
*!  snapshot
*!
*!  Revision 1.16  2008/09/28 00:31:57  elphel
*!  snapshot
*!
*!  Revision 1.15  2008/09/25 00:58:12  elphel
*!  snapshot
*!
*!  Revision 1.14  2008/09/22 22:55:49  elphel
*!  snapshot
*!
*!  Revision 1.13  2008/09/20 00:29:50  elphel
*!  moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
*!
*!  Revision 1.12  2008/09/19 04:37:26  elphel
*!  snapshot
*!
*!  Revision 1.11  2008/09/16 00:49:32  elphel
*!  snapshot
*!
*!  Revision 1.10  2008/09/12 00:24:00  elphel
*!  removed cc353.c, cc353.h
*!
*!  Revision 1.9  2008/09/07 19:48:09  elphel
*!  snapshot
*!
*!  Revision 1.8  2008/09/05 23:20:26  elphel
*!  just a snapshot
*!
*!  Revision 1.7  2008/09/02 21:01:07  elphel
*!  just next...
*!
*!  Revision 1.6  2008/07/29 01:15:07  elphel
*!  another snapshot
*!
*!  Revision 1.5  2008/07/27 23:25:07  elphel
*!  next snapshot
*!
*!  Revision 1.4  2008/07/27 04:27:49  elphel
*!  next snapshot
*!
*!  Revision 1.3  2008/06/24 00:43:44  elphel
*!  just a snapshot
*!
*!  Revision 1.2  2008/06/20 03:54:21  elphel
*!  another snapshot
*!
*!  Revision 1.1  2008/06/16 06:51:21  elphel
*!  work in progress, intermediate commit
*!
*!
*/

//copied from cxi2c.c - TODO:remove unneeded

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/vmalloc.h>

//#include <asm/system.h>
#include <asm/byteorder.h> // endians
#include <asm/io.h>

//#include <asm/arch/hwregs/intr_vect_defs.h> /// ETRAX interrupt registers

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
//#include <asm/elphel/fpgaconfa.h>
#include <elphel/exifa.h>
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "fpga_sdram.h" // use a single fpga_initSDRAM(void)

//#include "x3x3.h"
//#include "cc3x3.h"
//#include "cxdma.h"

#include "framepars.h"
#include "sensor_common.h"
//#include "pgm_functions.h"
#include "circbuf.h"
//#include "exif353.h"
//#include "histograms.h"
//#include "gamma_tables.h"
#include "quantization_tables.h"

#if ELPHEL_DEBUG
 #define ELPHEL_DEBUG_THIS 0
#else
 #define ELPHEL_DEBUG_THIS 0
#endif
#if ELPHEL_DEBUG_THIS
  #define MDF2(x) printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__); x
  #define MDD1(x) printk("%s:%d:",__FILE__,__LINE__); x ; udelay (ELPHEL_DEBUG_DELAY)
  #define MD1(x) printk("%s:%d:",__FILE__,__LINE__);x
  //#define MD1(x)
  #define MD12(x) printk("%s:%d:",__FILE__,__LINE__);x
  //#define MD12(x)
  #define MD13(x) printk("%s:%d:",__FILE__,__LINE__);x
  //#define MD13(x)
#else
  #define MDF2(x)
  #define MD1(x)
  #define MDD1(x)
  #define MD12(x)
  #define MD13(x)
#endif



#define X3X3_ELPHEL_DRIVER_NAME "Elphel (R) Model 353 Camera Driver"
static const char elphel_cam_name[] = "elphelcam353";

static volatile int JPEG_wp;
static volatile int JPEG_rp;
static int fpga_counter_prev=0; /// Previous value of the FPGA transfer counter (to find out if it did change)
  static struct meta_offsets_t { // works like a cache to time save on looking for tags in the directory (forced to recalcualte if does not match)
    int Image_DateTime;          // will have offset of the Exif_Image_DateTime data in meta page (Exif_Photo_SubSecTime should go immediately after in meta page)
    int Photo_DateTimeOriginal;
    int Photo_ExposureTime;
    int Image_FrameNumber;
    int Image_Orientation;
    int Photo_MakerNote;
  } meta_offsets;





///Defines/macros moved from the cc353.h
// #define EN_INTERRUPT(x)  port_csp0_addr[X313_WA_IRQ_ENA]= (1<< x )
// #define DIS_INTERRUPT(x) {port_csp0_addr[X313_WA_IRQ_DIS]= (1<< x ); port_csp0_addr[X313_WA_IRQ_RST]= (1<< x );}
// #define DIS_INTERRUPTS   {port_csp0_addr[X313_WA_IRQ_DIS]= 0xffff; port_csp0_addr[X313_WA_IRQ_RST]= 0xffff;}



int  camSeqGetJPEG_wp(void) {return JPEG_wp;}
int  camSeqGetJPEG_rp(void) {return JPEG_rp;}
void camSeqSetJPEG_rp(int p) {
                              JPEG_rp=p;
                              set_globalParam(G_CIRCBUFRP,  p<< 2);
                              set_globalParam(G_FREECIRCBUF,
                                    (((get_globalParam(G_CIRCBUFRP) <= get_globalParam(G_CIRCBUFWP))?
                                       get_globalParam(G_CIRCBUFSIZE):0)+ get_globalParam(G_CIRCBUFRP))
                                       - get_globalParam(G_CIRCBUFWP));
                               }
//EXPORT_SYMBOL_GPL(camSeqGetJPEG_wp);
//EXPORT_SYMBOL_GPL(camSeqGetJPEG_rp);
//EXPORT_SYMBOL_GPL(camSeqSetJPEG_rp);

/*!
   End of compressor-related code - TODO: move to a separate file?
*/


#define X3X3_IMAGEACQ_DRIVER_NAME "Elphel (R) Model 353 Image Acquisition device driver"
static struct sensorproc_t s_sensorproc; // sensor parameters and functions to call
struct sensorproc_t * sensorproc = NULL;
//EXPORT_SYMBOL_GPL(sensorproc);
//wait_queue_head_t image_acq_wait_queue;  /// queue for the sensor frame interrupts


struct image_acq_pd {
    int                minor;
//    struct wait_queue *image_acq_wait_queue;
// something else to be added here?
};

void tasklet_fpga_function(unsigned long arg);

/**
 * @brief Copy sensorproc structure, needed for multisensor board to be able to replace some of the functions
 * @param copy - pointer to a copy structure
 * @return pointer to a copy structure
 */

struct sensorproc_t * copy_sensorproc (struct sensorproc_t * copy) {
  memcpy(copy, sensorproc, sizeof(struct sensorproc_t)); /// copy sensor functions
  return copy;
}

#ifdef TEST_DISABLE_CODE

/// When should it be called?
//int init_sensor(void);
/// Not yet used??
/**
 * @brief Check FPGA version and initialize SDRAM (if not done yet)
 * TODO: when should it be called?
 * @return <0 error, 0 - just checked, nothing done,1 - needs sensor initialization 
 */
int init_FPGA(void) { //will check FPGA version, init SDRAM (if needed) and sensor
  int i;
//  int f1,f2;
   // Should be initial
  if ((fpga_state & FPGA_STATE_LOADED) ==  0) return -1; /// fpga is not configured

  if ((i=port_csp0_addr[X313__RA__MODEL]) < X313_MINMODREV) {
   printk ("too old fpga rev - found %x, software wants >= %x\n",i,X313_MINMODREV);
   return -1;   // too old FPGA
  } 
  if (i > X313_MAXMODREV) {
     printk ("too new fpga rev - found %x, software wants <= %x\n",i,X313_MAXMODREV);
     return -1;    // too new FPGA
  }
  fpga_state |= FPGA_STATE_INITIALIZED; /// what this initialization really mean?
  
  if (!X313_IS_SDRAM_ON)  fpga_initSDRAM();
// Was sensor initialized? (What if SDRAM was initialized by some application?)
  MD1(printk("init_FPGA, fpga_state=0x%x\n",fpga_state));
  if (X313_CHN0_USED!=0) return 0;
  return 1;
}

///
/// initializes structures for the image acquisition parameter
/// initializes hardware i2c controller and the command sequencer (leaves them stopped to ignore any frame syncs)
/// sets some default acquisition parameters
/// Maybe - set up DMA also?
/// TODO: Take care while turning off reset on i2c and cmd_sequencer so there will be no sensor vsync lost
/// (easier to do in FPGA)
/// Done:
///#define CCAM_VSYNC_ON  port_csp0_addr[X313_WA_DCR1]=X353_DCR1(BLOCKVSYNC,0)
///#define CCAM_VSYNC_OFF port_csp0_addr[X313_WA_DCR1]=X353_DCR1(BLOCKVSYNC,1)
///
int init_acq_sensor(void);

static int __init image_acq_init(void);

DECLARE_TASKLET(tasklet_fpga, tasklet_fpga_function, 0); /// 0 - no arguments for now



/**
 * @brief reads FPGA transfer pointer to update JPEG_wp
 * NOTE: should be called before compressor is reset - that would zero out that hardware register
 * @return 0 if compressor was off (no advance), 1 if write pointer did actually advance
 */
inline int updateIRQJPEG_wp(void) {
     int xferred;                                           /// number of 32-byte chunks transferred since compressor was reset
     int fpga_cntr=            X313_XFERCNTR;               /// using macro defined in x353.h that makes a dummy read (reads after writes can be wrong)
     xferred=                  fpga_cntr-fpga_counter_prev; /// Transferred since last JPEG_wp update (or counter reset)
#if 0 /// ELPHEL_DEBUG_THIS- address changed !!!
     set_globalParam          (0x300,get_globalParam (0x300)+1);
     set_globalParam          (0x302,fpga_cntr);
     if (xferred==0)           set_globalParam          (0x305,get_globalParam (0x305)+1);
#endif
     if (xferred==0)           return 0;                    /// no advance (compressor was off?)
     fpga_counter_prev=        fpga_cntr;
     if (xferred <0) xferred+= (1 <<24) ;                   /// Hardware counter is 24 bits - rolled over
     JPEG_wp+= (xferred << 3); //! counts in 32-byte ( 8 of 32bit words) chunks
     int circbuf_size=get_globalParam (G_CIRCBUFSIZE)>>2; //G_CIRCBUFSIZE G_CIRCBUFRP
     if (JPEG_wp > circbuf_size) JPEG_wp-=circbuf_size;
#if 0 ///ELPHEL_DEBUG_THIS - address changed !!!
     set_globalParam          (0x301,get_globalParam (0x301)+1);
     set_globalParam          (0x303,xferred);
     set_globalParam          (0x304,JPEG_wp);
#endif
     return 1;
}
/**
 * @brief Calculate/update CIRCBUF parameters available after compressor interrupt
 */
inline void updateIRQCircbuf(void) {
     set_globalParam (G_CIRCBUFWP, JPEG_wp<<2);
     set_globalParam (G_FREECIRCBUF, (((get_globalParam (G_CIRCBUFRP) <= get_globalParam (G_CIRCBUFWP))? get_globalParam (G_CIRCBUFSIZE):0)+ 
                                     get_globalParam (G_CIRCBUFRP)) -    get_globalParam (G_CIRCBUFWP));
}

/**
 * @brief Calculate/update focus parameters available after compressor interrupt
 * NOTE: currently global (latest), not per-frame
 */
inline void updateIRQFocus(void) {
    set_globalParam     (G_GFOCUS_VALUE, X313_HIGHFREQ);
    set_imageParamsThis (P_FOCUS_VALUE, X313_HIGHFREQ);
}



/**
 * @brief Locate area between frames in the circular buffer
 * @return pointer to interframe parameters structure
 */
inline struct interframe_params_t* updateIRQ_interframe(void) {
   int circbuf_size=get_globalParam (G_CIRCBUFSIZE)>>2;
   int alen = JPEG_wp-9; if (alen<0) alen+=circbuf_size;
   int jpeg_len=ccam_dma_buf_ptr[alen] & 0xffffff;
   set_globalParam(G_FRAME_SIZE,jpeg_len);
   int aframe_params=(alen & 0xfffffff8)-
                     (((jpeg_len + CCAM_MMAP_META + 3) & 0xffffffe0)>>2) /// multiple of 32-byte chunks to subtract
                     -8; /// size of the storage area to be filled before the frame
   if(aframe_params < 0) aframe_params += circbuf_size;
   struct interframe_params_t* interframe= (struct interframe_params_t*) &ccam_dma_buf_ptr[aframe_params];
/// should we use memcpy as before here?
   interframe->frame_length=jpeg_len;
   interframe->signffff=0xffff;
#if ELPHEL_DEBUG_THIS
    set_globalParam          (0x306,get_globalParam (0x306)+1);
#endif

   return interframe;
}

/**
 * @brief Fill exif data with the current frame data, save pointer to Exif page in the interframe area
 * @param interframe pointer to interframe parameters structure
 */
inline void updateIRQ_Exif(struct interframe_params_t* interframe) {
   int  index_time = JPEG_wp-11; if (index_time<0) index_time+=get_globalParam (G_CIRCBUFSIZE)>>2;
/// calculates datetime([20] and subsec[7], returns  pointer to char[27]
   char * exif_meta_time_string=encode_time(ccam_dma_buf_ptr[index_time], ccam_dma_buf_ptr[index_time+1]);
/// may be split in datetime/subsec - now it will not notice missing subseq field in template
   write_meta_irq(exif_meta_time_string, &meta_offsets.Photo_DateTimeOriginal,  Exif_Photo_DateTimeOriginal, 27);
   write_meta_irq(exif_meta_time_string, &meta_offsets.Image_DateTime,  Exif_Image_DateTime, 20); // may use 27 if room is provided
   putlong_meta_irq(get_imageParamsThis(P_EXPOS), &meta_offsets.Photo_ExposureTime,  Exif_Photo_ExposureTime);
   putlong_meta_irq(get_imageParamsThis(P_FRAME), &meta_offsets.Image_FrameNumber,   Exif_Image_FrameNumber);
//Exif_Photo_MakerNote
  int global_flips=(get_imageParamsThis(P_FLIPH) & 1) | ((get_imageParamsThis(P_FLIPV)<<1)  & 2);
  int extra_flips=0;
  if (get_imageParamsThis(P_MULTI_MODE)!=0) {
    extra_flips=get_imageParamsThis(P_MULTI_MODE_FLIPS);
    global_flips=extra_flips & 3;
  }
/*  unsigned char orientations[]={1,6,3,8,
                                2,7,4,5,
                                4,5,2,7,
                                3,8,1,6};
*/
  unsigned char orientations[]="1638274545273816";


  unsigned char orientation_short[2];
  orientation_short[0]=0;
  orientation_short[1]=0xf & orientations[(get_imageParamsThis(P_PORTRAIT)&3) | (global_flips<<2)];
  write_meta_irq(orientation_short, &meta_offsets.Image_Orientation,   Exif_Image_Orientation, 2);

//TODO - use memcpy
   int maker_offset;
   maker_offset=putlong_meta_irq(get_imageParamsThis(P_GAINR),   &meta_offsets.Photo_MakerNote, Exif_Photo_MakerNote);
   if (maker_offset>0) {
     putlong_meta_raw_irq(get_imageParamsThis(P_GAING),   maker_offset+4);
     putlong_meta_raw_irq(get_imageParamsThis(P_GAINGB),  maker_offset+8);
     putlong_meta_raw_irq(get_imageParamsThis(P_GAINB),   maker_offset+12);
     putlong_meta_raw_irq(get_imageParamsThis(P_GTAB_R),  maker_offset+16);
     putlong_meta_raw_irq(get_imageParamsThis(P_GTAB_G),  maker_offset+20);
     putlong_meta_raw_irq(get_imageParamsThis(P_GTAB_GB), maker_offset+24);
     putlong_meta_raw_irq(get_imageParamsThis(P_GTAB_B),  maker_offset+28);
     putlong_meta_raw_irq(get_imageParamsThis(P_WOI_LEFT) | (get_imageParamsThis(P_WOI_WIDTH)<<16),  maker_offset+32);
     putlong_meta_raw_irq(get_imageParamsThis(P_WOI_TOP) | (get_imageParamsThis(P_WOI_HEIGHT)<<16),  maker_offset+36);
     putlong_meta_raw_irq(  global_flips |
                          ((get_imageParamsThis(P_BAYER)<<2)     & 0xc) |
                          ((get_imageParamsThis(P_COLOR)<<4)     & 0xF0) |
                          ((get_imageParamsThis(P_DCM_HOR)<<8)   & 0xF00) |
                          ((get_imageParamsThis(P_DCM_VERT)<<12) & 0xF000) |
                          ((get_imageParamsThis(P_BIN_HOR)<<16)  & 0xF0000) |
                          ((get_imageParamsThis(P_BIN_VERT)<<20) & 0xF00000) |
                           (extra_flips <<24) ,  maker_offset+40);
     putlong_meta_raw_irq(get_imageParamsThis(P_MULTI_HEIGHT_BLANK1),   maker_offset+44);
     putlong_meta_raw_irq(get_imageParamsThis(P_MULTI_HEIGHT_BLANK2),   maker_offset+48);
//     putlong_meta_raw_irq(0x1234567,  maker_offset+52); // just testing
     putlong_meta_raw_irq(get_imageParamsThis(P_QUALITY) | ((get_imageParamsThis(P_PORTRAIT)&1)<<7) | (get_imageParamsThis(P_CORING_INDEX)<<16),  maker_offset+52);
     putlong_meta_raw_irq(get_globalParam(G_TEMPERATURE01),  maker_offset+56); // data should be provided by a running daemon
     putlong_meta_raw_irq(get_globalParam(G_TEMPERATURE23),  maker_offset+60);
//get_globalParam(G_TASKLET_CTL)
// left 1 long spare (+44)                          
   }
   interframe->meta_index=store_meta();
}


/**
 * @brief hardware IRQ service
 * most urgent tasks
 * @param irq 
 * @param dev_id 
 * @return 
 */
static irqreturn_t elphel_FPGA_interrupt(int irq, void *dev_id) {
   unsigned long irq_state;

   irq_state = X313_IRQSTATE; //!making dummy read - see c353.h
   DIS_INTERRUPTS;
   PROFILE_NEXT(0);
/// read hardware write pointer (will only advance if compression was on)
///find out if compressor was running and update pointers, exif, ...?
   if (updateIRQJPEG_wp()) { ///also fills P_FRAME ahead
      updateIRQCircbuf();
      updateIRQFocus(); ///NOTE: currently global (latest), not per-frame
      struct interframe_params_t* interframe= updateIRQ_interframe(); /// fills jpeg_len, signffff
/// should we use memcpy as before here?
//      interframe->frame_length=jpeg_len;
//      interframe->signffff=0xffff;
      updateIRQ_Exif(interframe);
      updateFramePars(X3X3_I2C_FRAME, interframe);
      wake_up_interruptible(&circbuf_wait_queue); /// only when frame is acquired
   } else {
      updateFramePars(X3X3_I2C_FRAME, NULL); 
   }
   PROFILE_NOW(1);
   wake_up_interruptible(&framepars_wait_queue); /// all interrupts, not just frames acquired
   tasklet_schedule(&tasklet_fpga); /// trigger software interrupt

   EN_INTERRUPT(SMART);

   return IRQ_HANDLED;
}

/**
 * @brief Tasklet - software interrupt
 * lower priority tasks
 * try to implement some balancing - if job is not finished - reduce FPS for it (alternate jobs)?
 * @param arg not used
 */

/*!TODO:

implement 2 modes of controlling when to calculate histograms:
1 - add mask to 3 frame number LSB (i.e. - 0/10000000/10001000/10101010/11111111) - 3 contol bits - en/dis and mode
2 - requested in advance (i.e. by autoexposure when writing new exposure or white balance - when writing balance

mode 1 will provide easy way to display histograms (no need to repetively request them),
mode 2 - useful for autoexposure

Modify waiting (LSEEK_*) for histogrames so they will only unfreeze if the histogram is available (skipping multiple frames))
For displaying histograms - try use latest available - not waiting fro a particular frame.


*/

/// HISTOGRAMS_WAKEUP_ALWAYS if 0 will only wakeup processes waiting for histograms when they become available, maybe never if they are disabled
/// if defined 1 - will wakeup each frame, regardless of the availability of the histograms
//#define HISTOGRAMS_WAKEUP_ALWAYS 0
void tasklet_fpga_function(unsigned long arg) {
  int hist_en;
  int tasklet_disable=get_globalParam(G_TASKLET_CTL);
  unsigned long thisFrameNumber=getThisFrameNumber();
  unsigned long prevFrameNumber=thisFrameNumber-1;
  unsigned long * hash32p=&(framepars[(thisFrameNumber-1) & PARS_FRAMES_MASK].pars[P_GTAB_R]);
  unsigned long * framep= &(framepars[(thisFrameNumber-1) & PARS_FRAMES_MASK].pars[P_FRAME]);
/// Time is out?
  if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
/// Histograms are available for the previous frame (that is already in circbuf if compressor was running)
/// Is Y histogram needed?
  PROFILE_NOW(2);
  switch ((tasklet_disable >> TASKLET_CTL_HISTY_BIT) & 7) {
     case TASKLET_HIST_NEVER:   /// never calculate
      hist_en=0;
      break;
     case TASKLET_HIST_HALF:    /// calculate each even (0,2,4,6 frme of 8)
      hist_en= ((thisFrameNumber & 1) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_Y));
      break;
     case TASKLET_HIST_QUATER:  /// calculate twice per 8 (0, 4)
      hist_en= ((thisFrameNumber & 3) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_Y));
      break;
     case TASKLET_HIST_ONCE:    /// calculate once  per 8 (0)
      hist_en= ((thisFrameNumber & 7) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_Y));
      break;
     case TASKLET_HIST_RQONLY:  /// calculate only when specifically requested
      hist_en= (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_Y));
      break;
     case TASKLET_HIST_ALL:     /// calculate each frame
     default:                   /// calculate always (safer)
       hist_en=1;
  }
  if (hist_en) {
/// after updateFramePars gammaHash are from framepars[this-1]
    set_histograms (prevFrameNumber, (1 << COLOR_Y_NUMBER), hash32p, framep); /// 0x2 Green1
    GLOBALPARS(G_HIST_Y_FRAME)=prevFrameNumber;           /// histogram corresponds to previous frame
    PROFILE_NOW(3);
/// Time is out?
    if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
#if HISTOGRAMS_WAKEUP_ALWAYS
  }
  wake_up_interruptible(&hist_y_wait_queue);    /// wait queue for the G1 histogram (used as Y)
#else
    wake_up_interruptible(&hist_y_wait_queue);    /// wait queue for the G1 histogram (used as Y)
  }
#endif
/// Process parameters 
  if ((tasklet_disable & (1 << TASKLET_CTL_PGM))   == 0) {
      processPars (sensorproc, getThisFrameNumber(), get_globalParam(G_MAXAHEAD)); /// program parameters
      PROFILE_NOW(4);
  }
/// Time is out?
  if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
/// Are C histograms needed?
  switch ((tasklet_disable >> TASKLET_CTL_HISTC_BIT) & 7) {
     case TASKLET_HIST_NEVER:   /// never calculate
       hist_en=0;
      break;
     case TASKLET_HIST_HALF:    /// calculate each even (0,2,4,6 frme of 8)
      hist_en= ((thisFrameNumber & 1) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_C));
      break;
     case TASKLET_HIST_QUATER:  /// calculate twice per 8 (0, 4)
      hist_en= ((thisFrameNumber & 3) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_C));
      break;
     case TASKLET_HIST_ONCE:    /// calculate once  per 8 (0)
      hist_en= ((thisFrameNumber & 7) ==0) || (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_C));
      break;
     case TASKLET_HIST_RQONLY:  /// calculate only when specifically requested
      hist_en= (get_imageParamsPrev(P_HISTRQ) & (1<<HISTRQ_BIT_C));
      break;
     case TASKLET_HIST_ALL:     /// calculate each frame
     default:                   /// calculate always (safer)
       hist_en=1;
  }
/*
GLOBALPARS(0x1040)=((thisFrameNumber & 1) ==0);
GLOBALPARS(0x1041)=((thisFrameNumber & 3) ==0);
GLOBALPARS(0x1042)=((thisFrameNumber & 7) ==0);
GLOBALPARS(0x1043)=hist_en;
GLOBALPARS(0x1044)=thisFrameNumber;
*/
  if (hist_en) {
/// after updateFramePars gammaHash are from framepars[this-1]
    set_histograms (prevFrameNumber, 0xf, hash32p, framep); /// all 4 colors, including Y (it will be skipped)
    GLOBALPARS(G_HIST_C_FRAME)=prevFrameNumber;           /// histogram corresponds to previous frame
    PROFILE_NOW(5);
/// Time is out?
    if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
#if HISTOGRAMS_WAKEUP_ALWAYS
  }
  wake_up_interruptible(&hist_c_wait_queue);     /// wait queue for all the other (R,G2,B) histograms (color)
#else
    wake_up_interruptible(&hist_c_wait_queue);   /// wait queue for all the other (R,G2,B) histograms (color)
  }
#endif
}

#endif /* TEST_DISABLE_CODE */

/**
 * @brief resets compressor and buffer pointers
 */
void reset_compressor(void) {
  unsigned long flags;
  local_irq_save(flags);
#ifdef TEST_DISABLE_CODE
  port_csp0_addr[X313_WA_COMP_CMD]= COMPCMD_RESET; /// bypasses command sequencer
  if (framepars) set_imageParamsR_all( P_COMPRESSOR_RUN, COMPRESSOR_RUN_STOP );
  else printk ("framepars is not initialized\n");
/// TODO: There still is a possibility, that there are compressor commands in the hardware que. Should we stop the hardware sequencer here (and restart it later)?
#endif /* TEST_DISABLE_CODE */
  JPEG_wp=0;
  JPEG_rp=0;
  //updateIRQCircbuf(); /// initialize  G_CIRCBUFWP, G_FREECIRCBUF
  fpga_counter_prev=0;
  local_irq_restore(flags);
}
//EXPORT_SYMBOL_GPL(reset_compressor);

/**
 * @brief Camera interrupts on/off  (currently both in the FPGA and in the CPU)
 * @param on 1 - enable, 0 - disable interrupts
 */
void camera_interrupts (int on) {
  MDF2(printk ("camera_interrupts(%d)\n",on));
#ifdef TEST_DISABLE_CODE
  if (on) {
    EN_INTERRUPT(SMART);
  }  else {
    DIS_INTERRUPTS;
  }
/// clear smart interrupt circuitry in any case
  port_csp0_addr[X313_WA_SMART_IRQ]=0x8000;

  reg_intr_vect_rw_mask intr_mask;
  intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);
  intr_mask.ext = on ? 1 : 0;
  REG_WR(intr_vect, regi_irq, rw_mask, intr_mask); 
#endif /* TEST_DISABLE_CODE */
}
//EXPORT_SYMBOL_GPL(camera_interrupts);


int image_acq_open(struct inode *inode, struct file *filp) ;
int image_acq_release(struct inode *inode, struct file *filp) ;
loff_t image_acq_fops_lseek (struct file * file, loff_t offset, int orig) ;
ssize_t image_acq_fops_write(struct file * file, const char * buf, size_t count, loff_t *off) ;
int image_acq_mmap (struct file *file, struct vm_area_struct *vma) ;

static struct file_operations image_acq_fops = {
   owner:    THIS_MODULE,
   llseek:   image_acq_fops_lseek,
//   read:     image_acq_fops_read,
   write:    image_acq_fops_write,
//   ioctl:    image_acq_ioctl,
   open:     image_acq_open,
   mmap:     image_acq_mmap,
   release:  image_acq_release
};
static int __init image_acq_init(void) {
   int res;
   sensorproc= &s_sensorproc;
   MDD1(printk("sensorproc=0x%x\n",(int) sensorproc));
   //init_ccam_dma_buf_ptr();  /// should it be done here? Why not in circbuf.c?
///   init_histograms(); - other initializations?
   res = register_chrdev(ELPHEL_MAJOR, elphel_cam_name, &image_acq_fops);
   if(res < 0) {
     printk(KERN_ERR "image_acq_init: couldn't get a major number %d.\n",ELPHEL_MAJOR);
     return res;
   }

#ifdef TEST_DISABLE_CODE

   if(request_irq(EXT_INTR_VECT,
                  elphel_FPGA_interrupt,
                  SA_INTERRUPT, // SA_SHIRQ | SA_INTERRUPT if it is a shared one.
                  "Elphel FPGA interrupts",
                  NULL)) {
               printk(KERN_ERR "Can't allocate Elphel FPGA interrupts");
               return -EBUSY;
    }

#endif

    printk("Elphel FPGA interrupts initialized\n");
//    init_waitqueue_head(&image_acq_wait_queue);
//    DMA_buf_start=x313_dma_init();
  MDD1(printk("reset_compressor()\n"));
    reset_compressor(); /// reset compressor and buffer pointers
  MDD1(printk("x313_dma_init()\n"));
    //x313_dma_init();    /// initialize ETRAX FS DMA
  MDD1(printk("init_pgm_proc ()\n"));
    //init_pgm_proc ();   /// setup pointers to functions (not sensor-specific)
  MDD1(printk("reset_qtables()\n"));
    reset_qtables(); /// force initialization at next access
    printk(X3X3_ELPHEL_DRIVER_NAME" - %d\n",ELPHEL_MAJOR);
    return 0;
}


int image_acq_open(struct inode *inode, struct file *filp) {
  return 0;
}
int image_acq_release(struct inode *inode, struct file *filp) {
  return 0;
}
loff_t image_acq_fops_lseek (struct file * file, loff_t offset, int orig) {
  return 0;
}
ssize_t image_acq_fops_write(struct file * file, const char * buf, size_t count, loff_t *off) {
  return 0;
}
int image_acq_mmap (struct file *file, struct vm_area_struct *vma) {
  return 0;
}


module_init(image_acq_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_IMAGEACQ_DRIVER_NAME);
