/** @file sensor_common.h
 * @brief This module handles sensor discovery, initialization and programming tasks
 * common for all sensors. Task that are implemented:
 * - system initialization (?)
 * - compressor write (and global read) pointers
 * - populating 32-byte interframe data
 * - interrupts handling
 * - tasklets
 * - device driver that includes waiting for the next frame regardless of compression
 *
 * Copyright (C) 2016 Elphel, Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//copied from cxi2c.c - TODO:remove unneeded
//#include <linux/module.h>
#include <linux/sched.h>
//#include <linux/slab.h>
//#include <linux/errno.h>
#include <linux/kernel.h>
//#include <linux/fs.h>
//#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
#include <linux/interrupt.h>
#include <linux/time.h>
//#include <linux/vmalloc.h>
#include <linux/platform_device.h>
//#include <linux/of.h>
//#include <linux/of_device.h>


//#include <asm/system.h>
//#include <asm/byteorder.h> // endians
//#include <asm/io.h>

//#include <asm/arch/hwregs/intr_vect_defs.h> /// ETRAX interrupt registers

//#include <asm/irq.h>

//#include <asm/delay.h>
//#include <asm/uaccess.h>
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
//#include <asm/elphel/fpgaconfa.h>
#include <elphel/exifa.h>
//#include <elphel/x393_types.h>
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
#include "x393_macro.h"
#include "x393.h"

/**
 * @brief driver name to display in log messages
 */
#define IMAGEACQ_DRIVER_NAME "Elphel (R) Model 393 Image Acquisition device driver"

/**@struct jpeg_ptr_t
 * @brief \e jpeg_ptr_t structure contains read and write pointers along with
 * IRQ number for a single channel
 * @var jpeg_ptr_t::jpeg_wr
 * JPEG write pointer in 32 bit words
 * @var jpeg_ptr_t::jpeg_rp
 * JPEG read pointer in 32 bit words
 * @var jpeg_ptr_t::fpga_cntr_prev
 * This field contains previous value of the FPGA transfer counter which is used
 * to find out if it has changed
 * @var jpeg_ptr_t::irq_num_comp
 * IRQ number associated with compressor
 * @var jpeg_ptr_t::irq_num_sens
 * IRQ number associated with sensor
 * @var jpeg_ptr_t::chn_num
 * Current channel number
 */
struct jpeg_ptr_t {
	volatile int jpeg_wp;
	volatile int jpeg_rp;
	volatile int fpga_cntr_prev;
	unsigned int irq_num_comp;
	unsigned int irq_num_sens;
	unsigned int chn_num;
	volatile unsigned int flags;
};

/**@struct image_acq_pd_t
 * @brief \e image_acq_pd contains private data for the image acquisition driver
 */
struct image_acq_pd_t {
    int minor;
    struct jpeg_ptr_t jpeg_ptr[IMAGE_CHN_NUM];
};

static struct image_acq_pd_t image_acq_priv;

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

#ifdef TEST_DISABLE_CODE
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
#endif /* TEST_DISABLE_CODE */

int camseq_get_jpeg_wp(unsigned int chn)
{
	return (chn < IMAGE_CHN_NUM) ? image_acq_priv.jpeg_ptr[chn].jpeg_wp : 0;
}

int camseq_get_jpeg_rp(unsigned int chn)
{
	return (chn < IMAGE_CHN_NUM) ? image_acq_priv.jpeg_ptr[chn].jpeg_rp : 0;
}

void camseq_set_jpeg_rp(unsigned int chn, int ptr)
{
	if (chn < IMAGE_CHN_NUM) {
		image_acq_priv.jpeg_ptr[chn].jpeg_rp = ptr;
	}
}

/*!
   End of compressor-related code - TODO: move to a separate file?
*/

static void dump_priv_data(int chn)
{
	int i;

	if (chn < IMAGE_CHN_NUM) {
		printk(KERN_DEBUG "jpeg_wp (in bytes): 0x%x\n", image_acq_priv.jpeg_ptr[chn].jpeg_wp << 2);
		printk(KERN_DEBUG "jpeg_rp (in bytes): 0x%x\n", image_acq_priv.jpeg_ptr[chn].jpeg_rp << 2);
		printk(KERN_DEBUG "fpga_cntr_prev: 0x%x\n", image_acq_priv.jpeg_ptr[chn].fpga_cntr_prev);
		printk(KERN_DEBUG "irq_num_comp: 0x%x\n", image_acq_priv.jpeg_ptr[chn].irq_num_comp);
		printk(KERN_DEBUG "irq_num_sens: 0x%x\n", image_acq_priv.jpeg_ptr[chn].irq_num_sens);
		printk(KERN_DEBUG "chn_num: 0x%x\n", image_acq_priv.jpeg_ptr[chn].chn_num);
		printk(KERN_DEBUG "flags: 0x%x\n", image_acq_priv.jpeg_ptr[chn].flags);
	} else {
		for (i = 0; i < IMAGE_CHN_NUM; i++) {
			printk(KERN_DEBUG "jpeg_wp: 0x%x\n", image_acq_priv.jpeg_ptr[i].jpeg_wp);
			printk(KERN_DEBUG "jpeg_rp: 0x%x\n", image_acq_priv.jpeg_ptr[i].jpeg_rp);
			printk(KERN_DEBUG "fpga_cntr_prev: 0x%x\n", image_acq_priv.jpeg_ptr[i].fpga_cntr_prev);
			printk(KERN_DEBUG "irq_num_comp: 0x%x\n", image_acq_priv.jpeg_ptr[i].irq_num_comp);
			printk(KERN_DEBUG "irq_num_sens: 0x%x\n", image_acq_priv.jpeg_ptr[i].irq_num_sens);
			printk(KERN_DEBUG "chn_num: 0x%x\n", image_acq_priv.jpeg_ptr[i].chn_num);
			printk(KERN_DEBUG "flags: 0x%x\n", image_acq_priv.jpeg_ptr[i].flags);
		}
	}
}


static const struct of_device_id elphel393_sensor_of_match[];
static struct sensorproc_t s_sensorproc; // sensor parameters and functions to call
struct sensorproc_t * sensorproc = NULL;
//EXPORT_SYMBOL_GPL(sensorproc);
//wait_queue_head_t image_acq_wait_queue;  /// queue for the sensor frame interrupts


void tasklet_fpga_function(unsigned long arg);

/**
 * @brief Copy #sensorproc structure, needed for multisensor board to be able
 * to replace some of the functions
 * @param[in]   copy   pointer to a copy structure
 * @return      pointer to a \b copy structure
 */
struct sensorproc_t * copy_sensorproc (struct sensorproc_t * copy)
{
	/** copy sensor functions */
	memcpy(copy, sensorproc, sizeof(struct sensorproc_t));
	return copy;
}

//#ifdef TEST_DISABLE_CODE


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

DECLARE_TASKLET(tasklet_fpga, tasklet_fpga_function, 0); /// 0 - no arguments for now



/**
 * @brief Reads FPGA data pointer from the channel given and updates its JPEG_wp
 *
 * This function gets current pointer inside frame buffer and compares it with the previous
 * value. It returns immediately if pointer has not advanced or updates \e jpeg_wr field in #jpeg_ptr_t for
 * current channel. It also tracks the situation when the pointer rolls over.
 * @param[in]   jptr   pointer to #jpeg_ptr_t structure for the channel which data is to be modified
 * @return 0 if compressor was off (no advance) or 1 if write pointer did actually advance
 */
static inline int updateIRQJPEG_wp(struct jpeg_ptr_t *jptr)
{
	int xferred;                                           /// number of 32-byte chunks transferred since compressor was reset
	x393_afimux_status_t stat = x393_afimux0_status(jptr->chn_num);
	//int circbuf_size = get_globalParam(G_CIRCBUFSIZE) >> 2;
	int circbuf_size = get_globalParam(G_CIRCBUFSIZE);

	xferred = stat.offset256 - jptr->fpga_cntr_prev;
	if (xferred == 0)
		return 0;                    /// no advance (compressor was off?)

	jptr->flags |= SENS_FLAG_IRQ;
	jptr->fpga_cntr_prev = stat.offset256;
	// increment in 32 bit words
	jptr->jpeg_wp += (xferred << 3);

	return 1;
}

/**
 * @brief Calculate/update CIRCBUF parameters available after compressor interrupt
 */
inline void update_irq_circbuf(struct jpeg_ptr_t *jptr) {
	/*set_globalParam (G_CIRCBUFWP, JPEG_wp<<2);
     set_globalParam (G_FREECIRCBUF, (((get_globalParam (G_CIRCBUFRP) <= get_globalParam (G_CIRCBUFWP))? get_globalParam (G_CIRCBUFSIZE):0)+ 
                                     get_globalParam (G_CIRCBUFRP)) -    get_globalParam (G_CIRCBUFWP));*/
	/* the concept of global parameters will be changed, use one channel only for testing */
	set_globalParam(G_CIRCBUFWP, jptr->jpeg_wp);
	set_globalParam (G_FREECIRCBUF, (((get_globalParam (G_CIRCBUFRP) <= get_globalParam (G_CIRCBUFWP))? get_globalParam (G_CIRCBUFSIZE):0)+
			get_globalParam (G_CIRCBUFRP)) -    get_globalParam (G_CIRCBUFWP));
}

/**
 * @brief Calculate/update focus parameters available after compressor interrupt
 * NOTE: currently global (latest), not per-frame
 */
inline void updateIRQFocus(struct jpeg_ptr_t *jptr)
{
    //set_globalParam     (G_GFOCUS_VALUE, X313_HIGHFREQ);
    //set_imageParamsThis (P_FOCUS_VALUE, X313_HIGHFREQ);
	u32 high_freq = x393_cmprs_hifreq(jptr->chn_num);
}

inline static void set_default_interframe(struct interframe_params_t *params)
{
	params->height = 1936;
	params->width = 2592;
	params->byrshift = 3;
	params->color = 0;
	params->quality2 = 100;
}

/**
 * @brief Locate area between frames in the circular buffer
 * @return pointer to interframe parameters structure
 */
inline struct interframe_params_t* updateIRQ_interframe(struct jpeg_ptr_t *jptr) {
//   int circbuf_size=get_globalParam (G_CIRCBUFSIZE)>>2;
//   int alen = JPEG_wp-9; if (alen<0) alen+=circbuf_size;
//   int jpeg_len=ccam_dma_buf_ptr[alen] & 0xffffff;
//   set_globalParam(G_FRAME_SIZE,jpeg_len);
//   int aframe_params=(alen & 0xfffffff8)-
//                     (((jpeg_len + CCAM_MMAP_META + 3) & 0xffffffe0)>>2) /// multiple of 32-byte chunks to subtract
//                     -8; /// size of the storage area to be filled before the frame
//   if(aframe_params < 0) aframe_params += circbuf_size;
//   struct interframe_params_t* interframe= (struct interframe_params_t*) &ccam_dma_buf_ptr[aframe_params];
///// should we use memcpy as before here?
//   interframe->frame_length=jpeg_len;
//   interframe->signffff=0xffff;
//#if ELPHEL_DEBUG_THIS
//    set_globalParam          (0x306,get_globalParam (0x306)+1);
//#endif

	struct interframe_params_t *interframe;
	int len_offset = X393_BUFFSUB(jptr->jpeg_wp, INTERFRAME_PARAMS_SZ + 1);
	int jpeg_len = circbuf_priv_ptr[jptr->chn_num].buf_ptr[len_offset] & FRAME_LENGTH_MASK;
	int jpeg_start = X393_BUFFSUB(DW2BYTE(jptr->jpeg_wp) - CHUNK_SIZE - INSERTED_BYTES(jpeg_len) - CCAM_MMAP_META, jpeg_len);
	// frame_params_offset points to interframe_params_t area before current frame (this parameters belong to the frame below in memory, not the previous)
	int frame_params_offset = BYTE2DW(X393_BUFFSUB(jpeg_start, CHUNK_SIZE));

	interframe = (struct interframe_params_t *) &circbuf_priv_ptr[jptr->chn_num].buf_ptr[frame_params_offset];
	interframe->frame_length = jpeg_len;
	interframe->signffff = 0xffff;

	set_default_interframe(interframe);

	set_globalParam(G_FRAME_SIZE, jpeg_len);

   return interframe;
}

/**
 * @brief Fill exif data with the current frame data, save pointer to Exif page in the interframe area
 * @param interframe pointer to interframe parameters structure
 */
inline void updateIRQ_Exif(struct interframe_params_t* interframe) {
   int  index_time = JPEG_wp-11; if (index_time<0) index_time+=get_globalParam (G_CIRCBUFSIZE)>>2;
#ifdef TES_DISABLE_CODE
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

#endif /* TES_DISABLE_CODE */
}


/**
 * @brief hardware IRQ service
 * most urgent tasks
 * @param irq 
 * @param dev_id 
 * @return 
 */
/*static irqreturn_t elphel_FPGA_interrupt(int irq, void *dev_id) {
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
}*/

/**
 * @brief Handle interrupts from sensor channels. This handler is installed without SA_INTERRUPT
 * flag meaning that interrupts are enabled during processing. Such behavior is recommended in LDD3.
 * @param[in]   irq   interrupt number
 * @param[in]   dev_id pointer to driver's private data structure #jpeg_ptr_t corresponding to
 * the channel which raise interrupt
 * @return \e IRQ_HANDLED if interrupt was processed and \e IRQ_NONE otherwise
 */
static irqreturn_t frame_sync_irq_handler(int irq, void *dev_id)
{
	struct jpeg_ptr_t *priv = dev_id;

	update_frame_pars();
	wake_up_interruptible(&framepars_wait_queue);
	tasklet_schedule(&tasklet_fpga);

	return IRQ_HANDLED;
}

/**
 * @brief Handle interrupts from JPEG compressor channels. This handler is installed without SA_INTERRUPT
 * flag meaning that interrupts are enabled during processing. Such behavior is recommended in LDD3.
 * @param[in]   irq   interrupt number
 * @param[in]   dev_id pointer to driver's private data structure #jpeg_ptr_t corresponding to
 * the channel which raise interrupt
 * @return \e IRQ_HANDLED if interrupt was processed and \e IRQ_NONE otherwise
 */
static irqreturn_t compressor_irq_handler(int irq, void *dev_id)
{
	struct jpeg_ptr_t *priv = dev_id;
	struct interframe_params_t *interframe;
	x393_cmprs_interrupts_t irq_ctrl;

	if (updateIRQJPEG_wp(priv)) {
		update_irq_circbuf(priv);
		updateIRQFocus(priv);
		interframe = updateIRQ_interframe(priv);
		//updateIRQ_Exif(interframe);
		wake_up_interruptible(&circbuf_wait_queue);
	}
	//wake_up_interruptible(&framepars_wait_queue);

	tasklet_schedule(&tasklet_fpga);
	irq_ctrl.interrupt_cmd = IRQ_CLEAR;
	x393_cmprs_interrupts(irq_ctrl, priv->chn_num);

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

  int i, j;
  int last_image_chunk;
  int len32;


#ifdef TEST_DISABLE_CODE
/// Time is out?
  if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
#endif /* TEST_DISABLE_CODE */
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
#ifdef TEST_DISABLE_CODE
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
#endif /* TEST_DISABLE_CODE */
/// Process parameters 
  if ((tasklet_disable & (1 << TASKLET_CTL_PGM))   == 0) {
      processPars (sensorproc, getThisFrameNumber(), get_globalParam(G_MAXAHEAD)); /// program parameters
      PROFILE_NOW(4);
  }
#ifdef TEST_DISABLE_CODE
/// Time is out?
  if ((getThisFrameNumber() ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; /// already next frame
/// Are C histograms needed?
#endif /* TEST_DISABLE_CODE */
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
#ifdef TEST_DISABLE_CODE
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
#endif /* TEST_DISABLE_CODE */
}

//#endif /* TEST_DISABLE_CODE */

/**
 * @brief resets compressor and buffer pointers
 */
void reset_compressor(unsigned int chn)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
#ifdef TEST_DISABLE_CODE
	port_csp0_addr[X313_WA_COMP_CMD]= COMPCMD_RESET; /// bypasses command sequencer
	if (framepars) set_imageParamsR_all( P_COMPRESSOR_RUN, COMPRESSOR_RUN_STOP );
	else printk ("framepars is not initialized\n");
	/// TODO: There still is a possibility, that there are compressor commands in the hardware que. Should we stop the hardware sequencer here (and restart it later)?
#endif /* TEST_DISABLE_CODE */
	image_acq_priv.jpeg_ptr[chn].jpeg_wp = 0;
	image_acq_priv.jpeg_ptr[chn].jpeg_rp = 0;
	image_acq_priv.jpeg_ptr[chn].fpga_cntr_prev = 0;
	image_acq_priv.jpeg_ptr[chn].flags = 0;
	//update_irq_circbuf(jptr);
	local_irq_restore(flags);
}

/**
 * @brief Camera interrupts on/off  (currently both in the FPGA and in the CPU)
 * @param on 1 - enable, 0 - disable interrupts
 */
void camera_interrupts (int on) {
	int i;
	x393_cmprs_interrupts_t irq_ctrl;

	//MDF2(printk ("camera_interrupts(%d)\n",on));
	dev_dbg(NULL, "set camera interrupts status: %d\n", on);
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

	irq_ctrl.interrupt_cmd = on ? IRQ_ENABLE : IRQ_DISABLE;
	for (i = 0; i < IMAGE_CHN_NUM; i++) {
		x393_cmprs_interrupts(irq_ctrl, i);
	}
}

/**
 * @brief sensor_common driver probing function
 * @param[in]   pdev   pointer to \b platform_device structure
 * @return      0 on success or negative error code otherwise
 */
//static int image_acq_init(struct platform_device *pdev)
int image_acq_init(struct platform_device *pdev)
{
	int i;
	int res;
	unsigned int irq;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	const char *frame_sync_irq_names[4] = {"frame_sync_irq_0", "frame_sync_irq_1",
			"frame_sync_irq_2", "frame_sync_irq_3"};
	const char *compressor_irq_names[4] = {"compr_irq_0", "compr_irq_1",
			"compr_irq_2", "compr_irq_3"};

	/* sanity check */
	/*match = of_match_device(elphel393_sensor_of_match, dev);
	if (!match)
		return -EINVAL;*/

	sensorproc= &s_sensorproc;
	//MDD1(printk("sensorproc=0x%x\n",(int) sensorproc));
	dev_dbg(dev, "sensorproc address: 0x%x\n", (int)sensorproc);

	for (i = 0; i < IMAGE_CHN_NUM; i++) {
		irq = platform_get_irq_byname(pdev, frame_sync_irq_names[i]);
		if (request_irq(irq,
				frame_sync_irq_handler,
				0, // no flags
				frame_sync_irq_names[i],
				&image_acq_priv.jpeg_ptr[i])) {
			dev_err(dev, "can not allocate Elphel FPGA interrupts\n");
			return -EBUSY;
		}

		irq = platform_get_irq_byname(pdev, compressor_irq_names[i]);
		if (request_irq(irq,
				compressor_irq_handler,
				0, // no flags
				compressor_irq_names[i],
				&image_acq_priv.jpeg_ptr[i])) {
			dev_err(dev, "can not allocate Elphel FPGA interrupts\n");
			return -EBUSY;
		}
		image_acq_priv.jpeg_ptr[i].irq_num_sens = irq;
		image_acq_priv.jpeg_ptr[i].irq_num_comp = irq;
		image_acq_priv.jpeg_ptr[i].chn_num = i;
	}

	if (init_mmio_ptr() < 0) {
		dev_err(dev, "unable to remap FPGA registers to memory region\n");
		return -EINVAL;
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

	dev_dbg(dev, "Elphel FPGA interrupts initialized\n");
	dev_dbg(dev, "reset all compressors\n");
	for (i = 0; i < IMAGE_CHN_NUM; i++) {
		reset_compressor(i);
	}
	//reset_compressor(); /// reset compressor and buffer pointers
	//MDD1(printk("x313_dma_init()\n"));
	//x313_dma_init();    /// initialize ETRAX FS DMA
	//MDD1(printk("init_pgm_proc ()\n"));
	//init_pgm_proc ();   /// setup pointers to functions (not sensor-specific)
	//MDD1(printk("reset_qtables()\n"));

	framepars_init(pdev);

	return 0;
}

int image_acq_stop(struct platform_device *pdev)
{
	return 0;
}

//static const struct of_device_id elphel393_sensor_of_match[] = {
//		{ .compatible = "elphel,elphel393-sensor-1.00" },
//		{ /* end of list */ }
//};
//MODULE_DEVICE_TABLE(of, elphel393_sensor_of_match);


/*static struct platform_driver elphel393_sensor_common = {
		.probe          = image_acq_init,
		.remove         = image_acq_stop,
		.driver = {
				.name = IMAGEACQ_DRIVER_NAME,
				.of_match_table = elphel393_sensor_of_match,
		}
};*/

//module_platform_driver(elphel393_sensor_common);

//MODULE_LICENSE("GPL");
//MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
//MODULE_DESCRIPTION(IMAGEACQ_DRIVER_NAME);
