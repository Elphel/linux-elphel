/**
 * @file sensor_common.c
 * @brief This module handles sensor discovery, initialization and programming tasks
 * common for all sensors. Task that are implemented:
 * - system initialization (?)
 * - compressor write (and global read) pointers
 * - populating 32-byte interframe data
 * - interrupts handling
 * - tasklets
 * - device driver that includes waiting for the next frame regardless of compression
 * @copyright Copyright (C) 2016 Elphel, Inc.
 * @par <b>License</b>
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
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

#include <linux/spinlock.h>


#include <uapi/elphel/c313a.h>
#include <uapi/elphel/exifa.h>
//#include <uapi/elphel/x393_devices.h>

#include "framepars.h"
#include "sensor_common.h"
#include "pgm_functions.h"
#include "circbuf.h"
#include "exif393.h"
#include "histograms.h"
#include "gamma_tables.h"
#include "quantization_tables.h"
#include "x393_macro.h"
#include "x393.h"
//#include "x393_helpers.h"

#include <asm/delay.h> // just for usleep1000()

#include "x393_fpga_functions.h"
// NC393 debug macros
#include "debug393.h"

static DEFINE_SPINLOCK(framepars_irq_0); ///<
static DEFINE_SPINLOCK(framepars_irq_1); ///<
static DEFINE_SPINLOCK(framepars_irq_2); ///<
static DEFINE_SPINLOCK(framepars_irq_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
spinlock_t * frame_irq_locks[4] = {&framepars_irq_0, &framepars_irq_1, &framepars_irq_2, &framepars_irq_3};

static DEFINE_SPINLOCK(compressor_irq_0); ///<
static DEFINE_SPINLOCK(compressor_irq_1); ///<
static DEFINE_SPINLOCK(compressor_irq_2); ///<
static DEFINE_SPINLOCK(compressor_irq_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
spinlock_t * compressor_locks[4] = {&compressor_irq_0, &compressor_irq_1, &compressor_irq_2, &compressor_irq_3};




/* Driver name to display in log messages.*/
//#define IMAGEACQ_DRIVER_DESCRIPTION      "Elphel (R) Model 393 Image Acquisition device driver"

/** @brief The size in bytes of L2 cache invalidation area. This size must be aligned to cache line size. 16 kbytes seems to be good starting point.*/
#define L2_INVAL_SIZE             (32 * 1024)

/** @brief Global pointer to basic device structure. This pointer is used in debugfs output functions */
static struct device *g_dev_ptr;

/** @brief Contains read and write pointers along with IRQ number for a single channel*/
struct jpeg_ptr_t {
    volatile u32 frame;          ///< Absolute frame number (last compressed)
	volatile int jpeg_wp;        ///< JPEG write pointer in 32 bit words
	volatile int jpeg_rp;        ///< JPEG read pointer in 32 bit words
	volatile int fpga_cntr_prev; ///< This field contains previous value of the FPGA transfer counter which is used to find out if it has changed. This pointer is in 32 bit words.
	unsigned int irq_num_comp;   ///< IRQ number associated with compressor
	unsigned int irq_num_sens;   ///< IRQ number associated with sensor
	unsigned int chn_num;        ///< Current channel number
	volatile unsigned int flags;
};

// just temporarily
 void i2c_run(void)      {}
 void i2c_stop_wait(void){}
 void udelay1000(int ms)
 {
     int i;
     for (i=0;i<ms;i++) udelay(1000);
 }




/** @brief Contains private data for the image acquisition driver */
struct image_acq_pd_t {
	int minor;								   ///< Driver minor number
	struct jpeg_ptr_t jpeg_ptr[SENSOR_PORTS];  ///< Array of read/write pointers
};


/* debug code follows */
long long zero_counter[SENSOR_PORTS] = {0};
long long corrected_offset[SENSOR_PORTS] = {0};
long long frame_counter[SENSOR_PORTS] = {0};
long long frame_pos[SENSOR_PORTS][1000] = {0};
long long get_zero_counter(unsigned int chn)
{
	return zero_counter[chn];
}
long long get_corrected_offset(unsigned int chn)
{
	return corrected_offset[chn];
}
long long get_frame_counter(unsigned int chn)
{
	return frame_counter[chn];
}
long long get_frame_pos(unsigned int chn, unsigned int pos)
{
	if (chn < SENSOR_PORTS && pos < 1000)
		return frame_pos[chn][pos];
	return 0;
}
/* end of debug code */

static struct image_acq_pd_t image_acq_priv;

u32 get_compressor_frame(unsigned int chn) ///< Sensor port number (0..3)
                                           ///< @return absolute compressed frame number
{
    return image_acq_priv.jpeg_ptr[chn & 3].frame;
}

//static volatile int JPEG_wp;
//static volatile int JPEG_rp;
//static int fpga_counter_prev=0; ///< Previous value of the FPGA transfer counter (to find out if it did change)

/** @brief  Works like a cache to time save on looking for tags in the directory (forced to recalculate if does not match)
 *  will have offset of the Exif_Image_DateTime data in meta page (Exif_Photo_SubSecTime should go immediately after in meta page)
 */
static struct meta_offsets_t { //
	int Image_DateTime;           ///< EXIF Date/Time offset
	                              ///< Has offset of the Exif_Image_DateTime data in meta page (Exif_Photo_SubSecTime should go immediately after in meta page)
	int Photo_DateTimeOriginal;   ///< EXIF Date/Time Original offset
	int Photo_ExposureTime;       ///< EXIF exposure offset
	int Image_ImageNumber;        ///< EXIF image number offset
	int Image_Orientation;        ///< EXIF image orientation offset
	int Photo_MakerNote;          ///< EXIF maker note (custom data for multi-frame composite images) offset
	int PageNumber;               ///< EXIF subchannel (0..3) number offset
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

/** Write pointer in circbuf, in bytes */
int camseq_get_jpeg_wp(unsigned int chn)
{
	return (chn < SENSOR_PORTS) ? image_acq_priv.jpeg_ptr[chn].jpeg_wp : 0;
}

int camseq_get_jpeg_rp(unsigned int chn)
{
	return (chn < SENSOR_PORTS) ? image_acq_priv.jpeg_ptr[chn].jpeg_rp : 0;
}

void camseq_set_jpeg_rp(unsigned int chn, int ptr)
{
	if (chn < SENSOR_PORTS) {
		image_acq_priv.jpeg_ptr[chn].jpeg_rp = ptr;

	    set_globalParam(chn, G_CIRCBUFRP, ptr);
	    set_globalParam(chn, G_FREECIRCBUF,
	            (((get_globalParam(chn, G_CIRCBUFRP) <= get_globalParam(chn, G_CIRCBUFWP))?
	                    get_globalParam(chn, G_CIRCBUFSIZE):0)+ get_globalParam(chn, G_CIRCBUFRP))
	                    - get_globalParam(chn, G_CIRCBUFWP));
	}
}
/** Get latest compressed frame number */
int camSeqGetJPEG_frame(unsigned int chn)
{
    return (chn < SENSOR_PORTS) ? image_acq_priv.jpeg_ptr[chn].frame : 0;
}

/** Write pointer in circbuf, in DWORDs */
int camSeqGetJPEG_wp(unsigned int chn)
{
    return (chn < SENSOR_PORTS) ? image_acq_priv.jpeg_ptr[chn].jpeg_wp : 0;
}

int camSeqGetJPEG_rp(unsigned int chn)
{
    return (chn < SENSOR_PORTS) ? image_acq_priv.jpeg_ptr[chn].jpeg_rp : 0;
}

void camSeqSetJPEG_rp(unsigned int chn, ///< channel (0..3)
                      int ptr)          /// DWORD index in the buffer

{
    if (chn < SENSOR_PORTS) {
        image_acq_priv.jpeg_ptr[chn].jpeg_rp = ptr << 2;

        set_globalParam(chn, G_CIRCBUFRP, ptr);
        set_globalParam(chn, G_FREECIRCBUF,
                (((get_globalParam(chn, G_CIRCBUFRP) <= get_globalParam(chn, G_CIRCBUFWP))?
                        get_globalParam(chn, G_CIRCBUFSIZE):0)+ get_globalParam(chn, G_CIRCBUFRP))
                        - get_globalParam(chn, G_CIRCBUFWP));
    }
}
#ifdef NC353
void camSeqSetJPEG_rp(int p) {
                              JPEG_rp=p;
                              set_globalParam(G_CIRCBUFRP,  p<< 2);
                              set_globalParam(G_FREECIRCBUF,
                                    (((get_globalParam(G_CIRCBUFRP) <= get_globalParam(G_CIRCBUFWP))?
                                       get_globalParam(G_CIRCBUFSIZE):0)+ get_globalParam(G_CIRCBUFRP))
                                       - get_globalParam(G_CIRCBUFWP));
                               }

#endif

/** Return current frame number, if it was from the compressor interrupt
 * get the compressed frame number (updated when compression is over,
 * can happen even later than start of the next frame.
 * Otherwise use command sequencer (switching at frame sync (start of frame)
 * TODO: at each frame interrupt check how far is compressor behind,
 * take over if >=2 ? */
int getHardFrameNumber(int sensor_port,     ///< Sensor port number
		               int use_compressor)  ///< 1 - use compressor frame number, 0 - use sequencer frame number
		                                    ///< @return hardware frame number (4 bit currently, mod PARS_FRAMES )
{
	x393_cmprs_status_t cmprs_status;
	x393_cmdseqmux_status_t cmdseqmux_status;
	if (use_compressor) {
		cmprs_status = x393_cmprs_status(sensor_port);
		return cmprs_status.frame;
	} else {
		cmdseqmux_status = x393_cmdseqmux_status();
		switch(sensor_port){
		case 0:  return cmdseqmux_status.frame_num0;
		case 1:  return cmdseqmux_status.frame_num1;
		case 2:  return cmdseqmux_status.frame_num2;
		default: return cmdseqmux_status.frame_num3;
		}
	}

}

/*!
   End of compressor-related code - TODO: move to a separate file?
 */

static const struct of_device_id elphel393_sensor_of_match[];
static struct sensorproc_t as_sensorproc[SENSOR_PORTS]; // sensor parameters and functions to call
struct sensorproc_t * asensorproc = NULL;
//EXPORT_SYMBOL_GPL(sensorproc);
//wait_queue_head_t image_acq_wait_queue;  // queue for the sensor frame interrupts


void tasklet_cmdseq_function(unsigned long arg);
void tasklet_compressor_function(unsigned long arg);

/**
 * @brief Copy #sensorproc structure, needed for multisensor board to be able
 * to replace some of the functions
 * @param[in]   sensor_port sensor port number
 * @param[in]   copy   pointer to a copy structure
 * @return      pointer to a \b copy structure
 */
struct sensorproc_t * copy_sensorproc (int sensor_port, struct sensorproc_t * copy)
{
	/** copy sensor functions */
	memcpy(copy, &asensorproc[sensor_port], sizeof(struct sensorproc_t));
	return copy;
}

//#ifdef TEST_DISABLE_CODE


//
// initializes structures for the image acquisition parameter
// initializes hardware i2c controller and the command sequencer (leaves them stopped to ignore any frame syncs)
// sets some default acquisition parameters
// Maybe - set up DMA also?
// TODO: Take care while turning off reset on i2c and cmd_sequencer so there will be no sensor vsync lost
// (easier to do in FPGA)
// Done:
// #define CCAM_VSYNC_ON  port_csp0_addr[X313_WA_DCR1]=X353_DCR1(BLOCKVSYNC,0)
// #define CCAM_VSYNC_OFF port_csp0_addr[X313_WA_DCR1]=X353_DCR1(BLOCKVSYNC,1)
//
int init_acq_sensor(void); // Never used?

//DECLARE_TASKLET(tasklet_cmdseq, tasklet_cmdseq_function, 0); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_cmdseq_0, tasklet_cmdseq_function, 0); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_cmdseq_1, tasklet_cmdseq_function, 1); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_cmdseq_2, tasklet_cmdseq_function, 2); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_cmdseq_3, tasklet_cmdseq_function, 3); // 0 - no arguments for now
static struct tasklet_struct *tasklet_cmdseq[SENSOR_PORTS] = {&tasklet_cmdseq_0, &tasklet_cmdseq_1, &tasklet_cmdseq_2, &tasklet_cmdseq_3};

DECLARE_TASKLET(tasklet_compressor_0, tasklet_compressor_function, 0); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_compressor_1, tasklet_compressor_function, 1); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_compressor_2, tasklet_compressor_function, 2); // 0 - no arguments for now
DECLARE_TASKLET(tasklet_compressor_3, tasklet_compressor_function, 3); // 0 - no arguments for now
static struct tasklet_struct *tasklets_compressor[SENSOR_PORTS] = {&tasklet_compressor_0, &tasklet_compressor_1, &tasklet_compressor_2, &tasklet_compressor_3};


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
	phys_addr_t phys_addr;
	void *virt_addr;
	//	int prev_dword;
	int xferred;                                           // number of 32-byte chunks transferred since compressor was reset
//	x393_cmdseqmux_status_t cmdseqmux_status;
	x393_afimux_status_t stat = x393_afimux0_status(jptr->chn_num);
	int frame16;
	u32 aframe = GLOBALPARS(jptr->chn_num,G_THIS_FRAME); // thisFrameNumber(jptr->chn_num); // current absolute frame number

	xferred = stat.offset256 - (jptr->fpga_cntr_prev >> 3);
	if (xferred == 0)
		return 0;                                          // no advance (compressor was off?)

	jptr->flags |= SENS_FLAG_IRQ;
	jptr->fpga_cntr_prev = jptr->jpeg_wp;
	jptr->jpeg_wp = (stat.offset256 << 3);

// Find absolute frame number just compressed WRONG, should use comressor frame number
//	cmdseqmux_status = x393_cmdseqmux_status(); // CMDSEQMUX status data (frame numbers and interrupts
	frame16 = getHardFrameNumber(jptr->chn_num, 1); // Use compressor

	if (frame16 > (aframe & PARS_FRAMES_MASK))
	    aframe -= 16;
	jptr->frame = (aframe & ~PARS_FRAMES_MASK) | frame16; // This is absolute compressed frame number, may lag behind current one

	/* debug code follows */
	frame_counter[jptr->chn_num] += 1;
	if (jptr->jpeg_wp == 0) {
		zero_counter[jptr->chn_num] += 1;
		if (zero_counter[jptr->chn_num] < 1000)
			frame_pos[jptr->chn_num][zero_counter[jptr->chn_num] - 1] = frame_counter[jptr->chn_num];
	}

	// invalidate CPU L1 and L2 caches
	// the code below was used to find cache coherence issues
	phys_addr = circbuf_priv_ptr[jptr->chn_num].phys_addr + DW2BYTE(jptr->jpeg_wp) - CHUNK_SIZE;
	virt_addr = circbuf_priv_ptr[jptr->chn_num].buf_ptr + jptr->jpeg_wp - INTERFRAME_PARAMS_SZ;
	outer_inv_range(phys_addr, phys_addr + (CHUNK_SIZE - 1));
	__cpuc_flush_dcache_area(virt_addr, CHUNK_SIZE);

	return 1;
}

/**
 * Calculate/update CIRCBUF parameters available after compressor interrupt
 */
inline void update_irq_circbuf(struct jpeg_ptr_t *jptr) {
	/*set_globalParam (G_CIRCBUFWP, JPEG_wp<<2);set_globalParam (G_FREECIRCBUF, (((get_globalParam (G_CIRCBUFRP) <= get_globalParam (G_CIRCBUFWP))? get_globalParam (G_CIRCBUFSIZE):0)+
                                     get_globalParam (G_CIRCBUFRP)) -    get_globalParam (G_CIRCBUFWP));*/
	/* the concept of global parameters will be changed, use one channel only for testing */
	set_globalParam(jptr->chn_num, G_CIRCBUFWP, jptr->jpeg_wp);
	set_globalParam (jptr->chn_num, G_FREECIRCBUF,
	        (((get_globalParam (jptr->chn_num, G_CIRCBUFRP) <= get_globalParam (jptr->chn_num, G_CIRCBUFWP))?get_globalParam (jptr->chn_num, G_CIRCBUFSIZE):0)+
			get_globalParam (jptr->chn_num, G_CIRCBUFRP)) -    get_globalParam (jptr->chn_num, G_CIRCBUFWP));
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
	set_globalParam     (jptr->chn_num, G_GFOCUS_VALUE, high_freq);
	set_imageParamsThis (jptr->chn_num, P_FOCUS_VALUE, high_freq);
}

/**
 * @brief Locate area between frames in the circular buffer
 * @return pointer to interframe parameters structure
 */
inline struct interframe_params_t* updateIRQ_interframe(struct jpeg_ptr_t *jptr)
{
	dma_addr_t phys_addr;
	void *virt_addr;
	int chn = jptr->chn_num;
	struct interframe_params_t *interframe = NULL;
	int len_offset = X393_BUFFSUB_CHN(DW2BYTE(jptr->jpeg_wp), CHUNK_SIZE + 4, chn);
	int jpeg_len = circbuf_priv_ptr[jptr->chn_num].buf_ptr[BYTE2DW(len_offset)] & FRAME_LENGTH_MASK;
	int jpeg_start = X393_BUFFSUB_CHN(DW2BYTE(jptr->jpeg_wp) - CHUNK_SIZE - INSERTED_BYTES(jpeg_len) - CCAM_MMAP_META, jpeg_len,chn);
	// frame_params_offset points to interframe_params_t area before current frame (this parameters belong to the frame below in memory, not the previous)
	int frame_params_offset = BYTE2DW(X393_BUFFSUB_CHN(jpeg_start, CHUNK_SIZE, chn));
	int prev_len32_off = X393_BUFFSUB_CHN(jpeg_start, CHUNK_SIZE + 4,chn);
	int prev_len32 = circbuf_priv_ptr[jptr->chn_num].buf_ptr[BYTE2DW(prev_len32_off)];

	if ((prev_len32 & MARKER_FF) != MARKER_FF) {
		// try to correct offset
		prev_len32_off = X393_BUFFSUB_CHN(prev_len32_off, 0x20,chn);
		prev_len32 = circbuf_priv_ptr[jptr->chn_num].buf_ptr[BYTE2DW(prev_len32_off)];
		if ((prev_len32 & MARKER_FF) == MARKER_FF) {
			frame_params_offset = BYTE2DW(X393_BUFFADD_CHN(prev_len32_off, 4, jptr->chn_num));
		}
	}

	interframe = (struct interframe_params_t *) &circbuf_priv_ptr[jptr->chn_num].buf_ptr[frame_params_offset];
	interframe->frame_length = jpeg_len;
	interframe->signffff = 0xffff;

	set_globalParam(jptr->chn_num, G_FRAME_SIZE, jpeg_len);

	// invalidate CPU L1 and L2 caches (in this order)
	phys_addr = circbuf_priv_ptr[jptr->chn_num].phys_addr + DW2BYTE(frame_params_offset);
	virt_addr = interframe;
	__cpuc_flush_dcache_area(virt_addr, CHUNK_SIZE);
	outer_inv_range(phys_addr, phys_addr + (CHUNK_SIZE - 1));

	return interframe;
}
#ifdef NC353
inline struct interframe_params_t* updateIRQ_interframe353(struct jpeg_ptr_t *jptr) {
   int circbuf_size=get_globalParam (G_CIRCBUFSIZE)>>2;
   int alen = jptr->jpeg_wp-9; if (alen<0) alen+=circbuf_size;
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
#endif





/** Fill exif data with the current frame data, save pointer to Exif page in the interframe area
 *
 * TODO NC393: Allow lag between current frame and compressor frame, use only previous parameters (among those copied)
 *
 *
 */
inline void updateIRQ_Exif(struct jpeg_ptr_t *jptr,                ///< pointer to jpeg_ptr_t structure with read/write image pointers
                           struct interframe_params_t* interframe) ///< pointer to interframe parameters structure
{
	unsigned char short_buff[2];
	unsigned int sensor_port = jptr->chn_num;
	int  index_time = jptr->jpeg_wp - 11;
    char time_buff[27];
    char * exif_meta_time_string;
    int global_flips, extra_flips;
    unsigned char orientations[]="1638274545273816";
    unsigned char orientation_short[2];
    int maker_offset;
    u32 frame =  jptr->frame;
//    int frame_index = frame & PASTPARS_SAVE_ENTRIES_MASK;
    // NC393: current parameters are valid at compressor done interrupt (after frame sync interrupts latest valid is new frame number - 2
    if (index_time<0) index_time+=get_globalParam (sensor_port, G_CIRCBUFSIZE)>>2;
	//   struct exif_datetime_t
	// calculates datetime([20] and subsec[7], returns  pointer to char[27]
	exif_meta_time_string=encode_time(time_buff, ccam_dma_buf_ptr[sensor_port][index_time], ccam_dma_buf_ptr[sensor_port][index_time+1]);
	// may be split in datetime/subsec - now it will not notice missing subsec field in template
	write_meta_irq(sensor_port, exif_meta_time_string, &meta_offsets.Photo_DateTimeOriginal,  Exif_Photo_DateTimeOriginal, 27);
	write_meta_irq(sensor_port, exif_meta_time_string, &meta_offsets.Image_DateTime,  Exif_Image_DateTime, 20); // may use 27 if room is provided
    putlong_meta_irq(sensor_port, get_imageParamsFrame(sensor_port, P_EXPOS, frame), &meta_offsets.Photo_ExposureTime,  Exif_Photo_ExposureTime);
	putlong_meta_irq(sensor_port, frame, &meta_offsets.Image_ImageNumber,   Exif_Image_ImageNumber);
	//Exif_Photo_MakerNote
	global_flips=(get_imageParamsFrame(sensor_port, P_FLIPH, frame) & 1) | ((get_imageParamsFrame(sensor_port, P_FLIPV, frame)<<1)  & 2);
	extra_flips=0;
	if (get_imageParamsFrame(sensor_port, P_MULTI_MODE,frame)!=0) {
		extra_flips=get_imageParamsFrame(sensor_port, P_MULTI_MODE_FLIPS,frame);
		global_flips=extra_flips & 3;
	}

	orientation_short[0]=0;
	orientation_short[1]=0xf & orientations[(get_imageParamsFrame(sensor_port, P_PORTRAIT, frame)&3) | (global_flips<<2)];
	write_meta_irq(sensor_port, orientation_short, &meta_offsets.Image_Orientation,   Exif_Image_Orientation, 2);

	// write sensor number
//	short_buff[0] = 0xf & (jptr->chn_num >> 8);
	short_buff[0] = 0;
	short_buff[1] = 0xf & sensor_port;
	write_meta_irq(sensor_port, short_buff, &meta_offsets.PageNumber, Exif_Image_PageNumber, 2);

	//TODO - use memcpy
	maker_offset=putlong_meta_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GAINR,frame),   &meta_offsets.Photo_MakerNote, Exif_Photo_MakerNote);
	if (maker_offset>0) {
#if 0 // just debugging
        putlong_meta_raw_irq(sensor_port, frame,                                            maker_offset+ 4);
        putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_EXPOS, frame), maker_offset+ 8);
        putlong_meta_raw_irq(sensor_port, ccam_dma_buf_ptr[sensor_port][index_time],        maker_offset+12);
        putlong_meta_raw_irq(sensor_port, ccam_dma_buf_ptr[sensor_port][index_time+1],      maker_offset+16);
#else
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GAING,frame),   maker_offset+4);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GAINGB,frame),  maker_offset+8);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GAINB,frame),   maker_offset+12);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GTAB_R,frame),  maker_offset+16);
#endif
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GTAB_G,frame),  maker_offset+20);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GTAB_GB,frame), maker_offset+24);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_GTAB_B,frame),  maker_offset+28);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_WOI_LEFT,frame) | (get_imageParamsFrame(sensor_port, P_WOI_WIDTH,frame)<<16),  maker_offset+32); //No Past
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_WOI_TOP,frame) | (get_imageParamsFrame(sensor_port, P_WOI_HEIGHT,frame)<<16),  maker_offset+36); //No Past
		putlong_meta_raw_irq(sensor_port,   global_flips |
				((get_imageParamsFrame(sensor_port, P_BAYER, frame)<<2)     & 0xc) |
				((get_imageParamsFrame(sensor_port, P_COLOR, frame)<<4)     & 0xF0) |
				((get_imageParamsFrame(sensor_port, P_DCM_HOR, frame)<<8)   & 0xF00) |
				((get_imageParamsFrame(sensor_port, P_DCM_VERT, frame)<<12) & 0xF000) |
				((get_imageParamsFrame(sensor_port, P_BIN_HOR, frame)<<16)  & 0xF0000) |
				((get_imageParamsFrame(sensor_port, P_BIN_VERT, frame)<<20) & 0xF00000) |
				(extra_flips <<24) ,  maker_offset+40);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_MULTI_HEIGHT_BLANK1,frame),   maker_offset+44);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_MULTI_HEIGHT_BLANK2,frame),   maker_offset+48);
		//     putlong_meta_raw_irq(0x1234567,  maker_offset+52); // just testing
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, P_QUALITY, frame) |
		        ((get_imageParamsFrame(sensor_port, P_PORTRAIT, frame)&1)<<7) |
		        ( get_imageParamsFrame(sensor_port, P_CORING_INDEX, frame)<<16),  maker_offset+52);
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, G_TEMPERATURE01, frame),  maker_offset+56); // data should be provided by a running daemon
		putlong_meta_raw_irq(sensor_port, get_imageParamsFrame(sensor_port, G_TEMPERATURE23, frame),  maker_offset+60);
		//get_globalParam(G_TASKLET_CTL)
		// left 1 long spare (+44)
	}
	interframe->meta_index=store_meta(sensor_port);
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
// read hardware write pointer (will only advance if compression was on)
///find out if compressor was running and update pointers, exif, ...?
   if (updateIRQJPEG_wp()) { ///also fills P_FRAME ahead
      updateIRQCircbuf();
      updateIRQFocus(); ///NOTE: currently global (latest), not per-frame
      struct interframe_params_t* interframe= updateIRQ_interframe(); // fills jpeg_len, signffff
// should we use memcpy as before here?
//      interframe->frame_length=jpeg_len;
//      interframe->signffff=0xffff;
      updateIRQ_Exif(interframe);
      updateFramePars(X3X3_I2C_FRAME, interframe);
      wake_up_interruptible(&circbuf_wait_queue); // only when frame is acquired
   } else {
      updateFramePars(X3X3_I2C_FRAME, NULL); 
   }
   PROFILE_NOW(1);
   wake_up_interruptible(&framepars_wait_queue); // all interrupts, not just frames acquired
   tasklet_schedule(&tasklet_cmdseq); // trigger software interrupt

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
	struct jpeg_ptr_t *jptr = dev_id;
    x393_cmdframeseq_mode_t cmdframeseq_mode = {.d32 = 0};
    unsigned long flags;
    int frame16;
    u32 aframe;
    cmdframeseq_mode.interrupt_cmd = IRQ_CLEAR;
//    local_irq_save(flags);
    spin_lock_irqsave(frame_irq_locks[jptr->chn_num],flags);
    aframe = GLOBALPARS(jptr->chn_num, G_THIS_FRAME); // thisFrameNumber(jptr->chn_num); // current absolute frame number
    frame16 = getHardFrameNumber(jptr->chn_num, 0); // Use sensor frame number
    updateFramePars     (jptr->chn_num,
                         frame16);
	wake_up_interruptible(&aframepars_wait_queue[jptr->chn_num]);
	//	tasklet_schedule(&tasklet_cmdseq);
	tasklet_schedule(tasklet_cmdseq[jptr->chn_num]);
	x393_cmdframeseq_ctrl(cmdframeseq_mode, jptr->chn_num);
//    local_irq_restore(flags);
    spin_unlock_irqrestore(frame_irq_locks[jptr->chn_num],flags);
	return IRQ_HANDLED;
}

/**
 * @brief Handle interrupts from JPEG compressor channels. This handler is installed without SA_INTERRUPT
 * flag meaning that interrupts are enabled during processing. Such behavior is recommended in LDD3.
 * @param[in]   irq   interrupt number
 * @param[in]   dev_id pointer to driver's private data structure #jpeg_ptr_t corresponding to
 * the channel which raised interrupt
 * @return \e IRQ_HANDLED if interrupt was processed and \e IRQ_NONE otherwise
 */
static irqreturn_t compressor_irq_handler(int irq, void *dev_id)
{
	struct jpeg_ptr_t *jptr = dev_id;
	struct interframe_params_t *interframe = NULL;
	x393_cmprs_interrupts_t irq_ctrl;
//	int frame16;
	unsigned long flags;

//	local_irq_save(flags);
    spin_lock_irqsave(compressor_locks[jptr->chn_num],flags);


	if (updateIRQJPEG_wp(jptr)) {                // Updates write pointer, invalidates cache, calculates absolute frame number (compressed)
		update_irq_circbuf(jptr);                // Update global parameters (accessible over mmap): G_CIRCBUFWP, G_FREECIRCBUF (depends on user-set G_CIRCBUFRP)
		updateIRQFocus(jptr);                    // Reads FPGA and updates both G_GFOCUS_VALUE and P_FOCUS_VALUE
		interframe = updateIRQ_interframe(jptr); // updates SOME data in the 32-byte gaps between the images:
		                                         // Calculates frame size, adds 0xffff signature to detect buffer overrun
		                                         // invalidates cache

		updateIRQ_Exif(jptr, interframe);        // Updates Exif data for compressed frame (separate ring buffer), extending interframe data
		                                         // interframe use: just adds meta_index

//	    frame16 = getHardFrameNumber(jptr->chn_num, 1); // Use compressor frame number
	    // Updarte G_COMPRESSOR_FRAME (from jptr->frame, set other interframe data from past parameters (subset of all parameters preserved after the frame)
	    updateInterFrame(jptr->chn_num, // Sensor port number (0..3)
	                     jptr->frame,   // absolute compressed frame number, updated in updateIRQJPEG_wp
	                                    // interrupt should be processed after frame sync interrupt
	                     interframe);   // pointer to the area in circbuf to save parameters
	                                    // copies some data from old (but not pastpars) to interframe
	    tasklet_schedule(tasklets_compressor[jptr->chn_num]);
//		wake_up_interruptible(&circbuf_wait_queue); // should be done in tasklet (after cache invalidation)
	}
	//wake_up_interruptible(&framepars_wait_queue);
	//	tasklet_schedule(&tasklet_cmdseq);
//	tasklet_schedule(tasklets_compressor[jptr->chn_num]);
	irq_ctrl.interrupt_cmd = IRQ_CLEAR;
	x393_cmprs_interrupts(irq_ctrl, jptr->chn_num);
//	local_irq_restore(flags);
    spin_unlock_irqrestore(compressor_locks[jptr->chn_num],flags);

	return IRQ_HANDLED;
}

/**
 * @brief Tasklet - software interrupt
 * lower priority tasks
 * try to implement some balancing - if job is not finished - reduce FPS for it (alternate jobs)?
 * @param arg not used
 */

void tasklet_compressor_function(unsigned long arg)
{
    dma_addr_t phys_addr_start, phys_addr_end;
    void *virt_addr_start;
//    int sensor_port = image_acq_priv.jpeg_ptr[arg].chn_num; // == arg & 3
    const struct jpeg_ptr_t *jptr = &image_acq_priv.jpeg_ptr[arg];
    unsigned int sz;
    u32 ccam_dma_size = circbuf_priv_ptr[jptr->chn_num].buf_size;
    /* invalidate L2 cache lines in the beginning of current frame */
    phys_addr_start = circbuf_priv_ptr[jptr->chn_num].phys_addr + DW2BYTE(jptr->fpga_cntr_prev);
    virt_addr_start = circbuf_priv_ptr[jptr->chn_num].buf_ptr + jptr->fpga_cntr_prev;
    sz = DW2BYTE(jptr->fpga_cntr_prev) + L2_INVAL_SIZE;
    if (sz < ccam_dma_size) {
        phys_addr_end = phys_addr_start + L2_INVAL_SIZE - 1;
        outer_inv_range(phys_addr_start, phys_addr_end);
        __cpuc_flush_dcache_area(virt_addr_start, L2_INVAL_SIZE);
    } else {
        phys_addr_end = phys_addr_start + (ccam_dma_size - DW2BYTE(jptr->fpga_cntr_prev) - 1);
        outer_inv_range(phys_addr_start, phys_addr_end);
        __cpuc_flush_dcache_area(virt_addr_start, ccam_dma_size - DW2BYTE(jptr->fpga_cntr_prev));

        phys_addr_start = circbuf_priv_ptr[jptr->chn_num].phys_addr;
        phys_addr_end = phys_addr_start + (sz - ccam_dma_size - 1);
        virt_addr_start = circbuf_priv_ptr[jptr->chn_num].buf_ptr;
        outer_inv_range(phys_addr_start, phys_addr_end);
        __cpuc_flush_dcache_area(virt_addr_start, sz - ccam_dma_size);
    }
    wake_up_interruptible(&circbuf_wait_queue); // should be done in here (after cache invalidation), not in ISR
}

/*!TODO:

implement 2 modes of controlling when to calculate histograms:
1 - add mask to 3 frame number LSB (i.e. - 0/10000000/10001000/10101010/11111111) - 3 contol bits - en/dis and mode
2 - requested in advance (i.e. by autoexposure when writing new exposure or white balance - when writing balance

mode 1 will provide easy way to display histograms (no need to repeatedly request them),
mode 2 - useful for autoexposure

Modify waiting (LSEEK_*) for histograms so they will only unfreeze if the histogram is available (skipping multiple frames))
For displaying histograms - try use latest available - not waiting fro a particular frame.


 */

// HISTOGRAMS_WAKEUP_ALWAYS if 0 will only wakeup processes waiting for histograms when they become available, maybe never if they are disabled
// if defined 1 - will wakeup each frame, regardless of the availability of the histograms
//#define HISTOGRAMS_WAKEUP_ALWAYS 0
void tasklet_cmdseq_function(unsigned long arg)
{
    int hist_en;
    int sensor_port = image_acq_priv.jpeg_ptr[arg].chn_num; // == arg & 3
    int tasklet_disable=get_globalParam(sensor_port, G_TASKLET_CTL);
    unsigned long thisFrameNumber=getThisFrameNumber(sensor_port);
#ifdef ISR_HISTOGRAMS
    unsigned long prevFrameNumber=thisFrameNumber-1;
    unsigned long * hash32p=&(aframepars[sensor_port][(thisFrameNumber-1) & PARS_FRAMES_MASK].pars[P_GTAB_R]); // same gamma for all sub-channels
    unsigned long * framep= &(aframepars[sensor_port][(thisFrameNumber-1) & PARS_FRAMES_MASK].pars[P_FRAME]);
    int subchn,hist_indx;
#endif


    // Time is out?
    if ((getThisFrameNumber(sensor_port) ^ getHardFrameNumber(sensor_port, 0))  & PARS_FRAMES_MASK) {
        dev_dbg(g_dev_ptr, "%s : port= %d, === already next frame (1)\n", __func__, sensor_port);
        return; // already next frame
    }
    // Histograms are available for the previous frame (that is already in circbuf if compressor was running)
    // Is Y histogram needed?
    PROFILE_NOW(2);
    switch ((tasklet_disable >> TASKLET_CTL_HISTY_BIT) & 7) {
    case TASKLET_HIST_NEVER:   // never calculate
        hist_en=0;
        break;
    case TASKLET_HIST_HALF:    // calculate each even (0,2,4,6 frme of 8)
        hist_en= ((thisFrameNumber & 1) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_Y));
        break;
    case TASKLET_HIST_QUATER:  // calculate twice per 8 (0, 4)
        hist_en= ((thisFrameNumber & 3) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_Y));
        break;
    case TASKLET_HIST_ONCE:    // calculate once  per 8 (0)
        hist_en= ((thisFrameNumber & 7) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_Y));
        break;
    case TASKLET_HIST_RQONLY:  // calculate only when specifically requested
        hist_en= (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_Y));
        break;
    case TASKLET_HIST_ALL:     // calculate each frame
    default:                   // calculate always (safer)
        hist_en=1;
    }

    // Actually in NC393 nothing has to be done with histograms at interrupts - do all the processing when histograms are requested
    //#ifdef TEST_DISABLE_CODE
    if (hist_en) {
#ifdef ISR_HISTOGRAMS
        histograms_check_init(); // check if histograms are initialized and initialize if not (structures and hardware)
        // after updateFramePars gammaHash are from framepars[this-1]
        for (subchn=0;subchn<MAX_SENSORS;subchn++) if (((hist_indx=get_hist_index(sensor_port,subchn)))>=0){
            if (PER_CHANNEL393) {
                set_histograms (sensor_port,
                        subchn,
                        prevFrameNumber,
                        (1 << COLOR_Y_NUMBER),
                        hash32p+hist_indx*16*sizeof (u32),
                        framep+hist_indx*32*sizeof (u32)); // 0x2 Green1
            } else {
                set_histograms (sensor_port, subchn, prevFrameNumber, (1 << COLOR_Y_NUMBER), hash32p, framep); // 0x2 Green1
            }
            GLOBALPARS(sensor_port, G_HIST_Y_FRAME + subchn) = prevFrameNumber;           // histogram corresponds to previous frame
        }
        PROFILE_NOW(3);
        // Time is out?
        // Old 353		if ((getThisFrameNumber(sensor_port) ^ X3X3_I2C_FRAME)  & PARS_FRAMES_MASK) return; // already next frame

        if ((getThisFrameNumber(sensor_port) ^ getHardFrameNumber(sensor_port, 0))  & PARS_FRAMES_MASK) {
            dev_dbg(g_dev_ptr, "%s : port= %d, === already next frame (1)\n", __func__, sensor_port);
            return; // already next frame
        }
#endif // ifdef ISR_HISTOGRAMS
#if HISTOGRAMS_WAKEUP_ALWAYS
    }
    wake_up_interruptible(&ahist_y_wait_queue[sensor_port]);    // wait queue for the G1 histogram (used as Y)
#else
    wake_up_interruptible(&ahist_y_wait_queue[sensor_port]);    // wait queue for the G1 histogram (used as Y)
    }
#endif
    // Process parameters
    if ((tasklet_disable & (1 << TASKLET_CTL_PGM))   == 0) {
        processPars (sensor_port, &asensorproc[sensor_port], getThisFrameNumber(sensor_port), get_globalParam(sensor_port, G_MAXAHEAD)); // program parameters
        PROFILE_NOW(3); // was 5 in NC353
    }
    // Time is out?
    if ((getThisFrameNumber(sensor_port) ^ getHardFrameNumber(sensor_port, 0))  & PARS_FRAMES_MASK) {
        dev_dbg(g_dev_ptr, "%s : port= %d, === already next frame (1)\n", __func__, sensor_port);
        return; // already next frame
    }
    // Are C histograms needed?
    switch ((tasklet_disable >> TASKLET_CTL_HISTC_BIT) & 7) {
    case TASKLET_HIST_NEVER:   // never calculate
        hist_en=0;
        break;
    case TASKLET_HIST_HALF:    // calculate each even (0,2,4,6 frme of 8)
        hist_en= ((thisFrameNumber & 1) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_C));
        break;
    case TASKLET_HIST_QUATER:  // calculate twice per 8 (0, 4)
        hist_en= ((thisFrameNumber & 3) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_C));
        break;
    case TASKLET_HIST_ONCE:    // calculate once  per 8 (0)
        hist_en= ((thisFrameNumber & 7) ==0) || (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_C));
        break;
    case TASKLET_HIST_RQONLY:  // calculate only when specifically requested
        hist_en= (get_imageParamsPrev(sensor_port, P_HISTRQ) & (1<<HISTRQ_BIT_C));
        break;
    case TASKLET_HIST_ALL:     // calculate each frame
    default:                   // calculate always (safer)
        hist_en=1;
    }
    if (hist_en) {
#ifdef ISR_HISTOGRAMS
        histograms_check_init(); // check if histograms are initialized and initialize if not (structures and hardware)
        // after updateFramePars gammaHash are from framepars[this-1]
        // after updateFramePars gammaHash are from framepars[this-1]
        for (subchn=0;subchn<MAX_SENSORS;subchn++) if (((hist_indx=get_hist_index(sensor_port,subchn)))>=0){
            if (PER_CHANNEL393) {
                set_histograms (sensor_port,
                        subchn,
                        prevFrameNumber,
                        0xf, // all colors
                        hash32p+hist_indx*16*sizeof (u32),
                        framep+hist_indx*32*sizeof (u32)); // 0x2 Green1
            } else {
                set_histograms (sensor_port, subchn, prevFrameNumber, 0xf, hash32p, framep); // 0x2 Green1
            }
            GLOBALPARS(sensor_port, G_HIST_C_FRAME + subchn) = prevFrameNumber;           // histogram corresponds to previous frame
        }
        PROFILE_NOW(4); // was 5 in NC353
        // Time is out?
        if ((getThisFrameNumber(sensor_port) ^ getHardFrameNumber(sensor_port, 0))  & PARS_FRAMES_MASK) {
            dev_dbg(g_dev_ptr, "%s : port= %d, === already next frame (1)\n", __func__, sensor_port);
            return; // already next frame
        }
#endif // ifdef ISR_HISTOGRAMS
#if HISTOGRAMS_WAKEUP_ALWAYS
    }
    wake_up_interruptible(&ahist_c_wait_queue[sensor_port]);     // wait queue for all the other (R,G2,B) histograms (color)
#else
        wake_up_interruptible(&ahist_c_wait_queue[sensor_port]);   // wait queue for all the other (R,G2,B) histograms (color)
    }
#endif
}

//#endif /* TEST_DISABLE_CODE */

int init_compressor_dma(int chn_mask, int reset)
{ int res;
    int status_mode = 3;
    int report_mode = 0;
    int port_afi = 0;
    dma_addr_t cmprs_sa[4];
    u32 cmprs_len[4];
    int chn;
    struct  circbuf_priv_t * cirbuf_data;
    for (chn = 0; chn <4; chn++) if (chn_mask & (1 << chn)){
        cirbuf_data = get_circbuf(chn);
        cmprs_sa[chn] =  cirbuf_data->phys_addr;
        cmprs_len[chn] = cirbuf_data->buf_size;
    }

    res = compressor_dma_setup (port_afi,
                                chn_mask,
                                reset,
                                status_mode,
                                report_mode,
                                cmprs_sa[0], cmprs_len[0],
                                cmprs_sa[1], cmprs_len[1],
                                cmprs_sa[2], cmprs_len[2],
                                cmprs_sa[3], cmprs_len[3]);
    return res;
}

/**
 * @brief resets compressor and buffer pointers
 */
void reset_compressor(unsigned int chn)
{
	unsigned long flags;

//	local_irq_save(flags);
    spin_lock_irqsave(frame_irq_locks[chn],flags);
#ifdef TEST_DISABLE_CODE
	port_csp0_addr[X313_WA_COMP_CMD]= COMPCMD_RESET; // bypasses command sequencer
	if (framepars) set_imageParamsR_all( P_COMPRESSOR_RUN, COMPRESSOR_RUN_STOP );
	else printk ("framepars is not initialized\n");
	// TODO: There still is a possibility, that there are compressor commands in the hardware que. Should we stop the hardware sequencer here (and restart it later)?
#endif /* TEST_DISABLE_CODE */
	image_acq_priv.jpeg_ptr[chn].jpeg_wp = 0;
	image_acq_priv.jpeg_ptr[chn].jpeg_rp = 0;
	image_acq_priv.jpeg_ptr[chn].fpga_cntr_prev = 0;
	image_acq_priv.jpeg_ptr[chn].flags = 0;
	//update_irq_circbuf(jptr);
//	local_irq_restore(flags);
    spin_unlock_irqrestore(frame_irq_locks[chn],flags);

}

/** Camera compressor interrupts on/off */
void compressor_interrupts (int on,   ///< 0 -interrupt disable, 1 - interrupt enable
                            int chn)  ///< compressor channel (0..3)
{
//	int i;
	x393_cmprs_interrupts_t irq_ctrl =         {.d32=0};

	//MDF2(printk ("compressor_interrupts(%d)\n",on));
	dev_dbg(g_dev_ptr, "Set compressor interrupts for channel %d: %d\n",chn, on);
#ifdef TEST_DISABLE_CODE
	if (on) {
		EN_INTERRUPT(SMART);
	}  else {
		DIS_INTERRUPTS;
	}
	// clear smart interrupt circuitry in any case
	port_csp0_addr[X313_WA_SMART_IRQ]=0x8000;

	reg_intr_vect_rw_mask intr_mask;
	intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);
	intr_mask.ext = on ? 1 : 0;
	REG_WR(intr_vect, regi_irq, rw_mask, intr_mask);
#endif /* TEST_DISABLE_CODE */

	//
	irq_ctrl.interrupt_cmd =         on ? IRQ_ENABLE : IRQ_DISABLE;
	x393_cmprs_interrupts(irq_ctrl, chn);
}

/** Camera sensor (frame sequencer) compressor interrupts on/off */
void sensor_interrupts (int on,   ///< 0 -interrupt disable, 1 - interrupt enable
                        int chn)  ///< compressor channel (0..3)
{
//    int i;
    x393_cmdframeseq_mode_t cmdframeseq_mode = {.d32=0};

    //MDF2(printk ("compressor_interrupts(%d)\n",on));
    dev_dbg(g_dev_ptr, "set sensor interrupts status: %d\n", on);
#ifdef TEST_DISABLE_CODE
    if (on) {
        EN_INTERRUPT(SMART);
    }  else {
        DIS_INTERRUPTS;
    }
    // clear smart interrupt circuitry in any case
    port_csp0_addr[X313_WA_SMART_IRQ]=0x8000;

    reg_intr_vect_rw_mask intr_mask;
    intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);
    intr_mask.ext = on ? 1 : 0;
    REG_WR(intr_vect, regi_irq, rw_mask, intr_mask);
#endif /* TEST_DISABLE_CODE */

    //
    cmdframeseq_mode.interrupt_cmd = on ? IRQ_ENABLE : IRQ_DISABLE;
    x393_cmdframeseq_ctrl(cmdframeseq_mode, chn);
}

int sequencer_stop_run_reset(int chn,  ///< Sensor port
                             int cmd)  ///< Command: SEQ_CMD_STOP=0 - stop, SEQ_CMD_RUN=1 - run, SEQ_CMD_RESET=2 - reset
                                       ///< @return always 0
{
    x393_cmdframeseq_mode_t cmdframeseq_mode = {.d32 = 0};
    x393_status_ctrl_t status_ctrl = {.d32=0};
    MDP(DBGB_SCRST,chn,"cmd = %d\n",cmd)
    switch (cmd){
    case SEQ_CMD_STOP:
        cmdframeseq_mode.run_cmd = 2;
        break;
    case SEQ_CMD_RUN:
        cmdframeseq_mode.run_cmd = 3;
        status_ctrl.mode = 3; // autoupdate, is anyway set to it when reading i2c
        break;
    case SEQ_CMD_RESET:
        cmdframeseq_mode.reset =   1;
    }
    if (cmdframeseq_mode.d32)
        x393_cmdframeseq_ctrl (cmdframeseq_mode, chn);
    if (cmd == SEQ_CMD_RESET)
        udelay(1);
    if (status_ctrl.mode)
        set_x393_cmdseqmux_status_ctrl(status_ctrl);
    // debug
    udelay(1);
    MDP(DBGB_SCRST,chn,"status_ctrl.d32 = 0x%x\n",status_ctrl.d32)
    return 0;
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
//	int res;
	unsigned int irq;
	struct device *dev = &pdev->dev;
//	const struct of_device_id *match;
	const char *frame_sync_irq_names[4] = {"frame_sync_irq_0", "frame_sync_irq_1",
			"frame_sync_irq_2", "frame_sync_irq_3"};
	const char *compressor_irq_names[4] = {"compr_irq_0", "compr_irq_1",
			"compr_irq_2", "compr_irq_3"};

	/* sanity check */
	/*match = of_match_device(elphel393_sensor_of_match, dev);
	if (!match)
		return -EINVAL;*/

//	asensorproc= &as_sensorproc[0];
    asensorproc= as_sensorproc;
	//MDD1(printk("sensorproc=0x%x\n",(int) sensorproc));
//	dev_dbg(dev, "sensorproc addresses: 0x%x\n", (int) sensorproc);

	for (i = 0; i < SENSOR_PORTS; i++) {
	    dev_dbg(dev, "sensorproc addresses: 0x%x\n", (int) &asensorproc[i]);
		irq = platform_get_irq_byname(pdev, frame_sync_irq_names[i]);
		if (request_irq(irq,
				frame_sync_irq_handler,
				0, // no flags
				frame_sync_irq_names[i],
				&image_acq_priv.jpeg_ptr[i])) {
			dev_err(dev, "can not allocate Elphel FPGA interrupts\n");
			return -EBUSY;
		}
		image_acq_priv.jpeg_ptr[i].irq_num_sens = irq;
        dev_info(dev, "Elphel FPGA interrupts set: %s\n",frame_sync_irq_names[i]);

        irq = platform_get_irq_byname(pdev, compressor_irq_names[i]);
		if (request_irq(irq,
				compressor_irq_handler,
				0, // no flags
				compressor_irq_names[i],
				&image_acq_priv.jpeg_ptr[i])) {
			dev_err(dev, "can not allocate Elphel FPGA interrupts\n");
			return -EBUSY;
		}
		image_acq_priv.jpeg_ptr[i].irq_num_comp = irq;
		image_acq_priv.jpeg_ptr[i].chn_num = i;
        dev_info(dev, "Elphel FPGA interrupts set: %s\n",compressor_irq_names[i]);

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

	dev_info(dev, "Elphel FPGA interrupts initialized\n");
	dev_dbg(dev, "reset all compressors\n");
	for (i = 0; i < SENSOR_PORTS; i++) {
		reset_compressor(i);
	    dev_info(dev, "init_pgm_proc (%d)\n",i);
	    init_pgm_proc (i);   // setup pointers to functions (not sensor-specific)
	}
	//reset_compressor(); // reset compressor and buffer pointers
	//MDD1(printk("x313_dma_init()\n"));
	//x313_dma_init();    // initialize ETRAX FS DMA
	dev_info(dev, "reset_qtables(0) (policy = COMMON_CACHE)\n");
	set_cache_policy(COMMON_CACHE);
    reset_qtables(0); // force initialization at next access (with COMMON_CACHE chyannel is ignored, with PER_CHN_CACHE - do for each chennel)
	pgm_functions_set_device(dev);
	g_dev_ptr = dev;
	return 0;
}

int image_acq_stop(struct platform_device *pdev)
{
	return 0;
}
//#define I2C359_INC                    2   ///< slave address increment between sensors in 10359A board (broadcast, 1,2,3)
/** Register i2c pages equal to slave address,
 * Use to convert 353 code  */
int legacy_i2c(int ports) ///< bitmask of the sensor ports to use
                          ///< @return 0 (may add errors)
{
    int sensor_port, subchn;
    x393_i2c_device_t *class_10359, *class_sensor, dev_sensor;
    class_10359 = xi2c_dev_get(name_10359);
    BUG_ON(!class_10359);
    class_sensor= xi2c_dev_get(name_sensor);
    BUG_ON(!class_sensor);
    memcpy(&dev_sensor, class_sensor, sizeof(x393_i2c_device_t));
    dev_dbg(g_dev_ptr, "Initializing sensor i2c for legacy commands, ports bit-mask= 0x%x\n",ports);
    for (sensor_port=0; sensor_port< SENSOR_PORTS; sensor_port++) if (ports & (1 << sensor_port)) {
        i2c_page_alloc_init(sensor_port); // reset all pages allocation
        i2c_page_register(sensor_port, class_10359->slave7);
        set_xi2c_wrc(class_10359, sensor_port, class_10359->slave7, 0);
        for (subchn = 0; subchn <4; subchn++){ // subchn == 0 - broadcast
            dev_sensor.slave7 = class_sensor->slave7 + I2C359_INC * subchn;
            i2c_page_register(sensor_port, dev_sensor.slave7);
            set_xi2c_wrc(&dev_sensor, sensor_port, dev_sensor.slave7, 0);
        }
        // Now register one page for reading 10359 and the sensor using sensor speed data
        memcpy(&dev_sensor, class_sensor, sizeof(x393_i2c_device_t)); // dev_sensor));
        i2c_page_register(sensor_port, LEGACY_READ_PAGE2);
        set_xi2c_rdc(&dev_sensor, sensor_port, LEGACY_READ_PAGE2);
        i2c_page_register(sensor_port, LEGACY_READ_PAGE4);
        dev_sensor.data_bytes=4; // for reading 10359 in 32-bit mode
        set_xi2c_rdc(&dev_sensor, sensor_port, LEGACY_READ_PAGE4);
        dev_dbg(g_dev_ptr, "Initialized sensor i2c for legacy commands, sensor_port= 0x%x\n",sensor_port);
    }
    return 0;
/*
 /home/eyesis/git/elphel393/poky/build/tmp/work-shared/elphel393/kernel-source/drivers/elphel/sensor_common.c: In function 'legacy_i2c':
/home/eyesis/git/elphel393/poky/build/tmp/work-shared/elphel393/kernel-source/drivers/elphel/sensor_common.c:1060:45:
  warning: argument to 'sizeof' in 'memcpy' call is the same pointer type 'x393_i2c_device_t * {aka struct <anonymous> *}'
    as the destination; expected 'x393_i2c_device_t {aka struct <anonymous>}' or an explicit length [-Wsizeof-pointer-memaccess]
     memcpy(&dev_sensor, class_sensor, sizeof(class_sensor));
invalid type argument of unary '*' (have 'x393_i2c_device_t {aka struct <anonymous>}')
 */
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
//MODULE_DESCRIPTION(IMAGEACQ_DRIVER_DESCRIPTION);
