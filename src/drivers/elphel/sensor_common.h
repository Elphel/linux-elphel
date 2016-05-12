#ifndef _SENSOR_COMMON_H
#define _SENSOR_COMMON_H

//extern struct sensor_t sensor; // current sensor (will be copied to by sensor driver), made external for the cc353.c to read/write i2c
extern struct sensorproc_t * sensorproc;
/// IRQ-safe "nice" FPGA table write and histogram read functions - they split the data in chunks of fixed size,
/// disable IRQ, transfer a chunk, then reenable interrupt before proceedg to the next chunk
#define FPGA_TABLE_CHUNK 64 // up to 64 words to send to the table/from histogram on a single IRQ-off transfer
//void fpga_table_write_nice (int addr, int len, unsigned long * data);
//void fpga_hist_read_nice (int addr, int len, unsigned long * data);
#ifdef CONFIG_ETRAX_ELPHEL_MT9X001
  #include "mt9x001.h"
#endif
//#include "multisensor.h"
//int  camSeqGetJPEG_wp(void);
//int  camSeqGetJPEG_rp(void);
//void camSeqSetJPEG_rp(int p);
int camseq_get_jpeg_wp(unsigned int chn);
int camseq_get_jpeg_rp(unsigned int chn);
void camseq_set_jpeg_rp(unsigned int chn, int ptr);

///CIRCBUF macros
extern unsigned long  * ccam_dma_buf_ptr;

/* move these lines to x313_macro.h
#define X313_LENGTH_MASK      0xff000000
#define X313_PADDED_FRAME(x)((((x)+67+CCAM_MMAP_META ) >>2) & 0xfffffff8)
#define X313_BUFFSUB(x,y) (((x)>=(y))? ((x)-(y)) : ((x)+ (CCAM_DMA_SIZE-(y))))
#define X313_BUFFADD(x,y) ((((x) + (y))<=CCAM_DMA_SIZE)? ((x) + (y)) : ((x) - (CCAM_DMA_SIZE-(y))))
*/

//int init_FPGA(void); /// can be initialized only after FPGA is configured, not at module init (NOTE was static??)
///can be verified with if (!X313_IS_SDRAM_ON)
void reset_compressor(unsigned int chn);
void camera_interrupts (int on);
struct sensorproc_t * copy_sensorproc (struct sensorproc_t * copy);

///NOTE: If profiling is enabled (TASKLET_CTL_ENPROF is set in G_TASKLET_CTL) - save current time in 2 of the 32-bit locations that can be read as pastpars (i.e. from PHP)
#ifdef TEST_DISABLE_CODE
	#define PROFILE_NOW(x) if (get_globalParam(G_TASKLET_CTL) & (1 << TASKLET_CTL_ENPROF)) \
							   X313_GET_FPGA_TIME((pastpars[getThisFrameNumber() & PASTPARS_SAVE_ENTRIES_MASK].past_pars[(PP_PROFILE_START + 0) + ((x)<<1)]),\
												  (pastpars[getThisFrameNumber() & PASTPARS_SAVE_ENTRIES_MASK].past_pars[(PP_PROFILE_START + 1) + ((x)<<1)]))
	///Put this to the next frame's data (before thisFrameNumber was incremented)
	#define PROFILE_NEXT(x) if (get_globalParam(G_TASKLET_CTL) & (1 << TASKLET_CTL_ENPROF)) \
							   X313_GET_FPGA_TIME((pastpars[(getThisFrameNumber()+1) & PASTPARS_SAVE_ENTRIES_MASK].past_pars[(PP_PROFILE_START + 0) + ((x)<<1)]),\
												  (pastpars[(getThisFrameNumber()+1) & PASTPARS_SAVE_ENTRIES_MASK].past_pars[(PP_PROFILE_START + 1) + ((x)<<1)]))
#else
	#define PROFILE_NOW(x) {}
	#define PROFILE_NEXT(x) {}
#endif

int image_acq_init(struct platform_device *pdev);

// indicate that this channel need attention; set in interrupt handler, reset in bottom half
#define SENS_FLAG_IRQ             0x01
// got 0x20 more than start of the new image
#define OFFSET_X40      0x40

/* debug code follows */
long long get_zero_counter(unsigned int chn);
long long get_corrected_offset(unsigned int chn);
long long get_frame_counter(unsigned int chn);
long long get_frame_pos(unsigned int chn, unsigned int pos);
/* end of debug code */

#endif
