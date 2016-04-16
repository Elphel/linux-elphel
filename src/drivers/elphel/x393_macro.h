/**
 * @file x393_macro.h
 * @brief This file contains various macros used in multiple files.
 */
#ifndef _X393_MACRO
#define _X393_MACRO

#include <elphel/driver_numbers.h>

/** @brief Number of image channels */
#define IMAGE_CHN_NUM        4

/** @brief Resolution of current/OEF pointer in bits */
#define OFFSET256_CNTR_RES   26

#define CHUNK_SIZE           32

/** @brief The size of #interframe_params_t structure in double words */
#define INTERFRAME_PARAMS_SZ 8

#define MARKER_FF            0xff000000

#define MARKER_FFFF          0xffff

#define FRAME_LENGTH_MASK    0xffffff

#define IRQ_NOP              0
#define IRQ_CLEAR            1
#define IRQ_DISABLE          2
#define IRQ_ENABLE           3

#define BYTE2DW(x)           ((x) >> 2)
#define DW2BYTE(x)           ((x) << 2)

// 4 bytes offset, this one comes from python code x393_cmprs_afi.py
#define ADJUSTMENT           4
#define INSERTED_BYTES(x)    (((CHUNK_SIZE - ((((x) % CHUNK_SIZE) + CCAM_MMAP_META) % CHUNK_SIZE) - ADJUSTMENT) % CHUNK_SIZE ) + ADJUSTMENT)

/* These macro were removed from sensor_common.h*/
#define X313_LENGTH_MASK      0xff000000
#define X393_BUFFSUB(x, y) (((x) >= (y)) ? ((x)-(y)) : ((x) + (CCAM_DMA_SIZE -(y))))
#define X393_BUFFADD(x, y) ((((x) + (y)) <= CCAM_DMA_SIZE) ? ((x) + (y)) : ((x) - (CCAM_DMA_SIZE -(y))))

/**
 * @brief Converts file minor number to image compressor channel.
 *
 * This function assumes that the least significant nibble of minor number contains image compressor channel number and
 * next nibble contains device type. Channel numbers and device type are defined in #driver_numbers.h
 * @param[in]   minor     file minor number
 * @param[out]  dev_type  pointer to a variable which will hold device type or NULL if this value is not needed
 * @return      compressor channel number in the range [0..#IMAGE_CHN_NUM)
 */
static inline unsigned int minor_to_chn(unsigned int minor, unsigned int *dev_type)
{
	if (dev_type != NULL) {
		if ((minor & 0xf0) == CIRCBUF_MINOR || (minor & 0xf0) == HUFFMAN_MINOR || (minor & 0xf0) == JPEGHEAD_MINOR)
			*dev_type = minor & 0xf0;
		else
			*dev_type = 0;
	}
	if ((minor & 0x0f) < IMAGE_CHN_NUM)
		return minor & 0x0f;
	else
		return 0;
}

#endif /* _X393_MACRO */
