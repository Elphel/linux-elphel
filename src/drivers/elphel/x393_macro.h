/**
 * @file x393_macro.h
 * @brief This file contains various macros used in multiple files.
 * @copyright Copyright (C) 2016 Elphel, Inc
 *
 * @par <b>License</b>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _X393_MACRO
#define _X393_MACRO

#include <uapi/elphel/x393_devices.h>
#include "circbuf.h" // for circbuf_priv (or disable X393_BUFSUB_CHN, X393_BUFADD_CHN if _CIRCBUF_H is not defined?


/** @brief Resolution of current/OEF pointer in bits */
#define OFFSET256_CNTR_RES   26
/** @brief The size of data transfer in bytes */
#define CHUNK_SIZE           32

/** @brief The size of #interframe_params_t structure in double words */
#define INTERFRAME_PARAMS_SZ 8
/** @brief Marker field in frame length double word value */
#define MARKER_FF            0xff000000
/** @brief #interframe_params_t has a guard field and its value must be equal to MARKER_FFFF, otherwise
 * it has been overwritten, as JPEG can not have two subsequent 0xFF */
#define MARKER_FFFF          0xffff
/** @brief Mask for frame length extraction from double word value */
#define FRAME_LENGTH_MASK    0xffffff

/** @brief No operations with compressor interrupt control register */
#define IRQ_NOP              0
/** @brief Clear compressor interrupt status */
#define IRQ_CLEAR            1
/** @brief Disable compressor interrupt */
#define IRQ_DISABLE          2
/** @brief Enable compressor interrupt */
#define IRQ_ENABLE           3
/** @brief Convert size in bytes to size in double words */
#define BYTE2DW(x)           ((x) >> 2)
/** @brief Convert size in double words to size in bytes */
#define DW2BYTE(x)           ((x) << 2)
/** @brief 4 bytes offset, this one comes from Python code x393_cmprs_afi.py */
#define ADJUSTMENT           4
#define INSERTED_BYTES(x)    (((CHUNK_SIZE - ((((x) % CHUNK_SIZE) + CCAM_MMAP_META) % CHUNK_SIZE) - ADJUSTMENT) % CHUNK_SIZE ) + ADJUSTMENT)

#define X313_LENGTH_MASK      0xff000000
/** @brief Subtract two offsets considering that the resulting offset can roll over the start of circular buffer */
//#define X393__BUFFSUB(x, y) (((x) >= (y)) ? ((x)-(y)) : ((x) + (CCAM__DMA_SIZE -(y))))
#define X393_BUFFSUB_CHN(x, y, chn) (((x) >= (y)) ? ((x)-(y)) : ((x) + (circbuf_priv_ptr[chn].buf_size -(y))))
#define X393_BUFFSUB32(x, y, chn) (((x) >= (y)) ? ((x)-(y)) : ((x) + (circbuf_priv_ptr[chn].buf_size32 -(y))))


/** @brief Add two offsets considering that the resulting offset can roll over the end of circular buffer */
//#define X393__BUFFADD(x, y) ((((x) + (y)) <= CCAM__DMA_SIZE) ? ((x) + (y)) : ((x) - (CCAM__DMA_SIZE -(y))))
#define X393_BUFFADD_CHN(x, y, chn) ((((x) + (y)) <= circbuf_priv_ptr[chn].buf_size) ? ((x) + (y)) : ((x) - (circbuf_priv_ptr[chn].buf_size -(y))))
#define X393_BUFFADD32(x, y, chn)   ((((x) + (y)) <= circbuf_priv_ptr[chn].buf_size32) ? ((x) + (y)) : ((x) - (circbuf_priv_ptr[chn].buf_size32 -(y))))

#if 0
#define TABLE_TYPE_QUANT     0
#define TABLE_TYPE_CORING    1
#define TABLE_TYPE_FOCUS     2
#define TABLE_TYPE_HUFFMAN   3
#endif

/**
 * @brief Converts file minor number to image compressor channel.
 *
 * This function assumes that the least significant nibble of minor number contains image compressor channel number and
 * next nibble contains device type. Channel numbers and device type are defined in #driver_numbers.h
 * @param[in]   minor     file minor number
 * @param[out]  dev_type  pointer to a variable which will hold device type or NULL if this value is not needed
 * @return      compressor channel number in the range [0..#SENSOR_PORTS)
 */
static inline unsigned int minor_to_chn(unsigned int minor, unsigned int *dev_type)
{
	if (dev_type != NULL) {
		if ((minor & 0xf0) == DEV393_MINOR(DEV393_CIRCBUF0) || (minor & 0xf0) == DEV393_MINOR(DEV393_HUFFMAN0) || (minor & 0xf0) == DEV393_MINOR(DEV393_JPEGHEAD0))
			*dev_type = minor & 0xf0;
		else
			*dev_type = 0;
	}
	if ((minor & 0x0f) < SENSOR_PORTS)
		return minor & 0x0f;
	else
		return 0;
}

#endif /* _X393_MACRO */
