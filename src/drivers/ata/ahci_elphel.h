/** @file ahci_elphel_ext.h
 *
 * @brief Elphel AHCI SATA platform driver for Elphel393 camera. This module provides
 * additional functions which allows to use a part of a disk (or entire disk) as a
 * raw circular buffer.
 *
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

#include "../elphel/circbuf.h"

#ifndef _AHCI_ELPHEL_EXT
#define _AHCI_ELPHEL_EXT

/** Flag indicating that IRQ should not be processed in ahci_port_interrupt */
#define IRQ_SIMPLE                (1 << 0)
/** Processing command in progress */
#define PROC_CMD                  (1 << 1)
/** Flag indicating that the remaining chunk of data will be recorder */
#define LAST_BLOCK                (1 << 2)
/** Flag indicating that recording should be stopped right after the last chunk of data
 * is written */
#define DELAYED_FINISH            (1 << 3)
/** The length of a command FIS in double words */
#define CMD_FIS_LEN               5
/** This is used to get 28-bit address from 64-bit value */
#define ADDR_MASK_28_BIT          ((u64)0xfffffff)
/** A maximum of length of 4MB may exist for PRDT entry */
#define MAX_PRDT_LEN              0x3fffff
/** An array or JPEG frame chunks contains pointers to JPEG leading marker,
 * JPEG header, Exif data if present, stuffing bytes chunk which aligns
 * the frame size to disk sector boundary, JPEG data which
 * can be split into two chunks, their corresponding align buffers, JPEG
 * trailing marker, and pointer to a buffer containing the remainder of a
 * frame. Ten chunks of data in total.
 * @todo Fix description */
#define MAX_DATA_CHUNKS           9
/** Default port number */
#define DEFAULT_PORT_NUM          0
/** Align buffers length to this amount of bytes */
#define ALIGNMENT_SIZE            32
/** Maximum number of entries in PRDT table. HW max is 64k.
 * Set this value the same as AHCI_MAX_SG in ahci.h */
#define MAX_SGL_LEN               168

/** This structure holds raw device buffer pointers */
struct drv_pointers {
	uint64_t lba_start;                          ///< raw buffer starting LBA
	uint64_t lba_end;                            ///< raw buffer ending LBA
	uint64_t lba_write;                          ///< current write pointer inside raw buffer
	uint16_t wr_count;                           ///< the number of LBA to write next time
};

//struct fvec {
//	void *iov_base;                             ///< pointer to allocated buffer
//	size_t iov_len;                             ///< the size (in bytes) of allocated buffer; set after allocation and is not modified during buffer lifetime
//	dma_addr_t iov_dma;                         ///< buffer physical address
//};

//struct frame_vects {
//	struct fvec prev_chunk;                      ///< remainder chunk of data from previous frame (NOT USED NOW)
//	struct fvec header_buff;                     ///< buffer for JPEG header
//	struct fvec leader;                          ///< JPEG leading marker, pointer to header_buff
//	struct fvec exif;                            ///< pointer to Exif data buffer
//	struct fvec header;                          ///< JPEG header data without leading marker, pointer to header_buff
//	struct fvec stuffing;                        ///< stuffing bytes for frame alignment (WILL BE REMOVED AFTER SPEED TEST)
//	struct kvec jpeg[2];                         ///< pointers to JPEG frame data which can be split across circular buffer boundary
//	struct fvec trailer;                         ///< JPEG trailing marker
//};

struct frame_buffers {
	struct fvec exif_buff;
	struct fvec jpheader_buff;
	struct fvec trailer_buff;
	struct fvec common_buff;
	struct fvec rem_buff;                        ///< remainder from previous frame
};

enum {
	CHUNK_LEADER,
	CHUNK_EXIF,
	CHUNK_HEADER,
	CHUNK_COMMON,
	CHUNK_DATA_0,
	CHUNK_DATA_1,
	CHUNK_TRAILER,
	CHUNK_ALIGN,
	CHUNK_REM
};

struct elphel_ahci_priv {
	u32 clb_offs;
	u32 fb_offs;
	u32 base_addr;
	u32 flags;
	int curr_cmd;
	size_t max_data_sz;
	struct drv_pointers lba_ptr;
	struct frame_buffers fbuffs;
	struct fvec data_chunks[MAX_DATA_CHUNKS];
	struct fvec sgl[MAX_SGL_LEN];
	int sg_elems;
	int curr_data_chunk;                         ///< index of a data chunk used during last transaction
	size_t curr_data_offset;                     ///< offset of the last byte in a data chunk pointed to by @e curr_data_chunk
};

#endif /* _AHCI_ELPHEL_EXT */
