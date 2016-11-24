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

#include <uapi/elphel/ahci_cmd.h>
#include <uapi/elphel/c313a.h>
#include "../elphel/circbuf.h"
#include "../elphel/x393_fpga_functions.h"

#ifndef _AHCI_ELPHEL_EXT
#define _AHCI_ELPHEL_EXT


#define IRQ_SIMPLE                (1 << 0)       ///< Flag indicating that IRQ corresponds to internal command and should not be
                                                 ///< processed in ahci_handle_port_interrupt
#define DISK_BUSY                 (1 << 1)       ///< Flag indicating that disk is currently busy. Access to this flag should be protected by
                                                 ///< spin locks to prevent race conditions
#define PROC_CMD                  (1 << 2)       ///< Processing driver's internal command is in progress
#define LAST_BLOCK                (1 << 3)       ///< Flag indicating that the remaining chunk of data will be recorded
#define DELAYED_FINISH            (1 << 4)       ///< Flag indicating that recording should be stopped right after the last chunk of data is written
#define LOCK_TAIL                 (1 << 5)       ///< Lock current command slot until all data buffers are assigned and the frame is aligned
#define CMD_FIS_LEN               5              ///< The length of a command FIS in double words
#define ADDR_MASK_28_BIT          ((u64)0xfffffff)///< This is used to get 28-bit address from 64-bit value
#define MAX_PRDT_LEN              0x3fffff       ///< A maximum of length of 4MB may exist for PRDT entry
#define MAX_DATA_CHUNKS           9              ///< An array or JPEG frame chunks contains pointers to JPEG leading marker,
                                                 ///< JPEG header, Exif data if present, stuffing bytes chunk which aligns
                                                 ///< the frame size to disk sector boundary, JPEG data which
                                                 ///< can be split into two chunks, align buffers, JPEG
                                                 ///< trailing marker, and pointer to a buffer containing the remainder of a
                                                 ///< frame. Nine chunks of data in total.
#define DEFAULT_PORT_NUM          0              ///< Default port number
#define ALIGNMENT_SIZE            32             ///< Align buffers length to this amount of bytes
#define MAX_SGL_LEN               168            ///< Maximum number of entries in PRDT table. HW max is 64k.
                                                 ///< Set this value the same as AHCI_MAX_SG in ahci.h
#define MAX_CMD_SLOTS             4              ///< Maximum number of frames which will be processed at the same time
#define MAX_LBA_COUNT             0xff           ///< Maximum number of sectors for READ DMA or WRITE DMA commands
#define MAX_LBA_COUNT_EXT         0xffff         ///< Maximum number of sectors for READ DMA EXT or WRITE_DMA EXT commands
#define PHY_BLOCK_SIZE            512            ///< Physical disk block size
#define JPEG_MARKER_LEN           2              ///< The size in bytes of JPEG marker
#define JPEG_SIZE_LEN             2              ///< The size in bytes of JPEG marker length field
#define INCLUDE_REM               1              ///< Include REM buffer to total size calculation
#define EXCLUDE_REM               0              ///< Exclude REM buffer from total size calculation
#define SPEED_SAMPLES_NUM         5              ///< Maximum number of samples for disk recording speed measurement

/** This structure is for collecting some recording statistics */
struct rec_stat {
	unsigned int samples_ptr;                    ///< pointer to next sample in rec_stat::samples
	unsigned int samples[SPEED_SAMPLES_NUM];     ///< calculated recording speed samples, the value of recording speed
	                                             ///< presented via sysfs is a median of this array
	sec_usec_t start_time;                       ///< time when current command has been issued
};

/** This structure holds raw device buffer pointers */
struct drv_pointers {
	uint64_t lba_start;                          ///< raw buffer starting LBA
	uint64_t lba_end;                            ///< raw buffer ending LBA
	uint64_t lba_write;                          ///< current write pointer inside raw buffer
	uint16_t wr_count;                           ///< the number of LBA to write next time
};

/** Container structure for frame buffers */
struct frame_buffers {
	struct fvec exif_buff;                       ///< Exif buffer
	struct fvec jpheader_buff;                   ///< JPEG header buffer
	struct fvec trailer_buff;                    ///< buffer for trailing marker
	struct fvec common_buff;                     ///< common buffer where other parts are combined
	struct fvec rem_buff;                        ///< remainder from previous frame
};

/** Symbolic names for slots in buffer pointers. Buffer alignment function relies on the order of these names, so
 * new names can be added but the overall order should not be changed */
enum {
	CHUNK_LEADER,                                ///< pointer to JPEG leading marker
	CHUNK_EXIF,                                  ///< pointer to Exif buffer
	CHUNK_HEADER,                                ///< pointer to JPEG header data excluding leading marker
	CHUNK_COMMON,                                ///< pointer to common buffer
	CHUNK_DATA_0,                                ///< pointer to JPEG data
	CHUNK_DATA_1,                                ///< pointer to the second half of JPEG data if a frame crosses circbuf boundary
	CHUNK_TRAILER,                               ///< pointer to JPEG trailing marker
	CHUNK_ALIGN,                                 ///< pointer to buffer where the second part of JPEG data should be aligned
	CHUNK_REM                                    ///< pointer to buffer containing the remainder of current frame. It will be recorded during next transaction
};

/** AHCI driver private structure */
struct elphel_ahci_priv {
	u32 clb_offs;                                ///< CLB offset, received from device tree
	u32 fb_offs;                                 ///< FB offset, received from device tree
	u32 base_addr;                               ///< controller base address
	u32 flags;                                   ///< flags indicating current state of the driver. Access to #DISK_BUSY flags is protected with
	                                             ///< a spin lock
	int curr_cmd;                                ///< current ATA command
	size_t max_data_sz;                          ///< maximum data size (in bytes) which can be processed with current ATA command
	struct drv_pointers lba_ptr;                 ///< disk buffer pointers
	struct frame_buffers fbuffs[MAX_CMD_SLOTS];  ///< a set of buffers for each command
	struct fvec data_chunks[MAX_CMD_SLOTS][MAX_DATA_CHUNKS];///< a set of vectors pointing to data buffers for each command
	struct fvec sgl[MAX_SGL_LEN];                ///< an array of data buffers mapped for next transaction
	int sg_elems;                                ///< the number of S/G vectors mapped for next transaction in @e sgl array
	int curr_data_chunk;                         ///< index of a data chunk used during last transaction
	size_t curr_data_offset;                     ///< offset of the last byte in a data chunk pointed to by @e curr_data_chunk
	size_t head_ptr;                             ///< pointer to command slot which will be written next
	size_t tail_ptr;                             ///< pointer to next free command slot
	spinlock_t flags_lock;                       ///< controls access to #DISK_BUSY flag in @e flags variable.
	                                             ///< This flag controls access to disk write operations either from
	                                             ///< the the driver itself or from the system.  Mutex is not used
	                                             ///< because this flag is accessed from interrupt context
	struct tasklet_struct bh;                    ///< command processing tasklet
	struct device *dev;                          ///< pointer to parent device structure
	struct rec_stat stat;                        ///< recording statistics
};

#endif /* _AHCI_ELPHEL_EXT */
