/** @file circbuf.c
 *
 * @brief Drivers to manipulate large circular buffer that holds compressed
 * images/video. Buffer frame data is filled in by the FPGA, frame pointers and
 * essential frames metadata filled during servicing of the interrupts.
 *
 * Copyright (C) 2016 Elphel, Inc
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
 * -----------------------------------------------------------------------------**
 */

#define DEBUG 1
#include <linux/device.h>

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
//#include <linux/string.h>
#include <linux/init.h>
//#include <linux/time.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

//#include <asm/system.h>
//#include <asm/arch/memmap.h>
//#include <asm/svinto.h> obsolete
//#include <asm/io.h>

/*#include <asm/arch/dma.h>
#include <asm/arch/hwregs/dma_defs.h>
#include <asm/arch/hwregs/dma.h>
#include <asm/arch/hwregs/reg_map.h>
#include <asm/arch/hwregs/bif_dma_defs.h>
*/

//#include <asm/irq.h>
//#include <asm/atomic.h>


//#include <asm/delay.h>
#include <asm/uaccess.h>
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
#include <elphel/elphel393-mem.h>

#include "framepars.h" // just for ELPHEL_DEBUG bit mask

#include "sensor_common.h"
#include "jpeghead.h"
#include "circbuf.h"
#include "exif.h"
#include "x393_macro.h"
#include "x393.h"

#define CIRCBUF_DRIVER_NAME "circbuf driver"

/** Wait queue for the processes waiting for a new frame to appear in the circular buffer */
wait_queue_head_t circbuf_wait_queue;

struct circbuf_priv_t circbuf_priv[SENSOR_PORTS];
struct circbuf_priv_t *circbuf_priv_ptr = circbuf_priv;

static struct device *g_dev_ptr;

static const struct of_device_id elphel393_circbuf_of_match[];

int init_ccam_dma_buf_ptr(struct platform_device *pdev)
{
	int i;
	dma_addr_t dma_handle;
	const size_t dma_size = (CCAM_DMA_SIZE + (PAGE_SIZE >> 2)) * sizeof(int);
	struct device *dev = &pdev->dev;
	unsigned long *ccam_dma_buf_ptr = NULL;

	// use Elphel_buf if it was allocated
	if (pElphel_buf != NULL) {
		ccam_dma_buf_ptr = pElphel_buf->vaddr;
		dma_handle = pElphel_buf->paddr;
		dev_info(dev, "using %lu bytes of DMA memory from pElphel_buf at address 0x%08x", pElphel_buf->size * PAGE_SIZE, dma_handle);
	} else {
		ccam_dma_buf_ptr = dmam_alloc_coherent(dev, dma_size, &dma_handle, GFP_KERNEL);
		if (!ccam_dma_buf_ptr) {
			dev_err(dev, "unable to allocate DMA buffer\n");
			return -ENOMEM;
		} else {
			dev_info(dev, "%d bytes of DMA memory allocated at address 0x%08x", dma_size , dma_handle);
		}
	}

	// set circular buffer size in bytes
	set_globalParam(G_CIRCBUFSIZE, CCAM_DMA_SIZE);

	for (i = 0; i < SENSOR_PORTS; i++) {
		circbuf_priv[i].buf_ptr = ccam_dma_buf_ptr + BYTE2DW(CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE);
		circbuf_priv[i].phys_addr = dma_handle + CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE;
	}

	return 0;
}

int circbuf_all_open(struct inode *inode, struct file *filp)
{
	int res;
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "circbuf_all_open, minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		res = circbuf_open(inode, filp);
		break;
	case JPEGHEAD_MINOR:
		res = jpeghead_open(inode, filp);
		break;
	case HUFFMAN_MINOR:
		res = huffman_open(inode, filp);
		break;
	default:
		return -EINVAL;
	}
	return res;
}

int circbuf_all_release(struct inode *inode, struct file *filp)
{
	int res=0;
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		//        res=circbuf_release(inode,filp);
		break;
	case JPEGHEAD_MINOR:
		//        res=jpeghead_release(inode,filp);
		break;
	case HUFFMAN_MINOR:
		//        res=huffman_release(inode,filp);
		break;
	default:
		return -EINVAL;
	}
	if (filp->private_data) kfree(filp->private_data);
	return res;
}

loff_t circbuf_all_lseek(struct file *file, loff_t offset, int orig)
{
	int rp;
	struct interframe_params_t *fp = NULL;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	unsigned int chn = minor_to_chn(minor, &dev_type);
	dev_dbg(g_dev_ptr, "circbuf_all_lseek, minor = 0x%x\n", minor);

	switch (dev_type) {
	case CIRCBUF_MINOR:
		return  circbuf_lseek(file, offset, orig);
	case JPEGHEAD_MINOR:
		if (orig == SEEK_END && offset > 0) {
			rp = BYTE2DW(offset) & (~7); // convert to index to long, align to 32-bytes
			fp = (struct interframe_params_t *) &circbuf_priv[chn].buf_ptr[X393_BUFFSUB(rp, 8)];
		}
		return  jpeghead_lseek(file, offset, orig, fp);
	case HUFFMAN_MINOR:
		return  huffman_lseek(file, offset, orig);
	default:
		return -EINVAL;
	}
}

ssize_t circbuf_all_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		return circbuf_read(file, buf, count, off);
	case JPEGHEAD_MINOR:
		return jpeghead_read(file, buf, count, off);
	case HUFFMAN_MINOR:
		return huffman_read(file, buf, count, off);
	default:
		return -EINVAL;
	}
}

ssize_t circbuf_all_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x, count = %d, off = %d\n", minor, (int)count, (int)*off);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		return circbuf_write (file, buf, count, off);
	case JPEGHEAD_MINOR:
		// no method for this module
		return -EINVAL;
	case HUFFMAN_MINOR:
		return huffman_write (file, buf, count, off);
	default:
		return -EINVAL;
	}
}

int circbuf_all_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		return circbuf_mmap(file, vma);
	default:
		return -EINVAL;
	}
}

unsigned int circbuf_all_poll (struct file *file, poll_table *wait)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		return circbuf_poll(file, wait);
	default:
		return -EINVAL;
	}
}

int circbuf_open(struct inode *inode, struct file *filp)
{
	inode->i_size = CCAM_DMA_SIZE;
	dev_dbg(g_dev_ptr, "inode->i_size = 0x%lx\n", inode->i_size);

	return 0;
}

void dump_interframe_params(struct interframe_params_t *params, int offset)
{
	dev_dbg(g_dev_ptr, "Dump of interframe parameters at offset 0x%x:\n", offset);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, params, sizeof(struct interframe_params_t));
}

/**
 * @brief
 */
unsigned long get_image_length(int byte_offset, unsigned int chn, int *last_chunk_offset)
{
	unsigned long len32;
	int last_image_chunk = byte_offset - OFFSET_X40;

	if (last_image_chunk < 0)
		last_image_chunk += CCAM_DMA_SIZE;
	len32 = circbuf_priv[chn].buf_ptr[BYTE2DW(last_image_chunk + (CHUNK_SIZE - CCAM_MMAP_META_LENGTH))];

	dev_dbg(g_dev_ptr, "got len32 = 0x%lx at 0x%x\n", len32, last_image_chunk + (CHUNK_SIZE - CCAM_MMAP_META_LENGTH));

	if (last_chunk_offset != NULL)
		*last_chunk_offset = last_image_chunk;

	return len32;
}

/**
 * @brief Check that read pointer is valid
 * @param[in]   rp   read pointer to be checked; this pointer is in bytes
 * @param[out]  fpp  pointer to #interframe_params_t structure, this pointer will be set to
 * frame header before \e rp and will point to its parameters
 * @param[in]   chn  specify compressor channel number which pointer should be checked
 * @return 0 if the pointer is for the frame yet to be acquired, 1 if there is a valid frame at this address,
 * -1 if there is no frame at this index, -2 if the pointer is not 32-bytes aligned
 * sets *fpp to the frame header, including signature and length
 */
int circbuf_valid_ptr(int rp, struct interframe_params_t **fpp, unsigned int chn)
{
	int last_image_chunk;
	unsigned int sec;
	unsigned int usec;
	int wp = camseq_get_jpeg_wp(chn);
	unsigned int len32 = get_image_length(DW2BYTE(wp), chn, &last_image_chunk);
	struct interframe_params_t *fp;

	if (rp & 0x1f) {
		// rp is not 32-bytes aligned
		dev_dbg(g_dev_ptr, "misaligned pointer rp = 0x%x for channel %d\n", rp, chn);
		return -2;
	}
	fp = (struct interframe_params_t *) &circbuf_priv[chn].buf_ptr[BYTE2DW(X393_BUFFSUB(rp, sizeof(struct interframe_params_t)))];
	*fpp = fp;

	dump_interframe_params(fp, X393_BUFFSUB(rp, sizeof(struct interframe_params_t)));

	if (BYTE2DW(rp) == wp)
		// read pointer and write pointer coincide, frame not yet acquired
		return 0;

	if (fp->signffff != MARKER_FFFF) {
		dev_dbg(g_dev_ptr, "interframe signature is overwritten\n");
		return -1;
	}

	return 1;
}

/**
 * @brief Reposition read/write file offset
 *
 * This function is overloaded with additional functionality in order to avoid ioctls.
 * In case user-space program set <em>orig == SEEK_END</em>, \e lseek will treat (offset > 0) as a command
 * to manipulate frame pointer(s) or wait for the image to be ready using these commands:
 *
 *  LSEEK_CIRC_TORP  - set file pointer to global (shared) read pointer;
 *  LSEEK_CIRC_TOWP  - set file pointer to FPGA write pointer (next frame to be acquired);
 *  LSEEK_CIRC_PREV  - move pointer to the previous frame, return \e -EOVERFLOW if there are none;
 *  LSEEK_CIRC_NEXT  - advance pointer to the next frame, return \e -EOVERFLOW if it was at the last frame
 *  LSEEK_CIRC_LAST  - move pointer to the last acquired frame (default after open), this is a combination
 *                     of LSEEK_CIRC_TOWP and LSEEK_CIRC_PREV;
 *  LSEEK_CIRC_FIRST - move pointer to the first acquired frame. It s not safe to rely
 *                     on this pointer if more frames are expected - next incoming frame
 *                     can overwrite this one;
 *  LSEEK_CIRC_SCND -  move pointer to the second oldest acquired frame. A slightly safer
 *                     to use instead of LSEEK_CIRC_FIRST when constant acquisition is on
 *                     and sensor provides new frames - this frame will likely survive longer;
 *  LSEEK_CIRC_SETP -  save current pointer to global read pointer
 *  LSEEK_CIRC_VALID - verify that the frame at current location is valid (not overrun in the buffer).
 *                     Returns file pointer if it is valid and -1 otherwise;
 *  LSEEK_CIRC_READY - verify frame at current location is available (valid and acquired).
 *                     Returns file pointer if it is ready or -1 otherwise
 *  LSEEK_CIRC_WAIT -  sleep until next frame is acquired.
 * All commands but (LSEEK_CIRC_TOWP, LSEEK_CIRC_LAST, LSEEK_CIRC_FIRST) will return -EINVAL if read
 * pointer is not valid (i.e buffer was overrun and data pointed is lost). In case of success, they return
 * current (byte *) to the start of the frame data (parameters are at offset - 32 from it).
 * (0, SEEK_CUR) also verifies that the header is not overwritten. It can be used after buffering frame data to
 * verify you got it all correctly.
 *
 * SEEK_CUR also supports the circular nature of the buffer and rolls over if needed.
 *
 * Additional commands for SEEK_END (they _DO_ modify the current file pointer):
 *  LSEEK_CIRC_FREE -  returns remaining memory in circbuf from the current file pointer,
 *                   or -EINVAL if the pointer is invalid. As this command uses the buffer write pointer
 *                   that is updated only when the complete frame is in the buffer, the actual
 *                   free memory may be less by a whole frame if compressor is running.
 *  LSEEK_CIRC_USED - returns memory used in the in circbuf from the current file pointer,
 *                   or -EINVAL if the pointer is invalid
 *  @param[in]   file   pointer to \e file structure
 *  @param[in]   offset offset inside buffer in bytes
 *  @param[in]   orig   origin
 *  @return      current file pointer position if operation was successful and error code otherwise
 */
loff_t circbuf_lseek(struct file *file, loff_t offset, int orig)
{
	unsigned int len32 = 0;
	int inserted_bytes;
	int last_image_chunk;
	int img_start, next_img, padded_frame;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	struct interframe_params_t * fp;
	int fvld = -1;
	int rp, bp;
	dev_dbg(g_dev_ptr, "start processing LSEEK operation: offset = 0x%x, orig = 0x%x\n",(int) offset, (int) orig);

	switch (orig) {
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		if (offset) file->f_pos += offset;
		else if (circbuf_valid_ptr(file->f_pos, &fp, chn) < 0 ) return -EINVAL; //!no frames at the specified location or pointer is not 32-byte aligned
		break;
	case SEEK_END:
		if (offset <= 0) {
			file->f_pos = CCAM_DMA_SIZE + offset;
		} else {
			// verify current frame pointer
			switch (offset) {
			case LSEEK_CIRC_TORP:
				file->f_pos = camseq_get_jpeg_rp(chn) << 2;
			case LSEEK_CIRC_PREV:
			case LSEEK_CIRC_NEXT:
			case LSEEK_CIRC_SETP:
			case LSEEK_CIRC_VALID:
			case LSEEK_CIRC_READY:
			case LSEEK_CIRC_FREE:
			case LSEEK_CIRC_USED:
				if ((fvld = circbuf_valid_ptr(file->f_pos, &fp, chn)) < 0)
					return -EINVAL; // no frames at the specified location
			}
			switch (offset) {
			case LSEEK_CIRC_FREE:
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_FREE: checking remaining memory in circbuf\n");
				bp = file->f_pos - (camseq_get_jpeg_wp(chn) << 2);
				return (file->f_pos = (bp > 0) ? bp : (bp + CCAM_DMA_SIZE)); //!Has a side effect of moving a file pointer!
			case LSEEK_CIRC_USED:
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_USED: checking used memory in circbuf\n");
				bp = (camseq_get_jpeg_wp(chn) << 2) - file->f_pos;
				return (file->f_pos = (bp > 0) ? bp : (bp + CCAM_DMA_SIZE)); //!Has a side effect of moving a file pointer!
			case LSEEK_CIRC_TORP:
				// no actions to be done here, the pointer was set on previous step
				break;
			case LSEEK_CIRC_TOWP:
				file->f_pos = camseq_get_jpeg_wp(chn) << 2;
				break;
			case LSEEK_CIRC_LAST:
				next_img = camseq_get_jpeg_wp(chn) << 2;

				dev_dbg(g_dev_ptr, "LSEEK_CIRC_LAST: next_img = 0x%x, fvld = %d\n", next_img, fvld);
				dev_dbg(g_dev_ptr, "mem dump of last 0x40 bytes in buffer number %d\n", chn);
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, &circbuf_priv[chn].buf_ptr[BYTE2DW(next_img - OFFSET_X40)], OFFSET_X40);

				len32 = get_image_length(next_img, chn, &last_image_chunk);
				if ((len32 & MARKER_FF) != MARKER_FF) {
					// we should not be here as the position was checked in circbufValidPointer
					dev_dbg(g_dev_ptr, "failed to get marker 0xFF at 0x%x, len32: 0x%x\n", next_img, len32);
					return -EOVERFLOW;
				}
				len32 &= FRAME_LENGTH_MASK;
				img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
				dev_dbg(g_dev_ptr, "calculated start address = 0x%x, length = 0x%x\n", img_start, len32);
				if (circbuf_valid_ptr(img_start, &fp, chn) < 0)
					return -EOVERFLOW;
				file->f_pos = img_start;
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_LAST: moving file->f_pos to 0x%llx\n", file->f_pos);
				break;
			case LSEEK_CIRC_PREV:
				rp = file->f_pos;
				fvld = circbuf_valid_ptr(rp, &fp, chn);

				dev_dbg(g_dev_ptr, "LSEEK_CIRC_PREV: rp = 0x%x, fvld = %d\n", rp, fvld);
				dev_dbg(g_dev_ptr, "mem dump of last 0x40 bytes in buffer number %d\n", chn);
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, &circbuf_priv[chn].buf_ptr[BYTE2DW(rp - OFFSET_X40)], OFFSET_X40);

				len32 = get_image_length(rp, chn, &last_image_chunk);
				if ((len32 & MARKER_FF) != MARKER_FF) {
					// we should not be here as the position was checked in circbufValidPointer
					dev_dbg(g_dev_ptr, "failed to get marker 0xFF at 0x%x, len32: 0x%x\n", rp, len32);
					return -EOVERFLOW;
				}
				len32 &= FRAME_LENGTH_MASK;
				img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_PREV: calculated start address = 0x%x, length = 0x%x\n", img_start, len32);

				// move file pointer only if previous frame valid
				len32 = get_image_length(img_start, chn, NULL);
				if (circbuf_valid_ptr(img_start, &fp, chn) < 0)
					return -EOVERFLOW;
				file->f_pos = img_start;
				break;
			case LSEEK_CIRC_NEXT:
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_NEXT: file->f_pos = 0x%lx, fvld = %d, fp->len32 = 0x%lx\n", file->f_pos, fvld, fp->frame_length);
				if (fvld <= 0)
					return -EOVERFLOW; //! no frames after current
				// calculate the full length of current frame and advance file pointer by this value
				padded_frame = fp->frame_length + INSERTED_BYTES(fp->frame_length) + CHUNK_SIZE + CCAM_MMAP_META;
				file->f_pos = X393_BUFFADD(file->f_pos, padded_frame); // do it even if the next frame does not yet exist
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_NEXT: moving file->f_pos to 0x%llx\n", file->f_pos);
				break;
			case LSEEK_CIRC_FIRST:
			case LSEEK_CIRC_SCND:
			{
				int nf = 0; // number of frames;
				int nz = 1; // number of start crossing of the circular buffer (counter to prevent looping forever)
				int prev_p, preprev_p;
				// starting from the write pointer to be able to count all the frames in the buffer
				rp = camseq_get_jpeg_wp(chn);
				prev_p = preprev_p = rp;
				file->f_pos = DW2BYTE(rp);
				while (((fvld = circbuf_valid_ptr(DW2BYTE(rp), &fp, chn)) >= 0) & (nz >= 0)) {
					nf++;
					preprev_p = prev_p; // second known good (at least first one)
					prev_p=rp;          // now - current, known good
					len32 = get_image_length(DW2BYTE(rp), chn, &last_image_chunk);
					dev_dbg(g_dev_ptr, "LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: number of frames = %d, rp = 0x%x, fvld = %d, len32 = 0x%x", nf, rp, fvld, len32);
					if ((len32 & MARKER_FF) != MARKER_FF ) break;  //! no frames before rp (==prev_p)
					//! move rp to the previous frame
					len32 &= FRAME_LENGTH_MASK;
					img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
					dev_dbg(g_dev_ptr, "LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: calculated start address = 0x%x, length = 0x%x\n", img_start, len32);
					rp = BYTE2DW(img_start);
					if (rp > prev_p) nz--; // rolled through zero - make sure we'll not stuck in this loop forever
				}
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: finish stepping back through frames, number of frames = %d, rp = 0x%x, fvld = %d, len32 = 0x%x", nf, rp, fvld, len32);
				file->f_pos = ((offset == LSEEK_CIRC_SCND) ? preprev_p : prev_p) << 2;
				break;
			}
			case LSEEK_CIRC_SETP:
				dev_dbg(g_dev_ptr, "LSEK_CIRC_SETP: Setting jpeg_rp for channel %d to file->f_pos = 0x%llx\n", chn, file->f_pos);
				camseq_set_jpeg_rp(chn, file->f_pos >> 2);
				break;
			case LSEEK_CIRC_VALID:
				// no actions to be done here, the pointer was checked on previous step
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_VALID: no action required\n");
				break;
			case LSEEK_CIRC_READY:
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_READY: checking fvld, fvld = %d\n", fvld);
				if (fvld <= 0) return -EINVAL; // no frame is available better code?
				break;
			case LSEEK_CIRC_WAIT:
				dev_dbg(g_dev_ptr, "LSEEK_CIRC_WAIT\n");
				while (((fvld=circbuf_valid_ptr(file->f_pos, &fp, chn)))==0) { // only while not ready, ready or BAD - return
					wait_event_interruptible(circbuf_wait_queue, (camseq_get_jpeg_wp(chn) << 2) != file->f_pos);
				}
				if (fvld < 0) return -ESPIPE;      // invalid seek - have better code?
				return file->f_pos ; // data already available, return file pointer
			default:
				if ((offset & ~0x1f)==LSEEK_DAEMON_CIRCBUF) {
					wait_event_interruptible(circbuf_wait_queue, get_imageParamsThis(P_DAEMON_EN) & (1<<(offset & 0x1f)));
				}
			}
			return ( file->f_pos ); // file position >= 0
		}
		break;
	default:
		return -EINVAL;
	}
	// roll-over position
	while (file->f_pos < 0) file->f_pos += CCAM_DMA_SIZE;
	while (file->f_pos > CCAM_DMA_SIZE) file->f_pos -= CCAM_DMA_SIZE;
	if ((orig !=SEEK_END) && (file->f_pos == CCAM_DMA_SIZE)) file->f_pos=0; // only for lseek(fd,0,SEEK_END) the result will be file size, others will roll to 0
	return  file->f_pos ;
}

/**
 * @brief This function handles write operations for circbuf files.
 * Note: never use \e file->f_pos in this function.
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer containing data
 * @param[in]   count number of bytes in buffer
 * @param[in]   off   offset
 * @return      number of bytes read form \e buf
 */
ssize_t circbuf_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	dev_dbg(g_dev_ptr, "minor = 0x%x, count = 0x%x, off = 0x%lx", minor, count, off);

	/* debug code follows*/
	switch (buf[0] - 0x30) {
	case 0:
		compressor_interrupts(0,chn);
		break;
	case 1:
		compressor_interrupts(1,chn);
		break;
	}
	/* debug code end */

	p = *off;
	if (p >= CCAM_DMA_SIZE)
		p = CCAM_DMA_SIZE;
	if ((p + count) > CCAM_DMA_SIZE)
		count = CCAM_DMA_SIZE - p;
	if (count) {
		if (copy_from_user(&circbuf_priv[chn].buf_ptr[BYTE2DW(p)], buf, count))
			return -EFAULT;
		*off += count;
	}
	return count;
}

/**
 * @brief This function handles read operations for circbuf files.
 * Note: never use \e file->f_pos in this function.
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer where data will be written to
 * @param[in]   count number of bytes written to \e buf
 * @param[in]   off   offset
 * @return      number of bytes written to \e buf
 */
ssize_t circbuf_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	dev_dbg(g_dev_ptr, "minor = 0x%x, count = 0x%x, off = 0x%llx", minor, count, off);

	p = *off;
	if (p >= CCAM_DMA_SIZE)
		p = CCAM_DMA_SIZE;
	if ((p + count) > CCAM_DMA_SIZE)
		count = CCAM_DMA_SIZE - p;
	if (count) {
		if (copy_to_user(buf, &circbuf_priv[chn].buf_ptr[BYTE2DW(p)], count))
			return -EFAULT;
		*off+=count;
	}
	return count;
}

int circbuf_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);

	dev_dbg(g_dev_ptr, "vm_start = 0x%lx\n", vma->vm_start);
	dev_dbg(g_dev_ptr, "vm_end = 0x%lx\n", vma->vm_end);
	dev_dbg(g_dev_ptr, "vm_pgoff = 0x%lx\n", vma->vm_pgoff);
	dev_dbg(g_dev_ptr, "vm_file = 0x%lx\n", (unsigned long)vma->vm_file);
	dev_dbg(g_dev_ptr, "ccam_dma_buf = 0x%lx\n", (unsigned long)circbuf_priv[chn].phys_addr);

	/* remap_pfn_range will mark the range VM_IO and VM_RESERVED */
	ret = remap_pfn_range(vma,
			vma->vm_start,
			circbuf_priv[chn].phys_addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

	dev_dbg(g_dev_ptr, "remap_pfn_range returned 0x%x\n", ret);
	if (ret) return -EAGAIN;

	return 0;
}

/**
 * @brief This driver method is called when user-space program performs <em>poll, select</em> or
 * \e epoll system call.
 *
 * If the current read pointer is invalid, circbuf_poll returns POLLHUP
 * as no data will be ever available until file pointer is reset.
 * If it is valid, wait is setup and blocking condition occurs in case
 * current file pointer is equal to the FPGA write pointer.
 * @param[in]   file   pointer to <em>struct file</em> structure
 * @param[in]   wait   pointer to <em>struct poll_table</em> structure
 * return       POLLHUP if pointer is invalid, (POLLIN | POLLRDNORM) if frame is ready,
 *              0 in case nothing is ready.
 */
unsigned int circbuf_poll (struct file *file, poll_table *wait)
{
	int w_ptr;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	struct interframe_params_t * fp;
	int rslt;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	rslt = circbuf_valid_ptr(file->f_pos, &fp, chn);
	if (rslt < 0) {
		// not a valid read pointer, probable buffer overrun
		dev_dbg(g_dev_ptr, "invalid pointer file->f_pos = 0x%llx\n", file->f_pos);
		return  POLLHUP ;
	} else if (rslt > 0) {
		return POLLIN | POLLRDNORM; //! there was frame already available
	} else {
		// pointer valid, no frame yet
		poll_wait(file, &circbuf_wait_queue, wait);
		// Frame might become available during call to poll_wait so nobody will wake us up,
		// let's see if there is still no frame.
		w_ptr = camseq_get_jpeg_wp(chn) << 2;
		if (w_ptr != file->f_pos)
			return POLLIN | POLLRDNORM; //! we are lucky - got it
	}
	return 0; // nothing ready
}

static struct file_operations circbuf_fops = {
		.owner          = THIS_MODULE,
		.llseek         = circbuf_all_lseek,
		.read           = circbuf_all_read,
		.write          = circbuf_all_write,
		//ioctl:    circbuf_all_ioctl,
		.open           = circbuf_all_open,
		.mmap           = circbuf_all_mmap,
		.poll           = circbuf_all_poll,
		.release        = circbuf_all_release
};

/**
 * @brief cirbuf driver probing function
 * @param[in]   pdev   pointer to \b platform_device structure
 * @return      0 on success or negative error code otherwise
 */
static int circbuf_all_init(struct platform_device *pdev)
{
   int res;
   struct device *dev = &pdev->dev;
   const struct of_device_id *match;

   /* sanity check */
   match = of_match_device(elphel393_circbuf_of_match, dev);
   if (!match)
	   return -EINVAL;

   dev_dbg(dev, "registering character device with name 'circbuf_operations'");
   res = register_chrdev(CIRCBUF_MAJOR, "circbuf_operations", &circbuf_fops);
   if(res < 0) {
	   dev_err(dev, "couldn't get a major number %d.\n", CIRCBUF_MAJOR);
	   return res;
   }
   dev_info(dev, "registered MAJOR: %d\n", CIRCBUF_MAJOR);

   res = init_ccam_dma_buf_ptr(pdev);
   if (res < 0) {
	   dev_err(dev, "ERROR allocating coherent DMA buffer\n");
	   return -ENOMEM;
   }

   dev_dbg(dev, "initialize circbuf wait queue\n");
   init_waitqueue_head(&circbuf_wait_queue);
   dev_dbg(dev, "initialize Huffman tables with default data\n");

   res = jpeghead_init(pdev);
   if (res < 0) {
	   dev_err(dev, "unable to initialize jpeghead module\n");
	   return res;
   }
   res = image_acq_init(pdev);
   if (res < 0) {
	   dev_err(dev, "unable to initialize sensor_common module\n");
	   return res;
   }

   g_dev_ptr = dev;

   return 0;
}

static int circbuf_remove(struct platform_device *pdev)
{
	unregister_chrdev(CIRCBUF_MAJOR, "circbuf_operations");

	return 0;
}

static const struct of_device_id elphel393_circbuf_of_match[] = {
		{ .compatible = "elphel,elphel393-circbuf-1.00" },
		{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_circbuf_of_match);

static struct platform_driver elphel393_circbuf = {
		.probe          = circbuf_all_init,
		.remove         = circbuf_remove,
		.driver = {
				.name = CIRCBUF_DRIVER_NAME,
				.of_match_table = elphel393_circbuf_of_match,
		},
};

module_platform_driver(elphel393_circbuf);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(CIRCBUF_DRIVER_NAME);
