/** @file circbuf.c
 *
 * @brief Drivers to manipulate large circular buffer that holds compressed
 * images/video. Buffer frame data is filled in by the FPGA, frame pointers and
 * essential frames metadata filled during servicing of the interrupts.
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uio.h>
#include <asm/uaccess.h>
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
#include <elphel/elphel393-mem.h>

#include "framepars.h"
#include "sensor_common.h"
#include "jpeghead.h"
#include "circbuf.h"
#include "exif.h"
#include "x393_macro.h"
#include "x393_helpers.h"

/** @brief Driver name displayed in system logs */
#define CIRCBUF_DRIVER_NAME "circbuf driver"

/** @brief Wait queue for the processes waiting for a new frame to appear in the circular buffer */
wait_queue_head_t circbuf_wait_queue;
struct circbuf_priv_t circbuf_priv[SENSOR_PORTS];
struct circbuf_priv_t *circbuf_priv_ptr = circbuf_priv;
/** @brief Global pointer to basic device structure. This pointer is used in debugfs output functions */
static struct device *g_dev_ptr;
/** @brief Structure used for matching a device from device tree */
static const struct of_device_id elphel393_circbuf_of_match[];
unsigned long *ccam_dma_buf_ptr[SENSOR_PORTS] = {NULL};

/**
 * @brief Set up pointers to memory where circular buffers are located. The memory
 * for circular buffers is reserved using CMA mechanism.
 * @param[in]   pdev   driver's platform device structure
 * @return      0 if pointers were set up and negative error code otherwise
 */
int init_ccam_dma_buf_ptr(struct platform_device *pdev)
{
	int i;
	dma_addr_t dma_handle;
	const size_t dma_size = (CCAM_DMA_SIZE + (PAGE_SIZE >> 2)) * sizeof(int);
	struct device *dev = &pdev->dev;
	unsigned long *dma_buf_ptr = NULL;

	// use Elphel_buf if it was allocated
	if (pElphel_buf != NULL) {
		dma_buf_ptr = pElphel_buf->vaddr;
		dma_handle = pElphel_buf->paddr;
		dev_info(dev, "using %lu bytes of DMA memory from pElphel_buf at address 0x%08x", pElphel_buf->size * PAGE_SIZE, dma_handle);
	} else {
		dma_buf_ptr = dmam_alloc_coherent(dev, dma_size, &dma_handle, GFP_KERNEL);
		if (!dma_buf_ptr) {
			dev_err(dev, "unable to allocate DMA buffer\n");
			return -ENOMEM;
		} else {
			dev_info(dev, "%d bytes of DMA memory allocated at address 0x%08x", dma_size , dma_handle);
		}
	}

	for (i = 0; i < SENSOR_PORTS; i++) {
		circbuf_priv[i].buf_ptr = dma_buf_ptr + BYTE2DW(CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE);
		circbuf_priv[i].phys_addr = dma_handle + CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE;
		ccam_dma_buf_ptr[i] = circbuf_priv[i].buf_ptr;
		// set circular buffer size in bytes
		set_globalParam(i, G_CIRCBUFSIZE, CCAM_DMA_SIZE);
	}

	return 0;
}

int circbuf_get_ptr(int sensor_port, size_t offset, size_t len, struct fvec *vect_0, struct fvec *vect_1)
{
	int ret = 1;

	if (offset > CCAM_DMA_SIZE || sensor_port >= SENSOR_PORTS)
		return -EINVAL;

	if (offset + len < CCAM_DMA_SIZE) {
		// the image is not split
		vect_0->iov_base = circbuf_priv[sensor_port].buf_ptr + offset;
		vect_0->iov_dma = circbuf_priv[sensor_port].phys_addr + offset;
		vect_0->iov_len = len;
		vect_1->iov_base = NULL;
		vect_1->iov_len = 0;
		vect_1->iov_dma = 0;
	} else {
		// the image is split into two segments
		vect_0->iov_base = circbuf_priv[sensor_port].buf_ptr + offset;
		vect_0->iov_dma = circbuf_priv[sensor_port].phys_addr + offset;
		vect_0->iov_len = CCAM_DMA_SIZE - offset;
		vect_1->iov_base = circbuf_priv[sensor_port].buf_ptr;
		vect_1->iov_dma = circbuf_priv[sensor_port].phys_addr;
		vect_1->iov_len = len - vect_0->iov_len;
		ret = 2;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(circbuf_get_ptr);

/**
 * @brief Process circular buffer file opening and define further action in accordance
 * with minor file number.
 * @param[in]   inode
 * @param[in]   filp
 * @return      0 if file was opened successfully and negative error code otherwise
 */
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

/**
 * @brief Process circular buffer file closing and define further action in accordance
 * with minor file number.
 * @param[in]   inode
 * @param[in]   filp
 * @return      0 if file was opened successfully and negative error code otherwise
 */
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

/**
 * @brief Process lseek operation on circular buffer file and define further
 * action in accordance with minor file number.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[in]   offset offset in bytes
 * @param[in]   orig   where the @e offset should start
 * @return      The resulting offset location in bytes from the beginning of the file.
 */
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
			rp = BYTE2DW(X393_BUFFSUB(offset, CHUNK_SIZE)) & (~7); // convert to index to long, align to 32-bytes
			fp = (struct interframe_params_t *) &circbuf_priv[chn].buf_ptr[rp];
		}
		return  jpeghead_lseek(file, offset, orig, fp);
	case HUFFMAN_MINOR:
		return  huffman_lseek(file, offset, orig);
	default:
		return -EINVAL;
	}
}

/**
 * @brief Process read operation on circular buffer file and define further
 * action in accordance with minor file number.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[out]  buf    pointer to user buffer where the data should be placed
 * @param[in]   count  the size of requested data transfer
 * @param[in]   off    file position the user is accessing
 * @return      The number of bytes copied or negative error code.
 */
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

/**
 * @brief Process write operation on circular buffer file and define further
 * action in accordance with minor file number.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[out]  buf    pointer to user buffer holding the data
 * @param[in]   count  the size of requested data transfer
 * @param[in]   off    file position the user is accessing
 * @return      The number of bytes copied or negative error code.
 */
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

/**
 * @brief Process memory map operation on circular buffer file and define further
 * action in accordance with minor file number. Only circular buffer itself supports
 * memory mapped operation, all other files will return an error.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[in]   vma    contains the information about the virtual address range
 * @return      0 if file was mapped successfully and negative error code otherwise
 */
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

/**
 * @brief Process poll operation on circular buffer file and define further
 * action in accordance with minor file number. Only circular buffer itself supports
 * memory mapped operation, all other files will return an error.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[in]   wait   pointer to a wait queue
 * @return      POLLHUP if pointer is invalid, (POLLIN | POLLRDNORM) if frame is ready,
 *              0 in case nothing is ready or incorrect minor number received.
 */
unsigned int circbuf_all_poll(struct file *file, poll_table *wait)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int dev_type;
	dev_dbg(g_dev_ptr, "minor = 0x%x\n", minor);

	minor_to_chn(minor, &dev_type);
	switch (dev_type) {
	case CIRCBUF_MINOR:
		return circbuf_poll(file, wait);
	default:
		return 0;
	}
}

/**
 * @brief Process circular buffer file opening.
 * @param[in]   inode
 * @param[in]   filp
 * @return      Always 0.
 */
int circbuf_open(struct inode *inode, struct file *filp)
{
	inode->i_size = CCAM_DMA_SIZE;
	dev_dbg(g_dev_ptr, "inode->i_size = 0x%lx\n", inode->i_size);

	return 0;
}

/**
 * @brief Debug function, prints the content of interframe parameters.
 * @param[in]   params   pointer to #interframe_params_t which should be printed
 * @param[in]   offset   offset in circular buffer where the @e params is located
 * @param[in]   chn      channel number this @e params belongs to
 * @return      None
 */
void dump_interframe_params(struct interframe_params_t *params, int offset, unsigned int chn)
{
	dev_dbg(g_dev_ptr, "Dump of interframe parameters at offset 0x%x [chn %u]:\n", offset, chn);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, params, sizeof(struct interframe_params_t) - 4);
}

/**
 * @brief Get the length of an image before given pointer. The pointer @e byte_offset
 * points to the beginning of new image data segment and this function calculates
 * the length of the previous image data segment and returns a pointer to the end of
 * this segment.
 * @param[in]   byte_offset   the offset of image data in circular buffer
 * @param[in]   chn           circular buffer channel
 * @param[out]  last_chunk_offset the offset where image data ends
 */
unsigned long get_image_length(int byte_offset, unsigned int chn, int *last_chunk_offset)
{
	unsigned int offset;
	unsigned long len32;
	int last_image_chunk = X393_BUFFSUB(byte_offset, OFFSET_X40);

	offset = X393_BUFFADD(last_image_chunk, CHUNK_SIZE - CCAM_MMAP_META_LENGTH);
	len32 = circbuf_priv[chn].buf_ptr[BYTE2DW(offset)];

	dev_dbg(g_dev_ptr, "[chn %d] byte_offset = 0x%x, calculated offset = 0x%x, len32 = 0x%lx\n", chn, byte_offset, offset, len32);

	if ((len32 & MARKER_FF) != MARKER_FF) {
		dev_dbg(g_dev_ptr, "[chn %u] failed to get 0xff marker at offset 0x%x\n", chn, offset);
		last_image_chunk = X393_BUFFSUB(byte_offset, OFFSET_X40 + CHUNK_SIZE);
		offset = X393_BUFFADD(last_image_chunk, CHUNK_SIZE - CCAM_MMAP_META_LENGTH);
		len32 = circbuf_priv[chn].buf_ptr[BYTE2DW(offset)];
		if ((len32 & MARKER_FF) != MARKER_FF) {
			dev_dbg(g_dev_ptr, "[chn %u] failed to get 0xff marker at CORRECTED offset 0x%x\n", chn, offset);
			return 0;
		}
	}

	dev_dbg(g_dev_ptr, "[chn %u] got len32 = 0x%lx at 0x%x\n", chn, len32, offset);

	if (last_chunk_offset != NULL)
		*last_chunk_offset = last_image_chunk;

	return len32;
}

/**
 * @brief Check that read pointer is valid
 * @param[in]   rp_offset read pointer to be checked; this pointer is in bytes
 * @param[out]  fpp  pointer to #interframe_params_t structure, this pointer will be set to
 * frame header before @e rp and will point to its parameters
 * @param[in]   chn  specify compressor channel number which pointer should be checked
 * @return 0 if the pointer is for the frame yet to be acquired, 1 if there is a valid frame at this address,
 * 2 if file pointer should be advanced by 32 bytes,
 * -1 if there is no frame at this index, -2 if the pointer is not 32-bytes aligned
 * sets *fpp to the frame header, including signature and length
 */
int circbuf_valid_ptr(loff_t *rp_offset, struct interframe_params_t **fpp, unsigned int chn)
{
	int rp = *rp_offset;
	int last_image_chunk;
	unsigned int sec;
	unsigned int usec;
	int wp = camseq_get_jpeg_wp(chn);
	unsigned int len32 = get_image_length(DW2BYTE(wp), chn, &last_image_chunk);
	struct interframe_params_t *fp, *fp_off;

	if (rp & 0x1f) {
		// rp is not 32-bytes aligned
		dev_dbg(g_dev_ptr, "misaligned pointer rp = 0x%x for channel %d\n", rp, chn);
		return -2;
	}
	fp = (struct interframe_params_t *) &circbuf_priv[chn].buf_ptr[BYTE2DW(X393_BUFFSUB(rp, sizeof(struct interframe_params_t) - 4))];
	*fpp = fp;

	dump_interframe_params(fp, X393_BUFFSUB(rp, sizeof(struct interframe_params_t) - 4), chn);

	if (BYTE2DW(rp) == wp)
		// read pointer and write pointer coincide, frame not yet acquired
		return 0;

	if (fp->signffff != MARKER_FFFF) {
		dev_dbg(g_dev_ptr, "[chn %u] interframe signature is overwritten, signffff = 0x%x\n", chn, fp->signffff);
		fp_off = (struct interframe_params_t *) &circbuf_priv[chn].buf_ptr[BYTE2DW(rp)];
		if (fp_off->signffff != MARKER_FFFF) {
			dev_dbg(g_dev_ptr, "[chn %u] interframe signature is overwritten at CORRECTED offset, signffff = 0x%x\n", chn, fp_off->signffff);
			return -1;
		} else {
			dev_dbg(g_dev_ptr, "[chn %u] interframe pointer and file ponter is advanced by 0x20\n", chn);
			*fpp = fp_off;
			*rp_offset = X393_BUFFADD(*rp_offset, CHUNK_SIZE);
			dump_interframe_params(fp_off, rp, chn);
			return 2;
		}
	}

	return 1;
}

/**
 * @brief Get image start offset pointed by its last data chunk
 * @param[in]   last_chunk_offset   offset of the last image data chunk
 * @param[in]   len32               length of image
 * @return      Image start offset
 */
inline int get_image_start(int last_chunk_offset, unsigned int len32)
{
	return X393_BUFFSUB(last_chunk_offset + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
}

/* debug code follows */
void stop_compressor(unsigned int chn)
{
	x393_cmprs_mode_t mode;

	mode.run = 1;
	mode.run_set = 1;
	x393_cmprs_control_reg(mode, chn);
}
void dump_state(unsigned int chn)
{
	int img_start, last_image_chunk;
	int len32;
	int prev_ptr, prevprev_ptr;
	int read_ptr = DW2BYTE(camseq_get_jpeg_wp(chn));
	unsigned int nf, nz;
	struct interframe_params_t *fp;

	nf = 0;
	nz = 1;
	printk(KERN_DEBUG "=== start of state dump, chn = %d ===\n", chn);
	printk(KERN_DEBUG "hardware pointer at 0x%x\n", read_ptr);
	// move to the beginning of last frame
	len32 = get_image_length(read_ptr, chn, &last_image_chunk);
	if ((len32 & MARKER_FF) != MARKER_FF) {
		printk(KERN_DEBUG "last acquired frame at location 0x%x is damaged\n", read_ptr);
		return;
	}
	len32 &= FRAME_LENGTH_MASK;
	//img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
	img_start = get_image_start(last_image_chunk, len32);
	read_ptr = img_start;
	// move back in history
	while ((circbuf_valid_ptr(&read_ptr, &fp, chn) >= 0) && (nz >= 0)) {
		printk(KERN_DEBUG "analyzing frame starting at 0x%x\n", read_ptr);
		//printk(KERN_DEBUG "mem dump of 0x40 bytes at (pointer - 0x20) = 0x%x:\n", read_ptr - OFFSET_X40 / 2);
		//print_hex_dump_bytes("\t\t", DUMP_PREFIX_OFFSET, &circbuf_priv[chn].buf_ptr[BYTE2DW(read_ptr - OFFSET_X40 / 2)], OFFSET_X40);
		nf++;
		prevprev_ptr = prev_ptr;
		prev_ptr = read_ptr;
		len32 = get_image_length(read_ptr, chn, &last_image_chunk);
		if ((len32 & MARKER_FF) != MARKER_FF) {
			printk(KERN_DEBUG "\t\tno image before 0x%x\n", read_ptr);
			break;
		}
		len32 &= FRAME_LENGTH_MASK;
		//printk(KERN_DEBUG "\t\tgot len32 = 0x%x", len32);
		//img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
		img_start = get_image_start(last_image_chunk, len32);
		read_ptr = img_start;
		if (read_ptr > prev_ptr)
			nz--;
	}
	printk(KERN_DEBUG "=== end of state dump, %d frame(s) analyzed ===\n", nf);
}
/* end of debug code */
/**
 * @brief Reposition read/write file offset
 *
 * This function is overloaded with additional functionality in order to avoid ioctls.
 * In case user-space program set <em>orig == SEEK_END</em>, @e lseek will treat (offset > 0) as a command
 * to manipulate frame pointer(s) or wait for the image to be ready using these commands:
 * command           | description
 * ------------------|------------------------------------------------------------------------------------
 *  LSEEK_CIRC_TORP  | set file pointer to global (shared) read pointer
 *  LSEEK_CIRC_TOWP  | set file pointer to FPGA write pointer (next frame to be acquired)
 *  LSEEK_CIRC_PREV  | move pointer to the previous frame, return @e -EOVERFLOW if there are none
 *  LSEEK_CIRC_NEXT  | advance pointer to the next frame, return @e -EOVERFLOW if it was at the last frame
 *  LSEEK_CIRC_LAST  | move pointer to the last acquired frame (default after open), this is a combination of LSEEK_CIRC_TOWP and LSEEK_CIRC_PREV
 *  LSEEK_CIRC_FIRST | move pointer to the first acquired frame. It s not safe to rely on this pointer if more frames are expected - next incoming frame can overwrite this one
 *  LSEEK_CIRC_SCND  | move pointer to the second oldest acquired frame. A slightly safer to use instead of LSEEK_CIRC_FIRST when constant acquisition is on and sensor provides new frames - this frame will likely survive longer
 *  LSEEK_CIRC_SETP  | save current pointer to global read pointer
 *  LSEEK_CIRC_VALID | verify that the frame at current location is valid (not overrun in the buffer). Returns file pointer if it is valid and -1 otherwise
 *  LSEEK_CIRC_READY | verify frame at current location is available (valid and acquired). Returns file pointer if it is ready or -1 otherwise
 *  LSEEK_CIRC_WAIT  | sleep until next frame is acquired
 * All commands but (LSEEK_CIRC_TOWP, LSEEK_CIRC_LAST, LSEEK_CIRC_FIRST) will return -EINVAL if read
 * pointer is not valid (i.e buffer was overrun and data pointed is lost). In case of success, they return
 * current (byte *) to the start of the frame data (parameters are at offset - 32 from it).
 * (0, SEEK_CUR) also verifies that the header is not overwritten. It can be used after buffering frame data to
 * verify you got it all correctly.
 *
 * SEEK_CUR also supports the circular nature of the buffer and rolls over if needed.
 *
 * Additional commands for SEEK_END (they _DO_ modify the current file pointer):
 * command           | description
 * ------------------|------------------------------------------------------------------------------------
 *  LSEEK_CIRC_FREE  | returns remaining memory in circbuf from the current file pointer, or -EINVAL if the pointer is invalid. As this command uses the buffer write pointer that is updated only when the complete frame is in the buffer, the actual free memory may be less by a whole frame if compressor is running
 *  LSEEK_CIRC_USED  | returns memory used in the in circbuf from the current file pointer, or -EINVAL if the pointer is invalid
 * The following command is used for profiling from user space applications. It does not change file pointer:
 *  LSEEK_CIRC_UTIME  return current value of microsecond counter.
 * @param[in]   file   pointer to @e file structure
 * @param[in]   offset offset inside buffer in bytes
 * @param[in]   orig   where the @e offset should start
 * @return      Current file pointer position if operation was successful and error code otherwise
 */
loff_t circbuf_lseek(struct file *file, loff_t offset, int orig)
{
	unsigned int len32 = 0;
	int last_image_chunk;
	int img_start, next_img, padded_frame;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	struct interframe_params_t * fp;
	int fvld = -1;
	int rp, bp;
	dev_dbg(g_dev_ptr, "start processing LSEEK operation: offset = 0x%x, orig = 0x%x, file->f_pos = 0x%llx\n",(int) offset, (int) orig, file->f_pos);

	switch (orig) {
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		if (offset) file->f_pos += offset;
		else if (circbuf_valid_ptr(&file->f_pos, &fp, chn) < 0 ) return -EINVAL; // no frames at the specified location or pointer is not 32-byte aligned
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
				if ((fvld = circbuf_valid_ptr(&file->f_pos, &fp, chn)) < 0)
					return -EINVAL; // no frames at the specified location
				break;
			/* debug code follows */
			case LSEEK_CIRC_STOP_COMPRESSOR:
			{
				int s;
				dev_dbg(g_dev_ptr, "stopping all compressors, current channel %d, fvld = %d, file->f_pos = 0x%llx\n", chn, fvld, file->f_pos);
				for (s = 0; s < SENSOR_PORTS; s++)
					stop_compressor(s);
				dump_state(chn);
			}
				break;
			/* end of debug code */
			}
			switch (offset) {
			case LSEEK_CIRC_FREE:
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_FREE: checking remaining memory in circbuf\n", chn);
				bp = file->f_pos - (camseq_get_jpeg_wp(chn) << 2);
				return (file->f_pos = (bp > 0) ? bp : (bp + CCAM_DMA_SIZE)); // Has a side effect of moving a file pointer!
			case LSEEK_CIRC_USED:
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_USED: checking used memory in circbuf\n", chn);
				bp = (camseq_get_jpeg_wp(chn) << 2) - file->f_pos;
				return (file->f_pos = (bp > 0) ? bp : (bp + CCAM_DMA_SIZE)); // Has a side effect of moving a file pointer!
			case LSEEK_CIRC_TORP:
				// no actions to be done here, the pointer was set on previous step
				break;
			case LSEEK_CIRC_TOWP:
				file->f_pos = camseq_get_jpeg_wp(chn) << 2;
				break;
			case LSEEK_CIRC_LAST:
				next_img = camseq_get_jpeg_wp(chn) << 2;

				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_LAST: next_img = 0x%x, fvld = %d\n", chn, next_img, fvld);
				dev_dbg(g_dev_ptr, "[chn %u] mem dump of last 0x40 bytes in buffer\n", chn);
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, &circbuf_priv[chn].buf_ptr[BYTE2DW(next_img - OFFSET_X40)], OFFSET_X40);

				len32 = get_image_length(next_img, chn, &last_image_chunk);
				if ((len32 & MARKER_FF) != MARKER_FF) {
					// we should not be here as the position was checked in circbufValidPointer
					dev_dbg(g_dev_ptr, "[chn %u] failed to get marker 0xFF at 0x%x, len32: 0x%x\n", chn, next_img, len32);
					return -EOVERFLOW;
				}
				len32 &= FRAME_LENGTH_MASK;
				//img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
				img_start = get_image_start(last_image_chunk, len32);
				dev_dbg(g_dev_ptr, "[chn %u] calculated start address = 0x%x, length = 0x%x\n", chn, img_start, len32);
				if (circbuf_valid_ptr(&img_start, &fp, chn) < 0)
					return -EOVERFLOW;
				file->f_pos = img_start;
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_LAST: moving file->f_pos to 0x%llx\n", chn, file->f_pos);
				break;
			case LSEEK_CIRC_PREV:
				rp = file->f_pos;
				fvld = circbuf_valid_ptr(&rp, &fp, chn);

				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_PREV: rp = 0x%x, fvld = %d\n", chn, rp, fvld);
				dev_dbg(g_dev_ptr, "[chn %u] mem dump of last 0x40 bytes in buffer number %d\n", chn, chn);
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, &circbuf_priv[chn].buf_ptr[BYTE2DW(rp - OFFSET_X40)], OFFSET_X40);

				len32 = get_image_length(rp, chn, &last_image_chunk);
				if ((len32 & MARKER_FF) != MARKER_FF) {
					// we should not be here as the position was checked in circbufValidPointer
					dev_dbg(g_dev_ptr, "[chn %u] failed to get marker 0xFF at 0x%x, len32: 0x%x\n", chn, rp, len32);
					return -EOVERFLOW;
				}
				len32 &= FRAME_LENGTH_MASK;
				//img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
				img_start = get_image_start(last_image_chunk, len32);
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_PREV: calculated start address = 0x%x, length = 0x%x\n", chn, img_start, len32);

				// move file pointer only if previous frame valid
				len32 = get_image_length(img_start, chn, NULL);
				if (circbuf_valid_ptr(&img_start, &fp, chn) < 0)
					return -EOVERFLOW;
				file->f_pos = img_start;
				break;
			case LSEEK_CIRC_NEXT:
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_NEXT: file->f_pos = 0x%llx, fvld = %d, fp->len32 = 0x%lx\n", chn,
						file->f_pos, fvld, fp->frame_length);
				if (fvld <= 0) {
					return -EOVERFLOW; // no frames after current
				} else if (fvld == 2) {
					//file->f_pos += CHUNK_SIZE;
					dev_dbg(g_dev_ptr, "[chn %u] read pointer file->f_pos was advanced by 0x20 bytes\n", chn);
				}
				// calculate the full length of current frame and advance file pointer by this value
				padded_frame = fp->frame_length + INSERTED_BYTES(fp->frame_length) + CHUNK_SIZE + CCAM_MMAP_META;
				file->f_pos = X393_BUFFADD(file->f_pos, padded_frame); // do it even if the next frame does not yet exist
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_NEXT: padded_frame = %d; moving file->f_pos to 0x%llx\n", chn, padded_frame, file->f_pos);
				break;
			case LSEEK_CIRC_FIRST:
			case LSEEK_CIRC_SCND:
			{
				int nf = 0; // number of frames;
				int nz = 1; // number of start crossing of the circular buffer (counter to prevent looping forever)
				int rp_b;
				int prev_p, preprev_p;
				// starting from the write pointer to be able to count all the frames in the buffer
				rp = camseq_get_jpeg_wp(chn);
				prev_p = preprev_p = DW2BYTE(rp);
				file->f_pos = DW2BYTE(rp);
				rp_b = DW2BYTE(rp);
				while (((fvld = circbuf_valid_ptr(&rp_b, &fp, chn)) >= 0) & (nz >= 0)) {
					nf++;
					preprev_p = prev_p; // second known good (at least first one)
					prev_p=rp_b;          // now - current, known good
					len32 = get_image_length(rp_b, chn, &last_image_chunk);
					dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: number of frames = %d, rp_b = 0x%x, fvld = %d, len32 = 0x%x", chn,
							nf, rp_b, fvld, len32);
					if ((len32 & MARKER_FF) != MARKER_FF ) break;  // no frames before rp (==prev_p)
					// move rp to the previous frame
					len32 &= FRAME_LENGTH_MASK;
					//img_start = X393_BUFFSUB(last_image_chunk + CHUNK_SIZE - INSERTED_BYTES(len32) - CCAM_MMAP_META, len32);
					img_start = get_image_start(last_image_chunk, len32);
					dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: calculated start address = 0x%x, length = 0x%x\n", chn,
							img_start, len32);
					rp_b = img_start;
					if (rp_b > prev_p) nz--; // rolled through zero - make sure we'll not stuck in this loop forever
				}
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_FIRST or LSEEK_CIRC_SCND: finish stepping back through frames, number of frames = %d, rp = 0x%x, fvld = %d, len32 = 0x%x",
						chn, nf, rp, fvld, len32);
				file->f_pos = ((offset == LSEEK_CIRC_SCND) ? preprev_p : prev_p) << 2;
				break;
			}
			case LSEEK_CIRC_SETP:
				dev_dbg(g_dev_ptr, "[chn %u] LSEK_CIRC_SETP: Setting jpeg_rp for channel %d to file->f_pos = 0x%llx\n", chn, chn, file->f_pos);
				camseq_set_jpeg_rp(chn, file->f_pos >> 2);
				break;
			case LSEEK_CIRC_VALID:
				// no actions to be done here, the pointer was checked on previous step
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_VALID: no action required, fvld = %d, file->f_pos = 0x%llx\n", chn, fvld, file->f_pos);
				break;
			case LSEEK_CIRC_READY:
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_READY: checking fvld, fvld = %d, file->f_pos = 0x%llx\n", chn, fvld, file->f_pos);
				if (fvld <= 0) return -EINVAL; // no frame is available better code?
				break;
			case LSEEK_CIRC_WAIT:
				dev_dbg(g_dev_ptr, "[chn %u] LSEEK_CIRC_WAIT: fvld = %d, file->f_pos = 0x%llx\n", chn, fvld, file->f_pos);
				while (((fvld=circbuf_valid_ptr(&file->f_pos, &fp, chn)))==0) { // only while not ready, ready or BAD - return
					wait_event_interruptible(circbuf_wait_queue, (camseq_get_jpeg_wp(chn) << 2) != file->f_pos);
				}
				if (fvld < 0) return -ESPIPE;      // invalid seek - have better code?
				return file->f_pos ; // data already available, return file pointer
			case LSEEK_CIRC_UTIME:
				return get_rtc_usec();
				break;
			default:
				if ((offset & ~0x1f)==LSEEK_DAEMON_CIRCBUF) {
					wait_event_interruptible(circbuf_wait_queue, get_imageParamsThis(chn, P_DAEMON_EN) & (1<<(offset & 0x1f)));
				}
			}
			dev_dbg(g_dev_ptr, "[chn %u] finish SEEK_END processing; return file->f_pos = %lld\n", chn, file->f_pos);
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
	dev_dbg(g_dev_ptr, "[chn %u] return file->f_pos = %lld\n", chn, file->f_pos);
	return  file->f_pos ;
}

unsigned short circbuf_quality = 100;
unsigned short circbuf_height = 1936;
unsigned short circbuf_width = 2592;
unsigned char circbuf_byrshift = 3;
/**
 * @brief This function handles write operations for circbuf files.
 * @note Never use @e file->f_pos in this function.
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer containing data
 * @param[in]   count number of bytes in buffer
 * @param[in]   off   offset
 * @return      number of bytes read form @e buf
 */
ssize_t circbuf_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	int i;
	int ret;
	long val;
	char *buf_copy = (char *)buf;
	dev_dbg(g_dev_ptr, "minor = 0x%x, count = 0x%x, off = 0x%llx", minor, count, off);

	/* debug code follows*/
	switch (buf[0] - 0x30) {
	case 0:
		camera_interrupts(0);
		break;
	case 1:
		camera_interrupts(1);
		break;
	case 3:
		/* update image quality */
		buf_copy[count - 1] = 0;
		ret = kstrtol(&buf_copy[2], 10, &val);
		dev_dbg(g_dev_ptr, "ret: %d, buf[2]: %s\n", ret, &buf_copy[2]);
		if (count > 2 &&  ret == 0) {
			if (val > 0 && val <= 100) {
				circbuf_quality = val;
				dev_dbg(g_dev_ptr, "set quality %d\n", circbuf_quality);
			}
		} else {
			dev_dbg(g_dev_ptr, "error, unable to process quality parameter\n");
		}
		break;
	case 4:
		for (i = 0; i < SENSOR_PORTS; i++)
			stop_compressor(i);
		dump_state(chn);
		break;
	case 5:
		/* print debug statistics */
		{
		int j, cntr;
		long long res;
		for (i = 0; i < SENSOR_PORTS; i++) {
			cntr = get_zero_counter(i);
			printk(KERN_DEBUG "channel = %d, hw pointer = 0x%x, zero_counter = %d, corrected_offset = %d, frame_counter = %lld\n",
					i, DW2BYTE(camseq_get_jpeg_wp(i)), cntr, get_corrected_offset(i), get_frame_counter(i));
			if (cntr != 0) {
				for (j = 0; j < cntr; j++) {
					res = get_frame_pos(i, j);
					printk(KERN_DEBUG "\t\tzero cross frame number: %lld\n", res);
				}
			}
			}
		}
		break;
	case 6:
		/* update frame size */
		{
		unsigned int w, h;
		int res = sscanf(&buf[2], "%u:%u", &w, &h);
		if (res == 2) {
			circbuf_width = w;
			circbuf_height = h;
			dev_dbg(g_dev_ptr, "set image size %u x %u\n", w, h);
		}
		}
		break;
	case 7:
		/* update Bayer shift */
		{
		unsigned char val;
		int res = sscanf(&buf[2], "%u", &val);
		if (res == 1) {
			circbuf_byrshift = val;
			dev_dbg(g_dev_ptr, "set new bayer shift: %u\n", val);
		}
		}
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
 * @note Never use @e file->f_pos in this function.
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer where data will be written to
 * @param[in]   count number of bytes written to @e buf
 * @param[in]   off   offset
 * @return      Number of bytes written to @e buf
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

/**
 * @brief Process memory map operation on circular buffer file.
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[in]   vma    contains the information about the virtual address range
 * @return      0 if file was mapped successfully and negative error code otherwise
 */
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
 * @e epoll system call.
 *
 * If the current read pointer is invalid, circbuf_poll returns POLLHUP
 * as no data will be ever available until file pointer is reset.
 * If it is valid, wait is setup and blocking condition occurs in case
 * current file pointer is equal to the FPGA write pointer.
 * @param[in]   file   pointer to <em>struct file</em> structure
 * @param[in]   wait   pointer to <em>struct poll_table</em> structure
 * @return      POLLHUP if pointer is invalid, (POLLIN | POLLRDNORM) if frame is ready,
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

	rslt = circbuf_valid_ptr(&file->f_pos, &fp, chn);
	if (rslt < 0) {
		// not a valid read pointer, probable buffer overrun
		dev_dbg(g_dev_ptr, "invalid pointer file->f_pos = 0x%llx\n", file->f_pos);
		return  POLLHUP ;
	} else if (rslt > 0) {
		return POLLIN | POLLRDNORM;     // there was frame already available
	} else {
		// pointer valid, no frame yet
		poll_wait(file, &circbuf_wait_queue, wait);
		// Frame might become available during call to poll_wait so nobody will wake us up,
		// let's see if there is still no frame.
		w_ptr = camseq_get_jpeg_wp(chn) << 2;
		if (w_ptr != file->f_pos)
			return POLLIN | POLLRDNORM; // we are lucky - got it
	}
	return 0;                           // nothing ready
}

static struct file_operations circbuf_fops = {
		.owner          = THIS_MODULE,
		.llseek         = circbuf_all_lseek,
		.read           = circbuf_all_read,
		.write          = circbuf_all_write,
		.open           = circbuf_all_open,
		.mmap           = circbuf_all_mmap,
		.poll           = circbuf_all_poll,
		.release        = circbuf_all_release
};

/**
 * @brief cirbuf driver probing function
 * @param[in]   pdev   pointer to @e platform_device structure
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
   res = init_ccam_dma_buf_ptr(pdev);
   if (res < 0) {
	   dev_err(dev, "ERROR allocating coherent DMA buffer\n");
	   return -ENOMEM;
   }

   dev_dbg(dev, "initialize circbuf wait queue\n");
   init_waitqueue_head(&circbuf_wait_queue);
   dev_dbg(dev, "initialize Huffman tables with default data\n");

   g_dev_ptr = dev;

   return 0;
}

/**
 * @brief cirbuf driver remove function
 * @param[in]   pdev   pointer to @e platform_device structure
 * @return      Always 0.
 */
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
