/** @file jpeghead.c
 *
 * @brief This file contains methods for JPEG tables and headers generation and
 * JPEG files composition from data compressed by FPGA.
 *
 * @copyright Copyright (C) 2016 Elphel, Inc
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

//#include <linux/module.h>
#include <linux/mm.h>
//#include <linux/sched.h>
#include <linux/slab.h>
//#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
//#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
//#include <linux/time.h>
//#include <linux/device.h>
#include <linux/platform_device.h>

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
#include <uapi/elphel/c313a.h>
//#include "fpga_io.h"//fpga_table_write_nice
#include "jpeghead.h"
//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
#include "framepars.h" // extern pastpars
#include "quantization_tables.h" // get_gtables()
//#include "x3x3.h"
//#include "cc3x3.h"
//#include "cxdma.h"
//#include "circbuf.h"
//#include "sensor_common.h"
//#include "exif.h"
#include "x393_macro.h"
#include "x393.h"

static struct device *g_dev_ptr = NULL;

/**
 * @brief All Huffman tables data to be read/written from the application
 */
struct huff_tables_t {
	struct huffman_encoded_t header_huffman_tables[4];
	unsigned long            fpga_huffman_table[512];
	union {
		unsigned char        dht_all[20];
		struct {
			unsigned char    dht_dc0[5]; /// DHT DC0 header (all constants but the length)
			unsigned char    dht_ac0[5]; /// DHT AC0 header (all constants but the length)
			unsigned char    dht_dc1[5]; /// DHT DC1 header (all constants but the length)
			unsigned char    dht_ac1[5]; /// DHT AC1 header (all constants but the length)
		};
	};
}/* huff_tables */;

static struct jpeghead_priv_t {
	struct huff_tables_t     huff_tables;
	unsigned int             fpga_programmed;
	unsigned long            jpeg_h_sz;  /// JPEG header size (no Exif)
	unsigned char            header[JPEG_HEADER_MAXSIZE];
} jpeghead_priv[SENSOR_PORTS];

#define HEADER_COPY_SOF(x) {buf[bpl] = sizeof( x ) + 8; \
                            buf[bp++] = sizeof( x ) / 3; \
                            memcpy((void *) &buf[bp], (void *) ( x ), sizeof ( x )); \
                            bp += sizeof( x );}
#define HEADER_COPY_SOS(x) {buf[bp++] = sizeof( x ) + 6; \
                            buf[bp++] = sizeof( x ) / 2; \
                            memcpy((void *) &buf[bp], (void *) ( x ), sizeof ( x )); \
                            bp += sizeof( x );}
/**
 * @brief Copy two quantization tables for the current frame (for the RTP streamer)
 * @param[in]   params   pointer to an array of parameters stored for the frame
 * @param[out]  buf      buffer to put the header to
 * @param[in]   chn      compressor channel number
 * @return      header length if successful, < 0 - error
 */
int qtables_create(struct interframe_params_t *params, unsigned char *buf, unsigned int chn)
{
	dev_dbg(g_dev_ptr, "params->quality2 = 0x%x\n", params->quality2);
	int rslt = get_qtable(params->quality2, &buf[0], &buf[64], chn); /// will copy both quantization tables
	if (rslt < 0) return rslt; /// bad quality table
	return 128;
}

/**
 * @brief Create JPEG header for the frame acquired earlier
 * @param[in]   params   pointer to an array of parameters stored for the frame
 * @param[our]  buf      buffer to put the header to
 * @return      header length if successful, < 0 - error
 */
int jpegheader_create(struct interframe_params_t *params, unsigned char *buf, unsigned int chn)
{
	int bp=0;   ///buffer pointer
	int bpl;    /// pointer to length word in the buffer
	int rslt;
	int len;
	int header_sos; /// start of SOS (variable)
	const int header_yqtable=    0x19;
	const int header_cqtable_hd= 0x59;
	const int header_cqtable=    0x5e;
	const int header_sof=        0x9e;
	/// first constant part of the header - 0x19 bytes
	const unsigned char jfif1[0x19]={0xff, 0xd8,                   /// SOI start of image
			0xff, 0xe0,                   /// APP0
			0x00, 0x10,                   /// (16 bytes long)
			0x4a, 0x46, 0x49, 0x46, 0x00, /// JFIF null terminated
			0x01, 0x01, 0x00, 0x00, 0x01,
			0x00, 0x01, 0x00, 0x00,
			0xff, 0xdb,                   /// DQT (define quantization table)
			0x00, 0x43,                   /// 0x43 bytes long
			0x00 };                       /// table number + (bytes-1)<<4 (0ne byte - 0, 2 bytes - 0x10)
	/// second constant part of the header (starting from byte 0x59 - 0x5 bytes)

	const unsigned char jfif2[0x5]= {0xff, 0xdb,                   /// DQT (define quantization table)
			0x00, 0x43,                   /// 0x43 bytes long
			0x01 };                       /// table number + (bytes-1)<<4 (0ne byte - 0, 2 bytes - 0x10)

	const unsigned char sof_color6[]= {0x01, 0x22, 0x00, /// id , freqx/freqy, q
			0x02, 0x11, 0x01,
			0x03, 0x11, 0x01};
	const unsigned char sos_color6[]= {0x01, 0x00, /// id, hufftable_dc/htable_ac
			0x02, 0x11,
			0x03, 0x11};

	const unsigned char sof_jp46dc[]= {0x01, 0x11, 0x00, /// id , freqx/freqy, q
			0x02, 0x11, 0x00,
			0x03, 0x11, 0x00,
			0x04, 0x11, 0x00,
			0x05, 0x11, 0x01,
			0x06, 0x11, 0x01};
	const unsigned char sos_jp46dc[]= {0x01, 0x00, /// id, hufftable_dc/htable_ac
			0x02, 0x00,
			0x03, 0x00,
			0x04, 0x00,
			0x05, 0x11,
			0x06, 0x11};

	const unsigned char sof_mono4[]=  {0x01, 0x22, 0x00}; /// id , freqx/freqy, q
	const unsigned char sos_mono4[]=  {0x01, 0x00}; /// id, hufftable_dc/htable_ac

	const unsigned char sof_jp4[]=    {0x04, 0x22, 0x00}; /// id , freqx/freqy, q
	const unsigned char sos_jp4[]=    {0x04, 0x00}; /// id, hufftable_dc/htable_ac

	const unsigned char sof_jp4dc[]=  {0x04, 0x11, 0x00, /// id , freqx/freqy, q
			0x05, 0x11, 0x00,
			0x06, 0x11, 0x00,
			0x07, 0x11, 0x00};
	const unsigned char sos_jp4dc[]=  {0x04, 0x00, /// id, hufftable_dc/htable_ac
			0x05, 0x00,
			0x06, 0x00,
			0x07, 0x00};

	const unsigned char sof_jp4diff[]={0x04, 0x11, 0x11, /// will be adjusted to bayer shift, same for jp4hdr
			0x05, 0x11, 0x11,
			0x06, 0x11, 0x11,
			0x07, 0x11, 0x11};
	const unsigned char sos_jp4diff[]={0x04, 0x11, /// id, hufftable_dc/htable_ac
			0x05, 0x11,
			0x06, 0x11,
			0x07, 0x11};

	struct huff_tables_t *huff_tables = &jpeghead_priv[chn].huff_tables;
	unsigned char *p = (unsigned char *)params;

	if (buf==NULL) return -1; /// buffer is not provided

	dev_dbg(g_dev_ptr, "list of parameters:\n");
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, p, 32);

	memcpy((void *) &buf[0],                 (void *) jfif1, sizeof (jfif1)); /// including DQT0 header
	memcpy((void *) &buf[header_cqtable_hd], (void *) jfif2, sizeof (jfif2)); /// DQT1 header
	rslt=get_qtable(params->quality2, &buf[header_yqtable], &buf[header_cqtable], chn); /// will copy both quantization tables
	if (rslt <0) return rslt; /// bad quality table
	bp=header_sof;
	buf[bp++]=0xff; buf[bp++]=0xc0;
	buf[bp++]=0;  /// high byte length - always 0
	bpl=bp;       /// save pointer to length (low byte)
	bp++;
	buf[bp++]=0x8; /// 8bpp
	buf[bp++]=params->height >> 8; buf[bp++]=params->height; /// big endian height
	buf[bp++]=params->width  >> 8; buf[bp++]=params->width;  /// big endian width
	/// copy SOF0 (constants combined with bayer shift for jp4diff/jp4hdr)
	switch (params->color) {
	case COLORMODE_MONO6:    /// monochrome, (4:2:0),
	case COLORMODE_COLOR:    /// color, 4:2:0, 18x18(old)
	case COLORMODE_COLOR20:  /// color, 4:2:0, 20x20, middle of the tile (not yet implemented)
	case COLORMODE_JP46:     /// jp4, original (4:2:0)
		HEADER_COPY_SOF(sof_color6);
		break;
	case COLORMODE_MONO4:   /// monochrome, 4 blocks (but still with 2x2 macroblocks)
		HEADER_COPY_SOF(sof_mono4);
		break;
	case COLORMODE_JP4:     /// jp4, 4 blocks
		HEADER_COPY_SOF(sof_jp4);
		break;
	case COLORMODE_JP46DC:   /// jp4, dc -improved (4:2:0)
		HEADER_COPY_SOF(sof_jp46dc);
		break;
	case COLORMODE_JP4DC:   /// jp4, 4 blocks, dc -improved
		HEADER_COPY_SOF(sof_jp4dc);
		break;
	case COLORMODE_JP4DIFF:   /// jp4, 4 blocks, differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
	case COLORMODE_JP4DIFF2:   /// jp4, 4 blocks, differential, divide differences by 2: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
		HEADER_COPY_SOF(sof_jp4diff);
		//header_sof
		//bshift
		buf[header_sof+12+3*((4-params->byrshift) & 3)]=0; /// set quantization table 0 for the "base color"
		break;
	case COLORMODE_JP4HDR:   /// jp4, 4 blocks, differential HDR: red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
	case COLORMODE_JP4HDR2:   /// jp4, 4 blocks, differential HDR: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2)
		HEADER_COPY_SOF(sof_jp4diff); /// same as for COLORMODE_JP4DIFF
		buf[header_sof+12+3*((4-params->byrshift) & 3)]=0; /// set quantization table 0 for the "base color"
		buf[header_sof+12+3*((6-params->byrshift) & 3)]=0; /// set quantization table 0 for the HDR color
		break;
	}
	/// Include 4 huffman tables
	memcpy((void *) &buf[bp], (void *) huff_tables->dht_dc0, 5); /// DHT DC0 header
	bp+=5;
	len= (huff_tables->dht_dc0[2]<<8)+huff_tables->dht_dc0[3]-3;  /// table length itself, excluding 2 length bytes and type byte
	memcpy((void *) &buf[bp], (void *) &huff_tables->header_huffman_tables[0], len);
	bp+=len;

	memcpy((void *) &buf[bp], (void *) huff_tables->dht_ac0, 5); /// DHT AC0 header
	bp+=5;
	len= (huff_tables->dht_ac0[2]<<8)+huff_tables->dht_ac0[3]-3;  /// table length itself, excluding 2 length bytes and type byte
	memcpy((void *) &buf[bp], (void *) &huff_tables->header_huffman_tables[1], len);
	bp+=len;

	memcpy((void *) &buf[bp], (void *) huff_tables->dht_dc1, 5); /// DHT DC1 header
	bp+=5;
	len= (huff_tables->dht_dc1[2]<<8)+huff_tables->dht_dc1[3]-3;  /// table length itself, excluding 2 length bytes and type byte
	memcpy((void *) &buf[bp], (void *) &huff_tables->header_huffman_tables[2], len);
	bp+=len;

	memcpy((void *) &buf[bp], (void *) huff_tables->dht_ac1, 5); /// DHT AC1 header
	bp+=5;
	len= (huff_tables->dht_ac1[2]<<8)+huff_tables->dht_ac1[3]-3;  /// table length itself, excluding 2 length bytes and type byte
	memcpy((void *) &buf[bp], (void *) &huff_tables->header_huffman_tables[3], len);
	bp+=len;

	/// copy SOS0 (constants combined with bayer shift for jp4diff/jp4hdr)
	header_sos=bp;
	buf[bp++]=0xff; buf[bp++]=0xda; /// SOS tag
	buf[bp++]=0;  /// high byte length - always 0
	switch (params->color) {
	case COLORMODE_MONO6:    /// monochrome, (4:2:0),
	case COLORMODE_COLOR:    /// color, 4:2:0, 18x18(old)
	case COLORMODE_COLOR20:  /// color, 4:2:0, 20x20, middle of the tile (not yet implemented)
	case COLORMODE_JP46:     /// jp4, original (4:2:0)
		HEADER_COPY_SOS(sos_color6);
		break;
	case COLORMODE_MONO4:   /// monochrome, 4 blocks (but still with 2x2 macroblocks)
		HEADER_COPY_SOS(sos_mono4);
		break;
	case COLORMODE_JP4:     /// jp4, 4 blocks
		HEADER_COPY_SOS(sos_jp4);
		break;
	case COLORMODE_JP46DC:   /// jp4, dc -improved (4:2:0)
		HEADER_COPY_SOS(sos_jp46dc);
		break;
	case COLORMODE_JP4DC:   /// jp4, 4 blocks, dc -improved
		HEADER_COPY_SOS(sos_jp4dc);
		break;
	case COLORMODE_JP4DIFF:   /// jp4, 4 blocks, differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
	case COLORMODE_JP4DIFF2:   /// jp4, 4 blocks, differential, divide differences by 2: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
		HEADER_COPY_SOS(sos_jp4diff);
		buf[header_sos+6+2*((4-params->byrshift) & 3)]=0; /// set huffman table 0 for the "base color"
		break;
	case COLORMODE_JP4HDR:   /// jp4, 4 blocks, differential HDR: red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
	case COLORMODE_JP4HDR2:   /// jp4, 4 blocks, differential HDR: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2)
		HEADER_COPY_SOS(sos_jp4diff); /// same as for COLORMODE_JP4DIFF
		buf[header_sos+6+2*((4-params->byrshift) & 3)]=0; /// set huffman table 0 for the "base color"
		buf[header_sos+6+2*((6-params->byrshift) & 3)]=0; /// set huffman table 0 for the HDR color
		break;
	}
	buf[bp++]=0x00; /// Spectral selection start
	buf[bp++]=0x3f; /// Spectral selection end
	buf[bp++]=0x00; /// Successive approximation (2 values 0..13)

	dev_dbg(g_dev_ptr, "JPEG header length = %d\n", bp);
	dev_dbg(g_dev_ptr, "list of parameters:\n");
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, p, 32);

	return bp; /// JPEG header length
}

int jpeghead_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);

	jpeghead_priv[chn].jpeg_h_sz = 0;
	inode->i_size=JPEG_HEADER_MAXSIZE; /// not the actual size
	return 0;
}

/*!=================================================================
 * Overloading lseek with additional functionality (to avoid ioctls)
 * with orig==SEEK_END lseek will treat (offset>0) as a byte pointer
 * in (char *)ccam_dma_buf_ptr of a frame pointer and use quality,
 * width and height to regenerate header.
 * frame pointers are 32-bytes aligned, so adding 1 to offest
 * will make sure it is always >0 (as offset=0, orig=SEEK_END
 * will just move pointer to the end and return file length.
 * 
 * When called with orig==SEEK_END, offset>0 lseek will position
 * file at the very beginning and return 0 if OK, -EINVAL if
 * frame header is not found for the specified offset
 *================================================================*/
loff_t jpeghead_lseek(struct file *file, loff_t offset, int orig,
		struct interframe_params_t *fp)
{
	int rp;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);

	dev_dbg(g_dev_ptr, "start processing LSEEK operation, minor = 0x%x, offset = 0x%llx, orig = 0x%x", minor, offset, orig);

	switch (orig)
	{
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		file->f_pos += offset;
		break;
	case SEEK_END:
		if (offset <= 0) {
			file->f_pos = jpeghead_priv[chn].jpeg_h_sz + offset;
		} else {
			file->f_pos = 0;                     // reset it to 0 anyway
			if ((fp->signffff != MARKER_FFFF) || // signature is overwritten
					((fp->timestamp_sec) & X313_LENGTH_MASK)) return -EINVAL; //! acquisition of this frame is not done yet - length word high byte is non-zero
			if ((offset & 0x1f) == 0x2)
				jpeghead_priv[chn].jpeg_h_sz = qtables_create(fp, jpeghead_priv[chn].header, chn);  /// just qunatization tables (128 bytes) - for the streamer
			else
				jpeghead_priv[chn].jpeg_h_sz = jpegheader_create(fp, jpeghead_priv[chn].header, chn); /// full JPEG header
			if (jpeghead_priv[chn].jpeg_h_sz < 0) {
				jpeghead_priv[chn].jpeg_h_sz = 0;
				return -EINVAL;                  // error in header
			}
			return ( file->f_pos );              // it is 0
		}
		break;
	default:
		return -EINVAL;
	}
	/// truncate position
	if (file->f_pos < 0) {
		file->f_pos = 0;
		return -EOVERFLOW;
	}

	if (file->f_pos > jpeghead_priv[chn].jpeg_h_sz) {
		file->f_pos = jpeghead_priv[chn].jpeg_h_sz;
	}
	return file->f_pos;
}

ssize_t jpeghead_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);

	dev_dbg(g_dev_ptr, "reading from jpeghead, minor = 0x%x, off = 0x%lld\n", minor, off);

	p = *off;
	if (p >= jpeghead_priv[chn].jpeg_h_sz)
		p = jpeghead_priv[chn].jpeg_h_sz;
	if ((p + count) > jpeghead_priv[chn].jpeg_h_sz) { /// truncate count
		count = jpeghead_priv[chn].jpeg_h_sz - p;
	}
	if (count) {
		if (copy_to_user(buf, &jpeghead_priv[chn].header[p], count))
			return -EFAULT;
		*off += count;
	}
	return count;
}


/**huffman_* file operations
 * write, read Huffman tables, initialize tables to default ones, program FPGA with the Huffman tables
 * file structure is the same as the struct huff_tables_t:
 * - 4 tables of 16+256 elements (16 frequencies followed by symbols)
 * - 2048 bytes (512 unsigned long) FPGA-encoded data - it is recalculated from the tables above
 * - 4 bytes - number of symbols in each table (calculated)
 */

int huffman_open(struct inode *inode, struct file *filp)
{
	inode->i_size = sizeof(struct huff_tables_t);

	return 0;
}

/*!=================================================================
 * Overloading lseek with additional functionality
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_DC0 - position at Huffman DC0
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_AC0 - position at Huffman DC0
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_DC1 - position at Huffman DC0
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_AC1 - position at Huffman DC0
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_FPGATAB - position at FPGA table
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_DEFAULT - fill in default tables
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_FPGACALC - calculate FPGA table
 * with orig==SEEK_END , offset==LSEEK_HUFFMAN_FPGAPGM - program FPGA table
 * those commands do not move the file pointer (return current),
 * or negative in the case of error (calculate FPGA table)
 *================================================================*/
loff_t huffman_lseek(struct file *file, loff_t offset, int orig)
{
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);

	dev_dbg(g_dev_ptr, "start processing LSEEK operation, minor = 0x%x, offset = 0x%llx, orig = 0x%x", minor, offset, orig);

	switch (orig)
	{
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		file->f_pos += offset;
		break;
	case SEEK_END:
		if (offset <= 0) {
			file->f_pos = sizeof(struct huff_tables_t) + offset;
		} else {
			switch (offset) {
			case LSEEK_HUFFMAN_DC0:     file->f_pos=0; break;
			case LSEEK_HUFFMAN_AC0:     file->f_pos=1*sizeof(struct huffman_encoded_t);break;
			case LSEEK_HUFFMAN_DC1:     file->f_pos=2*sizeof(struct huffman_encoded_t);break;
			case LSEEK_HUFFMAN_AC1:     file->f_pos=3*sizeof(struct huffman_encoded_t);break;
			case LSEEK_HUFFMAN_FPGATAB: file->f_pos=4*sizeof(struct huffman_encoded_t);break;
			case LSEEK_HUFFMAN_DEFAULT: jpeg_htable_init(chn); break; // no change to file pointer
			case LSEEK_HUFFMAN_FPGACALC:
				if (jpeg_htable_fpga_encode (chn) < 0) return -EINVAL;
				break;
			case LSEEK_HUFFMAN_FPGAPGM: jpeg_htable_fpga_pgm(chn); break;
			default: return -EINVAL;
			}
			return ( file->f_pos );
		}
		break;
	default:
		return -EINVAL;
	}
	// truncate position
	if (file->f_pos < 0) {
		file->f_pos = 0;
		return(-EOVERFLOW);
	}
	if (file->f_pos > sizeof(struct huff_tables_t)) file->f_pos = sizeof(struct huff_tables_t);
	return ( file->f_pos );
}

ssize_t huffman_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	unsigned char *uc_huff_tables = (unsigned char *) &jpeghead_priv[chn].huff_tables;

	dev_dbg(g_dev_ptr, "reading from huffman, minor = 0x%x, off = 0x%llx\n", minor, off);

	p = *off;
	if (p >= sizeof(struct huff_tables_t))
		p = sizeof(struct huff_tables_t);
	if ((p + count) > sizeof(struct huff_tables_t))
		count = sizeof(struct huff_tables_t) - p; /// truncate count
	if(count) {
		if (copy_to_user(buf, &uc_huff_tables[p], count)) return -EFAULT;
		*off += count;
	}
	return count;
}

ssize_t huffman_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int chn = minor_to_chn(minor, NULL);
	unsigned char * uc_huff_tables= (unsigned char *) &jpeghead_priv[chn].huff_tables;

	dev_dbg(g_dev_ptr, "writing to huffman, minor = 0x%x, off = 0x%llx\n", minor, off);

	p = *off;
	if (p >= sizeof(struct huff_tables_t))
		p = sizeof(struct huff_tables_t);
	if ((p + count) > sizeof(struct huff_tables_t))
		count = sizeof(struct huff_tables_t) - p; /// truncate count
	if (count) {
		if (copy_from_user(&uc_huff_tables[p], buf, count)) return -EFAULT;
	}

	return count;
}

/**
 * @brief Initialize Huffman tables with default data
 */
void jpeg_htable_init(unsigned int chn)
{
	unsigned char dc0[]={0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // number of codes of each length 1..16 (12 total)
			0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // symbols encoded (12)
			0x08, 0x09, 0x0a, 0x0b};

	unsigned char ac0[]={0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
			0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, // counts of codes of each length - 1..16 - total a2
			0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, // symbols encoded (0xa2)
			0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
			0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
			0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
			0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
			0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
			0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
			0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
			0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
			0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
			0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
			0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
			0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
			0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
			0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
			0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
			0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
			0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
			0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
			0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa};

	unsigned char dc1[]={0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
			0x08, 0x09, 0x0a, 0x0b};

	unsigned char ac1[]={0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
			0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
			0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
			0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
			0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
			0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
			0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
			0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
			0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
			0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
			0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
			0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
			0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
			0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
			0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
			0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
			0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
			0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
			0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
			0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
			0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
			0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa};

	struct huff_tables_t *huff_tables = &jpeghead_priv[chn].huff_tables;

	dev_dbg(g_dev_ptr, "initialize Huffman table with default data\n");

	memset ((void*) huff_tables, 0, sizeof(struct huff_tables_t));
	memcpy ((void*) huff_tables->header_huffman_tables[0].bits, dc0, sizeof(dc0));
	memcpy ((void*) huff_tables->header_huffman_tables[1].bits, ac0, sizeof(ac0));
	memcpy ((void*) huff_tables->header_huffman_tables[2].bits, dc1, sizeof(dc1));
	memcpy ((void*) huff_tables->header_huffman_tables[3].bits, ac1, sizeof(ac1));

	jpeg_htable_fpga_encode(chn);
}

/**
 * @brief Encode all 4 Huffman tables into FPGA format
 * additionally calculates number of symbols in each table
 * @return OK - 0, -1 - too many symbols, -2 bad table, -3 - bad table number 
 */
int jpeg_htable_fpga_encode(unsigned int chn)
{
	int ntab, i, rslt, a, length;
	const unsigned char dht_headers[20] = { /// length will be inserted later
			0xff, 0xc4, 0x00, 0x00, 0x00,
			0xff, 0xc4, 0x00, 0x00, 0x10,
			0xff, 0xc4, 0x00, 0x00, 0x01,
			0xff, 0xc4, 0x00, 0x00, 0x11 };
	struct huffman_fpga_code_t codes[256];
	unsigned long * icodes = (unsigned long *) codes;
	struct huff_tables_t *huff_tables = &jpeghead_priv[chn].huff_tables;

	dev_dbg(g_dev_ptr, "channel %d; encode all Huffman tables into FPGA format\n", chn);

	jpeghead_priv[chn].fpga_programmed = 0;
	/// Fill in the table headers:
	memcpy((void*) huff_tables->dht_all, (void*) dht_headers, sizeof(dht_headers)); /// all 4 headers (with zero length)
	for (ntab = 0; ntab < 4; ntab++) {
		dev_dbg(g_dev_ptr, "ntab = %d\n", ntab);
		memset(codes, 0, sizeof(codes));
		if ((rslt = jpeg_prep_htable(&huff_tables->header_huffman_tables[ntab], codes)) < 0) return rslt;
		if (ntab & 1) {
			a = ((ntab & 2) << 7);
			for (i = 0; i < 256; i += 16) {
				memcpy((void*) &(huff_tables->fpga_huffman_table[a]), (void*) &codes[i], 60); /// all but DC column
				a += 16;
			}
		} else {
			a = ((ntab & 2) << 7) + 0x0f; /// in FPGA DC use spare parts of AC table
			for (i = 0; i < 16; i++) {
				huff_tables->fpga_huffman_table[a] = icodes[i];
				a += 16;
			}
		}
		/// Fill in the table headers:
		length = 19; /// 2 length bytes, 1 type byte, 16 lengths bytes
		for (i = 0; i < 16; i++) length += huff_tables->header_huffman_tables[ntab].bits[i]; /// first 16 bytes in each table number of symbols
		huff_tables->dht_all[(5 * ntab) + 2] = length >> 8;  /// high byte (usually 0)
		huff_tables->dht_all[(5 * ntab) + 3] = length& 0xff; /// low  byte
	}

	dev_dbg(g_dev_ptr, "FPGA Huffman table:\n");
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, &huff_tables->fpga_huffman_table[0], sizeof(huff_tables->fpga_huffman_table));
	return 0;
}

/**
 * @brief Check if the FPGA is programmed to the new Huffman table
 * @param[in]   chn   compressor channel number
 * @return 1 - programmed, 0 - not programmed
 */
int jpeg_htable_is_programmed(unsigned int chn)
{
	return jpeghead_priv[chn].fpga_programmed;
}

/**
 * @brief program FPGA Huffman table (fram static array)
 * @param[in]   chn   compressor channel number
 * @return none
 */
void jpeg_htable_fpga_pgm(unsigned int chn)
{
	int i;
	unsigned long flags;
	x393_cmprs_table_addr_t table_addr;
	struct huff_tables_t *huff_tables = &jpeghead_priv[chn].huff_tables;

	table_addr.addr32 = 0;
	table_addr.type = 3;
	local_irq_save(flags);
	x393_cmprs_tables_address(table_addr, chn);
	for (i = 0; i < sizeof(huff_tables->fpga_huffman_table) / sizeof(huff_tables->fpga_huffman_table[0]); i++) {
		x393_cmprs_tables_data((u32)huff_tables->fpga_huffman_table[i], chn);
	}
	local_irq_restore(flags);
	jpeghead_priv[chn].fpga_programmed = 1;
}

/**
 * @brief Calculate huffman table (1 of 4) from the JPEG header to code length/value (for FPGA)
 *
 * The code of this function is based on jdhuff.c (from libjpeg)
 * @param htable encoded Huffman table - 16 length bytes followed by up to 256 symbols
 * @param hcodes combined (length<<16) | code table for each symbol
 * @return OK- 0, -1 - too many symbols, -2 bad table
 */
int jpeg_prep_htable(struct huffman_encoded_t *htable, struct huffman_fpga_code_t *hcodes)
{
	int p, i, l, si, numsymbols;
	unsigned int code;
	dev_dbg(g_dev_ptr, "calculate Huffman table from JPEG header\n");
	/// Figure C.1: make table of Huffman code length for each symbol
	p = 0;
	for (l = 1; l <= 16; l++) {
		i = htable->bits[l-1];
		if (i < 0 || p + i > 256) {
			dev_dbg(g_dev_ptr, "protect against table overrun\n");
			return -1 ; /// protect against table overrun
		}
		while (i--) hcodes[htable->huffval[p++]].length=l;
	}
	numsymbols = p;
	/// Figure C.2: generate the codes themselves
	/// We also validate that the counts represent a legal Huffman code tree.
	code = 0;
	si = hcodes[htable->huffval[0]].length;
	p = 0;
	///htable->huffval[N] - N-th symbol value
	while (p < numsymbols) {
		if ((hcodes[htable->huffval[p]].length < si) || (si>16)) {
			dev_err(g_dev_ptr, "bad table/bug\n");
			return -3; ///Bad table
		}
		while (hcodes[htable->huffval[p]].length == si) {
			hcodes[htable->huffval[p++]].value = code;
			code++;
		}

		/** code is now 1 more than the last code used for codelength si; but
		 * it must still fit in si bits, since no code is allowed to be all ones.
		 */
		if ( code >= (1 << si)) {
			dev_err(g_dev_ptr, "bad code\n");
			return -2; ///Bad code
		}
		code <<= 1;
		si++;
	}
	return 0;
}

int jpeghead_init(struct platform_device *pdev)
{
	int i;

	g_dev_ptr = &pdev->dev;
	for (i = 0; i < SENSOR_PORTS; i++) {
		jpeghead_priv[i].fpga_programmed = 0;
		jpeg_htable_init(i);
	}

	qt_init(pdev);
	dev_dbg(g_dev_ptr, "reset quantization tables\n");
	// force initialization at next access
	if (get_cache_policy() == COMMON_CACHE) {
		reset_qtables(0);
	} else if (get_cache_policy() == PER_CHN_CACHE) {
		for (i = 0; i < SENSOR_PORTS; i++)
			reset_qtables(i);
	}

	return 0;
}
