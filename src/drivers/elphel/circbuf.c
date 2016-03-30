/** @file circbuf.c
 *
 * @brief drivers to manipulate large circular buffer that holds compressed
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

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

//#include <asm/system.h>
//#include <asm/arch/memmap.h>
//#include <asm/svinto.h> obsolete
#include <asm/io.h>

/*#include <asm/arch/dma.h>
#include <asm/arch/hwregs/dma_defs.h>
#include <asm/arch/hwregs/dma.h>
#include <asm/arch/hwregs/reg_map.h>
#include <asm/arch/hwregs/bif_dma_defs.h>
*/

#include <asm/irq.h>
#include <asm/atomic.h>


#include <asm/delay.h>
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

#if ELPHEL_DEBUG
  #define MDF(x) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;}
  #define   D19(x) { if (GLOBALPARS(G_DEBUG) & (1 <<19)) {x; } ; }
  #define MDF19(x) { if (GLOBALPARS(G_DEBUG) & (1 <<19)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
  #define   D20(x) { if (GLOBALPARS(G_DEBUG) & (1 <<20)) {x; } ; }
  #define MDF20(x) { if (GLOBALPARS(G_DEBUG) & (1 <<20)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__ );x ;} }
#else
  #define MDF(x)
  #define   D19(x)
  #define MDF19(x)
  #define   D20(x)
  #define MDF20(x)

#endif

#define MD12(x)
#define D(x)

//#define MD7(x) printk("%s:%d:",__FILE__,__LINE__);x
#define MD7(x)

//#define MD10(x) printk("%s:%d:",__FILE__,__LINE__);x
#define MD10(x)
//#define MD11(x) printk("%s:%d:",__FILE__,__LINE__);x
#define MD11(x)

static const struct of_device_id elphel393_circbuf_of_match[];
/* really huge static DMA buffer (1288+8)*1032/3=445824 long-s */
// DMA_SIZE - in 32-bit words, not bytes
//static unsigned long ccam_dma_buf[CCAM_DMA_SIZE + (PAGE_SIZE>>2)] __attribute__ ((aligned (PAGE_SIZE)));
static unsigned long *ccam_dma_buf = NULL;
//!Without "static" system hangs after "Uncompressing Linux...
unsigned long  * ccam_dma_buf_ptr = NULL;
//EXPORT_SYMBOL_GPL(ccam_dma_buf_ptr);
//unsigned long * ccam_dma =         NULL; //! still used in autoexposure or something - why is in needed there?

int init_ccam_dma_buf_ptr(struct platform_device *pdev) {
	dma_addr_t dma_handle;
	const size_t dma_size = (CCAM_DMA_SIZE + (PAGE_SIZE >> 2)) * sizeof(int);
	struct device *dev = &pdev->dev;
    //ccam_dma_buf_ptr = ccam_dma_buf;
    //ccam_dma =         ccam_dma_buf; //&ccam_dma_buf[0]; Use in autoexposure

    // use Elphel_buf if it was allocated
	if (pElphel_buf != NULL) {
		ccam_dma_buf_ptr = pElphel_buf->vaddr;
		dma_handle = pElphel_buf->paddr;
		dev_info(dev, "using %d bytes of DMA memory from pElphel_buf at address 0x%08p", pElphel_buf->size * PAGE_SIZE, dma_handle);
	} else {
		ccam_dma_buf_ptr = dmam_alloc_coherent(dev, dma_size, &dma_handle, GFP_KERNEL);
		if (!ccam_dma_buf_ptr) {
			dev_err(dev, "unable to allocate DMA buffer\n");
			return -ENOMEM;
		} else {
			dev_info(dev, "%d bytes of DMA memory allocated at address 0x%08p", dma_size , dma_handle);
		}
	}
	ccam_dma_buf = ccam_dma_buf_ptr;

    return 0;
}
//extern struct interframe_params_t frame_params; // cc353.c
/*!======================================================================================
 *! Wait queue for the processes waiting for a new frame to appear in the circular buffer
 *!======================================================================================*/
wait_queue_head_t circbuf_wait_queue;
/*!=========================================================================================================
 *! circbuf top level device drivers. Minors are the same as before
 *! CMOSCAM_MINOR_CIRCBUF, CMOSCAM_MINOR_JPEAGHEAD - just a new major
 *!========================================================================================================*/
#define CIRCBUF_DRIVER_NAME "Elphel (R) Model 353 video buffer device driver"
static struct file_operations circbuf_fops = {
   owner:    THIS_MODULE,
   llseek:   circbuf_all_lseek,
   read:     circbuf_all_read,
   write:    circbuf_all_write,
   //ioctl:    circbuf_all_ioctl,
   open:     circbuf_all_open,
   mmap:     circbuf_all_mmap,
   poll:     circbuf_all_poll,
   release:  circbuf_all_release
};

// Read/write to circular buffer.  Needed to find out what Axis DMA is doing
// also - jpeg header
struct circbuf_pd {
    int       minor;        /// should be the first, same as in jpeghead_pd
    int       daemon_bit;   /// poll() will produce POLLHUP if this bit is >=0 (set through lseek (, LSEEK_DAEMON_CIRCBUF,SEEK_END)
                            /// and the corresponding bit in P_DAEMON_EN goes 0
    int       imageWidth;   /// image width to compare to current. G_SKIP_DIFF_FRAME
    int       imageHeight;  /// image height to compare to current. G_SKIP_DIFF_FRAME
    int       tolerated;    /// number of frames with different size tolerated
    struct wait_queue *circbuf_wait_queue; ///NOTE: not used at all?
// something else to be added here?
};
// CMOSCAM_MINOR_HUFFMAN // huffman tables R/W
int circbuf_all_open(struct inode *inode, struct file *filp) {
   int res;
 MD10(printk("circbuf_all_open, minor=0x%x\n",MINOR(inode->i_rdev)));
  switch (MINOR(inode->i_rdev)) {
    case  CMOSCAM_MINOR_CIRCBUF :
        res=circbuf_open(inode,filp);
        break;
    case  CMOSCAM_MINOR_JPEAGHEAD :
        res=jpeghead_open(inode,filp);
        break;
    case  CMOSCAM_MINOR_HUFFMAN :
        res=huffman_open(inode,filp);
        break;
    default:
//       kfree(filp->private_data); // already allocated
       return -EINVAL;
  }
  return res;
}
int circbuf_all_release(struct inode *inode, struct file *filp) {
  int res=0;
  int p = MINOR(inode->i_rdev);
 MD10(printk("circbuf_all_release, minor=0x%x\n",p));
  switch ( p ) {
    case  CMOSCAM_MINOR_CIRCBUF :
//        res=circbuf_release(inode,filp);
        break;
    case  CMOSCAM_MINOR_JPEAGHEAD :
//        res=jpeghead_release(inode,filp);
        break;
    case  CMOSCAM_MINOR_HUFFMAN :
//        res=huffman_release(inode,filp);
        break;
    default:
        return -EINVAL; //! do not need to free anything - "wrong number"
    }
  if (filp->private_data) kfree(filp->private_data);
  return res;
}

loff_t circbuf_all_lseek(struct file * file, loff_t offset, int orig) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
     struct interframe_params_t *fp = NULL;
     int rp;

 MD10(printk("circbuf_all_lseek, minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case CMOSCAM_MINOR_CIRCBUF :    return  circbuf_lseek  (file, offset, orig);
       case CMOSCAM_MINOR_JPEAGHEAD :
    	   if (orig == SEEK_END && offset > 0) {
			   rp= (offset >>2) & (~7); // convert to index to long, align to 32-bytes
			   fp = (struct interframe_params_t *) &ccam_dma_buf_ptr[X313_BUFFSUB(rp, 8)]; //! 32 bytes before the frame pointer, may roll-over to the end of ccam_dma_buf_ptr
    	   }
    	   return  jpeghead_lseek (file, offset, orig, fp);
       case CMOSCAM_MINOR_HUFFMAN :    return  huffman_lseek  (file, offset, orig);
       default:                        return -EINVAL;
     }
}

ssize_t circbuf_all_read(struct file * file, char * buf, size_t count, loff_t *off) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
 MD10(printk("circbuf_all_read, minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case CMOSCAM_MINOR_CIRCBUF :      return  circbuf_read   (file, buf, count, off);
       case CMOSCAM_MINOR_JPEAGHEAD :    return  jpeghead_read  (file, buf, count, off);
       case CMOSCAM_MINOR_HUFFMAN :      return  huffman_read  (file, buf, count, off);
       default:                          return -EINVAL;
    }
}
ssize_t circbuf_all_write(struct file * file, const char * buf, size_t count, loff_t *off) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
 MD10(printk("circbuf_all_write, minor=0x%x, count=%d, off=%d\n",privData-> minor, (int) count,(int)*off));
     switch (privData->minor) {
       case CMOSCAM_MINOR_CIRCBUF :       return  circbuf_write  (file, buf, count, off);
//       case CMOSCAM_MINOR_JPEAGHEAD :     return  jpeghead_write (file, buf, count, off); // same as other - write header
       case CMOSCAM_MINOR_HUFFMAN :       return  huffman_write  (file, buf, count, off);
       default:                           return -EINVAL;
    }
}


int circbuf_all_mmap (struct file *file, struct vm_area_struct *vma) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
 MD10(printk("circbuf_all_mmap, minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case  CMOSCAM_MINOR_CIRCBUF :   return  circbuf_mmap   (file, vma);
       default: return -EINVAL;
    }
}

unsigned int circbuf_all_poll (struct file *file, poll_table *wait) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
 MD10(printk("circbuf_all_poll, minor=0x%x\n",privData-> minor));
     switch (privData->minor) {
       case  CMOSCAM_MINOR_CIRCBUF :
          return circbuf_poll (file, wait);
       default:                           return -EINVAL;
    }
}

int circbuf_all_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) {
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) filp->private_data;
     printk("\n========== IOCTL is not implemented in circbuf_all_ioctl, minor=0x%x, cmd=0x%x, _IOC_NR(cmd)=0x%x, arg=0x%x\n",privData-> minor, (int) cmd, _IOC_NR(cmd), (int) arg);
     return -EINVAL;
}




int circbuf_open(struct inode *inode, struct file *filp) { // set filesize
   struct circbuf_pd * privData;
   privData= (struct circbuf_pd *) kmalloc(sizeof(struct circbuf_pd),GFP_KERNEL);
   if (!privData) return -ENOMEM;
   filp->private_data = privData;
   privData-> minor=MINOR(inode->i_rdev);

   inode->i_size = ((CCAM_DMA_SIZE) << 2);
 MD10(printk("circbuf_open, inode->i_size=0x%x\n", (int)inode->i_size));

//!should be removed (or else you can not ftp file - it will start from WP)
//   circbuf_lseek(filp, LSEEK_CIRC_LAST, SEEK_END); //! position at last acquired frame, ignore result
   return 0;
}

/*!=============================================================================================
 *! Overloading lseek with additional functionality (to avoid ioctls)
 *! with orig==SEEK_END lseek will treat (offset>0) as a command
 *! to manipulate frame pointer(s) or wait for the image to be ready
 *! using these commands
 *!  LSEEK_CIRC_TORP .- set filepointer to global (shared) read pointer
 *!  LSEEK_CIRC_TOWP  - set filepointer to FPGA write pointer (next frame to be acquired)
 *!  LSEEK_CIRC_PREV  - move pointer to the previous frame, return -EOVERFLOW if there are none
 *!  LSEEK_CIRC_NEXT  - advance pointer to the next frame, return -EOVERFLOW if was already
 *!      at the last
 *!  LSEEK_CIRC_LAST  - move pointer to the last acquired frame (default after open)
 *!      (it is combination of 2+3)
 *!  LSEEK_CIRC_FIRST - move pointer to the first acquired frame. It s not safe to rely
 *!                    on this pointer if more frames are expected - next incoming frame
 *!                    can overwrite this one.
 *!  LSEEK_CIRC_SCND -  move pointer to the second oldest acquired frame. A slightly safer
 *!                    to use instead of LSEEK_CIRC_FIRST when constant acquisition is on
 *!                    and sensor provides new frames - this frame will likely survive longer
 *!  LSEEK_CIRC_SETP -  save current pointer to global read pointer
 *!  LSEEK_CIRC_VALID - verify that the frame at current location is valid (not overrun in the buffer)
 *!                    Returns file pointer if valid, else - -1
 *!  LSEEK_CIRC_READY - verify frame at current loacation is available (valid and acquired)
 *!                    Returns file pointer if ready, else - -1
 *!  LSEEK_CIRC_WAIT - sleep until next frame is acquired 
 *! All commands but (LSEEK_CIRC_TOWP,LSEEK_CIRC_LAST,LSEEK_CIRC_FIRST) will return -EINVAL if read 
 *! pointer is not valid  (i.e buffer was overrun and data pointed is lost). if success they return
 *! the current (byte *) to the start of the frame data (parameters are at
 *! offsett =-32 from it)
 *! (0, SEEK_CUR) also verifies that the header is not overwritten. It can be used
 *! after buffering frame data to verify you got it all correctly
 *! SEEK_CUR also supports the circular nature of the buffer and rolls over if needed
 *! Additional commands for SEEK_END (they _DO_ modify the current file pointer !)
 *!  LSEEK_CIRC_FREE - returnes remaining memory in circbuf from the current file pointer,
 *!                   or -EINVAL if the pointer is invalid. As this command uses the buffer write pointer
 *!                   that is updated only when the complete frame is in the buffer, the actual
 *!                   free memory may be less by a whole frame if compressor is running.
 *!  LSEEK_CIRC_USED - returnes memory used in the in circbuf from the current file pointer,
 *!                   or -EINVAL if the pointer is invalid
 *!=============================================================================================*/
//!returns 0 if rp is a valid read ponter
//! returns 1 if there is a frame at this address
//! returns 0 if the pointer is for the frame yet to be acquired
//! returns -1 if there is no frame at this index
//! returns -2 if the rp is not 32-bytes aligned
//!sets *fpp to the frame header, including signature and length
int circbufValidPointer(int rp, struct interframe_params_t ** fpp) {
  if (rp & 0x1f) {  //!rp is not 32-bytes aligned
       MD10(printk("circbufValidPointer: misaligned pointer\n"));
       return -2;
  }
  int wp=camSeqGetJPEG_wp();
  int p= rp >> 2; // index inccam_dma_buf
  struct interframe_params_t * fp;
  fp = (struct interframe_params_t *) &ccam_dma_buf[X313_BUFFSUB(p, 8)]; //! 32 bytes before the frame pointer, may roll-over to the end of ccam_dma_buf

 MD10(printk("rp=0x%x (0x%x), offset=0x%x\n",rp,p,(int)&ccam_dma_buf[p]-(int)fp); dumpFrameParams(fp, "in circbufValidPointer:"));

  *fpp=fp;
 MD11(printk("p=0x%x , wp==0x%x\n",p,wp));
 if (p == wp) {
     return 0; // frame not yet acquired, fp - not valid
  }
  if (fp->signffff != 0xffff) { //! signature is overwritten
     MD10(printk("circbufValidPointer: signanure overwritten\n"));
     return -1;
  }
  if ((fp->timestamp_sec) & X313_LENGTH_MASK) {
     MDF(printk ("Should not get here - fp->timestamp_sec=0x%x\n",(int) fp->timestamp_sec));
     return 0; //! should not get here - should be caught by (p==wp)
  }
  return 1;
   
}

loff_t circbuf_lseek(struct file * file, loff_t offset, int orig) {
 //  orig 0: position from begning
 //  orig 1: relative from current
 //  orig 2: position from last  address
   int l = (CCAM_DMA_SIZE << 2);
   int fl=0;// frame length
   struct interframe_params_t * fp;
   int fvld=-1;
   int rp, bp, prev_p, preprev_p; //, p;
   int nf;   //! number of frames;
   int nz=1; //! number of crossing of start of the circular buffer (counter to prevent looping forever)
MD12(int dbg_i);
//   int pf; // previous frame
 MD11(printk("circbuf_lseek, offset=0x%x, orig=0x%x\n",(int) offset, (int) orig));
   switch(orig) {
   case SEEK_SET:
     file->f_pos = offset;
     break;
   case SEEK_CUR:
     if (offset) file->f_pos += offset;
     else if (circbufValidPointer(file->f_pos, &fp) <0 ) return -EINVAL; //!no frames at the specified location or pointer is not 32-byte aligned
     break;
   case SEEK_END:
     if (offset <= 0) {
        file->f_pos = l + offset;
     } else { //! New functionality
//!verify the frame pointer
        switch (offset) {
          case LSEEK_CIRC_TORP:
              file->f_pos=camSeqGetJPEG_rp()<<2; //! set file pointer to global read pointer, and proceed
          case LSEEK_CIRC_PREV:
          case LSEEK_CIRC_NEXT:
          case LSEEK_CIRC_SETP:
          case LSEEK_CIRC_VALID:
          case LSEEK_CIRC_READY:
          case LSEEK_CIRC_FREE:
          case LSEEK_CIRC_USED:
             if (((fvld=circbufValidPointer(file->f_pos, &fp))) <0 )
                return -EINVAL; //!no frames at the specified location
        }
        switch (offset) {
          case LSEEK_CIRC_FREE:
             bp=(file->f_pos - (camSeqGetJPEG_wp()<<2));
//             return (bp>0)?bp:(bp+l); //!will return full buffer size if current pointer is a write pointer (waiting for the next frame)
             return (file->f_pos=(bp>0)?bp:(bp+l)); //!Has a side effect of moving a file pointer!
          case LSEEK_CIRC_USED:
             bp=((camSeqGetJPEG_wp()<<2) - file->f_pos);
//             return (bp>=0)?bp:(bp+l); //!will return 0 if current pointer is a write pointer (waiting for the next frame)
             return (file->f_pos=(bp>0)?bp:(bp+l)); //!Has a side effect of moving a file pointer!
          case LSEEK_CIRC_TORP:
             break;
          case LSEEK_CIRC_TOWP:
             file->f_pos=camSeqGetJPEG_wp()<<2; // no checking if it is valid
             break;
          case LSEEK_CIRC_LAST:
             file->f_pos=camSeqGetJPEG_wp()<<2;
             fvld=circbufValidPointer(file->f_pos, &fp); //!set fp
          case LSEEK_CIRC_PREV:
             rp= file->f_pos >> 2;
             fl=ccam_dma_buf[X313_BUFFSUB(rp, 9)] ^ X313_LENGTH_MASK;
 MD11(printk("LSEEK_CIRC_PREV: rp=0x%x, fvld=%d, fl=0x%x\n", rp, fvld,fl));
             if (fl & X313_LENGTH_MASK) {
                if (offset==LSEEK_CIRC_LAST) break; // just don't move pointer, leave it at write pointer and return no error
                return -EOVERFLOW; //! no frames before current
             }
             bp = (X313_BUFFSUB(rp, X313_PADDED_FRAME(fl))<<2); // in bytes
 MD11(printk("LSEEK_CIRC_PREV: bp=0x%x (0x%x), circbufValidPointer=%d\n", bp, bp>>2,circbufValidPointer(rp>>2, &fp)));
             if (circbufValidPointer(bp, &fp) < 0 ) { //! no valid frames before current
                if (offset==LSEEK_CIRC_LAST) break; // just don't move pointer, leave it at write pointer and return no error
                return -EOVERFLOW; //! no frames before current
             }
 //! move frame pointer only if there is a valid frame there
             file->f_pos=bp;
             break;
          case LSEEK_CIRC_NEXT:
 MD11(printk("LSEEK_CIRC_NEXT: rp=0x%x, fvld=%d, fp->timestamp_sec=0x%x\n", file->f_pos >> 2, fvld, fp->timestamp_sec));
             if (fvld <=0) 
                return -EOVERFLOW; //! no frames after current
             file->f_pos = X313_BUFFADD(file->f_pos >> 2, X313_PADDED_FRAME(fp->timestamp_sec)) <<2 ;// do it even if the next frame does not yet exist
             break;
          case LSEEK_CIRC_FIRST:
          case LSEEK_CIRC_SCND:
//! Starting from the write pointer do be able to count all the frames in the buffer
             rp=camSeqGetJPEG_wp();
             prev_p=rp;
             preprev_p=prev_p; // for second
             nf=0;
             nz=1;
             file->f_pos=rp<<2;
             while ((((fvld=circbufValidPointer(rp<<2, &fp))) >= 0) & (nz>=0)) {
               nf++;
//               file->f_pos=rp<<2;
               preprev_p=prev_p; //! second known good (at least first one)
               prev_p=rp;        //!now - current, known good
               fl=ccam_dma_buf[X313_BUFFSUB(rp, 9)] ^ X313_LENGTH_MASK;
 MD11(printk("\nf=%d, rp=0x%x, fvld=%d, fl=0x%x\n",nf, rp, fvld, fl));
               if (fl & X313_LENGTH_MASK) break;  //! no frames before rp (==prev_p)
//! move rp to the previous frame
               rp= X313_BUFFSUB(rp, X313_PADDED_FRAME(fl));
               if (rp > prev_p) nz-- ; // rolled through zero - make sure we'll not stuck in this loop forever
             }

 MD11(printk("after while{}: nf=%d, rp=0x%x, fvld=%d, fl=0x%x\n",nf, rp, fvld, fl));
             file->f_pos=((offset==LSEEK_CIRC_SCND)?preprev_p:prev_p) << 2;
             break;
          case LSEEK_CIRC_SETP:
             camSeqSetJPEG_rp(file->f_pos>>2);
             break;
          case LSEEK_CIRC_VALID:
             break;
          case LSEEK_CIRC_READY:
             if (fvld <=0) return -EINVAL; //! no frame is available better code?
             break;
          case LSEEK_CIRC_WAIT:
              while (((fvld=circbufValidPointer(file->f_pos, &fp)))==0) { //! only while not ready, ready or BAD - return
                wait_event_interruptible(circbuf_wait_queue,(camSeqGetJPEG_wp()<<2)!=file->f_pos);
              }
              if (fvld < 0) return -ESPIPE;      //!invalid seek - have better code?
              return file->f_pos ; //! data already available, return file pointer
          default:
            if ((offset & ~0x1f)==LSEEK_DAEMON_CIRCBUF) {
              wait_event_interruptible(circbuf_wait_queue, get_imageParamsThis(P_DAEMON_EN) & (1<<(offset & 0x1f)));
            }
        }
        return ( file->f_pos ); //! file position >=0
     }
     break;
   default:
      return -EINVAL;
   }
   // roll-over position
   while (file->f_pos < 0) file->f_pos+=l;
   while (file->f_pos > l) file->f_pos-=l;
   if ((orig !=SEEK_END) && (file->f_pos == l)) file->f_pos=0; //!only for lseek (fd,0,SEEK_END) the result will be file size, others will roll to 0
   return  file->f_pos ;
}

/**
 * @brief This function handles write operations for circbuf file
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer containing data
 * @param[in]   count number of bytes in buffer
 * @param[in]   off   offset
 * @return      number of bytes read form \e buf
 */
ssize_t circbuf_write(struct file * file, const char * buf, size_t count, loff_t *off) {
	void __iomem *mmio;
	x393_afimux_status_t val;
	int port;

	unsigned long p;
	char *char_pb = (char *)ccam_dma_buf;
	struct circbuf_pd *priv = file->private_data;

	// convert char to number
	port = buf[0] - 0x30;
	/*mmio = ioremap(0x40002060, 16);
	if (!mmio) {
		printk(KERN_DEBUG "ERROR: can not ioremap region");
		return count;
	}*/
	if (init_mmio_ptr() < 0) {
		printk(KERN_DEBUG "ERROR: can not remap IO region\n");
	}
	if (port >= 0 && port < 4) {
		val = x393_afimux0_status(port);
		//val.d32 = readl((void*) (0x40002060 + 0x4 * port));
		//val.d32 = ioread32(mmio + 0x4 * port);
	} else {
		printk(KERN_DEBUG "Unrecognized port number\n");
	}

	printk(KERN_DEBUG "AFI MUX0 port: %d, AFI MUX0 offset: 0x%x, AFI MUX0 sequence number: %d\n", port, val.offset256 * 32, val.seq_num);
	printk(KERN_DEBUG "AFI MUX0 raw value: 0x%x\n", val.d32);
	printk(KERN_DEBUG "AFI MUX0 offset265: 0x%x, seq_num: 0x%x\n", val.offset256, val.seq_num);
	iounmap(mmio);
	mmio = NULL;
	return count;

	D(printk("circbuf_write\n"));
	/// ************* NOTE: Never use file->f_pos in write() and read() !!!
	p = *off;
	if(p >= (CCAM_DMA_SIZE << 2))
		p = (CCAM_DMA_SIZE << 2);
	if((p + count) > (CCAM_DMA_SIZE << 2)) { // truncate count
		count = (CCAM_DMA_SIZE << 2) - p;
	}
	if(count) {
		if(copy_from_user(&char_pb[p], buf, count))
			return -EFAULT;
		*off += count;
	}
	return count;
}

/**
 * @brief This function handles read operations for circbuf file
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer where data will be written to
 * @param[in]   count number of bytes written to \e buf
 * @param[in]   off   offset
 * @return      number of bytes written to \e buf
 */
ssize_t circbuf_read(struct file * file, char * buf, size_t count, loff_t *off) {
  unsigned long p;
  char * char_pb = (char *) ccam_dma_buf;
  struct circbuf_pd *priv = file->private_data;

  printk(KERN_DEBUG "%s\n", __func__);
  p = *off;
  D(printk("circbuf_read pos=%d,count=%d, off=%d\r\n",p,count,off ));
  if (p >= (CCAM_DMA_SIZE<<2))  p = (CCAM_DMA_SIZE<<2);
  if( (p + count) > (CCAM_DMA_SIZE<<2)) { // truncate count
    count = (CCAM_DMA_SIZE<<2) - p;
  }
  if (count) {
    if (copy_to_user(buf, &char_pb[p], count)) return -EFAULT;
//    file->f_pos+=count;
    *off+=count;
  }
  return count;
}

int   circbuf_mmap   (struct file *file, struct vm_area_struct *vma) {

     int rslt;

     MD7(printk("vm_start=%lx\r\n",vma->vm_start));
     MD7(printk("vm_end=%lx\r\n",vma->vm_end));
     MD7(printk("vm_pgoff=%lx\r\n",vma->vm_pgoff));
     MD7(printk("vm_file=%lx\r\n",(unsigned long) (vma->vm_file)));
     MD7(printk("ccam_dma_buf=%lx\r\n",(unsigned long) (virt_to_phys (&ccam_dma_buf[0]))));
   /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
     rslt=remap_pfn_range(vma,
             vma->vm_start,
//             ((unsigned long)(&ccam_dma_buf[0])) >> PAGE_SHIFT, // Should be page-aligned
             ((unsigned long) virt_to_phys(&ccam_dma_buf[0])) >> PAGE_SHIFT, // Should be page-aligned
             vma->vm_end-vma->vm_start,
             vma->vm_page_prot);

     MD7(printk("remap_pfn_range returned=%x\r\n",rslt));
     if (rslt) return -EAGAIN;
//   vma->vm_ops = &simple_remap_vm_ops;
//   simple_vma_open(vma);
     return 0;
}
/*!===========================================================================
 *! If the current read pointer is invalid, circbuf_poll returns POLLHUP
 *! as no data will be ever available until file poinetr is reset.
 *! if it is valid, wait is setup and the blocking condition occurs 
 *! ifthe current file pointer is equal to the FPGA write pointer
 *!===========================================================================*/
unsigned int circbuf_poll (struct file *file, poll_table *wait) {
     struct interframe_params_t * fp;
     struct circbuf_pd * privData;
     privData = (struct circbuf_pd *) file->private_data;
     int rslt; //!result of testing read poinetr
 MD10(printk("circbuf_poll\n"));
     rslt= circbufValidPointer(file->f_pos, &fp);
     if        (rslt < 0) { //! not a valid read pointer, probable buffer overrun 
 MD10(printk("circbuf_poll:invalid pointer\n"));
       return  POLLHUP ;
     } else if (rslt > 0) {
       return POLLIN | POLLRDNORM; //! there was frame already available
     } else { //! pointer valid, no frame yet
       poll_wait(file, &circbuf_wait_queue, wait);
//! Frame might become available during call to poll_wait so nobody will wake us up.
//! Let's see if there is stillno frame
	    if ((camSeqGetJPEG_wp()<<2)!=file->f_pos) return POLLIN | POLLRDNORM; //! we are lucky - got it
    }
    return 0; // nothing ready
}

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

   MDF19(printk("\n"));
   res = register_chrdev(CIRCBUF_MAJOR, "circbuf_operations", &circbuf_fops);
   if(res < 0) {
     printk(KERN_ERR "\ncircbuf_all_init: couldn't get a major number %d.\n",CIRCBUF_MAJOR);
     return res;
   }

   res = init_ccam_dma_buf_ptr(pdev);
   if (res < 0) {
	   dev_err(dev, "ERROR allocating coherent DMA buffer\n");
	   return -ENOMEM;
   }

   MDF19(printk("init_waitqueue_head()\n"));
   init_waitqueue_head(&circbuf_wait_queue);
   MDF19(printk("jpeg_htable_init()\n"));
   jpeg_htable_init ();         /// set default Huffman table, encode it for the FPGA
   dev_info(dev, "registered MAJOR: %d\n", CIRCBUF_MAJOR);

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
