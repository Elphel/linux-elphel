// FILE NAME  : cxsdma.h
// read/write image and FPN buffers from SDRAM

#ifndef _CIRCBUF_H
#define _CIRCBUF_H

#include <linux/poll.h>

int          circbuf_all_open  (struct inode *inode,  struct file *filp); // set filesize
int          circbuf_all_release(struct inode *inode, struct file *filp);
loff_t       circbuf_all_lseek (struct file * file,   loff_t offset, int orig);
ssize_t      circbuf_all_write (struct file * file,   const char * buf, size_t count, loff_t *off);
ssize_t      circbuf_all_read  (struct file * file,   char * buf, size_t count, loff_t *off);
int          circbuf_all_mmap  (struct file *file,    struct vm_area_struct *vma);
unsigned int circbuf_all_poll  (struct file *file,    poll_table *wait);
//!just to notify it is not implemented
int          circbuf_all_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);



int          circbuf_open  (struct inode *inode, struct file *filp); // set filesize
loff_t       circbuf_lseek (struct file * file, loff_t offset, int orig);
ssize_t      circbuf_write (struct file * file, const char * buf, size_t count, loff_t *off);
ssize_t      circbuf_read  (struct file * file, char * buf, size_t count, loff_t *off);
int          circbuf_mmap  (struct file *file, struct vm_area_struct *vma);
unsigned int circbuf_poll  (struct file *file,    poll_table *wait);

void init_ccam_dma_buf_ptr(void);
/*!======================================================================================
 *! Wait queue for the processes waiting for a new frame to appear in the circular buffer
 *!======================================================================================*/
extern wait_queue_head_t circbuf_wait_queue;

#endif /* _CIRCBUF_H */
