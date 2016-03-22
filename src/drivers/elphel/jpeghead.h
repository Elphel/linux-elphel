// FILE NAME  : jpeghead.h
///Structure used for FPGA data - lower 16 bits - code value, next 5 - code length (1..16).
///Actually FPGA only uses 20 bits, length 0 is interpreted as 16
struct huffman_fpga_code_t {
  unsigned short value;       /// code value
  unsigned short length;      /// code length
};
int     qtables_create  (struct interframe_params_t * params, unsigned char * buf);
int     jpegheader_create(struct interframe_params_t * params, unsigned char * buf);
int     jpeghead_open   (struct inode *inode, struct file *filp); // set filesize
loff_t  jpeghead_lseek  (struct file * file, loff_t offset, int orig);
ssize_t jpeghead_read   (struct file * file, char * buf, size_t count, loff_t *off);

int     huffman_open   (struct inode *inode, struct file *filp); // set filesize
int     huffman_release(struct inode *inode, struct file *filp);
loff_t  huffman_lseek  (struct file * file, loff_t offset, int orig);
ssize_t huffman_read   (struct file * file, char * buf, size_t count, loff_t *off);
ssize_t huffman_write  (struct file * file, const char * buf, size_t count, loff_t *off);


extern unsigned long  * ccam_dma_buf_ptr;
//void init_ccam_dma_buf_ptr(void);
#define JPEG_HEADER_MAXSIZE    0x300
struct jpeghead_pd {
    int                minor;/// should be the first, same as in circbuf_pd
    unsigned long      size; /// JPEG header size (no Exif)
    unsigned char      header[JPEG_HEADER_MAXSIZE];
};

struct huffman_pd {
    int                minor;/// should be the first, same as in circbuf_pd
};

int  jpeg_htable_is_programmed(void);
void jpeg_htable_init (void);
int  jpeg_htable_fpga_encode (void);
void jpeg_htable_fpga_pgm (void);
int  jpeg_prep_htable (struct huffman_encoded_t * htable, struct huffman_fpga_code_t * hcodes);
