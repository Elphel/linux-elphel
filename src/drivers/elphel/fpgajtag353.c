/**************************************************************************//**
 * @file  fpgajtag353.c
 * @brief TBD
 * @copyright Copyright 2002-20016 (C) Elphel, Inc.
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * @par <b>License</b>*
 */
#undef DEBUG
/****************** INCLUDE FILES SECTION ***********************************/

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/ioport.h> // needed?
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
//#include <linux/poll.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

//#include <linux/interrupt.h>
//#include <linux/spinlock.h>

#include <asm/io.h>
//#include <asm/system.h>
#include <asm/irq.h>

#include <uapi/elphel/x393_devices.h>
#include <uapi/elphel/c313a.h> // for #define SENSOR_PORTS 4

//#include <elphel/fpgaconfa.h> //defines for fpga_state fields


//#include <asm/delay.h>
#include <asm/uaccess.h>

//#include "fpgactrl.h" // extern fpga_state, defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h"     // FPGA registers and macros

#include "x393.h"


//#include <asm/fpgaconfa.h>


//#define JTAG_DISABLE_IRQ y

#define D(x)
//#define D(x) printk("%s:%d:",__FILE__,__LINE__);x
#define PARALLEL_JTAG
/*
port C 353:
 0 - TDO  (in)
 1 - TDI  (out)
 2 - TMS  (out)
 3 - TCK  (out)
 4 - NC (was INIT (i/o) )
 5 - DONE (in)
 6 - RSTBTN
 7 - PGM  (out)
 */
#define FPGAJTAG_TDO_BIT 0
#define FPGAJTAG_TDI_BIT 1
#define FPGAJTAG_TMS_BIT 2
#define FPGAJTAG_TCK_BIT 3
#define FPGAJTAG_DONE_BIT 5
#define FPGAJTAG_RSTBTN_BIT 6
#define FPGAJTAG_PGM_BIT 7


#ifndef XC2S300E_BITSIZE
#define XC3S1000_BITSIZE  3223488
#define XC3S1200E_BITSIZE 3841189
#define XC3S1200E_BOUNDARY_SIZE 772
//  #define XC3S1200E_BOUNDARY_SIZE 812
#define FJTAG_BUF_SIZE 0x77000
#define FJTAG_MAX_HEAD 0x1000
#define FJTAG_RAW_WSIZE 0x40000 // shared with  bitstream buffer
#define FJTAG_RAW_RSIZE 0x30000 // shared with  bitstream buffer
#define FJTAG_IDSIZE    0x40    // bits - ID and User

#endif

#define FPGA_JTAG_DRIVER_DESCRIPTION "Elphel (R) model 393 FPGA (Xilinx (R) XC3S1200E) configuration driver"
#define FPGA_JTAG_MAXMINOR 16 // 10


#define JTAG_RAW         0 // raw JTAG access to any FPGA
#define JTAG_MAIN_FPGA   1 // main board FPGA access  (10353)
#define JTAG_SENSOR_FPGA 2 // sensor board FPGA access (10347, 10359) 
#define JTAG_AUX_FPGA    3 // 
//#define JTAG_NCHANNELS   4
#define JTAG_NCHANNELS   16 // 4 << 2

//#define JTAG_SENSOR_OFFSET   4 // Sensor ports minors start (4..7) - (2 LSB should be 0)
//#define JTAG_SENSOR_CHANNELS 4 // Number of sensor ports for JTAG


#define JTAG_MODE_CLOSED   0 // JTAG channel is closed
#define JTAG_MODE_RDID     1 // JTAG channel read ID
#define JTAG_MODE_PGM      2 // JTAG channel PROGRAM
#define JTAG_MODE_BOUNDARY 3 // JTAG channel boundary scan (just opened, will become one of the 2:JTAG_MODE_SAMPLE, JTAG_MODE_EXTEST
#define JTAG_MODE_SAMPLE   4 // JTAG channel boundary scan - sample (the first operation after open is read)
#define JTAG_MODE_EXTEST   5 // JTAG channel boundary scan - EXTEST (the first operation after open is read)
#define JTAG_MODE_RAW      6 // JTAG raw command mode

// configuration and raw minors use whole buffer, ID and boundary can be opened at the same time
struct JTAG_channel_t {
    int mode; // 0..5 -JTAG_MODE_CLOSED...JTAG_MODE_EXTEST
    unsigned char * dbuf; // data buffer (shared, boundary mode use different parts)
    int sizew;            // byte size that can be written
    int sizer;            // byte size that can be read
    int bitsw;            // bit size to be written
    int bitsr;            // bit size to be read
    int wp;               // byte pointer for file write
    int rp;               // byte pointer for file read
    int wdirty;           // some data is buffered but not yet sent out in EXTEST mode
};
static unsigned char  bitstream_data[FJTAG_BUF_SIZE];    // will fit bitstream and the header (if any). Also used for boundary write
//static unsigned short *raw_fifo_w= (unsigned short) &bitstream_data[0];
static unsigned char *raw_fifo_w=  &bitstream_data[0];
static unsigned char *raw_fifo_r=  &bitstream_data[FJTAG_RAW_WSIZE];
static struct  JTAG_channel_t JTAG_channels[JTAG_NCHANNELS];
// boundary scan is read always at open. written - at close (only if there were any writes)
static int data_modified=0;

//static reg_gio_rw_pc_dout  pc_dout;
#define PC_DOUT_INITIAL 0
//static	int buf8i=0;	// current buffer length (in bytes!)
//static  int datastart=0;
//static int prev32;
//static int prev64;

//static int fpga_jtag_state=0; 

// inteface functions
static const char fpga_jtag_name[] = "fpga_jtag_loader";
static int minors[FPGA_JTAG_MAXMINOR+1];	// each minor can be opened only once
//static int thisminor;
static int     fpga_jtag_open   (struct inode *inode, struct file *filp);
static int     fpga_jtag_release(struct inode *inode, struct file *filp);
static ssize_t fpga_jtag_write  (struct file * file, const char * buf, size_t count, loff_t *off);
static loff_t  fpga_jtag_lseek  (struct file * file, loff_t offset, int orig);
static ssize_t fpga_jtag_read   (struct file * file, char * buf, size_t count, loff_t *off);
static int __init fpga_jtag_init(void);

static struct file_operations fpga_jtag_fops = {
        owner:    THIS_MODULE,
        open:     fpga_jtag_open,
        release:  fpga_jtag_release,
        llseek:   fpga_jtag_lseek,
        read:     fpga_jtag_read,
        write:    fpga_jtag_write
};

//static int sens_num = 0;

// internal functions
//loff_t fjtag_bitsize (int minor);
//loff_t fjtag_bytesize (int minor);
int JTAG_channel(int minor);
void initPortC(void);
void set_pgm_mode (int chn, int en);
void set_pgm (int chn, int pgmon);
int read_done (int chn);
int jtag_send (int chn, int tms, int len, int d);
int jtag_write_bits (int chn,
        unsigned char *buf, // data to write
        int len,            // number of bytes to write
        int check,          // compare readback data with previously written, abort on mismatch
        int last,           // output last bit with TMS=1
        int prev[2]);       // if null - don't use
int JTAG_configure (int chn, unsigned char * buf, int len);
int JTAG_readID (int chn, unsigned char * buf);
int JTAG_openChannel (int chn);
int JTAG_resetChannel (int chn);
int JTAG_CAPTURE (int chn, unsigned char * buf, int len);
int JTAG_EXTEST (int chn, unsigned char * buf, int len);

void JTAG_push_raw (int b);
int JTAG_process_raw(void);


int JTAG_channel(int minor) {
    if ((minor >= DEV393_MINOR(DEV393_JTAGS_CONF0)) && (minor < (DEV393_MINOR(DEV393_JTAGS_CONF0) + SENSOR_PORTS)))
        return (minor - DEV393_MINOR(DEV393_JTAGS_CONF0)) + (JTAG_SENSOR_FPGA << 2);


    if ((minor >= DEV393_MINOR(DEV393_JTAGS_BSCAN0)) && (minor < (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + SENSOR_PORTS)))
        return (minor - DEV393_MINOR(DEV393_JTAGS_BSCAN0)) + (JTAG_SENSOR_FPGA << 2);
    // maybe will never be used
#ifdef NC353
    switch (minor) {
    case DEV393_MINOR(DEV393_JTAG_RESET) : // same as RAW
        return JTAG_RAW << 2;
    case FPGA_JTAG_MINOR:
    case FPGA_JTAG_BOUNDARY_MINOR:
        return JTAG_MAIN_FPGA << 2;
    case FPGA_SJTAG_MINOR:
    case FPGA_SJTAG_BOUNDARY_MINOR:
        return JTAG_SENSOR_FPGA << 2;
    case FPGA_AJTAG_MINOR:
    case FPGA_AJTAG_BOUNDARY_MINOR:
        return JTAG_AUX_FPGA << 2;
    }
#endif
    return 0;
}
static int raw_fifo_w_wp;
static int raw_fifo_w_rp;
static int raw_fifo_r_wp;
static int raw_fifo_r_rp;
static int raw_chn;

/*
 * send raw JTAG commands. Each command consists of 2 bytes
 * byte 0 - data to send through TDI, lsb aligned (if less than 8 bits - hign bits are not used
 * byte 1 - 0001TNNN - send NNN?NNN:8 bits of byte 0 through TDI, keeping TDS at value of T (reads back TDO - before TCL)
 *        - 00100000 - select JTAG channel from byte 0 (reads back channel)
 *        - 00100010 - de-activate JTAG access         (reads back 0)
 *        - 00100011 - activate JTAG access            (reads back 0)
 *        - 00100100 - PGM off                         (reads back ready in bit 0)
 *        - 00100101 - PGM on                          (reads back 0xf0)
 *        - 1TTTTTTT - delay usec byte0+ ((byte1 &0x7f) << 8)  (reads back 80)
 *        - 0??????? (other) - nop                     (reads back 0xff)
 * if no channel is selected, no output is generated
void set_pgm_mode (int chn, int en);
void set_pgm (int chn, int pgmon);
 */
#define JTAG_RAW_SEND   0x10
#define JTAG_RAW_SETCHN 0x20
#define JTAG_RAW_DEACT  0x22
#define JTAG_RAW_ACT    0x23
#define JTAG_RAW_PGMOFF 0x24
#define JTAG_RAW_PGMON  0x25
#define JTAG_RAW_WAIT   0x80

void JTAG_push_raw (int b) {
    raw_fifo_r[raw_fifo_r_wp++]=b;
    if (raw_fifo_r_wp > FJTAG_RAW_RSIZE) raw_fifo_r_wp-=FJTAG_RAW_RSIZE;
}
// TODO: Not updated for 393. Is it needed?
int JTAG_process_raw(void) {
    unsigned char b0, b1;
    while (raw_fifo_w_rp != (raw_fifo_w_wp & ~1)) {
        b0=raw_fifo_w[raw_fifo_w_rp++];
        b1=raw_fifo_w[raw_fifo_w_rp++];
        if (raw_fifo_w_rp > FJTAG_RAW_WSIZE) raw_fifo_w_rp-=FJTAG_RAW_WSIZE;
        if        (b1 == JTAG_RAW_SETCHN) { // set channel number
            raw_chn = b0;
            if (raw_chn>=JTAG_NCHANNELS) raw_chn=0; //illegal channel
            JTAG_push_raw (raw_chn);
        } else if (raw_chn) { // ignore commands until the JTAG channel number is specified
            if        ((b1 & 0xf0) == JTAG_RAW_SEND) { // send JTAG data
                JTAG_push_raw (jtag_send(raw_chn, (b1 >> 3) & 1, b1 & 7, (int) b0 ));
            } else if ((b1 & 0x80) == JTAG_RAW_WAIT) { // delay
                /* possible bug here, udelay is used for delays less then 2 ms */
                udelay(((b1 & 0x7f) <<8) + b0);
                JTAG_push_raw (0x80);
            } else switch (b1) {
            case JTAG_RAW_DEACT:
                set_pgm_mode (raw_chn, 0);
                JTAG_push_raw (0x0);
                break;
            case JTAG_RAW_ACT:
                set_pgm_mode (raw_chn, 1);
                JTAG_push_raw (0x0);
                break;
            case JTAG_RAW_PGMOFF:
                set_pgm (raw_chn, 0);
                JTAG_push_raw (read_done(raw_chn));
                break; // was missing for 353
            case JTAG_RAW_PGMON:
                set_pgm (raw_chn, 1);
                JTAG_push_raw (0xf0);
                break;
            default:
                JTAG_push_raw (0xff);
            }
        } else { // make output always be 1 byte for 2 bytes input
            JTAG_push_raw (0xf0);
        } // end of if (raw_chn) /else
    } // while (raw_fifo_w_rp != (raw_fifo_w_wp & ~1))
    return 0; // will think of return value later
}

//returns 0 if all channels closed
// 
int JTAG_whatopen(void) {
    int i,r=0;
    for (i=0;i<JTAG_NCHANNELS;i++) r |= 1 << JTAG_channels[i].mode;
    return r;
}

//++++++++++++++++++++++++++++++++++++ open() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static int fpga_jtag_open(struct inode *inode, struct file *filp) {
    int i;
    //   int res;
    int p = MINOR(inode->i_rdev);
    int chn= JTAG_channel(p);
    //reg_intr_vect_rw_mask intr_mask;
    //D(printk("fpga_jtag_open: minor=%x, channel=%x, buf=%p\r\n",p,chn,bitstream_data ));
    dev_dbg(NULL, "fpga_jtag_open: minor=%x, channel=%x, buf=%p\r\n",p ,chn, bitstream_data);
    switch ( p ) {
    case DEV393_MINOR(DEV393_JTAG_RESET) : // same as RAW
        for (i=1; i<JTAG_NCHANNELS; i++) JTAG_channels[i].mode=JTAG_MODE_CLOSED;
        JTAG_channels[chn].mode=JTAG_MODE_RAW;
        JTAG_channels[chn].sizew =  FJTAG_RAW_WSIZE;
        JTAG_channels[chn].sizer =  FJTAG_RAW_RSIZE;
        JTAG_channels[chn].wp =  0;
        JTAG_channels[chn].rp =  0; // will read IDs if actually read
        raw_fifo_w_wp=0;
        raw_fifo_w_rp=0;
        raw_fifo_r_wp=0;
        raw_fifo_r_rp=0;
        raw_chn=0;
        break;
        //     case FPGA_JTAG_MINOR :
        //       if ( JTAG_whatopen() & 0x7e) return -EACCES; // none of the channels could be open when opening this file
        //       JTAG_channels[chn].mode  =  JTAG_MODE_PGM;
        //       JTAG_channels[chn].dbuf  = &bitstream_data[0];
        //       JTAG_channels[chn].sizew =  FJTAG_BUF_SIZE;
        //       JTAG_channels[chn].sizer =  FJTAG_IDSIZE >> 3;
        //       JTAG_channels[chn].wp =  0;
        //       JTAG_channels[chn].rp =  0; // will read IDs if actually read
#ifdef TEST_DISABLE_CODE
        fpga_state &= ~FPGA_STATE_LOADED;     // is it still used?
        fpga_state &= ~FPGA_STATE_SDRAM_INIT; // not needed
        // disable camera interrupts here (while reprogramming FPGA could generate stray interrupts;
        /* Disable external interrupts.. */
        intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);
        intr_mask.ext = 0;
        REG_WR(intr_vect, regi_irq, rw_mask, intr_mask);
#endif /* TEST_DISABLE_CODE */
        //     printk ("Camera interrupts disabled\r\n");
        //       break;
        // fall through
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 0): // ugly, fix
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 3):
#ifdef NC353
    case FPGA_SJTAG_MINOR :
    case FPGA_AJTAG_MINOR :
#endif
        if ( JTAG_whatopen() & 0x7e) return -EACCES; // none of the channels could be open when opening this file
        JTAG_channels[chn].mode  =  JTAG_MODE_PGM;
        JTAG_channels[chn].dbuf  = &bitstream_data[0];
        JTAG_channels[chn].sizew =  FJTAG_BUF_SIZE;
        JTAG_channels[chn].sizer =  FJTAG_IDSIZE >> 3;
        JTAG_channels[chn].wp =  0;
        JTAG_channels[chn].rp =  0; // will read IDs if actually read
        JTAG_channels[chn].bitsw = XC3S1200E_BITSIZE;  // bit size to be written
        JTAG_channels[chn].bitsr=  FJTAG_IDSIZE;
        JTAG_openChannel (chn); // configure channel access, reset JTAG and to RUN-TEST/IDLE state
        break;
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 0):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 3):
#ifdef NC353
    case FPGA_JTAG_BOUNDARY_MINOR :
    case FPGA_SJTAG_BOUNDARY_MINOR :
    case FPGA_AJTAG_BOUNDARY_MINOR :
#endif
        if ( JTAG_whatopen() & 0x46)  return -EACCES; // none of the channels could be open for program/id/raw when opening this file
        if ( JTAG_channels[chn].mode != JTAG_MODE_CLOSED) return -EACCES; // already open
        JTAG_channels[chn].mode  =  JTAG_MODE_BOUNDARY;
        JTAG_channels[chn].sizew =  (XC3S1200E_BOUNDARY_SIZE+7) >> 3;
        JTAG_channels[chn].sizer =  (XC3S1200E_BOUNDARY_SIZE+7) >> 3;
        JTAG_channels[chn].dbuf  = &bitstream_data[JTAG_channels[chn].sizew * chn];
        JTAG_channels[chn].wp =    0;
        JTAG_channels[chn].rp =    0; // will read IDs if actually read
        JTAG_channels[chn].bitsw = XC3S1200E_BOUNDARY_SIZE;  // bit size to be written
        JTAG_channels[chn].bitsr=  XC3S1200E_BOUNDARY_SIZE;
        JTAG_openChannel (chn); // configure channel access, reset JTAG and to RUN-TEST/IDLE state
        break;
    default: return -EINVAL;
    }
    dev_dbg(NULL, "fpga_jtag_open: chn=%x, JTAG_channels[chn].sizew=%x, JTAG_channels[chn].sizer=%x\r\n", chn, JTAG_channels[chn].sizew, JTAG_channels[chn].sizer);
    dev_dbg(NULL, "fpga_jtag_open: chn=%x, JTAG_channels[chn].bitsw=%x, JTAG_channels[chn].bitsr=%x\r\n", chn, JTAG_channels[chn].bitsw, JTAG_channels[chn].bitsr);
    JTAG_channels[chn].wdirty=0;
    inode->i_size=JTAG_channels[chn].sizer;
    minors[p]=p;
    filp->private_data = &minors[p];
    dev_dbg(NULL, "fpga_jtag_open: inode->i_size=%x, chn=%x\r\n", (int) inode->i_size, chn);
    return 0;
}

//++++++++++++++++++++++++++++++++++++ release() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static int fpga_jtag_release(struct inode *inode, struct file *filp) {
    int res=0;
    int p = MINOR(inode->i_rdev);
    int chn= JTAG_channel(p);
    dev_dbg(NULL, "fpga_jtag_release: p=%x,chn=%x,  wp=0x%x, rp=0x%x\r\n", p, chn, JTAG_channels[chn].wp, JTAG_channels[chn].rp);
    switch ( p ) {
    case DEV393_MINOR(DEV393_JTAG_RESET) : // same as RAW - do nothing, raw code should do it on it's own
        break;
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 0): // ugly, fix
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 3):
    //     case FPGA_JTAG_MINOR :
    //     case FPGA_SJTAG_MINOR :
    //     case FPGA_AJTAG_MINOR :
    if (JTAG_channels[chn].wp > 0) { // anything written?
        res=JTAG_configure (chn, JTAG_channels[chn].dbuf, JTAG_channels[chn].wp);
        JTAG_resetChannel (chn);
        if ((res >=0) & (chn == JTAG_MAIN_FPGA)) {
            // read FPGA model number/revision and OR it with current state
            //fpga_state =  (fpga_state & ~0xffff) | (port_csp0_addr[X313__RA__MODEL] & 0xffff);
        }
    } else JTAG_resetChannel (chn); /// reset initializing in any case:
    //if (chn == JTAG_MAIN_FPGA) fpga_state &=~FPGA_STATE_INITIALIZED;

    break;
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 0):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 3):
    //     case FPGA_JTAG_BOUNDARY_MINOR :
    //     case FPGA_SJTAG_BOUNDARY_MINOR :
    //     case FPGA_AJTAG_BOUNDARY_MINOR :
    // "dirty"? Send to JTAG
    if (JTAG_channels[chn].wp >0) {
        //		   D(printk("fpga_jtag_release(), JTAG_channels[%d].wp = 0x%x",chn,JTAG_channels[chn].wp));
        JTAG_EXTEST (chn, JTAG_channels[chn].dbuf, JTAG_channels[chn].bitsw); // size in bits
    }
    JTAG_resetChannel (chn);
    break;
    default: return -EINVAL;
    }
    minors[p]=0;
    JTAG_channels[chn].mode=JTAG_MODE_CLOSED;
    //D(printk("fpga_jtag_release:  done\r\n"));
    dev_dbg(NULL, "fpga_jtag_release:  done\r\n");
    dev_info(NULL, "fpga_jtag_release:  done, res= %d\n",res);
    return (res<0)?res:0;
}


//++++++++++++++++++++++++++++++++++++ write() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

//for boundary scan: writing before wp+1 will start EXTEST cycle (either rollover or lseek)
static ssize_t fpga_jtag_write(struct file * file, const char * buf, size_t count, loff_t *off) {
    int p = ((int *)file->private_data)[0];
    int chn= JTAG_channel(p);
    size_t size = JTAG_channels[chn].sizew;
    dev_dbg(NULL, "fpga_jtag_write: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, wp=%lx,size=0x%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].wp, (int) size);

    switch (p) {
    case DEV393_MINOR(DEV393_JTAG_RESET) : // same as RAW - do nothing, raw code should do it on it's own
        if (count > size) count= size;
        if ((raw_fifo_w_wp+count) > size) { // read tail, then roll over to the head to the total of count
            if (copy_from_user(&raw_fifo_w[raw_fifo_w_wp],buf,size-raw_fifo_w_wp)) return -EFAULT; // read tail
            if (copy_from_user(&raw_fifo_w[0],&buf[size-raw_fifo_w_wp],count+raw_fifo_w_wp-size)) return -EFAULT; // read head
        } else {
            if (copy_from_user(&raw_fifo_w[raw_fifo_w_wp],buf,count)) return -EFAULT; // read count
        }
        raw_fifo_w_wp+=count;
        if (raw_fifo_w_wp > size) raw_fifo_w_wp -= size;
        JTAG_process_raw(); // send all the received data to JTAG - will cause read fifo to get ~1/2 of the number of bytes written
        break;
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 0): // ugly, fix
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_CONF0) + 3):
    //     case FPGA_JTAG_MINOR  :
    //     case FPGA_SJTAG_MINOR :
    //     case FPGA_AJTAG_MINOR : // read configuration data to buffer
    if (*off > size) return -EFAULT;
    if ((*off + count) > size) count= (size - *off);
    if (copy_from_user(&(JTAG_channels[chn].dbuf[*off]),buf,count)) return -EFAULT;
    *off+=count;
    if (*off > JTAG_channels[chn].wp) JTAG_channels[chn].wp= *off;
    break;
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 0):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 1):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 2):
    case (DEV393_MINOR(DEV393_JTAGS_BSCAN0) + 3):
    //     case FPGA_JTAG_BOUNDARY_MINOR :
    //     case FPGA_SJTAG_BOUNDARY_MINOR :
    //     case FPGA_AJTAG_BOUNDARY_MINOR :
    if (*off > size) return -EFAULT;
    if ((*off + count) > size) count= (size - *off);
    if (*off < JTAG_channels[chn].wp) {
        // 	     D(printk("fpga_jtag_write(), JTAG_channels[%d].wp = 0x%x",chn, JTAG_channels[chn].wp));
        JTAG_EXTEST (chn, JTAG_channels[chn].dbuf, JTAG_channels[chn].bitsw); // writing "before" causes EXTEST to fill in boundary scan register
        JTAG_channels[chn].wdirty=0;
    }
    if (copy_from_user(&(JTAG_channels[chn].dbuf[*off]),buf,count)) return -EFAULT;
    *off+=count;
    JTAG_channels[chn].wp= *off; // before rolling over
    if (*off >= size) {
        *off=0; // roll over
    }
    JTAG_channels[chn].mode=JTAG_MODE_EXTEST; //should write the last byte before reading - or buffer data will be just lost
    JTAG_channels[chn].wdirty=1;
    break;
    default:               return -EINVAL;
    }
    dev_dbg(NULL, "fpga_jtag_write end: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, wp=%lx,size=0x%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].wp, (int) size);
    //D(printk("fpga_jtag_write end: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, wp=%lx,size=0x%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].wp, (int) size));
    return count;
}

//++++++++++++++++++++++++++++++++++++ read() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

ssize_t fpga_jtag_read(struct file * file, char * buf, size_t count, loff_t *off) {
    int p = ((int *)file->private_data)[0];
    int chn= JTAG_channel(p);
    size_t size = JTAG_channels[chn].sizer;
    int size_av;  // available data
    dev_dbg(NULL, "fpga_jtag_read: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, rp=%lx,size=0x%x\r\n", p, chn,(long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].rp, (int) size);
    switch (p) {
    case DEV393_MINOR(DEV393_JTAG_RESET) : // same as RAW - do nothing, raw code should do it on it's own
        size_av=(raw_fifo_r_wp >= raw_fifo_r_rp)?(raw_fifo_r_wp - raw_fifo_r_rp):(size+raw_fifo_r_wp - raw_fifo_r_rp);
        if (count > size_av) count= size_av;
        if ((raw_fifo_r_rp+count) > size) { // read tail, then roll over to the head to the total of count
            if (copy_to_user(buf, &raw_fifo_r[raw_fifo_r_rp],size-raw_fifo_r_rp)) return -EFAULT; // read tail
            if (copy_to_user(&buf[size-raw_fifo_r_rp],&raw_fifo_r[0],count+raw_fifo_r_rp-size)) return -EFAULT; // read head
        } else {
            if (copy_to_user(buf,&raw_fifo_w[raw_fifo_w_wp],count)) return -EFAULT; // read count
        }
        raw_fifo_r_rp+=count;
        if (raw_fifo_r_rp > size) raw_fifo_r_rp -= size;
        break;
    case DEV393_MINOR(DEV393_JTAGS_CONF0): // ugly, fix
    case DEV393_MINOR(DEV393_JTAGS_CONF1):
    case DEV393_MINOR(DEV393_JTAGS_CONF2):
    case DEV393_MINOR(DEV393_JTAGS_CONF3):
    //     case FPGA_JTAG_MINOR  :
    //     case FPGA_SJTAG_MINOR :
    //     case FPGA_AJTAG_MINOR : // read configuration data to buffer
    if ((JTAG_channels[chn].wp==0) && (JTAG_channels[chn].rp==0)) { // starting from read - get ID
        JTAG_channels[chn].mode=JTAG_MODE_RDID;
        JTAG_readID (chn, JTAG_channels[chn].dbuf);
    }
    if (*off > size) return -EFAULT;
    if ((*off + count) > size) count= (size - *off);
    dev_dbg(NULL, "fpga_jtag_read_01: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, rp=%lx,size=0x%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].rp, (int) size);

    if (copy_to_user(buf,&(JTAG_channels[chn].dbuf[*off]),count)) return -EFAULT;
    *off+=count;
    JTAG_channels[chn].rp= *off;
    break;
    case DEV393_MINOR(DEV393_JTAGS_BSCAN0):
    case DEV393_MINOR(DEV393_JTAGS_BSCAN1):
    case DEV393_MINOR(DEV393_JTAGS_BSCAN2):
    case DEV393_MINOR(DEV393_JTAGS_BSCAN3):
    //     case FPGA_JTAG_BOUNDARY_MINOR :
    //     case FPGA_SJTAG_BOUNDARY_MINOR :
    //     case FPGA_AJTAG_BOUNDARY_MINOR :
    if ((JTAG_channels[chn].mode==JTAG_MODE_EXTEST) && (JTAG_channels[chn].wdirty || (*off < JTAG_channels[chn].rp))) {
        JTAG_EXTEST (chn, JTAG_channels[chn].dbuf, JTAG_channels[chn].bitsr); // writing last byte causes EXTEST to fill in boundary scan register
        JTAG_channels[chn].wdirty=0;
    }
    // (re)-read capture pins if it was a roll-over or the first access after open
    if ((JTAG_channels[chn].mode!=JTAG_MODE_EXTEST) && ((*off < JTAG_channels[chn].rp) || (JTAG_channels[chn].mode==JTAG_MODE_BOUNDARY))) {
        JTAG_CAPTURE (chn, JTAG_channels[chn].dbuf, JTAG_channels[chn].bitsr);
    }
    if (*off > size) return -EFAULT;
    if ((*off + count) > size) count= (size - *off);
    dev_dbg(NULL, "fpga_jtag_read_01: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, rp=%lx,size=0x%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].rp, (int) size);
    if (copy_to_user(buf,&(JTAG_channels[chn].dbuf[*off]),count)) return -EFAULT;
    *off+=count;
    JTAG_channels[chn].rp= *off; // before rolling over
    if (*off >= size) {
        *off=0; // roll over
    }
    if (JTAG_channels[chn].mode == JTAG_MODE_BOUNDARY) JTAG_channels[chn].mode=JTAG_MODE_SAMPLE; //should write the last byte before reading - or buffer data will be just lost
    break;
    default:               return -EINVAL;
    }
    dev_dbg(NULL, "fpga_jtag_read_end: p=%x,chn=%x, buf address=%lx count=%lx *offs=%lx, rp=%lx,size=0x%x, mode=%x\r\n", p, chn, (long) buf, (long) count, (long) *off, (long)JTAG_channels[chn].rp, (int) size, JTAG_channels[chn].mode);
    return count;
}

//++++++++++++++++++++++++++++++++++++ lseek() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static loff_t  fpga_jtag_lseek(struct file * file, loff_t offset, int orig) {
    /*
     *  orig 0: position from begning of
     *  orig 1: relative from current position
     *  orig 2: position from last address
     */
    int p = ((int *)file->private_data)[0];
    int chn= JTAG_channel(p);
    size_t size;
    if (chn==JTAG_RAW) {
        size=raw_fifo_r_wp-raw_fifo_r_rp;
        if (size<0) size+=FJTAG_RAW_RSIZE;
    } else   size = JTAG_channels[chn].sizew;
    if (JTAG_channels[chn].mode == JTAG_MODE_RDID) size = JTAG_channels[chn].sizer;
    dev_dbg(NULL, "fpga_jtag_lseek, fsize= 0x%x\n", (int) size);
    switch (orig) {
    case 0:
        file->f_pos = offset;
        break;
    case 1:
        file->f_pos += offset;
        break;
    case 2:
        file->f_pos = size + offset;
        break;
    default:
        return -EINVAL;
    }
    /* truncate position */
    if (file->f_pos < 0) {
        file->f_pos = 0;
        return (-EOVERFLOW);
    }

    if (file->f_pos > size) {
        file->f_pos = size;
        return (-EOVERFLOW);
    }
    dev_dbg(NULL,"fpga_jtag_lseek, file->f_pos= 0x%x\n", (int) file->f_pos);
    return (file->f_pos);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Initialize GPIOs of the CPU to access JTAG/programming of the main FPGA
void initPortC(void) {
    // connect 8 lower bits of port C to GPIO, disconnect from IOP
    unsigned long tmp;
#ifdef TEST_DISABLE_CODE
    reg_pinmux_rw_pc_iop    pinmux_c_iop;
    reg_pinmux_rw_pc_gio    pinmux_c_gio;
    reg_gio_rw_pc_oe        pc_oe;

    pinmux_c_iop= REG_RD(pinmux, regi_pinmux, rw_pc_iop);
    tmp = REG_TYPE_CONV(unsigned long, reg_pinmux_rw_pc_iop, pinmux_c_iop);
    tmp &= ~0xff;
    pinmux_c_iop = REG_TYPE_CONV(reg_pinmux_rw_pc_iop, unsigned long, tmp);
    REG_WR(pinmux, regi_pinmux, rw_pc_iop, pinmux_c_iop);

    pinmux_c_gio= REG_RD(pinmux, regi_pinmux, rw_pc_gio);
    tmp = REG_TYPE_CONV(unsigned long, reg_pinmux_rw_pc_gio, pinmux_c_gio);
    tmp |= 0xff;
    pinmux_c_gio = REG_TYPE_CONV(reg_pinmux_rw_pc_gio, unsigned long, tmp);
    REG_WR(pinmux, regi_pinmux, rw_pc_gio, pinmux_c_gio);

    // now set data of port C pins (static pc_dout)
    pc_dout = REG_RD(gio, regi_gio, rw_pc_dout);
    pc_dout.data &= ~0xff;
    pc_dout.data |= PC_DOUT_INITIAL;
    REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);

    // now set directions of port C pins
    pc_oe = REG_RD(gio, regi_gio, rw_pc_oe);
    pc_oe.oe &= ~( (1 << FPGAJTAG_TDO_BIT) |
            (1 << FPGAJTAG_DONE_BIT) |
            (1 << FPGAJTAG_RSTBTN_BIT));
    pc_oe.oe |=  ( (1 << FPGAJTAG_TDI_BIT) |
            (1 << FPGAJTAG_TMS_BIT) |
            (1 << FPGAJTAG_TCK_BIT) |
            (1 << FPGAJTAG_PGM_BIT));
    REG_WR(gio, regi_gio, rw_pc_oe, pc_oe);
#endif /* TEST_DISABLE_CODE */
}

inline u32 prep_sensio_status(int sens_num)
{
    x393_status_sens_io_t stat;
    x393_status_ctrl_t stat_ctrl;

    stat_ctrl.d32 = 0;
    stat = x393_sensio_status(sens_num);
    stat_ctrl.seq_num = stat.seq_num + 1;
    stat_ctrl.mode = 1;
    set_x393_sensio_status_cntrl(stat_ctrl, sens_num);
    //	dev_dbg(NULL, "set seq_num = %d, chn = %d", stat_ctrl.seq_num, sens_num);
    return stat_ctrl.seq_num; // Sequence number to expect (wait for) with return data
}

inline x393_status_sens_io_t wait_sensio_status(int chn, u32 seq_num) // reducing number of hardware reads
{
    int i;
    int ret = 0;
    x393_status_sens_io_t stat;

    //	dev_dbg(NULL, "waiting for seq_num = %d, chn = %d", seq_num, chn);
    for (i = 0; i < 10; i++) {
        stat = x393_sensio_status(chn & 3); // sens_num);
        if (stat.seq_num == seq_num) {
            ret = -1;
            if (i)
                dev_dbg(NULL, "seq_num = %d received after %d wait cycles", seq_num, i);
            break;
        }
    }
    //	return ret;
    return stat;
}

inline u32 read_tdo(int sens_num)
{
    x393_status_sens_io_t stat;
    x393_status_ctrl_t stat_ctrl;
    int i;
    stat_ctrl.d32 = 0;
    stat = x393_sensio_status(sens_num);
    stat_ctrl.seq_num = stat.seq_num + 1;
    stat_ctrl.mode = 1;
    set_x393_sensio_status_cntrl(stat_ctrl, sens_num);
    for (i = 0; i < 10; i++) {
        stat = x393_sensio_status(sens_num & 3); // sens_num);
        if (likely(stat.seq_num == stat_ctrl.seq_num)) {
            return stat.xfpgatdo;
        }
    }
    dev_err(NULL,"read_tdo(%d): failed to  get expected seq_num in 10 cycles, expected = 0x%x, got 0x%x\n",sens_num,stat_ctrl.seq_num, stat.seq_num);
    return stat.xfpgatdo;
}

// read last 8 TDO bits, shifted at rising edge of TCL
inline u32 read_tdo_byte(int sens_num)
{
    x393_status_sens_io_t stat;
    x393_status_ctrl_t stat_ctrl;
    int i;
    stat_ctrl.d32 = 0;
    stat = x393_sensio_status(sens_num);
    stat_ctrl.seq_num = stat.seq_num + 1;
    stat_ctrl.mode = 1;
    set_x393_sensio_status_cntrl(stat_ctrl, sens_num);
    for (i = 0; i < 10; i++) {
        stat = x393_sensio_status(sens_num & 3); // sens_num);
        if (likely(stat.seq_num == stat_ctrl.seq_num)) {
            return stat.xfpgatdo_byte;
        }
    }
    dev_err(NULL,"read_tdo_byte(%d): failed to  get expected seq_num in 10 cycles, expected = 0x%x, got 0x%x\n",sens_num,stat_ctrl.seq_num, stat.seq_num);
    return stat.xfpgatdo_byte;
}



// set FPGA in programming/JTAG mode (only for sensor board)
// NOP for the main board FPGA configuration
void set_pgm_mode (int chn, int en) {
    u32 seq_num;
    x393_sensio_jtag_t data;
    dev_dbg(NULL, "set_pgm_mode (%d,%d)\n",chn,en);

    switch (chn >> 2) {
    case JTAG_SENSOR_FPGA:
        //port_csp0_addr[X313_WA_SENSFPGA] = (en ? (3 << SFPGA_PGMEN_BIT): (SFPGA_RD_SENSPGMPIN | (2 << SFPGA_PGMEN_BIT))) | (2  << SFPGA_TCK_BIT); // turn off TCK (if not turned off already)

        /* ? SFPGA_RD_SENSPGMPIN */
        data.d32 = 0;
        data.pgmen = (en) ? 1 : 0;
        data.pgmen_set = 1;
        data.tck = 0;
        data.tck_set = 1;
        /* check status register */
        x393_sensio_jtag(data, chn & 3); // sens_num);
        /* wait for status register update */
        wait_sensio_status(chn & 3, prep_sensio_status(chn & 3)) ; // Not needed here
        break;
    }
    udelay (2);
}

void set_pgm (int chn, int pgmon) {
    u32 seq_num;
    x393_sensio_jtag_t data;
    dev_dbg(NULL, "set_pgm (%d,%d)\n",chn,pgmon);

    switch (chn >> 2) {
    case JTAG_MAIN_FPGA:
#ifdef TEST_DISABLE_CODE
        if (pgmon) pc_dout.data  &= ~0x80; // set PGM low (active)
        else pc_dout.data  |= 0x80;        // set PGM high (inactive)
        REG_WR(gio, regi_gio, rw_pc_dout, pc_dout); // extend low?
#endif /* TEST_DISABLE_CODE */
        break;
    case JTAG_SENSOR_FPGA:
        //port_csp0_addr[X313_WA_SENSFPGA] = (2 | (pgmon & 1)) << SFPGA_PROG_BIT;
        data.prog= pgmon & 1;
        data.prog_set = 1;
        x393_sensio_jtag(data, chn >> 2); // sens_num);
        break;
    case JTAG_AUX_FPGA:
        break;
    }
    udelay (2);
}


int read_done (int chn) {
    x393_status_sens_io_t stat;
    x393_sensio_jtag_t data;

    switch (chn >> 2) {
#ifdef TEST_DISABLE_CODE
    case JTAG_MAIN_FPGA:
        return ((((REG_RD(gio, regi_gio, r_pc_din)).data & 0x20)==0) ? 0 : 1 );
#endif //* TEST_DISABLE_CODE */
    case JTAG_SENSOR_FPGA:
        //port_csp0_addr[X313_WA_SENSFPGA] = SFPGA_RD_DONE;
        //udelay (1);
        //return (port_csp0_addr[X313__RA__SENSFPGA] >> SFPGA_RD_BIT) & 1 ;

        stat = wait_sensio_status(chn & 3, prep_sensio_status(chn & 3)) ;
        return stat.xfpgadone;
    case JTAG_AUX_FPGA:
        return 0;
    }
    return 0; // just in case
}

//  send 1..8 bits through JTAG 
int jtag_send (int chn, int tms, int len, int d) {
    int sens_num = chn & 3;
    x393_sensio_jtag_t data;
    x393_status_sens_io_t stat;
    //	u32 seq_num;
    int i, bm = 0; //,m;
    int r=0;
    int d0;
    i = len & 7;
    if (i==0) i=8;
    d &= 0xff;
    d0=d;
    dev_dbg(NULL, "jtag_send(0x%x, 0x%x, 0x%x, 0x%x)\r\n", chn, tms,len,d);
    switch (chn >> 2) {
    case JTAG_MAIN_FPGA:
#ifdef TEST_DISABLE_CODE
        pc_dout.data &= ~0x0e;
        pc_dout.data |= (tms & 1) << FPGAJTAG_TMS_BIT;
        for (;i>0;i--){
            r= (r<<1)+ ((REG_RD(gio, regi_gio, r_pc_din)).data & 1); // read TDO before TCK pulse
            pc_dout.data = (pc_dout.data & ~0x0a) | (((d<<=1)>>7) & 2);
            REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
            pc_dout.data |= (1 << FPGAJTAG_TCK_BIT);
            REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
            pc_dout.data &= ~(1 << FPGAJTAG_TCK_BIT);
            REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
        }
#endif /* TEST_DISABLE_CODE */
        break;
    case JTAG_SENSOR_FPGA:
#ifdef TEST_DISABLE_CODE
        port_csp0_addr[X313_WA_SENSFPGA] = SFPGA_RD_TDO;
        udelay (1); // wait MUX
        for (;i>0;i--){
            port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay
            port_csp0_addr[X313_WA_SENSFPGA] = ((2 | (tms & 1)) << SFPGA_TMS_BIT) |
                    (((((d<<=1)>>8) & 1) | 2) << SFPGA_TDI_BIT) |
                    (2  << SFPGA_TCK_BIT) ;
            port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay
            port_csp0_addr[X313_WA_SENSFPGA] = (3  << SFPGA_TCK_BIT); // TCK=1
            r= (r<<1)+ ((port_csp0_addr[X313__RA__SENSFPGA] >> SFPGA_RD_BIT) & 1); // read TDO before TCK pulse
            port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay

        }
        port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0
#endif /* TEST_DISABLE_CODE */
        data.d32 =     0;
        data.tck_set = 1;
        data.tms_set = 1;
        data.tdi_set = 1;

        dev_dbg(NULL, "jtag_send(0x%x, 0x%x, 0x%x, 0x%x)\n", chn, tms,len,d);
        for ( ; i > 0; i--) {
            /* TCK = 0 - just a delay; is it really needed? */
            data.tck = 0;

            data.tms = tms & 1;
            data.tdi = ((d <<= 1) >> 8) & 1;
            data.tck = 0;

            x393_sensio_jtag(data, sens_num);
            /* repeat writel - just a delay; is it really needed? */
            //			x393_sensio_jtag(data, sens_num);

            /* read TDO before TCK pulse */
#ifndef PARALLEL_JTAG
            r = (r << 1) +  read_tdo(sens_num); // may need to read twice to increase delay?
#else
            bm = (bm <<1 ) | 1;
#endif
            data.tck = 1;
            x393_sensio_jtag(data, sens_num);  // keep other signals, set TCK == 1
            //			x393_sensio_jtag(data, sens_num);  // repeat if delay will be needed to increase length of the TCK signal

            data.tck = 0;
            //			x393_sensio_jtag(data, sens_num);
        }
        x393_sensio_jtag(data, sens_num);
#ifdef  PARALLEL_JTAG
        r = read_tdo_byte(sens_num) & bm;
#endif
        //		x393_sensio_jtag(data, sens_num);
        dev_dbg(NULL, " ---> %02x\n", r);

        break;
    case JTAG_AUX_FPGA:
        break;
    }
    return r;
}

//====================================
//         port_csp0_addr[X313_WA_SENSFPGA] = 0; // nop
// write data data bytes from buffer, read data, optionally compare/abort
// return: 0- OK, !=0 - readback mismatch error
// modified so it reads data in-place of the written one
// send/receive bits, raising TMS during the last one (if last==1). If number of bits are not multiple of 8, lower bits of the last byte will not be used.

int jtag_write_bits (int chn,
        unsigned char *buf, // data to write
        int len,            // number of bits to write
        int check,          // compare readback data with previously written, abort on mismatch
        int last,           // output last bit with TMS=1
        int prev[2])      // if null - don't use
{
    int sens_num = chn & 3;
    int i,j;
    int r =  0;
    int bm = 0;
    int d,d0;
    //	u32 seq_num;
    x393_status_sens_io_t stat;
    x393_sensio_jtag_t data;

    dev_dbg(NULL, "jtag_write_bits(0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\r\n", (int) chn, (int) buf, len, check, last);

    switch (chn >> 2) {
    case JTAG_MAIN_FPGA: //TODO: save some cycles like for FPGA_SJTAG_MINOR
#ifdef TEST_DISABLE_CODE
        for (i=0; len>0;i++) {
            pc_dout.data &= ~0x0e;
            d0=(d=buf[i]);
            for (j=0;j<8;j++) {
                //D(printk("i=%x, j=%x, len=%x, d=%x  ",i,j,len,d));
                if (len>0) {
                    r= (r<<1)+ ((REG_RD(gio, regi_gio, r_pc_din)).data & 1);
                    if ((len==1) && last) pc_dout.data = (pc_dout.data & ~0x0a) | (((d<<=1)>>7) & 2) | (1 << FPGAJTAG_TMS_BIT);
                    else                  pc_dout.data = (pc_dout.data & ~0x0a) | (((d<<=1)>>7) & 2);
                    REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
                    pc_dout.data |= (1 << FPGAJTAG_TCK_BIT);
                    REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
                    pc_dout.data &= ~(1 << FPGAJTAG_TCK_BIT);
                    REG_WR(gio, regi_gio, rw_pc_dout, pc_dout);
                } else r= (r<<1);
                len--;
                //D(printk(", r=%x\r\n",r));
            }
            buf[i]=r; // read back in-place
            if (check && ((r ^ (prev[1] >> 24)) & 0xff)) {
                return -((r & 0xff) | ((i+1) << 8)); //readback mismatch
            }
            if (prev) {
                //         prev64= (prev64<<8) | ((prev32>>24) & 0xff);
                //         prev32= (prev32<<8) | (d0 & 0xff);
                prev[1]= (prev[1]<<8) | ((prev[0]>>24) & 0xff);
                prev[0]= (prev[0]<<8) | (d0 & 0xff);
            }
        }
#endif /* TEST_DISABLE_CODE */
        break;
    case JTAG_SENSOR_FPGA:
#ifdef TEST_DISABLE_CODE
        port_csp0_addr[X313_WA_SENSFPGA] = SFPGA_RD_TDO; // just in case, it should be in that mode when calling jtag_write_bits()
        udelay (1); // wait MUX
        for (i=0; len>0;i++) {
            d0=(d=buf[i]);
            for (j=0;j<8;j++) {
                port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay
                if (len>0) {
                    if ((len==1) && last) port_csp0_addr[X313_WA_SENSFPGA] =
                            (3 << SFPGA_TMS_BIT) |
                            (((((d<<=1)>>8) & 1) | 2) << SFPGA_TDI_BIT) |
                            (2 << SFPGA_TMS_BIT) |
                            (2 << SFPGA_TCK_BIT) ;

                    else port_csp0_addr[X313_WA_SENSFPGA] =
                            (((((d<<=1)>>8) & 1) | 2) << SFPGA_TDI_BIT) |
                            (2 << SFPGA_TMS_BIT) |
                            (2 << SFPGA_TCK_BIT) ;
                    port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay
                    port_csp0_addr[X313_WA_SENSFPGA] = (3 << SFPGA_TCK_BIT); // TCK=1
                    // add delays here if long cable?
                    r= ((r<<1)+ ((port_csp0_addr[X313__RA__SENSFPGA] >> SFPGA_RD_BIT) & 1)); // read TDO before TCK pulse
                    port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0 - just a delay
                } else r= (r<<1);
                len--;
            }
            buf[i]=r; // read back in-place
            if (check && ((r ^ (prev[1]>>24)) & 0xff)) {
                return -((r & 0xff) | ((i+1) << 8)); //readback mismatch
            }
            if (prev) {
                //         prev64= (prev64<<8) | ((prev32>>24) & 0xff);
                //         prev32= (prev32<<8) | (d0 & 0xff);
                prev[1]= (prev[1]<<8) | ((prev[0]>>24) & 0xff);
                prev[0]= (prev[0]<<8) | (d0 & 0xff);
            }

        }
        port_csp0_addr[X313_WA_SENSFPGA] = (2  << SFPGA_TCK_BIT); // TCK=0
#endif /* TEST_DISABLE_CODE */
        // Can be done once
        data.d32 =     0;
        data.tck_set = 1;
        data.tms_set = 1;
        data.tdi_set = 1;

        for (i = 0; len > 0; i++) {
            d0 = (d = buf[i]);
            dev_dbg(NULL,"jtag_write_bits(), i=0x%x ", i);
            bm = 0;
            for (j = 0; j < 8; j++) {
                if (len > 0) {
                    data.tms = (len == 1 && last)? 1:0 ;
                    data.tdi = ((d <<= 1) >> 8) & 1;
                    data.tck = 0;
                    x393_sensio_jtag(data, sens_num);
#ifndef PARALLEL_JTAG
                    r = (r << 1) +  read_tdo(sens_num);
#else
                    bm = (bm <<1 ) | 1;
#endif
                    data.tck = 1;
                    x393_sensio_jtag(data, sens_num);

                    data.tck = 0;
                    x393_sensio_jtag(data, sens_num);
                } else {
                    r <<= 1;
                }
                len--;
            }
#ifdef  PARALLEL_JTAG
            r = read_tdo_byte(sens_num) & bm;
            if (unlikely(len < 0)){
                r <<= -len;
            }
#endif
            buf[i] = r;
            dev_dbg(NULL," ===> %02x\n", r);
            if (check && ((r ^ (prev[1]>>24)) & 0xff)) {
                return -((r & 0xff) | ((i+1) << 8)); //readback mismatch
            }
            if (prev) {
                prev[1]= (prev[1]<<8) | ((prev[0]>>24) & 0xff);
                prev[0]= (prev[0]<<8) | (d0 & 0xff);
            }
        }
        break;
    case JTAG_AUX_FPGA:
        break;
    }
    return 0;
}


int JTAG_configure (int chn, unsigned char * buf, int len) {
    int datastart, i, j ,r;
    //static int prev32;
    //static int prev64;
    int prev[2];
#ifdef JTAG_DISABLE_IRQ
    unsigned long flags;
#endif
    const unsigned char sync[]={0xff,0xff,0xff,0xff,0xaa,0x99,0x55,0x66};
    int skipvfy=8;

    dev_dbg(NULL, "JTAG_configure: chn=%x,  wp=0x%x, rp=0x%x, len=0x%x\r\n",chn, JTAG_channels[chn].wp, JTAG_channels[chn].rp, len);

    // all the programming goes here...
    // find sync:

    datastart=-1;
    for (i=0;i<(FJTAG_MAX_HEAD-8);i++) {
        j=0;
        while ((j<8) && (buf[i+j]==sync[j])) j++;
        if (j==8) {
            datastart=i;
            break;
        }
    }
    if (datastart<0) {
        dev_err(NULL,"Bitstream not found - bad file\r\n");
        return -EFAULT;
    }
    // check for right bitstream length
    if ((len-datastart)!=(XC3S1200E_BITSIZE>>3)) {
        dev_err(NULL,"Wrong bitstream size - XC3S1200E has bitstream of %d bits (%d bytes)\n",XC3S1200E_BITSIZE,XC3S1200E_BITSIZE>>3);
        dev_err(NULL,"header size - %d, data size - %d\r\n",datastart, len-datastart);
        return -EFAULT;
    }
    // enable programmimg mode (nop for the 10353 FPGA)
    set_pgm_mode(chn, 1);
    // reset device
    set_pgm (chn, 1);
    //udelay (1000); // needed?
    mdelay(1);
    set_pgm (chn, 0);
    // wait INIT over - no init connected, just wait >2ms
    //udelay (2500);
    mdelay(3);
    //*************************** NOW DISABLE INTERRUPS FOR THE WHOLE PROGRAMMING CYCLE ***********************
    //D( udelay (100000);printk("JTAG_configure(): IRQ off!\r\n"); udelay (100000););
    D( mdelay (100);printk("JTAG_configure(): IRQ off!\r\n"); mdelay (100););
#ifdef JTAG_DISABLE_IRQ
    local_irq_save(flags);
    //local_irq_disable();
#endif
    // prepare JTAG
    jtag_send(chn, 1, 5, 0   ); //step 1 - set Test-Logic-Reset state
    jtag_send(chn, 0, 1, 0   ); //step 2 - set Run-Test-Idle state
    jtag_send(chn, 1, 2, 0   ); //step 3 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 4 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0xa0); //step 5 - start of CFG_IN  ***NOW 6 bits ***
    jtag_send(chn, 1, 1, 0   ); //step 6 - finish CFG_IN
    jtag_send(chn, 1, 2, 0   ); //step 7 - set SELECT-DR state
    jtag_send(chn, 0, 2, 0   ); //step 8 - set SHIFT-DR state
    // write data (first 8 bytes - just fill the buffer, no readback comparison)
    jtag_write_bits (chn,
            &buf[datastart], // data to write
            skipvfy << 3,               // number of bytes to write
            0,                          // compare readback data with previously written, abort on mismatch
            0,                          // do not raise TMS at last bit
            prev);                      // 64 bits storage to verify configuration transmission

    if ((r=jtag_write_bits (chn,
            &buf[datastart+skipvfy],
            //                          (buf8i-(datastart+skipvfy)) << 3,
            (len-(datastart+skipvfy)) << 3,
            1,
            1, prev))<0) {
        r= -r;
        i= (r>>8) -1 + (datastart+skipvfy);
        r &= 0xff;
#ifdef JTAG_DISABLE_IRQ
        local_irq_restore(flags);
#endif
        set_pgm (chn, 1);
        set_pgm (chn, 0);
        // disable programmimg mode (nop for the 10353 FPGA)
        set_pgm_mode(chn, 0);
        dev_err(NULL,"**** Configuration failed at byte # %d (%x)****\n", (i-datastart),(i-datastart));
        dev_err(NULL,"**** r= %x, prev64=%x prev32=%x****\n", r,prev[1], prev[0]);
        return -EFAULT;
    }
    jtag_send(chn, 1, 1, 0   ); //step 11 - set UPDATE-DR state
    jtag_send(chn, 1, 2, 0   ); //step 12 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 13 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0x30); //step 14 - start of JSTART  ***NOW 6 bits ***
    jtag_send(chn, 1, 1, 0   ); //step 15 - finish JSTART
    jtag_send(chn, 1, 2, 0   ); //step 16 - set SELECT-DR state
    jtag_send(chn, 0, 0, 0   ); //step 17 - set SHIFT-DR , clock startup
    jtag_send(chn, 0, 0, 0   ); //step 17a - (total >=12 clocks)
    jtag_send(chn, 1, 2, 0   ); //step 18 - set UPDATE-DR state
    jtag_send(chn, 0, 1, 0   ); //step 19 - set Run-Test-Idle state
    jtag_send(chn, 0, 0, 0   ); //step 19a - only here starts the sequence - adding 17b does not help (5 - sets done, 6 - releases outputs, 7 - releases reset)
    jtag_send(chn, 0, 0, 0   ); //step 19b - one more?
    // ready or not - device should start now
#ifdef JTAG_DISABLE_IRQ
    local_irq_restore(flags);
#endif
    //*************************** END OF NO INTERRUPS ***********************
    r=read_done(chn);
    // disable programmimg mode (nop for the 10353 FPGA)
    set_pgm_mode(chn, 0);

    if (r==0) {
        dev_err(NULL,"*** FPGA did not start after configuration ***\r\n");
        return -EFAULT;
    }

    //D( udelay (100000);printk("\nJTAG_configure() OK!\r\n"));
    D( mdelay (100);printk("\nJTAG_configure() OK!\r\n"));
    dev_info(NULL,"JTAG_configure() OK!\n");
    return 0;
} //int JTAG_configure


//=============================
//
// enable access to JTAG pins. For sensor FPGA that is not possible w/o deprogramming the chip
// leaves in Run-Test-Idle state
int JTAG_openChannel (int chn)  {
    dev_dbg(NULL, "JTAG_openChannel (%d)\n",chn);
    // enable programmimg mode (nop for the 10353 FPGA)
    set_pgm_mode(chn, 1);
    // for shared JTAG/data bus we need to de-program the chip to be able to read JTAG :-(
    switch (chn >> 2) {
    case JTAG_SENSOR_FPGA:
        // reset device
        set_pgm (chn, 1);
        set_pgm (chn, 0);
        // wait INIT over - no init connected, just wait >2ms
        //udelay (2500);
        mdelay(3);
        break;
    }
    jtag_send(chn, 1, 5, 0   ); // set Test-Logic-Reset state
    jtag_send(chn, 0, 1, 0   ); // set Run-Test-Idle state
    return 0;
} // int JTAG_openChannel (int chn)
//
int JTAG_resetChannel (int chn)  {
    dev_dbg(NULL, "JTAG_resetChannel (%d)\n",chn);
    jtag_send(chn, 1, 5, 0   ); // set Test-Logic-Reset state
    // disable programmimg mode (nop for the 10353 FPGA)
    set_pgm_mode(chn, 0); // only for sensor FPGA
    return 0;
} // int JTAG_resetChannel (int chn)

int JTAG_readID (int chn, unsigned char * buf)  {
    int i;
    unsigned long d1,d2=0;
    unsigned long * dp;
#ifdef JTAG_DISABLE_IRQ
    unsigned long flags;
#endif

    // read dev id, user id
    //*************************** NOW DISABLE INTERRUPS FOR THE WHOLE JTAG SEQUENCE ***********************
#ifdef JTAG_DISABLE_IRQ
    local_irq_save(flags);
    //local_irq_disable();
#endif
    // prepare JTAG
    jtag_send(chn, 1, 5, 0   ); //step 1 - set Test-Logic-Reset state
    jtag_send(chn, 0, 1, 0   ); //step 2 - set Run-Test-Idle state
    jtag_send(chn, 1, 2, 0   ); //step 3 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 4 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0x90); //step 5 - start of IDCODE
    jtag_send(chn, 1, 1, 0   ); //step 6 - finish IDCODE
    jtag_send(chn, 1, 2, 0   ); //step 7 - set SELECT-DR state
    jtag_send(chn, 0, 2, 0   ); //step 8 - set CAPTURE-DR state
    jtag_write_bits (chn,
            &buf[0], // data to write
            32, // number of bits to write/read
            0,                          // don't compare readback data with previously written
            1, 0) ;                        // raise TMS at last bit
    jtag_send(chn, 1, 5, 0   ); //reset state machine to Test-Logic-Reset state
    jtag_send(chn, 0, 1, 0   ); //step 2 - set Run-Test-Idle state
    jtag_send(chn, 1, 2, 0   ); //step 3 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 4 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0x10); //step 5 - start of USERCODE
    jtag_send(chn, 1, 1, 0   ); //step 6 - finish USERCODE
    jtag_send(chn, 1, 2, 0   ); //step 7 - set SELECT-DR state
    jtag_send(chn, 0, 2, 0   ); //step 8 - set CAPTURE-DR state
    jtag_write_bits (chn,
            &buf[4], // data to write
            32, // number of bits to write/read
            0,                          // don't compare readback data with previously written
            1,0) ;                      // raise TMS at last bit
    jtag_send(chn,1, 5, 0   ); //reset state machine to Test-Logic-Reset state
#ifdef JTAG_DISABLE_IRQ
    local_irq_restore(flags);
#endif
    //*************************** END OF NO INTERRUPS ***********************
    // swap all bits in ID and user fields
    dp = (unsigned long *) &buf[0];
    d1= *dp;
    for (i=0;i<32;i++){
        d2 = (d2 << 1) | (d1 & 1);
        d1 >>= 1;
    }
    *dp=d2;
    dp = (unsigned long *) &buf[4];
    d1= *dp;
    for (i=0;i<32;i++){
        d2 = (d2 << 1) | (d1 & 1);
        d1 >>= 1;
    }
    *dp=d2;
    D(for (i=0; i<8;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");} );
    data_modified=0; //*************************************************
    return 0;
} // int JTAG_readID (int chn, unsigned char * buf)

/*
 * capture all pins w/o changing functionality
 * assuming Run-Test-Idle/UPDATE-DR leaving UPDATE-DR
 */

int JTAG_CAPTURE (int chn, unsigned char * buf, int len) {
    int i; // only in debug
#ifdef JTAG_DISABLE_IRQ
    unsigned long flags;
#endif
    dev_dbg(NULL,"JTAG_CAPTURE(): buf=%p\n",buf);
    //*************************** NOW DISABLE INTERRUPS FOR THE WHOLE JTAG SEQUENCE ***********************
#ifdef JTAG_DISABLE_IRQ
    local_irq_save(flags);
    //local_irq_disable();
#endif
    // prepare JTAG
    //     jtag_send(chn, 1, 5, 0   ); //step 1 - set Test-Logic-Reset state
    //     jtag_send(chn, 0, 1, 0   ); //step 2 - set Run-Test-Idle state
    jtag_send(chn, 1, 2, 0   ); //step 3 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 4 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0x80); //step 5 - start of SAMPLE (which bit goes first???)
    jtag_send(chn, 1, 1, 0   ); //step 6 - finish SAMPLE
    jtag_send(chn, 1, 2, 0   ); //step 7 - set SELECT-DR state
    jtag_send(chn, 0, 2, 0   ); //step 8 - set CAPTURE-DR state
    jtag_write_bits (chn,
            buf, // data to write
            len, // number of bits to read
            0,   // don't compare readback data with previously written
            1,0) ; // raise TMS at last bit
    //     D(printk ("\n"); for (i=0; i<((len+7)>>3) ;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");}printk ("\n"); );
    //     D(printk ("\n"); for (i=0; i<((len+7)>>3) ;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");}printk ("\n"); );
    jtag_send(chn, 1, 1, 0   ); //step 9 - set UPDATE-DR state
    //     D(printk ("\n"); for (i=0; i<((len+7)>>3) ;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");}printk ("\n"); );
    //     jtag_send(chn,1, 5, 0   ); //reset state machine to Test-Logic-Reset state
#ifdef JTAG_DISABLE_IRQ
    local_irq_restore(flags);
#endif
    //*************************** END OF NO INTERRUPS ***********************
    dev_dbg(NULL, "\n");
    for (i=0; i<((len+7)>>3) ;i++) {
        dev_dbg(NULL, "%3x ",(int) buf[i]);
        if ((i & 0xf) == 0xf) dev_dbg(NULL, "\n");
    }
    dev_dbg(NULL, "\n");
    data_modified=0;
    return 0;

} // JTAG_CAPTURE (int chn, unsigned char * buf, int len) {


/*
 * write new boundary registers (len should match BS register length), read pins in-place
 * TAP controller is supposed to be in "UPDATE-DR" state (or RUN-TEST/IDLE after just opening this mode)
 * TAP controller will be left in the same "UPDATE-DR" after the command
 */
int JTAG_EXTEST (int chn, unsigned char * buf, int len) {
#ifdef JTAG_DISABLE_IRQ
    unsigned long flags;
#endif
    int i; // only in debug
#ifdef JTAG_DISABLE_IRQ
    local_irq_save(flags);
    //local_irq_disable();
#endif
    //D(printk("EXTEST: buf=%p, len=0x%x\n",buf,len));
    //D(printk ("\n"); for (i=0; i<((len+7)>>3) ;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");}printk ("\n"); );

    //     jtag_send(chn, 1, 5, 0   ); //step 1 - set Test-Logic-Reset state
    //     jtag_send(chn, 0, 1, 0   ); //step 2 - set Run-Test-Idle state
    jtag_send(chn, 1, 2, 0   ); //step 3 - set SELECT-IR state
    jtag_send(chn, 0, 2, 0   ); //step 4 - set SHIFT-IR state
    jtag_send(chn, 0, 5, 0xf0); //step 5 - start of EXTEST
    jtag_send(chn, 1, 1, 0   ); //step 6 - finish EXTEST
    jtag_send(chn, 1, 2, 0   ); //step 7 - set SELECT-DR state
    jtag_send(chn, 0, 2, 0   ); //step 8 - set CAPTURE-DR state

    jtag_write_bits  (chn,
            buf,
            len, // number of bits to write
            0,                         // don't compare readback data with previously written
            1,0);                        // raise TMS at last bit
    jtag_send(chn, 1, 1, 0   ); //step 9 - set UPDATE-DR state
#ifdef JTAG_DISABLE_IRQ
    local_irq_restore(flags);
#endif
    //     D(printk ("\n"); for (i=0; i<((len+7)>>3) ;i++) {printk("%3x ",(int) buf[i]); if ((i & 0xf) == 0xf) printk ("\n");}printk ("\n"); );

    return 0;
} //int JTAG_EXTEST (int chn, unsigned char * buf, int len)

static int __init fpga_jtag_init(void) {
    int i,res;
    res = register_chrdev(DEV393_MAJOR(DEV393_JTAGS_CONF0), fpga_jtag_name, &fpga_jtag_fops);
    if(res < 0) {
        dev_err(NULL,"\nfpga_jtag_init: couldn't get a major number %d.\n",DEV393_MAJOR(DEV393_JTAGS_CONF0));
        return res;
    }
    dev_dbg(NULL,DEV393_NAME(DEV393_JTAGS_CONF0)" - %d\n",DEV393_MAJOR(DEV393_JTAGS_CONF0));
    for (i=0;i<=FPGA_JTAG_MAXMINOR;i++) minors[i]=0;
    initPortC();

    dev_dbg(NULL, "elphel test %s: MAJOR %d", DEV393_NAME(DEV393_JTAGS_CONF0), DEV393_MAJOR(DEV393_JTAGS_CONF0));
    res = init_mmio_ptr();
    if (res < 0)
        return -ENOMEM;

    return 0;
}

static void __exit fpga_jtag_exit(void)
{
    unregister_chrdev(DEV393_MAJOR(DEV393_JTAGS_CONF0), DEV393_NAME(DEV393_JTAGS_CONF0));
    dev_dbg(NULL, "unregistering driver");
}
module_exit(fpga_jtag_exit);

/* this makes sure that fpga_init is called during boot */

module_init(fpga_jtag_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>");
MODULE_DESCRIPTION(FPGA_JTAG_DRIVER_DESCRIPTION);

