/** @file imu_log393.c
 *
 * @brief reading logger data
 *
 * @copyright Copyright (C) 2011-2016 Elphel, Inc
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
 /*-----------------------------------------------------------------------------**
 *!  $Log: imu_log353.c,v $
 *!  Revision 1.5  2012/04/14 03:53:48  elphel
 *!  bug fix in the driver (was producing errors in 3-4 hours)
 *!
 *!  Revision 1.3  2011/08/13 00:54:08  elphel
 *!  added /dev/imu_ctl where it is possible to read current logger settings
 *!
 *!  Revision 1.2  2011/07/30 23:22:54  elphel
 *!  Modified to enable simultaneous access to IMU logger,
 *!  fixed bug noticed by Lemay
 *!
 *!  Revision 1.1  2011/05/20 03:33:48  elphel
 *!  IMU/GPS logger driver, initial commit
 *!
 */
/****************** INCLUDE FILES SECTION ***********************************/

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <asm/outercache.h>    // TODO: Implement cache operations for the logger !!!!
#include <asm/cacheflush.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <asm/delay.h>
#include <asm/uaccess.h> // copy_*_user
#include <uapi/elphel/c313a.h>
#include <elphel/elphel393-mem.h>
#include "imu_log393.h"
#include "x393.h"
#include "cci2c.h"
#include <uapi/elphel/x393_devices.h>


#if 1
#define D(x) x
//#define D0(x) x
//#define MD7(x) printk("%s:%d:",__FILE__,__LINE__);x
//#define MD8(x) printk("%s:%d:",__FILE__,__LINE__);x
//#define MD12(x) printk("%s:%d:",__FILE__,__LINE__);x
#else
#define D(x)
//#define D0(x)
//#define MD7(x)
//#define MD8(x)
//#define MD12(x)
#endif

//#define D1(x) x


#define IS_103695_REV_A 1

#define MULT_SAXI_CHN 0 ///< using channel 0 of a 4-channel DMA engine
#define LOGGER_DMA_RESET 0
#define LOGGER_DMA_STOP  1
#define LOGGER_DMA_RUN   2
#define LOGGER_STATUS_MODE 3    // autoupdate
#define MULT_SAXI_STATUS_MODE 3 // autoupdate
#define LOGGER_IRQ_DW_BIT 4     // Number of DWORD address bit to change to cause interrupt
#define LOGGER_USE_IRQ 1

#ifdef NC353
#define EXT_DMA_1_START \
        do { reg_bif_dma_rw_ch1_start c = {.run=1};\
        REG_WR(bif_dma, regi_bif_dma, rw_ch1_start, (reg_bif_dma_rw_ch1_start) c); } while( 0 )
#define EXT_DMA_1_STOP \
        do { reg_bif_dma_rw_ch1_start c = {.run=0};\
        REG_WR(bif_dma, regi_bif_dma, rw_ch1_start, (reg_bif_dma_rw_ch1_start) c); } while( 0 )
#define bytePtrMask  ((CCAM_DMA1_SIZE << 2)-1) // and byte pointer in the dma buffer to get index in the array
#else
//#define EXT_DMA_1_START
//#define EXT_DMA_1_STOP
#endif

#ifdef NC353
    #define XCLK_RATE 80000000 ///< 80MHz clock in NC353
#else
    #define XCLK_RATE 100000000 ///< 100MHz clock in NC393
#endif
#define RS232_RATE 19200   ///< RS232 bps
#define IMU_MODULE_DESCRIPTION "IMU logger for 10365 ext. board"
//#define	LOGGER_DRIVER_NAME "imu_logger"
#define IMU_MAXMINOR 10

#ifdef NC353
    #define   X313_WA_IOPINS    0x70  // bits [31:24] - enable channels (channel 0 -software, enabled at FPGA init)
    #define   X313_WA_IOPINS_EN_IMU  0xc0000000
    #define   X313_WA_IOPINS_DIS_IMU 0x80000000
    #define   X313_WA_IMU_DATA  0x7e
    #define   X313_WA_IMU_CTRL  0x7f
    #define   X313_RA_IMU_COUNT   0x7e // number of 64-byte samples recorded (24 bit counter)
#endif
//  #define   X313_RA_IMU_DATA   0x7e // use csp4
//  #define   X313_RA_IMU_STATUS 0x7f // use csp4

// Is it the same for 393?
#define   IMU_COUNT_OVERFLOW 0x1000000 ///< number of records written is modulo IMU_COUNT_OVERFLOW

#define   X313_IMU_PERIOD_ADDR     0x0  ///< request period for IMU (in SPI bit periods)
#define   X313_IMU_DIVISOR_ADDR    0x1  ///< xclk (80MHz) clock divisor for half SPI bit period 393: clock is Now clock is logger_clk=100MHz (200 MHz?)
#define   X313_IMU_RS232DIV_ADDR   0x2  ///< serial gps bit duration in xclk (80MHz) periods - 16 bits
#define   X313_IMU_CONFIGURE_ADDR  0x3  ///< IMU logger configuration


#define IMU_CONF(x,y)  (((((y) & ((1 << IMUCR__##x##__WIDTH)-1))) | (1 << IMUCR__##x##__WIDTH) ) << IMUCR__##x##__BITNM)
#define  IMUCR__IMU_SLOT__BITNM 0    ///< slot, where 103695 (imu) board is connected: 0 - none, 1 - J9, 2 - J10, 3 - J11)
#define  IMUCR__IMU_SLOT__WIDTH 2

#define  IMUCR__GPS_CONF__BITNM 3    ///< slot, where 103695 (imu) board is connected: 0 - none, 1 - J9, 2 - J10, 3 - J11)
#define  IMUCR__GPS_CONF__WIDTH 4    ///< bits 0,1 - slot #, same as for IMU_SLOT, bits 2,3:
// 0 - ext pulse, leading edge,
// 1 - ext pulse, trailing edge
// 2 - start of the first rs232 character after pause
// 3 - start of the last "$" character (start of each NMEA sentence)
#define  IMUCR__MSG_CONF__BITNM 8    ///< source of external pulses to log:
#define  IMUCR__MSG_CONF__WIDTH 5    ///< bits 0-3 - number of fpga GPIO input 0..11 (i.e. 0x0a - external optoisolated sync input (J15)
// 0x0f - disable MSG module
// bit 4 - invert polarity: 0 - timestamp leading edge, log at trailing edge, 1 - opposite
// software may set (up to 56 bytes) log message before trailing end of the pulse
#define  IMUCR__SYN_CONF__BITNM 14   ///< logging frame time stamps (may be synchronized by another camera and have timestamp of that camera)
#define  IMUCR__SYN_CONF__WIDTH 4    ///< 0 - disable, 1 - enable, per channel (was width==1 in NC353)

#define  IMUCR__RST_CONF__BITNM 19   ///< reset module // was 16 in NC353
#define  IMUCR__RST_CONF__WIDTH 1    ///< 0 - enable, 1 -reset (needs resettimng DMA address in ETRAX also)

#define  IMUCR__DBG_CONF__BITNM 21   ///< several axtra IMU configuration bits (was 18 for NC353)
#define  IMUCR__DBG_CONF__WIDTH 4    ///< 0 - config_long_sda_en, 1 -config_late_clk, 2 - config_single_wire, should be set for 103695 rev "A"

#define  IMUCR__SLOW_SPI__BITNM 26   ///< just for the driver, not written to FPGA (was 23 for NC353)
#define  IMUCR__SLOW_SPI__WIDTH 1    ///< 0 - normal, 1 - slow SPI (programmed over i2c)

#define  IMUCR__I2C_SA3__BITNM 28    ///< Low 3 bits of the SA7 of the PCA9500 slave address
#define  IMUCR__I2C_SA3__WIDTH 3     ///< Should match jumpers

#define   X313_IMU_REGISTERS_ADDR    0x4
#define   X313_IMU_NMEA_FORMAT_ADDR  0x20
#define   X313_IMU_MESSAGE_ADDR    0x40  ///< 40..4f, only first 0xe visible

// offsets in the file (during write)
#define   X313_IMU_PERIOD_OFFS     0x0
#define   X313_IMU_DIVISOR_OFFS    0x4

#define   X313_IMU_RS232DIV_OFFS   0x8
#define   X313_IMU_CONFIGURE_OFFS  0xc

#define   X313_IMU_SLEEP_OFFS      0x10
#define   X313_IMU_REGISTERS_OFFS  0x14 // .. 0x2f

#define   X313_IMU_NMEA_FORMAT_OFFS  0x30
#define   X313_IMU_MESSAGE_OFFS      0xB0 // 0xB0..0xE7

#define   PCA9500_PP_ADDR            0x40 ///< PCA9500 i2c slave addr for the parallel port (read will be 0x41)

#define DFLT_SLAVE_ADDR 3           ///< i2c slave addr modifier (0..7) for the IMU (103695) board
#define DFLT_SCLK_FREQ 5000000      ///< SCLK frequency
#define DFLT_DIVISOR ( XCLK_RATE / DFLT_SCLK_FREQ /2 ) 
#define DFLT_STALL_USEC 2           ///< stall time in usec
#define DFLT_STALL  (( DFLT_STALL_USEC * ( XCLK_RATE / DFLT_DIVISOR )) / 1000000 )    ///< stall time in usec

#define DFLT_SLEEP   30000 ///< usec, sleep if not ready (30ms)
//#define DFLT_FEQ 300
//#define DFLT_PERIOD ( XCLK_RATE / DFLT_DIVISOR / DFLT_FEQ ) // fixed scan period
#define DFLT_PERIOD 0xFFFFFFFF ///< read IMU when it is ready
#define DFLT_RS232DIV ( XCLK_RATE / 2 / RS232_RATE )

#if IS_103695_REV_A
#define EXTRA_CONF 0x4
#else
#define EXTRA_CONF 0x0
#endif
#define SLOW_SPI 0 ///< set to 1 for slower SPI (not ADIS-16375), it will increase SCLK period that does not end CS active

#define DFLT_CONFIG ( IMU_CONF(IMU_SLOT,1) | \
        IMU_CONF(GPS_CONF, ( 2 | 8) ) | \
        IMU_CONF(MSG_CONF,10)  | \
        IMU_CONF(SYN_CONF, 1) | \
        IMU_CONF(DBG_CONF, EXTRA_CONF) | \
        ((SLOW_SPI & 1)<<23) | \
        (DFLT_SLAVE_ADDR << 24))

#define  WHICH_INIT         1
#define  WHICH_RESET        2
#define  WHICH_RESET_SPI    4
#define  WHICH_DIVISOR      8
#define  WHICH_RS232DIV    16
#define  WHICH_NMEA        32
#define  WHICH_CONFIG      64
#define  WHICH_REGISTERS  128
#define  WHICH_MESSAGE    256
#define  WHICH_PERIOD     512
#define  WHICH_EN_DMA    1024
#define  WHICH_EN_LOGGER 2048

#define LSEEK_IMU_NEW       1 ///< start from the new data, discard buffer
#define LSEEK_IMU_STOP      2 ///< stop DMA1 and IMU
#define LSEEK_IMU_START     3 ///< start IMU and DMA1 (do not modify parameters)

static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions

const int i2cbus = 1;

static unsigned char dflt_wbuf[]=
{ DFLT_PERIOD & 0xff, ( DFLT_PERIOD >> 8 ) & 0xff, ( DFLT_PERIOD >> 16) & 0xff, ( DFLT_PERIOD >> 24 ) & 0xff,
        //   {0,0,0,0,            // period - off
        DFLT_DIVISOR, DFLT_STALL, 0,0, // clock divisor
        DFLT_RS232DIV & 0xff, ( DFLT_RS232DIV >> 8 ) & 0xff, ( DFLT_RS232DIV >> 16) & 0xff, ( DFLT_RS232DIV >> 24 ) & 0xff,
        DFLT_CONFIG & 0xff,   ( DFLT_CONFIG >> 8 )   & 0xff, ( DFLT_CONFIG >> 16) &   0xff, ( DFLT_CONFIG >> 24 ) & 0xff,
        DFLT_SLEEP & 0xff,    ( DFLT_SLEEP >> 8 )    & 0xff, ( DFLT_SLEEP >> 16) &    0xff, ( DFLT_SLEEP >> 24 ) & 0xff,
        0x10, // x gyro low
        0x12, // x gyro high
        0x14, // y gyro low
        0x16, // y gyro high
        0x18, // z gyro low
        0x1a, // z gyro high

        0x1c, // x accel low
        0x1e, // x accel high
        0x20, // y accel low
        0x22, // y accel high
        0x24, // z accel low
        0x26, // z accel high

        0x40, // x delta angle low
        0x42, // x delta angle high
        0x44, // y delta angle low
        0x46, // y delta angle high
        0x48, // z delta angle low
        0x4a, // z delta angle high

        0x4c, // x delta velocity low
        0x4e, // x delta velocity high
        0x50, // y delta velocity low
        0x52, // y delta velocity high
        0x54, // z delta velocity low
        0x56, // z delta velocity high

        0x0e, // temperature
        0x70, // time m/s
        0x72, // time d/h
        0x74,// time y/m
        // NMEA sentences
        // first three letters - sentence to log (letters after "$GP"). next "n"/"b" (up to 24 total) - "n" number (will be encoded 4 digits/byte, follwed by "0xF"
        // "b" - byte - all but last will have MSB 0 (& 0x7f), the last one - with MSB set (| 0x80). If there are no characters in the field 0xff will be output
        'R','M','C','n','b','n','b','n','b','n','n','n','n','b', 0,  0,  0,  0,  0,  0,  0,0,0,0,  0,0,0,0,  0,0,0,0,
        'G','G','A','n','n','b','n','b','n','n','n','n','b','n','b','b','b', 0,  0,  0,  0,0,0,0,  0,0,0,0,  0,0,0,0,
        'G','S','A','b','n','n','n','n','n','n','n','n','n','n','n','n','n','n','n','n', 0,0,0,0,  0,0,0,0,  0,0,0,0,
        'V','T','G','n','b','n','b','n','b','n','b', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,0,0,0,  0,0,0,0,  0,0,0,0,

        // Message - up to 56 bytes
        'O', 'd', 'o', 'm', 'e', 't', 'e', 'r', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e',
        0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0
};
static unsigned char wbuf[sizeof(dflt_wbuf)];

//static const char fpga_name[] = "imu_control";
static int     imu_open   (struct inode *inode, struct file *filp);
static int     imu_release(struct inode *inode, struct file *filp);
static ssize_t imu_write  (struct file * file, const char * buf, size_t count, loff_t *off);
static loff_t  imu_lseek  (struct file * file, loff_t offset, int orig);
static ssize_t imu_read   (struct file * file, char * buf, size_t count, loff_t *off);

#define IMU_MAXMINOR 10

static loff_t numBytesRead=0;        ///< total number of bytes read from the imu - global pointer (modified when open in write mode)
static loff_t numBytesWritten=0;     ///< total number of bytes written to the IMU buffer since it was started/restarted
static loff_t numBytesWrittenBase=0; ///< number of byte written in full buffers  since it was started/restarted


//TODO 393: Use allocated 4MB
static int dma_is_on=0;
#ifdef NC353
static int lastFPGABytes=0;          ///< last read FPGA counter (24 bits) << 6
static unsigned long ccam_dma1_buf[CCAM_DMA1_SIZE + (PAGE_SIZE>>2)] __attribute__ ((aligned (PAGE_SIZE)));
//!Without "static" system hangs after "Uncompressing Linux...
unsigned long  * logger_buffer = NULL;
//unsigned long * ccam_dma1 =         NULL; //! still used in autoexposure or something - why is in needed there?

void init_ccam_dma1_buf_ptr(void) {
    logger_buffer = ccam_dma1_buf;
    //    ccam_dma1 =         ccam_dma1_buf; //&ccam_dma_buf[0]; Use in autoexposure
}
#else
u32 * logger_buffer = NULL; ///< use instead of ccam_dma1_buf, logger_buffer. Initialize from the elphel393-mem
static u32 logger_size = 0;  // size in bytes, should be 2^n
static u32 logger_offs32 = 0; // write offset in the buffer, DWORDS
static u32 bytePtrMask = 0;
static dma_addr_t logger_phys; ///< physical address of the DMA memory start
static int logger_fpga_configured = 0;
static const struct of_device_id elphel393_logger_of_match[];
static struct device *g_dev_ptr; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
wait_queue_head_t logger_wait_queue;
#endif

static const char * const imu_devs[]={
	DEV393_DEVNAME(DEV393_LOGGER),
	DEV393_DEVNAME(DEV393_LOGGER_CTRL)
};

static const int imu_major = DEV393_MAJOR(DEV393_LOGGER);

static const int imu_minor[]={
	DEV393_MINOR(DEV393_LOGGER),
	DEV393_MINOR(DEV393_LOGGER_CTRL)
};

/** @brief Global device class for sysfs */
static struct class *imu_dev_class;

int logger_dma_ctrl(int cmd);
int logger_init_fpga(int force);

#ifdef NC353 // will update from mult_saxi pointer
void updateNumBytesWritten(void){
    x393_logger_status_t logger_status = x393_logger_status();
#ifdef NC353
    int thisFPGABytes=(int) port_csp0_addr[X313_RA_IMU_COUNT]<<6;
#else
    int thisFPGABytes = logger_status.sample << 6;
#endif
    int delta=(thisFPGABytes-lastFPGABytes);
    lastFPGABytes=thisFPGABytes;
    if (delta<0) delta+=(IMU_COUNT_OVERFLOW<<6);
    numBytesWritten+=delta; // long long
}
#endif

static struct file_operations imu_fops = {
        owner:    THIS_MODULE,
        open:     imu_open,
        release:  imu_release,
        //	ioctl:    umu_ioctl,
        llseek:   imu_lseek,
        read:     imu_read,
        write:    imu_write
};

static void set_logger_params(int which){ // 1 - program IOPINS, 2 - reset first, 4 - set divisor, 8 set regs, 16 - set period
    // IMU should be enabled through i2c before opening
    int i,j,b,f,n;
    int nmea_sel[16];
    int nmea_fpga_frmt[16];
    unsigned long d;
    unsigned long * period=     (unsigned long *) &wbuf[X313_IMU_PERIOD_OFFS];
    unsigned long * divisor=    (unsigned long *) &wbuf[X313_IMU_DIVISOR_OFFS];
    unsigned long * rs232_div=  (unsigned long *) &wbuf[X313_IMU_RS232DIV_OFFS];
    unsigned long * config=     (unsigned long *) &wbuf[X313_IMU_CONFIGURE_OFFS];
    unsigned long * message=    (unsigned long *) &wbuf[X313_IMU_MESSAGE_OFFS];
    char * nmea_format=         (char *) &wbuf[X313_IMU_NMEA_FORMAT_OFFS];

    x393_logger_address_t logger_address;
    x393_logger_data_t    logger_data;
    x393_gpio_set_pins_t  gpio_set_pins;
//    D(int i2c_err=0;)
    int i2c_err=0;

//    dev_info(g_dev_ptr,"============ which = 0x%x =============== \n",which);
    dev_dbg(g_dev_ptr,"============ which = 0x%x =============== \n",which);

    D(for (i=0; i< sizeof (wbuf); i++ ) {  if ((i & 0x1f)==0) printk("\n %03x",i);  printk(" %02x",(int) wbuf[i]); });
    if (which & WHICH_RESET) {
        if (logger_is_dma_on()!=0) {
            dev_dbg(g_dev_ptr,"Stopping DMA\n");
            logger_dma_stop();
        }

        dev_dbg(g_dev_ptr,"Resetting logger\n");
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_CONFIGURE_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = IMU_CONF(RST_CONF,1);
#else
        logger_address.d32=X313_IMU_CONFIGURE_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=IMU_CONF(RST_CONF,1);
        x393_logger_data(logger_data);
#endif
    }
    dev_dbg(g_dev_ptr,"============ which = 0x%x WHICH_INIT = 0x%x=============== \n",which, (int) WHICH_INIT);
    if (which & WHICH_INIT) {
//        unsigned char i2c_sa= PCA9500_PP_ADDR+((config[0]>>23) & 0xe);
        unsigned char i2c_sa8= PCA9500_PP_ADDR+(((config[0] >> IMUCR__I2C_SA3__BITNM) & 0x7)<<1); // Here 8-bit is needed, not SA7
        //
        unsigned char enable_IMU=0xfe; // maybe we need to reset it here? bit [1]
#ifdef NC353
        dev_dbg(g_dev_ptr,"Enabling I/O pins for IMU, written 0x%x to 0x%x\n", (int) X313_WA_IOPINS_EN_IMU, (int) X313_WA_IOPINS);
        port_csp0_addr[X313_WA_IOPINS] = X313_WA_IOPINS_EN_IMU;
#else
        dev_info(g_dev_ptr,"Enabling I/O pins for IMU\n");
        dev_dbg(g_dev_ptr,"Enabling I/O pins for IMU\n");
        gpio_set_pins.d32 =   0;
        gpio_set_pins.chn_c = 3; // enable
        x393_gpio_set_pins(gpio_set_pins);

#endif
        ///TODO: add enabling via i2c (bus=1&raw=0x2300&data=0xfe)
        //PCA9500_PP_ADDR
#if IS_103695_REV_A
//        enable_IMU = (((config[0] >>23) & 1)?0xfd:0xff); // bit[0] - reset IMU
        enable_IMU = ((config[0] & IMU_CONF(SLOW_SPI, 1))?0xfd:0xff); // bit[0] - reset IMU
#else
        enable_IMU=0xfe; // maybe we need to reset it here? bit [1]
#endif

        i2c_writeData(i2cbus,  // int n - bus (0 - to the sensor)
                i2c_sa8,        // unsigned char theSlave,
                &enable_IMU,   //unsigned char *theData,
                1,             // int size,
                1);            // int stop (send stop in the end)
        dev_dbg(g_dev_ptr,"Sent i2c command in raw mode - address=0x%x, data=0x%x, result=0x%x\n",(int)i2c_sa8, (int) enable_IMU, i2c_err);

        logger_init_fpga(0); // do not re-init if it already is
    }
    if (which & WHICH_RESET_SPI) {
        dev_dbg(g_dev_ptr,"stopped IMU logger (set period=0)\n");
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_PERIOD_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = 0; // reset IMU
#else
        logger_address.d32=                X313_IMU_PERIOD_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   0; // reset IMU
        x393_logger_data(logger_data);
#endif
    }
    dev_dbg(g_dev_ptr,"============ which = 0x%x WHICH_INIT = 0x%x=============== \n",which, (int) WHICH_INIT);
    if (which & WHICH_DIVISOR) {
        dev_dbg(g_dev_ptr,"IMU clock divisor= %ld\n", divisor[0]);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_DIVISOR_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = divisor[0]-1;
#else
        logger_address.d32=                X313_IMU_DIVISOR_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   divisor[0]-1;
        x393_logger_data(logger_data);
#endif
    }
    if (which & WHICH_RS232DIV) {
        dev_dbg(g_dev_ptr,"RS232 clock divisor= %ld\n", rs232_div[0]);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_RS232DIV_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = rs232_div[0]-1;
#else
        logger_address.d32=                X313_IMU_RS232DIV_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   rs232_div[0]-1;
        x393_logger_data(logger_data);
#endif
    }

    if (which & WHICH_NMEA) {

        for (i=0;i<16;i++) {
            nmea_sel[i]=0;
            nmea_fpga_frmt[i]=0;
        }
        for (n=0;n<4;n++) {
            nmea_format[32*n+27]=0; // just in case
            dev_dbg(g_dev_ptr,"Setting NMEA sentence format for $GP%s\n", &nmea_format[32*n]);
            dev_dbg(g_dev_ptr,"(0x%x, 0x%x, 0x%x\n",(int) nmea_format[32*n],(int) nmea_format[32*n+1],(int) nmea_format[32*n+2]);

            f=0;
            for (i=2;i>=0;i--) {
                b=nmea_format[32*n+i]; /// first 3 letters in each sentence
                dev_dbg(g_dev_ptr,"n=%d, i=%d, b=0x%x\n", n,i,b);
                for (j=4; j>=0; j--) {
                    f<<=1;
                    if ((b & (1<<j))!=0) f++;
                }
            }
            dev_dbg(g_dev_ptr,"n=%d, f=0x%x\n", n,f);
            for (i=0;i<15;i++)  if ((f & (1<<i))!=0) nmea_sel[i] |= (1<<n);
            f=0;
            nmea_fpga_frmt[n*4]=0;
            for (i=0; (i<24) && (nmea_format[32*n+3+i]!=0);i++ ) {
                b=nmea_format[32*n+3+i];
                if ((b=='b') || (b=='B')) f|=(1<<i);
                nmea_fpga_frmt[n*4]++;
            }
            nmea_fpga_frmt[n*4+1]=f & 0xff;
            nmea_fpga_frmt[n*4+2]=(f>> 8)&0xff;
            nmea_fpga_frmt[n*4+3]=(f>>16)&0xff;
        }
        dev_dbg(g_dev_ptr,"Selection data is %x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", nmea_sel[0],nmea_sel[1],nmea_sel[2],
                nmea_sel[3],nmea_sel[4],nmea_sel[5],nmea_sel[6],nmea_sel[7],nmea_sel[8],nmea_sel[9],
                nmea_sel[10],nmea_sel[11],nmea_sel[12],nmea_sel[13],nmea_sel[14]);
        dev_dbg(g_dev_ptr,"Format data for sentence 1 is %02x %02x %02x %02x\n", nmea_fpga_frmt[ 0],nmea_fpga_frmt[ 1],nmea_fpga_frmt[ 2],nmea_fpga_frmt[ 3]);
        dev_dbg(g_dev_ptr,"Format data for sentence 2 is %02x %02x %02x %02x\n", nmea_fpga_frmt[ 4],nmea_fpga_frmt[ 5],nmea_fpga_frmt[ 6],nmea_fpga_frmt[ 7]);
        dev_dbg(g_dev_ptr,"Format data for sentence 3 is %02x %02x %02x %02x\n", nmea_fpga_frmt[ 8],nmea_fpga_frmt[ 9],nmea_fpga_frmt[10],nmea_fpga_frmt[11]);
        dev_dbg(g_dev_ptr,"Format data for sentence 4 is %02x %02x %02x %02x\n", nmea_fpga_frmt[12],nmea_fpga_frmt[13],nmea_fpga_frmt[14],nmea_fpga_frmt[15]);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_NMEA_FORMAT_ADDR;
#else
        logger_address.d32=                X313_IMU_NMEA_FORMAT_ADDR;
        x393_logger_address(logger_address);
#endif
        for (i=0;i<16;i++) {
#ifdef NC353
            port_csp0_addr[X313_WA_IMU_DATA] = nmea_sel[i];
#else
            logger_data.d32=                   nmea_sel[i];
            x393_logger_data(logger_data);
#endif
            dev_dbg(g_dev_ptr,"Loaded imu fpga register 0x%x with 0x%x\n", X313_IMU_NMEA_FORMAT_ADDR+i, nmea_sel[i] );
        }
        for (i=0;i<16;i++) {
#ifdef NC353
            port_csp0_addr[X313_WA_IMU_DATA] = nmea_fpga_frmt[i];
#else
            logger_data.d32=                   nmea_fpga_frmt[i];
            x393_logger_data(logger_data);
#endif
            dev_dbg(g_dev_ptr,"Loading imu fpga register 0x%x with 0x%x\n", X313_IMU_NMEA_FORMAT_ADDR+i+16, nmea_fpga_frmt[i] );
        }
    }

    if (which & WHICH_CONFIG) {
        dev_dbg(g_dev_ptr,"Setting configuration= 0x%lx\n", config[0]);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_CONFIGURE_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] =(config[0] & 0xffffff); // MSB used for the i2c slave addr of the 10365
#else
        logger_address.d32=                X313_IMU_CONFIGURE_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   (config[0] & 0x3ffffff); // some bits in MSB are used for the i2c slave addr of the 10365
        x393_logger_data(logger_data);
#endif
    }

    if (which & WHICH_REGISTERS) {
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_REGISTERS_ADDR;
#else
        logger_address.d32=                X313_IMU_REGISTERS_ADDR;
        x393_logger_address(logger_address);
#endif
        for (i=X313_IMU_REGISTERS_OFFS; i< X313_IMU_NMEA_FORMAT_OFFS ;i++) {
            d=wbuf[i];
            dev_dbg(g_dev_ptr,"%d: logging IMU register with 0x%lx\n", (i-X313_IMU_REGISTERS_OFFS+1),d);
#ifdef NC353
            port_csp0_addr[X313_WA_IMU_DATA] = d;
#else
            logger_data.d32=                   d;
            x393_logger_data(logger_data);
#endif
        }
    }
    if (which & WHICH_MESSAGE) {
        dev_dbg(g_dev_ptr,"Setting odometer message %56s\n", (char *) message);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_MESSAGE_ADDR;
#else
        logger_address.d32=                X313_IMU_MESSAGE_ADDR;
        x393_logger_address(logger_address);
#endif
        for (i=0; i<(((sizeof(wbuf)-X313_IMU_MESSAGE_OFFS))>>2);i++) {
            dev_dbg(g_dev_ptr,"%d: message 4 bytes= 0x%x\n", i+1,(int) message[i]);
#ifdef NC353
            port_csp0_addr[X313_WA_IMU_DATA] = message[i];
#else
            logger_data.d32=                   message[i];
            x393_logger_data(logger_data);
#endif
        }
    }

    // setting IMU SPI period, turning it on
    if (which & WHICH_PERIOD) {
        dev_dbg(g_dev_ptr,"IMU cycle period= %ld\n", period[0]);
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_PERIOD_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = period[0];
#else
        logger_address.d32=                X313_IMU_PERIOD_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   period[0];
        x393_logger_data(logger_data);
#endif
    }
    if (which & WHICH_EN_DMA) {
        dev_dbg(g_dev_ptr,"Enabling DMA\n");
        /*!
TODO: (re)start DMA1 here !
         */
        /// for now - init everything again?
        if (logger_is_dma_on()!=0) {
            dev_dbg(g_dev_ptr,"Stopping DMA\n");
            logger_dma_stop();
        }
        x313_dma1_init();
        logger_dma_start();
    }

    if (which & WHICH_EN_LOGGER) {
        dev_dbg(g_dev_ptr,"Enabling logger\n");
#ifdef NC353
        port_csp0_addr[X313_WA_IMU_CTRL] = X313_IMU_CONFIGURE_ADDR;
        port_csp0_addr[X313_WA_IMU_DATA] = IMU_CONF(RST_CONF,0);
#else
        logger_address.d32=                X313_IMU_CONFIGURE_ADDR;
        x393_logger_address(logger_address);
        logger_data.d32=                   IMU_CONF(RST_CONF,0);
        x393_logger_data(logger_data);
#endif
    }

}

//filp->f_mode & FMODE_WRITE 
static int imu_open(struct inode *inode, struct file *filp) {
    int p= MINOR(inode->i_rdev);
    //        int res;
    int i;
    //        loff_t  numBytesWritten;
    dev_dbg(g_dev_ptr,"imu_open: minor=%x\n",p);
    dev_dbg(g_dev_ptr,"filp=%lx\n",(unsigned long)filp);

    switch ( p ) {
    case DEV393_MINOR(DEV393_LOGGER_CTRL):
        dev_dbg(g_dev_ptr,"IMU_ctl_open\n");
        inode->i_size=sizeof(wbuf);
        // nothing more here, after writing parameters should start imu (and dma), otherwise will use defaults on next open of /dev/imu
        break;
    case DEV393_MINOR(DEV393_LOGGER) :
    {
        dev_dbg(g_dev_ptr,"IMU_open\n");
        inode->i_size=sizeof(wbuf); // only in write mode

        /// See if initialization is needed
        if (logger_is_dma_on()==0) {
            /// copy defaults
            dev_dbg(g_dev_ptr,"Initializing IMU\n");
            dev_info(g_dev_ptr,"imu_open(): Initializing IMU\n");
            for (i=0;i<sizeof(wbuf);i++) wbuf[i]=dflt_wbuf[i];
            set_logger_params(WHICH_INIT |
                    WHICH_RESET |
                    WHICH_RESET_SPI |
                    WHICH_DIVISOR |
                    WHICH_RS232DIV |
                    WHICH_NMEA |
                    WHICH_CONFIG |
                    WHICH_REGISTERS |
                    WHICH_MESSAGE |
                    WHICH_PERIOD |
                    WHICH_EN_DMA |
                    WHICH_EN_LOGGER );
            numBytesRead=0;
        } else {
            dev_dbg(g_dev_ptr, "Skipping IMU initialization\n");
#ifdef NC353
            updateNumBytesWritten();
#endif
            if (filp->f_mode & FMODE_WRITE) { // write mode, use global read pointer
#ifdef NC353
                if ((numBytesWritten - numBytesRead)>=(CCAM_DMA1_SIZE<<2)) // there is still a chance to read as much as possible using lseek
#else
                if ((numBytesWritten - numBytesRead) >= logger_size) // there is still a chance to read as much as possible using lseek
#endif
                {
                    // alternatively - open at lower pointer?
                    dev_err(g_dev_ptr,"DMA1 buffer overrun (numBytesWritten=0x%llx, numBytesRead=0x%llx, resetting numBytesRead\n", numBytesWritten, numBytesRead);
                    numBytesRead=numBytesWritten;
                }
                //printk("imu opened in R/W mode, (numBytesWritten=0x%x, numBytesRead=0x%x\n", numBytesWritten, numBytesRead);
            } else { // read mode, use file pointer as read pointer, start from the latest data
                filp->f_pos=numBytesWritten; // there is still a chance to lseek to an earlier position - reopening at the position of the total number of bytes written to the buffer
                dev_dbg(g_dev_ptr, "imu opened in RDONLY mode, (numBytesWritten=0x%llx, numBytesRead=0x%llx\n", numBytesWritten, numBytesRead);
            }
        }
        break;
    }
    default: return -EINVAL;
    }
    filp->private_data = (int *) p; // store just minor there
    return 0;
}

static int imu_release(struct inode *inode, struct file *filp) {
    //   int res=0;
    int p = MINOR(inode->i_rdev);
    switch ( p ) {
    case DEV393_MINOR(DEV393_LOGGER) :
    case DEV393_MINOR(DEV393_LOGGER_CTRL):
        dev_dbg(g_dev_ptr,"Closing IMU device, numBytesWritten=0x%llx,  numBytesRead=0x%llx (only global pointer, does not include files opened in read mode)\n", numBytesWritten, numBytesRead);
        break;
    default: return -EINVAL;
    }
    dev_dbg(g_dev_ptr,"imu_release:  done\n");
    return 0;
}


static ssize_t imu_write(struct file * file, const char * buf, size_t count, loff_t *off) {
    unsigned long p=*off;
    unsigned long left;
    int which=0;
    //    dev_dbg(g_dev_ptr,"imu_write: ((int *)file->private_data)[0]= %x\n",((int *)file->private_data)[0]);
    dev_dbg(g_dev_ptr,"imu_write: (int)file->private_data)= %x\n",((int)file->private_data));
    //    switch (((int *)file->private_data)[0]) {
    switch ((int) file->private_data) {
    case DEV393_MINOR(DEV393_LOGGER) :
    case DEV393_MINOR(DEV393_LOGGER_CTRL):
        if (!(file->f_mode & FMODE_WRITE)) {
            return -EINVAL; // readonly
        }
        if (p >= sizeof(wbuf))  return -EINVAL; // bigger than all
        if( (p + count) > sizeof(wbuf)) { // truncate count
            count = sizeof(wbuf) - p;
        }
        left=count;
        if (left==0) return 0;
        if (copy_from_user(&wbuf[p], buf, count)) return -EFAULT;
        if (p<(X313_IMU_PERIOD_OFFS+4)) which |= WHICH_PERIOD;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        if ((p<(X313_IMU_DIVISOR_OFFS+4)) && ((p+count)>X313_IMU_DIVISOR_OFFS)) which |= WHICH_DIVISOR;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        if ((p<(X313_IMU_RS232DIV_OFFS+4)) && ((p+count)>X313_IMU_RS232DIV_OFFS)) which |= WHICH_RS232DIV;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        //       if ((p<(X313_IMU_CONFIGURE_OFFS+4)) && ((p+count)>X313_IMU_CONFIGURE_OFFS)) which |= WHICH_CONFIG;
        if ((p<(X313_IMU_CONFIGURE_OFFS+4)) && ((p+count)>X313_IMU_CONFIGURE_OFFS)) which |= WHICH_CONFIG | WHICH_INIT;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        if ((p<(X313_IMU_NMEA_FORMAT_OFFS)) && ((p+count)>X313_IMU_REGISTERS_OFFS)) which |= WHICH_REGISTERS;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        if ((p<(X313_IMU_MESSAGE_OFFS)) && ((p+count)>X313_IMU_NMEA_FORMAT_OFFS)) which |= WHICH_NMEA;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        if ((p+count)>X313_IMU_MESSAGE_OFFS) which |= WHICH_MESSAGE;
        dev_dbg(g_dev_ptr,"which= 0x%x\n",which);
        // will not add automatic restarts here
        set_logger_params(which);
        //       if (which &  WHICH_PERIOD) num_reads=0;
        *off+=count;
        return count;
    default: return -EINVAL;
    }
}

static loff_t  imu_lseek(struct file * file, loff_t offset, int orig) {
    int p=(int)file->private_data;
    dev_dbg(g_dev_ptr," file=%x, offset=%llx (%d), orig=%x\n", (int) file, offset,(int) offset, (int) orig);
    switch (p) {
    case DEV393_MINOR(DEV393_LOGGER):
    case DEV393_MINOR(DEV393_LOGGER_CTRL):
        switch (orig) {
        case SEEK_SET:
            file->f_pos = offset;
            break;
        case SEEK_CUR:
            file->f_pos += offset;
            break;
        case SEEK_END:
            //!overload later?
            if (offset<=0) {
                file->f_pos = sizeof(wbuf) + offset;
            } else {
                switch (offset) {
                case LSEEK_IMU_NEW: // sets file pointer to the last written data TODO: add lseeking to the earliest data?
#ifdef NC353
                    updateNumBytesWritten();
#endif
                    //                  numBytesRead=(int) port_csp0_addr[X313_RA_IMU_COUNT]<<6; //numBytesWritten
                    return file->f_pos;
                case LSEEK_IMU_STOP:
                    dev_dbg(g_dev_ptr,"got LSEEK_IMU_STOP\n");
                    set_logger_params(WHICH_RESET |
                            WHICH_RESET_SPI);
                    numBytesRead=0;

                    return file->f_pos;
                case LSEEK_IMU_START:
                    dev_dbg(g_dev_ptr,"got LSEEK_IMU_START\n");
                    set_logger_params(WHICH_RESET |
                            WHICH_RESET_SPI |
                            WHICH_PERIOD |
                            WHICH_EN_DMA |
                            WHICH_EN_LOGGER );
                    return file->f_pos;
                }
                /// Add stuff later?
            }
            break;
        default:
            dev_err(g_dev_ptr,"lseek: invalid orig=%d\n", orig);
            return -EINVAL;
        }
        break;
        default:
            dev_err(g_dev_ptr,"lseek: invalid minor=%d\n", p);
            return -EINVAL;
            /*
  #define LSEEK_IMU_STOP      1 // stop DMA1 and IMU
  #define LSEEK_IMU_START     2 // start IMU and DMA1 (do not modify parameters)

             */
    }
    /** truncate position */
    if (file->f_pos < 0) {
        dev_err(g_dev_ptr,"negative position: minor=%d, file->f_pos=0x%llx\n", p, file->f_pos);
        file->f_pos = 0;
        return (-EOVERFLOW);
    }
    /** enable seeking beyond buffer - it now is absolute position in the data stream*/
    if ((p==DEV393_MINOR(DEV393_LOGGER_CTRL)) && (file->f_pos > sizeof(wbuf))) {
        dev_err(g_dev_ptr,"beyond end: minor=%d, file->f_pos=0x%llx\n", p, file->f_pos);
        file->f_pos = sizeof(wbuf);
        return (-EOVERFLOW);
    }

    return (file->f_pos);
}

/** /dev/imu and /dev/imu read. If file is opened in R/W mode, reading updates global reade pointer, if readonly - each file
 * has own pointer */
static ssize_t imu_read(struct file * file, char * buf, size_t count, loff_t *off) {
    int err;
    unsigned long * sleep;
    char *charDMABuf;
    int idbg;
    int byteIndexRead;
    int byteIndexValid;
    int leftToRead;
    int pe;

    //    loff_t numBytesWritten; - it is global now, made absolute from the IMU start
    loff_t thisNumBytesRead;
#ifdef NC353
    reg_dma_rw_stat stat;
    reg_bif_dma_r_ch1_stat ch1_stat;
#endif
    dev_dbg(g_dev_ptr," file=%x, count=0x%x (%d), off=%x\n", (int) file, (int) count, (int) count, (int)(*off));
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_LOGGER_CTRL):
        //       if (*off >= sizeof(wbuf))  return -EINVAL; // bigger than all
        if (*off >= sizeof(wbuf))  return 0; // bigger than all
        if( (*off + count) > sizeof(wbuf)) { // truncate count
            count = sizeof(wbuf) - *off;
        }
        if (count==0) return 0;
        err=copy_to_user(buf, &wbuf[*off], count);
        if (err) {
            dev_err(g_dev_ptr,"0. tried to copy 0x%x bytes to offset 0x%llx, result=0x%x\n", count, *off,err);
            return -EFAULT;
        }
        *off+=count;
        return count;
        break;
    case DEV393_MINOR(DEV393_LOGGER) :
#ifdef NC353
        updateNumBytesWritten();
#endif
        thisNumBytesRead=(file->f_mode & FMODE_WRITE)?numBytesRead:*off; // is that needed ("write mode") ?
        charDMABuf = (char *) logger_buffer;
        sleep=  (unsigned long *) &wbuf[X313_IMU_SLEEP_OFFS];
        dev_dbg(g_dev_ptr,"numBytesWritten=0x%08x thisNumBytesRead=0x%08x, sleep=0x%x\n", (int) numBytesWritten, (int) thisNumBytesRead, (int) sleep[0]);
        /// should we wait for data?
        idbg=0;
#ifdef NC353
        while ((sleep[0]!=0) && ((numBytesWritten-thisNumBytesRead)<= 64)) { // last 32 bytes can get stuck in ETRAX dma channel
            schedule_usleep(*sleep); // ETRAX-specific wait, replace!
            updateNumBytesWritten();
            idbg++;
        }
#else
        wait_event_interruptible(logger_wait_queue, (sleep[0]==0) || ((numBytesWritten-thisNumBytesRead) > 64)); // AF2016 Why sleep[0] here?
#endif
        dev_dbg(g_dev_ptr,"After wait_event_interruptible: numBytesWritten=0x%08x thisNumBytesRead=0x%08x\n", (int) numBytesWritten, (int) thisNumBytesRead);

        if (idbg>0) {
            dev_dbg(g_dev_ptr,"slept %d times (%d usec)\n", idbg, (int) (*sleep * idbg));
        }
        // now read what is available (and required), roll over the buffer (if needed), copy data and advance numReadBytes
//TODO:Flush cache !!!!!!!!!!!!!!!!!!!!!!*********************************


        byteIndexRead=thisNumBytesRead & bytePtrMask; // requires buffer size to be 2**N
        byteIndexValid=(numBytesWritten-64) & bytePtrMask; // one record less to mitigate data hidden in ETRAX dma buffer
        dev_dbg(g_dev_ptr,"byteIndexRead=0x%08x byteIndexValid=0x%08x bytePtrMask=0x%08x\n", byteIndexRead, byteIndexValid, (int)bytePtrMask);
#ifdef NC353
        if (byteIndexValid<byteIndexRead) byteIndexValid += (CCAM_DMA1_SIZE<<2);
#else
        if (byteIndexValid<byteIndexRead) byteIndexValid += logger_size;
#endif
        if (count>(byteIndexValid-byteIndexRead)) count = (byteIndexValid-byteIndexRead);
        leftToRead=count;
        pe=byteIndexRead+leftToRead;
        dev_dbg(g_dev_ptr,"byteIndexValid=0x%08x count=0x%08x pe=0x%08x\n", byteIndexValid, (int) count, pe);
#ifdef NC353
        if (pe>(CCAM_DMA1_SIZE<<2)) pe=(CCAM_DMA1_SIZE<<2);
#else
//        if (pe>(logger_size << PAGE_SHIFT)) pe= logger_size;
        if (pe > logger_size) pe= logger_size;
#endif
        /// copy all (or first part)
        err=copy_to_user(buf, &charDMABuf[byteIndexRead], (pe-byteIndexRead));
        if (err) {
            dev_err(g_dev_ptr,"1. tried to copy 0x%x bytes from offset 0x%llx, result=0x%x\n", (pe-byteIndexRead), *off,err);
//[  811.889488] imu_logger elphel393-logger@0: 1. tried to copy 0x1000 bytes to offset 0x0, result=0x400000
            return -EFAULT;
        }
        //      advance pointers
        leftToRead -=      (pe-byteIndexRead);
        thisNumBytesRead+= (pe-byteIndexRead);
        dev_dbg(g_dev_ptr,"leftToRead=0x%08x thisNumBytesRead=0x%08x\n", leftToRead, (int) thisNumBytesRead);
        ///Do we need to copy from the beginning of the buffer?
        if (leftToRead>0) {
            //          err=copy_to_user(buf, &charDMABuf[0], leftToRead);
            err=copy_to_user(&buf[pe-byteIndexRead], &charDMABuf[0], leftToRead);
            byteIndexRead=0;
        }
        if (err) {
            dev_err(g_dev_ptr,"2. tried to copy 0x%x bytes to offset 0x%llx, result=0x%x\n", count, *off,err);
            return -EFAULT;
        }
        thisNumBytesRead+=leftToRead;
        dev_dbg(g_dev_ptr,"thisNumBytesRead=0x%08x\n", (int) thisNumBytesRead);

#ifdef NC353
        //Is it just for debug
        stat =    REG_RD(dma, regi_dma7, rw_stat);
        ch1_stat= REG_RD(bif_dma, regi_bif_dma, r_ch1_stat);
        dev_dbg(g_dev_ptr,"count=0x%x, thisNumBytesRead=0x%llx, numBytesWritten=0x%llx, stat.buf=0x%x, stat.mode=%x, ch1.run=%x ch1.cnt=0x%x\n", (int) count, thisNumBytesRead,  numBytesWritten, (int) stat.buf,(int) stat.mode, (int) ch1_stat.run, (int) ch1_stat.cnt );
#endif
        //printk(" file->f_mode & FMODE_WRITE=0x%d\n",file->f_mode & FMODE_WRITE);
        if (file->f_mode & FMODE_WRITE) numBytesRead=thisNumBytesRead;
        //        else *off=thisNumBytesRead;
        *off=thisNumBytesRead; // always update
        if (count<0) {
            dev_err(g_dev_ptr,"Count is negative ( 0x%x)\n", count);
        }
        dev_dbg(g_dev_ptr,"count=0x%08x\n", (int) count);
        return count;
    default:
        //printk(" Wrong minor=0x%x\n",((int *)file->private_data)[0]);
        dev_dbg(g_dev_ptr," Wrong minor=0x%x\n",(int)file->private_data);
        return -EINVAL;
    }
}

/** Handle interrupts the logger. This handler is installed without SA_INTERRUPT
 * flag meaning that interrupts are enabled during processing. Such behavior is recommended in LDD3. */
static irqreturn_t logger_irq_handler(int irq,      ///< [in] interrupt number
                                      void *dev_id) ///< [in] pointer to driver's private data structure
                                                    ///< @return \e IRQ_HANDLED if interrupt was processed and \e IRQ_NONE otherwise
{
    x393_mult_saxi_al_t mult_saxi_dwp = x393_mult_saxi_pointers(MULT_SAXI_CHN);
    if (mult_saxi_dwp.addr32 < logger_offs32){
        numBytesWrittenBase += logger_size;
    }
    logger_offs32 = mult_saxi_dwp.addr32;
    numBytesWritten = numBytesWrittenBase + (logger_offs32 <<2);
    logger_irq_cmd(X393_IRQ_RESET);
    wake_up_interruptible(&logger_wait_queue);
//thisFPGABytes
    return IRQ_HANDLED;
}



static int logger_init(struct platform_device *pdev)
{
    unsigned int irq;
    int res, i;
    struct device *dev = &pdev->dev;
    const struct of_device_id *match;
    const char *logger_irq_names[4] = {"mult_saxi_0", "mult_saxi_1", "mult_saxi_2", "mult_saxi_3"};
    // char device for sensor port
    struct device *chrdev;

    g_dev_ptr = dev;

    /* sanity check */
    match = of_match_device(elphel393_logger_of_match, dev);
    if (!match)
        return -EINVAL;

    dev_dbg(dev, "Registering character device with name "DEV393_NAME(DEV393_LOGGER));
    res = register_chrdev(DEV393_MAJOR(DEV393_LOGGER), DEV393_NAME(DEV393_LOGGER), &imu_fops);
    if(res < 0) {
        dev_err(dev, "\nlogger_init: couldn't get a major number  %d.\n ",DEV393_MAJOR(DEV393_LOGGER));
        return res;
    }
    dev_info(dev,DEV393_NAME(DEV393_LOGGER)"- %d\n",DEV393_MAJOR(DEV393_LOGGER));

	// create device class
	imu_dev_class = class_create(THIS_MODULE, DEV393_NAME(DEV393_LOGGER));
	if (IS_ERR(imu_dev_class)) {
		pr_err("Cannot create \"%s\" class", DEV393_NAME(DEV393_LOGGER));
		return PTR_ERR(imu_dev_class);
	}

	// create devices
	for (i=0;i<(sizeof(imu_minor)/sizeof(int));i++){
		chrdev = device_create(
				imu_dev_class,
				  &pdev->dev,
				  MKDEV(imu_major, imu_minor[i]),
				  NULL,
				  "%s",imu_devs[i]);
		if(IS_ERR(chrdev)){
			pr_err("Failed to create a device (imu_log). Error code: %ld\n",PTR_ERR(chrdev));
		}
	}

    // Setup memory buffer from CMA
    logger_buffer = (u32 *) pElphel_buf->logger_vaddr; // must be page-aligned!
    logger_size = pElphel_buf->logger_size << PAGE_SHIFT;
    bytePtrMask = logger_size - 1;
    logger_phys =   pElphel_buf->logger_paddr;
    dma_is_on = 0;
    // Setup interrupt
    irq = platform_get_irq_byname(pdev, logger_irq_names[MULT_SAXI_CHN]);
    if (request_irq(irq,
            logger_irq_handler,
            0, // no flags
            logger_irq_names[MULT_SAXI_CHN],
            NULL)) {
        dev_err(dev, "can not allocate interrupts for %s\n",logger_irq_names[MULT_SAXI_CHN]);
        return -EBUSY;
    }
//MULT_SAXI_CHN
    init_waitqueue_head(&logger_wait_queue);    // wait queue for logger
    //    init_ccam_dma1_buf_ptr();
    g_dev_ptr = dev; // for debugfs
    return 0;
}

/** Initialize FPGA DMA engine for the logger. Obviously requires bitstream to be loaded. */
int logger_init_fpga(int force) ///< if 0, only do if not already initialized
{
    x393_status_ctrl_t logger_status_ctrl=    {.d32 = 0};
    x393_status_ctrl_t mult_saxi_status_ctrl= {.d32 = 0};
    x393_mult_saxi_al_t    mult_saxi_a=   {.d32=0};
    x393_mult_saxi_al_t    mult_saxi_l=   {.d32=0};
    x393_mult_saxi_irqlen_t mult_saxi_irqlen=   {.d32=0};
    if (logger_fpga_configured && !force) return 0; // Already initialized
    mult_saxi_a.addr32 = logger_phys >> 2; // in DWORDs
    x393_mult_saxi_buf_address(mult_saxi_a,      MULT_SAXI_CHN);
    mult_saxi_l.addr32 = logger_size >> 2;
    x393_mult_saxi_buf_len    (mult_saxi_l,      MULT_SAXI_CHN);
    mult_saxi_irqlen.irqlen = LOGGER_IRQ_DW_BIT;
    x393_mult_saxi_irqlen     (mult_saxi_irqlen, MULT_SAXI_CHN);
    logger_status_ctrl.mode = LOGGER_STATUS_MODE;
    set_x393_logger_status_ctrl(logger_status_ctrl);
    if (MULT_SAXI_STATUS_MODE) { // do not set (overwrite other channels if 0)
        mult_saxi_status_ctrl.mode = MULT_SAXI_STATUS_MODE;
        set_x393_mult_saxi_status_ctrl(mult_saxi_status_ctrl);
    }
    // resets (do once?)
    logger_dma_ctrl(0); ///reset DMA
#if LOGGER_USE_IRQ
    logger_irq_cmd(X393_IRQ_RESET);
    logger_irq_cmd(X393_IRQ_ENABLE);
#endif /* LOGGER_USE_IRQ */
    logger_fpga_configured = 1;
    return 0;
}

/** DMA control for the logger (mult_saxi channel 0) */
int logger_dma_ctrl(int cmd) ///< commands: 0 - reset, 1 - stop, 2 - run
                             ///<@return 0/-EINVAL
{
    x393_mult_saxi_mode_t mult_saxi_mode= {.d32=0};
    int en,run;
    BUG_ON (!logger_buffer);
    mult_saxi_mode = get_x393_mult_saxi_mode(); // Now not needed, later to take care about other channels
    switch (cmd){
    case LOGGER_DMA_RESET: en=0; run=0; break;
    case LOGGER_DMA_STOP:  en=1; run=0; break;
    case LOGGER_DMA_RUN:   en=1; run=1; break;
    default: return -EINVAL;
    }
    if (!en) {
        logger_offs32 = 0; // byte offset in the buffer
        numBytesWritten=0;     ///< total number of bytes written to the IMU buffer since it was started/restarted
        numBytesWrittenBase=0; ///< number of byte written in full buffers  since it was started/restarted
    }
#if    MULT_SAXI_CHN == 0
    mult_saxi_mode.en0=en; mult_saxi_mode.run0=run;
#elif  MULT_SAXI_CHN == 1
    mult_saxi_mode.en1=en; mult_saxi_mode.run1=run;
#elif  MULT_SAXI_CHN == 2
    mult_saxi_mode.en2=en; mult_saxi_mode.run2=run;
#elif  MULT_SAXI_CHN == 3
    mult_saxi_mode.en3=en; mult_saxi_mode.run3=run;
#endif
    set_x393_mult_saxi_mode(mult_saxi_mode);
    return 0;
}

void logger_irq_cmd(int cmd) ///< interrupt command: 0 - nop, 1 - reset, 2 - disable, 3 - enable
{
    x393_mult_saxi_interrupts_t mult_saxi_interrupts= {.d32=0};
#if    MULT_SAXI_CHN == 0
    mult_saxi_interrupts.interrupt_cmd0 = cmd;
#elif  MULT_SAXI_CHN == 1
    mult_saxi_interrupts.interrupt_cmd1 = cmd;
#elif  MULT_SAXI_CHN == 2
    mult_saxi_interrupts.interrupt_cmd2 = cmd;
#elif  MULT_SAXI_CHN == 3
    mult_saxi_interrupts.interrupt_cmd3 = cmd;
#endif
    x393_mult_saxi_interrupts(mult_saxi_interrupts);
}
#ifdef NC353
///TODO: it seems we could use a single data descriptor (in LX data segment was limited to 16KB), but let's have minimal changes
//#define DMA_CHUNK 0x4000 // 32-bit words - may increase??
//#define CCAM_DESCR_DATA_NUM (( CCAM__DMA_SIZE  / DMA_CHUNK) +1 ) // number of data descriptors
#define CCAM_DESCR1_DATA_NUM (( CCAM__DMA1_SIZE  / DMA_CHUNK) +1 ) // number of data descriptors

static dma_descr_data    ccam_dma1_descr_data    [CCAM_DESCR1_DATA_NUM]  __attribute__ ((__aligned__(16)));
static dma_descr_context ccam_dma1_descr_context __attribute__ ((__aligned__(32)));

int            x313_setDMA1Buffer(void);
unsigned long  x313_DMA1_size (void);
#endif

/**
 * @brief tests if ETRAX DMA1 is running
 * @return 1 - DMA is on, 0 - DMA is off
 */
int           logger_is_dma_on(void) {
    return dma_is_on;
}

/**
 * @brief Stop ETRAX DMA1
 * @return 0
 */
int logger_dma_stop(void) {
    dma_is_on=0;
    dev_dbg(g_dev_ptr,"==========logger_dma_stop\n");
#ifdef NC353
    port_csp0_addr[X313_WA_DMACR] = 0x20; // disable DMA1, dot't modify DMA0
    EXT_DMA_1_STOP ; /// for testing - no reset DMA after acquisition
    udelay(10) ; //?
    DMA_RESET( regi_dma7 );
    // put here restoring of the .after pointer ?
#else
    logger_dma_ctrl(LOGGER_DMA_STOP);
    // TODO: stop logger first
#endif
    return 0;
}


/**
 * @brief Start ETRAX DMA for the IMU
 */

void logger_dma_start(void) {
#ifdef NC353
    unsigned long dai;
    int i = 0;
    dev_dbg(g_dev_ptr,"----------logger_dma_start\n");
    DMA_RESET(regi_dma7);
    /// need to restore pointers after previous stop DMA - maybe later move there?
    for(dai = 0; dai < CCAM_DMA1_SIZE; dai += DMA_CHUNK) { /// DMA_CHUNK==0x4000
        if(dai + DMA_CHUNK >= CCAM_DMA1_SIZE)  /// last descriptor
            ccam_dma1_descr_data[i].after = (char *)virt_to_phys(&logger_buffer[CCAM_DMA1_SIZE]);
        else  /// not the last one
            ccam_dma1_descr_data[i].after = (char *)virt_to_phys(&logger_buffer[dai + DMA_CHUNK]);
        //!TODO: does flush here IS IT STILL NEEDED HERE?
        flush_dma_descr( & ccam_dma1_descr_data[i], 0);
        i++;
    }
    DMA_ENABLE(regi_dma7);
    port_csp0_addr[X313_WA_DMACR] = 0x20; // disable DMA1, don't modify DMA0
    /// NOTE: needs to be here (not in x313_dma1_init) - word width is reset by channel reset !!!
    DMA_WR_CMD(regi_dma7, regk_dma_set_w_size4);  ///32-bit transfers
    /// point to the beginning of the buffer?
    ccam_dma1_descr_context.saved_data = (dma_descr_data*)virt_to_phys(&ccam_dma1_descr_data[0]);
    ccam_dma1_descr_context.saved_data_buf = ccam_dma1_descr_data[0].buf;
    //! need this also?
    flush_dma_descr((dma_descr_data*) & ccam_dma1_descr_context, 0);
    DMA_START_CONTEXT(regi_dma7, virt_to_phys(&ccam_dma1_descr_context));
    EXT_DMA_1_START ;
    port_csp0_addr[X313_WA_DMACR] = 0x28; // enable DMA1, don't modify DMA0
#else
    logger_dma_ctrl(LOGGER_DMA_RUN);
#endif
    dev_dbg(g_dev_ptr,"----------logger_dma_start\n");
    dev_info(g_dev_ptr,"----------logger_dma_start\n");
    dma_is_on=1;
}



///dma0 is using external dma 3 (input) with dma channel 9 
///dma1 (this) is using external dma 1 (input) with dma channel 7 (shared with async. serial 0, so do not use DMA there!)
unsigned long x313_dma1_init(void) {
#ifdef NC353
    int rslt;
    reg_dma_rw_cfg cfg = {.en = regk_dma_yes}; //  if disabled - will be busy and hang on attemt of DMA_WR_CMD
    reg_bif_dma_rw_ch1_ctrl exdma_ctrl = {
            .bw          = regk_bif_dma_bw32,
            .burst_len   = regk_bif_dma_burst8, // changed - updated FPGA to use 8-word bursts
            .cont        = 1,                   // continuous transfer mode (probably - don't care)
            .end_discard = 0,                   // discard end of burst date (don't care)
            .cnt         = 0,                   // transfer counter ignored
            .dreq_pin    = 2,                   // use hsh2
            .dreq_mode   = regk_bif_dma_norm,   // normal - active high DREQ from pin (see above)
            .tc_in_pin   = 0,                   // don't care - tc pin number
            .tc_in_mode  = 0,                   // no tc pin
            .bus_mode    = regk_bif_dma_master, // bus mode - master
            .rate_en     = 0                    // no rate limiting
    };
    reg_bif_dma_rw_ch1_addr exdma_addr = {.addr = ( MEM_CSR0_START + 4 ) | MEM_NON_CACHEABLE}; // fpga register 1
    reg_bif_dma_rw_pin2_cfg exdma_pin2 = {
            .master_ch   = 0,                   // don't care
            .master_mode = regk_bif_dma_off,    // off
            .slave_ch    = 0,                   // don't care
            .slave_mode  = regk_bif_dma_off     // off
    };
    reg_bif_dma_rw_pin3_cfg exdma_pin3 = {
            .master_ch   = 1,                   // ext DMA channel #
            .master_mode = regk_bif_dma_dack,   // use DACK, active high
            .slave_ch    = 1,                   // don't care
            .slave_mode  = regk_bif_dma_off     // off
    };
    // just in case - free DMA channel (we are only using it here)
    crisv32_free_dma(EXTDMA1_RX_DMA_NBR);
    dev_dbg(g_dev_ptr,"Initializing DMA registers for EXTDMA1\n");
    dev_dbg(g_dev_ptr,"x313_dma1_init(void)");

    dev_dbg(g_dev_ptr,"before crisv32_request_dma\n"); udelay (500000);
    rslt = crisv32_request_dma(EXTDMA1_RX_DMA_NBR,
            "imu data in from fpga",
            DMA_VERBOSE_ON_ERROR,
            0,
            dma_ext1);
    dev_dbg(g_dev_ptr,"after crisv32_request_dma - result=%d\n",rslt);  udelay(500000);


    if(rslt) {
        dev_dbg(g_dev_ptr,"failed\n");
        crisv32_free_dma(EXTDMA1_RX_DMA_NBR);
        dev_dbg(g_dev_ptr,"Can't allocate external dma port for compressed data in from fpga");
    } else { /// dma channel 7 allocated for ext dma 1
        /// setup source of hsh2, hsh3
        REG_WR(bif_dma, regi_bif_dma, rw_pin2_cfg, exdma_pin2); /// just in case - turn hsh2 off
        REG_WR(bif_dma, regi_bif_dma, rw_pin3_cfg, exdma_pin3); /// make hsh3 DACK
        /// Configure ext DMA 3
        REG_WR(bif_dma, regi_bif_dma, rw_ch1_ctrl, exdma_ctrl);
        REG_WR(bif_dma, regi_bif_dma, rw_ch1_addr, exdma_addr);
        REG_WR(dma, regi_dma7, rw_cfg, cfg);   ///  DMA configuration (bit 0 - enable, bit 1 - stop) - stopped
    }
    ///    DMABufferLength = 0;
    x313_setDMA1Buffer();
    return ((unsigned long)virt_to_phys(logger_buffer)) | 0x80000000;
#endif
    dma_is_on=0;
    return 0;
}

#ifdef NC353
int x313_setDMA1Buffer(void) {
    unsigned long dai;
    int i = 0;
    EXT_DMA_1_STOP; /// Stop DMA1 (just in case)
    for(dai = 0; dai < CCAM_DMA1_SIZE; dai += DMA_CHUNK) { /// DMA_CHUNK==0x4000
        ccam_dma1_descr_data[i].buf = (char *)virt_to_phys(&logger_buffer[dai]);
        ccam_dma1_descr_data[i].intr = 0;
        ccam_dma1_descr_data[i].wait = 0;
        ccam_dma1_descr_data[i].eol = 0; /// we probably do not need to use eol as the descriptors are linked in a loop anyway
        if(dai + DMA_CHUNK >= CCAM_DMA1_SIZE) { ///last descriptor
            ccam_dma1_descr_data[i].after = (char *)virt_to_phys(&logger_buffer[CCAM_DMA1_SIZE]);
            ccam_dma1_descr_data[i].next = (dma_descr_data*)virt_to_phys(&ccam_dma1_descr_data[0]);
        } else { /// not the last one
            ccam_dma1_descr_data[i].after = (char *)virt_to_phys(&logger_buffer[dai + DMA_CHUNK]);
            ccam_dma1_descr_data[i].next = (dma_descr_data*)virt_to_phys(&ccam_dma1_descr_data[i + 1]);
        }
        flush_dma_descr( & ccam_dma1_descr_data[i], 0);
        i++;
    }
    // TODO: make new global parameter?
    //    set_globalParam (G_CIRCBUFSIZE,CCAM__DMA_SIZE<<2); /// make it adjustable? TODO: initialize with others?
    //*********************** TEMPORARY ********************************
    dev_dbg(g_dev_ptr,"filling DMA1 buffer with natural numbers - just test \n");
    for(dai = 0; dai < CCAM_DMA1_SIZE; dai++) logger_buffer[dai] = dai;
    return 0;
}
#endif



/** IMU/GPS logger driver remove function */
static int logger_remove(struct platform_device *pdev) ///< [in] pointer to @e platform_device structure
                                                       ///< @return always 0
{
	int i;
	for (i=0;i<(sizeof(imu_minor)/sizeof(int));i++){
		device_destroy(
			imu_dev_class,
			MKDEV(imu_major, imu_minor[i]));
	}
    unregister_chrdev(DEV393_MAJOR(DEV393_LOGGER), DEV393_NAME(DEV393_LOGGER));
    return 0;
}

static const struct of_device_id elphel393_logger_of_match[] = {
        { .compatible = "elphel,elphel393-logger-1.00" },
        { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_logger_of_match);

static struct platform_driver elphel393_logger = {
        .probe          = logger_init,
        .remove         = logger_remove,
        .driver = {
                .name = DEV393_NAME(DEV393_LOGGER),
                .of_match_table = elphel393_logger_of_match,
        },
};

module_platform_driver(elphel393_logger);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(IMU_MODULE_DESCRIPTION);
