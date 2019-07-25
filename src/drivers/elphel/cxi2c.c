/** @file cxi2c.c
 *
 * @brief Pre-393 I2c driver for FPGA communicating to sensors, software implementation
 * Porting to get GPS communication, sesnors in NC393 are handled by sensor_i2c.c driver
 *
 * @copyright Copyright (C) 2002-2016 Elphel, Inc
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

/*********************************************************************************
 *! -----------------------------------------------------------------------------**
 *!  $Log: cxi2c.c,v $
 *!  Revision 1.7  2012/01/16 01:44:53  elphel
 *!  added comments, and release of SDA after reading byte
 *!
 *!  Revision 1.6  2011/12/22 05:37:09  elphel
 *!  added more known devices
 *!
 *!  Revision 1.5  2011/08/13 00:53:10  elphel
 *!  defined addresses for "granddaughter" ID eeprom and 8-bit ports
 *!
 *!  Revision 1.4  2010/08/10 21:09:23  elphel
 *!  improve i2c access to sensor board eeprom - turning off hardware i2c and disabling interrupts during software r/w operations
 *!
 *!  Revision 1.3  2010/04/06 20:35:42  elphel
 *!  8.0.7.5 - made the fpgaclock driver program 10359 clock in addition to the system one
 *!
 *!  Revision 1.2  2008/12/24 13:27:24  spectr_rain
 *!  removed io board pin ioctl
 *!
 *!  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
 *!
 *!
 *!  Revision 1.7  2008/09/20 00:29:49  elphel
 *!  moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
 *!
 *!  Revision 1.6  2008/09/12 00:23:58  elphel
 *!  removed cc353.c, cc353.h
 *!
 *!  Revision 1.5  2008/09/05 23:20:26  elphel
 *!  just a snapshot
 *!
 *!  Revision 1.4  2008/08/25 19:07:23  elphel
 *!  just a snapshot
 *!
 *!  Revision 1.3  2008/07/27 04:27:49  elphel
 *!  next snapshot
 *!
 *!  Revision 1.2  2008/06/19 02:17:36  elphel
 *!  continuing work - just a snapshot
 *!
 *!  Revision 1.7  2008/04/11 23:16:51  elphel
 *!  removed unneeded local_irq_disable() after local_irq_save_flags()
 *!
 *!  Revision 1.6  2008/03/16 01:25:15  elphel
 *!  increased default delays fro I2c bus 1 (EEPROM was not fast enough)
 *!
 *!  Revision 1.5  2008/02/18 20:02:34  elphel
 *!  added option to specify number of bytes to be actually written from device per read command (PHP feature fix)
 *!
 *!  Revision 1.4  2008/02/12 21:53:20  elphel
 *!  Modified I2c to support multiple buses, added raw access (no address registers) and per-slave protection bitmasks
 *!
 *!  Revision 1.3  2008/02/11 04:52:18  elphel
 *!  Modified I2C operations, added second bus to work with the 10369 (and future) board(s)
 *!
 *!  Revision 1.2  2007/10/16 23:18:31  elphel
 *!  added filtering I2C lines, r/w control of the I2C bit delays
 *!
 */

/****************** INCLUDE FILES SECTION ***********************************/

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/io.h>

#include <asm/irq.h>

#include <linux/platform_device.h> // For sysfs
#include <linux/of_device.h>
//#include <linux/of.h>
//#include <linux/of_fdt.h>
//#include <linux/of_net.h>
#include <linux/sysfs.h>

#include <asm/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/x393_devices.h>
#include <uapi/elphel/c313a.h>
#include "x393.h"

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr

//#include "x3x3.h"


#include "cci2c.h"

#define D(x)
//#define D(x) printk("%s:%d:",__FILE__,__LINE__);x

//#define PASSIVE_PULLUP y // Enable to remove active pullup on SCL SDA (just use pullup resistors)


// implementing read/write/lseek for i2c - using a new major and several minors:
// for 8 and 16 registers with and w/o autoincrement.
// next minors - for less commomn i2c devices.
// address space is considered to include 1 byte of register address and 7MSBs of slave address


/****************** I2C DEFINITION SECTION *************************/
#define CLOCK_LOW_TIME            8
#define CLOCK_HIGH_TIME           8
#define START_CONDITION_HOLD_TIME 8
#define STOP_CONDITION_HOLD_TIME  8
#define ENABLE_OUTPUT 0x01
#define ENABLE_INPUT 0x00
#define I2C_CLOCK_HIGH 1
#define I2C_CLOCK_LOW 0
#define I2C_DATA_HIGH 1
#define I2C_DATA_LOW 0

/* use the kernels delay routine */

//#define i2c_delay(usecs) udelay(usecs)
//#define i2c_delay(usecs) {}

//Just removing delays is absolutely wrong (does not work with other sensors) what can be done - replacing constants with some run-time value(s) and set them individually for different sensors. For now - I'll use the standard values again.

#define i2c_delay(usecs) udelay(usecs)
#define I2C_DELAY_SCALE 1 // with I2C_DELAY_SCALE==1 SCL period is 900 KHz - using udelay?
//#define X3X3_I2C_MAXMINOR 4  //
//#define X3X3_I2C_CHANNELS 2  // number of i2c channels

//volatile unsigned long ccam_cr_shadow=0;
//extern volatile unsigned long ccam_cr_shadow;

// currently delays are approximately  0.4usec+0.2usec*n and 0x01010000 works (~8usec/byte)
// with deafult 20MHz pixel clock - 0x01010202, after clock is set to 48MHz 0x01010000 is enough
//TODO: Make delays independent for 2 channels?
static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
static struct i2c_timing_t bitdelays[X3X3_I2C_CHANNELS];
static int xi2c_initialized=0; // configure GPIO puins access at first command;

#ifdef NC353
static int i2c_hardware_on=0; // shadow register for FPFA I2C controller
#endif

static const char * const xi2c_devs[]={
	DEV393_DEVNAME(DEV393_I2C_CTRL),
	DEV393_DEVNAME(DEV393_I2C_8_AINC),
	DEV393_DEVNAME(DEV393_I2C_16_AINC),
	DEV393_DEVNAME(DEV393_I2C1_8_AINC),
	DEV393_DEVNAME(DEV393_I2C1_16_AINC),
	DEV393_DEVNAME(DEV393_I2C_RAW),
	DEV393_DEVNAME(DEV393_I2C1_RAW),
	DEV393_DEVNAME(DEV393_I2C_ENABLE)
};

static const int xi2c_major = DEV393_MAJOR(DEV393_I2C_CTRL);

static const int xi2c_minor[]={
	DEV393_MINOR(DEV393_I2C_CTRL),
	DEV393_MINOR(DEV393_I2C_8_AINC),
	DEV393_MINOR(DEV393_I2C_16_AINC),
	DEV393_MINOR(DEV393_I2C1_8_AINC),
	DEV393_MINOR(DEV393_I2C1_16_AINC),
	DEV393_MINOR(DEV393_I2C_RAW),
	DEV393_MINOR(DEV393_I2C1_RAW),
	DEV393_MINOR(DEV393_I2C_ENABLE)
};

/** @brief Global device class for sysfs */
static struct class *xi2c_dev_class;

//void i2c_disable(int n);
//void i2c_dir_out(int n);
//void i2c_dir_in (int n);
void i2c_scl_0  (int n);
void i2c_scl_1  (int n);
//void i2c_sda    (int n, int d);    /* d is checked against zero only, it does not need to be "1" */
int i2c_getbit    (int n);
int i2c_getscl(int n);            /* just for i2c testing */
//int i2c_diagnose(int n);
int i2c_start(int n);
int i2c_restart(int n);
int i2c_stop(int n);
int i2c_outbyte(int n, unsigned char d);
unsigned char i2c_inbyte(int n, int more);
static void test_init_GPIO(void);
//void i2c_sendack(int n, int ackn); // ackn= 1 - send ackn (low level), 0 - no ackn.

// the following functions should be called with IRQ off. Maybe will replace with FPGA register read
#ifdef NC353
void i2c_reset_wait(void) {X3X3_I2C_RESET_WAIT;i2c_hardware_on=0;}
void i2c_stop_wait(void) {X3X3_I2C_STOP_WAIT;  i2c_hardware_on=0;}
void i2c_run(void)       {X3X3_I2C_RUN;        i2c_hardware_on=1;}
int  i2s_running(void)   {return i2c_hardware_on;}
#endif

#define i2c_ldelay(usecs) udelay(usecs)

/*
void i2c_ldelay(int dly){
    while (dly--) {
        __asm__ __volatile__("");
    }
}
*/
// Low level i2c pin functions


int i2c_delays (unsigned long delays) { // will only change bus 0 this way
    unsigned long * bitdelays_long=(unsigned long *) bitdelays;
    if (delays !=0) bitdelays_long[0]=delays;
    return (int) bitdelays_long[0];
}

/* I2C functions */
// redesign of i2c functions optimized for faster communications,
// including FPGA <->FPGA transfers.
// To make communications faster there are the following steps:
// 1 - SCL is always driven by master - no waiting for the slave
// 2 - SDA is actively driven high by the master (and expecting the same from the slave)
//     SDA is driven high during SCL=0 when other party is known not to drive it low.
// 3 - separate bus turn-over delays are added in addition to (shorter) bit delays
// 4 - 4 delays are programmed as a single 32-bit word that can be changed to communicate with
//     different slaves (i.e. FPGA and image sensor)
/*
#define x3x3_DELAY(x) {int iiii; for (iiii=0; iiii < (x); iiii++) X3X3_AFTERWRITE ; }
I2C_DELAY_SCALE
    #define   X313_WA_IOPINS    0x70  // bits [13:12] selecte  of the source of the control word:
    SCL=EXT[0]
    SDA=EXT[1]
/// software control of SDA0, SCL0 lines (when hardware i2c is off)
  #define X3X3_I2C_SCL_0_BITS   0x10000 
  #define X3X3_I2C_SCL_1_BITS   0x20000 
  #define X3X3_I2C_SCL_Z_BITS   0x30000 
  #define X3X3_I2C_SDA_0_BITS   0x40000 
  #define X3X3_I2C_SDA_1_BITS   0x80000 
  #define X3X3_I2C_SDA_Z_BITS   0xc0000 
X313_I2C_CMD
 */
#define SCL1_0   0x1
#define SCL1_OFF 0x3
#define SDA1_0   0x4
#define SDA1_OFF 0xc

#ifdef PASSIVE_PULLUP
#define SCL1_1   0x3  // same as off
#define SDA1_1   0xc  // same as off
#else
#define SCL1_1   0x2
#define SDA1_1   0x8
#endif

#define X313_PIOR__SCL1    0
#define X313_PIOR__SDA1    1
#ifdef NC353
#define X313_PIOR(x) ((port_csp0_addr[X313__RA__IOPINS] >> X313_PIOR__##x ) & 1)
#else
#define X313_PIOR(x) ((read_gpio() >> X313_PIOR__##x ) & 1)
#endif


u32 read_gpio(void)
{
    x393_gpio_status_t gpio_status;
    x393_status_ctrl_t stat_ctrl;
    int i;
    stat_ctrl.d32 = 0;
    gpio_status = x393_gpio_status();
    stat_ctrl.seq_num = gpio_status.seq_num + 1;
    stat_ctrl.mode = 1;
    set_x393_gpio_status_control(stat_ctrl);
    for (i = 0; i < 10; i++) {
        gpio_status = x393_gpio_status();
        if (likely(gpio_status.seq_num == stat_ctrl.seq_num)) {
            return gpio_status.d32 & 0x3ff; // lower 10 bits
        }
    }
    dev_err(NULL,"read_gpio(): failed to  get expected seq_num in 10 cycles, expected = 0x%x, got 0x%x\n",stat_ctrl.seq_num, gpio_status.seq_num);
    return gpio_status.d32 & 0x3ff;
}



/*
struct i2c_timing_t {
0        unsigned char scl_high; //0x02, //! SCL high:
1        unsigned char scl_low;  //0x02, //! SCL low:
2        unsigned char slave2master; //0x01, //! slave -> master
3        unsigned char master2slave; //0x01, //! master -> slave
4        unsigned char filter_sda;   //0x07, //! filter SDA read data by testing multiple times - currently just zero/non zero
5        unsigned char filter_scl;  //0x07};//! filter SCL read data by testing multiple times - currently just zero/non zero 
}
 */
// filtering noise/interference by repeating measurement 7 times and using the majority
int i2c_getbit(int n) {
#ifdef NC353
    if (bitdelays[n].filter_sda)
        return n? (((X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1))) >> 2) :
                (((X313_SR(SDA0)) + (X313_SR(SDA0)) + (X313_SR(SDA0)) + (X313_SR(SDA0)) + (X313_SR(SDA0)) + (X313_SR(SDA0)) + (X313_SR(SDA0))) >> 2) ;
    else     return n? X313_PIOR(SDA1) : X313_SR(SDA0);
#else
    if (bitdelays[n].filter_sda)
        return    (((X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1)) + (X313_PIOR(SDA1))) >> 2);
    else     return    X313_PIOR(SDA1);

#endif
}

int i2c_getscl(int n) {
#ifdef NC353
    if (bitdelays[n].filter_scl)
        return n? (((X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1))) >> 2) :
                (((X313_SR(SCL0)) + (X313_SR(SCL0)) + (X313_SR(SCL0)) + (X313_SR(SCL0)) + (X313_SR(SCL0)) + (X313_SR(SCL0)) + (X313_SR(SCL0))) >> 2) ;
    else     return n? X313_PIOR(SCL1) : X313_SR(SCL0);
#else
    if (bitdelays[n].filter_scl)
        return    (((X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1)) + (X313_PIOR(SCL1))) >> 2) ;
    else     return    X313_PIOR(SCL1);
#endif
}


void i2c_scl_0(int n) {
#ifdef NC353
    if (n) port_csp0_addr[X313_WA_IOPINS]=SCL1_0; // EXT[0] enabled, 0
    else   port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SCL_0_BITS;  // SCL= 0
#else
    x393_gpio_set_pins_t gpio_set_pins = {.d32 = SCL1_0};
    x393_gpio_set_pins  (gpio_set_pins);
#endif
}


void i2c_scl_1(int n) {
#ifdef NC353
    if (n)  port_csp0_addr[X313_WA_IOPINS]=SCL1_1; // modified by ifdef PASSIVE_PULLUP
    else   port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SCL_1_BITS;; // SCL=1
#else
    x393_gpio_set_pins_t gpio_set_pins = {.d32 = SCL1_1};
    x393_gpio_set_pins  (gpio_set_pins);
#endif
}


void i2c_sda_weak (int n, int d) {    // will also force sda enable
#ifdef NC353
    if (n) {
        if (d) port_csp0_addr[X313_WA_IOPINS]=SDA1_OFF; // turn off SDA
        else   port_csp0_addr[X313_WA_IOPINS]=SDA1_0; // turn on SDA ==0
    } else {
        if (d) port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SDA_Z_BITS; // turn off SDA ==1 (just in case)
        else   port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SDA_0_BITS; // turn on SDA ==0
    }
#else
    const x393_gpio_set_pins_t gpio_sda1_off = {.d32 = SDA1_OFF};
    const x393_gpio_set_pins_t gpio_sda1_0 =   {.d32 = SDA1_0};
    if (d) x393_gpio_set_pins(gpio_sda1_off); // turn off SDA
    else   x393_gpio_set_pins(gpio_sda1_0);   // turn on SDA ==0
#endif
}

void i2c_sda_strong (int n, int d) {    // will also force sda enable
#ifdef NC353
    if (n) {
        if (d) port_csp0_addr[X313_WA_IOPINS]=SDA1_1; // modified by ifdef PASSIVE_PULLUP
        else   port_csp0_addr[X313_WA_IOPINS]=SDA1_0; // turn on SDA ==0
    } else {
        if (d) port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SDA_1_BITS; // turn off SDA ==1 (just in case)
        else   port_csp0_addr[X313_I2C_CMD]=X3X3_I2C_SDA_0_BITS; // turn on SDA ==0
    }
#else
    const x393_gpio_set_pins_t gpio_sda1_0 = {.d32 = SDA1_0};
    const x393_gpio_set_pins_t gpio_sda1_1 = {.d32 = SDA1_1};
    if (d) x393_gpio_set_pins(gpio_sda1_1);   // turn off SDA ==1
    else   x393_gpio_set_pins(gpio_sda1_0);   // turn on SDA ==0
#endif
}

// generate i2c start condition and test bus
int    i2c_start(int n) {
    int i;
    unsigned long flags;
    test_init_GPIO();
    local_irq_save(flags);
    //local_irq_disable();
    dev_dbg(g_dev_ptr, "i2c_start:  bus=%x\r\n", n);
    // both SCL and SDA are supposed to be high - no waiting is needed
    // set SCL=1, release SDA, wait SCL high time and verify.
    i2c_scl_1(n);
    i2c_sda_weak (n, 1);
    i2c_ldelay(bitdelays[n].scl_high*I2C_DELAY_SCALE);
    // verify SDA is high
    if (!i2c_getbit(n)) { // try recovering by clocking SCL (8x slower)
        for (i=0; i<9; i++) {
            i2c_scl_0(n);
            i2c_ldelay(bitdelays[n].scl_low * I2C_DELAY_SCALE*8);
            i2c_scl_1(n);
            i2c_ldelay(bitdelays[n].scl_high * I2C_DELAY_SCALE*8);
            if (!i2c_getbit(n)) break;
        }
        if (!i2c_getbit(n)) {
            local_irq_restore(flags);
            return ERR_I2C_SDA_ST0; // error
        }
    }
    i2c_sda_weak (n, 0);
    i2c_ldelay(bitdelays[n].scl_low*I2C_DELAY_SCALE); // technically it is SCL==1
    i2c_scl_0(n);
    local_irq_restore(flags);
    return 0;
}

// generate i2c stop condition
int i2c_stop(int n) {
    unsigned long flags;
    local_irq_save(flags);
    //local_irq_disable();
    dev_dbg(g_dev_ptr, "i2c_stop:  bus=%x\r\n", n);
    // SCL=0, SDA - unknown. Wait for bus turnover
    i2c_sda_weak (n, 0);
    i2c_ldelay(bitdelays[n].slave2master*I2C_DELAY_SCALE); // maybe not needed  as it is 1->0 transition
    i2c_ldelay(bitdelays[n].scl_low*I2C_DELAY_SCALE); // regular SCL=0 delay
    i2c_scl_1(n);
    i2c_ldelay(bitdelays[n].scl_high*I2C_DELAY_SCALE); // regular SCL=1 delay
    i2c_sda_strong (n, 1);
    i2c_ldelay(bitdelays[n].scl_high*I2C_DELAY_SCALE); // regular SCL=1 delay
    i2c_sda_weak (n, 1);
    local_irq_restore(flags);
    return 0;
}

// generate i2c repeated start condition
int    i2c_restart(int n) {
    unsigned long flags;
    local_irq_save(flags);
    //local_irq_disable();
    dev_dbg(g_dev_ptr, "i2c_restart:  bus=%x\r\n", n);
    // SCL=0, SDA - unknown. Wait for bus turnover
    i2c_sda_weak (n, 1);
    i2c_ldelay(bitdelays[n].slave2master*I2C_DELAY_SCALE); // time for slave to release the bus
    i2c_sda_strong (n, 1);
    i2c_ldelay(bitdelays[n].scl_low*I2C_DELAY_SCALE); // regular SCL=0 delay
    i2c_scl_1(n);
    i2c_sda_weak (n, 1);
    i2c_ldelay(bitdelays[n].scl_high*I2C_DELAY_SCALE); // regular SCL=1 delay
    i2c_sda_weak (n, 0);
    i2c_ldelay(bitdelays[n].scl_low*I2C_DELAY_SCALE); // technically it is SCL==1
    i2c_scl_0(n);
    local_irq_restore(flags);
    return 0;
}

// write a byte to the i2c interface , return acknowledge (active high, inverted from SDA line)
int i2c_outbyte(int n, unsigned char d) { 
    int i;
    unsigned char x=d;    // make it non-destructive
    unsigned long flags;
    local_irq_save(flags);
    //local_irq_disable();
    dev_dbg(g_dev_ptr, "i2c_outbyte:  bus=%x byte=%x\r\n", n, x);
    i2c_sda_weak (n, 1);
    i2c_ldelay(bitdelays[n].slave2master * I2C_DELAY_SCALE); // time for slave to release the bus
    for (i = 0; i < 8; i++) { // assumed to be with SCL=0;
        i2c_sda_strong (n,(x & 0x80));    // checks only on non-zero, so no need to '>>'
        i2c_ldelay(bitdelays[n].scl_low * I2C_DELAY_SCALE); // SCL=0 delay
        i2c_scl_1(n);
        i2c_sda_weak (n,(x & 0x80));    // checks only on non-zero, so no need to '>>'
        i2c_ldelay(bitdelays[n].scl_high * I2C_DELAY_SCALE); // regular SCL=1 delay
        i2c_scl_0(n);
        x <<= 1;
    }
    // prepare to read ACKN
    i2c_sda_weak (n, 1);
    // TODO: Need to wait here long (maybe only if last bit was 0? - (x & 0x100)!=0 )

    i2c_ldelay(bitdelays[n].master2slave * I2C_DELAY_SCALE); // master -> slave delay
    i2c_ldelay(bitdelays[n].scl_low * I2C_DELAY_SCALE); // regular SCL=0 delay
    i2c_scl_1(n);
    i2c_ldelay(bitdelays[n].scl_high * I2C_DELAY_SCALE); // regular SCL=1 delay
    i= (1-i2c_getbit(n));
    i2c_scl_0(n);
    dev_dbg(g_dev_ptr, "i2c_outbyte:  ACK=%x\r\n", i);
    local_irq_restore(flags);
    return i;
}


// read a byte from the i2c interface, send "more" bit (SDA=0 - more, SDA=1 - that's enough)

unsigned char i2c_inbyte(int n, int more) { // assumed SCL=0, SDA=X
    unsigned char aBitByte = 0;
    int i;
    unsigned long flags;
    local_irq_save(flags);
    //local_irq_disable();
    dev_dbg(g_dev_ptr, "i2c_inbyte:  bus=%x\r\n", n);
    // prepare to read ACKN
    i2c_sda_weak (n, 1);
    i2c_ldelay(bitdelays[n].master2slave * I2C_DELAY_SCALE); // master -> slave delay
    // read bits
    for (i = 0; i < 8; i++) {
        i2c_ldelay(bitdelays[n].scl_low * I2C_DELAY_SCALE); // regular SCL=0 delay
        i2c_scl_1(n);
        i2c_ldelay(bitdelays[n].scl_high * I2C_DELAY_SCALE); // regular SCL=1 delay
        aBitByte = (aBitByte << 1) | i2c_getbit(n);
        i2c_scl_0(n);
    }
    // send "more"
    i2c_sda_weak (n, more ? 0 : 1);
    i2c_ldelay(bitdelays[n].slave2master * I2C_DELAY_SCALE); // time for slave to release the bus
    i2c_sda_strong (n, more ? 0 : 1);
    i2c_ldelay(bitdelays[n].scl_low * I2C_DELAY_SCALE); // SCL=0 delay
    i2c_scl_1(n);
    i2c_sda_weak (n, more ? 0 : 1);
    i2c_ldelay(bitdelays[n].scl_high * I2C_DELAY_SCALE); // regular SCL=1 delay
    i2c_scl_0(n);
    //TODO: (test next is OK - 2012-01-15)
    i2c_sda_weak (n, 1); // release SDA byte
    dev_dbg(g_dev_ptr, "i2c_inbyte:  data=%x\r\n", aBitByte);
    local_irq_restore(flags);
    return aBitByte;    // returns with SCL=0, SDA - OFF
}


/*#---------------------------------------------------------------------------
 *#
 *# FUNCTION NAME: i2c_writeData
 *#
 *# DESCRIPTION  : Writes a sequence of bytes to an I2C device
 *# removed retries, will add test-ready later as a separate function
 *# removed all "dummy stuff" - I never needed that
 *# added "stop" argument - don't send stop before read
 *#--------------------------------------------------------------------------*/
int i2c_writeData(int n, unsigned char theSlave, unsigned char *theData, int size, int stop) {
    int i,error=0;
    dev_dbg(g_dev_ptr, "i2c_writeData:  bus=%x theSlave=%x data=%x %x size=%x\r\n", n, theSlave, theData[0], theData[1], size);

    // generate start condition, test bus
    if ((error=i2c_start(n))) return error;
    // send slave address, wait for ack
    if(!i2c_outbyte(n,theSlave)) {
        i2c_stop(n);
        return ERR_I2C_BSY;    // device does not exist or not ready
    }
    // OK, now send theData verifying ACK goes after each byte
    for (i=0;i<size;i++) {
        if(!i2c_outbyte(n,theData[i])) {
            i2c_stop(n);
            return ERR_I2C_NACK;    // device failure
        }
    }
    if (stop) i2c_stop(n);
    return 0;
}

/*#---------------------------------------------------------------------------
 *#
 *# FUNCTION NAME: i2c_readData
 *#
 *# DESCRIPTION  : Reads a data from the i2c device, returns 0- OK, else - error code
 *#
 *#--------------------------------------------------------------------------*/

int i2c_readData(int n, unsigned char theSlave, unsigned char *theData, int size, int start) {
    int i, error=0;
    if (start) {
        if ((error=i2c_start(n))) return error;
    } else {
        if ((error=i2c_restart(n))) return error;
    }
    /* send slave address, wait for ack */
    dev_dbg(g_dev_ptr, "i2c_readData:  bus=%x theSlave=%x size=%x start=%d\r\n", n, theSlave, size, start);

    if(!i2c_outbyte(n,theSlave)) {
        i2c_stop(n);
        return ERR_I2C_BSY;    // device does not exist or not ready
    }
    for (i=0;i<size;i++) {
        theData[i]=i2c_inbyte(n, (i<(size-1))); // last one should have no ackn !!!
    }
    i2c_stop(n);
    return 0;
}

/* Main device API. ioctl's to write or read to/from i2c registers.  */
// for now - single register read/write only
int i2c_ioctl(struct inode *inode, struct file *file,
        unsigned int cmd, unsigned long arg) {
    unsigned char data[3];
    int error=0;
    dev_dbg(g_dev_ptr, "i2c_ioctl cmd= %x, arg= %x\n\r",cmd,(int) arg);
    dev_dbg(g_dev_ptr, "i2c_ioctl:  ((int *)file->private_data)[0]= %x\n\r",((int *)file->private_data)[0]);
    //   dev_dbg(g_dev_ptr, "i2c_ioctl:  ((int )file->private_data)= %x\n\r",(int)file->private_data);
    if(_IOC_TYPE(cmd) != CMOSCAM_IOCTYPE) {
        return -EINVAL;
    }
    // #define I2C_DELAYS   0x0   /* read/write bit deleys for I2C */
    //int i2c_delays (unsigned long delays) {

    switch (_IOC_NR(cmd)) {
    case I2C_DELAYS:
        return i2c_delays (arg);
    case I2C_WRITEREG:
        /* write to an i2c slave */
        dev_dbg(g_dev_ptr, "i2cw bus=%d, slave=%d, reg=%d, value=%d\n",
                (int) I2C_ARGBUS(arg),
                (int) I2C_ARGSLAVE(arg),
                (int) I2C_ARGREG(arg),
                (int) I2C_ARGVALUE(arg));
        data[0]=I2C_ARGREG(arg);
        data[1]=I2C_ARGVALUE(arg);
        return -i2c_writeData(I2C_ARGBUS(arg), I2C_ARGSLAVE(arg) & 0xfe, &data[0], 2, 1); // send stop
    case I2C_READREG:
        /* read from an i2c slave */
        dev_dbg(g_dev_ptr, "i2cr bus=%d, slave=%d, reg=%d ",
                (int) I2C_ARGBUS(arg),
                (int) I2C_ARGSLAVE(arg),
                (int) I2C_ARGREG(arg));
        data[0]=I2C_ARGREG(arg);
        error=i2c_writeData(I2C_ARGBUS(arg), I2C_ARGSLAVE(arg) & 0xfe, &data[0], 1, 0); // no stop
        if (error) return -error;
        error=i2c_readData(I2C_ARGBUS(arg), I2C_ARGSLAVE(arg) | 0x01, &data[1], 1, 0);  // will start with restart, not start
        if (error) return -error;
        dev_dbg(g_dev_ptr, "returned %d\n", data[1]);
        return data[1];

    case I2C_16_WRITEREG:
        /* write to an i2c slave */
        dev_dbg(g_dev_ptr, "i2c16w slave=%d, reg=%d, value=%d\n",
                (int) I2C_16_ARGSLAVE(arg),
                (int) I2C_16_ARGREG(arg),
                (int) I2C_16_ARGVALUE(arg));
        data[0]=I2C_16_ARGREG(arg);
        data[1]=I2C_16_ARGVALUE_H(arg);
        data[2]=I2C_16_ARGVALUE_L(arg);
        return -i2c_writeData(0, I2C_16_ARGSLAVE(arg) & 0xfe, &data[0], 3, 1); // send stop
    case I2C_16_READREG:
        /* read from an i2c slave */
        dev_dbg(g_dev_ptr, "i2c16r slave=%d, reg=%d ",
                (int) I2C_16_ARGSLAVE(arg),
                (int) I2C_16_ARGREG(arg));
        data[0]=I2C_16_ARGREG(arg);
        error=i2c_writeData(0, I2C_16_ARGSLAVE(arg) & 0xfe, &data[0], 1, 0); //no stop
        if (error) return -error;
        error=i2c_readData(0, I2C_16_ARGSLAVE(arg) | 0x01, &data[1], 2, 0);
        if (error) return -error;
        dev_dbg(g_dev_ptr, "returned %d\n", (data[1]<<8)+data[2]);
        return (data[1]<<8)+data[2];
    default:
        return -EINVAL;
    }
    return 0;
}

/*
 * I2C as character devices
 *
 */



//#define X3X3_I2C 134
// minors (add more later - maybe different minors for different speed - set speed when opening)
//#define DEV393_MINOR(DEV393_I2C_CTRL)     0  // control/reset i2c
//#define DEV393_MINOR(DEV393_I2C_8_AINC)  1  // 8bit  registers, autoincement while read/write
//#define DEV393_MINOR(DEV393_I2C_16_AINC) 2  // 16bit registers, autoincement while read/write
#define    X3X3_I2C_DRIVER_NAME "Elphel (R) model 353 i2c character device driver"
//#define X3X3_I2C_CHANNELS 2
//#define X3X3_I2C_MAXMINOR 4  //

#define  I2CBUFSIZE 8196
//static unsigned char i2cbuf[256*256+1]; //actually usually much less
static unsigned char i2c_enable[X3X3_I2C_CHANNELS*128]; //128 devices in a bus
//static unsigned char i2cbuf_all[( X3X3_I2C_CHANNELS + 1 ) * I2CBUFSIZE];
static unsigned char i2cbuf_all[ X3X3_I2C_CHANNELS + 1  * I2CBUFSIZE]; //! no buffers for control files
static loff_t sizes[X3X3_I2C_MAXMINOR + 1];
static int burst_sizes[X3X3_I2C_MAXMINOR + 1]; //!fix for PHP read (always 8192)
static loff_t inuse[X3X3_I2C_CHANNELS];

//static int thisminor =0;
//static int thissize  =0;
static int     xi2c_open   (struct inode *inode, struct file *filp);
static int     xi2c_release(struct inode *inode, struct file *filp);
static loff_t  xi2c_lseek  (struct file * file, loff_t offset, int orig);
static ssize_t xi2c_write  (struct file * file, const char * buf, size_t count, loff_t *off);
static ssize_t xi2c_read   (struct file * file, char * buf, size_t count, loff_t *off);
static int      xi2c_init  (struct platform_device *pdev);

static struct file_operations xi2c_fops = {
        owner:    THIS_MODULE,
        open:     xi2c_open,
        release:  xi2c_release,
        read:     xi2c_read,
        write:    xi2c_write,
        llseek:   xi2c_lseek
};

//!++++++++++++++++++++++++++++++++++++ open() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

int     xi2c_open(struct inode *inode, struct file *filp) {
    int p = MINOR(inode->i_rdev);
    int bus=-1;
    int * pd= (int *) &(filp->private_data);
    switch (p) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        bus=0;
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        bus=1;
        break;
    case DEV393_MINOR(DEV393_I2C_ENABLE):
    case DEV393_MINOR(DEV393_I2C_CTRL) :
        bus=-1;
        break;
    }

    dev_dbg(g_dev_ptr, "xi2c_open, minor=%d\n",p);
    if ((bus>=0) && (inuse[bus] !=0)) return -EACCES;
    dev_dbg(g_dev_ptr, "xi2c_open, minor=%d\n",p);
    inode->i_size=sizes[p];
    if (bus>=0) inuse[bus] =1;

    switch ( p ) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        inode->i_size=128*256;
        burst_sizes[p]=1; //!fix for PHP read (always 8192) -> 1 byte/read
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        inode->i_size=256*256;
        burst_sizes[p]=2; //!fix for PHP read (always 8192) -> 2 bytes/read
        break;
    case DEV393_MINOR(DEV393_I2C_CTRL) :
        inode->i_size=sizeof(bitdelays);
        break;
    case DEV393_MINOR(DEV393_I2C_ENABLE):
        inode->i_size=sizeof(i2c_enable);
        break;
    default:return -EINVAL;
    }
    //   thisminor=p;
    //   thissize=inode->i_size;
    //   filp->private_data = &thisminor;
    //   filp->private_data=p; // just a minor number
    pd[0]=p; // just a minor number
    return 0;
}

//!++++++++++++++++++++++++++++++++++++ release() ++++++++++++++++++++++++++++++++++++++++++++++++++++++
//static loff_t sizes[X3X3_I2C_MAXMINOR + 1];
//static loff_t inuse[X3X3_I2C_CHANNELS];

static int     xi2c_release(struct inode *inode, struct file *filp){
    int p = MINOR(inode->i_rdev);
    int bus=-1;
    switch (p) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        bus=0;
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        bus=1;
        break;
    case DEV393_MINOR(DEV393_I2C_ENABLE):
    case DEV393_MINOR(DEV393_I2C_CTRL) :
        bus=-1;
        break;
    }
    dev_dbg(g_dev_ptr, "xi2c_release, minor=%d\n",p);
    if (bus>=0) inuse[bus]=0;
    else if (p==DEV393_MINOR(DEV393_I2C_CTRL)) for (bus=0; bus < X3X3_I2C_CHANNELS; bus++) inuse[bus]=0;
    //  thisminor =0;
    return 0;
}

//!++++++++++++++++++++++++++++++++++++ lseek() ++++++++++++++++++++++++++++++++++++++++++++++++++++++
static loff_t  xi2c_lseek(struct file * file, loff_t offset, int orig) {
    /*
     *  orig 0: position from begning of eeprom
     *  orig 1: relative from current position
     *  orig 2: position from last address
     */
    int p=(int)file->private_data;
    int thissize=sizes[p];
    switch (orig) {
    case SEEK_SET:
        file->f_pos = offset;
        break;
    case SEEK_CUR:
        file->f_pos += offset;
        break;
    case SEEK_END:

        //       burst_sizes[p]=2; //!fix for PHP read (always 8192) -> 2 bytes/read

        //!overload
        if (offset<=0) {
            file->f_pos = thissize + offset;
        } else {
            burst_sizes[p]=offset; //!Number of bytes to read in a single (read) if 8192 is passed (odd for 16-bits will be fixed during read itself
        }
        break;
    default:
        return -EINVAL;
    }
    //   switch (((int *)file->private_data)[0]) {
    switch (p) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        (file->f_pos) &= ~(0x7f); // zero out 7 MSBs
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) : {
        if ((file->f_pos) & 1) (file->f_pos)++; // increment to the next (if odd) for 16-bit accesses
        break;
    }
    }

    /* truncate position */
    if (file->f_pos < 0) {
        file->f_pos = 0;
        return (-EOVERFLOW);
    }

    if (file->f_pos > thissize) {
        file->f_pos = thissize;
        return (-EOVERFLOW);
    }
    return (file->f_pos);
}

//!++++++++++++++++++++++++++++++++++++ read() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

ssize_t xi2c_read(struct file * file, char * buf, size_t count, loff_t *off) {
//    int hardware_i2c_running;
//    unsigned long flags;
    unsigned long p;
    int error;
    char * bbitdelays= (char*) bitdelays;
    p = *off;
    int bus=0;
    unsigned char * i2cbuf=&i2cbuf_all[0]; // initialize to keep compiler happy
    unsigned char * userbuf=&i2cbuf[1];
    int thissize=sizes[(int)file->private_data];
    int slave_adr;
    int en_mask=0;
    int en_bits=0;

    //  switch (((int *)file->private_data)[0]) {
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        bus=0;
        i2cbuf = &i2cbuf_all[0];
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        bus=1;
        i2cbuf = &i2cbuf_all[1*I2CBUFSIZE];
        break;
        //     case DEV393_MINOR(DEV393_I2C_CTRL) :
        //       i2cbuf = &i2cbuf_all[X3X3_I2C_CHANNELS*I2CBUFSIZE];
    }
    userbuf=&i2cbuf[1];

    //  switch (((int *)file->private_data)[0]) {
    slave_adr=(p >> 7) & 0xfe;
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        if (count == 8192) count=burst_sizes[(int)file->private_data]; //! PHP "feature" fix
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        p&=0xfffffffe;
        if (count == 8192) count=burst_sizes[(int)file->private_data]; //! PHP "feature" fix
        if (count & 1) count++;
        slave_adr=(p >> 8) & 0xfe;
        break;
    }
    dev_dbg(g_dev_ptr, "xi2c_read (bus=%d) from 0x%x, count=%d\n", bus, (int) *off, (int) count);
    //! Verify if this slave is enabled for the I2C operation requested
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        en_mask=(1 << X3X3_I2C_ENABLE_RD) | (1 <<X3X3_I2C_ENABLE_RAW);
        break;
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        en_mask=(1 << X3X3_I2C_ENABLE_RD) | (1 <<X3X3_I2C_ENABLE_8);
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        en_mask=(1 << X3X3_I2C_ENABLE_RD) | (1 <<X3X3_I2C_ENABLE_16);
        break;
    }
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        en_bits=i2c_enable[ slave_adr>>1 ];
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        en_bits=i2c_enable[(slave_adr>>1) + 128];
        break;
    }
    if ((en_bits & en_mask) ^ en_mask) {
        printk("tried disabled xi2c_read (bus=%d, slave=0x%x)\n", bus, slave_adr);
        dev_dbg(g_dev_ptr, "en_bits=0x%x, en_mask=0x%x (minor=%d)\n", (int) en_bits, (int) en_mask, (int)file->private_data);
        return -ENXIO;
    }

    if (p >= thissize)  return -EINVAL; // bigger than
    if( (p + count) > thissize) { // truncate count
        count = thissize - p;
    }
    if( count > (I2CBUFSIZE-1)) { // should not happen i2cbuf is larger than maximal thissize
        count = I2CBUFSIZE-1;
    }
    if (count==0) return 0;
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        i2cbuf[0]=p & 0xff;
        error=i2c_readData (bus, slave_adr | 0x01, &i2cbuf[1], count, 1);  // will start with start, not restart
        if (error) return -EINVAL;
        break;
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        i2cbuf[0]=p & 0xff;

        if (bus==0) {
#ifdef NC353
            local_irq_save(flags); /// IRQ Off
            hardware_i2c_running=i2s_running();
            if (hardware_i2c_running) i2c_stop_wait();
            error=              i2c_writeData(bus,  slave_adr,         &i2cbuf[0], 1, 0); // no stop
            if (error==0) error=i2c_readData (bus, slave_adr | 0x01, &i2cbuf[1], count,0);  // will start with restart, not start
            if (hardware_i2c_running) i2c_run(); /// turn hardware i2c back on
            local_irq_restore(flags); /// IRQ restore
#endif
        } else {
            error=              i2c_writeData(bus,  slave_adr,         &i2cbuf[0], 1, 0); // no stop
            if (error==0) error=i2c_readData (bus, slave_adr | 0x01, &i2cbuf[1], count,0);  // will start with restart, not start
        }

        if (error) return -EINVAL;
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        i2cbuf[0]=(p>>1) & 0xff;

        if (bus==0) {
#ifdef NC353
            local_irq_save(flags); /// IRQ Off
            hardware_i2c_running=i2s_running();
            if (hardware_i2c_running) i2c_stop_wait();
            error=              i2c_writeData(bus,  slave_adr,         &i2cbuf[0], 1, 0); // no stop
            if (error==0) error=i2c_readData (bus, slave_adr | 0x01, &i2cbuf[1], count,0);  // will start with restart, not start
            if (hardware_i2c_running) i2c_run(); /// turn hardware i2c back on
            local_irq_restore(flags); /// IRQ restore
#endif
        } else {
            error=              i2c_writeData(bus,  slave_adr,         &i2cbuf[0], 1, 0); // no stop
            if (error==0) error=i2c_readData (bus, slave_adr | 0x01, &i2cbuf[1], count,0);  // will start with restart, not start
        }

        if (error) return -EINVAL;
        break;
    case DEV393_MINOR(DEV393_I2C_CTRL) :
        userbuf=&bbitdelays[*off];
        //       memcpy(&i2cbuf[1],&bbitdelays[*off],count);
        break;
    case DEV393_MINOR(DEV393_I2C_ENABLE):
        userbuf=&i2c_enable[*off];
        break;
    default:return -EINVAL;
    }

    //  if (copy_to_user(buf,  &i2cbuf[1], count)) return -EFAULT;
    if (copy_to_user(buf,  userbuf, count)) return -EFAULT;

    //! do not increment pointer for raw accesses
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        *off &= ~(0x7f); // zero out 7 MSBs
        break;
    default:
        *off+=count;
    }
    dev_dbg(g_dev_ptr, "count= 0x%x, pos= 0x%x\n", (int) count, (int)*off);
    return count;
}

//!++++++++++++++++++++++++++++++++++++ write() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ssize_t xi2c_write(struct file * file, const char * buf, size_t count, loff_t *off) {
//   int hardware_i2c_running;
//    unsigned long flags;
    unsigned long p;
    int error;
    int bus=0;
    char * bbitdelays= (char*) bitdelays;
    unsigned char * i2cbuf=&i2cbuf_all[0]; // initialize to keep compiler happy
    unsigned char * userbuf=&i2cbuf[1];
    int thissize=sizes[(int)file->private_data];
    int slave_adr;
    int en_mask=0;
    int en_bits=0;
    p = *off;

    //  switch (((int *)file->private_data)[0]) {
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        bus=0;
        i2cbuf = &i2cbuf_all[0];
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        bus=1;
        i2cbuf = &i2cbuf_all[1*I2CBUFSIZE];
        break;
        //     case DEV393_MINOR(DEV393_I2C_CTRL) :
        //       i2cbuf = &i2cbuf_all[X3X3_I2C_CHANNELS*I2CBUFSIZE];
    }
    userbuf=&i2cbuf[1];
    slave_adr=(p >> 7) & 0xfe;
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        p&=0xfffffffe;
        if (count & 1) count++;
        slave_adr=(p >> 8) & 0xfe;
        break;
    }
    dev_dbg(g_dev_ptr, "xi2c_write (bus=%d) to 0x%x, count=%x\n", bus, (int) *off, (int) count);

    //! Verify if this slave is enabled for the I2C operation requested
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        en_mask=(1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW);
        break;
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        en_mask=(1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8);
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        en_mask=(1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16);
        break;
    }
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
        en_bits=i2c_enable[ slave_adr>>1 ];
        break;
    case DEV393_MINOR(DEV393_I2C1_RAW):
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        en_bits=i2c_enable[(slave_adr>>1) + 128];
        break;
    }
    if ((en_bits & en_mask) ^ en_mask) {
        printk("tried disabed xi2c_write (bus=%d, slave=0x%x)\n", bus, slave_adr);
        dev_dbg(g_dev_ptr, "en_bits=0x%x, en_mask=0x%x (minor=%d)\n", (int) en_bits, (int) en_mask, (int)file->private_data);
        return -ENXIO;
    }

    if (p >= thissize)  return -EINVAL;
    if( (p + count) > thissize) { // truncate count
        count = thissize - p;
    }
    if( count > (I2CBUFSIZE-1)) { // should not happen i2cbuf is larger than maximal thissize
        count = I2CBUFSIZE-1;
    }
    if (count==0) return 0;
    //!only for control files that write directly to static arrays, no buffering
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_CTRL) :
        userbuf=&bbitdelays[p];
        break;
    case DEV393_MINOR(DEV393_I2C_ENABLE):
        userbuf=&i2c_enable[p];
        break;
    }

    //  if (copy_from_user( &i2cbuf[1], buf, count)) return -EFAULT;
    if (copy_from_user( userbuf, buf, count)) return -EFAULT;
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):

        if (bus==0) {
            error = -EINVAL;
#ifdef NC353
            local_irq_save(flags); /// IRQ Off
            hardware_i2c_running=i2s_running();
            if (hardware_i2c_running) i2c_stop_wait();
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[1],count, 1); // send stop
            if (hardware_i2c_running) i2c_run(); /// turn hardware i2c back on
            local_irq_restore(flags); /// IRQ restore
#endif
        } else {
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[1],count, 1); // send stop
        }

        if (error) return -EINVAL;
        break;
    case DEV393_MINOR(DEV393_I2C_8_AINC) :
    case DEV393_MINOR(DEV393_I2C1_8_AINC) :
        i2cbuf[0]=p & 0xff;

        if (bus==0) {
            error = -EINVAL;
#ifdef NC353
            local_irq_save(flags); /// IRQ Off
            hardware_i2c_running=i2s_running();
            if (hardware_i2c_running) i2c_stop_wait();
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[0],count+1, 1); // send stop
            if (hardware_i2c_running) i2c_run(); /// turn hardware i2c back on
            local_irq_restore(flags); /// IRQ restore
#endif
        } else {
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[0],count+1, 1); // send stop
        }

        if (error) return -EINVAL;
        break;
    case DEV393_MINOR(DEV393_I2C_16_AINC) :
    case DEV393_MINOR(DEV393_I2C1_16_AINC) :
        i2cbuf[0]=(p>>1) & 0xff;

        if (bus==0) {
            error = -EINVAL;
#ifdef NC353
            local_irq_save(flags); /// IRQ Off
            hardware_i2c_running=i2s_running();
            if (hardware_i2c_running) i2c_stop_wait();
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[0],count+1, 1); // send stop
            if (hardware_i2c_running) i2c_run(); /// turn hardware i2c back on
            local_irq_restore(flags); /// IRQ restore
#endif
        } else {
            error=i2c_writeData(bus, slave_adr,  &i2cbuf[0],count+1, 1); // send stop
        }

        if (error) return -EINVAL;
        break;
        //     case DEV393_MINOR(DEV393_I2C_CTRL) :
        //       memcpy(&bbitdelays[*off],&i2cbuf[1],count);
        //       break;
    default:return -EINVAL;
    }
    //  *off+=count;
    //! do not increment pointer for raw accesses
    switch ((int)file->private_data) {
    case DEV393_MINOR(DEV393_I2C_RAW):
    case DEV393_MINOR(DEV393_I2C1_RAW):
        *off &= ~(0x7f); // zero out 7 MSBs
        break;
    default:
        *off+=count;
    }
    dev_dbg(g_dev_ptr, "count= 0x%x, pos= 0x%x\n", (int) count, (int)*off);
    return count;
}
/** Test if GPIO pins are not initialized, and do it if not (can nolt initialize at driver init as bitstream has to be loaded) */
static void test_init_GPIO(void)
{
    x393_gpio_set_pins_t gpio_set_pins = {.d32 = 0};
    if (xi2c_initialized) return;
    gpio_set_pins.soft = 3; // Enable software control of GPIO pins
    x393_gpio_set_pins  (gpio_set_pins);
    xi2c_initialized=1;

}

// TODO: Add sysfs interface here
#define SYSFS_PERMISSIONS           0644 /* default permissions for sysfs files */
#define SYSFS_READONLY              0444
#define SYSFS_WRITEONLY             0222

static ssize_t show_scl_high(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].scl_high);
}

static ssize_t store_scl_high(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].scl_high = (unsigned char) d;
    return count;
}
static ssize_t show_scl_low(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].scl_low);
}

static ssize_t store_scl_low(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].scl_low = (unsigned char) d;
    return count;
}

static ssize_t show_slave2master(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].slave2master);
}

static ssize_t store_slave2master(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].slave2master = (unsigned char) d;
    return count;
}

static ssize_t show_master2slave(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].master2slave);
}

static ssize_t store_master2slave(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].master2slave = (unsigned char) d;
    return count;
}

static ssize_t show_filter_sda(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].filter_sda);
}

static ssize_t store_filter_sda(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].filter_sda = (unsigned char) d;
    return count;
}

static ssize_t show_filter_scl(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", bitdelays[1].filter_scl);
}

static ssize_t store_filter_scl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int d;
    if (!sscanf(buf, "%u", &d)) {
        return - EINVAL;
    }
    bitdelays[1].filter_scl = (unsigned char) d;
    return count;
}

static DEVICE_ATTR(time_scl_high,              SYSFS_PERMISSIONS,     show_scl_high,              store_scl_high);
static DEVICE_ATTR(time_scl_low,               SYSFS_PERMISSIONS,     show_scl_low,               store_scl_low);
static DEVICE_ATTR(time_slave2master,          SYSFS_PERMISSIONS,     show_slave2master,          store_slave2master);
static DEVICE_ATTR(time_master2slave,          SYSFS_PERMISSIONS,     show_master2slave,          store_master2slave);
static DEVICE_ATTR(filter_sda,                 SYSFS_PERMISSIONS,     show_filter_sda,            store_filter_sda);
static DEVICE_ATTR(filter_scl,                 SYSFS_PERMISSIONS,     show_filter_scl,            store_filter_scl);





static struct attribute *root_dev_attrs[] = {
        &dev_attr_time_scl_high.attr,
        &dev_attr_time_scl_low.attr,
        &dev_attr_time_slave2master.attr,
        &dev_attr_time_master2slave.attr,
        &dev_attr_filter_sda.attr,
        &dev_attr_filter_scl.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_ext_i2c_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

// Add device tree support (known devices?)
static const struct of_device_id elphel393_ext_i2c_of_match[] = {
    { .compatible = "elphel,elphel393-ext-i2c-1.00" },
    { /* end of list */ }
};

static int elphel393_xi2c_init_of(struct platform_device *pdev) ///< Platform device structure for this driver
{
    const struct of_device_id *match;
    struct device_node *node;
    struct device *dev = &pdev->dev;

    match = of_match_device(elphel393_ext_i2c_of_match, dev);
    if (!match)
        return -EINVAL;

    node = of_find_node_by_name(NULL, "elphel393-ext-i2c");
    if (!node)
    {
        pr_err("elphel393-ext-i2c: No device tree node found\n");
        return -ENODEV;
    }
    bitdelays[1].scl_high =      be32_to_cpup((__be32 *)of_get_property(node,  "time_scl_high",     NULL));
    bitdelays[1].scl_low =       be32_to_cpup((__be32 *)of_get_property(node,  "time_scl_low",      NULL));
    bitdelays[1].slave2master =  be32_to_cpup((__be32 *)of_get_property(node,  "time_slave2master", NULL));
    bitdelays[1].master2slave =  be32_to_cpup((__be32 *)of_get_property(node,  "time_master2slave", NULL));
    bitdelays[1].filter_sda =    be32_to_cpup((__be32 *)of_get_property(node,  "filter_sda",        NULL));
    bitdelays[1].filter_scl =    be32_to_cpup((__be32 *)of_get_property(node,  "filter_scl",        NULL));

    return 0;
}

int xi2c_init(struct platform_device *pdev)
{
    int i,res;
    struct device *dev = &pdev->dev;
    // char device for sensor port
    struct device *chrdev;

    elphel393_ext_i2c_sysfs_register(pdev);
    dev_info(dev, DEV393_NAME(DEV393_I2C_CTRL)": registered sysfs\n");
    g_dev_ptr = dev;

    res = register_chrdev(DEV393_MAJOR(DEV393_I2C_CTRL), DEV393_NAME(DEV393_I2C_CTRL), &xi2c_fops);
    if(res < 0) {
        printk(KERN_ERR "\nxi2c_init: couldn't get a major number %d.\n",DEV393_MAJOR(DEV393_I2C_CTRL));
        return res;
    }
    printk(X3X3_I2C_DRIVER_NAME" - %d, %d channels\n",DEV393_MAJOR(DEV393_I2C_CTRL),X3X3_I2C_CHANNELS);
    //   thisminor =0;

	// create device class
	xi2c_dev_class = class_create(THIS_MODULE, DEV393_NAME(DEV393_I2C_CTRL));
	if (IS_ERR(xi2c_dev_class)) {
		pr_err("Cannot create \"%s\" class", DEV393_NAME(DEV393_I2C_CTRL));
		return PTR_ERR(xi2c_dev_class);
	}

	//create devices
	for (i=0;i<(sizeof(xi2c_minor)/sizeof(int));i++){
		chrdev = device_create(
				  xi2c_dev_class,
				  &pdev->dev,
				  MKDEV(xi2c_major, xi2c_minor[i]),
				  NULL,
				  "%s",xi2c_devs[i]);
		if(IS_ERR(chrdev)){
			pr_err("Failed to create a device (xi2c %d). Error code: %ld\n",i,PTR_ERR(chrdev));
		}
	}

    elphel393_xi2c_init_of(pdev);

    bitdelays[0].scl_high=2;      //! SCL high:
    bitdelays[0].scl_low=2;       //! SCL low:
    bitdelays[0].slave2master=1;  //! slave -> master
    bitdelays[0].master2slave=1;  //! master -> slave
    bitdelays[0].filter_sda=0x07; //! filter SDA read data by testing multiple times - currently just zero/non zero
    bitdelays[0].filter_scl=0x07; //! filter SCL read data by testing multiple times - currently just zero/non zero
    //! bus 1 - increased by 1 measured for EEPROM
    bitdelays[1].scl_high=2;      //! SCL high: (was 3)
    bitdelays[1].scl_low=2;       //! SCL low: with 2 -
    bitdelays[1].slave2master=1;  //! slave -> master
    bitdelays[1].master2slave=1;  //! master -> slave
    bitdelays[1].filter_sda=0x0;  //! filter SDA read data by testing multiple times - currently just zero/non zero
    bitdelays[1].filter_scl=0x0;  //! filter SCL read data by testing multiple times - currently just zero/non zero

    elphel393_xi2c_init_of(pdev); // may return negative errors

    for (i=0; i<X3X3_I2C_CHANNELS;i++) {
        inuse[i]=0;
    }
    sizes[DEV393_MINOR(DEV393_I2C_CTRL)]=     sizeof(bitdelays); // control/reset i2c
    sizes[DEV393_MINOR(DEV393_I2C_8_AINC)]=   128*256;           // 8bit  registers, autoincement while read/write
    sizes[DEV393_MINOR(DEV393_I2C_16_AINC)]=  256*256;           // 16bit registers, autoincement while read/write
    sizes[DEV393_MINOR(DEV393_I2C1_8_AINC)]=  128*256;           // 8bit  registers, autoincement while read/write (bus 1)
    sizes[DEV393_MINOR(DEV393_I2C1_16_AINC)]= 256*256;           // 16bit registers, autoincement while read/write (bus 1)
    sizes[DEV393_MINOR(DEV393_I2C_RAW)]=      128*256;           // 8bit single register,
    sizes[DEV393_MINOR(DEV393_I2C1_RAW)]=     128*256;           // 8bit single register,
    sizes[DEV393_MINOR(DEV393_I2C_ENABLE)]=   sizeof(i2c_enable); // enable particular types of accesses for I2C devices
    //! Enable/protect known I2C devices. For now - unprotect all unknown
    for (i=0; i<X3X3_I2C_CHANNELS*128; i++) i2c_enable[i]=0xff;
    //! now protect known devices from undesired/unintended accesses:
    //! bus0:
    i2c_enable[0x90 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! micron 1.3-3.0MPix
    i2c_enable[0xba >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! micron 5.0MPix
    i2c_enable[0x20 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! 10347 board (model 363)

    i2c_enable[0xa4 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[0xa6 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[0xa8 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[0xaa >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[0xac >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[0xae >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)

    i2c_enable[0x18 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[0x1a >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[0x1c >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[0x1e >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625

    i2c_enable[0x44 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[0x46 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[0x48 >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[0x4a >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[0x4c >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[0x4e >> 1]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw

    //!bus1:
    i2c_enable[128+(0x90 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! SA56004A (temperature)
    i2c_enable[128+(0xa2 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCF8563 (clock)
    i2c_enable[128+(0xa0 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0x40 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw

    //!granddaughter PCA9500
    i2c_enable[128+(0xa4 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0xa6 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0xa8 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0xaa >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0xac >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)
    i2c_enable[128+(0xae >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 0 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_8); //! PCA9500 (eeprom, write protect)

    i2c_enable[128+(0x18 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[128+(0x1a >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[128+(0x1c >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625
    i2c_enable[128+(0x1e >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_16); //! AD5625

    //!raw disabling does not work?
    i2c_enable[128+(0x44 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[128+(0x46 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[128+(0x48 >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[128+(0x4a >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[128+(0x4c >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw
    i2c_enable[128+(0x4e >> 1)]= (1 << X3X3_I2C_ENABLE_RD) | ( 1 << X3X3_I2C_ENABLE_WR) | (1 <<X3X3_I2C_ENABLE_RAW); //! PCA9500 I/O , raw


    //#define X3X3_I2C_RAW        5  // 8bit  registers, no address byte (just slave, then read/write byte(s)
    //#define DEV393_MINOR(DEV393_I2C1_RAW)       6  // 8bit  registers, no address byte (just slave, then read/write byte(s)
    //#define DEV393_MINOR(DEV393_I2C_ENABLE)     7  // enable(/protect) different I2C devices for different types of I2C accesses
    //static unsigned char i2c_enable[X3X3_I2C_CHANNELS*128]; //128 devices in a bus
    //#define X3X3_I2C_ENABLE_RD   0 // bit 0 - enable i2c read
    //#define X3X3_I2C_ENABLE_WR   1 // bit 1 - enable i2c write
    //#define X3X3_I2C_ENABLE_RAW  2 // bit 2 - enable i2c raw (no address byte)
    //#define X3X3_I2C_ENABLE_8    3 // bit 3 - enable i2c 8-bit registers access
    //#define X3X3_I2C_ENABLE_16   4 // bit 4 - enable i2c 16-bit registers access
    xi2c_initialized=0; // configure GPIO puins access at first command;
    return 0;
}

//-------------------------------

int xi2c_remove(struct platform_device *pdev)
{
	int i;
	for (i=0;i<(sizeof(xi2c_minor)/sizeof(int));i++){
		device_destroy(
			xi2c_dev_class,
			MKDEV(xi2c_major, xi2c_minor[i]));
	}
    unregister_chrdev(DEV393_MAJOR(DEV393_I2C_CTRL), DEV393_NAME(DEV393_I2C_CTRL));
    return 0;
}

MODULE_DEVICE_TABLE(of, elphel393_ext_i2c_of_match);

static struct platform_driver elphel393_ext_i2c = {
    .probe          = xi2c_init,
    .remove         = xi2c_remove,
    .driver         = {
        .name       = DEV393_NAME(DEV393_I2C_CTRL),
        .of_match_table = elphel393_ext_i2c_of_match,
    },
};

module_platform_driver(elphel393_ext_i2c);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_I2C_DRIVER_NAME);


