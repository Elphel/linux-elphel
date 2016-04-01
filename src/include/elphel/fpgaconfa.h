/*
os/linux-2.6/include/asm-cris/elphel/fpgaconfa.h
 */

#ifndef _ASM_FPGACONF_H
#define _ASM_FPGACONF_H
// most defines are from cmoscama.h ...
/* _IOC_TYPE, bits 8 to 15 in ioctl cmd */

#define FPGACONF_IOCTYPE     129

#define FPGACONF_READREG      140    // read 32-bit register  (through CSP0)
#define FPGACONF_WRITEREG     141    // write 32-bit register (through CSP0)
#define FPGACONF_READREG_L    142    // read lower 16 bits    (through CSP0)
#define FPGACONF_READREG_H    143    // read upper 16 bits    (through CSP0)
#define FPGACONF_READREG4     144    // read 32-bit register  (through CSP4)
#define FPGACONF_WRITEREG4    145    // write 32-bit register (through CSP4)
#define FPGACONF_READREG_L4   146    // read lower 16 bits    (through CSP4)
#define FPGACONF_READREG_H4   147    // read upper 16 bits    (through CSP4)

#define FPGACONF_GETSTATE     148    // read FPGA/clock state  (minor FPGACONF_MINOR_IORW)

#define FPGACONF_RD_WAITSTATES     150    // read R_WAITSTATES
#define FPGACONF_WR_WAITSTATES     151    // write R_WAITSTATES

#define FPGACONF_START_CAPTURE     152    // start capturing I/O pins to memory (IRQ off!). argument time 1=1ms (actually - longer)
#define FPGACONF_READ_CAPTURE      153    // read captured I/O pins. 1-st time (after FPGACONF_START_CAPTURE) - length, after that - data 

#define FPGACONF_CANON_IOBYTE      154    // write/read byte to CANON lens interface

#define FPGACONF_EXT_BOARD_PRESENT  156    	// return status of the IO extension board pin

#define FPGA_STATE_LOADED       0x0000FFFF //
#define FPGA_STATE_CLOCKS       0x000F0000 // 
#define FPGA_STATE_INITIALIZED  0x00F00000 // 
#define FPGA_STATE_SDRAM_INIT   0x00100000 // 

#define FPGACONF_CONTROL_REG      155    // modify FPGA control register (0x10)

#define FPGACONF_CR_MODIFY  0x0e	//(bit number)<<2 | op; op= 0 - nop, 1 - set, 2 - reset, 3 - toggle)
#define FPGACONF_CR_SHADOW  0x0f
#define FPGACONF_CR_SHADOW1 0x10



/* supported ioctl _IOC_NR's */
#ifndef I2C_WRITEARG
 #define I2C_WRITEARG(bus, slave, reg, value) (((bus) << 24) | ((slave) << 16) | ((reg) << 8) | (value))
 #define I2C_READARG(bus, slave, reg) (((bus) << 24) | ((slave) << 16) | ((reg) << 8))

 #define I2C_ARGBUS(arg) (((arg) >> 24)  & 0x1)
 #define I2C_ARGSLAVE(arg) (((arg) >> 16)  & 0xff)
 #define I2C_ARGREG(arg) (((arg) >> 8) & 0xff)
 #define I2C_ARGVALUE(arg) ((arg) & 0xff)

 #define I2C_WRITEREG 0x1   /* write to an i2c register */
 #define I2C_READREG  0x2   /* read from an i2c register */
#endif






#define FPGA_PGM     0x3   /* set FPGA pgm pin */
#define FPGA_STAT    0x4   /* read FPGA pins status (bit 0 - INIT, bit 1 - DOME) */
#define FPGA_JTAG    0x5   /* shift byte to FPGA JTAG */
#define FPGA_PA_RD   0x6   /* write data to port A and shadow */
#define FPGA_PA_WR   0x7   /* write data to port A and shadow */
#ifndef PGA_JTAG_ARG
  #define FPGA_JTAG_ARG(tms, len, d) (((tms) << 11) | ((len) << 8) | ((d) & 0xff))
  #define FPGA_JTAG_TMS(arg) ((arg >> 11) &    1)
  #define FPGA_JTAG_LEN(arg) ((arg >> 8)  &    7)
  #define FPGA_JTAG_DW(arg)  ( arg        & 0xff)
#endif

#define _FCCMD(x,y)  (_IO(FPGACONF_IOCTYPE, (x << 6) | (y & 0x3f)))


/* i2c errors */
#ifndef ERR_I2C_SCL_ST0
 #define	ERR_I2C_SCL_ST0		 1
 #define	ERR_I2C_SDA_ST0		 2
 #define	ERR_I2C_SCL_ST1		 4
 #define	ERR_I2C_SDA_ST1		 8
 #define	ERR_I2C_SCL_NOPULLUP 16
 #define	ERR_I2C_SDA_NOPULLUP 32
/* i2c_diagnose called by i2c_start (?) could not find any problems. Try again start */
 #define ERR_I2C_NOTDETECTED  64
 #define	ERR_I2C_SHORT		 128
 #define	ERR_I2C_BSY		 256
 #define	ERR_I2C_NACK		 512
#endif

/* direct register r/w*/
#define IO_CSP0R0    0x10  /* read and return CSP0+0  port data */
#define IO_CSP0W0    0x20  /* write data to port CSP0+0  */

#define IO_CSP0R(a) (IO_CSP0R0 + a)
#define IO_CSP0W(a) (IO_CSP0W0 + a)

#define IO_CSP0_R 1
#define IO_CSP0_W 2

#endif

