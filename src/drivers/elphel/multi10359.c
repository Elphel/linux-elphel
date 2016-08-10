/***************************************************************************//**
* @file      multi10359.c
* @brief     Control of the 10359 multiplexer board
* @copyright Copyright 2010-2016 (C) Elphel, Inc.
* @par <b>License</b>
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/

/*
*!   $Log: multisensor.c,v $
*!  Revision 1.19  2012/01/16 01:47:01  elphel
*!  more debug output (if enabled)
*!
*!  Revision 1.18  2010/12/15 16:45:53  elphel
*!  width in multisensor mopde
*!
*!  Revision 1.17  2010/11/08 23:41:41  elphel
*!  8.0.9.1 modifying HACT_WIDTH handling with 10359 board
*!
*!  Revision 1.16  2010/08/17 23:49:14  elphel
*!  improved sdram phase adjustment, removed obsolete register, additional debug output.
*!
*!  Revision 1.15  2010/08/12 20:03:46  elphel
*!  resolved race between individual and multi_ flips at startup
*!
*!  Revision 1.14  2010/08/12 06:02:06  elphel
*!  Fixed SDRAM phase adjust (was not critical as phase was still set in the acceptable range
*!
*!  Revision 1.13  2010/08/02 01:27:58  elphel
*!  internal HACT mode in 10359 is on by default now
*!
*!  Revision 1.12  2010/07/20 20:13:34  elphel
*!  8.0.8.33 - added MakerNote info for composite images made with multisensor cameras (with 10359 board)
*!
*!  Revision 1.11  2010/06/08 19:25:31  elphel
*!  optional delay (1 frame) for 10359 i2c commands
*!
*!  Revision 1.10  2010/06/06 04:25:52  elphel
*!  Fixed handling FLIPH/FLIPV when switching between composite/single modes and navigating between sensors in single mode
*!
*!  Revision 1.9  2010/06/02 16:31:04  elphel
*!  Added P_MULTI_SELECTED parameter
*!
*!  Revision 1.8  2010/06/01 08:30:36  elphel
*!  support for the FPGA code 03534022  with optional master time stamp over the inter-camera sync line(s)
*!
*!  Revision 1.7  2010/05/28 23:55:54  elphel
*!  fixed switching the sensor clock source, added line to the startup script to set this mode
*!
*!  Revision 1.6  2010/05/25 00:56:08  elphel
*!  Report error if 10359 SDRAM phase adjustment failed (one branch was missing)
*!
*!  Revision 1.5  2010/05/25 00:52:23  elphel
*!  8.0.8.20, working on multi-sensor
*!
*!  Revision 1.4  2010/05/21 06:12:16  elphel
*!  continue working on multi-sensor software
*!
*!  Revision 1.3  2010/05/17 16:03:46  elphel
*!  8.0.8.15, working on multisensor support
*!
*!  Revision 1.2  2010/05/16 02:03:47  elphel
*!  8.0.8.4 - driver working with individual/broadcast sensor registers
*!
*!  Revision 1.1  2010/05/13 03:31:08  elphel
*!  initial driver for multi-sensor applications
*/


/****************** INCLUDE FILES SECTION ***********************************/
#include <linux/types.h> // for div 64
#include <asm/div64.h>   // for div 64

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/time.h>


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>

//#include <asm/system.h>
#include <asm/io.h>

#include <asm/irq.h>

//#include <asm/delay.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/c313a.h>

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//#include "x3x3.h" // detect sensor

//#include "cci2c.h"
//#include "fpgaconfi2c.h"  //to control clocks
#include "mt9x001.h"
#include "multi10359.h"
#include "framepars.h"      // parameters manipulation
#include "sensor_common.h"
#include "pgm_functions.h"
#include "sensor_i2c.h"
#include "clock10359.h"
#include "x393.h"


/**
 * \def D(x) optional debug output 
 */


#if ELPHEL_DEBUG
 #define MDF(x) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;}
 #define  MDF1(x) { if (GLOBALPARS(G_DEBUG) & (1 << 1)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define  MDF3(x) { if (GLOBALPARS(G_DEBUG) & (1 << 3)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define  MDF4(x) { if (GLOBALPARS(G_DEBUG) & (1 << 4)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define MDF16(x) { if (GLOBALPARS(G_DEBUG) & (1 <<16)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define MDF24(x) { if (GLOBALPARS(G_DEBUG) & (1 <<24)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }
 #define MDF25(x) { if (GLOBALPARS(G_DEBUG) & (1 <<25)) {printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);x ;} }

// #define ELPHEL_DEBUG_THIS 0
 #define ELPHEL_DEBUG_THIS 1
#else
 #define MDF(x)
 #define MDF1(x)
 #define MDF3(x)
 #define MDF4(x)
 #define MDF16(x)
 #define MDF24(x)
 #define MDF25(x)
 #define ELPHEL_DEBUG_THIS 0
#endif

#ifdef NC353 // to hide old code
#endif

// porting 10353, 10359 r/w does not use sequencer, so pages allocation is handled by sensor_i2c module

//const char * name_10359  = "el10359"; // Get name from DT (together with port mask)
//const char * name_sensor = "mt9p006"; // Get name from DT (together with port mask)
//const char * name_clock =  "cy22393"; // Get name from DT (together with port mask)



//int x393_setClockFreq(int sensor_port, int nclock, int freq);
//int x393_getClockFreq(int sensor_port, int nclock);

#if 0
struct multi_pages_t {
	int w1;        ///< allocated page for single-byte writes to 10359
	int w2;        ///< allocated page for two byte writes to 10359
	int w4;        ///< allocated page for 4 byte writes to 10359
	int r1;        ///< allocated page for single-byte reads from 10359
	int r2;        ///< allocated page for two byte reads from 10359
	int r4;        ///< allocated page for 4 byte byte reads from 10359
	int sns_wr_all;///< allocated page to write to all sensors of the same port
	int sns_wr1;   ///< allocated page to write to sensor 1
	int sns_wr2;   ///< allocated page to write to sensor 2
	int sns_wr3;   ///< allocated page to write to sensor 3
	int sns_rd1;   ///< allocated page to read from sensor 1
	int sns_rd2;   ///< allocated page to read from sensor 2
	int sns_rd3;   ///< allocated page to read from sensor 3
	int clk_wr;    ///< allocated page to write to clock
	int clk_rd;    ///< allocated page to read from clock
};

// ** acquire i2c table pages for communicating with 10359 mux boards attached to the sensor ports * /
int setup_i2c_pages(int ports) ///< bitmask of the sensor ports to use
{
	int chn,pg;
	x393_i2c_device_t dev_10359, dev_sen, dev_clk;
	BUG_ON(!xi2c_dev_get(name_10359));
	// make a local copy of i2c device class to modify it
	memcpy(&dev_10359, xi2c_dev_get(name_10359), sizoef(xi2c_dev_get(name_10359)));

	for (chn=1; chn< SENSOR_PORTS; chn++) if (ports & (1 << chn)) {
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 1; alloc_pages[chn].w1 = pg; set_xi2c_wrc(dev_10359, chn, pg, 0);
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 2; alloc_pages[chn].w2 = pg; set_xi2c_wrc(dev_10359, chn, pg, 0);
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 4; alloc_pages[chn].w4 = pg; set_xi2c_wrc(dev_10359, chn, pg, 0);
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 1; alloc_pages[chn].r1 = pg; set_xi2c_rdc(dev_10359, chn, pg);
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 2; alloc_pages[chn].r2 = pg; set_xi2c_rdc(dev_10359, chn, pg);
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); dev_10359.data_bytes = 4; alloc_pages[chn].r4 = pg; set_xi2c_rdc(dev_10359, chn, pg);
		dev_10359.data_bytes = 2;
		pg =  i2c_page_alloc(chn); BUG_ON(pg<0); alloc_pages[chn].sns_wr_all = pg; set_xi2c_wrc(dev_10359, chn, pg, 0);




	}
}

#endif

// will set "i" to the result
#ifdef NC353 // to hide old code
#define MULTISENSOR_WRITE_I2C(sa,ra,v,sz) \
    {rslt |= multisensor_write_i2c((sa),(ra),(v),(sz)) ; \
     MDF1(printk(" multisensor_write_i2c(0x%x, 0x%x, 0x%x, %d) -> %d\n",(int)(sa),(int)(ra),(int)(v),(int)(sz),rslt));}
#define MULTISENSOR_WRITE_I2C16(ra,v) \
    {rslt |= multisensor_write_i2c((I2C359_SLAVEADDR),(ra),(v),2) ; \
     MDF1(printk(" multisensor_write_i2c(0x%x, 0x%x, 0x%x, %d) -> %d\n",(int)(I2C359_SLAVEADDR),(int)(ra),(int)(v),2,rslt));}
#define MULTISENSOR_WRITE_I2C32(ra,v) \
    {rslt |= multisensor_write_i2c((I2C359_SLAVEADDR),(I2C359_MSW),(v)>>16,2) ; \
     MDF1(printk(" multisensor_write_i2c(0x%x, 0x%x, 0x%x, %d) -> %d\n",(int)(I2C359_SLAVEADDR),I2C359_MSW,(int)(v)>>16,2,rslt)); \
     rslt |= multisensor_write_i2c((I2C359_SLAVEADDR),(ra),(v) & 0xffff,2) ; \
     MDF1(printk(" multisensor_write_i2c(0x%x, 0x%x, 0x%x, %d) -> %d\n",(int)(I2C359_SLAVEADDR),(int)(ra),(int)(v)&0xffff,2,rslt)); \
}
#else
// using new access in immediate mode by class name
#define MULTISENSOR_WRITE_I2C(port,name,offs,ra,v) \
    {rslt |= multisensor_write_i2c((port),(name),(offs),(ra),(v)) ; \
     MDF1(printk(" multisensor_write_i2c(%d, %s, 0x%x, 0x%x, 0x%x) -> %d\n",(int)(port),name,int(offs),(int)(ra),(int)(v),rslt));}
#define MULTISENSOR_WRITE_I2C16(port,ra,v) \
    {rslt |= multisensor_write_i2c((port),(name_10359),0,(ra),(v)) ; \
     MDF1(printk(" multisensor_write_i2c(%d, %s, 0x%x, 0x%x) -> %d\n",(int)(port),name_10359,(int)(ra),(int)(v),rslt));}
#define MULTISENSOR_WRITE_I2C32(port,ra,v) \
    {rslt |= multisensor_write_i2c((port),(name_10359),0,(I2C359_MSW),(v)>>16) ; \
     MDF1(printk(" multisensor_write_i2c(%d, %s, 0x%x, 0x%x) -> %d\n",(int)(port),name_10359,I2C359_MSW,(int)(v)>>16,rslt)); \
     rslt |= multisensor_write_i2c((port),(name_10359),0,(ra),        (v) & 0xffff) ; \
     MDF1(printk(" multisensor_write_i2c(%d, %s, 0x%x, 0x%x) -> %d\n",(int)(port),name_10359,(int)(ra),(int)(v)&0xffff,2,rslt)); \
}
#endif
//pars_to_update - local variable
#define MULTISENSOR_WRITE_I2C16_SHADOW(p,ra,v) \
    { pars_to_update[nupdate  ].num= P_M10359_REGS+(ra) ;\
      pars_to_update[nupdate++].val=(v);\
      MULTISENSOR_WRITE_I2C16((p),(ra),(v));\
    }

#define MULTISENSOR_WRITE_I2C32_SHADOW(p,ra,v) \
    { pars_to_update[nupdate  ].num= P_M10359_REGS+(ra) ;\
      pars_to_update[nupdate++].val=(v);\
      MULTISENSOR_WRITE_I2C32((p),(ra),(v));\
    }



#define SET_10359_PAR16(p,f,r,v) \
    { pars_to_update[nupdate  ].num= P_M10359_REGS+(r) ;\
      pars_to_update[nupdate++].val=(v);\
      X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (r), (v)); \
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(r),(int)(v)));\
    }

#define SET_10359_PAR32(p,f,r,v) \
    { pars_to_update[nupdate  ].num= P_M10359_REGS+(r) ;\
      pars_to_update[nupdate++].val=(v);\
      unsigned long flags; \
      local_irq_save(flags); \
      X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (I2C359_MSW), (v)>>16); \
      X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (r), (v) & 0xffff); \
      local_irq_restore(flags); \
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(I2C359_MSW),(int)(v)>>16));\
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(r),(int)(v) & 0xffff));\
    }

#define SET_10359_PAR(p, f,r,v) \
    { if ((r)>=P_M10359_REGS32BIT) SET_10359_PAR32(p, f,r,v) \
      else                         SET_10359_PAR16(p, f,r,v) \
    }
#define SET_10359_COND(p, f,r,v) \
   {if (thispars->pars[P_M10359_REGS+(r)]!=(v)) SET_10359_PAR(p,f,r,v)}


//     if (sequence!=thispars->pars[P_M10359_REGS+I2C359_CHNSEQ])

// same functions, but do not schedule updating parameter shadows, just send i2c
#define SET_10359_REG16(p, f,r,v) \
    { X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (r), (v)); \
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(r),(int)(v)));\
    }

#define SET_10359_REG32(p, f,r,v) \
    { unsigned long flags; \
      local_irq_save(flags); \
      X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (I2C359_MSW), (v)>>16); \
      X3X3_I2C_SEND2((p), (f), (I2C359_SLAVEADDR), (r), (v) & 0xffff); \
      local_irq_restore(flags); \
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(I2C359_MSW),(int)(v)>>16));\
      MDF1(printk(" X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n",(int)(f),(int)(I2C359_SLAVEADDR),(int)(r),(int)(v) & 0xffff));\
    }

#define SET_10359_REG(p,f,r,v) \
    { if ((r)>=P_M10359_REGS32BIT) SET_10359_REG32(p,f,r,v) \
      else                         SET_10359_REG16(p,f,r,v) \
    }


#define SENSOR_IN_SEQ(x,s)     ((((s)>>((x)<<1)) & 3)-1) // returns hardware channel number (0..2) for frame 0..2 in sequence , -1 if disabled
#define SENSOR_IN_SEQ_EN(x,s,m) ((((s)>>((x)<<1)) & 3)  && (((m)>> ((((s)>>((x)<<1)) & 3)-1)) & 1)) // returns hardware channel number (0..2) for frame 0..2 in sequence , -1 if disabled


//int multisensor_read_i2c(unsigned char theSlave, unsigned char theRegister, unsigned long *theData, int size);
int multisensor_read_i2c           (int sensor_port, const char * class_name, int sa7_offs, u32 reg_addr, u32 * reg_datap);
//int multisensor_write_i2c(unsigned char theSlave, unsigned char theRegister, unsigned long theData, int size);
int multisensor_write_i2c          (int sensor_port, const char * class_name,int sa7_offs, u32 reg_addr, u32 reg_data);
int multisensor_pgm_multisens      (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int multisensor_pgm_sensorphase    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int multisensor_set_freq           (int sensor_port, int first, struct framepars_t * thispars);
int multisensor_set_phase_verify   (int sensor_port, int reg_addr, int resetDCM, unsigned long newPhase, unsigned long oldPhase);
int multisensor_set_phase_recover  (int sensor_port, int reg_addr, int resetDCM, unsigned long newPhase, unsigned long oldPhase);

int multisensor_set_phase          (int sensor_port, int reg_addr, int resetDCM, unsigned long newPhase, unsigned long oldPhase);
int multisensor_initSDRAM          (int sensor_port, struct framepars_t * thispars);
int calcThisPhase                  (int clk_period, long FPGADelay, long cableDelay, long sensorDelay);
int multisensor_memphase_debug     (int sensor_port, int write);
int multisensor_memphase           (int sensor_port, int * centroid0x10000);
int multisensor_adjustSDRAM        (int sensor_port, int maxAdjust);
int multisensor_pgm_sensorregs     (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
//int multisensor_pgm_afterinit0  (struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16);
int multisensor_pgm_window         (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16) ;
int multisensor_pgm_window_safe    (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16) ;
int multisensor_pgm_window_common  (int sensor_port, struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame16) ;

// These two parameters are copied from sensor
 static int multi_phases_initialized;
 static struct sensorproc_t s_sensorproc_phys; // physical sensor parameters and functions to call
 struct sensorproc_t * sensorproc_phys = &s_sensorproc_phys;

/** program sensor WOI and mirroring
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands
 * As different sensors may produce "bad frames" for different WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped) */
int multisensor_pgm_window      (int sensor_port,               ///< sensor port number (0..3)
								 struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
								 struct framepars_t * thispars, ///< sensor current parameters
								 struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
								 int frame16)                   ///< 4-bit (hardware) frame number parameters should
															    ///< be applied to,  negative - ASAP
															    ///< @return always 0
{
  int rslt;
  MDF4(printk(" frame16=%d\n",frame16));
  rslt = multisensor_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
  return rslt;

}

/** program sensor WOI and mirroring, "safe" mode, but currently does the same as multisensor_pgm_window
 * Validating, changing related parameters/scheduling actions, scheduling i2c commands
 * As different sensors may produce "bad frames" for differnt WOI changes (i.e. MT9P001 seems to do fine with FLIP, but not WOI_WIDTH)
 * pgm_window and pgm_window_safe will do the same - they will just be called with different latencies and with compressor stopped) */
int multisensor_pgm_window_safe (int sensor_port,               ///< sensor port number (0..3)
								 struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
								 struct framepars_t * thispars, ///< sensor current parameters
								 struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
								 int frame16)                   ///< 4-bit (hardware) frame number parameters should
																///< be applied to,  negative - ASAP
																///< @return always 0
{
  int rslt;
  MDF4(printk(" frame16=%d\n",frame16));
  rslt = multisensor_pgm_window_common (sensor_port, sensor,  thispars, prevpars, frame16);
  return rslt;
}

/** Calculate adjusted window left margin */
inline int sensor_wl(int wl,                   ///< Specified window left margin
		             int ww,                   ///< Specified window width
					 int flipX,                ///< Flip image around vertical axis
					 int dh,                   ///< horizontal decimation
					 int bh,                   ///< horizontal binning
					 int oversize,             ///< use full sensor array, not just designated pixels
					 struct sensor_t * sensor) ///< sensor description data
					                           ///< @return corrected left margin
{
    int i,d;
    if (!oversize) {
      if(flipX) {
         wl = sensor->clearWidth - ww - wl - (2 * COLOR_MARGINS) * dh;
         if(wl < 0) wl = 0;
      }
// apply clearTop/clearLeft
      wl = (wl + sensor->clearLeft) & 0xfffe; 
// apply binning restrictions
      switch(sensor->sensorType & 7) {
      case MT9P_TYP:
         d = (bh > 1) ? ((bh > 2) ? 16 : 8) : 4;
         if(flipX)
            i = d * ((wl - 2) / d) + 2;
         else
            i = d * (wl / d);
         if(i < wl)
            i += d;
         wl = i;
         break;
      case MT9T_TYP:
         d = (bh > 1) ? ((bh > 2) ? (16) : (8)) : (4);
         if(flipX)
            i = d * ((wl - 2) / d) + 2;
         else
            i = d * (wl / d);
         if(i < wl)
            i += d;
         wl = i;
         break;
      }
  }
  return wl;
}

/** Calculate adjusted window top margin */
inline int sensor_wt(int wt,                   ///< Specified window top margin
		             int wh,                   ///< Specified window height
					 int flipY,                ///< Flip image around horizontal axis
					 int dv,                   ///< vertical decimation
					 int oversize,             ///< use full sensor array, not just designated pixels
					 struct sensor_t * sensor) ///< sensor description data
											   ///< @return corrected top margin
{
  if (!oversize) {
      if(flipY) {
         wt = sensor->clearHeight - wh - wt - (2 * COLOR_MARGINS) * dv;
         if(wt < 0) wt = 0;
      }
// apply clearTop/clearLeft
      wt = (wt + sensor->clearTop) & 0xfffe; 
// apply binning restrictions - none for vertical
  }
  return wt;
}

//MULTI_TOPSENSOR - calculate and save
//#define EARLY_10359 1 //set it to 0 when there will be one extra level of buffering registers in 10359. Currently 10359 is one frame ahead of the sensor
// May add function argument from pgm_window / pgm_window_safe?
/** Common part of programming sensor window */
int multisensor_pgm_window_common  (int sensor_port,               ///< sensor port number (0..3)
									struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
									struct framepars_t * thispars, ///< sensor current parameters
									struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
									int frame16)                   ///< 4-bit (hardware) frame number parameters should
																   ///< be applied to,  negative - ASAP
																   ///< @return always 0
{
  struct frameparspair_t  pars_to_update[50];  // 11 for 10359,sensor: 3 - broadcast (can be x4) and 4 individual (x3), 11+3*4+4*3+1+1 = 37
  int nupdate=0;
  MDF4(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//  int fpga_addr=   (frame16 <0) ? X313_I2C_ASAP : (X313_I2C_FRAME0+frame16);
  int fpga_addr=   frame16 <0;

//  int fpga_addr359= (frame16 <0) ? X313_I2C_ASAP : (X313_I2C_FRAME0+((frame16 + ((GLOBALPARS(G_MULTI_CFG)>>G_MULTI_CFG_DLYI2C) & 1)) & PARS_FRAMES_MASK)); // will point to one i2c frame later than fpga_addr
  int fpga_addr359= frame16 + ((GLOBALPARS(sensor_port, G_MULTI_CFG)>>G_MULTI_CFG_DLYI2C) & 1); // will point to one i2c frame later than fpga_addr

  int height1,height2,height3,vblank2,vblank3;
  unsigned long wois[12];
  int i,dv,dh,bv,bh,ww,w359,wh,wl,wt,flip,flipX,flipY,d, v;
  int multiFlipX=0;
  int multiFlipY=0;
  int relFlipX; //= (multiFlipX ^ (multiFlipX>>1))& 3;
  int relFlipY; //= (multiFlipY ^ (multiFlipY>>1))& 3;
  int sFlipX[3];
  int sFlipY[3];
  int hactDelay=0;
  int horBlank=thispars->pars[P_MULTI_HBLANK];
// Calculate and program 10359 registers
  int composite=     (thispars->pars[P_MULTI_MODE])?1:0;
  int async=        (thispars->pars[P_TRIG] & 4)?1:0;
  int selected =     thispars->pars[P_MULTI_SELECTED]; // 1..3 - selected in single frame mode

  if (composite && (!async)) {
    printk("*** ERROR (Should be disabled in multisensor_pgm_multisens() ) CANNOT USE COMPOSITE MODE WITH FREE RUNNING SENSOR ***\n");
    composite=0;
    SETFRAMEPARS_SET(P_MULTI_MODE,0); // Do we need to force anything here? If it was async->free transition? Or just TRIG mode should have all the dependencies of P_MULTI_MODE
  }

  int sequence=thispars->pars[P_MULTI_SEQUENCE];
  int active=0; // sensors to program
//  int sensor_mask=(thispars->pars[P_MULTISENS_EN]) & GLOBALPARS(G_SENS_AVAIL);  // sensor mask should already be applied to sequence in afterinit
  flip=((thispars->pars[P_FLIPH] & 1) | ((thispars->pars[P_FLIPV] & 1) << 1 )) ^ sensor->init_flips; // 10338 is _not_ flipped (as the ther boards, but for legacy compatibility....)
  flipX =  flip & 1;
  flipY = (flip & 2)? 1:0;
  for (i=0;i<3;i++) {
    sFlipX[i]=flipX ^ (composite ?((thispars->pars[P_MULTI_FLIPH] & (1<<i))?1:0):0); // in single (non -composite) mode use just flipX
    sFlipY[i]=flipY ^ (composite ?((thispars->pars[P_MULTI_FLIPV] & (1<<i))?1:0):0); // in single (non -composite) mode use just flipY
  }
//sFlip* are now per-sensor absolute flips
  MDF25(printk("selected=%x flipX=%x flipY=%x sFlipX[0]=%x sFlipY[0]=%x sFlipX[1]=%x sFlipY[1]=%x sFlipX[2]=%x sFlipY[2]=%x\n", selected, flipX,flipY,sFlipX[0],sFlipY[0],sFlipX[1],sFlipY[1],sFlipX[2],sFlipY[2]));
// calculations valid for individual and composite frames
  int styp = sensor->sensorType & 7;
  dh=  thispars->pars[P_DCM_HOR];
  dv=  thispars->pars[P_DCM_VERT];
  bh=  thispars->pars[P_BIN_HOR];
  bv=  thispars->pars[P_BIN_VERT];
  ww=  thispars->pars[P_SENSOR_PIXH] * dh;
//  SETFRAMEPARS_SET(P_SENSOR_PIXV,  height+(2 * COLOR_MARGINS)); // full height for the sensor (after decimation), including margins
  wh=  thispars->pars[P_SENSOR_PIXV] * dv; // number of scan lines read from the sensor multipled by decimation
  d = 2 * bh * (ww / (2 * bh));
  if (unlikely(d != ww)) { // correct window width if needed
     SETFRAMEPARS_SET(P_SENSOR_PIXH, d / dh);
         ww = d;
  }
// height
  d = (wh+1) & 0xfffe; // round up to even
  if (unlikely(d != wh)) { // correct window height if needed
     SETFRAMEPARS_SET(P_SENSOR_PIXV, d / dv);
     wh = d;
  }
// Margins
  wl = thispars->pars[P_WOI_LEFT];
  wt = thispars->pars[P_WOI_TOP];
// flip margins for mirrored images (except oversized, not to rely on sensor->clearWidth/sensor->clearHeight
//-------------------------
  MDF25(printk("dv=0x%x dh=0x%x ww=0x%x wh=0x%x wl=0x%x wt=0x%x \n",dv,dh,ww,wh,wl,wt));

  memcpy(wois, &(thispars->pars[P_MULTI_WOI]), sizeof(wois)); // copy WOI parameters for 3 sensors
  if (composite && flipY) { // reverse sequence
      for (i=sequence,sequence=0; i & 3; i>>=2)
         sequence = (sequence<<2) | (i & 3);
  }
// Now sequence may be opposite to that of the P_MULTI_SEQUENCE
//  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+0] += (2 * COLOR_MARGINS)*dv; // include margins into individual sensor WOI-sizes
//  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+1] += (2 * COLOR_MARGINS)*dv;
//  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+2] += (2 * COLOR_MARGINS)*dv;
  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+0] = (((wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+0]/dv) & ~1)+ (2 * COLOR_MARGINS))*dv; // include margins into individual sensor WOI-sizes, make even number of output lines
  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+1] = (((wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+1]/dv) & ~1)+ (2 * COLOR_MARGINS))*dv;
  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+2] = (((wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+2]/dv) & ~1)+ (2 * COLOR_MARGINS))*dv;



  height1= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]; // these are like WOI_HEIGHT, margins not included
  height2= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(1,sequence)]; // may be garbage if disabled
  height3= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(2,sequence)]; // may be garbage if disabled
  vblank3=vblank2=  ((thispars->pars[ P_MULTI_VBLANK]) & ~1)*dv; // even
  if (composite ) {
// wl,ww, wh,wt, - before decimation,!!!!!!!!!!!!!!!! - where to add it? to afterinit? - no just handle it here

// Modify sequence if one or 2 frames is skipped from top because of P_WOI_TOP
// Margins
    if (flipX) {
      wl= (thispars->pars[P_SENSOR_WIDTH]+dh*(2 * COLOR_MARGINS))-ww-wl; // simplified (ww includes margins*dh)
      if(wl < 0) wl = 0;
    }
    if (flipY) {
      wt= (thispars->pars[P_SENSOR_HEIGHT]+dv*(2 * COLOR_MARGINS))-wh-wt; // simplified (wh includes margins*dv)
      if(wt < 0) wt = 0;
    }
    if ((wt>height1) && (sequence >>2)) { // only skip if anything is left
      sequence>>=2;
      wt-=height1+vblank2;
      if (wt<0) wt=0; // can not start from blank;
      height1=height2;
      height2=height3;

      if ((wt>height1) && (sequence >>2)) { // only skip if anything is left
        sequence>>=2;
        wt-=height1+vblank2;
        if (wt<0) wt=0; // can not start from blank;
        height1=height2;
        height2=height3;
      }
    }
// correct heights if frame 1 is the same channel as one of the next, but fewer lines (no way to make number of lines in direct channel less than full sensor output)
    if ((((sequence ^ (sequence>>2)) & 3)==0) && (((sequence ^ (sequence>>4)) & 3)==0)) {//  all 3 frames are the same channel
      if ((height1 < height2) || (height1 < height2)) {
        if (height3>height2) {
          i=height3; height3=height1; height1=i;
        } else  {
          i=height2; height2=height1; height1=i;
        } 
      }
    } else if (((sequence ^ (sequence>>2)) & 3)==0) { // 1 and 2 frames are the same sensor
      if (height1 < height2) {
        i=height2; height2=height1; height1=i;
      }
    } else if (((sequence ^ (sequence>>4)) & 3)==0) { // 1 and 3 frames are the same sensor
      if (height1 < height3) {
        i=height3; height3=height1; height1=i;
      }
    }
    for (i=0; (v=SENSOR_IN_SEQ(i,sequence))>=0;i++) active  |= (1<<v);
  MDF25(printk("height1=0x%x height2=0x%x height3=0x%x\n",height1, height2, height3));



// Set number of sensor used in direct frame, so pgm_sensorin can correctly set Bayer shift. Used only in composite mode
    SETFRAMEPARS_COND(P_MULTI_TOPSENSOR, SENSOR_IN_SEQ(0,sequence));
// Adjust vblank2, vblank3 for common Bayer (if there is any vertical flips)
    for (i=0;i<3;i++) {
      v=SENSOR_IN_SEQ(i,sequence);
      if (v>=0) {
//        multiFlipX |= ((thispars->pars[P_MULTI_FLIPH]>>v)<<i); // thispars->pars[P_MULTI_FLIPH] applies to sensor ports, not frames
//        multiFlipY |= ((thispars->pars[P_MULTI_FLIPV]>>v)<<i);
        multiFlipX |= (((thispars->pars[P_MULTI_FLIPH]>>v) & 1)<<i); // thispars->pars[P_MULTI_FLIPH] applies to sensor ports, not frames
        multiFlipY |= (((thispars->pars[P_MULTI_FLIPV]>>v) & 1)<<i);
      }
    }
// multiFlip* are set in the sub-frame sequence (accounting for the global FLIP_V, bit 0 - top subframe,1 - middle, 2 - last
    MDF25(printk("multiFlipX=0x%x multiFlipY=0x%x\n",multiFlipX, multiFlipY));

// modify - multiFlipX,multiFlipY applies to sensors, not frames ???? No, they apply to sub-frames
    relFlipX= (multiFlipX ^ (multiFlipX>>1))& 3;
    relFlipY= (multiFlipY ^ (multiFlipY>>1))& 3;
    MDF25(printk("relFlipX=0x%x relFlipY=0x%x\n",relFlipX, relFlipY));
    if (relFlipY & 1) vblank2+=dv;
    if (relFlipY & 2) vblank3+=((vblank3>=dv) && (vblank2>vblank3))?-dv:dv; // Keep the total height constant, if possible. If not possible - increase

    hactDelay=(relFlipX ^ (relFlipX << 1)) & 3;


    MDF25(printk("hactDelay=0x%x vblank2=0x%x, vblank3=0x%x\n",hactDelay, vblank2, vblank3));
// Apply height1, height2, height3
    wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]=height1; // always on,>= other frame heights
    if ((SENSOR_IN_SEQ(1,sequence)>=0) && (SENSOR_IN_SEQ(1,sequence)!=SENSOR_IN_SEQ(0,sequence))) {
      if (SENSOR_IN_SEQ(1,sequence)==SENSOR_IN_SEQ(2,sequence)) {// same channel for frames 2,3
         wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(1,sequence)]=(height2>height3)?height2:height3; // maximum of 2 frames
      } else {
         wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(1,sequence)]=height2;
         if ((SENSOR_IN_SEQ(2,sequence)>=0) && (SENSOR_IN_SEQ(2,sequence)!=SENSOR_IN_SEQ(0,sequence))) {
           wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(2,sequence)]=height3;
         }
      }
    }
// Apply wt to the sensor read as direct frame
    wois[(P_MULTI_TOP1-   P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]+=wt;
    wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]-=wt;
// Apply wl to all channels
    wois[(P_MULTI_LEFT1-P_MULTI_WOI)+0]+=wl;
    wois[(P_MULTI_LEFT1-P_MULTI_WOI)+1]+=wl;
    wois[(P_MULTI_LEFT1-P_MULTI_WOI)+2]+=wl;
// Apply ww to all channels
    wois[(P_MULTI_WIDTH1-P_MULTI_WOI)+0]=ww; // all channels - same width, include margins (before decimation)
    wois[(P_MULTI_WIDTH1-P_MULTI_WOI)+1]=ww;
    wois[(P_MULTI_WIDTH1-P_MULTI_WOI)+2]=ww;
    MDF25(printk("wois[LEFT1]=0x%lx, wois[LEFT2]=0x%lx, wois[LEFT3]=0x%lx\n", wois[(P_MULTI_LEFT1-P_MULTI_WOI)+0],wois[(P_MULTI_LEFT1-P_MULTI_WOI)+1],wois[(P_MULTI_LEFT1-P_MULTI_WOI)+2] ));
    MDF25(printk("wois[WIDTH1]=0x%lx\n", wois[(P_MULTI_WIDTH1-P_MULTI_WOI)+0] ));
    MDF25(printk("wois[TOP1]=0x%lx, wois[TOP2]=0x%lx, wois[TOP3]=0x%lx\n", wois[(P_MULTI_TOP1-P_MULTI_WOI)+0],wois[(P_MULTI_TOP1-P_MULTI_WOI)+1],wois[(P_MULTI_TOP1-P_MULTI_WOI)+2] ));
    MDF25(printk("wois[HEIGHT1]=0x%lx, wois[HEIGHT2]=0x%lx, wois[HEIGHT3]=0x%lx\n", wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+0],wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+1],wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+2] ));


// TODO: Apply required height to the channels. If nothing but the direct channel is needed - apply it to sensor parameters
    if ((wt+wh) < (height1+vblank2+(sensor->minHeight*dv))) { // Only first (direct) channel is needed, modify sensor parameters
// Modify first sensor height even if it is increased to include part of the gap and minHeight
      wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]= wh;
      sequence &= 3; // turn off frames 2, 3
      MDF25(printk("sequence=%x height1=0x%x vblank2=0x%x height2=0x%x vblank3=0x%x height3=0x%x \n",sequence, height1, vblank2, height2, vblank3 ,height3 ));
    } else if ((wt+wh-height1-vblank2) < (height2+vblank3+(sensor->minHeight*dv))) { // Only first 2 sensors are needed - reduce second frame (10359, not the sensor)
      height2=wh+wt-height1-vblank2; // only 10359, not the sensor
      sequence &= 0xf; // turn off frame  3
      MDF25(printk("sequence=%x height1=0x%x vblank2=0x%x height2=0x%x vblank3=0x%x height3=0x%x \n",sequence, height1, vblank2, height2, vblank3 ,height3 ));
    } else { // all 3 sensors needed, adjust height3 to make sure 10359 sends out exactly wh/dv lines (vblank* could be adjusted to compensate for Bayer when flipping)
      height3=wh+wt-height1-vblank2-height2-vblank3;
      MDF25(printk("sequence=%x height1=0x%x vblank2=0x%x height2=0x%x vblank3=0x%x height3=0x%x \n",sequence, height1, vblank2, height2, vblank3 ,height3 ));
    }

// zero out unused frames/blanks before them
    if (SENSOR_IN_SEQ(1,sequence) <0 ) {
      height2=0;
      vblank2=0;
    }
    if ((SENSOR_IN_SEQ(1,sequence) <0) || (SENSOR_IN_SEQ(2,sequence) <0)) {
      height3=0;
      vblank3=0;
    }
// un-apply dv from height1,height2,height3,vblank2,vblank3;
    if (dv>1) {
      height1/=dv;
      vblank2/=dv;
      height2/=dv;
      vblank3/=dv;
      height3/=dv;
    }
    MDF25(printk("height1=0x%x vblank2=0x%x height2=0x%x vblank3=0x%x height3=0x%x \n",height1, vblank2, height2, vblank3 ,height3 ));
// un-apply dh from ww - number of pixels to be read in a line in frames2,3
    if (dh>1) ww/=dh;
    w359=ww;
// horBlank

// Set P_MULTI_MODE_FLIPS, P_MULTI_HEIGHT_BLANK1, P_MULTI_HEIGHT_BLANK2 to be used in MakerNote data
    int multi_mode_flips= ( multiFlipX & 1)     | ((multiFlipX & 2) << 1)  | ((multiFlipX & 4) << 2) |
                          ((multiFlipY & 1)<<1) | ((multiFlipY & 2) << 2)  | ((multiFlipY & 4) << 3) | 0x40; // 0x40 as a composite frame mark (test with &0xc0!=0 - future)
    if (flipX)  multi_mode_flips ^= 0x15;
    if (flipY)  multi_mode_flips ^= 0x2a;
    MDF25(printk("sequence=0x%x flipX=%x flipY=%x multi_mode_flips=0x%x \n",sequence,flipX,flipY,multi_mode_flips ));
    SETFRAMEPARS_COND(P_MULTI_MODE_FLIPS,    multi_mode_flips);
//    subtract (2 * COLOR_MARGINS) from the first and last frame

//    SETFRAMEPARS_COND(P_MULTI_HEIGHT_BLANK1, ((height1 - ((2 * COLOR_MARGINS)>>1)*(((height2==0) && (height3==0))?2:1)) & 0xffff) | (vblank2<<16));
//    SETFRAMEPARS_COND(P_MULTI_HEIGHT_BLANK2, ((height2 - ((2 * COLOR_MARGINS)>>1)*(((height2 >0) && (height3==0))?1:0)) & 0xffff) | (vblank3<<16));
// Currently (2 * COLOR_MARGINS) (4 pixels) are subtracted from the very last sub-frame only
    SETFRAMEPARS_COND(P_MULTI_HEIGHT_BLANK1, ((height1 - (2 * COLOR_MARGINS)*(((height2==0) && (height3==0))?1:0)) & 0xffff) | (vblank2<<16));
    SETFRAMEPARS_COND(P_MULTI_HEIGHT_BLANK2, ((height2 - (2 * COLOR_MARGINS)*(((height2 >0) && (height3==0))?1:0)) & 0xffff) | (vblank3<<16));

if (GLOBALPARS(sensor_port, G_MULTI_CFG) & (1 <<G_MULTI_CFG_BEFORE)) {
//TODO: Program 10359 registers here
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HACT_WIDTH, w359);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_WIDTH2,     w359);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_WIDTH3,     w359);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HEIGHT2,    height2);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HEIGHT3,    height3);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_VBLANK2,    vblank2);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_VBLANK3,    vblank3);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HBLANK2,    horBlank);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HBLANK3,    horBlank);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HACT_DLY,   hactDelay);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_CHNSEQ,     sequence);
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_MODE,       I2C359_MODE_MUTI_EN); // Mode=4 - combined frames
}
  } else { // single-sensor mode
     w359=ww; // just to make compiler happy
     SET_10359_COND(sensor_port, fpga_addr359, I2C359_HACT_WIDTH, w359);
     active=(1<<(selected-1));
     wois[(P_MULTI_WIDTH1- P_MULTI_WOI)+(selected-1)]=ww;
     wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+(selected-1)]=wh;
     wois[(P_MULTI_LEFT1-  P_MULTI_WOI)+(selected-1)]=wl;
     wois[(P_MULTI_TOP1-   P_MULTI_WOI)+(selected-1)]=wt;
// Program 10359 register(s) here - just mode=0?
    if (GLOBALPARS(sensor_port, G_MULTI_CFG) & (1 <<G_MULTI_CFG_BEFORE)) {
      SET_10359_COND(sensor_port, fpga_addr359, I2C359_CHNSEQ, (sequence & 0x3c) | (selected & 3));
      SET_10359_COND(sensor_port, fpga_addr359, I2C359_MODE,   0); // Mode=0 - single frame
    }
    SETFRAMEPARS_COND(P_MULTI_MODE_FLIPS,0); // zero out multi mode, leave P_MULTI_HEIGHT_BLANK1, P_MULTI_HEIGHT_BLANK2 unchanged (may be garbage)
  }

// program 1-3 active sensors (first - common/broadcast parameters?)
// TODO - use individual WOI

// comparisons are only valid if there are no individual parameters that were changed
// program sensors width (same for all sensors, use broadcast mode)
  ww=wois[(P_MULTI_WIDTH1- P_MULTI_WOI)+(composite?SENSOR_IN_SEQ(0,sequence):(selected-1))];
  MDF25(printk(" selected=%x, thispars->pars[P_MULTI_SELECTED]=%x composite=%x sequence=%x\n",    selected, (int) thispars->pars[P_MULTI_SELECTED], composite, sequence));

  if ((ww-1) != thispars->pars[P_SENSOR_REGS+P_MT9X001_WIDTH]) {
     SET_SENSOR_MBPAR(sensor_port, fpga_addr, sensor->i2c_addr, P_MT9X001_WIDTH, ww-1);
     MDF4(printk("  SET_SENSOR_MBPAR(0x%x,0x%x, 0x%x, 0x%x)\n", fpga_addr,  (int) sensor->i2c_addr, (int) P_MT9X001_WIDTH, (int) ww-1));
  }
// Program binning/decimation (also common but some older sensors)
  if((styp == MT9T_TYP) || (styp == MT9P_TYP)) { // 3MPix and 5MPix sensors
     v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM] & 0xff88) | ((bv - 1) << 4) | (dv - 1) ;
     if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RAM]) {
        SET_SENSOR_PAR(sensor_port, fpga_addr,sensor->i2c_addr, P_MT9X001_RAM, v);
        MDF4(printk("  SET_SENSOR_PAR(0x%x,0x%x, 0x%x, 0x%x)\n", fpga_addr,  (int) sensor->i2c_addr, (int) P_MT9X001_RAM, (int) v));
     }
     v=(thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM] & 0xff88) | ((bh - 1) << 4) | (dh - 1);
     if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_CAM]) {
        SET_SENSOR_PAR(sensor_port, fpga_addr,sensor->i2c_addr, P_MT9X001_CAM, v);
        MDF4(printk("  SET_SENSOR_PAR(0x%x,0x%x, 0x%x, 0x%x)\n", fpga_addr,  (int) sensor->i2c_addr, (int) P_MT9X001_CAM, (int) v));
     }
  } else { // 1.3 and 2 MPix sensors
     v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1] & 0xffc3) | // preserve other bits from shadows (trigger mode moved to other function)
         ((dh == 4) ? (1 << 2) : 0) | // Column skip 4
         ((dv == 4) ? (1 << 3) : 0) | // Row skip    4
         ((dh == 8) ? (1 << 4) : 0) | // Column skip 8
         ((dv == 8) ? (1 << 5) : 0) ; // Row skip    8
     if (v != thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE1]) {
        SET_SENSOR_MBPAR(sensor_port,fpga_addr,sensor->i2c_addr, P_MT9X001_RMODE1, v);
        MDF4(printk("  SET_SENSOR_MBPAR(0x%x,0x%x, 0x%x, 0x%x)\n", fpga_addr,  (int) sensor->i2c_addr, (int) P_MT9X001_RMODE1, (int) v));
     }
   }
// Other registers are programmed individually
   for (i=0; i<3;i++) if (active & (1<<i)) {
// Calculate individual parameters here
     wh=wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(i,sequence)];
     wl=sensor_wl(wois[(P_MULTI_LEFT1-  P_MULTI_WOI)+SENSOR_IN_SEQ(i,sequence)], ww, sFlipX[i], dh, bh, thispars->pars[P_OVERSIZE], sensor);
     wt=sensor_wt(wois[(P_MULTI_TOP1-   P_MULTI_WOI)+SENSOR_IN_SEQ(i,sequence)], wh, sFlipY[i], dv,     thispars->pars[P_OVERSIZE], sensor);

// program sensor height
     SET_SENSOR_MIBPAR_COND(sensor_port,fpga_addr,sensor->i2c_addr, i, P_MT9X001_HEIGHT, wh-1);
// Program sensor left margin
     SET_SENSOR_MIBPAR_COND(sensor_port,fpga_addr,sensor->i2c_addr, i, P_MT9X001_COLSTART, wl);
// Program sensor top margin
       SET_SENSOR_MIBPAR_COND(sensor_port,fpga_addr,sensor->i2c_addr, i, P_MT9X001_ROWSTART, wt);

     if((styp == MT9T_TYP) || (styp == MT9P_TYP)) { // 3MPix and 5MPix sensors
       v= (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fff) | // preserve other bits from shadows
           (sFlipX[i] ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
           (sFlipY[i] ? (1 << 15) : 0) ; // FLIPV
       SET_SENSOR_MIBPAR_COND(sensor_port,fpga_addr,sensor->i2c_addr, i, P_MT9X001_RMODE2, v);
     } else { // 1.3 and 2 MPix sensors
       v=  (thispars->pars[P_SENSOR_REGS+P_MT9X001_RMODE2] & 0x3fe7) | // preserve other bits from shadows
           ((dh == 2) ? (1 << 3) : 0) | // Column skip 2
           ((dv == 2) ? (1 << 4) : 0) | // Row skip    2
           (sFlipX[i] ? (1 << 14) : 0) | // FLIPH - will control just alternative rows
           (sFlipY[i] ? (1 << 15) : 0) ; // FLIPV
       SET_SENSOR_MIBPAR_COND(sensor_port,fpga_addr,sensor->i2c_addr, i, P_MT9X001_RMODE2, v);
     }
   }
  if (!(GLOBALPARS(sensor_port, G_MULTI_CFG) & (1 <<G_MULTI_CFG_BEFORE))) {  // try after sensors
	  if (composite ) {
	//TODO: Program 10359 registers here
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HACT_WIDTH, w359);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_WIDTH2,     w359);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_WIDTH3,     w359);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HEIGHT2,    height2);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HEIGHT3,    height3);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_VBLANK2,    vblank2);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_VBLANK3,    vblank3);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HBLANK2,    horBlank);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HBLANK3,    horBlank);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_HACT_DLY,   hactDelay);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_CHNSEQ,     sequence);
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_MODE,       I2C359_MODE_MUTI_EN); // Mode=4 - combined frames
	  } else { // single-sensor mode
	// Program 10359 register(s) here - just mode=0?
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_CHNSEQ, (sequence & 0x3c) | (selected & 3));
		 SET_10359_COND(sensor_port, fpga_addr359, I2C359_MODE,   0); // Mode=0 - single frame
	  }
	}
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return 0;
}


#ifdef NC353 // to hide old code
/* *
 * @brief Disable hardware i2c to sensor and read 1..4 bytes from specified slave/register (big endian)
 * @param theSlave  i2c slave address (0x02..0xfe)
 * @param theRegister register address
 * @param theData  pointer to data to be returned
 * @param size - number of bytes to read 
 * @return OK - 0, <0 - error
 */
int multisensor_read_i2c(unsigned char theSlave, unsigned char theRegister, unsigned long *theData, int size) {
  unsigned long flags; // this function uses software i2c operations - they need to have interrupts (and hardware i2c off)
  unsigned char    i2c_read_data[4]; // each two bytes - one short word, big endian
  int i;
  if ((size<1) || (size>4))return -1;
  local_irq_save(flags); // IRQ Off
    int hardware_i2c_running=i2s_running();
    if (hardware_i2c_running) i2c_stop_wait();
    i=i2c_writeData(0, theSlave & 0xfe, &theRegister, 1, 0); // no stop before read  (cxi2c.c)
    if (i==0) i=i2c_readData (0, theSlave | 1,    i2c_read_data, size, 0); // restart, not strart  (cxi2c.c)
    if (hardware_i2c_running) i2c_run(); // turn hardware i2c back on
  local_irq_restore(flags); // IRQ restore
//  if (i!=0) return -1;
  if (i!=0) return -i; // negative error code
  theData[0]=0;
  for (i=0;i<size;i++)  theData[0]=(theData[0]<<8) | i2c_read_data[i] ;
  return 0;
 }
#else
/** read (2 bytes) from 10359 board or one of the sensors */
int multisensor_read_i2c(int  sensor_port, ///< sensor port number
						 const char * class_name, ///< device class name ("el10359", "mt9p006")
						 int          sa7_offs,   ///< 0 for 10359, 2/4/6 for sensor channels
						 u32          reg_addr,   ///< 8-bit register number
		                 u32        * reg_datap)  ///< received data (2 bytes)
		                                          ///< @return 0: success, negative - error
{
	return x393_xi2c_read_reg(class_name,       // device class name
							  sensor_port,      // sensor port number
							  sa7_offs,         // slave address (7-bit) offset from the class defined slave address
							  reg_addr,         // register address (width is defined by class)
							  reg_datap);       // data to write (width is defined by class)
}
#endif


#ifdef NC353 // to hide old code
/* *
 * @brief Disable hardware i2c to sensor and write 1..4 bytes to specified slave/register (big endian)
 * @param theSlave  i2c slave address (0x02..0xfe)
 * @param theRegister register address
 * @param theData data to be written
 * @param size - number of bytes to write
 * @return OK - 0, <0 - error
 */
int multisensor_write_i2c(unsigned char theSlave, unsigned char theRegister, unsigned long theData, int size) {
  unsigned long flags; // this function uses software i2c operations - they need to have interrupts (and hardware i2c off)
  unsigned char    i2c_write_data[5];
  int i;
  if ((size<1) || (size>4))return -1;
  i2c_write_data[0]= theRegister;
  for (i=size;i>0;i--) {
     i2c_write_data[i]=theData & 0xff;
     theData >>= 8;
  }
  local_irq_save(flags); // IRQ Off
    int hardware_i2c_running=i2s_running();
    if (hardware_i2c_running) i2c_stop_wait();
    i=i2c_writeData(0, theSlave & 0xfe,i2c_write_data, size+1, 1); // send stop in the end
    if (hardware_i2c_running) i2c_run(); // turn hardware i2c back on
  local_irq_restore(flags); // IRQ restore
  if (i!=0) return -1;
  return 0;
}
#else
/** Write (2 bytes) to 10359 board or sensor*/
int multisensor_write_i2c(int          sensor_port,///< sensor port number
                          const char * class_name, ///< device class name ("el10359", "mt9p006")
						  int          sa7_offs,   ///< 0 for 10359 and sensor broadcast, 2/4/6 for sensor channels
		                  u32 reg_addr,            ///< 8-bit register number
		                  u32 reg_data)            ///< data to send (2 bytes)
		                                           ///< @return 0: success, negative - error
{
	return x393_xi2c_write_reg(class_name,       // device class name
							   sensor_port,      // sensor port number
							   sa7_offs,         // slave address (7-bit) offset from the class defined slave address
							   reg_addr,         // register address (width is defined by class)
							   reg_data);        // data to write (width is defined by class)
}
#endif


static int multi_unitialized=0; ///< temporary hack to resolve race between individual and multi_ flips at startup

/**
 * @brief detect and initialize sensor and related data structures
 * - detect sensor type.
 * - if successful, proceed to:,
 * - copy sensor static structure
 * - setup appropriate pgm_* functions
 * - read sensor registers to shadows
 * - initialize appropriate P_* registers (including sensor register shadows) - that initialization will schedule related pgm_* functions
 *
 * TODO: when is P_CLK_FPGA initialized? Needs to be done before this
 * hardware i2c is expected to be reset and initialized - no wrong, it will be programmed in 
 * onchange_i2c should be the first after init sensor (even before onchange_sensorphase)
 * onchange_sensorphase will be triggered after this
 * hardware i2c after this function will be disabled, will need onchange_sensorphase to initialize/start it. */
int multisensor_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
									struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
									struct framepars_t * thispars, ///< sensor current parameters
									struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
									int frame16)                   ///< 4-bit (hardware) frame number parameters should
																   ///< be applied to,  negative - ASAP
																   ///< @return always 0
{
  struct frameparspair_t pars_to_update[64]; // so far 39 actually needed
  int nupdate=0;
  x393_sensio_ctl_t sensio_ctl = {.d32=0};
  x393_camsync_mode_t camsync_mode = {.d32=0};

  unsigned long    bitstream_version;
  unsigned long    sensor_id[MAX_SENSORS];
  int rslt=0; // or-ed by MULTISENSOR_WRITE_I2C(sa,ra,v,sz)
  int i;
  int this_sensor_type;
//   .hact_delay  = -2500,    // -2.5ns delay in ps
//   .sensorDelay = 2460,     // Delay from sensor clock at FPGA output to pixel data transition (FPGA input), short cable (ps)
  multi_unitialized=0; // reset this static variable - it will prevent copying individual flips to multiple until composite mode is used
  MDF3(printk(" frame16=%d\n",frame16));
  GLOBALPARS(sensor_port,G_SENS_AVAIL)=0; // no 10359A board present
  if (frame16 >= 0) return -1; // can only work in ASAP mode
  if (thispars->pars[P_SENSOR]) return 0; // Sensor is already detected - do not bother (to re-detect it P_SENSOR should be set to 0)
  MDF24(printk("Probing 10359 board, i2c bitdelays=0x%08x, hardware_i2c_running=%d\n",i2c_delays(0),i2s_running()));
#ifdef NC353
  if (multisensor_read_i2c(I2C359_SLAVEADDR, I2C359_VERSION, &bitstream_version, 4)<0) return -1;
#else
  if (X3X3_I2C_RCV4(sensor_port, I2C359_SLAVEADDR, I2C359_VERSION, &bitstream_version)<0) return -1;
#endif
// Here we see that 10359A responds somehow. If next fails, no sense to try other sensor types.
  multi_phases_initialized=0;
  sensor->sensorType=SENSOR_NONE;
  add_sensor_proc(onchange_multisens,      &multisensor_pgm_multisens);        // process 10359-specific parameters
  add_sensor_proc(onchange_sensorphase,    &multisensor_pgm_sensorphase);     // set clock/phase for the 10359A

  if ((((bitstream_version ^ I2C359_MINVERSION) & 0xffff0000)!=0) || ((bitstream_version & 0xffff) < (I2C359_MINVERSION & 0xffff))) {
    printk ("invalid 10359 bitstream version, found 0x%08lx, required >= 0x%08x\n",bitstream_version, I2C359_MINVERSION );
    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType);
    return -1;
  }
  printk("10359 bitstream version =0x%08lx\n",bitstream_version);
// now set sensor clock in both system board and 10359A to 96MHz - currently we support only 5MPix in thias mode
#ifdef NC353
  setFramePar(sensor_port, thispars, P_CLK_FPGA,  getClockFreq(0)); // just in case - read the actual fpga clock frequency and store it (no actions)
  setFramePar(sensor_port, thispars, P_CLK_SENSOR,  96000000);
  setClockFreq(sensor_port, 1, thispars->pars[P_CLK_SENSOR]);
#endif
  printk("10353 sensor clock set to %d\n",(int) thispars->pars[P_CLK_SENSOR]);

  udelay (100);// 0.0001 sec to stabilize clocks
//  X3X3_RSTSENSDCM;  // FPGA DCM can fail after clock change, needs to be reset
//  X3X3_SENSDCM_CLK2X_RESET; // reset pclk2x DCM also
  // NC393 reset mmcm - is it needed? Clock was not changed yet
  sensio_ctl.mmcm_rst = 1;
  sensio_ctl.mmcm_rst_set = 1;
  x393_sensio_ctrl(sensio_ctl,sensor_port);

// reset system and SDRAM DCMs on 10359
  MULTISENSOR_WRITE_I2C16(sensor_port,I2C359_DCM_SYSTEM,  I2C359_DCM_RESET | I2C359_DCM_RESET90);
  MULTISENSOR_WRITE_I2C16(sensor_port,I2C359_DCM_SDRAM,   I2C359_DCM_RESET | I2C359_DCM_RESET90);
  multisensor_initSDRAM(sensor_port, thispars); // init 10359 SDRAM
  rslt=0;
// TODO: read other?
//  MULTISENSOR_WRITE_I2C16_SHADOW(I2C359_I2CMUX, I2C359_I2CMUX_2MEM);

//TODO: Is it needed for NC393?
#ifdef NC353
  if (rslt!=0) { // select memory/clock i2c bus
    printk ("10359A did not respond after changing 10353 sensor clock frequency to 96MHz\n");
    setFramePar(sensor_port, thispars, P_CLK_SENSOR,  20000000);
    setClockFreq(1, thispars->pars[P_CLK_SENSOR]);
    printk("10353 sensor clock set to %d\n",(int) thispars->pars[P_CLK_SENSOR]);
    mdelay (50);// 0.05 sec to stabilize clocks
    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return -1;
  }
#endif
  rslt=multisensor_set_freq  (sensor_port, 1, thispars); // first time (1)
  if (rslt>0)       printk("10359A sensor clock set to %d\n", rslt);
  else if (rslt==0) printk("10359A sensors are using 10353 system clock, as set in configuration\n");
  else              printk("10359  sensor clock failure, will use system clock from 10353 board\n");
// Try to read chip version from each of the 3 possible sensors
  printk("removing MRST from the sensor\n");
//
  sensio_ctl.d32 = 0;
  sensio_ctl.mrst =     1;
  sensio_ctl.mrst_set = 1;
  sensio_ctl.arst =     1;
  sensio_ctl.arst_set = 1;
  x393_sensio_ctrl(sensio_ctl,sensor_port);
  camsync_mode.trig =     0;
  camsync_mode.trig_set = 1;
  x393_camsync_mode (camsync_mode);
  udelay (100);

  GLOBALPARS(sensor_port,G_SENS_AVAIL) |= 1<< (GLOBALPARS(sensor_port,G_SENS_AVAIL)); // temporary to indicate sensor detection functions that they need to initialize multisensor registers
  for (i=0;i<MAX_SENSORS;i++) {
    MDF24(printk("Probing sensor port %d, i2c bitdelays=0x%08x\n",i,i2c_delays(0)));
#ifdef NC353
    rslt=  multisensor_read_i2c(sensor_port,
                                MT9P001_I2C_ADDR + ((i+1) * I2C359_INC),
                                P_MT9X001_CHIPVER,
                                &sensor_id[i],
                                2);
#else
    rslt=  X3X3_I2C_RCV2 (sensor_port,
                          MT9P001_I2C_ADDR + ((i+1) * I2C359_INC),
                          P_MT9X001_CHIPVER,
                          &sensor_id[i]);
#endif
    MDF24(printk("Probing sensor port %d, i2c bitdelays=0x%08x, rslt=0x%x\n",i,i2c_delays(0),rslt));
    if (rslt==0) {
       if (((sensor_id[i] ^ MT9P001_PARTID) & MT9X001_PARTIDMASK)==0) {
          printk("Found MT9P001 2592x1944 sensor on 10359A port %d, chip ID=%lx\n",(i+1), sensor_id[i]);
          GLOBALPARS(sensor_port,G_SENS_AVAIL) |= 1<<i;
       } else {
          printk("Found UNSUPPORTED sensor on port %d, chip ID=0x%lx\n",(i+1),sensor_id[i]);
       }
    } else sensor_id[i]=0;
  }
  GLOBALPARS(sensor_port,G_SENS_AVAIL) &= (1<< (GLOBALPARS(sensor_port,G_SENS_AVAIL)))-1; // remove flag used to indicate sensor detection functions that they need to initialize multisesnor registers
  if (GLOBALPARS(sensor_port,G_SENS_AVAIL)==0) {
    printk ("No supported sensors connected to 10359A board\n");
    setFramePar(sensor_port, thispars, P_SENSOR,  sensor->sensorType);
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
    return 0;
  }
  printk ("Setting internal HACT generation\n");
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HACT_MODE, 7);
// At least one MT9P0X1 sensor found, initializing them in broadcast mode (will still need to modify phases - both 10353 and 10359
  this_sensor_type=mt9x001_pgm_detectsensor(sensor_port, sensor,  thispars, prevpars, frame16);  // try Micron 5.0 Mpixel - should return sensor type
//  for (i=0;i<8;i++) {
//    MDF(printk("i=%d, m=0x%lx\n",i,GLOBALPARS(G_MULTI_REGSM+i)));
//  }
  initMultiPars(sensor_port); // this time the registors that need to have individual shadows are known, initialize the corresponding data structures
//  memcpy(psensor, sensor, sizeof(struct sensor_t)); // copy physical sensor definitions to the save area (so some can be replaced by modified ones)
//  MDF24(printk(" before: sensorproc_phys->sensor.sensorDelay=0x%x\n", sensorproc_phys->sensor.sensorDelay));
  copy_sensorproc(sensor_port, sensorproc_phys);                // save physical sensor functions
//  MDF24(printk(" after: sensorproc_phys->sensor.sensorDelay=0x%x\n", sensorproc_phys->sensor.sensorDelay));

  // Now calculate phases, swap ones from the sensor
  long * multiOutDelay= (long *) &GLOBALPARS(sensor_port, G_DLY359_OUT);
// these two will be used to calulate sensor/hact phase in 10353

  sensor->hact_delay=0; // No hact delay on 10359 output
  sensor->sensorDelay=multiOutDelay[0];
  MDF24(printk("replaced: sensor->sensorDelay=0x%x\n", sensor->sensorDelay));
//sensorproc_phys->sensor
/*
Now overwrite sensor functions with it's own (originals (physical sensor ones) are already copied to the local structure
*/
//  add_sensor_proc(onchange_multisens,      &multisensor_pgm_multisens);        // process 10359-specific parameters
  add_sensor_proc(onchange_sensorregs,  &multisensor_pgm_sensorregs);  // write sensor 10359 registers (only changed from outside the driver)
  add_sensor_proc(onchange_window,      &multisensor_pgm_window);       // program sensor WOI and mirroring (flipping)
  add_sensor_proc(onchange_window_safe, &multisensor_pgm_window_safe);  // program sensor WOI and mirroring (flipping) - now - only flipping? with lower latency

// Initialize other 10359A parameters (move it to init/afterinit?)

// SET_10359_PAR(f,r,v)

  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_MODE,I2C359_MODE_RESET | I2C359_MODE_DISABLE); // disable output + reset to all regs
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_MODE,                    I2C359_MODE_DISABLE); // disable output, reset off
// mcontr channels configuration
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CHEN,I2C359_SDRAM_RUN(0) |\
                                                   I2C359_SDRAM_RUN(1) |\
                                                   I2C359_SDRAM_RUN(2) |\
                                                   I2C359_SDRAM_RUN(3) |\
                                                   I2C359_SDRAM_RUN(4) |\
                                                   I2C359_SDRAM_RUN(5) |\
                                                   I2C359_SDRAM_RUN(6) |\
                                                   I2C359_SDRAM_RUN(7) |\
                                                   I2C359_SDRAM_RUN(8));
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CONF01A,   0xf020f); // parameters for write channel 0
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CONF01B,0x1fff10ff); // parameters for write channel 0
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CONF23A,   0xf020f); // parameters for write channel 0
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CONF23B,0x1fff10ff); // parameters for write channel 0
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_MAGIC43,0x1c000c21); // "don't remember"
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CHEN,I2C359_SDRAM_STOP(0) |\
                                                   I2C359_SDRAM_STOP(1) |\
                                                   I2C359_SDRAM_STOP(2) |\
                                                   I2C359_SDRAM_STOP(3) |\
                                                   I2C359_SDRAM_STOP(4) |\
                                                   I2C359_SDRAM_STOP(5) |\
                                                   I2C359_SDRAM_STOP(6) |\
                                                   I2C359_SDRAM_STOP(7) |\
                                                   I2C359_SDRAM_STOP(8)); // Do you really need to stop refresh?
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_START0,0); // start address for channel 0?
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_START1,0); // start address for channel 0?
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_START2,0x10000); // start address for channel 0?
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_START3,0x10000); // start address for channel 0?
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CHEN,I2C359_SDRAM_RUN(0) |\
                                                   I2C359_SDRAM_RUN(1) |\
                                                   I2C359_SDRAM_RUN(2) |\
                                                   I2C359_SDRAM_RUN(3) |\
                                                   I2C359_SDRAM_RUN(4) |\
                                                   I2C359_SDRAM_RUN(5) |\
                                                   I2C359_SDRAM_RUN(6) |\
                                                   I2C359_SDRAM_RUN(7) |\
                                                   I2C359_SDRAM_RUN(8));

  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HACT_WIDTH,               2596); // internal HACT generation
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_WIDTH3,                   2596); // frame width,  channel 3
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HEIGHT3,                  1940); // frame height, channel 3
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_WIDTH2,                   2596); // frame width,  channel 2
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HEIGHT2,                  1940); // frame height, channel 2

  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HBLANK3,                 0x100); // extra pixels in line (horizontal blank), frame 3
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_VBLANK3,                   0x8); // extra lines before frame 3
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_HBLANK2,                 0x100); // extra pixels in line (horizontal blank), frame 2
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_VBLANK2,                   0x8); // extra lines before frame 2
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_INTERFRAME_DELAY,        0);    // not needed
//  MULTISENSOR_WRITE_I2C16_SHADOW(I2C359_CHNSEQ,I2C359_SEQ(1,2,3));      // channel 1 - direct, channel2, 3 - through memory, in that sequence
  MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_MODE,  0);           // enable combined mode if >1 sensors and trigered mode

//  int async=(thispars->pars[P_TRIG] & 4)?1:0;
//  SETFRAMEPARS_SET(P_MULTI_SEQUENCE,    I2C359_SEQ(1,2,3));
  SETFRAMEPARS_SET(P_MULTI_SEQUENCE,    0); // will set sequence later
  SETFRAMEPARS_SET(P_MULTI_FLIPH,       0);
  SETFRAMEPARS_SET(P_MULTI_FLIPV,       0);
  SETFRAMEPARS_SET(P_MULTI_MODE,        0);
  SETFRAMEPARS_SET(P_MULTI_HBLANK,  0x100);
  SETFRAMEPARS_SET(P_MULTI_VBLANK,      8);
  SETFRAMEPARS_SET(P_MULTI_HEIGHT1,  1940);
  SETFRAMEPARS_SET(P_MULTI_HEIGHT2,  1940);
  SETFRAMEPARS_SET(P_MULTI_HEIGHT3,  1940);

#ifdef NC353
  CCAM_RESET_MCONTR_ON ; // Set mode that resets memory controller pointers after each frame sync. TODO: Later - make it work without?
  CCAM_ENDFRAMES_EN ;    // Enable ending frame being compressed if no more data will be available (frame ended before specified number of blocks compressed)
#endif
  if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
  return this_sensor_type;
}

/**
 * @brief Switches between single sensor/composite modes (composite possible only in triggered mode), prepares WOI parameters
 * NOTE: some parameters are not processed - i.e. sensor clock frequency - add them to the sensor?
 * add multisensor_pgm_sensorphase - it is not used by the mt9x!
 *
 * TODO: Save/restore individual FLIPS (MULTI_C_FLIPS).
 * NOTE: Should [P_SENSOR_HEIGHT] include gaps multiplied by dv? */
int multisensor_pgm_multisens (int sensor_port,               ///< sensor port number (0..3)
                               struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                               struct framepars_t * thispars, ///< sensor current parameters
                               struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                               int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                              ///< be applied to,  negative - ASAP
                                                              ///< @return always 0
{
  struct frameparspair_t pars_to_update[40]; // 35 needed
  int nupdate=0;
  int i,j,sh;
  unsigned long wois[12];
  int dv=  thispars->pars[P_DCM_VERT];
  MDF3(printk(" frame16=%d\n",frame16));
// Handling sensor mask - which sensors (of the available) to use
  int sensor_mask=(thispars->pars[P_MULTISENS_EN]) & GLOBALPARS(sensor_port, G_SENS_AVAIL);
  if (!sensor_mask) sensor_mask=GLOBALPARS(sensor_port, G_SENS_AVAIL) ;// if none sensors were enabled - enable all what is available (same as with WOI size)
// handling P_MULTI_SEQUENCE
  int sequence=       thispars->pars[P_MULTI_SEQUENCE];
//  int prev_sequence=  prevpars->pars[P_MULTI_SEQUENCE];
  int async=          (thispars->pars[P_TRIG] & 4)?1:0;
  int composite=      (thispars->pars[P_MULTI_MODE])?1:0;
  int prev_composite= (prevpars->pars[P_MULTI_MODE])?1:0;
  int selected =      thispars->pars[P_MULTI_SELECTED];
  int prev_selected = prevpars->pars[P_MULTI_SELECTED];

  if (composite && (!async)) {
    printk("*** CANNOT USE COMPOSITE MODE WITH FREE RUNNING SENSOR ***\n");
    composite=0;
    SETFRAMEPARS_SET(P_MULTI_MODE,0); // Do we need to force anything here? If it was async->free transition? Or just TRIG mode should have all the dependencies of P_MULTI_MODE
  }
// TODO: recalculate sequence when MULTISENS_EN is chnaged (tried 5 - did not modify sequence, showed first frame only)
// Here - only recalculate SENSOR_HEIGHT, skipping disabled sensors. in multisensor_pgm_window() will need to skip disabled also
// No above is not really needed, sequence+enable can be handled by application software
  MDF25(printk(" composite=0x%x\n",composite));
  MDF25(printk(" sequence=0x%x\n",    sequence));
// if sequence is zero, put "1-2-3"
  if (sequence==0) {
    j=1;
    sh=0;
    for (i=GLOBALPARS(sensor_port, G_SENS_AVAIL); i; i>>=1,j++) { // use available, not enabled sensors here
      if (i&1) {
        sequence |= (j<<sh);
        sh+=2;
      }
    }
  } else { // validate (and fix if needed) the sensor sequence (uses 'available', not 'enabled')
// first entry should be non-zero, all the rest should be 0 or available
    if (!(sequence & 3)) sequence |=1;
    for (i=0;i<6;i+=2) {
      j=(sequence>>i) & 3;
      if (j) { // allow zeros
        if (((GLOBALPARS(sensor_port, G_SENS_AVAIL)>>(j-1))&1)==0) {
          while (((GLOBALPARS(sensor_port, G_SENS_AVAIL)>>(j-1))&1)==0) { // (GLOBALPARS(G_SENS_AVAIL) & 7) !=0
            j++;
            if (j>3) j=1;
          }
// replace that channel in the sequence
          sequence &= ~(3<<i) ;
          sequence |=   j<<i ;
        }
      }
    }
  }
  if ((selected==0) || !((1 << (selected-1)) & GLOBALPARS(sensor_port, G_SENS_AVAIL))) selected=sequence & 3; // if not set or invalid - set to first in sequence

  MDF25(printk(" selected=0x%x, thispars->pars[P_MULTI_SELECTED]=0x%x\n",    selected, (int) thispars->pars[P_MULTI_SELECTED]));
  MDF25(printk(" sequence=0x%x\n",    sequence));
  MDF25(printk(" sensor_mask=0x%x\n", sensor_mask));
  SETFRAMEPARS_COND(P_MULTI_SELECTED, selected);
  SETFRAMEPARS_COND(P_MULTI_SEQUENCE, sequence);
  SETFRAMEPARS_COND(P_MULTISENS_EN,   sensor_mask);
  memcpy(wois, &(thispars->pars[P_MULTI_WOI]), sizeof(wois)); // copy WOI parameters for 3 sensors
  int multi_fliph=thispars->pars[P_MULTI_FLIPH];
  int multi_flipv=thispars->pars[P_MULTI_FLIPV];
  int old_sensor=prev_selected-1; // may be <0
  int new_sensor=selected-1;     // >=0
  if (multi_unitialized && (!prev_composite) && (old_sensor>=0)) { // was single-sensor mode, copy P_WOI_* to individual sensor WOI and FLIPS
    MDF25(printk(" multi_unitialized=%d  old_sensor=%x, multi_fliph=%x multi_flipv=%x\n",  multi_unitialized,  old_sensor, multi_fliph,multi_flipv));
    wois[(P_MULTI_WIDTH1- P_MULTI_WOI)+old_sensor]= prevpars->pars[P_WOI_WIDTH];
    wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+old_sensor]= prevpars->pars[P_WOI_HEIGHT];
    wois[(P_MULTI_LEFT1-  P_MULTI_WOI)+old_sensor]= prevpars->pars[P_WOI_LEFT];
    wois[(P_MULTI_TOP1-   P_MULTI_WOI)+old_sensor]= prevpars->pars[P_WOI_TOP];
    multi_fliph=    (multi_fliph & (~(1<<old_sensor)))  | ((prevpars->pars[P_FLIPH] & 1) << old_sensor);
    multi_flipv=    (multi_flipv & (~(1<<old_sensor)))  | ((prevpars->pars[P_FLIPV] & 1) << old_sensor);
    MDF25(printk(" multi_unitialized=%d old_sensor=%x, multi_fliph=%x multi_flipv=%x\n",  multi_unitialized,  old_sensor, multi_fliph,multi_flipv));
  }
  if (multi_unitialized && (!composite) && (prev_composite || ((new_sensor>=0) && (old_sensor!=new_sensor)))) { // now single-sensor mode, set P_WOI* from saved parameters
    if ((wois[(P_MULTI_WIDTH1-  P_MULTI_WOI)+new_sensor]==0) || (wois[(P_MULTI_HEIGHT1-  P_MULTI_WOI)+new_sensor]==0)) {
// This particular sensor was never initialized
      if (prev_composite || // should never get here - if that was composite mode before, P_MULTI_* should be already set. Using sensor defaults
         (old_sensor<0) || // ther was no old sensor or it was disabled
         ((prevpars->pars[P_WOI_WIDTH]==0) || (prevpars->pars[P_WOI_HEIGHT]==0)) ) {
// Nothing to take from old, use sensor defaults
            wois[(P_MULTI_WIDTH1- P_MULTI_WOI)+new_sensor]= sensor->imageWidth;
            wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+new_sensor]= sensor->imageHeight;
            wois[(P_MULTI_LEFT1-  P_MULTI_WOI)+new_sensor]= 0;
            wois[(P_MULTI_TOP1-   P_MULTI_WOI)+new_sensor]= 0;
            multi_fliph=    (multi_fliph & (~(1<<new_sensor))); // =0
    MDF25(printk(" new_sensor=%x old_sensor=%x, multi_fliph=%x multi_flipv=%x\n",  new_sensor,  old_sensor, multi_fliph,multi_flipv));
            multi_flipv=    (multi_flipv & (~(1<<new_sensor))); // =0
      } else { // was one single channel, now the different (not initialized window) one - copy window parameters
        wois[(P_MULTI_WIDTH1- P_MULTI_WOI)+new_sensor]= prevpars->pars[P_WOI_WIDTH];
        wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+new_sensor]= prevpars->pars[P_WOI_HEIGHT];
        wois[(P_MULTI_LEFT1-  P_MULTI_WOI)+new_sensor]= prevpars->pars[P_WOI_LEFT];
        wois[(P_MULTI_TOP1-   P_MULTI_WOI)+new_sensor]= prevpars->pars[P_WOI_TOP];
        multi_fliph=    (multi_fliph & (~(1<<new_sensor)))  | ((prevpars->pars[P_FLIPH] & 1) << new_sensor);
        multi_flipv=    (multi_flipv & (~(1<<new_sensor)))  | ((prevpars->pars[P_FLIPV] & 1) << new_sensor);
    MDF25(printk(" new_sensor=%x old_sensor=%x, multi_fliph=%x multi_flipv=%x\n",  new_sensor,  old_sensor, multi_fliph,multi_flipv));
      }
    }
 // saved sensor WOI are OK (or just fixed), use them
    SETFRAMEPARS_COND(P_WOI_WIDTH,  wois[(P_MULTI_WIDTH1-  P_MULTI_WOI)+new_sensor]);
    SETFRAMEPARS_COND(P_WOI_HEIGHT, wois[(P_MULTI_HEIGHT1- P_MULTI_WOI)+new_sensor] );
    SETFRAMEPARS_COND(P_WOI_LEFT,   wois[(P_MULTI_LEFT1-   P_MULTI_WOI)+new_sensor] );
    SETFRAMEPARS_COND(P_WOI_TOP,    wois[(P_MULTI_TOP1-    P_MULTI_WOI)+new_sensor] );
    SETFRAMEPARS_COND(P_FLIPH,      (multi_fliph>>new_sensor) & 1);
    SETFRAMEPARS_COND(P_FLIPV,      (multi_flipv>>new_sensor) & 1);
    MDF25(printk(" new_sensor=%x old_sensor=%x, multi_fliph=%x multi_flipv=%x\n",  new_sensor,  old_sensor, multi_fliph,multi_flipv));
  }
// Validate hights for all enabled channels (OK to skip disabled here)
  int oversize=thispars->pars[P_OVERSIZE];
// some may be garbage if the channel is disabled, but it will not be used
  int height1= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)];
  int height2= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(1,sequence)];
  int height3= wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(2,sequence)];
  int vblank=  (thispars->pars[P_MULTI_VBLANK]) & ~1; // even
  int total_height=0;
  if      (!height1)                      height1=sensor->imageHeight;
  if      (height1 < sensor->minHeight)   height1=sensor->minHeight;
  else if (height1 > sensor->arrayHeight) height1=sensor->arrayHeight; // Includes black pixels
  else if ((!oversize) && (height1 > sensor->imageHeight)) height1=sensor->imageHeight; // use clearHeight here?
  wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(0,sequence)]=height1;
  total_height=height1;
  MDF25(printk(" total_height=0x%x\n",total_height));
// is there frame 2 enabled?
  int multi_frame=0;
  if (composite && SENSOR_IN_SEQ_EN(1,sequence,sensor_mask)) {  // specified in sequence is enabled
    multi_frame=1;
    total_height+=(vblank+(2 * COLOR_MARGINS))*dv;
  MDF25(printk(" total_height=0x%x\n",total_height));
    if      (!height2)                      height2=sensor->imageHeight;
    if      (height2 < sensor->minHeight)   height2=sensor->minHeight;
    else if (height2 > sensor->arrayHeight) height2=sensor->arrayHeight;
    else if ((!oversize) && (height2 > sensor->imageHeight)) height2=sensor->imageHeight; // use clearHeight here?
    wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(1,sequence)]=height2;
    total_height+=height2;
  MDF25(printk(" total_height=0x%x\n",total_height));
    if (SENSOR_IN_SEQ_EN(2,sequence,sensor_mask)) {
      total_height+=(vblank+(2 * COLOR_MARGINS))*dv;
  MDF25(printk(" total_height=0x%x\n",total_height));
      if      (!height3)                      height3=sensor->imageHeight;
      if      (height3 < sensor->minHeight)   height3=sensor->minHeight;
      else if (height3 > sensor->arrayHeight) height3=sensor->arrayHeight;
      else if ((!oversize) && (height2 > sensor->imageHeight)) height3=sensor->imageHeight; // use clearHeight here?
      wois[(P_MULTI_HEIGHT1-P_MULTI_WOI)+SENSOR_IN_SEQ(2,sequence)]=height3;
      total_height+=height3;
  MDF25(printk(" total_height=0x%x\n",total_height));
    }
  }
  if (composite) {
    if (!prev_composite) { // restore from saved values
      if ((thispars->pars[P_MULTI_CWIDTH]==0) || (thispars->pars[P_MULTI_CHEIGHT]==0)) { // Were never set before - use full size
        SETFRAMEPARS_COND(P_WOI_HEIGHT,        total_height); // add FRAMEPAIR_FORCE_NEW to trigger?
        SETFRAMEPARS_COND(P_MULTI_CHEIGHT,     total_height);
        SETFRAMEPARS_COND(P_WOI_WIDTH,         sensor->imageWidth);
        SETFRAMEPARS_COND(P_MULTI_CWIDTH,      sensor->imageWidth);
        SETFRAMEPARS_COND(P_WOI_LEFT,          0);
        SETFRAMEPARS_COND(P_MULTI_CLEFT,       0);
        SETFRAMEPARS_COND(P_WOI_TOP,           0);
        SETFRAMEPARS_COND(P_MULTI_CTOP,        0);
        SETFRAMEPARS_COND(P_MULTI_CFLIPH,      0);
        SETFRAMEPARS_COND(P_MULTI_CFLIPV,      0);
      } else { // single->composite, not the first time, use saved parameters
        SETFRAMEPARS_COND(P_WOI_HEIGHT,  thispars->pars[P_MULTI_CHEIGHT]); // add FRAMEPAIR_FORCE_NEW to trigger?
        SETFRAMEPARS_COND(P_WOI_WIDTH,   thispars->pars[P_MULTI_CWIDTH]);
        SETFRAMEPARS_COND(P_WOI_LEFT,    thispars->pars[P_MULTI_CLEFT]);
        SETFRAMEPARS_COND(P_WOI_TOP,     thispars->pars[P_MULTI_CTOP]);
        SETFRAMEPARS_COND(P_FLIPH,       thispars->pars[P_MULTI_CFLIPH]); // Set global flips from saved values
        SETFRAMEPARS_COND(P_FLIPV,       thispars->pars[P_MULTI_CFLIPV]);

      }
    } else { // composite->composite, ust update P_MULTI_C* if different from P_WOI_*
       SETFRAMEPARS_COND(P_MULTI_CHEIGHT, thispars->pars[P_WOI_HEIGHT]);
       SETFRAMEPARS_COND(P_MULTI_CWIDTH,  thispars->pars[P_WOI_WIDTH]);
       SETFRAMEPARS_COND(P_MULTI_CLEFT,   thispars->pars[P_WOI_LEFT]);
       SETFRAMEPARS_COND(P_MULTI_CTOP,    thispars->pars[P_WOI_TOP]);
       SETFRAMEPARS_COND(P_MULTI_CFLIPH,  thispars->pars[P_FLIPH]);
       SETFRAMEPARS_COND(P_MULTI_CFLIPV,  thispars->pars[P_FLIPV]);
    }
  }
  for (i=0;i<sizeof(wois)/sizeof(wois[0]);i++) {
//    if (wois[i] && (wois[i]!=thispars->pars[P_MULTI_WOI+i])) SETFRAMEPARS_SET(P_MULTI_WOI+i, wois[i]); no need to skip wois[i]==0
    SETFRAMEPARS_COND(P_MULTI_WOI+i, wois[i]);
  }
  SETFRAMEPARS_COND(P_MULTI_FLIPH, multi_fliph);
  SETFRAMEPARS_COND(P_MULTI_FLIPV, multi_flipv);
  MDF25(printk(" total_height=0x%x\n",total_height));
  SETFRAMEPARS_COND(P_SENSOR_HEIGHT, total_height);
  if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes, schedule functions

  if ( composite) multi_unitialized=1; // now mark as used - will enable copying from individual to composite flips

  return 0;
}

/** Calculates sensor phase from clock period, FPGA delay, cable delay and sensor delay
 */
int calcThisPhase(int clk_period,   ///< cklock period (Hz)
		          long FPGADelay,   ///< FPGA delay (ps?)
				  long cableDelay,  ///< cable delay (ps?)
				  long sensorDelay) ///< sensor delay (ps?)
				                    ///< @return   0x80000  | //(modified)
				                    ///< <90-degree-delay> << 16 |
				                    ///< <finedelay>
{
    MDF16(printk ("cableDelay1=%ld, FPGADelay1=%ld, clk_period=%d\r\n",cableDelay, FPGADelay, clk_period));
    int px_delay=-(clk_period/2 - FPGADelay- cableDelay - sensorDelay) ; // static int sensorDelay
    MDF16(printk ("px_delay1=%d\r\n",px_delay));
    int px_delay90=(4*px_delay+clk_period/2)/clk_period;
    px_delay -= (px_delay90*clk_period)/4; // -clk_period/8<= now px_delay <= +clk_period/8
    MDF16(printk ("px_delay=%d, px_delay90=%d\r\n",px_delay,px_delay90));
    px_delay/= FPGA_DCM_STEP; // in DCM steps
    return (px_delay & 0xffff) | ((px_delay90 & 3) <<16) | 0x80000;
}
/** Program sensor phase, programs 10359 SDRAM phase and all 3 sensor phases*/
int multisensor_pgm_sensorphase(int sensor_port,               ///< sensor port number (0..3)
								struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
								struct framepars_t * thispars, ///< sensor current parameters
								struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
								int frame16)                   ///< 4-bit (hardware) frame number parameters should
															   ///< be applied to,  negative - ASAP
															   ///< @return always 0
{
  struct frameparspair_t pars_to_update[6]; // ??? needed, increase if more entries will be added
  int nupdate=0;
  int reset=0;
  int resetThisDCM;
  int rslt=0;
  long * cableDelay;
  long * FPGADelay;
  int clk_period;
  MDF3(printk(" frame16=%d\n",frame16));
  long sdram_chen=thispars->pars[P_M10359_REGS+I2C359_SDRAM_CHEN];
  int adjustSDRAMNeed=0;
  int thisPhaseSDRAM=thispars->pars[P_MULTI_PHASE_SDRAM];
  int thisPhase1=    thispars->pars[P_MULTI_PHASE1];
  int thisPhase2=    thispars->pars[P_MULTI_PHASE2];
  int thisPhase3=    thispars->pars[P_MULTI_PHASE3];
  uint64_t ull_result = 1000000000000LL;
  if (frame16 >= 0) return -1; // can only work in ASAP mode
 //changed (just set) clock frequency initiates calculation of phase settings
  if (!multi_phases_initialized || (thispars->pars[P_CLK_SENSOR] != prevpars->pars[P_CLK_SENSOR]))  { // system clock is already set to the new frequency
    if (thispars->pars[P_CLK_SENSOR] == prevpars->pars[P_CLK_SENSOR]) {
     printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__); printk ("WARNING: ((thispars->pars[P_CLK_SENSOR] == prevpars->pars[P_CLK_SENSOR])) but multi_phases_initialized is not yet set (re-init?)\n");
    }
    multisensor_set_freq (sensor_port,  0, thispars); // not the first time. Assuming no clock chip if clock4 is not set
    reset=1;
    // TODO: put here calculation of the sensor phases in 10359 from bitstream data and clock rate
//    clk_period= 1000000000000.0f/thispars->pars[P_CLK_SENSOR];  // period in ps
    do_div(ull_result,thispars->pars[P_CLK_SENSOR]);
    clk_period= ull_result;
// Now for each of 3 sensor ports of the 10359
    cableDelay= (long *) &GLOBALPARS(sensor_port, G_DLY359_C1);
    FPGADelay=  (long *) &GLOBALPARS(sensor_port, G_DLY359_P1);
    thisPhase1=calcThisPhase(clk_period, FPGADelay[0], cableDelay[0], sensorproc_phys->sensor.sensorDelay) | 0x80000;
    MDF24(printk("cableDelay1=0x%lx FPGADelay1= 0x%lx sensorproc_phys->sensor.sensorDelay=0x%x, thisPhase1=0x%x\n",cableDelay[0] ,FPGADelay[0], sensorproc_phys->sensor.sensorDelay,thisPhase1));

    cableDelay= (long *) &GLOBALPARS(sensor_port, G_DLY359_C2);
    FPGADelay=  (long *) &GLOBALPARS(sensor_port, G_DLY359_P2);
    thisPhase2=calcThisPhase(clk_period, FPGADelay[0], cableDelay[0], sensorproc_phys->sensor.sensorDelay) | 0x80000;
    MDF24(printk("cableDelay2=0x%lx FPGADelay2= 0x%lx sensorproc_phys->sensor.sensorDelay=0x%x, thisPhase3=0x%x\n",cableDelay[0] ,FPGADelay[0], sensorproc_phys->sensor.sensorDelay,thisPhase2));

    cableDelay= (long *) &GLOBALPARS(sensor_port, G_DLY359_C3);
    FPGADelay=  (long *) &GLOBALPARS(sensor_port, G_DLY359_P3);
    thisPhase3=calcThisPhase(clk_period, FPGADelay[0], cableDelay[0], sensorproc_phys->sensor.sensorDelay) | 0x80000;
    MDF24(printk("cableDelay3=0x%lx FPGADelay3= 0x%lx sensorproc_phys->sensor.sensorDelay=0x%x, thisPhase3=0x%x\n",cableDelay[0] ,FPGADelay[0], sensorproc_phys->sensor.sensorDelay,thisPhase3));

// TODO: calculate SDRAM phase here too.
    adjustSDRAMNeed=1;
    multi_phases_initialized=1;
  }

  if (reset) {
    MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_DCM_SYSTEM,  I2C359_DCM_RESET | I2C359_DCM_RESET90);
    MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_DCM_SDRAM,   I2C359_DCM_RESET | I2C359_DCM_RESET90);
    multisensor_initSDRAM(sensor_port, thispars); // init 10359 SDRAM
  }
  if ((thisPhaseSDRAM != prevpars->pars[P_MULTI_PHASE_SDRAM]) ||  adjustSDRAMNeed)  {
    if (adjustSDRAMNeed || (thisPhaseSDRAM & 0x200000)) { // at boot, after frequency change or manually requested (0x200000)
      thisPhaseSDRAM=multisensor_adjustSDRAM (sensor_port, FPGA_DCM_RANGE);
      if (thisPhaseSDRAM>=0) {
         SETFRAMEPARS_SET(P_MULTI_PHASE_SDRAM,  thisPhaseSDRAM);
         printk("10359 SDRAM clock phase is set to %d/%s%d\n",
               90*((thisPhaseSDRAM>>16) & 3),
              (thisPhaseSDRAM & 0x8000)?"-":"+",
              (thisPhaseSDRAM & 0x8000)?(0x10000-(thisPhaseSDRAM & 0xffff)):(thisPhaseSDRAM & 0xffff));
      } else {
         printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);
         printk ("**** ERROR adjusting SDRAM clock phase in %s:%d:%s, result=0x%x\n",__FILE__,__LINE__,__FUNCTION__,thisPhaseSDRAM);
      }
    } else {
      resetThisDCM=reset || (thisPhaseSDRAM & 0x80000);
      rslt= multisensor_set_phase (sensor_port, I2C359_DCM_SDRAM, resetThisDCM, thisPhaseSDRAM, prevpars->pars[P_MULTI_PHASE_SDRAM]);
      if ((rslt>=0) && (rslt != thisPhaseSDRAM)) SETFRAMEPARS_SET(P_MULTI_PHASE_SDRAM,  rslt);
      if (resetThisDCM) {
         MDF16(printk("re-initializing SDRAM on 10359 after DCM reset\n"));
         multisensor_initSDRAM(sensor_port, thispars); // init 10359 SDRAM
      }
// Test memory phase here
      printk ("\nMULTI_PHASE_SDRAM=%01x %04x\n", (rslt>>16), rslt & 0xffff);
      if (thisPhaseSDRAM & 0x100000) {
        for (rslt=0;rslt<16;rslt++) {
          multisensor_memphase_debug(sensor_port, -1);
        }
      } else {
        multisensor_memphase_debug(sensor_port, 1);
#if 0
        printk ("\n");
        multisensor_memphase_debug(0);
        printk ("\n");
        multisensor_memphase_debug(0);
#endif
     }
   }
  }
  if (thisPhase1 != prevpars->pars[P_MULTI_PHASE1])  {
    resetThisDCM=reset || (thisPhase1 & 0x80000);
    rslt= multisensor_set_phase (sensor_port, I2C359_DCM_SENSOR1, resetThisDCM, thisPhase1, prevpars->pars[P_MULTI_PHASE1]);
    if ((rslt>=0) && (rslt != thisPhase1)) SETFRAMEPARS_SET(P_MULTI_PHASE1,  rslt);
  }

  if (thisPhase2 != prevpars->pars[P_MULTI_PHASE2])  {
    resetThisDCM=reset || (thisPhase2 & 0x80000);
    rslt= multisensor_set_phase (sensor_port, I2C359_DCM_SENSOR2, resetThisDCM, thisPhase2, prevpars->pars[P_MULTI_PHASE2]);
    if ((rslt>=0) && (rslt != thisPhase2)) SETFRAMEPARS_SET(P_MULTI_PHASE2,  rslt);
  }

  if (thisPhase3 != prevpars->pars[P_MULTI_PHASE3])  {
    resetThisDCM=reset || (thisPhase3 & 0x80000);
    rslt= multisensor_set_phase (sensor_port, I2C359_DCM_SENSOR3, resetThisDCM, thisPhase3, prevpars->pars[P_MULTI_PHASE3]);
    if ((rslt>=0) && (rslt != thisPhase3)) SETFRAMEPARS_SET(P_MULTI_PHASE3,  rslt);
  }
  MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CHEN,    sdram_chen); // Restore 10359 SDRAM channels enable
  if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes, schedule functions
  return 0;
}


/** Set 10359A clock frequency (take care of 10359-0) */
int multisensor_set_freq  (int sensor_port,                ///< sensor port number (0..3)
		                   int first,                      ///< was never programmed before
						   struct framepars_t * thispars)  ///< sensor current parameters
						                                   ///< @return >0 - (frequency), -1 - error (i.e. 10359-0).
						                                   ///< 0 - using system clock as on-board one is disabled by G_MULTI_CFG
{
  struct frameparspair_t pars_to_update[3];
  int nupdate=0;
  int rslt=0;
  int i;
  int was_sensor_freq=0;
  if (!(GLOBALPARS(sensor_port, G_MULTI_CFG) && (1<<G_MULTI_CFG_SYSCLK))) { // skip local clock if disabled in configuration
//    was_sensor_freq=getClockFreq(I2C359_CLK_NUMBER);
    was_sensor_freq=x393_getClockFreq(sensor_port, I2C359_CLK_NUMBER & 3); // clock 0
    if (first || (was_sensor_freq !=0)) { // Otherwise it is likely rev 0 - no clock
//      was_sensor_freq=getClockFreq(1);
      was_sensor_freq=90000000; // TODO: Find out how to read actual clock frequency for sensor ports
//      i=setClockFreq(I2C359_CLK_NUMBER, was_sensor_freq);
      i=x393_setClockFreq(sensor_port, I2C359_CLK_NUMBER & 3, was_sensor_freq);
      if (i>0) {
        MULTISENSOR_WRITE_I2C16_SHADOW(sensor_port, I2C359_CLKSRC, I2C359_CLKSRC_LOCAL);
        mdelay (50); // 0.05 sec to stabilize clocks - will miss multiple frames
      } else {
        was_sensor_freq=-1; // error
      }
    }
  }
  MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_DCM_SENSOR1, I2C359_DCM_RESET | I2C359_DCM_RESET90 | I2C359_DCM_HACT_RESET);
  MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_DCM_SENSOR2, I2C359_DCM_RESET | I2C359_DCM_RESET90 | I2C359_DCM_HACT_RESET);
  MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_DCM_SENSOR3, I2C359_DCM_RESET | I2C359_DCM_RESET90 | I2C359_DCM_HACT_RESET);
  if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to sensor register shadows
  return was_sensor_freq;
}

/** Set 10359A clock phase, verify DCM is OK */
int multisensor_set_phase_verify (int           sensor_port, ///< sensor port number (0..3)
		                          int           reg_addr,    ///< DCM register address (should have all the same control in 4 LSB)
		                          int           resetDCM,    ///< 1 - start from zero phase, reset DCM, 0 - only update from oldPhase
								  unsigned long newPhase,    ///< target phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
								  unsigned long oldPhase)    ///< previous phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
														     ///< @return >= 0 - OK (returns new combined phase),
														     ///< - -1 - no communication with 10359
														     ///< - -2 - unknown DCM number
														     ///< - -3 - Overflow (most too far adjusted, need DCM reset)
														     ///< - -4 - not DONE (why?)
														     ///< - -5 - not LOCKED
{
    int rslt=multisensor_set_phase (sensor_port, reg_addr, resetDCM, newPhase, oldPhase);
    unsigned long status=0;
    int channel=-1;
    if (rslt <0) return rslt;
    switch (reg_addr) {
    case I2C359_DCM_SDRAM:  channel=0;break;
    case I2C359_DCM_SENSOR1:channel=1;break;
    case I2C359_DCM_SENSOR2:channel=2;break;
    case I2C359_DCM_SENSOR3:channel=3;break;
    }
    if (channel<0)                    rslt= -I2C359_DCM_ERR_UNKNOWN;  // unknown DCM
#ifdef NC353
    multisensor_read_i2c(I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status, 2);
#else
    X3X3_I2C_RCV2(sensor_port,I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status);
#endif
    if ( I2C359_DCM_OFL(0,status))    rslt= -I2C359_DCM_ERR_OVFL;     // overflow
    if (!I2C359_DCM_DONE(0,status))   rslt= -I2C359_DCM_ERR_NODONE;   // not DONE - seems to bee too far - reset and put maximum (by abs value)
    if (!I2C359_DCM_LOCKED(0,status)) rslt= -I2C359_DCM_ERR_NOLOCKED; // not LOCKED
    return rslt;
}

/** Set 10359A clock phase, try to recover from overflows */
int multisensor_set_phase_recover(int           sensor_port, ///< sensor port number (0..3)
								  int           reg_addr,    ///< DCM register address (should have all the same control in 4 LSB)
								  int           resetDCM,    ///< 1 - start from zero phase, reset DCM, 0 - only update from oldPhase
								  unsigned long newPhase,    ///< target phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
								  unsigned long oldPhase)    ///< previous phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
														     ///< @return >= 0 - OK (returns new combined phase),
														     ///< - -1 - no communication with 10359
														     ///< - -2 - unknown DCM number
														     ///< - -3 - Overflow (most too far adjusted, need DCM reset)
														     ///< - -4 - not DONE (why?)
														     ///< - -5 - not LOCKED
{
    int rslt=multisensor_set_phase (sensor_port, reg_addr, resetDCM, newPhase, oldPhase);
    unsigned long status=0;
    int channel=-1;
    if (rslt <0) return rslt;
    switch (reg_addr) {
    case I2C359_DCM_SDRAM:  channel=0;break;
    case I2C359_DCM_SENSOR1:channel=1;break;
    case I2C359_DCM_SENSOR2:channel=2;break;
    case I2C359_DCM_SENSOR3:channel=3;break;
    }
    if (channel<0)                    rslt= -I2C359_DCM_ERR_UNKNOWN;  // unknown DCM
#ifdef NC353
    multisensor_read_i2c     (I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status, 2);
#else
    X3X3_I2C_RCV2(sensor_port,I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status);
#endif

    if ( I2C359_DCM_OFL(0,status))    rslt= -I2C359_DCM_ERR_OVFL;     // overflow
    if (!I2C359_DCM_DONE(0,status))   rslt= -I2C359_DCM_ERR_NODONE;   // not DONE - seems to bee too far - reset and put maximum (by abs value)
    if (!I2C359_DCM_LOCKED(0,status)) rslt= -I2C359_DCM_ERR_NOLOCKED; // not LOCKED
    if ((rslt == -I2C359_DCM_ERR_NODONE) || (rslt== -I2C359_DCM_ERR_OVFL)) { // Not Done - DCM overflow, try reducing the phase (will need reset)
        unsigned long goodPhase,badPhase;
        short * newPhaseShort=  (short *) &newPhase;
        short * goodPhaseShort= (short *) &goodPhase;
        short * badPhaseShort=  (short *) &badPhase;
        badPhase=newPhase;
        goodPhase = badPhase & 0x30000; // only phase90 is preserved
        while ((badPhaseShort[0]< (goodPhaseShort[0]-1)) || (badPhaseShort[0] > (goodPhaseShort[0]+1))) {
            newPhaseShort[0]=(badPhaseShort[0]+goodPhaseShort[0])/2;
            rslt=multisensor_set_phase (sensor_port, reg_addr, (rslt<0), newPhase, goodPhase);
            if (rslt<0) return rslt;
            // check DCM status
#ifdef NC353
            multisensor_read_i2c(sensor_port, I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status, 2);
#else
            X3X3_I2C_RCV2       (sensor_port, I2C359_SLAVEADDR, I2C359_DCM_STATUS, &status);
#endif
            if ( I2C359_DCM_OFL(0,status))    rslt= -I2C359_DCM_ERR_OVFL;     // overflow
            if (!I2C359_DCM_DONE(0,status))   rslt= -I2C359_DCM_ERR_NODONE;   // not DONE - seems to bee too far - reset and put maximum (by abs value)
            if (!I2C359_DCM_LOCKED(0,status)) rslt= -I2C359_DCM_ERR_NOLOCKED; // not LOCKED
            if ((rslt<0) && (rslt != -I2C359_DCM_ERR_NODONE) && (rslt != -I2C359_DCM_ERR_OVFL)) return rslt; // Other errors - consider fatal;
            if (rslt<0) badPhase=   newPhase;
            else        goodPhase = newPhase;
        }
        rslt=goodPhase;
        MDF24(printk("Reduced NewPhase, goodPhase= 0x%lx oldPhase= 0x%lx\r\n", goodPhase, oldPhase ));
    }
    return rslt;
}



/** Set 10359A clock phase */
int multisensor_set_phase (int           sensor_port, ///< sensor port number (0..3)
						   int           reg_addr,    ///< DCM register address (should have all the same control in 4 LSB)
						   int           resetDCM,    ///< 1 - start from zero phase, reset DCM, 0 - only update from oldPhase
						   unsigned long newPhase,    ///< target phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
						   unsigned long oldPhase)    ///< previous phase (low 16 bits - signed steps, bits [17:16] - 90-degree phase
													  ///< @return >= 0 - OK (returns new combined phase),
													  ///< - -1 - no communication with 10359
{
  int old_phase;
  int old_phase90;
  int phase;
  int phase90;
  int diff_phase;
  int diff_phase90;
  int rslt=0;
  int i;
  MDF24(printk("reg_addr=0x%x resetDCM=%d newPhase90=0x%lx newPhase= 0x%lx oldPhase= 0x%lx\r\n",\
                reg_addr, resetDCM, newPhase>>16, newPhase & 0xffff, oldPhase ));
  if (resetDCM || (newPhase & 0x8000)) {
    MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_RESET | I2C359_DCM_RESET90);
    oldPhase=0;
  }
  if (newPhase!=oldPhase) {
   MDF16(printk("newPhase= 0x%lx oldPhase= 0x%lx\r\n", newPhase, oldPhase ));
     old_phase= oldPhase & 0xffff;  if (old_phase>=0x8000) old_phase-=0x10000; // make it signed
     old_phase90= (oldPhase >> 16) & 3;
     if ((old_phase > 255) || (old_phase < -255)) {
       MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_RESET | I2C359_DCM_RESET90);
       old_phase=   0;
       old_phase90= 0;
     }
     phase=   newPhase & 0xffff;    if (phase>=    0x8000) phase-=    0x10000; // make it signed
     phase90=(newPhase >> 16) & 3;
     if      (phase >  255) phase=  255;
     else if (phase < -255) phase= -255;
     diff_phase=  phase-  old_phase;
     diff_phase90=phase90-old_phase90;
     if (diff_phase90>2) diff_phase90-=4;
     else if (diff_phase90<-1)diff_phase90+=4;
   MDF16(printk("old_phase= 0x%x old_phase90= 0x%x\r\n", old_phase, old_phase90 ));
   MDF16(printk("phase=     0x%x phase90=     0x%x\r\n", phase, phase90 ));
   MDF16(printk("diff_phase=0x%x diff_phase90=0x%x\r\n", diff_phase, diff_phase90 ));
     if      (diff_phase > 0) for (i=diff_phase; i > 0; i--)      MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_INC   )
     else if (diff_phase < 0) for (i=diff_phase; i < 0; i++)      MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_DEC   )
     if      (diff_phase90 > 0) for (i=diff_phase90; i > 0; i--)  MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_INC90 )
     else if (diff_phase90 < 0) for (i=diff_phase90; i < 0; i++)  MULTISENSOR_WRITE_I2C16(sensor_port, reg_addr, I2C359_DCM_DEC90 )
    if (rslt) return -1;
    return (phase & 0xffff) | (phase90 << 16);
  }
  return 0;
}


/** Init SDRAM in 10359 board */
 int multisensor_initSDRAM(int                 sensor_port, ///< sensor_port Sensor port (0..3)
		                  struct framepars_t * thispars)    ///< sensor current parameters
		                                                    ///< @return 0 - OK, -1 - error (i.e. 10359-0
 {
    struct frameparspair_t pars_to_update[1];
    int nupdate=0;
    int rslt=0; // or-ed by MULTISENSOR_WRITE_I2C(sa,ra,v,sz)
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_CHEN,    0x00015555); // Disable refresh, disable all channels
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00017FFF); // precharge, a[10]=1 - all banks
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00002002); // load extended mode register - enable DLL, reduced drive strength(2)
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00000163); // load mode register - CL=2.5, burst=8 (no full fage available), reset DLL
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00008000); // refresh
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00008000); // refresh
    MULTISENSOR_WRITE_I2C32       (sensor_port, I2C359_SDRAM_MANCMD,  0x00017FFF); // precharge, a[10]=1 - all banks
    MULTISENSOR_WRITE_I2C32_SHADOW(sensor_port, I2C359_SDRAM_CHEN,    0x00020000); // Enable refresh, do nothing with the channels
    if (nupdate)  setFramePars(sensor_port,thispars, nupdate, pars_to_update);  // save changes to shadows
    return rslt;
 }


/** Adjust SDRAM clock phase */
int  multisensor_adjustSDRAM (int sensor_port, ///< sensor_port Sensor port (0..3)
		                      int maxAdjust)   ///< maximal DCM phase adjust (make it 250)
		                                       ///< @return >=0 - phase to be set (as phase90<<16 | phase), -1 - error failed to adjust
{
  int centroids90[4];
  int results90 [4];
  int i;
  int oldPhase=0;
  oldPhase=  multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, 1, 0, oldPhase); // reset SDRAM phase
  if (oldPhase<0) return oldPhase; // failed to reset
  int needReset=0;
  int ok90=0;
  int oks90=0;
  int low90=-1;
  int high90=-1;
  int low_l, low_h, high_l,high_h;
  for (i=0; i<4; i++) {
    oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, i<<16, oldPhase); // do not reset SDRAM phase - no fine tuning
    if (oldPhase<0) return oldPhase; // any error is fatal here
    results90 [i]=multisensor_memphase (sensor_port,&centroids90[i]);
    if (results90 [i]==0) {
      ok90|=(1<<i);
      oks90++;
    }
  }
   switch (ok90) {
// 1 OK
     case  1: low90=0;high90=0;break;
     case  2: low90=1;high90=1;break;
     case  4: low90=2;high90=2;break;
     case  8: low90=3;high90=3;break;
 // 2 OKs one after another
     case  3: low90=0;high90=1;break;
     case  6: low90=1;high90=2;break;
     case 12: low90=2;high90=3;break;
     case  9: low90=3;high90=0;break;
// No OK phases without DCM phase adjustments - look --> plus crossing
     case 0:
     for (i=0;i<4;i++) {
       if ((results90[i]<0) && (results90[(i+1)&3]>0)) {
         low90=i;high90=(i+1)&3;
         if ((centroids90[low90]+centroids90[high90])>0) high90=low90; // result of low is better
         else                                            low90= high90;// result of high is better
         break;
       }
     }
     if (low90 < 0){
         printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);
         printk ("**** ERROR adjusting SDRAM clock phase in %s:%d:%s\n",__FILE__,__LINE__,__FUNCTION__);
         printk ("oks90=%d, ok90=0x%x, centroids90=0x%x  0x%x 0x%x 0x%x\n",oks90,ok90,centroids90[0],centroids90[1],centroids90[2],centroids90[3]);
         return -1;
     }
     break;
// was: fall through to default branch
     default:
       printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);
       printk ("**** ERROR adjusting SDRAM clock phase in %s:%d:%s\n",__FILE__,__LINE__,__FUNCTION__);
       printk ("oks90=%d, ok90=0x%x, centroids90=0x%x  0x%x 0x%x 0x%x\n",oks90,ok90,centroids90[0],centroids90[1],centroids90[2],centroids90[3]);
       return -1;
  }
// now find low margin
  if (results90 [low90]) { // was bad
    low_l= 0;
    low_h= maxAdjust;
  } else {  // was good
    low_l= -maxAdjust;
    low_h= 0;
  }
/*#define I2C359_DCM_ERR_UNKNOWN  2
  #define I2C359_DCM_ERR_OVFL     3
  #define I2C359_DCM_ERR_NODONE   4
  #define I2C359_DCM_ERR_NOLOCKED 5*/
  oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, 1, (low90<<16), oldPhase); // reset dcm, set 90-degree phase
  if (oldPhase<0) return oldPhase;  // any error is fatal here - fine phase is 0
  needReset=0;
  while ((low_h-low_l)>1) {
    i=(low_l+low_h)/2;
    oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, (low90<<16) | (i & 0xffff), oldPhase); // try middle phase, no DCM reset
    if (oldPhase<0)  {
  MDF24 (printk(" DCM error=%d\n",-oldPhase));
      needReset=1;
      if ((oldPhase!=-I2C359_DCM_ERR_OVFL) && (oldPhase!=-I2C359_DCM_ERR_NODONE))  return oldPhase; // other errors fatal
      else                                low_l=i; // bad
    } else {
      needReset=0;
      if (multisensor_memphase (sensor_port,NULL)==0) low_h=i; // good
      else                                            low_l=i; // bad
   }
  }
// now find high margin
  if (results90 [high90]) { // was bad
    high_l= -maxAdjust;
    high_h= 0;
  } else { // was good
    high_l= 0;
    high_h= maxAdjust;
  }
  oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, 1, (high90<<16), oldPhase); // reset dcm, set 90-degree phase
  if (oldPhase<0) return oldPhase;  // any error is fatal here - fine phase is 0
  needReset=0;
  while ((high_h-high_l)>1) {
    i=(high_h+high_l)/2;
    oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, (high90<<16) | (i & 0xffff), oldPhase); // try middle phase, no DCM reset
    if (oldPhase<0)  {
  MDF24 (printk(" DCM error=%d\n",-oldPhase));
      needReset=1;
      if ((oldPhase!=-I2C359_DCM_ERR_OVFL) && (oldPhase!=-I2C359_DCM_ERR_NODONE))  return oldPhase; // other errors fatal
      else                                high_h=i; // bad
    } else {
      needReset=0;
      if (multisensor_memphase (sensor_port,NULL)==0) high_l=i; // good
      else                                            high_h=i; // bad
    }
  }
  MDF24 (printk(" low90=%d, low=%d, high90=%d, high=%d\n",low90,low_h,high90,high_l));
  if (high90==low90) { // 0,1 OK phases
    if (high_l>low_h) { // 0,1 OK phases
      i= (high90<<16) | (((high_l+low_h)>>1) & 0xffff);
      oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, i, oldPhase); // middle phase, same 90-degree
      if (oldPhase<0) return oldPhase;
// Verify that final phase is OK
      if (multisensor_memphase (sensor_port,NULL)==0)      return i;
       printk ("**** ERROR adjusting SDRAM clock phase (bad between two good points) in %s:%d:%s\n",__FILE__,__LINE__,__FUNCTION__);
       printk ("low90=%d, low_l=%d, high90=%d, high_l=%d, i=0x%x\n",low90, low_l, high90, high_l,i);
       return -10;
    } else {
       printk ("**** ERROR adjusting SDRAM clock phase in %s:%d:%s\n",__FILE__,__LINE__,__FUNCTION__);
       printk ("low90=%d, low_l=%d, high90=%d, high_l=%d\n",low90, low_l, high90, high_l);
       return -1;
    }
  }
// There were two good quarter phases (high90!=low90)
// two solution: 1 - put 45 calculate phase knowing the period and DCM step or
//               2 - find which of the two is closer to the middle, re-measuere one of the margins.
// Second solution is better - may fail on the lowest frequencies?
  if ((low_h+high_l)>0) { // low90 is closer to the margin - start from high90,. recalculate fine adjustment low_h
    low90=high90;
    low_l= -maxAdjust;
    low_h= 0;
    oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, 1, (low90<<16), oldPhase); // reset dcm, set 90-degree phase
    if (oldPhase<0) return oldPhase;  // any error is fatal here - fine phase is 0
    needReset=0;
    while ((low_h-low_l)>1) {
      i=(low_l+low_h)/2;
      oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, (low90<<16) | (i & 0xffff), oldPhase); // try middle phase, no DCM reset
      if (oldPhase<0)  {
  MDF24 (printk(" DCM error=%d\n",-oldPhase));
        needReset=1;
        if ((oldPhase!=-I2C359_DCM_ERR_OVFL) && (oldPhase!=-I2C359_DCM_ERR_NODONE)) return oldPhase; // other errors fatal
        else                                low_l=i; // bad
      } else {
        needReset=0;
        if (multisensor_memphase (sensor_port, NULL)==0) low_h=i; // good
        else                                             low_l=i; // bad
      }
    }
  } else { // high90 is closer to the margin - start from low90,. recalculate fine adjustment high_l
    high90=low90;
    high_l= 0;
    high_h= maxAdjust;
    oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, 1, (high90<<16), oldPhase); // reset dcm, set 90-degree phase
    if (oldPhase<0) return oldPhase;  // any error is fatal here - fine phase is 0
    needReset=0;
    while ((high_h-high_l)>1) {
      i=(high_h+high_l)/2;
      oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, (high90<<16) | (i & 0xffff), oldPhase); // try middle phase, no DCM reset
      if (oldPhase<0)  {
  MDF24 (printk(" DCM error=%d\n",-oldPhase));
        needReset=1;
        if ((oldPhase!=-I2C359_DCM_ERR_OVFL) && (oldPhase!=-I2C359_DCM_ERR_NODONE)) return oldPhase; // other errors fatal
        else                                high_h=i; // bad
      } else {
        needReset=0;
        if (multisensor_memphase (sensor_port, NULL)==0) high_l=i; // good
        else                                             high_h=i; // bad
      }
    }
  }
  MDF24 (printk("Re-measured to the same 90-degree phase low90=%d, low=%d, high90=%d, high=%d\n",low90,low_h,high90,high_l));
  if (high_l>low_h) { // 0,1 OK phases
      i= (high90<<16) | (((high_l+low_h)>>1) & 0xffff);
      oldPhase= multisensor_set_phase_verify (sensor_port, I2C359_DCM_SDRAM, needReset, i, oldPhase); // middle phase, same 90-degree
      return oldPhase; // (both >=0 or error (<0)
  } else { // something strange - should not get here
       printk("%s:%d:%s ",__FILE__,__LINE__,__FUNCTION__);
       printk ("**** BUG - should not get here (there were two good 90-degree phases, now none)\n");
       printk ("ERROR adjusting SDRAM clock phase in %s:%d:%s\n",__FILE__,__LINE__,__FUNCTION__);
       printk ("low90=%d, low_l=%d, high90=%d, high_l=%d\n",low90, low_l, high90, high_l);
       return -1;
  }
}
/** Measure SDRAM on 10359 board phase? */
int multisensor_memphase (int sensor_port,          ///< Sensor port
		                  int * centroid0x10000)    ///< result?
		                                            ///< @return 0 if OK,  if !OK but n=0 - return n=1 (1/0x10000, actually)
{
    unsigned long data[64];
    int setbits [8];
    int i,n,d;
    int s=0;
    int sx=0;
    int rslt=0;
    MULTISENSOR_WRITE_I2C32(sensor_port, I2C359_SDRAM_CHEN,  I2C359_SDRAM_STOP(4) | I2C359_SDRAM_STOP(5)); // initialize write and read channels, reset SDRAM and buffer addresses
    MULTISENSOR_WRITE_I2C32(sensor_port, I2C359_SDRAM_CHEN,  I2C359_SDRAM_RUN(4)  | I2C359_SDRAM_RUN(5));  // enable write and read channels
    for (i=0; i<64;i++) {
        MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_DATA, (((i&7)==3) || ((i&7)==4) || ((i&7)==5))?0xffff:0); // pattern of 5 zeores, 3 ffff-s
    }
    MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_PAGE_WR, 0);                  // start page write
    MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_PAGE_RD, 0);                  // start page read (expecting i2c to be much slower than page wr/rd
    for (i=0; i<64;i++) {
#ifdef NC353
        multisensor_read_i2c     (I2C359_SLAVEADDR, I2C359_SDRAM_DATA, &data[i], 2);
#else
        X3X3_I2C_RCV2(sensor_port,I2C359_SLAVEADDR, I2C359_SDRAM_DATA, &data[i]);
#endif

    }
    for (i=0;i<8;i++) setbits[i]=0;
    for (n=0;n<64;n++) {
        d=data[n];
        for (i=0;i<16;i++) {
            if (d & 1) {
                setbits[n & 7]++;
                sx+=(n & 7)-4;
                s++;
            }
            d>>=1;
        }
    }
    int OK=(setbits[0]==0) && (setbits[1]==0) && (setbits[2]==0) && (setbits[3]==0x80) && (setbits[4]==0x80) && (setbits[5]==0x80) && (setbits[6]==0) && (setbits[7]==0);
    //    for (i=0; i<8;i++)    printk (" %03x ",setbits[i]); printk("\n");
    n=(0x10000*sx)/s;
    if (centroid0x10000) centroid0x10000[0]=n;
    MDF24 (printk("centroid=0x%x, OK=%d\n",n,OK));
    return OK?0:(n?n:1); // so if !OK but n=0 - return n=1 (1/0x10000, actually)
}

/** Measure SDRAM on 10359 board phase in debug/verbose mode? */
int multisensor_memphase_debug (int sensor_port, ///< Sesnor port number (0..3)
		                        int write)       ///< Enable memory write when testing (0 - read only mode)
		                                         ///< @return centroid
{
    unsigned long data[64];
    int setbits [8];
    int i,n,d;
    int s=0;
    int sx=0;
    int rslt=0;
    if (write >=0) {
        MULTISENSOR_WRITE_I2C32(sensor_port, I2C359_SDRAM_CHEN,  I2C359_SDRAM_STOP(4) | I2C359_SDRAM_STOP(5)); // initialize write and read channels, reset SDRAM and buffer addresses
        MULTISENSOR_WRITE_I2C32(sensor_port, I2C359_SDRAM_CHEN,  I2C359_SDRAM_RUN(4)  | I2C359_SDRAM_RUN(5));  // enable write and read channels
        if (write) {
            for (i=0; i<64;i++) {
                MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_DATA, (((i&7)==3) || ((i&7)==4) || ((i&7)==5))?0xffff:0); // pattern of 5 zeores, 3 ffff-s
            }
            MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_PAGE_WR, 0);                  // start page write
        }
    }
    MULTISENSOR_WRITE_I2C16(sensor_port, I2C359_SDRAM_PAGE_RD, 0);                  // start page read (expecting i2c to be much slower than page wr/rd
    for (i=0; i<64;i++) {
#ifdef NC353
        multisensor_read_i2c     (I2C359_SLAVEADDR, I2C359_SDRAM_DATA, &data[i], 2);
#else
        X3X3_I2C_RCV2(sensor_port,I2C359_SLAVEADDR, I2C359_SDRAM_DATA, &data[i]);
#endif

    }
    for (i=0; i<8;i++)    printk (" %02x  ",i); printk("\n");
    for (n=0;n<64;n+=8) {
        for (i=0; i<8;i++)  printk ("%04lx ", data[n+i]); printk("\n");
    }
    for (i=0;i<8;i++) setbits[i]=0;
    for (n=0;n<64;n++) {
        d=data[n];
        for (i=0;i<16;i++) {
            if (d & 1) {
                setbits[n & 7]++;
                sx+=(n & 7)-4;
                s++;
            }
            d>>=1;
        }
    }
    int OK=(setbits[0]==0) && (setbits[1]==0) && (setbits[2]==0) && (setbits[3]==0x80) && (setbits[4]==0x80) && (setbits[5]==0x80) && (setbits[6]==0) && (setbits[7]==0);
    for (i=0; i<8;i++)    printk (" %03x ",setbits[i]); printk("\n");
    n=(0x10000*sx)/s;
    printk ("Centroid - 0x%x, sx=0x%x, s=0x%x. OK=%d\n",n, sx,s,OK);
    return n;
}

/** Program sensor registers (probably just those that are manually set).
 *
 * NOTE: all modes but ASAP are limited to 64 registers/frame, no overflow checks are performed! */

int multisensor_pgm_sensorregs (int sensor_port,               ///< sensor port number (0..3)
								struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
								struct framepars_t * thispars, ///< sensor current parameters
								struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
								int frame16)                   ///< 4-bit (hardware) frame number parameters should
															   ///< be applied to,  negative - ASAP
															   ///< @return always 0

{
  MDF4(printk(" frame16=%d\n",frame16));
  if (frame16 >= PARS_FRAMES) return -1; // wrong frame
//  int fpga_addr=(frame16 <0) ? X313_I2C_ASAP : (X313_I2C_FRAME0+frame16);
  int fpga_addr=frame16;
// do whatever is needed for 10359 registers, then call sesnor function (if available)
//            rslt=sensorproc_phys->pgm_func[i+32] ( &(sensorproc->sensor), procpars, prevpars, seq_frame);
//  add_sensor_proc(onchange_sensorregs,  &mt9x001_pgm_sensorregs);   // write sensor registers (only changed from outside the driver as they may have different latencies)?
//#define SET_10359_REG(f,r,v) 
//  send all parameters marked as "needed to be processed" to the sensor, clear those flags
// mask out all non sensor pars
// It will be the first for the frame (before automatic sensor changes).
// Add testing for programmed sensor and move vbalues to later frames (not here but in the pgm_functions)
  unsigned long bmask32= ((thispars->mod32) >> (P_M10359_REGS>>5)) & (( 1 << (P_M10359_NUMREGS >> 5))-1) ;
  MDF4(printk(" bmask32=0x%lx, thispars->mod32=0x%lx, P_M10359_REGS=0x%x, P_M10359_NUMREGS=0x%x\n",bmask32,thispars->mod32,P_M10359_REGS,P_M10359_NUMREGS));
  unsigned long mask;
  int index,index32;
  if (bmask32) {
    for (index32=(P_M10359_REGS>>5); bmask32; index32++, bmask32 >>= 1) {
       MDF4(printk(" index32=0x%x, bmask32=0x%lx\n",index32,bmask32));
       if (bmask32 & 1) {
         mask=thispars->mod[index32];
         MDF4(printk(" mask=0x%lx\n",mask));
         for (index=(index32<<5); mask; index++, mask >>= 1) {
           if (mask & 1) {
             SET_10359_REG (sensor_port, fpga_addr, (index-P_M10359_REGS), thispars->pars[index]);
           }
         }
         thispars->mod[index32]=0;
       }
    }
    thispars->mod32=0;
  }
// Now proceed with physical sensor(s) i2c registers - both broadcast and individual
  if (sensorproc_phys->pgm_func[onchange_sensorregs+32]) return sensorproc_phys->pgm_func[onchange_sensorregs+32] (sensor_port, sensor, thispars, prevpars, frame16);
  return 0; // physical sensor function does not exist
} 


