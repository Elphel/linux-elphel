/***************************************************************************//**
* @file      pgm_functions.h
* @brief     Sensor/FPGA programming functions, called from  IRQ/tasklet in
*            response to the parameter changes
* @copyright Copyright 2008-2016 (C) Elphel, Inc.
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

#ifndef PGM_FUNCTIONS_H
#define PGM_FUNCTIONS_H

#include "sensor_i2c.h"
#include "detect_sensors.h"

#define COLOR_MARGINS 2 // add this many pixels each side
#define X313_TIMESTAMPLEN 28 // pixels used for timestamp (in linescan mode added after the line)
#define X393_TILEHOR 16
#define X393_TILEVERT 16
#define X393_MAXWIDTH       65536 // 4096 // multiple of 128
#define X393_MAXHEIGHT      65536 // 16384 // multiple of 16 - unsafe - not enough room for black level subtraction
#define X393_MAXHEIGHT_SAFE 65536 // 4096 // multiple of 16  OK for black level subtraction TODO: disable black level if unsafe

void pgm_functions_set_device(struct device *dev);
int init_pgm_proc(int sensor_port);
int add_sensor_proc(int port, int index, int (*sens_func)(int sensor_port, struct sensor_t * ,  struct framepars_t * , struct framepars_t *, int ));
unsigned long camsync_to_sensor(unsigned long camsync_time, unsigned long sensor_clk);
unsigned long sensor_to_camsync(unsigned long pixel_time, unsigned long sensor_clk);

/// Commands through sequencer: switch between ASAP (frame <0) and absolute
/// @param port - sensor port (0..3)
/// @param frame - <0 for ASAP command, otherwise absolute frame number to program for 4 LSB only are used)
/// @param func - part of the command write through sequencer w/o 'seqr_/seqa_ prefix
/// @param data - appropriate data type (matching function definition) to be written
#define X393_SEQ_SEND1(port,frame,func,data) {if ((frame) < 0) seqr_##func (0,       (data), (port)); \
                                              else             seqa_##func ((frame), (data), (port)); }
#define X393_SEQ_SEND1S(port,frame,func,data,subchn) {if ((frame) < 0) seqr_##func (0,       (data), (port), (subchn)); \
                                                      else             seqa_##func ((frame), (data), (port), (subchn)); }

/** Tells if parameter is modified
 * @param x parameter index to test
 * @return nonzero if modified */
#define FRAMEPAR_MODIFIED(x) (thispars->mod[(x) >> 5] & (1<<((x) & 0x1f)))
/** Adds new parameter/value pair to the modification queue
 * @param p parameter index
 * @param v parameter value(32 bits) */
#define SETFRAMEPARS_SET(p,v)        { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=(v);}
/** Puts current parameter value  into the modification queue
 * @param p parameter index (all high bits/attributes will be removed, only 12 LSBs preserved)*/
#define SETFRAMEPARS_UPDATE(p)       { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=thispars->pars[(p) & 0xffff];}
/** Puts new (or current) parameter value  into the modification queue
 * Uses specified value only if the current one is zero (not initialized already), otherwise updates with current
 * @param p parameter index (all high bits/attributes will be removed, only 12 LSBs preserved)
 * @param v parameter value(32 bits) */
#define SETFRAMEPARS_UPDATE_SET(p,v) { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=thispars->pars[(p) & 0xffff]?thispars->pars[(p) & 0xffff]:(v);}
/** Adds new parameter/value pair to the modification queue only if it is different from tghe current
 * @param p parameter index (all high bits/attributes will be removed, only 12 LSBs preserved)
 * @param v parameter value(32 bits) */
#define SETFRAMEPARS_COND(p,v)       { if (unlikely((v)!=thispars->pars[(p) & 0xffff])) { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=(v);} }
//#define SETFRAMEPARS_COND(p,v)       { if (unlikely((v)!=thispars->pars[p])) { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=(v);} }

/*
struct sensor_port_config_t {
	u32  mux;                                    ///< Sensor multiplexer, currently 0 (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
    u32  sensor[MAX_SENSORS];                    ///< Without mux only [0] is used, with 10359 - 0..2 are used (i2c addressing is shifted so 0 is broadcast)
    u16  par2addr[MAX_SENSORS][MAX_SENSOR_REGS]; ///< Big LUT. SENSOR_REGSxxx par to sensor reg 'yyy' internal address: haddr+laddr for 16 bit
    u8   haddr2rec[MAX_SENSORS][MAX_FPGA_RECS];  ///< Big LUT (but almost empty). Sensor's page address (haddr of reg addr) to fpga i2c record number (fpga line#)
    unsigned short *pages_ptr[MAX_SENSORS];
};

extern struct sensor_port_config_t *pSensorPortConfig;
*/

/**Set parameter for the sensor register and send to hardware i2c sequencer
 * @param port Sensor port number
 * @param frame Frame number to apply, <0 - ASAP
 * @param sa7 I2C slave address, 7 bit
 * @param reg sensor register address (8-bit)
 * @param data value to set (16 bits) */
#define SET_SENSOR_PAR(port,frame,sa7,reg,data) { pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
                                                  pars_to_update[nupdate++].val=(data);\
                                                  X3X3_I2C_SEND2((port),(frame), (sa7), (reg), (data));\
                                                }


/**
 * Set parameter for the sensor register and send to hardware i2c sequencer
 * @param port  sensor port number
 * @param frame frame number to apply, <0 - ASAP
 * @param reg   sensor register address (8-bit)
 * @param data  value to set (16 bits)
 */
#define SET_SENSOR_PAR_LUT(port,frame,reg,data) {\
	int _I = pSensorPortConfig[(port)].broadcast_addr;\
	int _ADDR = pSensorPortConfig[(port)].par2addr[_I][(reg)];\
	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg);\
    pars_to_update[nupdate++].val=(data);\
    if (!(_ADDR&0xffff0000)) {\
    	X3X3_I2C_SEND2_LUT((port),(frame), _I, _ADDR, (data));\
    }\
}


/**Set parameter for the same register in multiple multiplexed sensors and send to hardware i2c sequencer
 * Similar to SET_SENSOR_PAR, but broadcast set for parameters with individual values.
 * Updates both "parent" parameter (used without multiplexer) and the individual ones
 * (can use 4 elements in pars_to_update array, increase size where needed!).
 * Relies on the fact that individual parameters are processed later, so it verifies that broadcast
 * does not modify individual if they are also modified and scheduled to be applied.
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 * @see SET_SENSOR_PAR */
#define SET_SENSOR_MBPAR(p,f,s,r,v)  { pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
                                       pars_to_update[nupdate++].val=(v);\
                                       X3X3_I2C_SEND2((p), (f), (s), (r), (v)); \
                                       {int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),0); \
                                         if (_MINDEX) { \
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                           if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                             pars_to_update[nupdate  ].num= _MINDEX ;\
                                             pars_to_update[nupdate++].val=(v);\
                                           } \
                                           _MINDEX++ ;\
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                           if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                             pars_to_update[nupdate  ].num= _MINDEX ;\
                                             pars_to_update[nupdate++].val=(v);\
                                           } \
                                           _MINDEX++ ;\
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                           if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                             pars_to_update[nupdate  ].num= _MINDEX ;\
                                             pars_to_update[nupdate++].val=(v);\
                                           } \
                                         } \
                                       } \
                                     }

/**Set parameter for the same register in multiple multiplexed sensors and send to hardware i2c sequencer
 * Similar to SET_SENSOR_PAR_LUT, but broadcast set for parameters with individual values.
 * Updates both "parent" parameter (used without multiplexer) and the individual ones
 * (can use 4 elements in pars_to_update array, increase size where needed!).
 * Relies on the fact that individual parameters are processed later, so it verifies that broadcast
 * does not modify individual if they are also modified and scheduled to be applied.
 * @param p - sensor port number
 * @param f - frame number to apply, <0 - ASAP
 * @param r - sensor register address (8-bit)
 * @param v - value to set (16 bits)
 * @see SET_SENSOR_PAR_LUT */
#define SET_SENSOR_MBPAR_LUT(p,f,r,v)  {\
	int _I = pSensorPortConfig[(p)].broadcast_addr;\
	int _ADDR = pSensorPortConfig[(p)].par2addr[_I][(r)];\
    if (!(_ADDR&0xffff0000)) {\
        X3X3_I2C_SEND2_LUT((p), (f), _I, _ADDR, (v));\
    }\
    pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
    pars_to_update[nupdate++].val=(v);\
    {\
	    int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),0); \
	    if (_MINDEX) { \
	  	    EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
	  	    		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
	  	    if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
			    pars_to_update[nupdate  ].num= _MINDEX;\
			    pars_to_update[nupdate++].val=(v);\
		    }\
		    _MINDEX++;\
		    EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
		    		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
		    if (!FRAMEPAR_MODIFIED(_MINDEX)) {\
			    pars_to_update[nupdate  ].num= _MINDEX;\
			    pars_to_update[nupdate++].val=(v);\
		    }\
		    _MINDEX++;\
		    EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
		    		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
		    if (!FRAMEPAR_MODIFIED(_MINDEX)) {\
			    pars_to_update[nupdate  ].num= _MINDEX;\
			    pars_to_update[nupdate++].val=(v);\
		    }\
	    }\
    }\
}

/**Set parameter for the same register in multiple multiplexed sensors and send to hardware i2c sequencer
 * Similar to SET_SENSOR_MBPAR, but it does not update "parent" parameter, only individual ones.
 * (can use 4 elements in pars_to_update array, increase size where needed!).
 * Relies on the fact that individual parameters are processed later, so it verifies that broadcast
 * does not modify individual if they are also modified and scheduled to be applied.
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 * @see SET_SENSOR_MBPAR */
#define SET_SENSOR_MBOPAR(p,f,s,r,v)  { X3X3_I2C_SEND2((p), (f), (s), (r), (v)); \
                                        { \
                                          int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),0); \
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                          if (_MINDEX) { \
                                            if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                              pars_to_update[nupdate  ].num= _MINDEX ;\
                                              pars_to_update[nupdate++].val=(v);\
                                            } \
                                            _MINDEX++ ;\
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                            if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                              pars_to_update[nupdate  ].num= _MINDEX ;\
                                              pars_to_update[nupdate++].val=(v);\
                                            } \
                                            _MINDEX++ ;\
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
                                            if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
                                              pars_to_update[nupdate  ].num= _MINDEX ;\
                                              pars_to_update[nupdate++].val=(v);\
                                            } \
                                          } \
                                        } \
                                     }

/**Set parameter for the same register in multiple multiplexed sensors and send to hardware i2c sequencer
 * Similar to SET_SENSOR_MBPAR, but it does not update "parent" parameter, only individual ones.
 * (can use 4 elements in pars_to_update array, increase size where needed!).
 * Relies on the fact that individual parameters are processed later, so it verifies that broadcast
 * does not modify individual if they are also modified and scheduled to be applied.
 * @param p sensor port number
 * @param f frame number to apply, <0 - ASAP
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 * @see SET_SENSOR_MBPAR */
#define SET_SENSOR_MBOPAR_LUT(p,f,r,v)  {\
	int _I = pSensorPortConfig[(p)].broadcast_addr;\
	int _ADDR = pSensorPortConfig[(p)].par2addr[_I][(r)];\
	if (!(_ADDR&0xffff0000)) {\
		X3X3_I2C_SEND2_LUT((p), (f), _I, _ADDR, (v));\
	}\
    {\
    	int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),0); \
        EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
        		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
        if (_MINDEX) { \
        	if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
        		pars_to_update[nupdate  ].num= _MINDEX ;\
                pars_to_update[nupdate++].val=(v);\
            } \
            _MINDEX++ ;\
            EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
            		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
            if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
            	pars_to_update[nupdate  ].num= _MINDEX ;\
                pars_to_update[nupdate++].val=(v);\
            }\
            _MINDEX++ ;\
            EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s _MINDEX=0x%x, v=0x%x, FRAMEPAR_MODIFIED(_MINDEX)=0x%x\n",\
            		__FILE__,__LINE__,__FUNCTION__, _MINDEX, (int) (v), (int) FRAMEPAR_MODIFIED(_MINDEX) ));\
            if (!FRAMEPAR_MODIFIED(_MINDEX)) { \
            	pars_to_update[nupdate  ].num= _MINDEX ;\
                pars_to_update[nupdate++].val=(v);\
            } \
        } \
    } \
}

/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * Do nothing if there is no individual parameter reserved
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param i sensor index (1..3)
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 */
#define SET_SENSOR_MIPAR(p,f,s,i,r,v)  { int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
                                       if (_MINDEX) { \
                                         pars_to_update[nupdate  ].num= _MINDEX ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p), (f), ( s )+( I2C359_INC * ( i )), ( r ), ( v )); \
                                       } \
                                     }

/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * Do nothing if there is no individual parameter reserved
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param i mux sensor index: 0..2 - individual, 3 - broadcast
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 */
#define SET_SENSOR_MIPAR_LUT(p,f,i,r,v)  {\
	int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
    if (_MINDEX) { \
    	pars_to_update[nupdate  ].num= _MINDEX ;\
        pars_to_update[nupdate++].val=(v);\
    	int _ADDR = pSensorPortConfig[(p)].par2addr[(i)][(r)];\
    	if (!(_ADDR&0xffff0000)) {\
    		X3X3_I2C_SEND2_LUT((p), (f), (i), _ADDR, (v));\
    	}\
    } \
}

/**Set individual (multiplexed) sensor parameter if the new value is different from the shadow
 * (and send to hardware i2c sequencer).
 * Do nothing if there is no individual parameter reserved
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param i sensor index (1..3)
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 */
#define SET_SENSOR_MIPAR_COND (p,f,s,i,r,v) \
                                     { int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
                                       if ((_MINDEX) && ((v) != thispars->pars[_MINDEX])) { \
                                         pars_to_update[nupdate  ].num= _MINDEX ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p), (f), ( s )+( I2C359_INC * (( i )+1)), ( r ), ( v )); \
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s  X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n", \
                       __FILE__,__LINE__,__FUNCTION__,(int) (f), (int)(( s )+( I2C359_INC * (( i )+1))),(int) ( r ),(int) ( v ) )); \
                                       } \
                                     }

/**
 * Set individual (multiplexed) sensor parameter if the new value is different from the shadow
 * (and send to hardware i2c sequencer).
 * Do nothing if there is no individual parameter reserved
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param i mux sensor index: 0..2 - individual, 3 - broadcast
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 */
#define SET_SENSOR_MIPAR_COND_LUT (p,f,i,r,v) {\
	int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
	if ((_MINDEX) && ((v) != thispars->pars[_MINDEX])) { \
		pars_to_update[nupdate  ].num= _MINDEX ;\
		pars_to_update[nupdate++].val=(v);\
    	int _ADDR = pSensorPortConfig[(p)].par2addr[(i)][(r)];\
    	if (!(_ADDR&0xffff0000)) {\
    		X3X3_I2C_SEND2_LUT((p), (f), (i), _ADDR, (v));\
    	}\
		EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s  X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n"\
				,__FILE__,__LINE__,__FUNCTION__,(int)(f),(int)(i),(int)_ADDR,(int)(v) )); \
	 } \
 }

/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * Fall back to common parameter if no individual exists
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param i sensor index (1..3)
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits) */
#define SET_SENSOR_MIBPAR(p,f,s,i,r,v) { int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
                                       if (_MINDEX) { \
                                         pars_to_update[nupdate  ].num= _MINDEX ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p), (f), ( s )+( I2C359_INC * (( i )+1 )), ( r ), ( v )); \
                                       }  else { \
                                         pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p), (f), (s), (r), (v)); \
                                       } \
                                     }


/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * Fall back to common parameter if no individual exists
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param i mux sensor index: 0..2 - individual, 3 - broadcast
 * @param r sensor register address (8-bit) - looked up to 16 bit
 * @param v value to set (16 bits)
 * */
#define SET_SENSOR_MIBPAR_LUT(p,f,i,r,v) {\
	int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
	int _I = (i);\
    if (_MINDEX) { \
    	pars_to_update[nupdate  ].num= _MINDEX ;\
        pars_to_update[nupdate++].val=(v);\
    }else{ \
        pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
        pars_to_update[nupdate++].val=(v);\
        _I = MUX_BROADCAST_INDEX;\
    } \
    int _ADDR = pSensorPortConfig[(p)].par2addr[_I][(r)];\
    if (!(_ADDR&0xffff0000)) {\
    	X3X3_I2C_SEND2_LUT((p), (f), _I, _ADDR, (v));\
    }\
}


/// same, but only if different from the shadow
/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * only if the new value is different from the current one.
 * Fall back to common parameter if no individual exists
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param i sensor index (1..3) - actually 0..2, then slave address = (s) + I2C359_INC*(i+1), normal (s) is broadcast
 * @param r sensor register address (8-bit)
 * @param v value to set (16 bits)
 * @see SET_SENSOR_MIBPAR */
#define SET_SENSOR_MIBPAR_COND(p,f,s,i,r,v) \
                                     { int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
                                       if (_MINDEX) { \
                                         if ((v) != thispars->pars[_MINDEX]) { \
                                           pars_to_update[nupdate  ].num= _MINDEX ;\
                                           pars_to_update[nupdate++].val=(v);\
                                           X3X3_I2C_SEND2((p), (f), ( s )+( I2C359_INC * (( i )+1)), ( r ), ( v )); \
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s  X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n", \
                       __FILE__,__LINE__,__FUNCTION__,(int) (f), (int)(( s )+( I2C359_INC * (( i )+1))),(int) ( r ),(int) ( v ) )); \
                                         } \
                                       }  else { \
                                         if ((v) != thispars->pars[P_SENSOR_REGS+(r)]) { \
                                           pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
                                           pars_to_update[nupdate++].val=(v);\
                                           X3X3_I2C_SEND2((p), (f), (s), (r), (v)); \
                       EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s  X3X3_I2C_SEND2(0x%x, 0x%x, 0x%x, 0x%x)\n", \
                       __FILE__,__LINE__,__FUNCTION__,(int) (f), (int)( s ),(int) ( r ),(int) ( v ) )); \
                                         } \
                                       } \
                                     }


/// same, but only if different from the shadow
/**
 * - Set individual (muxed) sensor parameter (and send to hardware i2c sequencer)
 * only if the new value is different from the current one.
 * - Fall back to common parameter if no individual exists
 * @param p sensor port number
 * @param f frame number to apply, <0 - ASAP
 * @param i mux sensor index: 0..2 - individual, 3 - broadcast
 * @param r sensor register address, 8 bits are looked up to 16 bits
 * @param v value to set - 16 bits
 * @see SET_SENSOR_MIBPAR_LUT
 * */
#define SET_SENSOR_MIBPAR_COND_LUT(p,f,i,r,v) {\
    int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i));\
    int _I = (i);\
    int _ADDR;\
    if (_MINDEX) { \
    	if ((v) != thispars->pars[_MINDEX]) { \
    		pars_to_update[nupdate  ].num= _MINDEX ;\
            pars_to_update[nupdate++].val=(v);\
    	}\
    }else{\
        if ((v) != thispars->pars[P_SENSOR_REGS+(r)]) { \
        	pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
            pars_to_update[nupdate++].val=(v);\
            _I = pSensorPortConfig[(p)].broadcast_addr;\
        } \
    } \
    _ADDR = pSensorPortConfig[(p)].par2addr[_I][(r)];\
    if (!(_ADDR&0xffff0000)) {\
    	X3X3_I2C_SEND2_LUT((p), (f), _I, _ADDR, (v));\
    }\
    EDBG(if (GLOBALPARS(G_DEBUG) & (1 <<4)) printk("%s:%d:%s X3X3_I2C_SEND2_LUT(0x%x, 0x%x, 0x%x, 0x%x)\n",\
    		__FILE__,__LINE__,__FUNCTION__,(int)(f),(int)_I,(int)_ADDR,(int)(v) )); \
}

#endif
