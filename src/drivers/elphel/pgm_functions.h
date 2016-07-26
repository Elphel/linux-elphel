//pgm_functions.h
///extern struct sensorproc_t * sensorproc;

#include "sensor_i2c.h"
#define LEGACY_READ_PAGE 0xff

int init_pgm_proc(void);
int add_sensor_proc(int index, int (*sens_func)(int sensor_port, struct sensor_t * ,  struct framepars_t * , struct framepars_t *, int ));


/** Perform I2C write (8  bits address, a6 bits data in "legacy" mode,
 * pages matching slave address should be registered.
 *
 * TODO: Add registering single sensors as in multi10359. Registering twice is OK.
 * Slave address is now 7-bit,old was 8-bit, change (10359 already done)
 * @param port - sensor port
 * @param frame Frame number to apply, <0 - ASAP
 * @param sa7 I2C slave address, 7 bit
 * @param reg sensor register address (8-bit)
 * @param data value to set (16 bits) */
#define X3X3_I2C_SEND2(port,frame,sa7,reg,data) write_xi2c_reg16_abs_asap(port,sa7,frame,reg,data)


/** Tells if parameter is modifies
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


/**Set parameter for the sensor register and send to hardware i2c sequencer
 * @param port Sensor port number
 * @param frame Frame number to apply, <0 - ASAP
 * @param sa7 I2C slave address, 7 bit
 * @param reg sensor register address (8-bit)
 * @param data value to set (16 bits) */
#define SET_SENSOR_PAR(port,frame,sa7,reg,data) { pars_to_update[nupdate  ].num= P_SENSOR_REGS+(reg) ;\
                                                 pars_to_update[nupdate++].val=(data);\
                                                 X3X3_I2C_SEND2((port),(frame), (sa7), (reg), (data)); \
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
                                       int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),0); \
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

/// same, but only if different from the shadow
/**Set individual (multiplexed) sensor parameter (and send to hardware i2c sequencer)
 * only if the new value is different from the current one.
 * Fall back to common parameter if no individual exists
 * @param p Sensor port number
 * @param f Frame number to apply, <0 - ASAP
 * @param s I2C slave address, 7 bit
 * @param i sensor index (1..3)
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

