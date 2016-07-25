//pgm_functions.h
///extern struct sensorproc_t * sensorproc;

#include "sensor_i2c.h"
#define LEGACY_READ_PAGE 0xff

int init_pgm_proc(void); /// initialize array of functions that program different acquisition parameters (some are sensor dependent)
int add_sensor_proc(int index, int (*sens_func)(struct sensor_t * ,  struct framepars_t * , struct framepars_t *, int ));

//int pgm_detectsensor  (struct sensor_t * sensor,  struct framepars_t * thispars, struct framepars_t * prevpars, int frame8);

// Slave address is now 7-bit,old was 8-bit
// TODO: add registering single sensor as in multi10359. Registering twice is OK
#define X3X3_I2C_SEND2(p,a,s,r,d) write_xi2c_reg16_abs_asap(p,s,a,r,d)

/*
//  Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode
void  write_xi2c_reg16_abs_asap (int chn,  // sensor port
	 	                         int page, // index in the table (8 bits)
	 	                    int frame, // absolute frame number modulo PARS_FRAMES
                            int addr, // low byte of the register address (high is in the table), 8 bits
	      				    u32 data); //16 or 8-bit data (LSB aligned)
 */



#define FRAMEPAR_MODIFIED(x) (thispars->mod[(x) >> 5] & (1<<((x) & 0x1f)))
#define SETFRAMEPARS_SET(p,v)        { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=(v);}
#define SETFRAMEPARS_UPDATE(p)       { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=thispars->pars[(p) & 0xffff];}
#define SETFRAMEPARS_UPDATE_SET(p,v) { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=thispars->pars[(p) & 0xffff]?thispars->pars[(p) & 0xffff]:(v);}
///Like SETFRAMEPARS_SET(p,v), but do nothing if not chnaged
#define SETFRAMEPARS_COND(p,v)  { if (unlikely((v)!=thispars->pars[p])) { pars_to_update[nupdate  ].num= (p) ;  pars_to_update[nupdate++].val=(v);} }


/*
    if (unlikely((thispars->pars[P_FLIPH] & sensor->flips & 1)!=thispars->pars[P_FLIPH])) { /// remove unsupoported flips
       SETFRAMEPARS_SET(P_FLIPH, (thispars->pars[P_FLIPH] & sensor->flips & 1));
    }

*/


///set parameter for the sensor register and hardware itself
/// f - fpga address, s - i2c slave address, r - sensor register, v - value to set
#define SET_SENSOR_PAR(p, f,s,r,v)     { pars_to_update[nupdate  ].num= P_SENSOR_REGS+(r) ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p),(f), (s), (r), (v)); \
                                       }
/// same, but broadcast set for parameters with individual values. Updates individual ones (can use 4 elements in pars_to_update array, increase size where needed!)
/// relies that individual parameters are processed later, so it verifyis that broadcast does not modify individual if they are also modified and scheduled to be applied
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
/// same, but do not update the "parent" parameter, only the individual ones
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



/// set individual sensor parameter, do nothing if no individual exists
#define SET_SENSOR_MIPAR(p,f,s,i,r,v)  { int _MINDEX= MULTIREG(p,P_SENSOR_REGS+(r),(i)); \
                                       if (_MINDEX) { \
                                         pars_to_update[nupdate  ].num= _MINDEX ;\
                                         pars_to_update[nupdate++].val=(v);\
                                         X3X3_I2C_SEND2((p), (f), ( s )+( I2C359_INC * ( i )), ( r ), ( v )); \
                                       } \
                                     }
/// Same but only if different from the shadow
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




/// set individual sensor parameter, fall back to common parameter if no individual exists
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

