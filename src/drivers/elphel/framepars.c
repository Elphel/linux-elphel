/** @file framepars.c
 * @brief Handling of frame parameters, making use of FPGA i2c
 * and command sequencer that accepts commands up to 14 frames ahead.
 *
 * This module includes parameter storage, code called from ISR,
 * from other kernel drivers as well as from the user space
 * @copyright Copyright (C) 2016 Elphel, Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//copied from cxi2c.c - TODO:remove unneeded

//#define DEBUG // should be before linux/module.h - enables dev_dbg at boot in this file (needs "debug" in bootarg)
#include <linux/types.h>        // div for 64
#include <asm/div64.h>          // div for 64

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
//#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

//#include <asm/system.h>
//#include <asm/byteorder.h> // endians
//#include <asm/io.h>

//#include <asm/irq.h>

//#include <asm/delay.h>
#include <asm/uaccess.h>
#include <uapi/elphel/c313a.h>
#include <uapi/elphel/exifa.h>
#include <uapi/elphel/x393_devices.h>

//#include "fpgactrl.h"  // defines port_csp0_adsensor_common.hdr, port_csp4_addr
//#include "cc3x3.h"
//#include "x3x3.h"           // hardware definitions

#include "x393_fpga_functions.h"
#include "sensor_common.h"
#include "framepars.h"
#include "param_depend.h" // specifies what functions should be called for different parameters changed
// needed for lseek commands
//#include "cxdma.h" // x313_dma_init
//#include "cci2c.h" // to use void i2c_reset_wait(void), reset shadow static 'i2c_hardware_on'
#include "x393_macro.h"
#include "x393.h"
#include "sensor_i2c.h" // read_xi2c_frame()
#include "klogger_393.h"

/**
 * \def MDF1(x) optional debug output
 */

#undef LOCK_BH_PROCESSPARS

//#undef LOCK_BH_FRAMEPARS
#define LOCK_BH_FRAMEPARS

//#define USE_KLOG393



#ifdef LOCK_BH_FRAMEPARS
    #define FLAGS_IBH
    #define LOCK_IBH(x)   spin_lock_bh(x)
    #define UNLOCK_IBH(x) spin_unlock_bh(x)
#else
    #define FLAGS_IBH     unsigned long flags;
    #define LOCK_IBH(x)   spin_lock_irqsave(x,flags)
    #define UNLOCK_IBH(x) spin_unlock_irqrestore(x,flags)
#endif


#include "debug393.h"

#define  DRIVER_NAME        DEV393_NAME(DEV393_FRAMEPARS0)
#define  DRIVER_DESCRIPTION "Elphel (R) Model 393 Frame Parameters device driver"

#define ELPHEL_DEBUG 0

// Below are old NC353 debug macros, remove them

#if ELPHEL_DEBUG
// #define MDF2(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 2)) { printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; } }
// setFrameParsAtomic
 #define MDF5(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 5)) { printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; } }
 #define   D5(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 5)) { x; } }
// processPars
 #define MDF6(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 6)) { printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; } }
 #define   D6(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 6)) { x; } }
//update FramePars
 #define MDF7(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 7)) { printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; } }
 #define   D7(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 7)) { x; } }
// setFramePar[s]
 #define MDF8(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 8)) { printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; } }
 #define   D8(x) { if (GLOBALPARS(0,G_DEBUG) & (1 << 8)) { x; } }
 #define D5(chn,...)
 #define D6(chn,...)
 #define D7(chn,...)
 #define D8(chn,...)

 #define ELPHEL_DEBUG_THIS 0
// #define ELPHEL_DEBUG_THIS 1
#else
 #define MDF2(x)
 #define MDF5(x)
 #define D5(x)
 #define MDF6(x)
 #define D6(x)
 #define MDF7(x)
 #define D7(x)
 #define MDF8(x)
 #define D8(x)
 #define ELPHEL_DEBUG_THIS 0
#endif


#if ELPHEL_DEBUG_THIS
  #define MDD1(x) printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x; udelay(ELPHEL_DEBUG_DELAY)
  #define MDF1(x) printk("%s:%d:%s ", __FILE__, __LINE__, __FUNCTION__); x
  #define D1(x) x
  #define D1I(x)

#else
  #define MDD1(x)
  #define MDF1(x)
  #define D1(x)
  #define D1I(x) x
#endif

/**
 * devices names, "/dev/"+dev_name[i] - /dev/frameparsall0..3
 */
static const char * const framepars_name[] = {
	DEV393_DEVNAME(DEV393_FRAMEPARS0),
	DEV393_DEVNAME(DEV393_FRAMEPARS1),
	DEV393_DEVNAME(DEV393_FRAMEPARS2),
	DEV393_DEVNAME(DEV393_FRAMEPARS3)
};

/**
 * devices major - is one number
 * TODO: switch to dynamic major and get rid of this
 */
static const int framepars_major = DEV393_MAJOR(DEV393_FRAMEPARS0);

/**
 * devices minors - basically base+i - can be a single number
 * TODO: switch to dynamic assignment and get rid of this
 */
static const int framepars_minor[] = {
 	DEV393_MINOR(DEV393_FRAMEPARS0),
 	DEV393_MINOR(DEV393_FRAMEPARS1),
 	DEV393_MINOR(DEV393_FRAMEPARS2),
	DEV393_MINOR(DEV393_FRAMEPARS3)
};

/**
 * device class
 */
static struct class *framepars_dev_class;

static int hardware_initialized = 0;


/* 393: sFrameParsAll is an array of 4per-port structures */
static struct framepars_all_t sFrameParsAll[SENSOR_PORTS] __attribute__ ((aligned(PAGE_SIZE)));  ///< Sensor Parameters, currently 16 pages all and 2048 pages some, static struct
unsigned int frameParsInitialized[SENSOR_PORTS];                                                     // set to 0 at startup, 1 after initialization that is triggered by setParsAtomic()
#define  thisFrameNumber(p)            GLOBALPARS(p,G_THIS_FRAME)                       // Current frame number (may lag from the hardware)
#define  thisCompressorFrameNumber(p)  GLOBALPARS(p,G_COMPRESSOR_FRAME)                 // Current compressed frame number (lags from thisFrameNumber)

#ifdef NC353
	struct framepars_all_t  *frameparsall = NULL;           // - will be mmap-ed
	struct framepars_t      *framepars =   NULL;            ///< getting rid of static to be able to use extern
	struct framepars_past_t *pastpars =    NULL;            ///< getting rid of static to be able to use extern
	unsigned long           *funcs2call =  NULL;            //  sFrameParsAll.func2call.pars; - each parameter has a 32-bit mask of what pgm_function to call - other fields not used
	unsigned long           *globalPars =  NULL;            // parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both
	unsigned long           *multiSensIndex = NULL;         // index for per-sensor alternatives
	unsigned long           *multiSensRvrsIndex = NULL;     // reverse index (to parent) for the multiSensIndex
wait_queue_head_t framepars_wait_queue;                 // used to wait for the frame to be acquired
#endif
/* 393 : Changing to per-port */
struct framepars_all_t  *aframeparsall = NULL;              // - will be mmap-ed, in 393 points to an array of structures
struct framepars_t      *aframepars[SENSOR_PORTS];          ///< getting rid of static to be able to use extern
struct framepars_past_t *apastpars[SENSOR_PORTS];            ///< getting rid of static to be able to use extern
unsigned long           *afuncs2call[SENSOR_PORTS];          //  sFrameParsAll.func2call.pars; - each parameter has a 32-bit mask of what pgm_function to call - other fields not used
unsigned long           *aglobalPars[SENSOR_PORTS];          // parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both
unsigned long           *amultiSensIndex[SENSOR_PORTS];      // index for per-sensor alternatives
unsigned long           *amultiSensRvrsIndex[SENSOR_PORTS];  // reverse index (to parent) for the multiSensIndex
wait_queue_head_t       aframepars_wait_queue[SENSOR_PORTS];// used to wait for the frame to be acquired

static DEFINE_SPINLOCK(framepars_lock_0); ///<
static DEFINE_SPINLOCK(framepars_lock_1); ///<
static DEFINE_SPINLOCK(framepars_lock_2); ///<
static DEFINE_SPINLOCK(framepars_lock_3); ///<
/** Define array of pointers to locks - hardware allows concurrent writes to different ports tables */
spinlock_t * framepars_locks[4] = {&framepars_lock_0, &framepars_lock_1, &framepars_lock_2, &framepars_lock_3};

static struct common_pars_t scommon_pars = {
        .master_chn =       0,
        .sensors=           {0,0,0,0}, // maybe not needed (it is whom to notify of
        .updated =          {0,0,0,0}, // set by master, cleared by other channels
        .trig_period =      0,
        .trig_bitlength =   0,
        .extern_timestamp = 0,
        .xmit_timestamp =   0,
        .trig_condition =   0,
        .trig_out =         0,
        .trig_mode =        TRIGMODE_FREERUN
};
/*
#define TRIGMODE_FREERUN  0
#define TRIGMODE_SNAPSHOT 4
#define TRIGMODE_GRR      20

 */
struct common_pars_t *common_pars = NULL;
/* Remove after compilation OK */
//struct sensorproc_t * sensorproc = NULL;
//void compressor_interrupts (int on) {}
#if 0
#define wait_event_interruptible(wq, condition)				\
({									\
	int __ret = 0;							\
	might_sleep();							\
	if (!(condition))						\
		__ret = __wait_event_interruptible(wq, condition);	\
	__ret;								\
})

#define __wait_event_interruptible_timeout(wq, condition, timeout)	\
	___wait_event(wq, ___wait_cond_timeout(condition),		\
		      TASK_INTERRUPTIBLE, 0, timeout,			\
		      __ret = schedule_timeout(__ret))

#define __wait_event_interruptible(wq, condition)			\
	___wait_event(wq, condition, TASK_INTERRUPTIBLE, 0, 0,		\
		      schedule())

#endif

/** @brief Global pointer to basic device structure. This pointer is used in debugfs output functions */
static struct device *g_devfp_ptr;

//wait_queue_head_t framepars_wait_queue; // used to wait for the frame to be acquired

/**
 * @brief file private data
 */
struct framepars_pd {
	int minor;                                      ///< file minor value
	struct wait_queue *framepars_wait_queue;        ///< wait queue (waiting for file number to increase)  //NOTE: not used at all?
// something else to be added here?
};

static u32 debug_flags = 0;

/**
 * @brief not used
 */
static struct class *framepars_device_class; /* global device class */

/**
 * @brief assign non-static pointers to static data to be used as extern
 */
void init_framepars_ptr(int sensor_port)
{
//	frameparsall =      &sFrameParsAll; // - will be mmap-ed
	aframeparsall =                    sFrameParsAll; // - will be mmap-ed
	aframepars[sensor_port] =          sFrameParsAll[sensor_port].framePars;
	apastpars[sensor_port] =           sFrameParsAll[sensor_port].pastPars;
	afuncs2call[sensor_port] =         sFrameParsAll[sensor_port].func2call.pars;       // each parameter has a 32-bit mask of what pgm_function to call - other fields not used
	aglobalPars[sensor_port] =         sFrameParsAll[sensor_port].globalPars;           // parameters that are not frame-related, their changes do not initiate any actions so they can be mmaped for both
	amultiSensIndex[sensor_port] =     sFrameParsAll[sensor_port].multiSensIndex;      // indexes of individual sensor register shadows (first of 3) - now for all parameters, not just sensor ones
	amultiSensRvrsIndex[sensor_port] = sFrameParsAll[sensor_port].multiSensRvrsIndex;  // reverse index (to parent) for the multiSensIndex
	common_pars =                      &scommon_pars;
}

int        framepars_open(struct inode *inode, struct file *filp);
int        framepars_release(struct inode *inode, struct file *filp);
loff_t     framepars_lseek(struct file * file, loff_t offset, int orig);
ssize_t    framepars_write(struct file * file, const char * buf, size_t count, loff_t *off);
int        framepars_mmap(struct file *file, struct vm_area_struct *vma);
void       trigSlaveUpdate(int sensor_port);

/**
 * @brief Reset hardware sequencers (i2c, command) and initialize framepars structure
 * Does not seem to do anything with the sequencers
 */
int initSequencers(int sensor_port)
{
//    x393_rtc_sec_t  rtc_sec =  {.d32=0};
    sec_usec_t sec_usec;
    FLAGS_IBH
    if (!is_fpga_programmed()){
        dev_err(g_devfp_ptr,"*** Attempted to access hardware without bitstream ***\n");
        return - ENODEV;
    }
    if (!hardware_initialized) {
        dev_dbg(g_devfp_ptr,"Configuring compressor DMA channels\n");
        dev_info(g_devfp_ptr,"Configuring compressor DMA channels\n");
        init_compressor_dma(0xf, // all channels (TODO: NC393 - select channels in DT or use existing for sensors?
                              0); // not to interfere with python setting the same
        // Start RTC by writing 0 to seconds if it was not already set, otherwise preserve current time
        get_fpga_rtc(&sec_usec);
        set_fpga_rtc( sec_usec);
        // Start RTC by writing 0 to seconds
//        set_x393_rtc_sec_set (rtc_sec); // started RTC, correct time may be set later or before
        hardware_initialized = 1;
    }
	dev_dbg(g_devfp_ptr,"port= %d,initSequencers:resetting both sequencers (not really?)\n", sensor_port);
#ifdef TEST_DISABLE_CODE
	local_ irq_save(flags);
	X3X3_SEQ_RESET;
	i2c_reset_wait();
	local_irq_restore(flags);
#endif
    initFramePars(sensor_port);
    return 0;
}
/** Enable/disable sesnor channel (will not generate SoF/EoF pulses and interrupts if disabled). Used to turn off missing channels */
void enDisSensorChn(int sensor_port, ///< sensor_port sensor port number (0..3)
                    int en)          ///< enable channel
{
    x393_sens_mode_t sens_mode =  {.d32=0};
    sens_mode.chn_en =     en;
    sens_mode.chn_en_set = 1;
    x393_sens_mode(sens_mode,sensor_port);
    dev_dbg(g_devfp_ptr,"enDisSensorChn(%d,%d)\n", sensor_port, en);
}




/** Stop frame sequencer, optionally disable interrupts also */
void stopFrameSequencer(int sensor_port, ///< sensor_port sensor port number (0..3)
                        int dis_int)     ///< disable interrupts
{
    x393_cmdframeseq_mode_t cmdframeseq_mode = {.d32=0};
    // TODO: Add locking for sequence reset?
    cmdframeseq_mode.run_cmd = 2; // Stop
    if (dis_int) cmdframeseq_mode.interrupt_cmd = 2; // disable
    x393_cmdframeseq_ctrl(cmdframeseq_mode, sensor_port);
    dev_dbg(g_devfp_ptr,"Stop command sequencer for port= %d,  dis_int = %d\n", sensor_port, dis_int);
}

/** Reset absolute frame number \b thisFrameNumber to \b frame16, optionally reset/restart sequencer */
void resetFrameNumber(int sensor_port, ///< sensor_port sensor port number (0..3)
                      u32 aframe,      ///< absolute frame number to set (lower 4 bits will be set from hardware)
                      int hreset)      ///< Reset hardware sequencer itself
{
	int i;
	x393_cmdseqmux_status_t stat;
	x393_status_ctrl_t      stat_ctrl;
	x393_cmdframeseq_mode_t cmdframeseq_mode = {.d32=0};
	u32 frame16;
	// TODO: Add locking for sequence reset?
	if (hreset){
	    cmdframeseq_mode.reset = 1;
	    x393_cmdframeseq_ctrl(cmdframeseq_mode, sensor_port);
	    udelay(2);
        cmdframeseq_mode.d32 =     0;
        cmdframeseq_mode.run_cmd = 3; // Run
        x393_cmdframeseq_ctrl(cmdframeseq_mode, sensor_port);
        dev_dbg(g_devfp_ptr,"Reset command sequencer (all channels !): port= %d,  thisFrameNumber=0x%lx\n", sensor_port, thisFrameNumber(sensor_port));
	}

	/* Check if the status update mode for command sequencer is not 3 (auto), set/wait if needed */
	stat_ctrl =  get_x393_cmdseqmux_status_ctrl();
	if (stat_ctrl.mode !=3) {
		stat = x393_cmdseqmux_status();
		stat_ctrl.seq_num = stat.seq_num + 1;
		stat_ctrl.mode = 3;
		set_x393_cmdseqmux_status_ctrl(stat_ctrl);
		for (i = 0; i < 10; i++) {
			stat = x393_cmdseqmux_status();
			if (likely(stat.seq_num == stat_ctrl.seq_num))
				break;
		}
	} else {
		stat = x393_cmdseqmux_status();
	}
	switch (sensor_port) {
	//		case 0: thisFrameNumber(sensor_port) = stat.frame_num0; break;
	case 0: frame16 = stat.frame_num0; break;
	case 1: frame16 = stat.frame_num1; break;
	case 2: frame16 = stat.frame_num2; break;
	default:
	    frame16 = stat.frame_num3; break;
	}
    thisFrameNumber(sensor_port) = (aframe & PARS_FRAMES_MASK) | frame16;
#ifdef NC353
	thisFrameNumber(sensor_port) = X3X3_I2C_FRAME;
#endif
	dev_dbg(g_devfp_ptr,"Resetting frame number, port= %d,  thisFrameNumber=0x%lx\n", sensor_port, thisFrameNumber(sensor_port));
// write absolute frame numbers
//    for (i = thisFrameNumber(sensor_port); i < (thisFrameNumber(sensor_port) + PARS_FRAMES); i++) aframepars[sensor_port][i & PARS_FRAMES_MASK].pars[P_FRAME] = i;
    for (i = 0; i <  PARS_FRAMES; i++) aframepars[sensor_port][(i+frame16) & PARS_FRAMES_MASK].pars[P_FRAME] = thisFrameNumber(sensor_port) + i;
}

/**
 * @brief initialize all parameters, set \b thisFrameNumber to \b frame number read from hardware hardware ( 0 after resetting i2c and cmd_seq)
 * @param sensor_port sensor port number (0..3)
 */
void initFramePars(int sensor_port)
{
	int i;
	FLAGS_IBH;
	LOCK_IBH(framepars_locks[sensor_port]);
	memset(aframepars[sensor_port], 0, sizeof(struct framepars_t) * PARS_FRAMES);
	resetFrameNumber(sensor_port, 0, 1);
// initialize frameParsDeps.pars masks:
	for (i = 0; i < (sizeof(param_depend_tab) / 8); i++) {
		afuncs2call[sensor_port][param_depend_tab[2 * i] & 0xffff] = param_depend_tab[2 * i + 1]; // remove possible flags
		dev_dbg(g_devfp_ptr,"%s : port= %d, funcs2call[0x%lx]=0x%08lx\n",__func__, sensor_port, param_depend_tab[2 * i] & 0xffff, param_depend_tab[2 * i + 1]);
	}
	for (i = 0; i < P_SENSOR_NUMREGS; i++) afuncs2call[sensor_port][P_SENSOR_REGS + i] = ONCHANGE_SENSORREGS;     // by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function
// Same for 10359 registers - will not change anything if there is no 10359 - these registers will not be changed, and if will be it wil cause no action
	for (i = 0; i < P_M10359_NUMREGS; i++) afuncs2call[sensor_port][P_M10359_REGS + i] = ONCHANGE_SENSORREGS;     // by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function

	initMultiPars(sensor_port);                                                                                // initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called after/during sensor detection
	frameParsInitialized[sensor_port] = 1;
	UNLOCK_IBH(framepars_locks[sensor_port]);
    dev_dbg(g_devfp_ptr,"%s port %d: DONE, frameParsInitialized[%d]=%d\n",
            __func__, sensor_port, sensor_port, frameParsInitialized[sensor_port]);
}

/**
 * @brief reset all global parameters, set default for debug mask (if ELPHEL_DEBUG)
 * @param sensor_port sensor port number (0..3)
 */
void initGlobalPars(int sensor_port)
{
	memset(&aglobalPars[sensor_port][GLOBALS_PRESERVE], 0, (NUM_GPAR - GLOBALS_PRESERVE) * sizeof(unsigned long));
//   MDF(GLOBALPARS(G_DEBUG) = ELPHEL_DEBUG_STARTUP;// removed - add write to fpga init script
	dev_dbg(g_devfp_ptr,"%s : port= %d, GLOBALPARS(0,G_DEBUG)=%lx\n",__func__, sensor_port, GLOBALPARS(0,G_DEBUG));
}

/**
 * @brief initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called after/during sensor detection
 * @param sensor_port sensor port number (0..3)
 * @return number of multi-regs
 */
int initMultiPars(int sensor_port)
{
	int i, j, n;
	int ireg = P_MULTI_REGS; // multi-reg shadows start index
	unsigned long m;

//	memset(amultiSensIndex[sensor_port], 0, sizeof(struct framepars_all_t.multiSensIndex));
	memset(amultiSensIndex[sensor_port], 0, P_MAX_PAR_ROUNDUP * sizeof(int));
//	memset(multiSensRvrsIndex, 0, sizeof(multiSensRvrsIndex));
	memset(amultiSensRvrsIndex[sensor_port], 0, P_MAX_PAR_ROUNDUP * sizeof(int));
	GLOBALPARS(sensor_port,G_MULTI_NUM) = 0;
	for (i = 0; i < 8; i++) {
		m = GLOBALPARS(sensor_port, G_MULTI_REGSM + i); // 1 bit per register that need individual shadows
//     dev_dbg(g_devfp_ptr,"%s i=%d, m=0x%lx\n",__func__,i,m);
		for (j = P_SENSOR_REGS + (i << 5); m && (GLOBALPARS(sensor_port,G_MULTI_NUM) < P_MULTI_NUMREGS); j++, m >>= 1) {
			if (m & 1) {
				amultiSensIndex[sensor_port][j] = ireg;
//         dev_dbg(g_devfp_ptr,"%s j=0x%x ireg=0x%x\n",__func__,j,ireg);
				for (n = 0; n < MAX_SENSORS; n++) {
					afuncs2call[sensor_port][ireg] = ONCHANGE_SENSORREGS; // by default each "manual" write to any of these registers will trigger pgm_sensorreg function
					amultiSensRvrsIndex[sensor_port][ireg++] = j | ((n + 1) << 16);
				}
				GLOBALPARS(sensor_port,G_MULTI_NUM)++;
			}
		}
	}
	// remark: the line below is called from initFramePars, consider removing it
	for (i = 0; i < P_SENSOR_NUMREGS; i++) afuncs2call[sensor_port][P_SENSOR_REGS + i] = ONCHANGE_SENSORREGS;  // by default each "manual" write to any of 256 registers will trigger pgm_sensorreg function
	dev_dbg(g_devfp_ptr,"%s : port= %d, GLOBALPARS(G_MULTI_NUM)=%lx\n",__func__, sensor_port, GLOBALPARS(sensor_port, G_MULTI_NUM));
	return GLOBALPARS(sensor_port, G_MULTI_NUM);
}

/** Reads parameters for the specified frame number, OK to use in compressor ISR using last compressed frame */
inline unsigned long get_imageParamsFrame(int sensor_port, ///< sensor port (0..3)
                                          int n,           ///< parameter index (should be 128..143)
                                          int frame)       ///< absolute frame number
{
    return aframepars[sensor_port][frame & PARS_FRAMES_MASK].pars[n];
}

inline unsigned long * get_imageParamsFramePtr(int sensor_port, ///< sensor port (0..3)
                                               int n,           ///< parameter index (should be 128..143)
                                               int frame)       ///< absolute frame number
{
    return &(aframepars[sensor_port][frame & PARS_FRAMES_MASK].pars[n]);
}

inline unsigned long * get_imageParamsPastPtr(int sensor_port, ///< sensor port (0..3)
                                              int n,           ///< parameter index (should be 128..143)
                                              int frame)       ///< absolute frame number
{
    return &(apastpars[sensor_port][frame & PASTPARS_SAVE_ENTRIES_MASK].past_pars[n-PARS_SAVE_FROM]);
}




/**
 * @brief reads parameters from the current frame (matching hardware index)
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to read
 * @return parameter value (unsigned long)
 */
inline unsigned long get_imageParamsThis(int sensor_port, int n)
{
	return aframepars[sensor_port][thisFrameNumber(sensor_port) & PARS_FRAMES_MASK].pars[n];
}

/**
 * @brief reads parameters from the previous frame (matching hardware index) - used to determine if historam was needed
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to read
 * @return parameter value (unsigned long)
 */
inline unsigned long get_imageParamsPrev(int sensor_port, int n)
{
	return aframepars[sensor_port][(thisFrameNumber(sensor_port) - 1) & PARS_FRAMES_MASK].pars[n];
}

/** Reads past parameters (small subset of all) for absolute frame number */
inline unsigned long get_imageParamsPast(int sensor_port, ///< sensor port (0..3)
                                         int n,           ///< parameter index (should be 128..143)
                                         int frame)       ///< absolute frame number
{
    return apastpars[sensor_port][frame & PASTPARS_SAVE_ENTRIES_MASK].past_pars[n-PARS_SAVE_FROM];
}



/**
 * @brief writes read-only parameter to the current frame (does not propagate to next frames as setFramePar() does)
 * In most cases you really need to use setFramePar() instead;
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to set
 * @param d data to write to the selected parameter
 */

inline void set_imageParamsThis(int sensor_port, int n, unsigned long d)
{
	aframepars[sensor_port][thisFrameNumber(sensor_port) & PARS_FRAMES_MASK].pars[n] = d;
}

/**
 * @brief reads global (not related to particular frames) parameters
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to read (numbers start from FRAMEPAR_GLOBALS)
 * @return parameter value (unsigned long)
 */
inline unsigned long get_globalParam(int sensor_port, int n)
{
	return GLOBALPARS(sensor_port, n);
}
/**
 * @brief sets global (not related to particular frames) parameters
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to set  (numbers start from FRAMEPAR_GLOBALS)
 * @param d data to write to the selected parameter
 */

inline void          set_globalParam(int sensor_port, int n, unsigned long d)
{
	GLOBALPARS(sensor_port, n) = d;
}

/**
 * @brief set same parameters in all frames
 * currently used only in compressor reset
 * @param sensor_port sensor port number (0..3)
 * @param n number of a parameter to set
 * @param d data to write to the selected parameter
 */
inline void          set_imageParamsR_all(int sensor_port, int n, unsigned long d)
{
	int i;
	for (i = 0; i < PARS_FRAMES; i++) aframepars[sensor_port][i].pars[n] = d;
}
/**
 * Update "interframe" parameters (inside the circular buffer). Called from ISR (comressor done)
 * @param sensor_port - sensor
 * @param frame16
 * @param interframe_pars
 */

void updateInterFrame(int sensor_port,      ///< Sensor port number (0..3)
                      u32 compressed_frame, ///< number of frame just compressed
                                            ///< interrupt should be processed after frame sync interrupt
                      struct interframe_params_t * interframe_pars) ///< pointer to the area in circbuf to save parameters
{
//    int findex_this, findex_prev, findex_future, findex_next;
//    int index;
//    int index32;
//    unsigned long bmask, bmask32;
    int pastParsIndex;
    int compressedIndex = compressed_frame & PARS_FRAMES_MASK; // last compressed frame mod 16
    struct framepars_t *framepars = aframepars[sensor_port];

    thisCompressorFrameNumber(sensor_port) = compressed_frame;
    pastParsIndex = compressed_frame & PASTPARS_SAVE_ENTRIES_MASK; // copying from what was past frame that might include histogram data
// After Frame sync interrupt processed, oldest valid will be is sensor frame number -2, compressor lag is 1 frame (0..2), so read not past-pars is OK
#ifdef USE_PASTPARS_IN_CMPRSIRQ
    memcpy(interframe_pars,      &apastpars[sensor_port][pastParsIndex].past_pars[P_GTAB_R-PARS_SAVE_FROM], 24); // will leave some gaps, but copy [P_ACTUAL_WIDTH]
    interframe_pars->height =     apastpars[sensor_port][pastParsIndex].past_pars[P_ACTUAL_HEIGHT-PARS_SAVE_FROM];        // NOTE: P_ACTUAL_WIDTH,P_QUALITY copied with memcpy
    interframe_pars->color =      apastpars[sensor_port][pastParsIndex].past_pars[P_COLOR-PARS_SAVE_FROM];
    interframe_pars->byrshift =   apastpars[sensor_port][pastParsIndex].past_pars[P_COMPMOD_BYRSH-PARS_SAVE_FROM];
    interframe_pars->quality2 |= (apastpars[sensor_port][pastParsIndex].past_pars[P_PORTRAIT-PARS_SAVE_FROM] & 1) << 7;

#else
    memcpy (interframe_pars,  &framepars[compressedIndex].pars[P_GTAB_R], 24); /// will leave some gaps, but copy [P_ACTUAL_WIDTH]
    interframe_pars->height=  framepars[compressedIndex].pars[P_ACTUAL_HEIGHT]; /// NOTE: P_ACTUAL_WIDTH,P_QUALITY copied with memcpy
    interframe_pars->color=   framepars[compressedIndex].pars[P_COLOR];
    interframe_pars->byrshift=framepars[compressedIndex].pars[P_COMPMOD_BYRSH];
    interframe_pars->quality2 |= (framepars[compressedIndex].pars[P_PORTRAIT] & 1) << 7;
#endif


}

//++++++++++++++++++++++++++++++++++++++++++
/** Called from ISR (sensor frame interrupt)- advance thisFrameNumber to match hardware frame16, copy parameters as needed.
 * before: (thisFrameNumber mod 16      pointed to current (for the software) parameters frame (now behind by at least 1, maybe 2)
 *         (thisFrameNumber-1) mod 16 - oldest with parameters preserved, also containes histograms results (+image timestamp, size?)
 *                                        subset of that frame data is copied to pastpars
 *         (thisFrameNumber-2) mod 16- farthest in the future frame
 * after:   thisFrameNumber matches hardware pointer
 */

//TODO: Remove interframe, handle it with compressor interrupts (above)
/// Among other things copies subset of parameters to pastpars

void updateFramePars(int sensor_port, ///< sensor port number (0..3)
                     int frame16)     ///< Sensor sequencer (hardware) frame number
{
	int findex_this, findex_prev, findex_future, findex_next;
	int index, index32;
	unsigned long bmask, bmask32;
	int pastParsIndex;
	struct framepars_t *framepars = aframepars[sensor_port];
#define DEBUG_BYRSH
#ifdef DEBUG_BYRSH
	int comp_frame16 = getHardFrameNumber(sensor_port, 1);    // Use compressor frame number
	u32 comp_aframe = thisCompressorFrameNumber(sensor_port);
	x393_cmprs_mode_t cmprs_mode = get_x393_cmprs_control_reg(sensor_port);
#endif
    dev_dbg(g_devfp_ptr,"%s : port= %d, frame16=%d\n",__func__, sensor_port, frame16);
	while ((frame16 ^ thisFrameNumber(sensor_port)) & PARS_FRAMES_MASK) { // While hardware pointer is still ahead of the software maintained one
	    dev_dbg(g_devfp_ptr,"%s : port= %d, frame16=%d, thisFrameNumber(%d)=%d\n",__func__, sensor_port, frame16, sensor_port, (int)thisFrameNumber(sensor_port));
// before update:
//   framepars[findex_prev]  holds previous frame data (oldest availble - latest?)
//   framepars[findex_future] holds farthest in the future one
// after update:
//   framepars[findex_prev]  holds farthest in the future one ("this" will become "prev")
		findex_this =  thisFrameNumber(sensor_port)  & PARS_FRAMES_MASK; // Nothing is yet done for this frame (mod 16)
		findex_prev =  (findex_this - 1)  & PARS_FRAMES_MASK;            // Holds previous frame data (mod 16)  (latest available)
		findex_future = (findex_this - 2)  & PARS_FRAMES_MASK;           // farthest in the future (mod 16)  (not to overwrite latest
		findex_next =  (findex_this + 1)  & PARS_FRAMES_MASK;            // Just next frame (mod 16) after the current
// copy subset of the parameters to the long buffer of past parameters. TODO: fill Exif also here?
// TODO:DONE: Change - make pastpars be save for all frames, not just compressed ones
// With PASTPARS_SAVE_ENTRIES being multiple of PARS_FRAMES - make it possible to calculate past_index from thisFrameNumber
//    pastParsIndex= thisFrameNumber & PASTPARS_SAVE_ENTRIES_MASK;
		pastParsIndex = (thisFrameNumber(sensor_port) - 1) & PASTPARS_SAVE_ENTRIES_MASK; // copying from what was past frame that might include histogram data
//    memcpy (pastpars[pastParsIndex].past_pars, &framepars[findex_prev].pars[PARS_SAVE_FROM], sizeof(pastpars[0].past_pars));
//		memcpy(apastpars[sensor_port][pastParsIndex].past_pars, &framepars[findex_prev].pars[PARS_SAVE_FROM], PARS_SAVE_COPY * sizeof(int));
        memcpy(apastpars[sensor_port][pastParsIndex].past_pars, &framepars[findex_prev].pars[PARS_SAVE_FROM], PARS_SAVE_COPY << 2);

#ifdef DEBUG_BYRSH
        apastpars[sensor_port][pastParsIndex].past_pars[PARS_SAVE_COPY + 0] = comp_frame16;
        apastpars[sensor_port][pastParsIndex].past_pars[PARS_SAVE_COPY + 1] = comp_aframe;
        apastpars[sensor_port][pastParsIndex].past_pars[PARS_SAVE_COPY + 2] = cmprs_mode.d32;
        apastpars[sensor_port][pastParsIndex].past_pars[PARS_SAVE_COPY + 3] = pastParsIndex;
#endif

		// Now update interframe_pars (interframe area) used to create JPEG headers. Interframe area survives exactly as long as the frames themselves (not like pastpars)

//      debug code - save compressor states in the past parameters
// copy parameters from findex_future (old "farthest in the future") to findex_prev (new "fartherst in the future") if it was changed since
		if ((bmask32 = framepars[findex_prev].modsince32)) {
			dev_dbg(g_devfp_ptr,"%s framepars[%d].modsince32=0x%lx\n",__func__, findex_prev, bmask32);
			for (index32 = 0; bmask32; index32++, bmask32 >>= 1) {
				if (bmask32 & 1) {
					for (index = (index32 << 5), bmask = framepars[findex_prev].modsince[index32]; bmask; index++, bmask >>= 1)
						if (bmask & 1) {
							framepars[findex_prev].pars[index] = framepars[findex_future].pars[index];
							dev_dbg(g_devfp_ptr,"%s hw=%d framepars[%d].pars[%d]=framepars[%d].pars[%d]=0x%lx\n",__func__, frame16, findex_prev, index, findex_future, index, framepars[findex_future].pars[index]);
						}
					framepars[findex_prev].modsince[index32] = 0; //  mark as not "modified since" (yet)
				}
			}
			framepars[findex_prev].modsince32 = 0; //  mark as not "modified since" super index
		}
// clear "modified" and  flags on the brand new future frame
		// remark: framepars[findex_prev].mod is 31 dwords long abd here 32 dwords are cleared
		// explanation: mod32 goes after mod[31] and it is cleared too
		if (framepars[findex_prev].mod32) memset(framepars[findex_prev].mod, 0, 32 * 4);        // .mod[0]-.mod[30], .mod32
		framepars[findex_prev].functions = 0;                                                   // No functions yet needed on the brand new frame
		// remark: replace number 7 with named constant, it should correspond to total frame num (16 in new camera)
		framepars[findex_prev].pars[P_FRAME] = thisFrameNumber(sensor_port) + (PARS_FRAMES-1);  // that will be the full frame number
// NOTE: Handle past due - copy functions, and mod if functions were non-zero
		if (framepars[findex_this].functions) {                                                 // Some functions were not yet processed (past due)
			if (!(get_globalParam(sensor_port, G_TASKLET_CTL) & (1 << TASKLET_CTL_IGNPAST))) {
				framepars[findex_next].functions |= framepars[findex_this].functions;
				if ((bmask32 = framepars[findex_this].mod32)) {
					for (index32 = 0; bmask32; index32++, bmask32 >>= 1) {
						if (bmask32 & 1) {
							framepars[findex_next].mod[index32] |= framepars[findex_this].mod[index32];
						}
						framepars[findex_next].mod32 |= framepars[findex_this].mod32;
					}
				}
				dev_dbg(g_devfp_ptr,"%s resubmitting past due functions = 0x%lx for frame=%ld (0x%x)\n",__func__, framepars[findex_this].functions, thisFrameNumber(sensor_port), findex_this);
			} else {
				dev_dbg(g_devfp_ptr,"%s Ignored past due functions = 0x%lx for frame=%ld (0x%x)\n",__func__, framepars[findex_this].functions, thisFrameNumber(sensor_port), findex_this);
			}
		}
		thisFrameNumber(sensor_port)++;
//        if (thisFrameNumber(sensor_port)<20) {
//            dev_dbg(g_devfp_ptr,"thisFrameNumber(%d)=0x%x\n",sensor_port, (int) thisFrameNumber(sensor_port));
//        }

	}
}

/** Process parameters that are overdue or due in ASAP mode (not through the sequencer)
 * Called twice from processPars - at the beginning and at the end to finish off any derivatives (needed?)
 * Should never be called from outside processPars() where there is a per-port lock */
inline void _processParsASAP(int sensor_port,                   ///< sensor port number (0..3)
                             struct sensorproc_t * sensorproc,  ///< per-port array of sensor capabilities and 32+32 on-change functions
                             int frame16)                       ///< Hardware sequencer frame number, or -1 for ASAP
{
	unsigned long todo, mask, remain;
	int pars_ahead;                 // considering parameter "pars_ahead" of the (frame16+job_ahead) mod 8
	int frame_proc;                 // current frame for which parameters are considered
	struct framepars_t * procpars;
	struct framepars_t * prevpars;  // maybe - drop calculation for each function, move it to pgm_* where needed?
	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long * p_nasap = &GLOBALPARS(sensor_port, G_CALLNASAP);
	int i;
	int rslt;

#ifdef ELPHEL_DEBUG_0
	unsigned long allfunctions = framepars[0].functions  | framepars[1].functions | framepars[2].functions | framepars[3].functions |
				     framepars[4].functions | framepars[5].functions | framepars[6].functions | framepars[7].functions;
	if (allfunctions) MDF6(printk("frame16=%d, functions: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n", frame16, framepars[0].functions,
			framepars[1].functions, framepars[2].functions, framepars[3].functions, framepars[4].functions, framepars[5].functions,
			framepars[6].functions, framepars[7].functions));
#endif
	if (debug_flags)
	    dev_dbg(g_devfp_ptr,"ASAP: port=%d frame16=%d\n",   sensor_port,  frame16);
	else
        dev_dbg(g_devfp_ptr,"ASAP: port=%d frame16=%d\n",   sensor_port,  frame16);

    if (!sensorproc){
        dev_err(g_devfp_ptr,"sensorproc==NULL !!!! port=%d frame16=%d \n", sensor_port,  frame16);
        return;
    }

// do all ASAP tasks (they should not be done ahead of the corresponding interrupt!)
// Now try overdue functions with latencies >=1 and try them in ASAP mode
	for (pars_ahead = 0; pars_ahead <= 4; pars_ahead++ ) {
		frame_proc = (frame16 + pars_ahead) & PARS_FRAMES_MASK;
		procpars  = &framepars[frame_proc];
		prevpars  = &framepars[(frame_proc - 1) & PARS_FRAMES_MASK];
		i = 0;
		mask = 1;
		remain = 0xffffffff;

	    if (debug_flags) {
	        todo = (pars_ahead) ?
	                           (p_nasap[pars_ahead] & (procpars->functions) & remain) :
	                           (procpars->functions & remain);
	        dev_dbg(g_devfp_ptr,"port=%d frame16=%d todo=0x%08lx,p_nasap[%d]=0x%08lx procpars->functions=0x%08lx\n",
	                sensor_port,  frame16, todo, pars_ahead, p_nasap[pars_ahead], procpars->functions);
            MDP(DBGB_FASAP,sensor_port,"frame16=%d todo=0x%08lx,p_nasap[%d]=0x%08lx procpars->functions=0x%08lx\n",
                    frame16, todo, pars_ahead, p_nasap[pars_ahead], procpars->functions)

	    } else {
            todo = (pars_ahead) ?
                               (p_nasap[pars_ahead] & (procpars->functions) & remain) :
                               (procpars->functions & remain);
            dev_dbg(g_devfp_ptr,"port=%d frame16=%d todo=0x%08lx,p_nasap[%d]=0x%08lx procpars->functions=0x%08lx\n",
                    sensor_port,  frame16, todo, pars_ahead, p_nasap[pars_ahead], procpars->functions);
	    }
		while ((todo = (pars_ahead) ?
			       (p_nasap[pars_ahead] & (procpars->functions) & remain) :
			       (procpars->functions & remain) )) {      //none, *1, *2,*3,*4

			while (!(todo & mask)) {                        // skip zeros - todo will stay current (.functions will not change
				i++;
				mask   <<= 1;
				remain <<= 1;
			}
// now (todo & mask) !=0
	        if (debug_flags) {
	            dev_dbg(g_devfp_ptr,"port= %d, todo=0x%08lx (curr=0x%08lx) frame16=%d, pars_ahead=%d, frame_proc=%d i=%d, mask=0x%08lx func=0x%08x\n",
	                    sensor_port, todo, procpars->functions, frame16, pars_ahead, frame_proc, i, mask, (int)sensorproc->pgm_func[i]);
	            MDP(DBGB_FASAP,sensor_port,"todo=0x%08lx (curr=0x%08lx) frame16=%d, pars_ahead=%d, frame_proc=%d i=%d, mask=0x%08lx func=0x%08x\n",
	                    todo, procpars->functions, frame16, pars_ahead, frame_proc, i, mask, (int)sensorproc->pgm_func[i])

	            dev_dbg(g_devfp_ptr,"port= %d, %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	                    sensor_port,
	                    framepars[0].functions, framepars[1].functions, framepars[2].functions, framepars[3].functions,
	                    framepars[4].functions, framepars[5].functions, framepars[6].functions, framepars[7].functions);
                MDP(DBGB_FASAP,sensor_port,"%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
                        framepars[ 0].functions, framepars[ 1].functions, framepars[ 2].functions, framepars[ 3].functions,
                        framepars[ 4].functions, framepars[ 5].functions, framepars[ 6].functions, framepars[ 7].functions,
                        framepars[ 8].functions, framepars[ 9].functions, framepars[10].functions, framepars[11].functions,
                        framepars[12].functions, framepars[13].functions, framepars[14].functions, framepars[15].functions)
	        } else {
	            dev_dbg(g_devfp_ptr,"port= %d, todo=0x%08lx (curr=0x%08lx) frame16=%d, pars_ahead=%d, frame_proc=%d i=%d, mask=0x%08lx func=0x%08x\n",
	                    sensor_port, todo, procpars->functions, frame16, pars_ahead, frame_proc, i, mask, (int)sensorproc->pgm_func[i]);

	            dev_dbg(g_devfp_ptr,"port= %d, %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	                    sensor_port,
	                    framepars[0].functions, framepars[1].functions, framepars[2].functions, framepars[3].functions,
	                    framepars[4].functions, framepars[5].functions, framepars[6].functions, framepars[7].functions);
	        }
			if (sensorproc->pgm_func[i]) {
	            dev_dbg(g_devfp_ptr,"port= %d, Calling GENERIC pgm_func[%d] ASAP, now frame = 0x%lx\n",sensor_port,i,thisFrameNumber(sensor_port));
	            MDP(DBGB_FASAP,sensor_port,"Calling GENERIC pgm_func[%d] ASAP, now frame = 0x%x\n",i,thisFrameNumber(sensor_port))
				rslt = sensorproc->pgm_func[i]    (sensor_port, &(sensorproc->sensor), procpars, prevpars, -1);
			} else rslt = 0;                                        // only sensor-specific function, nothing to do common to all sensors
			if ((rslt >= 0) && (sensorproc->pgm_func[i + 32])) {    // sensor - specific functions, called after the main ones
                dev_dbg(g_devfp_ptr,"port= %d, Calling SENSOR-SPECIFIC  pgm_func[%d] ASAP, now frame = 0x%lx\n",sensor_port,i,thisFrameNumber(sensor_port));
                MDP(DBGB_FASAP,sensor_port,"Calling SENSOR-SPECIFIC  pgm_func[%d] ASAP, now frame = 0x%x\n",i,thisFrameNumber(sensor_port))
				rslt = sensorproc->pgm_func[i + 32] (sensor_port, &(sensorproc->sensor), procpars, prevpars, -1);
			}
			// it was dev_warn - and it AFFECTED other ports.
			if (rslt < 0) dev_dbg(g_devfp_ptr,"port %d: %s:%d:%s - error=%d",sensor_port,  __FILE__, __LINE__, __FUNCTION__, rslt); // Nothing to do with errors here - just report?
			procpars->functions &= ~mask;
			dev_dbg(g_devfp_ptr,"%s : port= %d, .functions=0x%08lx)\n",__func__, sensor_port, procpars->functions);
			i++;
			mask   <<= 1;
			remain <<= 1;
		}
	}
    dev_dbg(g_devfp_ptr,"%s port=%d DONE\n",  __func__, sensor_port);

}

// Next 5 should go in that sequence
//#define G_CALLNASAP    119 // bitmask - what functions can be used not only in the current frame (ASAP) mode
//#define G_CALLNEXT1      120 // bitmask of actions to be one   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT2..G_CALLNEXT4)
//#define G_CALLNEXT2      121 // bitmask of actions to be two   or more frames ahead of the programmed one (OR-ed with G_CALLNEXT3..G_CALLNEXT4)
//#define G_CALLNEXT3      122 // bitmask of actions to be three or more frames ahead of the programmed one (OR-ed with G_CALLNEXT4)
//#define G_CALLNEXT4      123 // bitmask of actions to be four  or more frames ahead of the programmed one

/** Process parameters in "normal way" - not ASAP or overdue
 * Called twice from processPars - at the beginning and at the end to finish off any derivatives (needed?)
 * Should never be called from outside processPars() where there is a per-port lock */

inline void _processParsSeq(int sensor_port,                   ///< sensor port number (0..3)
                            struct sensorproc_t * sensorproc,  ///< per-port array of sensor capabilities and 32+32 on-change functions
                            int frame16,                       ///< Hardware sequencer frame number, or -1 for ASAP
                            int maxahead)                      ///< Maximal number of frames ahead to process. TODO NC393: Set G_MAXAHEAD >0 (max latency, was 2 in NC353!),
{
	unsigned long todo, mask, remain;
	int job_ahead;                  // doing job "job_ahead" ahead of needed
	int pars_ahead;                 // considering parameter "pars_ahead" of the (frame16+job_ahead) mod 16
	int frame_proc;                 // current frame for which parameters are considered
	struct framepars_t * procpars;
	struct framepars_t * prevpars;  // maybe - drop calculation for each function, move it to pgm_* where needed?
	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long * p_nasap = &GLOBALPARS(sensor_port, G_CALLNASAP);
	int seq_frame;                  // sequencer frame for which pgm_* function should schedule data
	int i;
	int rslt;
	int max_par_ahead;
	int this_ahead;
    dev_dbg(g_devfp_ptr,"%s port=%d frame16=%d,  maxahead=%d\n",  __func__, sensor_port,  frame16, maxahead);

	if (maxahead > (PARS_FRAMES - 3)) maxahead = PARS_FRAMES - 3;  // use 5 if maxahead >5
// commands that use FPGA queues for the i2c/sequencer commands, executed at frame syncs
// Modifying - as soon as found the frame to process with non-zero masked .functions - process all functions for that
// frame with appropriate sequencer frame.
// For now - scan p_nasap[i] to find latency - improve that later
	for (job_ahead = 0; job_ahead <= maxahead; job_ahead++ ) { // If have some time - do future job (not required just now)
		max_par_ahead = min(5, (PARS_FRAMES - 3) - job_ahead);
		for (pars_ahead = 0; pars_ahead < max_par_ahead; pars_ahead++ ) {
			frame_proc = (frame16 + job_ahead + pars_ahead + 1) & PARS_FRAMES_MASK; //
			procpars  = &framepars[frame_proc];
// Check if at least one function is needed for frame_proc
			if (procpars->functions &
			    p_nasap[pars_ahead] & //all, *1, *2,*3,*4 - for all will have G_CALLNASAP twice
			    p_nasap[0]) {
				prevpars  = &framepars[(frame_proc - 1) & PARS_FRAMES_MASK];
//        seq_frame=  (frame16+job_ahead+1) & PARS_FRAMES_MASK;
				i = 0;
				mask = 1;
				remain = 0xffffffff;
				while ((todo = procpars->functions &
//                     p_nasap[pars_ahead] &  //all, *1, *2,*3,*4 - for all will have G_CALLNASAP twice
					       p_nasap[0] & remain)) {  // eliminate ASAP-only function
					while (!(todo & mask)) {        // skip zeros - todo will stay current (.functions will not change)
						i++;
						mask   <<= 1;
						remain <<= 1;
					}
// now (todo & mask) !=0
// find the right latency
					for (this_ahead = 1; (p_nasap[this_ahead] & todo & mask) && (this_ahead <= 4); this_ahead++) ;  // this_ahead==1..5
//          seq_frame=  (frame16 + job_ahead + this_ahead) & PARS_FRAMES_MASK;
					seq_frame =  (frame_proc + 1 - this_ahead) & PARS_FRAMES_MASK;

					dev_dbg(g_devfp_ptr,"%s port=%d todo=0x%08lx (curr=0x%08lx) frame16=%d, frame_proc=%d, seq_frame=%d, i=%d, mask=0x%08lx\n",
					        __func__, sensor_port, todo, procpars->functions, frame16, frame_proc, seq_frame, i, mask);
					dev_dbg(g_devfp_ptr,"%s port=%d %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
					        __func__, sensor_port, framepars[0].functions, framepars[1].functions, framepars[2].functions, framepars[3].functions, framepars[4].functions, framepars[5].functions, framepars[6].functions, framepars[7].functions);

					if (sensorproc->pgm_func[i]) {
						// NOTE: Was (frame16+job_ahead +1) & PARS_FRAMES_MASK
                        dev_dbg(g_devfp_ptr,"port= %d, Calling GENERIC pgm_func[%d], seq_frame = 0x%x, now frame = 0x%lx\n",sensor_port,i,seq_frame,thisFrameNumber(sensor_port));
                        MDP(DBGB_FSEQ,sensor_port,"Calling GENERIC pgm_func[%d],  seq_frame = 0x%x, now frame = 0x%x\n",i,seq_frame,thisFrameNumber(sensor_port))
						rslt = sensorproc->pgm_func[i]    (sensor_port, &(sensorproc->sensor), procpars, prevpars, seq_frame);
					} else rslt = 0;                                        // only sensor-specific function, nothing to do common to all sensors
					if ((rslt >= 0) && (sensorproc->pgm_func[i + 32])) {    // sensor - specific functions, called after the main ones
                        dev_dbg(g_devfp_ptr,"port= %d, Calling SENSOR-SPECIFIC pgm_func[%d], seq_frame = 0x%x, now frame = 0x%lx\n",sensor_port,i,seq_frame,thisFrameNumber(sensor_port));
                        MDP(DBGB_FSEQ,sensor_port,"Calling SENSOR-SPECIFIC pgm_func[%d],  seq_frame = 0x%x, now frame = 0x%x\n",i,seq_frame,thisFrameNumber(sensor_port))
					    rslt = sensorproc->pgm_func[i + 32] (sensor_port, &(sensorproc->sensor), procpars, prevpars, seq_frame);
					}
					if (rslt >= 0) {
						procpars->functions &= ~mask; // mark it done
					} else {
						dev_dbg(g_devfp_ptr,"%s port %d: Error - function result was %d\n",__func__, sensor_port, rslt);
					}
					i++;
					mask   <<= 1;
					remain <<= 1;
				}
			}
		}
	}
}

/**
 * @brief Program image acquisition, according to the parameters changed
 * Called from tasklet and user (setFrameParsAtomic) and LSEEK_SENSORPROC
 * @param sensor_port sensor port number (0..3)
 * @param sensorproc pointer to sensor static parameters and functions
 * @param frame16     current hardware frame number
 * @param maxahead   maximal number of frames to program ahead of the current (make it  P_* parameter ?)
 * @return always 0 ?
 */
//TODO: "Do it later" should be the only reason not to erase todo bit
//#define P_CALLASAP       107 // bitmask - what functions work only in the current frame (ASAP) mode
void _processPars(int sensor_port, struct sensorproc_t * sensorproc, int frame16, int maxahead)
{
    frame16 &= PARS_FRAMES_MASK;
    if (debug_flags) {
        dev_dbg(g_devfp_ptr,"port= %d,  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
        MDP(DBGB_FPPI,sensor_port,"frame16=%d, maxahead=%d\n",
                frame16, maxahead)
    }
    dev_dbg(g_devfp_ptr,"port= %d,  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    if (!sensorproc){
        dev_err(g_devfp_ptr,"port=%d frame16=%d sensorproc==NULL !!!! \n", sensor_port,  frame16);
        return;
    }
    // Check if master channel updated trigger parameters, schedule them to be updated
    trigSlaveUpdate(sensor_port); // that will possible schedule more parameters
    //    int spin_trylock(spinlock_t *lock);
    // first - do all ASAP tasks (they should not be done ahead of the corresponding interrupt!)
    //   dev_dbg(g_devfp_ptr,"%s before first _processParsASAP\n",__func__);

    _processParsASAP(sensor_port, sensorproc, frame16); // NC393: never gets here ? Only after _processParsSeq?

    if (debug_flags) {
        MDP(DBGB_FPPI,sensor_port,"(after first _processParsASAP),  frame16=%d, maxahead=%d\n",
                frame16, maxahead)
        dev_dbg(g_devfp_ptr,"port= %d (after first _processParsASAP),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    }
    dev_dbg(g_devfp_ptr,"port= %d (after first _processParsASAP),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);

    // now - the rest commands that use FPGA queues for the i2c/sequencer commands, executed at frame syncs
    // for jobahead =0 it is still possible to have some functions in ASAP mode with non-zero latency
    //   dev_dbg(g_devfp_ptr,"%s before _processParsSeq\n",__func__);
    _processParsSeq(sensor_port, sensorproc, frame16, maxahead);
    if (debug_flags) {
        MDP(DBGB_FPPI,sensor_port,"(after _processParsSeq),  frame16=%d, maxahead=%d\n",
                frame16, maxahead)
        dev_dbg(g_devfp_ptr,"port= %d (after _processParsSeq),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    }
    dev_dbg(g_devfp_ptr,"port= %d (after _processParsSeq),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);

    // re-test ASAP tasks - they might appear as a result of other commands executed
    //   dev_dbg(g_devfp_ptr,"%s before second _processParsASAP\n",__func__);
    _processParsASAP(sensor_port, sensorproc, frame16);
    if (debug_flags) {
        MDP(DBGB_FPPI,sensor_port,"(after second _processParsASAP),  frame16=%d, maxahead=%d\n",
                frame16, maxahead)
        dev_dbg(g_devfp_ptr,"port= %d (after second _processParsASAP),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    }
    dev_dbg(g_devfp_ptr,"port= %d (after second _processParsASAP),  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    if (debug_flags)   {
        dev_dbg(g_devfp_ptr,"debug_flags= %d \n", debug_flags);
        debug_flags--;
    }

}

#if 0
void processPars(int sensor_port, struct sensorproc_t * sensorproc, int frame16, int maxahead)
{
    frame16 &= PARS_FRAMES_MASK;
    dev_dbg(g_devfp_ptr,"port= %d,  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    if (!sensorproc){
        dev_err(g_devfp_ptr,"%s port=%d frame16=%d sensorproc==NULL !!!! \n",  __func__, sensor_port,  frame16);
        return;
    }
#ifdef LOCK_BH_PROCESSPARS
    spin_lock_bh(framepars_locks[sensor_port]);
    // WARNING: CPU: 1 PID: 2329 at /home/eyesis/git/elphel393/poky/build/tmp/work-shared/elphel393/kernel-source/kernel/softirq.c:150 __local_bh_enable_ip+0xb0/0x100()
#else
    // Here we can get from both a tasklet and from LSEEK_SENSORPROC. We do not need to do this twice - if processPars() is ran by
    // somebody else - that is OK not to do it again (what about maxahead?)
    if (!spin_trylock(framepars_locks[sensor_port])) {
        dev_dbg(g_devfp_ptr,"framepars_locks[%d] is locked, we do not need to re-run it. frame16=%d maxahead=%d \n",  sensor_port,  frame16, maxahead);
        return;
    }
#endif
    _processPars(sensor_port, sensorproc, frame16, maxahead);
#ifdef    LOCK_BH_PROCESSPARS
    spin_unlock_bh(framepars_locks[sensor_port]); // removed, see above
#else
    spin_unlock(framepars_locks[sensor_port]);
#endif
}
#else
int processPars(int sensor_port, struct sensorproc_t * sensorproc, int frame16, int maxahead)
{
    FLAGS_IBH
    frame16 &= PARS_FRAMES_MASK;
    if (debug_flags) {
        MDP(DBGB_FPPT,sensor_port,"==from tasklet: frame16=%d, maxahead=%d\n",
                frame16, maxahead)
        dev_dbg(g_devfp_ptr,"==from tasklet: port= %d,  frame16=%d, maxahead=%d  now=0x%lx\n", sensor_port, frame16, maxahead, getThisFrameNumber(sensor_port));
    }

    dev_dbg(g_devfp_ptr,"port= %d,  frame16=%d, maxahead=%d\n", sensor_port, frame16, maxahead);
    if (!sensorproc){
        dev_err(g_devfp_ptr,"%s port=%d frame16=%d sensorproc==NULL !!!! \n",  __func__, sensor_port,  frame16);
        return -ENODEV;
    }
    LOCK_IBH(framepars_locks[sensor_port]);
    _processPars(sensor_port, sensorproc, frame16, maxahead);
    UNLOCK_IBH(framepars_locks[sensor_port]);
    if (debug_flags) {
        MDP(DBGB_FPPT,sensor_port,"==Done from tasklet: frame16=%d, maxahead=%d\n",
                frame16, maxahead)
        dev_dbg(g_devfp_ptr,"== Done from tasklet: port= %d,  frame16=%d, maxahead=%d now=0x%lx\n", sensor_port, frame16, maxahead, getThisFrameNumber(sensor_port));
    }
    return 0;
}
#endif

/**
 * @brief schedule pgm_func to be executed for selected frame (frame16)
 * @param sensor_port sensor port number (0..3)
 * @param frame16 frame number (4-bit) to schedule a function for
 * @param func_num function number to schedule
 */
void schedule_pgm_func(int sensor_port, int frame16, int func_num)
{
	aframepars[sensor_port][frame16 & PARS_FRAMES_MASK].functions |= 1 << func_num;
    dev_dbg(g_devfp_ptr,"func_num=%d, aframepars[%d][%d].functions=0x%08lx\n",
            func_num, sensor_port, frame16, aframepars[sensor_port][frame16 & PARS_FRAMES_MASK].functions);
    MDP(DBGB_FSCF,sensor_port,"func_num=%d, aframepars[%d][%d].functions=0x%08lx\n",
            func_num, sensor_port, frame16,aframepars[sensor_port][frame16 & PARS_FRAMES_MASK].functions)

}

/**
 * @brief schedule pgm_func to be executed for this_framepars->pars[P_FRAME] & PARS_FRAMES_MASK
 * @param sensor_port sensor port number (0..3)
 * @param this_framepars pointer to frame parameters structure
 * @param func_num number of function to schedule
 */
void schedule_this_pgm_func(int sensor_port, struct framepars_t * this_framepars, int func_num)
{
	int frame16 = this_framepars->pars[P_FRAME] & PARS_FRAMES_MASK;
	aframepars[sensor_port][frame16].functions |= 1 << func_num;
    dev_dbg(g_devfp_ptr,"func_num=%d, aframepars[%d][%d].functions=0x08%lx\n",
            func_num, sensor_port, frame16, aframepars[sensor_port][frame16 & PARS_FRAMES_MASK].functions);
    MDP(DBGB_FSCF,sensor_port,"func_num=%d, aframepars[%d][%d].functions=0x08%lx\n",
            func_num, sensor_port, frame16,aframepars[sensor_port][frame16 & PARS_FRAMES_MASK].functions)
}


/**
 * @brief just return current thisFrameNumber
 * @param sensor_port sensor port number (0..3)
 * @return current value of thisFrameNumber
 */
unsigned long getThisFrameNumber(int sensor_port)
{
	return thisFrameNumber(sensor_port);
}


/**
 * @brief Set a single parameter to all frames (during sensor detection)
 * @param sensor_port sensor port number (0..3)
 * @param numPars number of parameters to set
 * @param pars array of parameters (number/value pairs)
 * @return always 0
 */
int setFrameParStatic(int sensor_port,                     ///< sensor port number (0..3)
                      unsigned long index,                 ///< parameter number
                      unsigned long val)                   ///< parameter value to set
                                                           ///< @return 0 - OK, -ERR_FRAMEPARS_BADINDEX

{
    struct framepars_t *framepars = aframepars[sensor_port];
    int nframe;
    index &= 0xffff;  // get rid of any modifier (not applicable here)
	if (index > P_MAX_PAR) return -ERR_FRAMEPARS_BADINDEX;
    for (nframe = 0; nframe < PARS_FRAMES; nframe++) {
        framepars[nframe].pars[index] = val;
	}
	return 0;
}

/**
 * @brief Set parameters that will never change (usually after sensor discovery), other fields are supposed to be cleared
 */
int setFrameParsStatic(int sensor_port,               ///< sensor_port sensor port number (0..3)
                       int numPars,                   ///< numPars number of parameters to set
                       struct frameparspair_t * pars) ///< pars array of parameters (number/value pairs)
                                                      ///< @return always 0
{
    int npar, nframe, index;
    struct framepars_t *framepars = aframepars[sensor_port];
    for (npar = 0; npar < numPars; npar++) {
        index = pars[npar].num & 0xffff; // get rid of any modifier (not applicable here)
        if (index > P_MAX_PAR) return -ERR_FRAMEPARS_BADINDEX;
        for (nframe = 0; nframe < PARS_FRAMES; nframe++) {
            framepars[nframe].pars[index] = pars[npar].val;
        }
    }
    return 0;
}


/** Set parameters for the specified frame  (atomic, with interrupts off). Used from applications through driver write */
//TODO: Check that writes never to the future or past frame (only 6 of 8 are allowed -> 14 of 16). Have seen just_this to flood all
int setFrameParsAtomic(int sensor_port,               ///< sensor port number (0..3)
                       unsigned long frameno,         ///< absolute (full) frame number parameters should be applied to
                                                      ///< frameno = 0xffffffff => use maxlatency -1, frame = 0
                       int maxLatency,                ///< maximal command latency (parameters should be set not less than maxLatency ahead of the current frame)
                                                      ///< maxLatency < 0 - don't check latency (i.e. only commands that are not releted to particular frames),
                                                      ///< with negative and frameno< current frame will make it current, to use with ASAP
                       int numPars,                   ///< number of parameters to set (0 is OK to just test if it is too early/too late)
                       struct frameparspair_t * pars) ///< array of parameters (number/value pairs). FRAMEPAIR_FORCE_NEW modifier to parameter number
                                                      ///< @return 0 - OK, -ERR_FRAMEPARS_TOOEARLY, -ERR_FRAMEPARS_TOOLATE
{
    FLAGS_IBH
    int npar, nframe, res;
	unsigned long val, bmask, bmask32;
	int index, bindex;
	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long      *funcs2call =afuncs2call[sensor_port];
	int findex_this, findex_prev, findex_future, frame16;
	if (frameno == 0xffffffff){
	    maxLatency = -1;
	    frameno = 0;
	    dev_dbg(g_devfp_ptr,"port= %d, frameno was 0xffffffff, modifying maxLatency=0x%x, frameno = 0x%08lx\n",sensor_port, maxLatency, frameno);
	}

    findex_this =  thisFrameNumber(sensor_port) & PARS_FRAMES_MASK;
    findex_prev = (findex_this - 1)  & PARS_FRAMES_MASK;
    findex_future = (findex_this - 2)  & PARS_FRAMES_MASK; // actually - fartherst in the future??
	MDP(DBGB_FSFA,sensor_port,"frameno=0x%lx, findex_this=%ld (0x%lx) maxLatency=%d, numPars=%d, frameParsInitialized[%d]=%d\n",
            frameno, findex_this, thisFrameNumber(sensor_port), maxLatency, numPars, sensor_port, frameParsInitialized[sensor_port])
    dev_dbg(g_devfp_ptr,"port= %d, frameno=0x%lx, findex_this=%d (0x%lx) maxLatency=%d, numPars=%d, frameParsInitialized[%d]=%d\n",
            sensor_port, frameno, findex_this, thisFrameNumber(sensor_port), maxLatency, numPars, sensor_port, frameParsInitialized[sensor_port]);
    //int klog393_ts(const char * str);

	if (!frameParsInitialized[sensor_port]) {
		res = initSequencers(sensor_port); // Will call  initFramePars(); and initialize functions
		if (res <0) return res;
	}
	LOCK_IBH(framepars_locks[sensor_port]);
	PROFILE_NOW(5); // Was 6, but no 7 in NC393
	if (maxLatency >= 0) {
		if (frameno <= (thisFrameNumber(sensor_port) + maxLatency)) {
		    UNLOCK_IBH(framepars_locks[sensor_port]);
		    dev_dbg(g_devfp_ptr,"port=%d, ERR_FRAMEPARS_TOOLATE, frameno = 0x%x\n",sensor_port, (int)frameno);
			return -ERR_FRAMEPARS_TOOLATE;
		}else if (frameno >= (thisFrameNumber(sensor_port) + (PARS_FRAMES - 1))) {
            UNLOCK_IBH(framepars_locks[sensor_port]);
            dev_dbg(g_devfp_ptr,"port=%d, ERR_FRAMEPARS_TOOEARLY, frameno = 0x%x\n",sensor_port, (int)frameno);
			return -ERR_FRAMEPARS_TOOEARLY;
		}
	} else {
        dev_dbg(g_devfp_ptr,"port=%d, Using negative maxLatency, frameno = 0x%x, thisFrameNumber(%d) = 0x%x\n",
                sensor_port, (int)frameno, sensor_port, (int) thisFrameNumber(sensor_port));
        if (frameno < thisFrameNumber(sensor_port)){
            dev_dbg(g_devfp_ptr,"port=%d, Replacing earlier supplied frame number = 0x%x with the current one= 0x%x\n",
                    sensor_port, (int)frameno, (int) thisFrameNumber(sensor_port));
            frameno = thisFrameNumber(sensor_port);
        }
	}
	// not too late, not too early, go ahead (or maxlatency <0 - ASAP only)
	for (npar = 0; npar < numPars; npar++) {
	    dev_dbg(g_devfp_ptr,"port= %d,  --pars[%d].num=0x%lx, pars[%d].val=0x%lx\n", sensor_port, npar, pars[npar].num, npar, pars[npar].val);
        MDP(DBGB_FSFA,sensor_port,"  --pars[%d].num=0x%lx, pars[%d].val=0x%lx\n", npar, pars[npar].num, npar, pars[npar].val)

//    frame16=      (pars[npar].num & FRAMEPAR_GLOBALS)? -1: (frameno  & PARS_FRAMES_MASK);
		frame16 =      frameno  & PARS_FRAMES_MASK;
		val = pars[npar].val;
		index = pars[npar].num & 0xffff;
		if (index > ((index >= FRAMEPAR_GLOBALS) ? (P_MAX_GPAR + FRAMEPAR_GLOBALS) : P_MAX_PAR)) {
            UNLOCK_IBH(framepars_locks[sensor_port]);
            dev_dbg(g_devfp_ptr,"port=%d, ERR_FRAMEPARS_BADINDEX, frameno = 0x%x\n",sensor_port, (int)frameno);
			return -ERR_FRAMEPARS_BADINDEX;
		}
		dev_dbg(g_devfp_ptr,"port= %d, index=0x%x, val=0x%lx", sensor_port, index, val);
		if (index >= FRAMEPAR_GLOBALS) {                        // ignore frame logic, set "static" parameters to frame 0
			if (pars[npar].num & FRAMEPAIR_MASK_BYTES) {    // combine new value with the old one
				val = FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, GLOBALPARS(sensor_port,index), val);
			}
			GLOBALPARS(sensor_port, index) = val;
			D5(printk(" set GLOBALPARS(0x%x)=0x%lx\n", index, val));
		} else if (pars[npar].num  & FRAMEPAIR_FRAME_FUNC) {
			funcs2call[index] = val;
			dev_dbg(g_devfp_ptr,"port= %d, set funcs2call[0x%x]=0x%lx\n", sensor_port, index, val);
//    } else if ((frameno !=findex_prev) && (frameno != findex_future)) { // do not write parameters in the future otherwise
		} else if ((frame16 != findex_future) || ((pars[npar].num & FRAMEPAIR_JUST_THIS) == 0)) {        // do not write "JUST_THIS" parameters in the future otherwise they'll stick
			if (pars[npar].num & FRAMEPAIR_MASK_BYTES) {                                            // combine new value with the old one
				val = FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, framepars[frame16].pars[index], val);
			}
//TODO: optimize to use mask several parameters together
			dev_dbg(g_devfp_ptr,"port= %d, frame16=0x%x\n", sensor_port, frame16);
	        MDP(DBGB_FSFA,sensor_port,"framepars[%d].pars[%d]=0x%lx =?= val = 0x%lx, pars[%d].num=0x%08lx & 0x%08x\n",
	                frame16, index, framepars[frame16].pars[index], val, npar, pars[npar].num, FRAMEPAIR_FORCE_NEW)
			if ((framepars[frame16].pars[index] != val) || (pars[npar].num & FRAMEPAIR_FORCE_NEW)) {
				bmask =   1 << (index & 31);
				bindex = index >> 5;
				bmask32 = 1 << bindex;
//   Set this parameter for specified frame
				framepars[frame16].pars[index]       = val;
				framepars[frame16].mod[bindex]      |= bmask;
				framepars[frame16].mod32            |= bmask32;
				framepars[frame16].functions        |= funcs2call[index]; //Mark which functions will be needed to process the parameters
				dev_dbg(g_devfp_ptr,"port= %d,  bindex=0x%x, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",
				        sensor_port, bindex, bmask, bmask32, framepars[frame16].functions);
                MDP(DBGB_FSFA,sensor_port,"bindex=0x%x, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",
                        bindex, bmask, bmask32, framepars[frame16].functions)

// Write parameter to the next frames up to the one that have the same parameter already modified (only if not FRAMEPAIR_JUST_THIS)
				if ((pars[npar].num & FRAMEPAIR_JUST_THIS) == 0) {
					dev_dbg(g_devfp_ptr,"port= %d,       ---   setting next frames, pars[%d].num=0x%lx\n",sensor_port,npar,pars[npar].num);
                    MDP(DBGB_FSFA,sensor_port,"    ---   setting next frames, pars[%d].num=0x%lx\n",npar,pars[npar].num)

					for (nframe = (frame16 + 1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe = (nframe + 1) & PARS_FRAMES_MASK) {
						framepars[nframe].pars[index] = val;
						dev_dbg(g_devfp_ptr,"framepars[%d].pars[%d] <- 0x%08lx  ", nframe, index, val);
	                    MDP(DBGB_FSFA,sensor_port,"framepars[%d].pars[%d] <- 0x%08lx  ", nframe, index, val)
					}
					frame16 = (frame16 - 1) & PARS_FRAMES_MASK; // for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
                    dev_dbg(g_devfp_ptr,"\n");
	                MDP(DBGB_FSFA,sensor_port,"%s\n","")
//					D5(printk("\n"));
				}
// Mark this parameter in all previous frames as "modified since"
// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
				for (nframe = frame16; nframe != findex_future; nframe = (nframe - 1) & PARS_FRAMES_MASK) { //NOTE: frame16 is modified here
					framepars[nframe].modsince[bindex] |= bmask;
					framepars[nframe].modsince32       |= bmask32;
	                MDP(DBGB_FSFA,sensor_port,"framepars[%d].modsince[%d] |= 0x%08x, framepars[%d].modsince32 |= 0x%08x  ",
	                        nframe, bindex, bmask, nframe, bmask32)
				}
                MDP(DBGB_FSFA,sensor_port,"%s\n","")
			}
		} else { // error - trying to write "just this" to the "future" - that would stick if allowed
            UNLOCK_IBH(framepars_locks[sensor_port]);
            dev_dbg(g_devfp_ptr,"port= %d, Tried to write JUST_THIS parameter (0x%lx) too far in the future", sensor_port, pars[npar].num);
            dev_err(g_devfp_ptr,"port= %d, Tried to write JUST_THIS parameter (0x%lx) too far in the future", sensor_port, pars[npar].num);
			return -ERR_FRAMEPARS_TOOEARLY;
		}
	}
// Try to process parameters immediately after written. If 0, only non-ASAP will be processed to prevent
// effects of uncertainty of when was it called relative to frame sync

// ASAP - needed to set with sequencer is stopped!

// Changed to all (don't care about uncertainty - they will trigger only if it is too late or during sensor detection/initialization)
	debug_flags = 20; // enable debug print several times
	if (!(get_globalParam(sensor_port, G_TASKLET_CTL) & (1 << TASKLET_CTL_NOSAME))) {
//    _processParsSeq (sensorproc, thisFrameNumber & PARS_FRAMES_MASK, 0); //maxahead=0, the rest will be processed after frame sync, from the tasklet
		dev_dbg(g_devfp_ptr,"G_TASKLET_CTL -> 0x%x (port = %d )\n", (int) get_globalParam(sensor_port, G_TASKLET_CTL),sensor_port);
//		processPars(sensor_port, &asensorproc[sensor_port], thisFrameNumber(sensor_port) & PARS_FRAMES_MASK, 0); //maxahead=0, the rest will be processed after frame sync, from the tasklet
		// Already having lock, call inner function. When called from tasklet, it will have to acquire lock
		MDP(DBGB_FSFA,sensor_port,"G_TASKLET_CTL -> 0x%x\n",
	            (int) get_globalParam(sensor_port, G_TASKLET_CTL))
        _processPars(sensor_port, &asensorproc[sensor_port], thisFrameNumber(sensor_port) & PARS_FRAMES_MASK, 0); //maxahead=0, the rest will be processed after frame sync, from the tasklet
        MDP(DBGB_FSFA,sensor_port,"kthread _processPars( .., 0x%x) DONE\n",
                thisFrameNumber(sensor_port))
        dev_dbg(g_devfp_ptr,"kthread _processPars(%d, .., 0x%lx) DONE\n",sensor_port, thisFrameNumber(sensor_port));
	} else {
        dev_dbg(g_devfp_ptr,"kthread: NOT calling _processPars(%d, .., 0x%lx) DONE\n",sensor_port, thisFrameNumber(sensor_port));
	}
	PROFILE_NOW(6); // 7); // no (7) in NC393
    UNLOCK_IBH(framepars_locks[sensor_port]);
	return 0;
}
//#define FRAMEPAIR_JUST_THIS     0x40000 // write only to this frame, don't propagate
// (like "single frame" - compressor, sensor) first write "stop", then - "single" with FRAMEPAIR_JUST_THIS

/** Set a single output (calculated) parameter for the frame referenced by this_framepars structure.
 * Schedules action only if the FRAMEPAIR_FORCE_PROC modifier bit is set in mindex
 * Should only be called when IRQ (just bh preferrably) is off and lock on framepars_locks[sensor_port] (all pgm_* have that)  */
int setFramePar(int sensor_port,                     ///< sensor port number (0..3)
                struct framepars_t * this_framepars, ///< pointer to the current parameters structure
                unsigned long mindex,                ///< parameter number (with optional modifiers in high bits)
                unsigned long val)                   ///< parameter value to set
                                                     ///< @return 0 - OK, -ERR_FRAMEPARS_BADINDEX
{
	int frame16 =  (this_framepars->pars[P_FRAME]) & PARS_FRAMES_MASK;
	//  unsigned long flags; should only be called when interruypts disabled and lock obtained
	int nframe;
	unsigned long bmask, bmask32, bindex;
	int findex_this =  thisFrameNumber(sensor_port) & PARS_FRAMES_MASK;
	int findex_prev = (findex_this - 1)  & PARS_FRAMES_MASK;
	int findex_future = (findex_this - 2)  & PARS_FRAMES_MASK;
	int index =    mindex & 0xffff;
	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long      *funcs2call =afuncs2call[sensor_port];

	dev_dbg(g_devfp_ptr, "port=%d, thisFrameNumber=0x%lx frame16=%d index= %d (0x%lx), val=0x%lx\n",
	        sensor_port, thisFrameNumber(sensor_port), frame16, index, mindex, val);
    MDP(DBGB_FSFP,sensor_port,"thisFrameNumber=0x%lx frame16=%d index= %d (0x%lx), val=0x%lx\n",
            thisFrameNumber(sensor_port), frame16, index, mindex, val)


//  if (index > P_MAX_PAR) {
	if (index > ((index >= FRAMEPAR_GLOBALS) ? (P_MAX_GPAR + FRAMEPAR_GLOBALS) : P_MAX_PAR)) {
		return -ERR_FRAMEPARS_BADINDEX;
	}
//TODO: optimize to use mask several parameters together
	if (index >= FRAMEPAR_GLOBALS) {                // ignore frame logic, set "static" parameters to frame 0
		if (mindex & FRAMEPAIR_MASK_BYTES) {    // combine new value with the old one
			val = FRAMEPAIR_FRAME_MASK_NEW(mindex, GLOBALPARS(sensor_port,index), val);
		}
		GLOBALPARS(sensor_port,index) = val;
	} else if (mindex  & FRAMEPAIR_FRAME_FUNC) { // write to func_proc[] instead
		funcs2call[index] = val;
	} else if ((frame16 != findex_future) || ((mindex & FRAMEPAIR_JUST_THIS) == 0)) {        // do not write "JUST_THIS" parameters in the future otherwise they'll stick
		if (mindex & FRAMEPAIR_MASK_BYTES) {                                            // combine new value with the old one
			val = FRAMEPAIR_FRAME_MASK_NEW(mindex, framepars[frame16].pars[index], val);
		    dev_dbg(g_devfp_ptr, "val updated to 0x%lx\n", val);
		    MDP(DBGB_FSFP,sensor_port,"val updated to 0x%lx\n", val)

		}
		if ((framepars[frame16].pars[index] != val) || (mindex & (FRAMEPAIR_FORCE_NEW | FRAMEPAIR_FORCE_PROC))) {
			bmask =   1 << (index & 31);
			bindex = index >> 5;
			bmask32 = 1 << bindex;
//   Set this parameter for specified frame, (for now - unconditionally mark as modified, even if the value is the same as it was - CHANGED!
			framepars[frame16].pars[index]       = val;
			framepars[frame16].mod[bindex]      |= bmask;
			framepars[frame16].mod32            |= bmask32;
			if (mindex & FRAMEPAIR_FORCE_PROC) {
				framepars[frame16].functions        |= funcs2call[index]; //Mark which functions will be needed to process the parameters
			}
			dev_dbg(g_devfp_ptr, " bindex=0x%lx, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",
			        bindex, bmask, bmask32, framepars[frame16].functions);
            MDP(DBGB_FSFP,sensor_port," bindex=0x%lx, bmask=0x%08lx, bmask32=0x%08lx, functions=0x%08lx\n",
                    bindex, bmask, bmask32, framepars[frame16].functions)
// Write parameter to the next frames up to the one that have the same parameter already modified
			if ((mindex & FRAMEPAIR_JUST_THIS) == 0) {
//				MDF8(printk(":        ---   setting next frames"));
//        for (nframe=(frame16+1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[frame16].mod[bindex] & bmask)); nframe=(nframe+1) & PARS_FRAMES_MASK) {
				for (nframe = (frame16 + 1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe = (nframe + 1) & PARS_FRAMES_MASK) {
					framepars[nframe].pars[index] = val;
//					D8(printk(" %d", nframe));
				    MDP(DBGB_FSFV,sensor_port,"framepars[%d].pars[%d] <- 0x%08x  ", nframe, index, val)
				}
				frame16 = (frame16 - 1) & PARS_FRAMES_MASK; // for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
                MDP(DBGB_FSFV,sensor_port,"%s\n","")
			}
//      dev_dbg(g_devfp_ptr,"%s \n",__func__);
// Mark this parameter in all previous frames as "modified since"
// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
//			MDF8(printk(":        >>>   setting modsince"));
//    for (nframe=(frame16-1) & PARS_FRAMES_MASK; nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) {
			for (nframe = frame16; nframe != findex_future; nframe = (nframe - 1) & PARS_FRAMES_MASK) { //NOTE: frame16 is modified here
				framepars[nframe].modsince[bindex] |= bmask;
				framepars[nframe].modsince32       |= bmask32;
                MDP(DBGB_FSFV,sensor_port,"framepars[%d].modsince[%d] |= 0x%08x, framepars[%d].modsince32 |= 0x%08x  ",
                        nframe, bindex, bmask, nframe, bmask32)
//				D8(printk(" %d", nframe));
			}
            MDP(DBGB_FSFV,sensor_port,"%s\n", "")
//			D8(printk("\n"));
		}
	} else { // error - trying to write "just this" to the "future" - that would stick if allowed
        dev_dbg(g_devfp_ptr, "Tried to write JUST_THIS parameter (0x%lx) too far in the future", mindex);
		dev_err(g_devfp_ptr, "Tried to write JUST_THIS parameter (0x%lx) too far in the future", mindex);
        MDP(DBGB_FSFP,sensor_port,"Tried to write JUST_THIS parameter (0x%lx) too far in the future", mindex)
		return -ERR_FRAMEPARS_TOOEARLY;
	}
	return 0;
}

/** Set a single output (calculated) parameter for the frame referenced by this_framepars structure from the user program
 * Obtain lock, disable IRQ and  call setFramePar
 * Schedules action only if the FRAMEPAIR_FORCE_PROC modifier bit is set in mindex
 * Should only be called when IRQ (just bh preferrably) is off and lock on framepars_locks[sensor_port] (all pgm_* have that)  */
int setFrameParLocked(int sensor_port,                     ///< sensor port number (0..3)
                      struct framepars_t * this_framepars, ///< pointer to the current parameters structure
                      unsigned long mindex,                ///< parameter number (with optional modifiers in high bits)
                      unsigned long val)                   ///< parameter value to set
                                                           ///< @return 0 - OK, -ERR_FRAMEPARS_BADINDEX
{
    FLAGS_IBH
    int rslt;
    LOCK_IBH(framepars_locks[sensor_port]);
    MDP(DBGB_FSFP,sensor_port,"mindex=0x%08lx, val=0x%08lx", mindex,val)
    rslt =  setFramePar(sensor_port,    // sensor port number (0..3)
                        this_framepars, // pointer to the current parameters structure
                        mindex,         // parameter number (with optional modifiers in high bits)
                        val);           // parameter value to set
                                        // @return 0 - OK, -ERR_FRAMEPARS_BADINDEX
    UNLOCK_IBH(framepars_locks[sensor_port]);
    MDP(DBGB_FSFP,sensor_port,"DONE: mindex=0x%08lx, val=0x%08lx", mindex,val)
    return rslt;
}

void trigSlaveUpdate(int sensor_port)  ///< sensor port number (0..3)
{
    struct framepars_t *framepars =  aframepars[sensor_port];
    struct frameparspair_t pars_to_update[8];
    int nupdate = 0;
    int updated_period = 0;
    while (common_pars->updated[sensor_port]) {
        dev_dbg(g_devfp_ptr,"port= %d,  thisFrameNumber[%d] = %d\n", sensor_port, sensor_port, (int) thisFrameNumber(sensor_port));
        common_pars->updated[sensor_port] = 0;
        if (pars_to_update[nupdate  ].num != P_TRIG_PERIOD){ // updated any other parameter - treat as period is updated (make sure it will not trigger)
            updated_period = FRAMEPAIR_FORCE_PROC;
        }
        pars_to_update[nupdate  ].num= P_TRIG_MASTER ;                  pars_to_update[nupdate++].val = common_pars->master_chn;
        pars_to_update[nupdate  ].num= P_TRIG_PERIOD | updated_period ; pars_to_update[nupdate++].val = common_pars->trig_period;
        pars_to_update[nupdate  ].num= P_TRIG_BITLENGTH ;               pars_to_update[nupdate++].val = common_pars->trig_bitlength;
        pars_to_update[nupdate  ].num= P_EXTERN_TIMESTAMP ;             pars_to_update[nupdate++].val = common_pars->extern_timestamp;
        pars_to_update[nupdate  ].num= P_XMIT_TIMESTAMP ;               pars_to_update[nupdate++].val = common_pars->xmit_timestamp;
        pars_to_update[nupdate  ].num= P_TRIG_CONDITION ;               pars_to_update[nupdate++].val = common_pars->trig_condition;
        pars_to_update[nupdate  ].num= P_TRIG_OUT ;                     pars_to_update[nupdate++].val = common_pars->trig_out;
        pars_to_update[nupdate  ].num= P_TRIG ;                         pars_to_update[nupdate++].val = common_pars->trig_mode;
//        if (nupdate)  setFramePars(sensor_port, &framepars[frame16], nupdate, pars_to_update);  // save changes, schedule functions
        if (nupdate)  setFramePars(sensor_port, &framepars[thisFrameNumber(sensor_port)], nupdate, pars_to_update);  // save changes, schedule functions

    }
}


/** Set multiple output (calculated) parameters for the frame referenced by this_framepars structure.
 * Schedules action only if the FRAMEPAIR_FORCE_PROC modifier bit is set in the particular parameter index
 * Called from tasklets (while executing (*_)pgm_* functions
 * Should only be called when IRQ (just bh preferrably) is off and lock on framepars_locks[sensor_port] (all pgm_* have that)  */

int setFramePars(int sensor_port,                     ///< sensor port number (0..3)
                 struct framepars_t * this_framepars, ///< this_framepars pointer to the current parameters structure
                 int numPars,                         ///< number of parameters to set
                 struct frameparspair_t * pars)       ///<array of parameters (number/value pairs). Parameter numbers accept modifiers
                                                      ///< @return  0 - OK, -ERR_FRAMEPARS_BADINDEX
{
	int frame16;
//	unsigned long flags; should only be called when interrupts disabled and lock obtained
	int npar, nframe;
	unsigned long val, bmask, bmask32;
	int index, bindex;
	int findex_this =  thisFrameNumber(sensor_port) & PARS_FRAMES_MASK;
	int findex_prev = (findex_this - 1)  & PARS_FRAMES_MASK;
	int findex_future = (findex_this - 2)  & PARS_FRAMES_MASK;
	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long      *funcs2call =afuncs2call[sensor_port];

	dev_dbg(g_devfp_ptr, "port= %d, this_framepars=0x%x numPars=%d\n", sensor_port, (int)this_framepars, numPars);
//	dev_info(g_devfp_ptr, "port= %d, this_framepars=0x%x numPars=%d\n", sensor_port, (int)this_framepars, numPars);

    MDP(DBGB_FSFP,sensor_port,"this_framepars=0x%x numPars=%d\n", (int)this_framepars, numPars)

	for (npar = 0; npar < numPars; npar++) {
		frame16 = (this_framepars->pars[P_FRAME]) & PARS_FRAMES_MASK;
		val = pars[npar].val;
		index = pars[npar].num & 0xffff;
		dev_dbg(g_devfp_ptr, ":    ---   frame16=%d index=%d (0x%x) val=0x%x, findex_future = 0x%x\n", frame16, index, (int)pars[npar].num, (int)val, findex_future);

//		dev_info(g_devfp_ptr, ":    ---   frame16=%d index=%d (0x%x) val=0x%x, findex_future = 0x%x\n", frame16, index, (int)pars[npar].num, (int)val, findex_future);

	    MDP(DBGB_FSFV,sensor_port,"  ---   frame16=%d index=%d (0x%x) val=0x%x\n", frame16, index, (int)pars[npar].num, (int)val)
		// remark: code below looks similar to setFramePar function, call it instead
		if (index > ((index >= FRAMEPAR_GLOBALS) ? (P_MAX_GPAR + FRAMEPAR_GLOBALS) : P_MAX_PAR)) {
			dev_err(g_devfp_ptr, " bad index=%d > %d\n", index, P_MAX_PAR);
			return -ERR_FRAMEPARS_BADINDEX;
		}
		if (index >= FRAMEPAR_GLOBALS) {                        // ignore frame logic, set "static" parameters to frame 0
			if (pars[npar].num & FRAMEPAIR_MASK_BYTES) {    // combine new value with the old one
				val = FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, GLOBALPARS(sensor_port,index), val);
			}
			GLOBALPARS(sensor_port,index) = val;
		} else if (pars[npar].num  & FRAMEPAIR_FRAME_FUNC) {
			funcs2call[index] = val;
		} else if ((frame16 != findex_future) || ((pars[npar].num & FRAMEPAIR_JUST_THIS) == 0)) {        // do not write "JUST_THIS" parameters in the future otherwise they'll stick
			if (pars[npar].num & FRAMEPAIR_MASK_BYTES) {                                            // combine new value with the old one
				val = FRAMEPAIR_FRAME_MASK_NEW(pars[npar].num, framepars[frame16].pars[index], val);
			}
//TODO: optimize to use mask several parameters together
	        dev_dbg(g_devfp_ptr, "{%d}:  framepars[%d].pars[0x%x] = 0x%x, val=0x%x\n", sensor_port, frame16, index, (int) framepars[frame16].pars[index], (int)val);
			if ((framepars[frame16].pars[index] != val) || (pars[npar].num & (FRAMEPAIR_FORCE_NEW | FRAMEPAIR_FORCE_PROC))) {
				bmask =   1 << (index & 31);
				bindex = index >> 5;
				bmask32 = 1 << bindex;
//   Set this parameter for specified frame, (for now - unconditionally mark as modified, even if the value is the same as it was - CHANGED!
				framepars[frame16].pars[index]       = val;
				framepars[frame16].mod[bindex]      |= bmask;
				framepars[frame16].mod32            |= bmask32;
				if (pars[npar].num & FRAMEPAIR_FORCE_PROC) {
					framepars[frame16].functions        |= funcs2call[index]; //Mark which functions will be needed to process the parameters
				}
	            dev_dbg(g_devfp_ptr, "{%d}:  framepars[%d].functions=0x%x\n", sensor_port, frame16, (int) framepars[frame16].functions);
// Write parameter to the next frames up to the one that have the same parameter already modified (only if not FRAMEPAIR_JUST_THIS)
				if ((pars[npar].num & FRAMEPAIR_JUST_THIS) == 0) {
//					MDF8(printk(":        ---   setting next frames"));
					for (nframe = (frame16 + 1) & PARS_FRAMES_MASK; (nframe != findex_prev) && (!(framepars[nframe].mod[bindex] & bmask)); nframe = (nframe + 1) & PARS_FRAMES_MASK) {
						framepars[nframe].pars[index] = val;
//                        D8(printk(" %d", nframe));
	                    MDP(DBGB_FSFV,sensor_port,"framepars[%d].pars[%d] <- 0x%08x  ", nframe, index, val)

					}
					frame16 = (frame16 - 1) & PARS_FRAMES_MASK; // for " regular parameters "modified since" do not include the target frame itself, for "JUST_THIS" - does
                    MDP(DBGB_FSFV,sensor_port,"%s\n", "")
//					D8(printk("\n"));
				}
// Mark this parameter in all previous frames as "modified since"
// TODO: consider alternative way - first iterate through all parameters, build masks, then apply them
//      for (nframe=(frame16-1) & PARS_FRAMES_MASK; nframe != findex_future; nframe=(nframe-1) & PARS_FRAMES_MASK) {
				for (nframe = frame16; nframe != findex_future; nframe = (nframe - 1) & PARS_FRAMES_MASK) { //NOTE: frame16 is modified here
					framepars[nframe].modsince[bindex] |= bmask;
					framepars[nframe].modsince32       |= bmask32;
	                MDP(DBGB_FSFV,sensor_port,"framepars[%d].modsince[%d] |= 0x%08x, framepars[%d].modsince32 |= 0x%08x  ",
	                        nframe, bindex, bmask, nframe, bmask32)
				}
                MDP(DBGB_FSFV,sensor_port,"%s\n", "")
			}
		} else { // error - trying to write "just this" to the "future" - that would stick if allowed
			dev_err(g_devfp_ptr, "Tried to write JUST_THIS parameter (0x%lx) too far in the future", pars[npar].num);
			return -ERR_FRAMEPARS_TOOEARLY;
		}
	}
	return 0;
}

/**
 * @brief wait for target frame number
 * @param sensor_port
 * @param frame_num target frame
 * @return 0
 */
int waitFrame(int sensor_port, unsigned long frame_num){
	wait_event_interruptible(aframepars_wait_queue[sensor_port], getThisFrameNumber(sensor_port) >= frame_num);
	return 0;
}

//TODO: make some parameters readonly (prohibited from modification from the userland)
// File operations:
// open, release - nop
// read - none
// write -> setFrameParsAtomic (first 4 bytes - absolute frame number, next 4 bytes - latency, then each 8 bytes - index/value)
// can use current file pointer or special indexes (0x****ff01 - set frame number, 0x****ff02 - set latency) that should come before actual parameters
// file pointer - absolute frame number
// lseek (SEEK_SET, value) - set absolute frame number
// lseek (SEEK_CUR, value) - set frame number relative to the current frame number (thisFrameNumber),
// lseek (SEEK_CUR, 0)     - (used by ftell()) also modifies file pointer - set it to thisFrameNumber,
// lseek (SEEK_END, value <= 0) - do nothing?, do not modify file pointer
// lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer (and actually use it - frame number the command applies to)
// mmap (should be used read only)

static struct file_operations framepars_fops = {
	owner:    THIS_MODULE,
	llseek:   framepars_lseek,
	write:    framepars_write,
	open:     framepars_open,
	mmap:     framepars_mmap,
	release:  framepars_release
};

/**
 * @brief Driver OPEN method
 * @param inode  inode
 * @param filp   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int framepars_open(struct inode *inode, struct file *filp)
{
	int res;
	struct framepars_pd * privData;
	privData = (struct framepars_pd*)kmalloc(sizeof(struct framepars_pd), GFP_KERNEL);
	if (!privData) return -ENOMEM;
	filp->private_data = privData;
	privData->minor = MINOR(inode->i_rdev);
	dev_dbg(g_devfp_ptr,"minor=0x%x\n", privData->minor);
	switch (privData->minor) {
	case  DEV393_MINOR(DEV393_FRAMEPARS0):
	case  DEV393_MINOR(DEV393_FRAMEPARS1):
	case  DEV393_MINOR(DEV393_FRAMEPARS2):
	case  DEV393_MINOR(DEV393_FRAMEPARS3):
	    MDP(DBGB_FFOP,privData->minor-DEV393_MINOR(DEV393_FRAMEPARS0),"minor=0x%x\n",privData->minor)
		inode->i_size = 0; //or return 8 - number of frame pages?
		return 0;
	default:
		kfree(filp->private_data); // already allocated
		return -EINVAL;
	}
	return res;
}

/**
 * @brief Driver RELEASE method
 * @param inode  inode
 * @param filp   file pointer
 * @return OK - 0, -EINVAL for wrong minor
 */
int framepars_release(struct inode *inode, struct file *filp)
{
	int res = 0;
	int p = MINOR(inode->i_rdev);

	dev_dbg(g_devfp_ptr,"%s : minor=0x%x\n",__func__, p);
	switch ( p ) {
	case  DEV393_MINOR(DEV393_FRAMEPARS0):
	case  DEV393_MINOR(DEV393_FRAMEPARS1):
	case  DEV393_MINOR(DEV393_FRAMEPARS2):
	case  DEV393_MINOR(DEV393_FRAMEPARS3):
	    MDP(DBGB_FFOP,p-DEV393_MINOR(DEV393_FRAMEPARS0),"minor=0x%x\n",p)
		break;
	default:
		return -EINVAL; //! do not need to free anything - "wrong number"
	}
	kfree(filp->private_data);
	return res;
}

/**
 * @brief Driver LSEEK  method (and execute commands)
 * - lseek (SEEK_SET, value) - set absolute frame number
 * - lseek (SEEK_CUR, value) - set frame number relative to the current frame number (thisFrameNumber),
 * - lseek (SEEK_CUR, 0)     - (used by ftell()) DOES NOT modify file pointer, returns thisFrameNumber,
 * - lseek (SEEK_END, value <= 0) - do nothing?, do not modify file pointer
 * - lseek (SEEK_END, value >  0) - execute commands, do not modify file pointer (and actually use it - frame number the command applies to)
 * - no commands yet
 * @param file
 * @param offset
 * @param orig SEEK_SET, SEEK_CUR or SEEK_SET END
 * @return file position (absolute frame number)
 */
loff_t framepars_lseek(struct file * file, loff_t offset, int orig)
{
	unsigned long target_frame;
	struct framepars_pd * privData = (struct framepars_pd*) file -> private_data;
	int sensor_port = privData -> minor - DEV393_MINOR(DEV393_FRAMEPARS0);
    sec_usec_t sec_usec;
    int res;
//	struct framepars_t *framepars = aframepars[sensor_port];
    dev_dbg(g_devfp_ptr, "(framepars_lseek)  offset=0x%x, orig=0x%x, sensor_port = %d\n", (int)offset, (int)orig, sensor_port);
    MDP(DBGB_FFOP, sensor_port, "(framepars_lseek)  offset=0x%x, orig=0x%x\n", (int)offset, (int)orig)

	switch (orig) {
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		if (offset == 0) return getThisFrameNumber(sensor_port);   // do not modify frame number
		file->f_pos = getThisFrameNumber(sensor_port) + offset;    // modifies frame number, but it is better to use absolute SEEK SET
		break;
	case SEEK_END:
		if (offset <= 0) {
			break;
		} else if (offset >= LSEEK_FRAME_WAIT_REL) {
			if (offset >= LSEEK_FRAME_WAIT_ABS) target_frame = offset - LSEEK_FRAME_WAIT_ABS;       // Wait for absolute frame number
			else target_frame = getThisFrameNumber(sensor_port) + offset - LSEEK_FRAME_WAIT_REL;
			// Skip 0..255 frames
//			wait_event_interruptible (framepars_wait_queue, getThisFrameNumber()>=target_frame);
		    dev_dbg(g_devfp_ptr, "getThisFrameNumber(%d) == 0x%x, target_frame = 0x%x\n", sensor_port, (int) getThisFrameNumber(sensor_port), (int)target_frame);
		    waitFrame(sensor_port,target_frame);
		    //wait_event_interruptible(aframepars_wait_queue[sensor_port], getThisFrameNumber(sensor_port) >= target_frame);
            dev_dbg(g_devfp_ptr, "getThisFrameNumber(%d) == 0x%x\n", sensor_port, (int) getThisFrameNumber(sensor_port));

//       if (getThisFrameNumber()<target_frame) wait_event_interruptible (framepars_wait_queue,getThisFrameNumber()>=target_frame);
			return getThisFrameNumber(sensor_port);    // Does not modify current frame pointer? lseek (,0,SEEK_CUR) anyway returns getThisFrameNumber()
		} else {                                //! Other lseek commands
			switch (offset & ~0x1f) {
			case  LSEEK_DAEMON_FRAME:       // wait the daemon enabled and a new frame interrupt (sensor frame sync)
				waitFrame(sensor_port,get_imageParamsThis(sensor_port, P_DAEMON_EN) & (1 << (offset & 0x1f)));
				//wait_event_interruptible(aframepars_wait_queue[sensor_port], get_imageParamsThis(sensor_port, P_DAEMON_EN) & (1 << (offset & 0x1f)));
				break;
			default:
				switch (offset) {
				case LSEEK_GET_FPGA_TIME:
					//X313_GET_FPGA_TIME( GLOBALPARS(G_SECONDS), GLOBALPARS(G_MICROSECONDS) );
                    if (!get_fpga_rtc(&sec_usec))
                        return - ENODEV;
                    GLOBALPARS(sensor_port,G_SECONDS) =      sec_usec.sec;
                    GLOBALPARS(sensor_port,G_MICROSECONDS) = sec_usec.usec;
                    dev_dbg(g_devfp_ptr, "LSEEK_GET_FPGA_TIME: sec=%ld, usec=%ld\n", sec_usec.sec, sec_usec.usec);
					break;
				case LSEEK_SET_FPGA_TIME: // better to use write, not lseek to set FPGA time
					//X313_SET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
                    sec_usec.sec =  GLOBALPARS(sensor_port,G_SECONDS);
                    sec_usec.usec = GLOBALPARS(sensor_port,G_MICROSECONDS);
                    res = set_fpga_rtc(sec_usec);
                    if (res <0){
                        return - ENODEV;
                    }
                    dev_dbg(g_devfp_ptr, "LSEEK_SET_FPGA_TIME: sec=%ld, usec=%ld\n", sec_usec.sec, sec_usec.usec);
					break;
				case LSEEK_FRAMEPARS_INIT:      // reset hardware sequencers, init framepars structure
				    dev_dbg(g_devfp_ptr, "LSEEK_FRAMEPARS_INIT\n");
					initGlobalPars(sensor_port);       // reset all global parameters but the first 32
					res = initSequencers(sensor_port);
					if (res <0) return res;
					break;
				case LSEEK_FRAME_RESET: // reset absolute frame number to avoid integer frame number overflow
                    dev_dbg(g_devfp_ptr, "LSEEK_FRAME_RESET\n");
					resetFrameNumber(sensor_port, 0, 0 ); // just high bits, do not reset hardware
					break;
				case LSEEK_SENSORPROC:                  // process modified parameters in frame 0 (to start sensor detection)
				    dev_dbg(g_devfp_ptr, "LSEEK_SENSORPROC: aframepars[%d][0].functions=0x%08lx\n",
				            sensor_port, aframepars[sensor_port][0].functions);
					res = processPars(sensor_port, &asensorproc[sensor_port], 0, PARS_FRAMES);  //frame0, all 8 frames (maxAhead==8)
					if (res <0) return res;
					break;
				case LSEEK_DMA_INIT:                    // initialize ETRAX DMA (normally done in sensor_common.c at driver init
				    dev_dbg(g_devfp_ptr, "LSEEK_DMA_INIT\n");
				    trigger_restart  (); // LSEEK_DMA_INIT is unused
					//x313_dma_init();
					break;
				case LSEEK_DMA_STOP: // stop DMA
				    dev_dbg(g_devfp_ptr, "LSEEK_DMA_STOP\n");
					//x313_dma_stop();           //
					break;
				case LSEEK_DMA_START: // start DMA
				    dev_dbg(g_devfp_ptr, "LSEEK_DMA_START\n");
					//x313_dma_start();          //
					break;
				case LSEEK_COMPRESSOR_RESET: // reset compressor and buffer pointers
				    dev_dbg(g_devfp_ptr, "LSEEK_COMPRESSOR_RESET\n");
					//reset_compressor();
					break;
				case LSEEK_INTERRUPT_OFF: // disable camera interrupts
				    dev_dbg(g_devfp_ptr, "LSEEK_INTERRUPT_OFF\n");
//					compressor_interrupts(0,sensor_port);
                    sensor_interrupts(0,sensor_port);
					break;
				case LSEEK_INTERRUPT_ON: // enable camera interrupts
				    dev_dbg(g_devfp_ptr, "LSEEK_INTERRUPT_ON\n");
					sensor_interrupts(1,sensor_port);
					break;
				}
			}
			break;
		}
		break;
	default:
		return -EINVAL;
	}
	return file->f_pos;
}

/**
 * @brief Driver WRITE method
 * writes all at once, no provisions to continue in the next call
 * @param file
 * @param buf
 * @param count
 * @param off
 * @return OK - number of bytes written, negative - errors
 */
ssize_t framepars_write(struct file * file, const char * buf, size_t count, loff_t *off)
{
	struct frameparspair_t pars_static[256]; // will be sufficient for most calls
	struct frameparspair_t * pars = pars_static;
	struct framepars_pd * privData = (struct framepars_pd*)file->private_data;
	int sensor_port = privData -> minor - DEV393_MINOR(DEV393_FRAMEPARS0);
	//	struct framepars_t *framepars = aframepars[sensor_port];
	unsigned long frame = *off; // ************* NOTE: Never use file->f_pos in write() and read() !!!
	int latency = -1;
	int first = 0;
	int last;
	int result = 0;
	int port_mask=0; // to apply to several channels simultaneously
	unsigned long frames[SENSOR_PORTS];
	sec_usec_t sec_usec;
//	dev_dbg(g_devfp_ptr,"%s : file->f_pos=0x%x, *off=0x%x, count=0x%x\n",__func__, (int)file->f_pos, (int)*off, (int)count);
    dev_dbg(g_devfp_ptr, "file->f_pos=0x%x, *off=0x%x, count=0x%x, minor=0x%x\n",
            (int)file->f_pos, (int)*off, (int)count, (int) privData->minor);

    MDP(DBGB_FFOP, sensor_port, "file->f_pos=0x%x, *off=0x%x, count=0x%x, minor=0x%x\n",
            (int)file->f_pos, (int)*off, (int)count, (int) privData->minor)

	count &= ~7; // sizeof (struct frameparspair_t)==8
	switch (privData->minor) {
	case DEV393_MINOR(DEV393_FRAMEPARS0):
    case DEV393_MINOR(DEV393_FRAMEPARS1):
    case DEV393_MINOR(DEV393_FRAMEPARS2):
    case DEV393_MINOR(DEV393_FRAMEPARS3):
		if (count > sizeof(pars_static)) // only allocate if static is not enough
			pars =     (struct frameparspair_t*)kmalloc(count, GFP_KERNEL);
		if (!pars) return -ENOMEM;
		count >>= 3; // divide by sizeof(struct frameparspair_t); // 8
		if (count) {
			if (copy_from_user((char*)pars, buf, count << 3)) {
				if (count > sizeof(pars_static)) kfree(pars);
				return -EFAULT;
			}
			while (first < count) {
				while ((first < count) && ((pars[first].num & 0xff00) == 0xff00)) { // process special instructions
				    dev_dbg(g_devfp_ptr, "pars[%d].num = 0x%lx pars[%d].val = 0x%lx\n",first,pars[first].num, first,pars[first].val);
					switch (pars[first].num & 0xff0f) {
#if 0
					case FRAMEPARS_SETFRAME:
						frame = pars[first].val;
						port_mask = (pars[first].num >> 4) & ((1 << SENSOR_PORTS) - 1);
						if (port_mask){
						    // Retry if got different results - mostly for in-sync running sensors and just swicthed
						    // TODO: What to do: triggered sensors have frame sync delayed by exposure time from the common trigger
						    int frames_diff=1;
                            int ii;
						    for (ii =0; ii < SENSOR_PORTS; ii++) frames[ii] = 0xffffffff;
						    while (frames_diff) {
						        frames_diff = 0;
						        frame = pars[first].val;
						        //TODO: Disable interrupts here to freeze frame difference
						        for (ii =0; ii < SENSOR_PORTS; ii++){
                                    frame = pars[first].val + getThisFrameNumber(ii) - getThisFrameNumber(sensor_port);
                                    if (frame != frames[ii]) frames_diff = 1;
						            frames[ii] = frame;
						        }
						    }
						}
						break;
#endif
                    case FRAMEPARS_SETFRAME:
                        frame = pars[first].val;
                        port_mask = (pars[first].num >> 4) & ((1 << SENSOR_PORTS) - 1);
                        // No correction - frames should be exctly synchronized to work this way, otherwise use relative
                        if (port_mask){
                            int ii;
                            for (ii =0; ii < SENSOR_PORTS; ii++) if (port_mask & (1 << ii)){
                                frames[ii] = frame;
                            }
                        }
                        dev_dbg(g_devfp_ptr, "port_mask=0x%x frames[0]=0x%lx frames[1]=0x%lx frames[2]=0x%lx frames[3]=0x%lx\n",
                                port_mask, frames[0], frames[1], frames[2], frames[3]);
                        break;
					case FRAMEPARS_SETFRAMEREL:
						frame = pars[first].val + getThisFrameNumber(sensor_port);
                        port_mask = (pars[first].num >> 4) & ((1 << SENSOR_PORTS) - 1);
                        if (port_mask){
                            int ii;
                            for (ii =0; ii < SENSOR_PORTS; ii++) if (port_mask & (1 << ii)){
                                frames[ii] = pars[first].val + getThisFrameNumber(ii);
                            }
                        }
						break;
					case FRAMEPARS_SETLATENCY:
						latency = (pars[first].val & 0x80000000) ? -1 : pars[first].val;
						break;
					case FRAMEPARS_SETFPGATIME: //ignore value (should be already set in G_SECONDS, G_MICROSECONDS frame0)
                        //X313_SET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
					    sec_usec.sec =  GLOBALPARS(sensor_port,G_SECONDS);
                        sec_usec.usec = GLOBALPARS(sensor_port,G_MICROSECONDS);
                        result = set_fpga_rtc(sec_usec);
                        if (result <0)
                            return result;
						break;
					case FRAMEPARS_GETFPGATIME: //ignore value, set frame 0 G_SECONDS, G_MICROSECONDS to FPGA registers
						//X313_GET_FPGA_TIME( GLOBALPARS(G_SECONDS) , GLOBALPARS(G_MICROSECONDS) );
					    if (!get_fpga_rtc(&sec_usec))
					        return - ENODEV;
					    GLOBALPARS(sensor_port,G_SECONDS) =      sec_usec.sec;
                        GLOBALPARS(sensor_port,G_MICROSECONDS) = sec_usec.usec;
						break;
					default:
					    dev_err(g_devfp_ptr, "%s 0x%x: invalid special instruction: 0x%x\n",
					            __func__, (int) privData->minor,  (int)pars[first].num);
					}
					first++;
				}
				last = first + 1;
				while ((last < count) && ((pars[last].num & 0xff00) != 0xff00)) last++;  // skip to the end or next special instructions
                if (port_mask) {
                    int ii;
                    dev_dbg(g_devfp_ptr, "0x%x: port_mask=0x%x\n", (int) privData->minor, port_mask);
                    for (ii =0; ii < SENSOR_PORTS; ii++) if (port_mask & (1 << ii)){
                        dev_dbg(g_devfp_ptr, "0x%x: setFrameParsAtomic(%d, %ld, %d, %d)\n",
                                (int) privData->minor, ii, frame, latency, last - first);
                        MDP(DBGB_FFOP, sensor_port, "0x%x: setFrameParsAtomic(%d, %ld, %d, %d)\n",
                                (int) privData->minor, ii, frame, latency, last - first)
                        result |= setFrameParsAtomic(ii, frames[ii], latency, last - first, &pars[first]);
                    }
                } else {
                    dev_dbg(g_devfp_ptr, "0x%x: setFrameParsAtomic(%ld, %d, %d)\n",
                            (int) privData->minor, frame, latency, last - first);
                    MDP(DBGB_FFOP, sensor_port, "0x%x: setFrameParsAtomic(%ld, %d, %d)\n",
                            (int) privData->minor, frame, latency, last - first)
                    result = setFrameParsAtomic(sensor_port,frame, latency, last - first, &pars[first]);
                }
				if (result < 0) {
					if (count > sizeof(pars_static)) kfree(pars);
					return -EFAULT;
				}
				first = last;
			}
		}
		if (count > sizeof(pars_static)) kfree(pars);
		return count << 3; // *sizeof(struct frameparspair_t);
	default: return -EINVAL;
	}
} // the frame size of 2080 bytes is larger than 1024 bytes [-Wframe-larger-than=]

/**
 * @brief Driver MMAP method (should be used read only)
 * provides access to both 8-frame parameters (including future ones), frame - 0 - some static ones too and
 * much longer retained past parameters - subset of all parameters
 * @param file
 * @param vma
 * @return OK - 0, negative - errors
 */
int framepars_mmap(struct file *file, struct vm_area_struct *vma)
{
	int result;
	struct framepars_pd * privData = (struct framepars_pd*)file->private_data;
	int sensor_port = privData -> minor - DEV393_MINOR(DEV393_FRAMEPARS0);

	dev_dbg(g_devfp_ptr,"%s : minor=0x%x\n",__func__, privData->minor);
	switch (privData->minor) {
	case DEV393_MINOR(DEV393_FRAMEPARS0):
	case DEV393_MINOR(DEV393_FRAMEPARS1):
	case DEV393_MINOR(DEV393_FRAMEPARS2):
	case DEV393_MINOR(DEV393_FRAMEPARS3):
	result = remap_pfn_range(vma,
	        vma->vm_start,
	        ((unsigned long)virt_to_phys(&aframeparsall[sensor_port])) >> PAGE_SHIFT, // Should be page-aligned
	        vma->vm_end - vma->vm_start,
	        vma->vm_page_prot);
	dev_dbg(g_devfp_ptr, "remap_pfn_range returned=%x\n", result);
	MDP(DBGB_FFOP, sensor_port, "remap_pfn_range returned=%x\n", result)

	if (result) return -EAGAIN;
	return 0;
	default: return -EINVAL;
	}
}
// SysFS interface to framepars
#define SYSFS_PERMISSIONS           0644 /* default permissions for sysfs files */
#define SYSFS_READONLY              0444
#define SYSFS_WRITEONLY             0222
/** Sysfs helper function - get channel number from the last character of the attribute name*/
static int get_channel_from_name(struct device_attribute *attr) ///< Linux kernel interface for exporting device attributes
                                                                ///< @return channel number
{
    int reg = 0;
    sscanf(attr->attr.name + (strlen(attr->attr.name)-1), "%du", &reg);
    return reg;
}

static ssize_t show_this_frame(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", (int) GLOBALPARS(get_channel_from_name(attr), G_THIS_FRAME));
}

static ssize_t store_this_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u32 aframe;
    if (sscanf(buf, "%u", &aframe)>0) {
        resetFrameNumber(get_channel_from_name(attr),
                         aframe,
                         (aframe < PARS_FRAMES)?1:0); // reset hardware if aframe is small
    } else {
        stopFrameSequencer(get_channel_from_name(attr), 1); // 0); // do not disable interrupts here?
//        return - EINVAL;
    }
    return count;
}
static ssize_t store_endis_chn(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u32 en;
    if (sscanf(buf, "%u", &en)>0) {
        enDisSensorChn(get_channel_from_name(attr),en);
    } else {
        return - EINVAL;
    }
    return count;
}



//    sscanf(buf, "%i", &buffer_settings.frame_start[get_channel_from_name(attr)], &len);

static ssize_t show_fpga_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (!hardware_initialized)
        return -EBUSY;
    return sprintf(buf,"0x%08x\n", x393_fpga_version());
}

static ssize_t show_fpga_sensor_interface(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (!hardware_initialized)
        return -EBUSY;
    switch (x393_sensor_interface()){
    case FPGA_PAR12:
        return sprintf(buf,"PAR12\n");
    case FPGA_HISPI:
        return sprintf(buf,"HISPI\n");
    case FPGA_VOSPI:
        return sprintf(buf,"VOSPI\n");
    default:
        return sprintf(buf,"0x%08x\n", (int) x393_sensor_interface());
    }
}



static ssize_t show_fpga_time(struct device *dev, struct device_attribute *attr, char *buf)
{
    sec_usec_t sec_usec;
    if (!get_fpga_rtc(&sec_usec))
        return - ENODEV;
    return sprintf(buf,"%lu.%lu\n", sec_usec.sec, sec_usec.usec);
}

static ssize_t store_fpga_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sec_usec_t sec_usec={.sec=0, .usec=0};
    // avoiding floating point calcualtions in the kernel
    char *cp = (char *) buf;
    int i;
    if (sscanf(buf, "%lu.%s", &sec_usec.sec, cp)>0){
        sscanf(cp,"%lu",&sec_usec.usec);
        for (i=strlen(cp); i<6;i++)
            sec_usec.usec *=10;
        i = set_fpga_rtc(sec_usec);
        if (i<0) return i;
    } else {
        return - EINVAL;
    }
    return count;
}

static ssize_t show_all_frames(struct device *dev, struct device_attribute *attr, char *buf)
{
    int chn;
    int sensor_frame16[SENSOR_PORTS];
    int compressor_frame16[SENSOR_PORTS];
    int i2c_frame16[SENSOR_PORTS];
    int sensor_aframe[SENSOR_PORTS];
    int compressor_aframe[SENSOR_PORTS];
    int compressor_gframe[SENSOR_PORTS];
    char * buf0=buf;
    for (chn = 0; chn < SENSOR_PORTS; chn++){
        sensor_frame16[chn] =     getHardFrameNumber(chn,0);
        compressor_frame16[chn] = getHardFrameNumber(chn,1);
        i2c_frame16[chn] =    read_xi2c_frame(chn);
        sensor_aframe[chn] =     (int) GLOBALPARS(chn, G_THIS_FRAME);
        compressor_aframe[chn] = (int) get_compressor_frame(chn);
        compressor_gframe[chn] = (int) GLOBALPARS(chn, G_COMPRESSOR_FRAME);
    }
    buf += sprintf(buf,"    sensor_fram16 ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"       0x%x ",sensor_frame16[chn]);
    buf += sprintf(buf,"\n");
    buf += sprintf(buf,"compressor_fram16 ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"       0x%x ",compressor_frame16[chn]);
    buf += sprintf(buf,"\n");
    buf += sprintf(buf,"      i2c_frame16 ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"       0x%x ",i2c_frame16[chn]);
    buf += sprintf(buf,"\n");
    buf += sprintf(buf,"    sensor_aframe ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"0x%08x ",sensor_aframe[chn]);
    buf += sprintf(buf,"\n");
    buf += sprintf(buf,"compressor_aframe ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"0x%08x ",compressor_aframe[chn]);
    buf += sprintf(buf,"\n");
    buf += sprintf(buf,"compressor_gframe ");
    for (chn = 0; chn < SENSOR_PORTS; chn++) buf += sprintf(buf,"0x%08x ",compressor_gframe[chn]);
    buf += sprintf(buf,"\n");
    return buf-buf0;
}

static DEVICE_ATTR(this_frame0,               SYSFS_PERMISSIONS,     show_this_frame,                store_this_frame);
static DEVICE_ATTR(this_frame1,               SYSFS_PERMISSIONS,     show_this_frame,                store_this_frame);
static DEVICE_ATTR(this_frame2,               SYSFS_PERMISSIONS,     show_this_frame,                store_this_frame);
static DEVICE_ATTR(this_frame3,               SYSFS_PERMISSIONS,     show_this_frame,                store_this_frame);
static DEVICE_ATTR(chn_en0,                   SYSFS_PERMISSIONS,     NULL,                           store_endis_chn);
static DEVICE_ATTR(chn_en1,                   SYSFS_PERMISSIONS,     NULL,                           store_endis_chn);
static DEVICE_ATTR(chn_en2,                   SYSFS_PERMISSIONS,     NULL,                           store_endis_chn);
static DEVICE_ATTR(chn_en3,                   SYSFS_PERMISSIONS,     NULL,                           store_endis_chn);
//static DEVICE_ATTR(chn_en0,                 SYSFS_WRITEONLY,       NULL,                           store_endis_chn);
//static DEVICE_ATTR(chn_en1,                 SYSFS_WRITEONLY,       NULL,                           store_endis_chn);
//static DEVICE_ATTR(chn_en2,                 SYSFS_WRITEONLY,       NULL,                           store_endis_chn);
//static DEVICE_ATTR(chn_en3,                 SYSFS_WRITEONLY,       NULL,                           store_endis_chn);

static DEVICE_ATTR(all_frames,                SYSFS_READONLY,        show_all_frames,                NULL);
static DEVICE_ATTR(fpga_time,                 SYSFS_PERMISSIONS,     show_fpga_time,                 store_fpga_time);
static DEVICE_ATTR(fpga_version,              SYSFS_READONLY,        show_fpga_version,              NULL);
static DEVICE_ATTR(fpga_sensor_interface,     SYSFS_READONLY,        show_fpga_sensor_interface,     NULL);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_this_frame0.attr,
        &dev_attr_this_frame1.attr,
        &dev_attr_this_frame2.attr,
        &dev_attr_this_frame3.attr,
        &dev_attr_all_frames.attr,
        &dev_attr_fpga_time.attr,
        &dev_attr_chn_en0.attr,
        &dev_attr_chn_en1.attr,
        &dev_attr_chn_en2.attr,
        &dev_attr_chn_en3.attr,
        &dev_attr_fpga_version.attr,
        &dev_attr_fpga_sensor_interface.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_framepars_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

/**
 * @brief framepars driver probing function
 * @param[in]   pdev   pointer to \b platform_device structure
 * @return      0 on success or negative error code otherwise
 */
int framepars_init(struct platform_device *pdev)
{
	//struct elphel_drvdata drvdata;
	int res;
	struct device *dev = &pdev->dev;
//	const struct of_device_id *match;
	int i;

	// char device for sensor port
	struct device *chrdev;

	for (i = 0; i < SENSOR_PORTS; i++) {
		init_framepars_ptr(i);
		initGlobalPars(i);       // sets default debug if enabled - not anymore. Add here?
		initMultiPars(i);        // just clear - needs to be called again when sensor is recognized
	    frameParsInitialized[i] = 0;
	}

	// register character device
	res = register_chrdev(DEV393_MAJOR(DEV393_FRAMEPARS0), DEV393_NAME(DEV393_FRAMEPARS0), &framepars_fops);
	if (res < 0) {
	    dev_err(dev, "framepars_init: couldn't get a major number %d (DEV393_MAJOR(DEV393_FRAMEPARS0)).\n",
		        DEV393_MAJOR(DEV393_FRAMEPARS0));
		return res;
	}
	dev_info(dev, DEV393_NAME(DEV393_FRAMEPARS0)": registered MAJOR: %d\n", DEV393_MAJOR(DEV393_FRAMEPARS0));

	// register to sysfs
	elphel393_framepars_sysfs_register(pdev);
	/* Create device class needed by udev */
	framepars_dev_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(framepars_dev_class)) {
		pr_err("Cannot create \"%s\" class", DRIVER_NAME);
		return PTR_ERR(framepars_dev_class);
	}

	for (i = 0; i < (sizeof(framepars_minor)/sizeof(int)); i++) {
		pr_debug("Creating device with major-minor: %d-%d, name: %s\n",framepars_major,framepars_minor[i],framepars_name[i]);
		chrdev = device_create(
				  framepars_dev_class,
				  &pdev->dev,
			      MKDEV(framepars_major, framepars_minor[i]),
			      NULL,
			      "%s", framepars_name[i]);
		if(IS_ERR(chrdev)){
			pr_err("Failed to create a device. Error code: %ld\n",PTR_ERR(chrdev));
		}

	}

	for (i = 0; i < SENSOR_PORTS; i++) {
		init_waitqueue_head(&aframepars_wait_queue[i]);
	}

    dev_info(dev, DEV393_NAME(DEV393_FRAMEPARS0)": registered sysfs\n");
    g_devfp_ptr = dev;
    hardware_initialized = 0;

	return 0;
}

int framepars_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < (sizeof(framepars_minor)/sizeof(int)); i++) {
		device_destroy(
			framepars_dev_class,
			MKDEV(framepars_major,framepars_minor[i]));
	}

	unregister_chrdev(DEV393_MAJOR(DEV393_FRAMEPARS0), DEV393_NAME(DEV393_FRAMEPARS0));
	return 0;
}

static const struct of_device_id elphel393_framepars_of_match[] = {
	{ .compatible = "elphel,elphel393-framepars-1.00" },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, elphel393_framepars_of_match);

static struct platform_driver elphel393_framepars = {
	.probe = framepars_init,
	.remove	= framepars_remove,
	.driver	= {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = elphel393_framepars_of_match,
	},
};

module_platform_driver(elphel393_framepars);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);

