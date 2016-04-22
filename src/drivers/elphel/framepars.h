#ifndef _FRAMEPARS_H
#define _FRAMEPARS_H
#ifndef SENSOR_PORTS
	#include <elphel/c313a.h> // to get SENSOR_PORTS
#endif
//extern struct framepars_t (*framepars)[PARS_FRAMES];
extern struct framepars_t      *aframepars[SENSOR_PORTS];
extern struct framepars_past_t *apastpars[SENSOR_PORTS];
extern unsigned long           *aglobalPars[SENSOR_PORTS];
extern unsigned long           *amultiSensIndex[SENSOR_PORTS];
extern unsigned long           *amultiSensRvrsIndex[SENSOR_PORTS];
extern wait_queue_head_t        framepars_wait_queue[SENSOR_PORTS];

///TODO: init framepars (zero parameters) before initscripts (not when detecting the sensors) - then initscript will be able to overwrite some
void init_framepars_ptr(int sensor_port);
void initSequencers    (int sensor_port); ///Move to sensorcommon? currently it is used through frameparsall file (lseek)
void initGlobalPars    (int sensor_port); /// resets all global parameters but debug mask (if ELPHEL_DEBUG)
int  initMultiPars     (int sensor_port); /// initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called aftre/during sensor detection
void initFramePars     (int sensor_port); ///initialize all parameters, set thisFrameNumber to frame8 (read from hardware, usually 0 after resetting i2c and cmd_seq)
void resetFrameNumber  (int sensor_port); /// reset this frame number (called from initFramePars(), also can be used to avoid frame number integer overflow)

unsigned long get_imageParamsThis (int sensor_port, int n);
unsigned long get_imageParamsPrev (int sensor_port, int n);

void          set_imageParamsThis (int sensor_port, int n, unsigned long d);
unsigned long get_globalParam     (int sensor_port, int n);

void          set_globalParam     (int sensor_port, int n, unsigned long d);
void          set_imageParamsR_all(int sensor_port, int n, unsigned long d);
void          updateFramePars     (int sensor_port, int frame8, struct interframe_params_t * frame_pars); /// called from ISR - advance thisFrameNumber to match hardware frame8, copy parameters as needed.
                                   /// frame8 usually is just next after thisFrameNumber
                                   /// frame_pars - pointer to structure (between frames in the frame buffer) to save a pointer to past parameters
int           setFrameParsStatic  (int sensor_port, int numPars, struct frameparspair_t * pars);

unsigned long getThisFrameNumber  (int sensor_port); /// just return current thisFrameNumber
/// set parameters for the frame number frameno, knowing that they should be set not less than maxLatency ahead (may be sensor - dependent)
/// Parameters (numPars of them) will be updated all at once, with interrupts disabled
/// Return - 0 if OK, -ERR_FRAMEPARS_TOOEARLY or -ERR_FRAMEPARS_TOOLATE if it is too early or too late to set parameters (numPars may be 0 to just test)
///
/// NOTE: When writing parameter to P_SENSOR_RUN or P_COMPRESSOR_RUN "*_RUN_SINGLE", first write "*SENSOR_RUN_STOP" (it will propagate to all next frames) and then
/// write "*_RUN_SINGLE", to (P_*_RUN | FRAMEPAIR_JUST_THIS) - then this *_RUN_SINGLE will not propagate to the next frames (they will stay stopped)


/// TODO: Make (an "unlimited") future commands que based on lists and a tree frame index

int setFrameParsAtomic            (int sensor_port, unsigned long frameno, int maxLatency, int numPars, struct frameparspair_t * pars);

/// set output/calculated parameter and propagate changes - will not trigger any actions
int setFramePar                   (int sensor_port, struct framepars_t * this_framepars, unsigned long mindex, unsigned long val);
///same for several pars at once
int setFramePars                  (int sensor_port, struct framepars_t * this_framepars, int numPars, struct frameparspair_t * pars);
/// schedule pgm_func to be executed for selected frame
void schedule_pgm_func            (int sensor_port, int frame8, int func_num);
/// schedule pgm_func to be executed for the current frame
void schedule_this_pgm_func       (int sensor_port, struct framepars_t * this_framepars, int func_num);
  
/// program acquisition, according to the parameters changed.
/// maxahead - how many frames ahead of time (start with most urgent, then 1 ahead, ...)
/// make maxahead - P_* parameter?
/* 393: See if sesnor port is needed here */
inline void processParsASAP      (int sensor_port, struct sensorproc_t * sensorproc, int frame8);
inline void processParsSeq       (int sensor_port, struct sensorproc_t * sensorproc, int frame8, int maxahead);

void        processPars          (int sensor_port, struct sensorproc_t * sensorproc, int frame8, int maxahead);
///*** TODO: Add option (flag?) to write "single" (like single compress, single sensor) so it will not make all the next frames "single"

int         framepars_init       (struct platform_device *pdev);
int         framepars_remove     (struct platform_device *pdev);

#endif
