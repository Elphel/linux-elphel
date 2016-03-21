#ifndef _FRAMEPARS_H
#define _FRAMEPARS_H

//extern struct framepars_t (*framepars)[PARS_FRAMES];
extern struct framepars_t      *framepars;
extern struct framepars_past_t *pastpars;
extern unsigned long           *globalPars;
extern unsigned long           *multiSensIndex;
extern unsigned long           *multiSensRvrsIndex;

extern wait_queue_head_t framepars_wait_queue;
///TODO: init framepars (zero parameters) before initscripts (not when detecting the sensors) - then initscript will be able to overwrite some
void init_framepars_ptr(void);
void initSequencers(void);         ///Move to sensorcommon? currently it is used through frameparsall file (lseek)
void initGlobalPars(void);   /// resets all global parameters but debug mask (if ELPHEL_DEBUG)
int  initMultiPars(void);  /// initialize structures for individual per-sensor parameters. Now only works for sensor registers using G_MULTI_REGSM. Should be called aftre/during sensor detection
void initFramePars(void);    ///initialize all parameters, set thisFrameNumber to frame8 (read from hardware, usually 0 after resetting i2c and cmd_seq)
void resetFrameNumber(void); /// reset this frame number (called from initFramePars(), also can be used to avoid frame number integer overflow)

unsigned long get_imageParamsThis (int n);
unsigned long get_imageParamsPrev (int n);

void          set_imageParamsThis (int n,unsigned long d);
unsigned long get_globalParam     (int n);

void          set_globalParam   (int n, unsigned long d);
void          set_imageParamsR_all(int n, unsigned long d);
void updateFramePars(int frame8, struct interframe_params_t * frame_pars); /// called from ISR - advance thisFrameNumber to match hardware frame8, copy parameters as needed.
                                   /// frame8 usually is just next after thisFrameNumber
                                   /// frame_pars - pointer to structure (between frames in the frame buffer) to save a pointer to past parameters
int setFrameParsStatic(int numPars, struct frameparspair_t * pars);

unsigned long getThisFrameNumber(void); /// just return current thisFrameNumber
/// set parameters for the frame number frameno, knowing that they should be set not less than maxLatency ahead (may be sensor - dependent)
/// Parameters (numPars of them) will be updated all at once, with interrupts disabled
/// Return - 0 if OK, -ERR_FRAMEPARS_TOOEARLY or -ERR_FRAMEPARS_TOOLATE if it is too early or too late to set parameters (numPars may be 0 to just test)
///
/// NOTE: When writing parameter to P_SENSOR_RUN or P_COMPRESSOR_RUN "*_RUN_SINGLE", first write "*SENSOR_RUN_STOP" (it will propagate to all next frames) and then
/// write "*_RUN_SINGLE", to (P_*_RUN | FRAMEPAIR_JUST_THIS) - then this *_RUN_SINGLE will not propagate to the next frames (they will stay stopped)


/// TODO: Make (an "unlimited") future commands que based on lists and a tree frame index

int setFrameParsAtomic(unsigned long frameno, int maxLatency, int numPars, struct frameparspair_t * pars);

/// set output/calculated parameter and propogate changes - will not trigger any actions
int setFramePar(struct framepars_t * this_framepars, unsigned long mindex, unsigned long val);
///same for several pars at once
int setFramePars(struct framepars_t * this_framepars, int numPars, struct frameparspair_t * pars);
/// schedule pgm_func to be executed for selected frame
void schedule_pgm_func(int frame8, int func_num);
/// schedule pgm_func to be executed for the current frame
void schedule_this_pgm_func(struct framepars_t * this_framepars, int func_num);
  
/// program acquisition, according to the parameters changed.
/// maxahead - how many frames ahead of time (start with most urgent, then 1 ahead, ...)
/// make maxahead - P_* parameter?
inline void processParsASAP (struct sensorproc_t * sensorproc, int frame8);
inline void processParsSeq (struct sensorproc_t * sensorproc, int frame8, int maxahead);

void processPars (struct sensorproc_t * sensorproc, int frame8, int maxahead);
///*** TODO: Add option (flag?) to write "single" (like single compress, single sensor) so it will not make all the next frames "single"
#endif
