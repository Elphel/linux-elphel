/***************************************************************************//**
* @file      x393_videomem.c
* @brief     Driver for the external DDR3 memory of x393 (currently 0.5GB)
* @copyright Copyright 2016 (C) Elphel, Inc.
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
//#define DEBUG
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <asm/outercache.h>     // TODO: Implement cache operations for the membridge !!!!
#include <asm/cacheflush.h>

#include <linux/errno.h>
#include <linux/fs.h>
//#include <linux/spinlock.h>

#include <asm/uaccess.h>

#include "x393.h"
#include "x393_macro.h"
#include "pgm_functions.h"
#include "x393_videomem.h"
#include "x393_fpga_functions.h"
#include "framepars.h"

#include <uapi/elphel/c313a.h>
#include <uapi/elphel/x393_devices.h>

#include <elphel/elphel393-mem.h>

#define VIDEOMEM_MODULE_DESCRIPTION "Video buffer driver"
//#define VIDEOMEM_DRIVER_NAME        "video_mem"
// NC393 debug macros
#include "debug393.h"

#define MEMBRIDGE_CMD_IRQ_EN 0x3

static const struct of_device_id elphel393_videomem_of_match[];
static struct device *g_dev_ptr; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions

wait_queue_head_t videomem_wait_queue;

static DEFINE_SPINLOCK(lock);             ///< for read-modify-write channel enable
static DEFINE_SPINLOCK(membridge_lock);   ///< to lock membridge, unlock on membridge_done interrupt

static int membridge_locked = 0;

/**
 * VIDEOMEM_RAW & IMAGE_RAW are for debug memory access.
 * Indexed devices are for per channel access
 */
static const char * const videomem_devs[]={
	DEV393_DEVNAME(DEV393_VIDEOMEM_RAW),
	DEV393_DEVNAME(DEV393_IMAGE_RAW),
	DEV393_DEVNAME(DEV393_IMAGE_RAW0),
	DEV393_DEVNAME(DEV393_IMAGE_RAW1),
	DEV393_DEVNAME(DEV393_IMAGE_RAW2),
	DEV393_DEVNAME(DEV393_IMAGE_RAW3)
};

static const int videomem_major = DEV393_MAJOR(DEV393_VIDEOMEM_RAW);

static const int videomem_minor[]={
	DEV393_MINOR(DEV393_VIDEOMEM_RAW),
	DEV393_MINOR(DEV393_IMAGE_RAW),
	DEV393_MINOR(DEV393_IMAGE_RAW0),
	DEV393_MINOR(DEV393_IMAGE_RAW1),
	DEV393_MINOR(DEV393_IMAGE_RAW2),
	DEV393_MINOR(DEV393_IMAGE_RAW3)
};

/** @brief Global device class for sysfs */
static struct class *videomem_dev_class;

// About frame full width:
// https://blog.elphel.com/2015/05/nc393-development-progress-multichannel-memory-controller-for-the-multi-sensor-camera/#Memory_mapping_and_access_types
static struct elphel_video_buf_t buffer_settings = { ///< some default settings, same as in DT
        .frame_start =      {0x00000000, 0x08000000, 0x10000000, 0x18000000}, /* Frame starts (in bytes) */
        .frame_full_width = {      8192,       8192,       8192,       8192}, /* Frame full widths (in bytes). 1 memory page is 2048 bytes (128 bursts) */
        .frame_height =     {      8192,       8192,       8192,       8192}, /* Channel 3 maximal frame height in pixel lines */
        .frames_in_buffer = {         2,          2,          2,          2}  /* Number of frames in channel 3 buffer */
};

static int video_frame_number = 0;
static int membridge_direction = 0; // 0 - from pl to ps, 1 - from ps to pl
static int hardware_initialized = 0;

struct raw_info_t {
	unsigned long frame_num[4];
	unsigned long sec[4];
	unsigned long usec[4];
	int width[4];          ///< frame width - valid pixels [0..width-1]
	int fullwidth[4];      ///< width aligned to DDR memory bursts (128 bits)
	int height[4];
	int bpp[4];
	loff_t offset[4];
};

static struct raw_info_t raw_info = {
	.frame_num = {0,0,0,0},
	.sec       = {0,0,0,0},
	.usec      = {0,0,0,0},
	.width     = {0,0,0,0},
	.fullwidth = {0,0,0,0},
	.height    = {0,0,0,0},
	.bpp       = {0,0,0,0},
	.offset    = {0,0,0,0}
};

/** Setup memory bridge system memory */
int setup_membridge_system_memory(
		int lo_addr64, ///< start address of the system memory range in QWORDs (4 LSBs==0)
		int size64,    ///< size of the system memory range in QWORDs (4 LSBs==0), rolls over
		int start64,   ///< start of transfer offset to system memory range in QWORDs (4 LSBs==0)
		int len64,     ///< Full length of transfer in QWORDs
		int width64)   ///< Frame width in QWORDs (last xfer in each line may be partial)
{

	u29_t lo_addr = {.d32=0};
	u29_t size =    {.d32=0};
	u29_t start =   {.d32=0};
	u29_t len =     {.d32=0};
	u29_t width =   {.d32=0};

	// no need to wait for anything
	pr_debug("setup_membridge_system_memory: lo_addr64=0x%08x  size64=0x%08x  start64=0x%08x  len64=0x%08x  width64=0x%08x\n",
			lo_addr64,
			size64,
			start64,
			len64,
			width64);

	lo_addr.addr64 = lo_addr64;
	size.addr64    = size64;
	start.addr64   = start64;
	len.addr64     = len64;
	width.addr64   = width64;

	x393_membridge_lo_addr64(lo_addr);
	x393_membridge_size64(size);
	x393_membridge_start64(start);
	x393_membridge_len64(len);
	x393_membridge_width64(width);

	return 0;
}

/** Setup memory bridge for a specified sensor channel */
int setup_membridge_memory(
		int num_sensor,       ///< sensor port number (0..3)
		int write_direction,  ///< 0 - from fpga mem to system mem, 1 - otherwise
		int video_frame_number, ///< 0 or 1 for now
        int window_width,     ///< 13-bit - in 8*16=128 bit bursts
        int window_height,    ///< 16-bit window height (in scan lines)
        int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
        int window_top,       ///< 16-bit window top margin (in scan lines
        int start_x,          ///< START X ...
		int start_y,          ///< START Y ...
        x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
        int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                              ///< @return 0 -OK
{
   int frame_sa =         buffer_settings.frame_start[num_sensor] >> (4 + 3) ;
   int frame_full_width = buffer_settings.frame_full_width[num_sensor] >> 4;
   int frame_sa_inc =     frame_full_width * (buffer_settings.frame_height[num_sensor] >>3);
   int last_frame_num =   buffer_settings.frames_in_buffer[num_sensor] - 1;

   x393_mcntrl_window_frame_sa_t          window_frame_sa =       {.d32=0};
   x393_mcntrl_window_frame_sa_inc_t      window_frame_sa_inc =   {.d32=0};
   x393_mcntrl_window_last_frame_num_t    window_last_frame_num = {.d32=0};
   x393_mcntrl_window_full_width_t        window_full_width =     {.d32=0};
   x393_mcntrl_window_width_height_t      window_width_height =   {.d32=0};
   x393_mcntrl_window_left_top_t          window_left_top =       {.d32=0};
   x393_mcntrl_window_startx_starty_t     window_startx_starty =  {.d32=0};

   if (video_frame_number>last_frame_num){
	   video_frame_number = last_frame_num;
   }

   frame_sa += frame_sa_inc*video_frame_number;

   window_frame_sa.frame_sa =             frame_sa;
   window_frame_sa_inc.frame_sa_inc =     frame_sa_inc;
   window_last_frame_num.last_frame_num = last_frame_num;
   window_full_width.full_width =         frame_full_width;
   window_width_height.width =            window_width;
   window_width_height.height =           window_height;
   window_left_top.left =                 window_left;
   window_left_top.top =                  window_top;
   window_startx_starty.start_x =         start_x;
   window_startx_starty.start_y =         start_y;

   dev_dbg(g_dev_ptr,"{sensor %d} setup_membridge_memory frame16=%d, command=%d\n",num_sensor,frame16, (int) x393cmd);
   dev_dbg(g_dev_ptr,"sa=0x%08x  sa_inc=0x%08x  lfn=0x%08x  fw=0x%08x  wh=0x%08x  lt=0x%08x  sxy=0x%08x\n",
           window_frame_sa.d32,window_frame_sa_inc.d32, window_last_frame_num.d32, window_full_width.d32,
           window_width_height.d32,window_left_top.d32, window_startx_starty.d32);

   membridge_direction = write_direction;

   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
	   /**
	    * Possible but unlikely future implementation
	    */
	   /*
       seqr_x393_membridge_scanline_startaddr       (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqr_x393_membridge_scanline_frame_size      (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqr_x393_membridge_scanline_frame_last      (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqr_x393_membridge_scanline_frame_full_width(frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqr_x393_membridge_scanline_window_wh       (frame16, window_width_height,     num_sensor); // Set frame window size
       seqr_x393_membridge_scanline_window_x0y0     (frame16, window_left_top,         num_sensor); // Set frame position
       seqr_x393_membridge_scanline_startxy         (frame16, window_startx_starty,    num_sensor); // Set start x & y ...
       */
       break;
   case ABSOLUTE:
	   /**
	    * Possible but unlikely future implementation
	    */
	   /*
       seqa_x393_membridge_scanline_startaddr       (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqa_x393_membridge_scanline_frame_size      (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqa_x393_membridge_scanline_frame_last      (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqa_x393_membridge_scanline_frame_full_width(frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqa_x393_membridge_scanline_window_wh       (frame16, window_width_height,     num_sensor); // Set frame window size
       seqa_x393_membridge_scanline_window_x0y0     (frame16, window_left_top,         num_sensor); // Set frame position
       seqa_x393_membridge_scanline_startxy         (frame16, window_startx_starty,    num_sensor); // Set start x & y ...
       */
       break;
   case DIRECT:
	   x393_membridge_scanline_startaddr             (window_frame_sa);       // Set frame start address
	   x393_membridge_scanline_frame_size            (window_frame_sa_inc);   // Set frame size (address increment)
	   x393_membridge_scanline_frame_last            (window_last_frame_num); // Set last frame number (number of frames in buffer minus 1)
	   x393_membridge_scanline_frame_full_width      (window_full_width);     // Set frame full(padded) width
       x393_membridge_scanline_window_wh             (window_width_height);   // Set frame window size
       x393_membridge_scanline_window_x0y0           (window_left_top);       // Set frame position
       x393_membridge_scanline_startxy               (window_startx_starty);  // Set start x & y ...
       break;
   }

   return 0;
}

/** Control membridge memory */
int  control_membridge_memory (int num_sensor,    ///< sensor port number (0..3)
                               int cmd,           ///< command: 0 stop, 1 - single, 2 - repetitive, 3 - reset
                               x393cmd_t x393cmd, ///< how to apply commands - directly or through channel sequencer
                               int frame16)       ///< Frame number the command should be applied to (if not immediate mode)
                                                  ///< @return 0 -OK
{

   x393_mcntrl_mode_scan_t membridge_mode = {.enable =        1, // [    0] (1) enable requests from this channel ( 0 will let current to finish, but not raise want/need)
                                             .chn_nreset =    1, // [    1] (1) 0: immediately reset all the internal circuitry
                                             .write_mem =     0, // [    2] (0) 0 - read from memory, 1 - write to memory
                                             .extra_pages =   1, // [ 4: 3] (0) 2-bit number of extra pages that need to stay (not to be overwritten) in the buffer
                                             .keep_open =     0, // [    5] (0) (NA in linescan) for 8 or less rows - do not close page between accesses (not used in scanline mode)
                                             .byte32 =        1, // [    6] (1) (NA in linescan) 32-byte columns (0 - 16-byte), not used in scanline mode
                                             .reset_frame =   1, // [    8] (0) reset frame number
                                             .single =        0, // [    9] (0) run single frame
                                             .repetitive =    1, // [   10] (1) run repetitive frames
                                             .disable_need =  1, // [   11] (0) disable 'need' generation, only 'want' (compressor channels)
                                             .skip_too_late = 1, // [   12] (0) Skip over missed blocks to preserve frame structure (increment pointers)
	                                         .copy_frame =    1, // [   13] (0) Copy frame number from the master (sensor) channel. Combine with reset_frame to reset bjuffer
                                             .abort_late =    1};// [   14] (0) abort frame if not finished by the new frame sync (wait pending memory transfers)

   x393_membridge_cmd_t membridge_cmd = {.enable      = 1, // [    0] (0) enable membridge
		   	   	   	   	   	   	   	   	 .start_reset = 3};// [ 2: 1] (0) 1 - start (from current address), 3 - start from reset address

   membridge_mode.reset_frame = 1;

   membridge_mode.single = 1;
   membridge_mode.repetitive = 0;
   //membridge_mode.single = 0;
   //membridge_mode.repetitive = 1;

   membridge_mode.extra_pages = 0;

   // TODO: direction can be changed to copy data to fpga memory (the one that's not system memory)
   membridge_mode.write_mem = membridge_direction;

   membridge_cmd.enable = cmd;
   membridge_cmd.start_reset = cmd;

   //membridge_cmd.enable = 1;
   //membridge_cmd.start_reset = 3;

   // enable channel 1 (it's a memory controller hardcoded channel)
   if (membridge_mode.enable){
          memchan_enable(1, 1); // just enable - nothing will change if it was already enabled. Never disabled
   }

   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
	   /** no need */
       //seqr_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case ABSOLUTE:
	   /** no need */
       //seqa_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case DIRECT:

	   // TODO: 1. wait until absolute frame number?
	   //       2. wait until copying is done? or not?

	   x393_membridge_scanline_mode(membridge_mode); // Set mode register (write last after other channel registers are set)
	   x393_membridge_ctrl(membridge_cmd); // Set mode register (write last after other channel registers are set)
	   //get_x393_membridge_status_cntrl();?!
       break;
   }

   return 0;
}

/** Setup memory controller for a sensor channel */
int  setup_sensor_memory (int num_sensor,      ///< sensor port number (0..3)
                         int window_width,     ///< 13-bit - in 8*16=128 bit bursts
                         int window_height,    ///< 16-bit window height (in scan lines)
                         int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
                         int window_top,       ///< 16-bit window top margin (in scan lines)
                         x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                         int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                               ///< @return 0 -OK
//typedef enum {DIRECT,ABSOLUTE,RELATIVE} x393cmd_t;
{
   int frame_sa =         buffer_settings.frame_start[num_sensor] >> (4 + 3) ;
   int frame_full_width = buffer_settings.frame_full_width[num_sensor] >> 4;
   int frame_sa_inc =     frame_full_width * (buffer_settings.frame_height[num_sensor] >>3);
   int last_frame_num =   buffer_settings.frames_in_buffer[num_sensor] - 1;

   x393_mcntrl_window_frame_sa_t          window_frame_sa =       {.d32=0};
   x393_mcntrl_window_frame_sa_inc_t      window_frame_sa_inc =   {.d32=0};
   x393_mcntrl_window_last_frame_num_t    window_last_frame_num = {.d32=0};
   x393_mcntrl_window_full_width_t        window_full_width =     {.d32=0};
   x393_mcntrl_window_width_height_t      window_width_height =   {.d32=0};
   x393_mcntrl_window_left_top_t          window_left_top =       {.d32=0};

   window_frame_sa.frame_sa =             frame_sa;
   window_frame_sa_inc.frame_sa_inc =     frame_sa_inc;
   window_last_frame_num.last_frame_num = last_frame_num;
   window_full_width.full_width =         frame_full_width;
   window_width_height.width =            window_width;
   window_width_height.height =           window_height;
   window_left_top.left =                 window_left;
   window_left_top.top =                  window_top;

   dev_dbg(g_dev_ptr,"{%d} setup_sensor_memory frame16=%d, command=%d\n",num_sensor,frame16, (int) x393cmd);
   dev_dbg(g_dev_ptr,"sa=0x%08x sa_inc=0x%08x lfn=0x%08x fw=0x%08x wh=0x%08x lt=0x%08x\n",
           window_frame_sa.d32,window_frame_sa_inc.d32, window_last_frame_num.d32, window_full_width.d32,window_width_height.d32,window_left_top.d32);

   MDP(DBGB_VM,num_sensor,"frame16=%d, command=%d\n", frame16, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"sa=0x%08x sa_inc=0x%08x lfn=0x%08x fw=0x%08x wh=0x%08x lt=0x%08x\n",
           window_frame_sa.d32,window_frame_sa_inc.d32, window_last_frame_num.d32, window_full_width.d32,window_width_height.d32,window_left_top.d32)


   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
       seqr_x393_sens_mcntrl_scanline_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqr_x393_sens_mcntrl_scanline_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqr_x393_sens_mcntrl_scanline_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqr_x393_sens_mcntrl_scanline_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqr_x393_sens_mcntrl_scanline_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
       seqr_x393_sens_mcntrl_scanline_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
       break;
   case ABSOLUTE:
       seqa_x393_sens_mcntrl_scanline_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqa_x393_sens_mcntrl_scanline_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqa_x393_sens_mcntrl_scanline_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqa_x393_sens_mcntrl_scanline_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqa_x393_sens_mcntrl_scanline_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
       seqa_x393_sens_mcntrl_scanline_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
       break;
   case DIRECT:
       x393_sens_mcntrl_scanline_startaddr             (window_frame_sa,         num_sensor); // Set frame start address
       x393_sens_mcntrl_scanline_frame_size            (window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       x393_sens_mcntrl_scanline_frame_last            (window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       x393_sens_mcntrl_scanline_frame_full_width      (window_full_width,       num_sensor); // Set frame full(padded) width
       x393_sens_mcntrl_scanline_window_wh             (window_width_height,     num_sensor); // Set frame window size
       x393_sens_mcntrl_scanline_window_x0y0           (window_left_top,         num_sensor); // Set frame position
       break;
   }

   return 0;
}

/** Control (stop/single/run/reset) memory controller for a sensor channel */
int  control_sensor_memory (int num_sensor,       ///< sensor port number (0..3)
                            int cmd,              ///< command: 0 stop, 1 - single, 2 - repetitive, 3 - reset
                            int reset_frame,      ///< reset addresses to the start of frame, reset buffer (1 of 4) pointer.
                                                  ///< Should only be used if the channel controller was stopped before
                            x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                            int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                                  ///< @return 0 -OK
{

   x393_mcntrl_mode_scan_t mcntrl_mode = {.enable =        1, // [    0] (1) enable requests from this channel ( 0 will let current to finish, but not raise want/need)
                                          .chn_nreset =    1, // [    1] (1) 0: immediately reset all the internal circuitry
                                          .write_mem =     1, // [    2] (0) 0 - read from memory, 1 - write to memory
                                          .extra_pages =   0, // [ 4: 3] (0) 2-bit number of extra pages that need to stay (not to be overwritten) in the buffer
                                          .keep_open =     0, // [    5] (0) (NA in linescan) for 8 or less rows - do not close page between accesses (not used in scanline mode)
                                          .byte32 =        0, // [    6] (1) (NA in linescan) 32-byte columns (0 - 16-byte), not used in scanline mode
                                          .reset_frame =   0, // [    8] (0) reset frame number
                                          .single =        0, // [    9] (0) run single frame
                                          .repetitive =    1, // [   10] (1) run repetitive frames
                                          .disable_need =  0, // [   11] (0) disable 'need' generation, only 'want' (compressor channels)
                                          .skip_too_late = 1, // [   12] (0) Skip over missed blocks to preserve frame structure (increment pointers)
                                          .abort_late =    1};// [   14] (0) abort frame if not finished by the new frame sync (wait pending memory transfers)
   mcntrl_mode.reset_frame =  reset_frame;
   switch (cmd){
   case SENSOR_RUN_STOP:
       mcntrl_mode.enable = 0;
       break;
   case SENSOR_RUN_SINGLE:
       mcntrl_mode.single = 1;
       mcntrl_mode.repetitive = 0;
       break;
   case SENSOR_RUN_CONT:
       break;
   case SENSOR_RUN_RESET:
       mcntrl_mode.enable = 0;
       mcntrl_mode.chn_nreset = 0;
       break;
   default:
       return -EINVAL;
   }

   dev_dbg(g_dev_ptr,"{%d}  frame16=%d, cmd=%d, x393cms=%d\n",num_sensor,frame16, cmd, (int) x393cmd);
   dev_dbg(g_dev_ptr,"control_sensor_memory mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32,         mcntrl_mode.enable,     mcntrl_mode.chn_nreset,   mcntrl_mode.write_mem,
		   mcntrl_mode.extra_pages, mcntrl_mode.keep_open,  mcntrl_mode.byte32,       mcntrl_mode.reset_frame,
		   mcntrl_mode.single,      mcntrl_mode.repetitive, mcntrl_mode.disable_need, mcntrl_mode.skip_too_late);
   MDP(DBGB_VM,num_sensor,"frame16=%d, cmd=%d, x393cms=%d\n",frame16, cmd, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"control_sensor_memory mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32,         mcntrl_mode.enable,    mcntrl_mode.chn_nreset,   mcntrl_mode.write_mem,
		   mcntrl_mode.extra_pages, mcntrl_mode.keep_open, mcntrl_mode.byte32,       mcntrl_mode.reset_frame,
		   mcntrl_mode.single,      mcntrl_mode.repetitive,mcntrl_mode.disable_need, mcntrl_mode.skip_too_late)


   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
       seqr_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case ABSOLUTE:
       seqa_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case DIRECT:
       x393_sens_mcntrl_scanline_mode                  (mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   }
   if (mcntrl_mode.enable){
       memchan_enable(num_sensor + VIDEOMEM_SENSOR_CHANNEL0, 1); // just enable - nothing will change if it was already enabled. Never disabled
       MDP(DBGB_VM,num_sensor,"memchan_enable(%d,1)\n",num_sensor + VIDEOMEM_SENSOR_CHANNEL0)
   }
   return 0;
}

/** Setup memory controller for a compressor channel */
int setup_compressor_memory (int num_sensor,       ///< sensor port number (0..3)
                            int window_width,     ///< 13-bit - in 8*16=128 bit bursts
                            int window_height,    ///< 16-bit window height (in scan lines)
                            int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
                            int window_top,       ///< 16-bit window top margin (in scan lines
                            int tile_width,       ///< tile width in bursts (16-pixels each)
                            int tile_height,      ///< tile height: 18 for color JPEG, 16 for JP4 flavors // = 18
                            int tile_vstep,       ///< tile vertical step in pixel rows (JPEG18/jp4 = 16) // = 16
                            x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                            int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                                  ///< @return 0 - OK
{
    int frame_sa =         buffer_settings.frame_start[num_sensor] >> (4 + 3) ;
    int frame_full_width = buffer_settings.frame_full_width[num_sensor] >> 4;
    int frame_sa_inc =     frame_full_width * (buffer_settings.frame_height[num_sensor] >>3);
    int last_frame_num =   buffer_settings.frames_in_buffer[num_sensor] - 1;
//    int byte32 =           1;   ///< 1 - 32-byte columns (currently used), 0 - 16 byte columns

   x393_mcntrl_window_frame_sa_t          window_frame_sa =       {.d32=0};
   x393_mcntrl_window_frame_sa_inc_t      window_frame_sa_inc =   {.d32=0};
   x393_mcntrl_window_last_frame_num_t    window_last_frame_num = {.d32=0};
   x393_mcntrl_window_full_width_t        window_full_width =     {.d32=0};
   x393_mcntrl_window_width_height_t      window_width_height =   {.d32=0};
   x393_mcntrl_window_left_top_t          window_left_top =       {.d32=0};
   x393_mcntrl_window_tile_whs_t          window_tile_whs =       {.d32=0};

   window_frame_sa.frame_sa =             frame_sa;
   window_frame_sa_inc.frame_sa_inc =     frame_sa_inc;
   window_last_frame_num.last_frame_num = last_frame_num;
   window_full_width.full_width =         frame_full_width;
   window_width_height.width =            window_width;
   window_width_height.height =           window_height;
   window_left_top.left =                 window_left;
   window_left_top.top =                  window_top;
   window_tile_whs.tile_width =           tile_width;
   window_tile_whs.vert_step =            tile_vstep;
   window_tile_whs.tile_height =          tile_height;

   dev_dbg(g_dev_ptr,"{%d} setup_compressor_memory frame16=%d, command=%d\n",num_sensor,frame16, (int) x393cmd);
   dev_dbg(g_dev_ptr,"sa=0x%08x sa_inc=0x%08x lfn=0x%08x fw=0x%08x wh=0x%08x lt=0x%08x whs=0x%08x\n",
           window_frame_sa.d32,window_frame_sa_inc.d32, window_last_frame_num.d32, window_full_width.d32,
           window_width_height.d32,window_left_top.d32, window_tile_whs.d32);

   MDP(DBGB_VM,num_sensor,"frame16=%d, command=%d\n", frame16, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"sa=0x%08x sa_inc=0x%08x lfn=0x%08x fw=0x%08x wh=0x%08x lt=0x%08x whs=0x%08x\n",
           window_frame_sa.d32,window_frame_sa_inc.d32, window_last_frame_num, window_full_width.d32,
           window_width_height.d32,window_left_top.d32, window_tile_whs.d32)




   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
       seqr_x393_sens_mcntrl_tiled_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqr_x393_sens_mcntrl_tiled_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqr_x393_sens_mcntrl_tiled_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqr_x393_sens_mcntrl_tiled_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqr_x393_sens_mcntrl_tiled_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
       seqr_x393_sens_mcntrl_tiled_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
       seqr_x393_sens_mcntrl_tiled_tile_whs         (frame16, window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
       break;
   case ABSOLUTE:
       seqa_x393_sens_mcntrl_tiled_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
       seqa_x393_sens_mcntrl_tiled_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       seqa_x393_sens_mcntrl_tiled_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       seqa_x393_sens_mcntrl_tiled_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
       seqa_x393_sens_mcntrl_tiled_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
       seqa_x393_sens_mcntrl_tiled_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
       seqa_x393_sens_mcntrl_tiled_tile_whs         (frame16, window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
       break;
   case DIRECT:
       x393_sens_mcntrl_tiled_startaddr             (window_frame_sa,         num_sensor); // Set frame start address
       x393_sens_mcntrl_tiled_frame_size            (window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
       x393_sens_mcntrl_tiled_frame_last            (window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
       x393_sens_mcntrl_tiled_frame_full_width      (window_full_width,       num_sensor); // Set frame full(padded) width
       x393_sens_mcntrl_tiled_window_wh             (window_width_height,     num_sensor); // Set frame window size
       x393_sens_mcntrl_tiled_window_x0y0           (window_left_top,         num_sensor); // Set frame position
       x393_sens_mcntrl_tiled_tile_whs              (window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
       break;
   }

   return 0;
}


/** Control memory controller (stop/single/run/reset) for a compressor channel */
int control_compressor_memory (int num_sensor,       ///< sensor port number (0..3)
                               int cmd,              ///< command: 0 stop, 1 - single, 2 - repetitive, 3 - reset
                               int reset_frame,      ///< reset addresses to the start of frame, reset buffer (1 of 4) pointer.
                                                     ///< Should only be used if the channel controller was stopped before
                               int linear,           ///< 0  tiled, 1 - linescan (for raw images)
                               int extra_pages,      ///< extra pages needed (1) - number of previous pages to keep in a 4-page buffer
                               int disable_need,     ///< disable "need" (yield to sensor channels - they can not wait)
                               x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                               int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                                     ///< @return 0 - OK
{
   x393_mcntrl_mode_scan_t mcntrl_mode = {.chn_nreset =    1, // [    0] (1) 0: immediately reset all the internal circuitry
                                          .enable =        1, // [    1] (1)  enable requests from this channel ( 0 will let current to finish, but not raise want/need)
                                          .write_mem =     0, // [    2] (0) 0 - read from memory, 1 - write to memory
                                          .extra_pages =   1, // [ 4: 3] (0) 2-bit number of extra pages that need to stay (not to be overwritten) in the buffer
                                          .keep_open =     0, // [    5] (0) (NA in linescan) for 8 or less rows - do not close page between accesses (not used in scanline mode)
                                          .byte32 =        1, // [    6] (1) (NA in linescan) 32-byte columns (0 - 16-byte), not used in scanline mode
                                          .linear =   linear, // [    7] (1) Use linear mode instead of tiled (for raw image files): extra_pages=0, keep_open=x, byte32=x
                                          .reset_frame =   1, // [    8] (0) reset frame number
                                          .single =        0, // [    9] (0) run single frame
                                          .repetitive =    1, // [   10] (1) run repetitive frames
                                          .disable_need =  1, // [   11] (0) disable 'need' generation, only 'want' (compressor channels)
                                          .skip_too_late = 1, // [   12] (0) Skip over missed blocks to preserve frame structure (increment pointers)
                                          .copy_frame =    1, // [   13] (0) Copy frame number from the master (sensor) channel. Combine with reset_frame to reset bjuffer
                                          .abort_late =    1};// [   14] (0) abort frame if not finished by the new frame sync (wait pending memory transfers)
   mcntrl_mode.disable_need = disable_need;
   mcntrl_mode.extra_pages =  extra_pages;
   mcntrl_mode.reset_frame =  reset_frame;
   switch (cmd){
   case COMPRESSOR_RUN_STOP:
       mcntrl_mode.enable = 0;
       break;
   case COMPRESSOR_RUN_SINGLE:
       mcntrl_mode.single = 1;
       mcntrl_mode.repetitive = 0;
       break;
   case COMPRESSOR_RUN_CONT:
       break;
   case COMPRESSOR_RUN_RESET:
       mcntrl_mode.chn_nreset = 0;
       mcntrl_mode.enable = 0;
       break;
   default:
       return -EINVAL;
   }
   dev_dbg(g_dev_ptr,"{%d}  frame16=%d, cmd=%d, x393cms=%d\n",num_sensor,frame16, cmd, (int) x393cmd);
   dev_dbg(g_dev_ptr,"control_compressor_memory mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, linear=%d, reset_frame=%d, single=%d, repetitive=%d, disable_need=%d, skip_too_late=%d, copy_frame=%d, abort_late=%d)\n",
           mcntrl_mode.d32,           mcntrl_mode.enable,     mcntrl_mode.chn_nreset, mcntrl_mode.write_mem,
		   mcntrl_mode.extra_pages,   mcntrl_mode.keep_open,  mcntrl_mode.byte32,     mcntrl_mode.linear,
		   mcntrl_mode.reset_frame,   mcntrl_mode.single,     mcntrl_mode.repetitive, mcntrl_mode.disable_need,
		   mcntrl_mode.skip_too_late, mcntrl_mode.copy_frame, mcntrl_mode.abort_late);
   MDP(DBGB_VM,num_sensor,"frame16=%d, cmd=%d, x393cms=%d\n",frame16, cmd, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"control_compressor_memory mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, linear=%d, reset_frame=%d, single=%d, repetitive=%d, disable_need=%d, skip_too_late=%d, copy_frame=%d, abort_late=%d)\n",
           mcntrl_mode.d32,           mcntrl_mode.enable,     mcntrl_mode.chn_nreset, mcntrl_mode.write_mem,
		   mcntrl_mode.extra_pages,   mcntrl_mode.keep_open,  mcntrl_mode.byte32,     mcntrl_mode.linear,
		   mcntrl_mode.reset_frame,   mcntrl_mode.single,     mcntrl_mode.repetitive, mcntrl_mode.disable_need,
		   mcntrl_mode.skip_too_late, mcntrl_mode.copy_frame, mcntrl_mode.abort_late)

   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
       seqr_x393_sens_mcntrl_tiled_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case ABSOLUTE:
       seqa_x393_sens_mcntrl_tiled_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case DIRECT:
       x393_sens_mcntrl_tiled_mode                  (mcntrl_mode,                      num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   }
   if (mcntrl_mode.enable){
       memchan_enable(num_sensor + VIDEOMEM_COMPRESSOR_CHANNEL0, 1); // just enable - nothing will change if it was already enabled. Never disabled
       MDP(DBGB_VM,num_sensor,"memchan_enable(%d,1)\n",num_sensor + VIDEOMEM_COMPRESSOR_CHANNEL0)
   }
   return 0;
}

/** Return number of rfames in videobuffer for the sensor port minus 1 (0 means single frame buffer) */
int frames_in_buffer_minus_one(int num_sensor) ///< sensor port number (0..3)
                                               ///< @return number of frames in buffer - 1 (<0 for disabled channels ?)
{
    return buffer_settings.frames_in_buffer[num_sensor] - 1;

}

/** Enable/disable extrenal memory channel. Uses read-modify-write, so does not work through the sequencer */
void memchan_enable(int chn,    ///< External memory channel (0..15)
                    int enable) ///< 1 - enable, 0 - disable
{
    x393_mcntr_chn_en_t chn_en;
    spin_lock(&lock);
    chn_en= get_x393_mcntrl_chn_en();
    if (enable){
        chn_en.chn_en |= 1 << chn;
    } else {
        chn_en.chn_en &= ~(1 << chn);
    }
    set_x393_mcntrl_chn_en(chn_en);
    spin_unlock(&lock);
}

/** Set priority of a memory channel.
 *  Each channel has a 16-bit counter that increments each time any other channel is granted, and is reset to the specified priority when
 *  this channel is granted memory access.
 *
 *  There are two groups of urgency implemented "want" (can accept/provide data) and "need" (buffer will soon be full for memory writes
 *  or empty for memory reads). "Need" capability can be disable for some channels - by default for compressor ones as they may tolerate
 *  waiting longer.
 *  Arbiter selects channel with highest value of the counter (how long it waited, but started from priority) first among channels in
 *  "need", then among those just "wanting" memory access.
 *  By default all channels have equal priority of 0 */
void set_memchannel_priority(int chn,       ///< Memory channel (0..16). When using for sensor/compressor - add VIDEOMEM_SENSOR_CHANNEL0/VIDEOMEM_COMPRESSOR_CHANNEL0
                             u16 priority)  ///< 16-bit unsigned channel priority, 0 - lowest
{
    x393_arbiter_pri_t arbiter_pri;
    arbiter_pri.priority = priority;
    set_x393_mcntrl_arbiter_priority(arbiter_pri, chn);

}

/** Get current channel priority
 *  @see set_memchannel_priority */
u16 get_memchannel_priority(int chn)       ///< Memory channel (0..16). When using for sensor/compressor - add VIDEOMEM_SENSOR_CHANNEL0/VIDEOMEM_COMPRESSOR_CHANNEL0
                                           ///< @return 16-bit unsigned channel priority, 0 - lowest
{
    x393_arbiter_pri_t arbiter_pri = get_x393_mcntrl_arbiter_priority(chn);
    return arbiter_pri.priority;
}

/** @brief Videomem buffer private data */
struct raw_priv_t {
	int                 sensor_port;                       ///< sensor number
	unsigned long       *buf_ptr;                          ///< pointer to raw buffer memory region
	unsigned long       buf_size;                          ///< circular region size in bytes
	dma_addr_t          phys_addr;                         ///< physical address of memory region reported by memory driver
};

static inline int membridge_is_busy(void){
	x393_status_membridge_t status = x393_membridge_status();
	return status.busy;
}

int membridge_start(int sensor_port, unsigned long target_frame){

	unsigned long frame_num;
	unsigned long seconds, useconds;
	//unsigned long frame_num_this, frame_num_past;
	int p_color, p_pf_height;
	int width_marg, height_marg, width_bursts;

	dma_addr_t     phys_addr;
	unsigned long  buf_size;

	x393_status_membridge_t status;

	// to enable interrupt
    x393_membridge_ctrl_irq_t membridge_irq_en = {.d32=0};
	membridge_irq_en.interrupt_cmd = MEMBRIDGE_CMD_IRQ_EN;

	// check fpga
	if (!is_fpga_programmed()){
		pr_err("*** Attempted to access hardware without bitstream ***\n");
	    return - ENODEV;
	}

	// extra variable
	if (!hardware_initialized){
	    // enable interrupt
	    x393_membridge_ctrl_irq(membridge_irq_en);
	    hardware_initialized = 1;
	}

	// wait for target frame here
	// if target_frame is less or equal than current frame
	waitFrame(sensor_port,target_frame);

	// update sizes (local vars)
	switch(sensor_port){
		case 0:
			phys_addr = pElphel_buf->raw_chn0_paddr;
			buf_size = pElphel_buf->raw_chn0_size*PAGE_SIZE;
			break;
		case 1:
			phys_addr = pElphel_buf->raw_chn1_paddr;
			buf_size = pElphel_buf->raw_chn1_size*PAGE_SIZE;
			break;
		case 2:
			phys_addr = pElphel_buf->raw_chn2_paddr;
			buf_size = pElphel_buf->raw_chn2_size*PAGE_SIZE;
			break;
		case 3:
			phys_addr = pElphel_buf->raw_chn3_paddr;
			buf_size = pElphel_buf->raw_chn3_size*PAGE_SIZE;
			break;
		default:
			// temporary
			phys_addr = pElphel_buf->raw_chn0_paddr;
			buf_size = pElphel_buf->raw_chn0_size*PAGE_SIZE;
	}


	// membridge busy lock
	spin_lock(&membridge_lock);

	if (membridge_locked==0){
		membridge_locked=1;
	}else{
		spin_unlock(&membridge_lock);
		return -EBUSY;
	}

	// if went through - get frame number and parameters
	frame_num = getThisFrameNumber(sensor_port);

	seconds  = get_globalParam(sensor_port,G_SECONDS);
	useconds = get_globalParam(sensor_port,G_MICROSECONDS);

	//frame_num_this = get_imageParamsThis(sensor_port,P_FRAME);
	//frame_num_past = get_imageParamsPast(sensor_port,P_FRAME,frame_num);

	// get all required parameters under spinlock

    width_marg  = get_imageParamsThis(sensor_port,P_ACTUAL_WIDTH);
    height_marg = get_imageParamsThis(sensor_port,P_ACTUAL_HEIGHT);

    p_color     = get_imageParamsThis(sensor_port,P_COLOR);
    p_pf_height = get_imageParamsThis(sensor_port,P_PF_HEIGHT);

	spin_unlock(&membridge_lock);


    switch(p_color){
        case COLORMODE_COLOR:
        case COLORMODE_COLOR20:
            width_marg += (2 * COLOR_MARGINS);
            if ((p_pf_height & 0xffff)==0) { // not a photofinish
                height_marg += (2 * COLOR_MARGINS);
            }
            break;
     }

     width_bursts = (width_marg >> 4) + ((width_marg & 0xf) ? 1 : 0);

     pr_debug("VIDEOMEM: frame_num=%d width_marg=%d  height_marg=%d width_burts=%d", frame_num, width_marg, height_marg, width_bursts);

     setup_membridge_memory(sensor_port, ///< sensor port number (0..3)
     					   0,                     ///< 0 - from fpga mem to system mem, 1 - otherwise
     					   video_frame_number,    ///< 0/1 - for now, frame number in video memory
 						   width_bursts,          ///< 13-bit - in 8*16=128 bit bursts
 						   height_marg,           ///< 16-bit window height (in scan lines)
 						   0,                     ///< 13-bit window left margin in 8-bursts (16 bytes)
 						   0,                     ///< 16-bit window top margin (in scan lines)
 						   0,                     ///< START X ...
 						   0,                     ///< START Y ...
 						   DIRECT,                ///< how to apply commands - directly or through channel sequencer
						   frame_num);            ///< Frame number the command should be applied to (if not immediate mode)

 	// setup membridge system memory - everything is in QW
 	setup_membridge_system_memory(
 			phys_addr>>3,
 			buf_size>>3,
 			0, // start offset?
 			(width_bursts<<1)*height_marg,
 			(width_bursts<<1)
 			);

	control_membridge_memory(sensor_port,1,DIRECT,target_frame);

	status = x393_membridge_status();
	pr_debug("membridge status is %d\n",status.busy);

	/*
	// timeout exit?
	while(true){
		status = x393_membridge_status();
		if (!status.busy) break;
	}
	*/

	wait_event_interruptible(videomem_wait_queue, membridge_is_busy()==0);
	pr_debug("Got back from interrupt\n");

	status = x393_membridge_status();
	pr_debug("membridge status is %d\n",status.busy);

	// fill out raw_info
	raw_info.frame_num[sensor_port] = frame_num;
	raw_info.sec[sensor_port]       = seconds;
	raw_info.usec[sensor_port]      = useconds;
	raw_info.width[sensor_port]     = width_marg;
	raw_info.fullwidth[sensor_port] = width_bursts<<4;
	raw_info.height[sensor_port]    = height_marg;
	raw_info.bpp[sensor_port]       = 8;
	raw_info.offset[sensor_port]    = 0;

	//NOTES:

	//// get this frame number
    //#define  thisFrameNumber(p)            GLOBALPARS(p,G_THIS_FRAME)                       // Current frame number (may lag from the hardware)
    //#define  thisCompressorFrameNumber(p)  GLOBALPARS(p,G_COMPRESSOR_FRAME)                 // Current compressed frame number (lags from thisFrameNumber)

	//// wait for certain frame number
	// wait_event_interruptible(aframepars_wait_queue[sensor_port], getThisFrameNumber(sensor_port) >= target_frame);

	//// get frame sizes
	//unsigned long get_imageParamsPast(int sensor_port,  int n,  int frame);

     return 0;
}

/**
 * @brief This function handles read operations for raw files.
 * @param[in]   file  pointer to <em>struct file</em>
 * @param[in]   buf   pointer to buffer where data will be written to
 * @param[in]   count number of bytes written to @e buf
 * @param[in]   off   offset
 * @return      Number of bytes written to @e buf
 */
ssize_t vm_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	unsigned long p;
	struct raw_priv_t * privData = (struct raw_priv_t*) file->private_data;

	p = *off;

	if (p >= privData->buf_size) {
		p = privData->buf_size;
	}

	if ((p + count) > privData->buf_size){
		count = privData->buf_size - p;
	}

	if (count) {
		if (copy_to_user(buf, &privData->buf_ptr[BYTE2DW(p)], count)){
			return -EFAULT;
		}
		*off+=count;
	}
	return count;
}

// 16MB = 4096 pages
// 32MB = 8192 pages
// TODO: don't forget to reset to default value after testing

// TODO: Implement file operations using membridge for both raw memory access and per-channel image raw (including 16-bit)
//static int     videomem_open    (struct inode *inode, struct file *filp) {return 0;}
//static int     videomem_release (struct inode *inode, struct file *filp) {return 0;}
//static loff_t  videomem_lseek   (struct file * file, loff_t offset, int orig) {return 0;}
//static ssize_t videomem_read    (struct file * file, char * buf, size_t count, loff_t *off) {return 0;}
ssize_t videomem_write   (struct file * file, const char * buf, size_t count, loff_t *off) {return 0;}

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
loff_t  videomem_lseek(struct file * file, loff_t offset, int orig){

	unsigned long target_frame;
	struct raw_priv_t * privData = (struct raw_priv_t*) file -> private_data;
	int sensor_port = privData->sensor_port;
    //sec_usec_t sec_usec;
    int res;

    pr_debug("(videomem_lseek) offset=0x%x, orig=0x%x, sensor_port = %d\n", (int)offset, (int)orig, sensor_port);

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
			// ?
			file->f_pos = privData->buf_size + offset;
			break;
		} else {

			target_frame = offset;
			res = membridge_start(sensor_port,target_frame);
			if (res<0) return res;

		}
		break;
	default:
		return -EINVAL;
	}
	// roll-over position
	//while (file->f_pos < 0) file->f_pos+= circbuf_priv_ptr[chn].buf_size;
	//while (file->f_pos > circbuf_priv_ptr[chn].buf_size) file->f_pos-= circbuf_priv_ptr[chn].buf_size;
	return file->f_pos;
}

/**
 * @brief Process raw buffer file opening.
 * @param[in]   inode
 * @param[in]   filp
 * @return      Always 0.
 */
int videomem_open(struct inode *inode, struct file *filp)
{
	struct raw_priv_t *privData;

	int minor;
	int sensor_port;

	pr_debug("VIDEOMEM_OPEN \n");

	privData = (struct raw_priv_t*)kmalloc(sizeof(struct raw_priv_t), GFP_KERNEL);
	if (!privData) {
		return -ENOMEM;
	}
	filp->private_data = privData;

	minor = MINOR(inode->i_rdev);

	sensor_port = minor - DEV393_MINOR(DEV393_IMAGE_RAW0);

	// temporary
	if (sensor_port < 0){
		sensor_port =  0;
	}

	privData->sensor_port = sensor_port;

	switch(minor){
		case DEV393_MINOR(DEV393_IMAGE_RAW0):
			privData->buf_ptr = pElphel_buf->raw_chn0_vaddr;
			privData->phys_addr = pElphel_buf->raw_chn0_paddr;
			privData->buf_size = pElphel_buf->raw_chn0_size*PAGE_SIZE;
			break;
		case DEV393_MINOR(DEV393_IMAGE_RAW1):
			privData->buf_ptr = pElphel_buf->raw_chn1_vaddr;
			privData->phys_addr = pElphel_buf->raw_chn1_paddr;
			privData->buf_size = pElphel_buf->raw_chn1_size*PAGE_SIZE;
			break;
		case DEV393_MINOR(DEV393_IMAGE_RAW2):
			privData->buf_ptr = pElphel_buf->raw_chn2_vaddr;
			privData->phys_addr = pElphel_buf->raw_chn2_paddr;
			privData->buf_size = pElphel_buf->raw_chn2_size*PAGE_SIZE;
			break;
		case DEV393_MINOR(DEV393_IMAGE_RAW3):
			privData->buf_ptr = pElphel_buf->raw_chn3_vaddr;
			privData->phys_addr = pElphel_buf->raw_chn3_paddr;
			privData->buf_size = pElphel_buf->raw_chn3_size*PAGE_SIZE;
			break;
		default:
			// temporary
			privData->buf_ptr = pElphel_buf->raw_chn0_vaddr;
			privData->phys_addr = pElphel_buf->raw_chn0_paddr;
			privData->buf_size = pElphel_buf->raw_chn0_size*PAGE_SIZE;
	}

	// TODO: remove once lseek is tested
	//membridge_start(sensor_port,0);

	return 0;
}

/**
 * @brief Process read operation on raw buffer file
 * @param[in]   file   pointer to file structure corresponding to the circular buffer file
 * @param[out]  buf    pointer to user buffer where the data should be placed
 * @param[in]   count  the size of requested data transfer
 * @param[in]   off    file position the user is accessing
 * @return      The number of bytes copied or negative error code.
 */
ssize_t videomem_read(struct file *file, char *buf, size_t count, loff_t *off)
{
	//unsigned int minor = MINOR(file->f_inode->i_rdev);
	return vm_read(file, buf, count, off);
}

/**
 * @brief Process memory map operation on videomem buffer file.
 * @param[in]   file   pointer to file structure corresponding to the videomem buffer file
 * @param[in]   vma    contains the information about the virtual address range
 * @return      0 if file was mapped successfully and negative error code otherwise
 */
int videomem_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	struct raw_priv_t * privData = (struct raw_priv_t*) file -> private_data;

	dev_dbg(g_dev_ptr, "vm_start = 0x%lx\n", vma->vm_start);
	dev_dbg(g_dev_ptr, "vm_end = 0x%lx\n", vma->vm_end);
	dev_dbg(g_dev_ptr, "vm_pgoff = 0x%lx\n", vma->vm_pgoff);
	dev_dbg(g_dev_ptr, "vm_file = 0x%lx\n", (unsigned long)vma->vm_file);
	dev_dbg(g_dev_ptr, "videomem_buffer_phys_addr = 0x%lx\n", (unsigned long)privData->phys_addr);

	/* remap_pfn_range will mark the range VM_IO and VM_RESERVED */
	ret = remap_pfn_range(vma,
			vma->vm_start,
			privData->phys_addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

	pr_debug("remap_pfn_range returned 0x%x\n", ret);
	if (ret) return -EAGAIN;

	return 0;
}

/**
 * @brief Process raw buffer file closing
 * @param[in]   inode
 * @param[in]   filp
 * @return      0
 */
int videomem_release(struct inode *inode, struct file *filp)
{
	pr_debug("VIDEOMEM_RELEASE\n");

	if (filp->private_data) {
		kfree(filp->private_data);
	}

	return 0;
}

static struct file_operations videomem_fops = {
        owner:    THIS_MODULE,
        open:     videomem_open,
        release:  videomem_release,
        llseek:   videomem_lseek,
        read:     videomem_read,
		mmap:     videomem_mmap,
        write:    videomem_write
};

/** Handle interrupts from membridge (video memory <-> system memory transfers). This handler is installed without SA_INTERRUPT
 * flag meaning that interrupts are enabled during processing. Such behavior is recommended in LDD3. */
static irqreturn_t videomem_irq_handler(int irq,       ///< [in]   irq   interrupt number
                                        void *dev_id)  ///< [in]   dev_id pointer to driver's private data structure
                                                       ///< @return \e IRQ_HANDLED if interrupt was processed and \e IRQ_NONE otherwise
{
    x393_membridge_ctrl_irq_t ctrl_interrupts= {.d32=0};
    ctrl_interrupts.interrupt_cmd = X393_IRQ_RESET;

    //TODO: Do what is needed here
    x393_membridge_ctrl_irq(ctrl_interrupts); // reset interrupt
    membridge_locked = 0;
    wake_up_interruptible(&videomem_wait_queue);
    return IRQ_HANDLED;
}


// SysFS interface to read/modify video memory map
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

static ssize_t show_frame_start(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", buffer_settings.frame_start[get_channel_from_name(attr)]);
}
static ssize_t show_full_width(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", buffer_settings.frame_full_width[get_channel_from_name(attr)]);
}
static ssize_t show_frame_height(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", buffer_settings.frame_height[get_channel_from_name(attr)]);
}
static ssize_t show_frames_in_buffer(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", buffer_settings.frames_in_buffer[get_channel_from_name(attr)]);
}

static ssize_t get_membridge_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	x393_status_membridge_t status = x393_membridge_status();
    return sprintf(buf,"0x%08x\n", status);
}

static ssize_t get_video_frame_num(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", video_frame_number);
}

/**
raw frame info file format (for read):

  absolute frame number = xxx
  width = xxx
  height = xxx
  bits per pixel = xxx

*/
static ssize_t get_raw_frame_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	char * buf0=buf;
	int sensor_port = get_channel_from_name(attr);

    buf += sprintf(buf,"absolute frame number = %lu\n",raw_info.frame_num[sensor_port]);
    buf += sprintf(buf,"timestamp,sec = %lu\n" ,raw_info.sec[sensor_port]);
    buf += sprintf(buf,"timestamp,usec = %lu\n" ,raw_info.usec[sensor_port]);
    buf += sprintf(buf,"fullwidth = %d\n" ,raw_info.fullwidth[sensor_port]);
    buf += sprintf(buf,"width = %d\n" ,raw_info.width[sensor_port]);
    buf += sprintf(buf,"height = %d\n" ,raw_info.height[sensor_port]);
    buf += sprintf(buf,"bits per pixel = %d\n" ,raw_info.bpp[sensor_port]);
    buf += sprintf(buf,"buffer offset = 0x%08x\n" ,raw_info.offset[sensor_port]);

	return buf-buf0;
}

static ssize_t get_mctrl_status_chn(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn = get_channel_from_name(attr);

	x393_status_mcntrl_lintile_t status;

	switch(chn){
		case 1: status = x393_mcntrl_chn1_status(); break;
		case 2: status = x393_mcntrl_chn2_status(); break;
		case 3: status = x393_mcntrl_chn3_status(); break;
		case 4: status = x393_mcntrl_chn4_status(); break;
	}

    return sprintf(buf,"0x%08x\n", status);
}

static ssize_t get_cmprs_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	x393_cmprs_status_t status = x393_cmprs_status(get_channel_from_name(attr));
    return sprintf(buf,"0x%08x\n", status);
}

static ssize_t store_frame_start(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &buffer_settings.frame_start[get_channel_from_name(attr)]);
    return count;
}
static ssize_t store_full_width(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &buffer_settings.frame_full_width[get_channel_from_name(attr)]);
    return count;
}
static ssize_t store_frame_height(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &buffer_settings.frame_height[get_channel_from_name(attr)]);
    return count;
}
static ssize_t store_frames_in_buffer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &buffer_settings.frames_in_buffer[get_channel_from_name(attr)]);
    return count;
}

static ssize_t set_membridge_status_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int in;
	x393_status_ctrl_t d = {.d32=0};
    sscanf(buf, "%x", &in);
    d.d32 = in;
    set_x393_membridge_status_cntrl(d);
    d = get_x393_membridge_status_cntrl();
    pr_info("Set membridge status register to 0x%08x, status: 0x%08x\n",in,d);

    return count;
}

static ssize_t set_membridge(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sensor_port = get_channel_from_name(attr);
	unsigned long target_frame;
	sscanf(buf, "%lu", &target_frame);
	membridge_start(sensor_port,target_frame);
    return count;
}

static ssize_t set_video_frame_num(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%i", &video_frame_number);
    return count;
}

static DEVICE_ATTR(frame_start0,               SYSFS_PERMISSIONS,     show_frame_start,                store_frame_start);
static DEVICE_ATTR(frame_start1,               SYSFS_PERMISSIONS,     show_frame_start,                store_frame_start);
static DEVICE_ATTR(frame_start2,               SYSFS_PERMISSIONS,     show_frame_start,                store_frame_start);
static DEVICE_ATTR(frame_start3,               SYSFS_PERMISSIONS,     show_frame_start,                store_frame_start);
static DEVICE_ATTR(frame_full_width0,          SYSFS_PERMISSIONS,     show_full_width,                 store_full_width);
static DEVICE_ATTR(frame_full_width1,          SYSFS_PERMISSIONS,     show_full_width,                 store_full_width);
static DEVICE_ATTR(frame_full_width2,          SYSFS_PERMISSIONS,     show_full_width,                 store_full_width);
static DEVICE_ATTR(frame_full_width3,          SYSFS_PERMISSIONS,     show_full_width,                 store_full_width);
static DEVICE_ATTR(frame_height0,              SYSFS_PERMISSIONS,     show_frame_height,               store_frame_height);
static DEVICE_ATTR(frame_height1,              SYSFS_PERMISSIONS,     show_frame_height,               store_frame_height);
static DEVICE_ATTR(frame_height2,              SYSFS_PERMISSIONS,     show_frame_height,               store_frame_height);
static DEVICE_ATTR(frame_height3,              SYSFS_PERMISSIONS,     show_frame_height,               store_frame_height);
static DEVICE_ATTR(frames_in_buffer0,          SYSFS_PERMISSIONS,     show_frames_in_buffer,           store_frames_in_buffer);
static DEVICE_ATTR(frames_in_buffer1,          SYSFS_PERMISSIONS,     show_frames_in_buffer,           store_frames_in_buffer);
static DEVICE_ATTR(frames_in_buffer2,          SYSFS_PERMISSIONS,     show_frames_in_buffer,           store_frames_in_buffer);
static DEVICE_ATTR(frames_in_buffer3,          SYSFS_PERMISSIONS,     show_frames_in_buffer,           store_frames_in_buffer);

static DEVICE_ATTR(membridge_status,           SYSFS_PERMISSIONS,     get_membridge_status,            set_membridge_status_reg);
static DEVICE_ATTR(membridge_start0,           SYSFS_PERMISSIONS,     NULL,            set_membridge);
static DEVICE_ATTR(membridge_start1,           SYSFS_PERMISSIONS,     NULL,            set_membridge);
static DEVICE_ATTR(membridge_start2,           SYSFS_PERMISSIONS,     NULL,            set_membridge);
static DEVICE_ATTR(membridge_start3,           SYSFS_PERMISSIONS,     NULL,            set_membridge);

static DEVICE_ATTR(video_frame_number,         SYSFS_PERMISSIONS,     get_video_frame_num,           set_video_frame_num);

static DEVICE_ATTR(raw_frame_info0,            SYSFS_PERMISSIONS,    get_raw_frame_info, NULL);
static DEVICE_ATTR(raw_frame_info1,            SYSFS_PERMISSIONS,    get_raw_frame_info, NULL);
static DEVICE_ATTR(raw_frame_info2,            SYSFS_PERMISSIONS,    get_raw_frame_info, NULL);
static DEVICE_ATTR(raw_frame_info3,            SYSFS_PERMISSIONS,    get_raw_frame_info, NULL);

static DEVICE_ATTR(mctrl_status_chn1,    SYSFS_PERMISSIONS,    get_mctrl_status_chn,  NULL);
static DEVICE_ATTR(mctrl_status_chn2,    SYSFS_PERMISSIONS,    get_mctrl_status_chn,  NULL);
static DEVICE_ATTR(mctrl_status_chn3,    SYSFS_PERMISSIONS,    get_mctrl_status_chn,  NULL);
static DEVICE_ATTR(mctrl_status_chn4,    SYSFS_PERMISSIONS,    get_mctrl_status_chn,  NULL);

static DEVICE_ATTR(compressor_status0,    SYSFS_PERMISSIONS,    get_cmprs_status,  NULL);
static DEVICE_ATTR(compressor_status1,    SYSFS_PERMISSIONS,    get_cmprs_status,  NULL);
static DEVICE_ATTR(compressor_status2,    SYSFS_PERMISSIONS,    get_cmprs_status,  NULL);
static DEVICE_ATTR(compressor_status3,    SYSFS_PERMISSIONS,    get_cmprs_status,  NULL);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_frame_start0.attr,
        &dev_attr_frame_start1.attr,
        &dev_attr_frame_start2.attr,
        &dev_attr_frame_start3.attr,
        &dev_attr_frame_full_width0.attr,
        &dev_attr_frame_full_width1.attr,
        &dev_attr_frame_full_width2.attr,
        &dev_attr_frame_full_width3.attr,
        &dev_attr_frame_height0.attr,
        &dev_attr_frame_height1.attr,
        &dev_attr_frame_height2.attr,
        &dev_attr_frame_height3.attr,
        &dev_attr_frames_in_buffer0.attr,
        &dev_attr_frames_in_buffer1.attr,
        &dev_attr_frames_in_buffer2.attr,
        &dev_attr_frames_in_buffer3.attr,
		&dev_attr_membridge_start0.attr,
		&dev_attr_membridge_start1.attr,
		&dev_attr_membridge_start2.attr,
		&dev_attr_membridge_start3.attr,
		&dev_attr_membridge_status.attr,
		&dev_attr_video_frame_number.attr,
		&dev_attr_raw_frame_info0.attr,
		&dev_attr_raw_frame_info1.attr,
		&dev_attr_raw_frame_info2.attr,
		&dev_attr_raw_frame_info3.attr,
		&dev_attr_mctrl_status_chn1.attr,
		&dev_attr_mctrl_status_chn2.attr,
		&dev_attr_mctrl_status_chn3.attr,
		&dev_attr_mctrl_status_chn4.attr,
		&dev_attr_compressor_status0.attr,
		&dev_attr_compressor_status1.attr,
		&dev_attr_compressor_status2.attr,
		&dev_attr_compressor_status3.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};


static int elphel393_videomem_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}

static int videomem_probe(struct platform_device *pdev)
{
    unsigned int irq;
    int res, i;
    struct device *dev = &pdev->dev;
    const struct of_device_id *match;
//    const __be32 *bufsize_be;
    struct device_node *node;
    // char device for sensor port
    struct device *chrdev;

    elphel393_videomem_sysfs_register(pdev);

    /* sanity check */
    match = of_match_device(elphel393_videomem_of_match, dev);
    if (!match)
        return -EINVAL;

    node = of_find_node_by_name(NULL, "elphel393-videomem");
    if (!node)
    {
        pr_err("Videomem ERROR: No device tree node found\n");
        return -ENODEV;
    }

    buffer_settings.frame_start[0] =      be32_to_cpup((__be32 *)of_get_property(node,  "frame_start_chn0", NULL));
    buffer_settings.frame_start[1] =      be32_to_cpup((__be32 *)of_get_property(node,  "frame_start_chn1", NULL));
    buffer_settings.frame_start[2] =      be32_to_cpup((__be32 *)of_get_property(node,  "frame_start_chn2", NULL));
    buffer_settings.frame_start[3] =      be32_to_cpup((__be32 *)of_get_property(node,  "frame_start_chn3", NULL));
    buffer_settings.frame_full_width[0] = be32_to_cpup((__be32 *)of_get_property(node,  "frame_full_width_chn0", NULL));
    buffer_settings.frame_full_width[1] = be32_to_cpup((__be32 *)of_get_property(node,  "frame_full_width_chn1", NULL));
    buffer_settings.frame_full_width[2] = be32_to_cpup((__be32 *)of_get_property(node,  "frame_full_width_chn2", NULL));
    buffer_settings.frame_full_width[3] = be32_to_cpup((__be32 *)of_get_property(node,  "frame_full_width_chn3", NULL));
    buffer_settings.frame_height[0] =     be32_to_cpup((__be32 *)of_get_property(node,  "frame_height_chn0", NULL));
    buffer_settings.frame_height[1] =     be32_to_cpup((__be32 *)of_get_property(node,  "frame_height_chn1", NULL));
    buffer_settings.frame_height[2] =     be32_to_cpup((__be32 *)of_get_property(node,  "frame_height_chn2", NULL));
    buffer_settings.frame_height[3] =     be32_to_cpup((__be32 *)of_get_property(node,  "frame_height_chn3", NULL));
    buffer_settings.frames_in_buffer[0] = be32_to_cpup((__be32 *)of_get_property(node,  "frames_in_buffer_chn0", NULL));
    buffer_settings.frames_in_buffer[1] = be32_to_cpup((__be32 *)of_get_property(node,  "frames_in_buffer_chn1", NULL));
    buffer_settings.frames_in_buffer[2] = be32_to_cpup((__be32 *)of_get_property(node,  "frames_in_buffer_chn2", NULL));
    buffer_settings.frames_in_buffer[3] = be32_to_cpup((__be32 *)of_get_property(node,  "frames_in_buffer_chn3", NULL));
    // Add more parameters here?
    // Add sysfs modification


    dev_dbg(dev, "Registering character device with name "DEV393_NAME(DEV393_VIDEOMEM_RAW));
//    res = register_chrdev(VIDEOMEM_MAJOR, VIDEOMEM_DRIVER_NAME, &videomem_fops);
    res = register_chrdev(DEV393_MAJOR(DEV393_VIDEOMEM_RAW), DEV393_NAME(DEV393_VIDEOMEM_RAW), &videomem_fops);

    if(res < 0) {
        dev_err(dev, "\videomem_probe: couldn't get a major number  %d.\n ",DEV393_MAJOR(DEV393_VIDEOMEM_RAW));
        return res;
    }

	// create device class
	videomem_dev_class = class_create(THIS_MODULE, DEV393_NAME(DEV393_VIDEOMEM_RAW));
	if (IS_ERR(videomem_dev_class)) {
		pr_err("Cannot create \"%s\" class", DEV393_NAME(DEV393_VIDEOMEM_RAW));
		return PTR_ERR(videomem_dev_class);
	}

	// create devices
	for (i=0;i<(sizeof(videomem_minor)/sizeof(int));i++){
		chrdev = device_create(
				  videomem_dev_class,
				  &pdev->dev,
				  MKDEV(videomem_major, videomem_minor[i]),
				  NULL,
				  "%s",videomem_devs[i]);
		if(IS_ERR(chrdev)){
			pr_err("Failed to create a device (videomem %d). Error code: %ld\n",i,PTR_ERR(chrdev));
		}
	}

    // Setup interrupt
    irq = platform_get_irq_byname(pdev, "membridge_irq");
    if (request_irq(irq,
            videomem_irq_handler,
            0, // no flags
            "membridge_irq",
			NULL)) {
        dev_err(dev, "can not allocate interrupts for %s\n","membridge_irq");
        return -EBUSY;
    }
    init_waitqueue_head(&videomem_wait_queue);    // wait queue for video memory driver
    g_dev_ptr = dev; // for debugfs

    return 0;
}


/** Video memory driver remove function */
static int videomem_remove(struct platform_device *pdev) ///< [in] pointer to @e platform_device structure
                                                         ///< @return always 0
{
	int i;
	for (i=0;i<(sizeof(videomem_minor)/sizeof(int));i++){
		device_destroy(
			videomem_dev_class,
			MKDEV(videomem_major, videomem_minor[i]));
	}
    unregister_chrdev(DEV393_MAJOR(DEV393_VIDEOMEM_RAW), DEV393_NAME(DEV393_VIDEOMEM_RAW));

    return 0;
}

static const struct of_device_id elphel393_videomem_of_match[] = {
        { .compatible = "elphel,elphel393-videomem-1.00" },
        { /* end of list */ }
};

MODULE_DEVICE_TABLE(of, elphel393_videomem_of_match);

static struct platform_driver elphel393_videomem = {
        .probe          = videomem_probe,
        .remove         = videomem_remove,
        .driver = {
                .name =           DEV393_NAME(DEV393_VIDEOMEM_RAW),
                .of_match_table = elphel393_videomem_of_match,
        },
};

module_platform_driver(elphel393_videomem);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(VIDEOMEM_MODULE_DESCRIPTION);
