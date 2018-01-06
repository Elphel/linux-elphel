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
#define DEBUG
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <asm/outercache.h>     // TODO: Implement cache operations for the membridge !!!!
#include <asm/cacheflush.h>

#include <linux/errno.h>
#include <linux/fs.h>

#include <asm/uaccess.h>

#include "x393.h"
#include "x393_macro.h"
#include "pgm_functions.h"
#include "x393_videomem.h"

#include <uapi/elphel/c313a.h>
#include <uapi/elphel/x393_devices.h>

#include <elphel/elphel393-mem.h>

#define VIDEOMEM_MODULE_DESCRIPTION "Video buffer driver"
//#define VIDEOMEM_DRIVER_NAME        "video_mem"
// NC393 debug macros
#include "debug393.h"

static const struct of_device_id elphel393_videomem_of_match[];
static struct device *g_dev_ptr; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
wait_queue_head_t videomem_wait_queue;
static DEFINE_SPINLOCK(lock); // for read-modify-write channel enable

static const char * const videomem_devs[]={
	DEV393_DEVNAME(DEV393_VIDEOMEM_RAW),
	DEV393_DEVNAME(DEV393_IMAGE_RAW)
};

static const int videomem_major = DEV393_MAJOR(DEV393_VIDEOMEM_RAW);

static const int videomem_minor[]={
	DEV393_MINOR(DEV393_VIDEOMEM_RAW),
	DEV393_MINOR(DEV393_IMAGE_RAW)
};

/** @brief Global device class for sysfs */
static struct class *videomem_dev_class;

static struct elphel_video_buf_t buffer_settings = { ///< some default settings, same as in DT
        .frame_start =      {0x00000000, 0x08000000, 0x10000000, 0x08000000}, /* Frame starts (in bytes) */
        .frame_full_width = {      8192,       8192,       8192,       8192}, /* Frame full widths (in bytes). 1 memory page is 2048 bytes (128 bursts) */
        .frame_height =     {      8192,       8192,       8192,       8192}, /* Channel 3 maximal frame height in pixel lines */
        .frames_in_buffer = {         2,          2,          2,          2}  /* Number of frames in channel 3 buffer */
};

static int membridge_sensor_port = 0; // 0..3
static int membridge_direction = 0; // 0 - from pl to ps, 1 - from ps to pl

/** Setup memory bridge system memory */
int setup_membridge_system_memory(
		int lo_addr64, ///< start address of the system memory range in QWORDs (4 LSBs==0)
		int size64,    ///< size of the system memory range in QWORDs (4 LSBs==0), rolls over
		int start64,   ///< start of transfer offset to system memory range in QWORDs (4 LSBs==0)
		int len64,     ///< Full length of transfer in QWORDs
		int width64)   ///< Frame width in QWORDs (last xfer in each line may be partial)
{

	// no need to wait for anything
	pr_debug("setup_membridge_system_memory: lo_addr64=0x%08x  size64=0x%08x  start64=0x%08x  len64=0x%08x  width64=0x%08x\n",
			lo_addr64, size64, start64, len64, width64
			);

	u29_t lo_addr = {.d32=0};
	u29_t size =    {.d32=0};
	u29_t start =   {.d32=0};
	u29_t len =     {.d32=0};
	u29_t width =   {.d32=0};

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
   pr_debug("write direction is: %d (should be 0)\n",membridge_direction);

   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
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

   x393_membridge_cmd_t membridge_cmd = {.enable      = 0, // [    0] (0) enable membridge
		   	   	   	   	   	   	   	   	 .start_reset = 0};// [ 2: 1] (0) 1 - start (from current address), 3 - start from reset address

   membridge_mode.write_mem = membridge_direction;

   membridge_cmd.enable = cmd;
   membridge_cmd.start_reset = cmd;

   //membridge_cmd.enable = 1;
   //membridge_cmd.start_reset = 3;

   switch (x393cmd){
   case ASAP:
       frame16 = 0;
       // no break
   case RELATIVE:
       //seqr_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case ABSOLUTE:
       //seqa_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
       break;
   case DIRECT:

	   // 1. wait until absolute frame number?
	   // 2. wait until copying is done? or not?

	   x393_membridge_scanline_mode(membridge_mode); // Set mode register (write last after other channel registers are set)
	   x393_membridge_ctrl(membridge_cmd); // Set mode register (write last after other channel registers are set)
	   //get_x393_membridge_status_cntrl();?!
       break;
   }

   return 0;
}

/** Setup memory controller for a sensor channel */
int  setup_sensor_memory (int num_sensor,       ///< sensor port number (0..3)
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
   dev_dbg(g_dev_ptr,"mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32, mcntrl_mode.enable, mcntrl_mode.chn_nreset, mcntrl_mode.write_mem, mcntrl_mode.extra_pages,
           mcntrl_mode.keep_open, mcntrl_mode.byte32, mcntrl_mode.reset_frame, mcntrl_mode.single, mcntrl_mode.repetitive,
           mcntrl_mode.disable_need, mcntrl_mode.skip_too_late);
   MDP(DBGB_VM,num_sensor,"frame16=%d, cmd=%d, x393cms=%d\n",frame16, cmd, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32, mcntrl_mode.enable, mcntrl_mode.chn_nreset, mcntrl_mode.write_mem, mcntrl_mode.extra_pages,
           mcntrl_mode.keep_open, mcntrl_mode.byte32, mcntrl_mode.reset_frame, mcntrl_mode.single, mcntrl_mode.repetitive,
           mcntrl_mode.disable_need, mcntrl_mode.skip_too_late)


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
   dev_dbg(g_dev_ptr,"mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32, mcntrl_mode.enable, mcntrl_mode.chn_nreset, mcntrl_mode.write_mem, mcntrl_mode.extra_pages,
           mcntrl_mode.keep_open, mcntrl_mode.byte32, mcntrl_mode.reset_frame, mcntrl_mode.single, mcntrl_mode.repetitive,
           mcntrl_mode.disable_need, mcntrl_mode.skip_too_late);
   MDP(DBGB_VM,num_sensor,"frame16=%d, cmd=%d, x393cms=%d\n",frame16, cmd, (int) x393cmd)
   MDP(DBGB_VM,num_sensor,"mode=0x%x (enable=%d, chn_nreset=%d, write_mem=%d, extra_pages=%d, keep_open=%d, byte32=%d, reset_frame=%d, single=%d,repetitive=%d, disable_need=%d,skip_too_late=%d )\n",
           mcntrl_mode.d32, mcntrl_mode.enable, mcntrl_mode.chn_nreset, mcntrl_mode.write_mem, mcntrl_mode.extra_pages,
           mcntrl_mode.keep_open, mcntrl_mode.byte32, mcntrl_mode.reset_frame, mcntrl_mode.single, mcntrl_mode.repetitive,
           mcntrl_mode.disable_need, mcntrl_mode.skip_too_late)

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
	int                 minor;                             ///< device file minor number
	unsigned long       *buf_ptr;                          ///< pointer to raw buffer memory region
	unsigned long       buf_size;                          ///< circular region size in bytes
    unsigned long       buf_size32;                        ///< circular region size in dwords
	dma_addr_t          phys_addr;                         ///< physical address of memory region reported by memory driver
};

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

/*
// raw channel 0 (in Coherent DMA buffer)
.raw_chn0_vaddr = NULL,
.raw_chn0_paddr = 0,
.raw_chn0_size = 8192,

// raw channel 1 (in Coherent DMA buffer)
.raw_chn1_vaddr = NULL,
.raw_chn1_paddr = 0,
.raw_chn1_size = 0,

// raw channel 2 (in Coherent DMA buffer)
.raw_chn2_vaddr = NULL,
.raw_chn2_paddr = 0,
.raw_chn2_size = 0,

// raw channel 3 (in Coherent DMA buffer)
.raw_chn3_vaddr = NULL,
.raw_chn3_paddr = 0,
.raw_chn3_size = 0
*/
//struct elphel_buf_t *pElphel_buf;

// 16MB = 4096 pages
// 32MB = 8192 pages
// TODO: don't forget to reset to default value after testing

// TODO: Implement file operations using membridge for both raw memory access and per-channel image raw (including 16-bit)
//static int     videomem_open    (struct inode *inode, struct file *filp) {return 0;}
//static int     videomem_release (struct inode *inode, struct file *filp) {return 0;}
static loff_t  videomem_lseek   (struct file * file, loff_t offset, int orig) {return 0;}
//static ssize_t videomem_read    (struct file * file, char * buf, size_t count, loff_t *off) {return 0;}
static ssize_t videomem_write   (struct file * file, const char * buf, size_t count, loff_t *off) {return 0;}

/**
 * @brief Process raw buffer file opening.
 * @param[in]   inode
 * @param[in]   filp
 * @return      Always 0.
 */
static int videomem_open(struct inode *inode, struct file *filp)
{
	struct raw_priv_t *privData;

	int test_channel=0;

	int p_color, p_pf_height;
	int width_marg, height_marg, width_bursts;
	int frame_number;

	x393_status_ctrl_t s1, s2;

	pr_debug("VIDEOMEM_OPEN \n");

	// Problem: pass channel number to the driver,
	// solution 1: set channel number through sysfs
	// solution 2: create N links for a char dev:
	//    /dev/raw0 -> /dev/image_raw
	//    /dev/raw1 -> /dev/image_raw
	// there should be a way (there is no way) to read link name then extract channel number from it

	pr_debug("DEV OPENED: %s\n",filp->f_path.dentry->d_iname);

	privData = (struct raw_priv_t*)kmalloc(sizeof(struct raw_priv_t), GFP_KERNEL);
	if (!privData) {
		return -ENOMEM;
	}
	filp->private_data = privData;

	privData->minor = MINOR(inode->i_rdev);
	privData->buf_ptr = pElphel_buf->raw_chn0_vaddr;
	privData->phys_addr = pElphel_buf->raw_chn0_paddr;
	privData->buf_size = pElphel_buf->raw_chn0_size*PAGE_SIZE;
	privData->buf_size32 = (privData->buf_size)>>2;

	test_channel = 2;
	membridge_sensor_port = test_channel;

	frame_number = GLOBALPARS(membridge_sensor_port,G_THIS_FRAME) & PARS_FRAMES_MASK;

	//calculate things

    //if (frame16 >= PARS_FRAMES) return -1; // wrong frame

    width_marg =  get_imageParamsThis(membridge_sensor_port,P_ACTUAL_WIDTH);
    height_marg = get_imageParamsThis(membridge_sensor_port,P_ACTUAL_HEIGHT);

    p_color = get_imageParamsThis(membridge_sensor_port,P_COLOR);
    p_pf_height = get_imageParamsThis(membridge_sensor_port,P_PF_HEIGHT);

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

    //int setup_sensor_memory (int num_sensor,       ///< sensor port number (0..3)
    //                         int window_width,     ///< 13-bit - in 8*16=128 bit bursts
    //                         int window_height,    ///< 16-bit window height (in scan lines)
    //                         int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
    //                         int window_top,       ///< 16-bit window top margin (in scan lines)
    //                         x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
    //                         int frame16)          ///< Frame number the command should be applied to (if not immediate mode)

    //setup_sensor_memory (sensor_port,       // sensor port number (0..3)
    //                     width_bursts,      // 13-bit - in 8*16=128 bit bursts
    //                     height_marg,       // 16-bit window height (in scan lines)
    //                     0,                 // 13-bit window left margin in 8-bursts (16 bytes)
    //                     0,                 // 16-bit window top margin (in scan lines)
    //                     (frame16<0)? ASAP: ABSOLUTE,  // how to apply commands - directly or through channel sequencer
    //                     frame16);          // Frame number the command should be applied to (if not immediate mode)

    // setup membridge fpga memory

    //int setup_membridge_memory(
    //		int num_sensor,       ///< sensor port number (0..3)
    //		int write_direction,  ///< 0 - from fpga mem to system mem, 1 - otherwise
    //      int window_width,     ///< 13-bit - in 8*16=128 bit bursts
    //      int window_height,    ///< 16-bit window height (in scan lines)
    //      int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
    //      int window_top,       ///< 16-bit window top margin (in scan lines)
    //      int start_x,          ///< START X ...
    //		int start_y,          ///< START Y ...
    //      x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
    //      int frame16)          ///< Frame number the command should be applied to (if not immediate mode)

    setup_membridge_memory(membridge_sensor_port, ///< sensor port number (0..3)
    					   0,                     ///< 0 - from fpga mem to system mem, 1 - otherwise
						   width_bursts,          ///< 13-bit - in 8*16=128 bit bursts
						   height_marg,           ///< 16-bit window height (in scan lines)
						   0,                     ///< 13-bit window left margin in 8-bursts (16 bytes)
						   0,                     ///< 16-bit window top margin (in scan lines)
						   0,                     ///< START X ...
						   0,                     ///< START Y ...
						   DIRECT,                ///< how to apply commands - directly or through channel sequencer
						   frame_number);         ///< Frame number the command should be applied to (if not immediate mode)

	// setup membridge system memory - everything is in QW
	setup_membridge_system_memory(
			(privData->phys_addr)>>3,
			(privData->buf_size)>>3,
			0, // start offset?
			width_bursts*height_marg,
			width_bursts
			);

	s1 = get_x393_membridge_scanline_status_cntrl();
	s2 = get_x393_membridge_status_cntrl();

	pr_debug("MEMBRIDGE_STATUS \"0\": scanline_status:0x%08x status:0x%08x\n",s1,s2);

	control_membridge_memory(membridge_sensor_port,1,DIRECT,frame_number);

	//NOTES:

	//// get this frame number
    //#define  thisFrameNumber(p)            GLOBALPARS(p,G_THIS_FRAME)                       // Current frame number (may lag from the hardware)
    //#define  thisCompressorFrameNumber(p)  GLOBALPARS(p,G_COMPRESSOR_FRAME)                 // Current compressed frame number (lags from thisFrameNumber)

	//// wait for certain frame number
	// wait_event_interruptible(aframepars_wait_queue[sensor_port], getThisFrameNumber(sensor_port) >= target_frame);

	//// get frame sizes
	//unsigned long get_imageParamsPast(int sensor_port,  int n,  int frame);

	// FROM:

	// from dts: frame_start_chn2 =     <0x10000000>;      // Channel 2 frame start (in bytes)
	//x393_membridge_scanline_startaddr(buffer_settings.frame_start[test_channel]);
	// what units? Bytes?
	////x393_membridge_scanline_frame_size(privData->buf_size);
	// number of frames in buffer - 1
	//x393_membridge_scanline_frame_last(buffer_settings.frames_in_buffer[test_channel]);
	// Set frame full(padded) width - get from some parameter
	////x393_membridge_scanline_frame_full_width(2592);
	// set frame window size?!
	// (window_height << 16)
	////x393_membridge_scanline_window_wh(1940<<16);
	// Set startXY register
	// (window_top << 16)
	////x393_membridge_scanline_window_x0y0(0<<16);

	////x393_membridge_scanline_startxy(0);

	// TO:

	// start address of the system memory range in QWORDs (4 LSBs==0)
	///x393_membridge_lo_addr64((privData->phys_addr)>>4);
	// size of the system memory range in QWORDs (4 LSBs==0), rolls over
	///x393_membridge_size64((privData->buf_size)>>4);
	// start of transfer offset to system memory range in QWORDs (4 LSBs==0)
	///x393_membridge_start64(0);
	// Full length of transfer in QWORDs
	///x393_membridge_len64((8192*PAGE_SIZE)>>4);
	// Frame width in QWORDs (last xfer in each line may be partial)
	///x393_membridge_width64(2592>>4);

	// start?
	////x393_membridge_ctrl();

	// wait until transfer is done
	////get_x393_membridge_status_cntrl();

	return 0;
}

/**
 * @brief Process raw buffer file closing
 * @param[in]   inode
 * @param[in]   filp
 * @return      0
 */
static int videomem_release(struct inode *inode, struct file *filp)
{
	pr_debug("VIDEOMEM_RELEASE\n");

	if (filp->private_data) {
		kfree(filp->private_data);
	}

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
	x393_status_ctrl_t s1, s2;
	s1 = get_x393_membridge_scanline_status_cntrl();
	s2 = get_x393_membridge_status_cntrl();
	pr_debug("MEMBRIDGE_STATUSES: scanline_status:0x%08x status:0x%08x\n",s1,s2);

	return vm_read(file, buf, count, off);
}

static struct file_operations videomem_fops = {
        owner:    THIS_MODULE,
        open:     videomem_open,
        release:  videomem_release,
        llseek:   videomem_lseek,
        read:     videomem_read,
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
static ssize_t get_num_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n", membridge_sensor_port);
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

static ssize_t set_num_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &membridge_sensor_port);
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

// selected sensor channel == port
static DEVICE_ATTR(channel,                    SYSFS_PERMISSIONS,     get_num_sensor,           set_num_sensor);

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
		&dev_attr_channel.attr,
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
