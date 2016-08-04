/***************************************************************************//**
* @file      x393_fpga_functions.c
* @brief     Reimplementation of Python methods to program FPGA parameters
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
#include "x393.h"
#include "x393_fpga_functions.h"
 /** Setup memory controller for a sensor channel */
int  setup_sensor_memory (int num_sensor,       ///< sensor port number (0..3)
                          int frame_sa,         ///< 22-bit frame start address ((3 CA LSBs==0. BA==0)
                          int frame_sa_inc,     ///< 22-bit frame start address increment  ((3 CA LSBs==0. BA==0)
                          int last_frame_num,   ///< 16-bit number of the last frame in a buffer
                          int frame_full_width, ///< 13-bit Padded line length (8-row increment), in 8-bursts (16 bytes)
                          int window_width,     ///< 13-bit - in 8*16=128 bit bursts
                          int window_height,    ///< 16-bit window height (in scan lines)
                          int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
                          int window_top,       ///< 16-bit window top margin (in scan lines
                          x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                          int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                                ///< @return 0 -OK
//typedef enum {DIRECT,ABSOLUTE,RELATIVE} x393cmd_t;
{
    x393_mcntrl_mode_scan_t mcntrl_mode = {.enable =        1, // [    0] (1) enable requests from this channel ( 0 will let current to finish, but not raise want/need)
                                           .chn_nreset =    0, // [    1] (1) 0: immediately reset all the internal circuitry
                                           .write_mem =     1, // [    2] (0) 0 - read from memory, 1 - write to memory
                                           .extra_pages =   0, // [ 4: 3] (0) 2-bit number of extra pages that need to stay (not to be overwritten) in the buffer
                                           .keep_open =     0, // [    5] (0) (NA in linescan) for 8 or less rows - do not close page between accesses (not used in scanline mode)
                                           .byte32 =        0, // [    6] (1) (NA in linescan) 32-byte columns (0 - 16-byte), not used in scanline mode
                                           .reset_frame =   0, // [    8] (0) reset frame number
                                           .single =        0, // [    9] (0) run single frame
                                           .repetitive =    1, // [   10] (1) run repetitive frames
                                           .disable_need =  0, // [   11] (0) disable 'need' generation, only 'want' (compressor channels)
                                           .skip_too_late = 1}; // [   12] (0) Skip over missed blocks to preserve frame structure (increment pointers)

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

    switch (x393cmd){
    case RELATIVE:
        seqr_x393_sens_mcntrl_scanline_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
        seqr_x393_sens_mcntrl_scanline_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        seqr_x393_sens_mcntrl_scanline_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        seqr_x393_sens_mcntrl_scanline_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
        seqr_x393_sens_mcntrl_scanline_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
        seqr_x393_sens_mcntrl_scanline_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
        seqr_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    case ABSOLUTE:
        seqa_x393_sens_mcntrl_scanline_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
        seqa_x393_sens_mcntrl_scanline_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        seqa_x393_sens_mcntrl_scanline_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        seqa_x393_sens_mcntrl_scanline_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
        seqa_x393_sens_mcntrl_scanline_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
        seqa_x393_sens_mcntrl_scanline_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
        seqa_x393_sens_mcntrl_scanline_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    case DIRECT:
        x393_sens_mcntrl_scanline_startaddr             (window_frame_sa,         num_sensor); // Set frame start address
        x393_sens_mcntrl_scanline_frame_size            (window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        x393_sens_mcntrl_scanline_frame_last            (window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        x393_sens_mcntrl_scanline_frame_full_width      (window_full_width,       num_sensor); // Set frame full(padded) width
        x393_sens_mcntrl_scanline_window_wh             (window_width_height,     num_sensor); // Set frame window size
        x393_sens_mcntrl_scanline_window_x0y0           (window_left_top,         num_sensor); // Set frame position
        x393_sens_mcntrl_scanline_mode                  (mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    }

    return 0;
}

/** Setup memory controller for a compressor channel */
int setup_compressor_memory (int num_sensor,       ///< sensor port number (0..3)
                             int frame_sa,         ///< 22-bit frame start address ((3 CA LSBs==0. BA==0)
                             int frame_sa_inc,     ///< 22-bit frame start address increment  ((3 CA LSBs==0. BA==0)
                             int last_frame_num,   ///< 16-bit number of the last frame in a buffer
                             int frame_full_width, ///< 13-bit Padded line length (8-row increment), in 8-bursts (16 bytes)
                             int window_width,     ///< 13-bit - in 8*16=128 bit bursts
                             int window_height,    ///< 16-bit window height (in scan lines)
                             int window_left,      ///< 13-bit window left margin in 8-bursts (16 bytes)
                             int window_top,       ///< 16-bit window top margin (in scan lines
                             int byte32,           ///< 1 - 32-byte columns (currently used), 0 - 16 byte columns
                             int tile_width,       ///< tile width in pixels
                             int tile_vstep,       ///< tile vertical step in pixel rows (JPEG18/jp4 = 16) // = 16
                             int tile_height,      ///< tile height: 18 for color JPEG, 16 for JP4 flavors // = 18
                             int extra_pages,      ///< extra pages needed (1) - number of previous pages to keep in a 4-page buffer
                             int disable_need,     ///< disable "need" (yield to sensor channels - they can not wait)
                             x393cmd_t x393cmd,    ///< how to apply commands - directly or through channel sequencer
                             int frame16)          ///< Frame number the command should be applied to (if not immediate mode)
                                                   ///< @return 0 - OK
{
    x393_mcntrl_mode_scan_t mcntrl_mode = {.enable =        1, // [    0] (1) enable requests from this channel ( 0 will let current to finish, but not raise want/need)
                                           .chn_nreset =    0, // [    1] (1) 0: immediately reset all the internal circuitry
                                           .write_mem =     0, // [    2] (0) 0 - read from memory, 1 - write to memory
                                           .extra_pages =   1, // [ 4: 3] (0) 2-bit number of extra pages that need to stay (not to be overwritten) in the buffer
                                           .keep_open =     0, // [    5] (0) (NA in linescan) for 8 or less rows - do not close page between accesses (not used in scanline mode)
                                           .byte32 =        1, // [    6] (1) (NA in linescan) 32-byte columns (0 - 16-byte), not used in scanline mode
                                           .reset_frame =   0, // [    8] (0) reset frame number
                                           .single =        0, // [    9] (0) run single frame
                                           .repetitive =    1, // [   10] (1) run repetitive frames
                                           .disable_need =  1, // [   11] (0) disable 'need' generation, only 'want' (compressor channels)
                                           .skip_too_late = 1};// [   12] (0) Skip over missed blocks to preserve frame structure (increment pointers)
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

    mcntrl_mode.disable_need = disable_need; // non-constant parameter
    mcntrl_mode.extra_pages =  extra_pages;  // non-constant parameter

    switch (x393cmd){
    case RELATIVE:
        seqr_x393_sens_mcntrl_tiled_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
        seqr_x393_sens_mcntrl_tiled_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        seqr_x393_sens_mcntrl_tiled_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        seqr_x393_sens_mcntrl_tiled_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
        seqr_x393_sens_mcntrl_tiled_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
        seqr_x393_sens_mcntrl_tiled_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
        seqr_x393_sens_mcntrl_tiled_tile_whs         (frame16, window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
        seqr_x393_sens_mcntrl_tiled_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    case ABSOLUTE:
        seqa_x393_sens_mcntrl_tiled_startaddr        (frame16, window_frame_sa,         num_sensor); // Set frame start address
        seqa_x393_sens_mcntrl_tiled_frame_size       (frame16, window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        seqa_x393_sens_mcntrl_tiled_frame_last       (frame16, window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        seqa_x393_sens_mcntrl_tiled_frame_full_width (frame16, window_full_width,       num_sensor); // Set frame full(padded) width
        seqa_x393_sens_mcntrl_tiled_window_wh        (frame16, window_width_height,     num_sensor); // Set frame window size
        seqa_x393_sens_mcntrl_tiled_window_x0y0      (frame16, window_left_top,         num_sensor); // Set frame position
        seqa_x393_sens_mcntrl_tiled_tile_whs         (frame16, window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
        seqa_x393_sens_mcntrl_tiled_mode             (frame16, mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    case DIRECT:
        x393_sens_mcntrl_tiled_startaddr             (window_frame_sa,         num_sensor); // Set frame start address
        x393_sens_mcntrl_tiled_frame_size            (window_frame_sa_inc,     num_sensor); // Set frame size (address increment)
        x393_sens_mcntrl_tiled_frame_last            (window_last_frame_num,   num_sensor); // Set last frame number (number of frames in buffer minus 1)
        x393_sens_mcntrl_tiled_frame_full_width      (window_full_width,       num_sensor); // Set frame full(padded) width
        x393_sens_mcntrl_tiled_window_wh             (window_width_height,     num_sensor); // Set frame window size
        x393_sens_mcntrl_tiled_window_x0y0           (window_left_top,         num_sensor); // Set frame position
        x393_sens_mcntrl_tiled_tile_whs              (window_tile_whs,         num_sensor); // Set tile size/step (tiled mode only)
        x393_sens_mcntrl_tiled_mode                  (mcntrl_mode,             num_sensor); // Set mode register (write last after other channel registers are set)
        break;
    }

    return 0;
}


/*

 */

