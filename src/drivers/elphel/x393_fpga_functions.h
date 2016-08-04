/***************************************************************************//**
* @file      x393_fpga_functions.h
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
//typedef enum {DIRECT,ABSOLUTE,RELATIVE} x393cmd_t;
#include "x393.h"
int  setup_sensor_memory (int num_sensor, int frame_sa, int frame_sa_inc, int last_frame_num, int frame_full_width,
        int window_width, int window_height, int window_left, int window_top, x393cmd_t x393cmd, int frame16);
int setup_compressor_memory (int num_sensor, int frame_sa, int frame_sa_inc, int last_frame_num, int frame_full_width,
        int window_width, int window_height, int window_left, int window_top, int byte32, int tile_width, int tile_vstep,
        int tile_height, int extra_pages, int disable_need, x393cmd_t x393cmd, int frame16);
