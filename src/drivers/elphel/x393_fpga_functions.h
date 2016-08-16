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
//void        fpga_table_write_nice (int addr, int len, unsigned long * data);
sec_usec_t * get_fpga_rtc(sec_usec_t * ts);
void         set_fpga_rtc (sec_usec_t ts);

