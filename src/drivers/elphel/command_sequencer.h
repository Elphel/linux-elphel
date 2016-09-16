/***************************************************************************//**
* @file  command_sequencer.h
* @brief  Interface to FPGA-based command sequencer sequencer
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
/* Write command to the 16-frame sequencer, relative to the current frame */
int write_seq_rel (int chn,   int frame, u32 addr,  u32 data);
/* Write command to the 16-frame sequencer, absolute frame */
int write_seq_abs (int chn, int frame, u32 addr,  u32 * data);
 
