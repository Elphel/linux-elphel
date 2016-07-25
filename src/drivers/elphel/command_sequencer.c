/***************************************************************************//**
* @file  command_sequencer.c
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
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <elphel/c313a.h> // PARS_FRAMES_MASK
#include "x393.h"
static DEFINE_SPINLOCK(lock);
 
/** Write command to the 16-frame sequencer, relative to the current frame */
int write_seq_rel (int chn,     ///< sensor port
                   int frame,  ///< relative frame number modulo 16 (0 - ASAP)
                   u32 addr,   ///< command (register) address in bytes (2 LSBs are will be discarded)
                   u32 data)   ///< command data.
                                ///< @return 0 on success, negative error if table shadow is not initialized to data
{
	if (unlikely(frame < 0)) frame = 0;
	else if (unlikely(frame >= PARS_FRAMES_MASK)){
		return -EINVAL; // too far in the future
	}
	spin_lock(&lock);
	x393_cmdframeseq_rel(addr>>2, chn, frame);
	x393_cmdframeseq_rel(data,    chn, frame);
	spin_unlock(&lock);
	return 0;
}
EXPORT_SYMBOL_GPL(write_seq_rel);

/* Write command to the 16-frame sequencer, absolute frame */
int write_seq_abs (int chn,     ///< sensor port
                   int frame,  ///< absolute frame number modulo 16
                   u32 addr,   ///< command (register) address in bytes (2 LSBs are will be discarded)
                   u32 * data) ///< data array. Length written is defined by the pre-configured table entry.
                                ///< MSB (data is sent MSB first) of the first entry is index in the table.
                                ///< @return 0 on success, negative error if table shadow is not initialized to data
{
	frame &= PARS_FRAMES_MASK;
	spin_lock(&lock);
	x393_cmdframeseq_rel(addr>>2, chn, frame);
	x393_cmdframeseq_rel(data,    chn, frame);
	spin_unlock(&lock);
	return 0;
}
EXPORT_SYMBOL_GPL(write_seq_abs);
// TODO: Add other sequencer control here
/*
// Command sequencer control

// Controller is programmed through 32 locations. Each register but the control requires two writes:
// First write - register address (AXI_WR_ADDR_BITS bits), second - register data (32 bits)
// Writing to the contol register (0x1f) resets the first/second counter so the next write will be "first"
// 0x0..0xf write directly to the frame number [3:0] modulo 16, except if you write to the frame
//           "just missed" - in that case data will go to the current frame.
//  0x10 - write seq commands to be sent ASAP
//  0x11 - write seq commands to be sent after the next frame starts
//
//  0x1e - write seq commands to be sent after the next 14 frame start pulses
//  0x1f - control register:
//      [14] -   reset all FIFO (takes 32 clock pulses), also - stops seq until run command
//      [13:12] - 3 - run seq, 2 - stop seq , 1,0 - no change to run state
//        [1:0] - 0: NOP, 1: clear IRQ, 2 - Clear IE, 3: set IE
void                         x393_cmdframeseq_ctrl               (x393_cmdframeseq_mode_t d, int sens_chn){writel(d.d32, mmio_ptr + (0x1e7c + 0x80 * sens_chn));} // CMDFRAMESEQ control register
void                         x393_cmdframeseq_abs                (u32 d, int sens_chn, int offset){writel(d, mmio_ptr + (0x1e00 + 0x80 * sens_chn + 0x4 * offset));} // CMDFRAMESEQ absolute frame address/command
void                         x393_cmdframeseq_rel                (u32 d, int sens_chn, int offset){writel(d, mmio_ptr + (0x1e40 + 0x80 * sens_chn + 0x4 * offset));} // CMDFRAMESEQ relative frame address/command
// Command sequencer multiplexer, provides current frame number for each sensor channel and interrupt status/interrupt masks for them.
// Interrupts and interrupt masks are controlled through channel CMDFRAMESEQ module
void                         set_x393_cmdseqmux_status_ctrl      (x393_status_ctrl_t d){writel(d.d32, mmio_ptr + 0x1c08);}         // CMDSEQMUX status control mode (status provides current frame numbers)
x393_status_ctrl_t           get_x393_cmdseqmux_status_ctrl      (void)              { x393_status_ctrl_t d; d.d32 = readl(mmio_ptr + 0x1c08); return d; }
x393_cmdseqmux_status_t      x393_cmdseqmux_status               (void)              { x393_cmdseqmux_status_t d; d.d32 = readl(mmio_ptr + 0x20e0); return d; } // CMDSEQMUX status data (frame numbers and interrupts
 */
