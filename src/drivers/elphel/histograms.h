/***************************************************************************//**
* @file      histograms.h
* @brief     Handles histograms storage, access and percentile calculation
* @copyright Copyright 2008-2016 (C) Elphel, Inc.
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
#ifndef HISTOGRAMS_H
#define HISTOGRAMS_H
#undef ISR_HISTOGRAMS // to histograms-related disable code in ISR - not needed in NC393

// These wait queues will be advanced each frame after the histogram data is transferred to the FPGA.
// It will happen even if the corresponding task is disabled, with the only exception:
// hist_c_wait_queue will not be awaken in the current frame if it is too late (frame counter incremented while serving tasklet)
extern wait_queue_head_t hist_y_wait_queue; /// wait queue for the G1 histogram (used as Y)
extern wait_queue_head_t hist_c_wait_queue; /// wait queue for all the other (R,G2,B) histograms (color)

// void init_histograms(int chn_mask);
int histograms_check_init(void);
int get_hist_index (int sensor_port, int sensor_chn); //no hardware involved
int set_histograms (int sensor_port, int sensor_chn, unsigned long frame, int needed, unsigned long * gammaHash, unsigned long * framep);
int get_histograms (int sensor_port, int sensor_chn, unsigned long frame, int needed);

//int histograms_init_hardware(void);
void histograms_dma_ctrl(int mode); // 0 - reset, 1 - disable, 2 - enable
#endif
