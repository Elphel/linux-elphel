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


#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/errno.h>

#include <uapi/elphel/c313a.h> // PARS_FRAMES_MASK

#include "x393.h"
#include "x393_fpga_functions.h"

#define REPEAT_STATUS_READ         10 ///< Number of times status is re-read waiting for a new timestamp

static void __iomem* zynq_devcfg_ptr = NULL; // 0xF8007000..f8007fff /f800700c needed
static DEFINE_SPINLOCK(fpga_time_lock);

// Nothing to init? Started in pgm_detectsesnor through sensor_common:sequencer_stop_run_reset
#if 0
int init_command_sequencer(int sensor_port)
{
    return 0;
}
#endif
/** Read time (seconds and microseconds) from the FPGA RTC */
sec_usec_t * get_fpga_rtc(sec_usec_t * ts) ///< Pointer to a sec/usec structure to fill in
                                           ///< @return structure link to a passed structure
{
//    sec_usec_t  ts = {.sec=0, .usec=0};
    x393_rtc_status_t  stat;
    x393_status_ctrl_t stat_ctrl={.d32=0};
//    x393_rtc_usec_t    usec;
//    x393_rtc_sec_t     sec;
    int i;
    if (!ts) return NULL;
    spin_lock_bh(&fpga_time_lock);
    stat = x393_rtc_status();
    stat_ctrl.mode = 1;
    stat_ctrl.seq_num = stat.seq_num + 1;
    set_x393_rtc_set_status(stat_ctrl);  // Requests single status, latches current time
    for (i = 0; i < REPEAT_STATUS_READ; i++) {
        stat = x393_rtc_status();
        if (stat.seq_num == stat_ctrl.seq_num) {
            ts->sec = x393_rtc_status_sec().sec;
            ts->usec = x393_rtc_status_usec().usec;
            break;
        }
    }
    spin_unlock_bh(&fpga_time_lock);
    return ts;
}

/** Set FPGA RTC to specified time */
void set_fpga_rtc (sec_usec_t ts) ///< timestamp providing seconds and microseconds
{
    x393_rtc_usec_t    usec;
    x393_rtc_sec_t     sec;
    usec.usec = ts.usec;
    sec.sec =   ts.sec;
    spin_lock_bh(&fpga_time_lock);
    set_x393_rtc_usec(usec);
    set_x393_rtc_sec_set(sec);  // And apply
    spin_unlock_bh(&fpga_time_lock);
}
/** Check if bitstream is loaded */
int is_fpga_programmed(void) ///< @return 0 - bitstream is NOT loaded, 1 - bitsteam IS loaded, -ENOMEM - error in ioremap
{
    if (!zynq_devcfg_ptr){
        zynq_devcfg_ptr = ioremap(0xf8007000, 0x00010000);
        if (!zynq_devcfg_ptr)
            return -ENOMEM;
    }
    return (readl(zynq_devcfg_ptr + 0x000c) & 4)? 1: 0;
}
