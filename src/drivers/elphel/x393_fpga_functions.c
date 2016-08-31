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
#include <linux/delay.h>

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

int compressor_dma_setup (int        port_afi,      ///< number of AFI port (0 - afi 1, 1 - afi2) (currently only 0 is routed)
                          int        chn_mask,      ///< compressor channels to use, bitmask
                          int        reset,         ///< 1 - reset all channels
                          int        status_mode,   ///< status update mode status mode (3 for auto)
                          int        report_mode,   ///< readback mode:
                                                    ///< * 0 - show EOF pointer, internal
                                                    ///< * 1 - show EOF pointer, confirmed written to the system memory
                                                    ///< * 2 - show show current pointer, internal (debug mode)
                                                    ///< * 3 - show current pointer, confirmed written to the system memory (debug mode)
                          dma_addr_t cmprs0_sa,     ///< input channel 0 start address, 32-bytes aligned
                          u32        cmprs0_len,    ///< input channel 0 buffer length, 32-byte aligned
                          dma_addr_t cmprs1_sa,     ///< input channel 1 start address, 32-bytes aligned
                          u32        cmprs1_len,    ///< input channel 1 buffer length, 32-byte aligned
                          dma_addr_t cmprs2_sa,     ///< input channel 2 start address, 32-bytes aligned
                          u32        cmprs2_len,    ///< input channel 2 buffer length, 32-byte aligned
                          dma_addr_t cmprs3_sa,     ///< input channel 3 start address, 32-bytes aligned
                          u32        cmprs3_len)    ///< input channel 3 buffer length, 32-byte aligned
{
    x393_status_ctrl_t   status_ctrl = {.d32=0};
    x393_afimux_report_t afimux_report= {.d32=0};
    x393_afimux_sa_t     afimux_sa[4]={{.d32=0},{.d32=0},{.d32=0},{.d32=0}};
    x393_afimux_len_t    afimux_len[4]={{.d32=0},{.d32=0},{.d32=0},{.d32=0}};
    x393_afimux_rst_t    afimux_rst={.d32=0};
    x393_afimux_en_t     afimux_en = {.d32=0};
    int chn;
    if ((cmprs0_sa | cmprs0_len | cmprs1_sa | cmprs1_len | cmprs2_sa | cmprs2_len | cmprs3_sa | cmprs3_len) & 0x1f){
        return -EINVAL;
    }
    status_ctrl.mode = status_mode;
    afimux_report.mode0 = report_mode;
    afimux_report.mode0_set = 1 & (chn_mask >> 0);
    afimux_report.mode1 = report_mode;
    afimux_report.mode1_set = 1 & (chn_mask >> 1);
    afimux_report.mode2 = report_mode;
    afimux_report.mode2_set = 1 & (chn_mask >> 2);
    afimux_report.mode3 = report_mode;
    afimux_report.mode3_set = 1 & (chn_mask >> 3);

    afimux_sa[0].sa256 = (u32) cmprs0_sa >> 5;
    afimux_sa[1].sa256 = (u32) cmprs1_sa >> 5;
    afimux_sa[2].sa256 = (u32) cmprs2_sa >> 5;
    afimux_sa[3].sa256 = (u32) cmprs3_sa >> 5;

    afimux_len[0].len256 = cmprs0_len >> 5;
    afimux_len[1].len256 = cmprs1_len >> 5;
    afimux_len[2].len256 = cmprs2_len >> 5;
    afimux_len[3].len256 = cmprs3_len >> 5;

    afimux_en.en =      1; // global enable
    afimux_en.en_set =  1; // set global enable
    afimux_en.en0 =     1;
    afimux_en.en0_set = 1 & (chn_mask >> 0);
    afimux_en.en1 =     1;
    afimux_en.en1_set = 1 & (chn_mask >> 1);
    afimux_en.en2 =     1;
    afimux_en.en2_set = 1 & (chn_mask >> 2);
    afimux_en.en3 =     1;
    afimux_en.en3_set = 1 & (chn_mask >> 3);

    for (chn = 0; chn < 4; chn++) if (chn_mask & (1 << chn)){
        if (!port_afi) set_x393_afimux0_status_control (status_ctrl, chn);  // AFI MUX 0 status report mode
        else           set_x393_afimux1_status_control (status_ctrl, chn);  // AFI MUX 1 status report mode
    }
    if (!port_afi) x393_afimux0_report_mode (afimux_report);                // AFI MUX 0 readout pointer report mode
    else           x393_afimux1_report_mode (afimux_report);                // AFI MUX 1 readout pointer report mode
    if (reset) {
        afimux_rst.rst0 =  1 & (chn_mask >> 0);
        afimux_rst.rst1 =  1 & (chn_mask >> 1);
        afimux_rst.rst2 =  1 & (chn_mask >> 2);
        afimux_rst.rst0 =  1 & (chn_mask >> 3);
        if (!port_afi) set_x393_afimux0_rst(afimux_rst);
        else           set_x393_afimux1_rst(afimux_rst);
        udelay(1);
        afimux_rst.rst0 =  0;
        afimux_rst.rst1 =  0;
        afimux_rst.rst2 =  0;
        afimux_rst.rst0 =  0;
        if (!port_afi) set_x393_afimux0_rst(afimux_rst);
        else           set_x393_afimux1_rst(afimux_rst);
        udelay(1);
    }
    for (chn = 0; chn < 4; chn++) if (chn_mask & (1 << chn)){
        if (!port_afi) {
            set_x393_afimux0_sa  (afimux_sa[chn], chn);    // AFI MUX 0 DMA buffer start address in 32-byte blocks
            set_x393_afimux0_len (afimux_len[chn], chn);   // AFI MUX 0 DMA buffer length in 32-byte blocks
        }  else {
            set_x393_afimux1_sa (afimux_sa[chn], chn);     // AFI MUX 1 DMA buffer start address in 32-byte blocks
            set_x393_afimux1_len (afimux_len[chn], chn);   // AFI MUX 0 DMA buffer length in 32-byte blocks
        }
    }
    if (!port_afi) x393_afimux0_en (afimux_en);   // AFI MUX 0 global/port run/pause control
    else           x393_afimux1_en (afimux_en);   // AFI MUX 1 global/port run/pause control

    return 0;
}

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
    if (!is_fpga_programmed()) {
        ts->sec = 0;
        ts->usec = 0;
        return ts;
    }
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
int set_fpga_rtc (sec_usec_t ts) ///< timestamp providing seconds and microseconds
{
    x393_rtc_usec_t    usec;
    x393_rtc_sec_t     sec;
    usec.usec = ts.usec;
    sec.sec =   ts.sec;
    if (!is_fpga_programmed())
        return -ENODEV;
    spin_lock_bh(&fpga_time_lock);
    set_x393_rtc_usec(usec);
    set_x393_rtc_sec_set(sec);  // And apply
    spin_unlock_bh(&fpga_time_lock);
    return 0;
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
