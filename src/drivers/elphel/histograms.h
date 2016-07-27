//histograms.h

/// These wait queues will be advanced each frame after the histogram data is transferred to the FPGA.
/// It will happen even if the corresponding task is disabled, with the only exception:
/// hist_c_wait_queue will not be awaken in the current frame if it is too late (frame counter incremented while serving tasklet)
extern wait_queue_head_t hist_y_wait_queue; /// wait queue for the G1 histogram (used as Y)
extern wait_queue_head_t hist_c_wait_queue; /// wait queue for all the other (R,G2,B) histograms (color)

void init_histograms(int chn_mask);

int get_hist_index (int sensor_port, int sensor_chn);

int set_histograms (int sensor_port, int sensor_chn, unsigned long frame, int needed, unsigned long * gammaHash, unsigned long * framep);

/**
 * @brief Get histograms from the FPGA (called as tasklet?) and/or calculate derivatives (if needed)
 * @param frame absolute frame number (Caller should match it to the hardware frame) frame==0xffffffff - find the latest available frame, don't wait
 * TODO: should it be one frame behind current?
 * @param needed bits specify what histograms (color, type) are requested
 * TODO: Add P_* parameter - what to read from tasklet,  turn colors it off for high FPS/small window
 * each group of 4 bits covers 4 colors of the same type:
 * - bits 0..3 - read raw histograms from the FPGA - normally called from IRQ/tasklet (use just 1 color for autoexposure to speed up?)
 * - bits 4..7 - calculate cumulative histograms (sum of raw ones) - normally called from applications
 * - bits 8..11 - calculate percentiles (reverse cumulative histograms) - normally called from applications
 * "needed" for raw histograms should be specified explicitly (can not be read from FPGA later),
 * "needed" for cumul_hist will be added automatically if percentiles are requested
 * @return index of the histogram (>=0) if OK, otherwise:
 * - -1 not reading FPGA and frame number stored is different from the requested (too late - histogram buffer overrun?)
 * - -2 - not reading from FPGA, but some needed raw histograms are missing (should be requested/loaded earlier)
 */
int get_histograms (unsigned long frame, int needed);

int histograms_init_hardware(void);
void histograms_dma_ctrl(int mode); // 0 - reset, 1 - disable, 2 - enable
