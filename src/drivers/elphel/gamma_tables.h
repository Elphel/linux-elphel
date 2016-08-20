//gamma_tables.h
#ifndef _GAMMA_TABLES_H
#define _GAMMA_TABLES_H

void init_gammas(void);
int is_gamma_current (unsigned short hash16, unsigned short scale, int index);
int is_gamma_valid  (unsigned short hash16, unsigned short scale, int index);
//             int prev_locked_color[4];

int unlock_gamma_node (int color, int sensor_port, int sensor_subchn); /// NOTE: Not needed anymore
///
/// return a pointer to the gamma table (single color) encoded in FPGA format (NULL if there is to table ready)
///
unsigned long * get_gamma_fpga(int color, int sensor_port, int sensor_subchn);
int fpga_gamma_write_nice    (int color, int sensor_port, int sensor_subchn, unsigned long * gamma);


int gamma_new_node(void);
void gamma_encode_fpga(unsigned short * gamma_in, unsigned long * gamma_out);///Hardware-dependent encoding of the FPGA "gamma" table. Converts unsigned short array of 257 16-bit values (only 10 msb-s are used) to 256 unsigned long words to be written to FPGA
void  gamma_calc_scaled (unsigned short scale,unsigned short * gamma_in, unsigned short * gamma_out);/// scale gamma table by (scale>>GAMMA_SCALE_SHIFT), saturate to 0..0xffff
//void  gamma_calc_reverse(unsigned short * gamma_in, unsigned short * gamma_out);/// calculate reverse gamma table (16-bit output) that matches 1-byte gamma-converted data to the input data (in the 0..ffff range)
void  gamma_calc_reverse(unsigned short * gamma_in, unsigned char * gamma_out);/// calculate reverse gamma table (8-bit output) that matches 1-byte gamma-converted data to the input data (in the 0..ffff range)
// return index of the specified hash/scale, insert new table (gamma_proto) if needed
// If no table is specified (null) - return 0 if no prototype is found
// if (not_nice) - don't re-enable interrupts between atomic actions (may fail)
// if "hardware" is non-zero, color/frame pair will be used to lock node to it, fpga-encoded table will be calculated (if not done so earlier)
// #define GAMMA_MODE_NOT_NICE     1  // if set, no interrupts will be enabled between steps, whole operation is atomic
// #define GAMMA_MODE_NEED_REVERSE 2  // reverse gamma table is needed
// #define GAMMA_MODE_HARDWARE     4  // the table is needed to program FPGA: fpga-encoded table will be calculated (if not yet), node will be locked for specified
                                   // color/frame pair
int set_gamma_table (unsigned short hash16, unsigned short scale, unsigned short * gamma_proto,  unsigned char mode, int color, int sensor_port, int sensor_subchn);
unsigned long get_locked_hash32(int color, int sensor_port,int sensor_subchn);

#endif
