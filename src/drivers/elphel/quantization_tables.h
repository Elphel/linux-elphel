/// @file quantization_tables.h
#ifndef _QUANTIZATION_TABLES_H
#define _QUANTIZATION_TABLES_H

/** @brief Quantization tables cache usage policy */
enum {
	// @brief Use common cache for all compressors
	COMMON_CACHE = 0,
	// @brief Use separate cache for each compressor
	PER_CHN_CACHE
};

/**
 * @brief initialization of quantization tables (direct - JPEG header ones) cache
 * \n TODO: add \b init_qtable_head_cache to module initialization
 *
 * used in quantization_tables.c only
 */
void init_qtable_head_cache(unsigned int chn);
/**
 * @brief calculates a pair of direct (JPEG header) tables for the specified quality (2-bytes )
 * @param quality2 single byte (standard) or a pair of bytes (see file header description)
 * @param y_tab caller-provided pointer to a 64-byte Y (intensity) quantization table (NULL - don't copy)
 * @param c_tab caller-provided pointer to a 64-byte C (color) quantization table (NULL - don't copy)
 * @return 0 - cache hit, 1 - cache miss (recalculated), -1 - invalid quality
 *
 * used in quantization_tables.c and jpeghead.c
 */
int get_qtable(int quality2, unsigned char *y_tab, unsigned char *c_tab, unsigned int chn);
/**
 * @brief initialization of quantization tables (reverse, for the FPGA) cache
 * \n TODO: add \b init_qtable_fpga to module initialization
 *
 * used in quantization_tables.c only
 */
void init_qtable_fpga(unsigned int chn);
/**
 * @brief Finds an already programmed FPGA page or calculates (an programms FPGA with) a new one
 * @param quality2 single byte (standard) or a pair of bytes (see file header description)
 * @return table page number used (0..7) or -1 - invalid q
 *
 * used in quantization_table.c and pgm_functions.c
 */
int set_qtable_fpga(int quality2, unsigned int chn);

/**
 * @brief Temporary function to directly set one of the coring LUTs (currently 100 0.0 to 9.9 with 0.1 step)
 *        to one of 16 FPGA tables (even - for Y,odd - for C)
 * @param coring_number 0..99 - function number
 * @param fpga_number 0.. 15 - fpga table number
 * @return table page number used (0..7) or -1 - invalid q
 *
 * used in quantization_table.c and pgm_functions.c
 */
void set_coring_fpga(unsigned int coring_number, int fpga_number, unsigned int chn);

void reset_qtables(unsigned int chn);

int get_cache_policy(void);
void set_cache_policy(int policy);
void qt_init(struct platform_device *pdev);
#endif
