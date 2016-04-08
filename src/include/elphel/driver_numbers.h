/// driver_numbers.h
/// see packages/devices/elphel/Makefile - major numbers should match

//#define CMOSCAM_MAJOR    126
#define X3X3_EXIF_MAJOR 125
#define ELPHEL_MAJOR     126
#define STREAM_MAJOR     127
#define FPGA_MAJOR       129
#define FPGA_JTAG_MAJOR  132
#define FPGA_CLOCK_MAJOR 133
#define X3X3_I2C_MAJOR   134
#define CIRCBUF_MAJOR    135
//#define FRAMEPARS_MAJOR  136
#define FRAMEPARS_MAJOR  130
#define GAMMAS_MAJOR     137
#define HISTOGRAMS_MAJOR 138
//#define IMAGERAW_MAJOR   139
#define IMAGERAW_MAJOR   131
#define IMAGEACQ_MAJOR   140
#define IMU_MAJOR        141


/// MINORS
#define IMU_MINOR               1
#define IMU_CTL_MINOR           2
#define IMAGERAW_MINOR_FRAME	1
#define IMAGERAW_MINOR_FPN	2
#define IMAGERAW_MINOR_UNLOCK	3

#define CIRCBUF_MINOR_CHN_OFFSET 30
#define CIRCBUF_MINOR_CHN_0      30
#define CIRCBUF_MINOR_CHN_1      31
#define CIRCBUF_MINOR_CHN_2      32
#define CIRCBUF_MINOR_CHN_3      33


#define CMOSCAM_MINOR_RWTABLES    9
#define CMOSCAM_MINOR_CIRCBUF    11
#define CMOSCAM_MINOR_HISTOGRAM  12
#define CMOSCAM_MINOR_JPEAGHEAD  13
#define CMOSCAM_MINOR_GAMMA      14
#define CMOSCAM_MINOR_FRAMEPARS  16
#define CMOSCAM_MINOR_GAMMAS     17
#define CMOSCAM_MINOR_HISTOGRAMS 18
#define CMOSCAM_MINOR_IMAGEACQ   19
#define CMOSCAM_MINOR_HUFFMAN    20

#define FPGACONF_MINOR_IORW	 3   /* direct R/W FPGA registers */
#define FPGACONF_MINOR_SDRAM     4   /* read/write SDRAM through PIO */
#define FPGACONF_MINOR_TABLES    6   /// Write FPGA tables directly
 

#define FPGA_CLOCK_MINOR    2
#define FPGA_CLOCK_MINOR_I2C        2
#define FPGA_CLOCK_MINOR_CLOCKS     3


#define FPGA_JTAG_RESET_MINOR   0  // just close open files
#define FPGA_JTAG_RAW_MINOR 0  // just close open files
#define FPGA_JTAG_MINOR     1
#define FPGA_SJTAG_MINOR    2
#define FPGA_AJTAG_MINOR    3
#define FPGA_JTAG_BOUNDARY_MINOR    5  // read/write boundary pins of the main FPGA
#define FPGA_SJTAG_BOUNDARY_MINOR   6  // read/write boundary pins of the sensor board FPGA
#define FPGA_AJTAG_BOUNDARY_MINOR   7  // read/write boundary pins of the aux board FPGA


#define X3X3_EXIF_EXIF    0 // read encoded Exif data (SEEK_END,
#define X3X3_EXIF_META    1 // write metadata, concurently opened files. All writes atomic
// control/setup devices
#define X3X3_EXIF_TEMPLATE 2 // write Exif template
#define X3X3_EXIF_METADIR  3 // write metadata to Exif header translation (dir_table[MAX_EXIF_FIELDS])
// those 2 files will disable exif_enable and exif_valid, truncate file size to file pointer on release.
#define X3X3_EXIF_TIME     4 // write today/tomorrow date (YYYY:MM:DD) and number of seconds at today/tomorrow
                             // midnight (00:00:00) in seconds from epoch (long, startting from LSB)
