/***************************************************************************//**
* @file      x393_devices.h
* @brief     Definitions of static device files, major and minor numbers
* and userland.
* @copyright Copyright 2002-2016 (C) Elphel, Inc.
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

/* Each device node should be specified in the following format:
#define <DEVICE_RFEFERENCE>   ("<dev path>",    "<driver_name>",  major, minor,"<permissions>","<b/c>") [optional comment]
 * Access to individual fields is provide with the macros defined below, for example:
 * DEV393_PATH(DEV393_EXIF_TEMPLATE) returns "/dev/exif_template,
 * DEV393_MAJOR(DEV393_EXIF_TEMPLATE) returns 2
 * DO NOT COMMET OUT defines with // ! */

#define DEV393_EXIF_TEMPLATE  ("exif_template",     "exif_elphel",   125,  2, "0666", "c")  ///< write Exif template
#define DEV393_EXIF_METADIR   ("exif_metadir",      "exif_elphel",   125,  3, "0666", "c")  ///< write metadata to Exif header translation (dir_table[MAX_EXIF_FIELDS])
#define DEV393_EXIF_TIME      ("exif_time",         "exif_elphel",   125,  4, "0666", "c")  ///< write today/tomorrow date (YYYY:MM:DD) and number of seconds at today/tomorrow
                                                                             ///< midnight (00:00:00) in seconds from epoch (long, starting from LSB)
#define DEV393_TIFF_TEMPLATE  ("tiff_template",     "exif_elphel",   125,  5, "0666", "c")  ///< write Tiff template

#define DEV393_EXIF0          ("exif_exif0",        "exif_elphel",   125, 16, "0666", "c")  ///< sensor port 0: read encoded Exif data (SEEK_END)
#define DEV393_EXIF1          ("exif_exif1",        "exif_elphel",   125, 17, "0666", "c")  ///< sensor port 1: read encoded Exif data (SEEK_END)
#define DEV393_EXIF2          ("exif_exif2",        "exif_elphel",   125, 18, "0666", "c")  ///< sensor port 2: read encoded Exif data (SEEK_END)
#define DEV393_EXIF3          ("exif_exif3",        "exif_elphel",   125, 19, "0666", "c")  ///< sensor port 3: read encoded Exif data (SEEK_END)

#define DEV393_EXIF_META0     ("exif_meta0",        "exif_elphel",   125, 32, "0666", "c")  ///< sensor port 0: write metadata, concurrently opened files. All writes are atomic
#define DEV393_EXIF_META1     ("exif_meta1",        "exif_elphel",   125, 33, "0666", "c")  ///< sensor port 1: write metadata, concurrently opened files. All writes are atomic
#define DEV393_EXIF_META2     ("exif_meta2",        "exif_elphel",   125, 34, "0666", "c")  ///< sensor port 2: write metadata, concurrently opened files. All writes are atomic
#define DEV393_EXIF_META3     ("exif_meta3",        "exif_elphel",   125, 35, "0666", "c")  ///< sensor port 3: write metadata, concurrently opened files. All writes are atomic

#define DEV393_TIFF0          ("tiff_tiff0",        "exif_elphel",   125, 24, "0666", "c")  ///< sensor port 0: read encoded Tiff data (SEEK_END)
#define DEV393_TIFF1          ("tiff_tiff1",        "exif_elphel",   125, 25, "0666", "c")  ///< sensor port 1: read encoded Tiff data (SEEK_END)
#define DEV393_TIFF2          ("tiff_tiff2",        "exif_elphel",   125, 26, "0666", "c")  ///< sensor port 2: read encoded Tiff data (SEEK_END)
#define DEV393_TIFF3          ("tiff_tiff3",        "exif_elphel",   125, 27, "0666", "c")  ///< sensor port 3: read encoded Tiff data (SEEK_END)

// Meta data common for Exif and Tiff

#define DEV393_FRAMEPARS0     ("frameparsall0","framepars_operations",130,80, "0666", "c")  ///< Access frame parameters for channel 0 (schedule modification, read with mmap)
#define DEV393_FRAMEPARS1     ("frameparsall1","framepars_operations",130,81, "0666", "c")  ///< Access frame parameters for channel 1 (schedule modification, read with mmap)
#define DEV393_FRAMEPARS2     ("frameparsall2","framepars_operations",130,82, "0666", "c")  ///< Access frame parameters for channel 2 (schedule modification, read with mmap)
#define DEV393_FRAMEPARS3     ("frameparsall3","framepars_operations",130,83, "0666", "c")  ///< Access frame parameters for channel 3 (schedule modification, read with mmap)

#define DEV393_JTAG_RESET     ("fpgaresetjtag",     "x393_jtag",     132,  0, "0666", "c")  ///< Just close open files (same as jtagraw)
#define DEV393_JTAG_RAW       ("jtagraw",           "x393_jtag",     132,  0, "0666", "c")  ///< Just close open files (same as jtagraw)

#define DEV393_JTAGS_CONF0    ("sfpgaconfjtag0",    "x393_jtag",     132,  8, "0666", "c")  ///< JTAG configure sensor port 0 FPGA
#define DEV393_JTAGS_CONF1    ("sfpgaconfjtag1",    "x393_jtag",     132,  9, "0666", "c")  ///< JTAG configure sensor port 1 FPGA
#define DEV393_JTAGS_CONF2    ("sfpgaconfjtag2",    "x393_jtag",     132, 10, "0666", "c")  ///< JTAG configure sensor port 2 FPGA
#define DEV393_JTAGS_CONF3    ("sfpgaconfjtag3",    "x393_jtag",     132, 11, "0666", "c")  ///< JTAG configure sensor port 3 FPGA

#define DEV393_JTAGS_BSCAN0   ("sfpgabscan0",       "x393_jtag",     132, 12, "0666", "c")  ///< JTAG boundary scan sensor port 0 FPGA
#define DEV393_JTAGS_BSCAN1   ("sfpgabscan1",       "x393_jtag",     132, 13, "0666", "c")  ///< JTAG boundary scan sensor port 1 FPGA
#define DEV393_JTAGS_BSCAN2   ("sfpgabscan2",       "x393_jtag",     132, 14, "0666", "c")  ///< JTAG boundary scan sensor port 2 FPGA
#define DEV393_JTAGS_BSCAN3   ("sfpgabscan3",       "x393_jtag",     132, 15, "0666", "c")  ///< JTAG boundary scan sensor port 3 FPGA

#define DEV393_I2C_CTRL       ("xi2cctl",           "fpga_xi2c",     134,  0, "0666", "c")  ///< control/reset i2c
#define DEV393_I2C_8_AINC     ("xi2c8",             "fpga_xi2c",     134,  1, "0666", "c")  ///< 8bit  registers, autoincrement while read/write (NC393: Not used)
#define DEV393_I2C_16_AINC    ("xi2c16",            "fpga_xi2c",     134,  2, "0666", "c")  ///< 16bit registers, autoincrement while read/write (NC393: Not used)
#define DEV393_I2C1_8_AINC    ("xi2c8_aux",         "fpga_xi2c",     134,  3, "0666", "c")  ///< 8bit  registers, autoincrement while read/write (bus 1)
#define DEV393_I2C1_16_AINC   ("xi2c16_aux",        "fpga_xi2c",     134,  4, "0666", "c")  ///< 16bit registers, autoincrement while read/write (bus 1)
#define DEV393_I2C_RAW        ("xi2craw",           "fpga_xi2c",     134,  5, "0666", "c")  ///< 8bit  registers, no address byte (just slave, then read/write byte(s)  (NC393: Not used)
#define DEV393_I2C1_RAW       ("xi2craw_aux",       "fpga_xi2c",     134,  6, "0666", "c")  ///< 8bit  registers, no address byte (just slave, then read/write byte(s) bus 1
#define DEV393_I2C_ENABLE     ("xi2cenable",        "fpga_xi2c",     134,  7, "0666", "c")  ///< enable(/protect) different I2C devices for different types of I2C accesses

#define DEV393_CIRCBUF0       ("circbuf0",          "circbuf",       135, 32, "0666", "c")  ///< circbuf for channel 0
#define DEV393_CIRCBUF1       ("circbuf1",          "circbuf",       135, 33, "0666", "c")  ///< circbuf for channel 1
#define DEV393_CIRCBUF2       ("circbuf2",          "circbuf",       135, 34, "0666", "c")  ///< circbuf for channel 2
#define DEV393_CIRCBUF3       ("circbuf3",          "circbuf",       135, 35, "0666", "c")  ///< circbuf for channel 3

#define DEV393_JPEGHEAD0      ("jpeghead0",         "circbuf",       135, 48, "0666", "c")  ///< JPEG header for channel 0
#define DEV393_JPEGHEAD1      ("jpeghead1",         "circbuf",       135, 49, "0666", "c")  ///< JPEG header for channel 1
#define DEV393_JPEGHEAD2      ("jpeghead2",         "circbuf",       135, 50, "0666", "c")  ///< JPEG header for channel 2
#define DEV393_JPEGHEAD3      ("jpeghead3",         "circbuf",       135, 51, "0666", "c")  ///< JPEG header for channel 3

#define DEV393_HUFFMAN0       ("huffman0",          "circbuf",       135, 64, "0666", "c")  ///< Huffman table for channel 0
#define DEV393_HUFFMAN1       ("huffman1",          "circbuf",       135, 65, "0666", "c")  ///< Huffman table for channel 1
#define DEV393_HUFFMAN2       ("huffman2",          "circbuf",       135, 66, "0666", "c")  ///< Huffman table for channel 2
#define DEV393_HUFFMAN3       ("huffman3",          "circbuf",       135, 67, "0666", "c")  ///< Huffman table for channel 3

#define DEV393_GAMMA          ("gamma_cache","gamma_tables_operations",137,17,"0666", "c")  ///< Cache for calculated gamma tables (common for all ports/channels/colors)
#ifdef INDIVIDUAL_HISTOGRAMS
#define DEV393_HISTOGRAM00    ("histogram_cache00","histograms_operations",138,80,"0666","c") ///< Access to acquired/calculated histograms for port 0 /subcchanne 0
#define DEV393_HISTOGRAM01    ("histogram_cache01","histograms_operations",138,81,"0666","c") ///< Access to acquired/calculated histograms for port 0 /subcchanne 1
#define DEV393_HISTOGRAM02    ("histogram_cache02","histograms_operations",138,82,"0666","c") ///< Access to acquired/calculated histograms for port 0 /subcchanne 2
#define DEV393_HISTOGRAM03    ("histogram_cache03","histograms_operations",138,83,"0666","c") ///< Access to acquired/calculated histograms for port 0 /subcchanne 3
#define DEV393_HISTOGRAM10    ("histogram_cache10","histograms_operations",138,84,"0666","c") ///< Access to acquired/calculated histograms for port 1 /subcchanne 0
#define DEV393_HISTOGRAM11    ("histogram_cache11","histograms_operations",138,85,"0666","c") ///< Access to acquired/calculated histograms for port 1 /subcchanne 1
#define DEV393_HISTOGRAM12    ("histogram_cache12","histograms_operations",138,86,"0666","c") ///< Access to acquired/calculated histograms for port 1 /subcchanne 2
#define DEV393_HISTOGRAM13    ("histogram_cache13","histograms_operations",138,87,"0666","c") ///< Access to acquired/calculated histograms for port 1 /subcchanne 3
#define DEV393_HISTOGRAM20    ("histogram_cache20","histograms_operations",138,88,"0666","c") ///< Access to acquired/calculated histograms for port 2 /subcchanne 0
#define DEV393_HISTOGRAM21    ("histogram_cache21","histograms_operations",138,89,"0666","c") ///< Access to acquired/calculated histograms for port 2 /subcchanne 1
#define DEV393_HISTOGRAM22    ("histogram_cache22","histograms_operations",138,90,"0666","c") ///< Access to acquired/calculated histograms for port 2 /subcchanne 2
#define DEV393_HISTOGRAM23    ("histogram_cache23","histograms_operations",138,91,"0666","c") ///< Access to acquired/calculated histograms for port 2 /subcchanne 3
#define DEV393_HISTOGRAM30    ("histogram_cache30","histograms_operations",138,92,"0666","c") ///< Access to acquired/calculated histograms for port 3 /subcchanne 0
#define DEV393_HISTOGRAM31    ("histogram_cache31","histograms_operations",138,93,"0666","c") ///< Access to acquired/calculated histograms for port 3 /subcchanne 1
#define DEV393_HISTOGRAM32    ("histogram_cache32","histograms_operations",138,94,"0666","c") ///< Access to acquired/calculated histograms for port 3 /subcchanne 2
#define DEV393_HISTOGRAM33    ("histogram_cache33","histograms_operations",138,95,"0666","c") ///< Access to acquired/calculated histograms for port 3 /subcchanne 3
#else
#define DEV393_HISTOGRAM      ("histogram_cache","histograms_operations",138,18,"0666","c") ///< Access to acquired/calculated histograms for all ports/subchannels
#endif

#define DEV393_LOGGER         ("imu",               "imu_logger",    141,  1, "0666", "c")  ///< IMU/GPS/images logs data access
#define DEV393_LOGGER_CTRL    ("imu_ctl",           "imu_logger",    141,  2, "0666", "c")  ///< IMU/GPS/images logger control
// Video memory access uses a single (shared) membridge module, so device driver should have exclusive access
#define DEV393_VIDEOMEM_RAW   ("videomem_raw",      "video_mem",     142,  1, "0666", "c")  ///< Raw access to video memory using membridge module (NC393: Not yet implemented)
#define DEV393_IMAGE_RAW      ("image_raw",         "video_mem",     142,  2, "0666", "c")  ///< Access to raw (uncompressed) data in video memory, frame-organized
// 80-83 - minor to use minor_to_chn function
#define DEV393_IMAGE_RAW0     ("image_raw0",        "video_mem",     142, 80, "0666", "c")  ///< Channel 0. Access to raw (uncompressed) data in video memory, frame-organized
#define DEV393_IMAGE_RAW1     ("image_raw1",        "video_mem",     142, 81, "0666", "c")  ///< Channel 1. Access to raw (uncompressed) data in video memory, frame-organized
#define DEV393_IMAGE_RAW2     ("image_raw2",        "video_mem",     142, 82, "0666", "c")  ///< Channel 2. Access to raw (uncompressed) data in video memory, frame-organized
#define DEV393_IMAGE_RAW3     ("image_raw3",        "video_mem",     142, 83, "0666", "c")  ///< Channel 3. Access to raw (uncompressed) data in video memory, frame-organized

#define DEV393_DETECT_SENSORS ("detect_sensors",    "detect_sensors",143,  1, "0666", "c")  ///< Probably not needed, only sysfs is used
#define DEV393_I2C_SENSORS    ("",            "elphel393-sensor-i2c", -1, -1, "0666", "c")  ///< Used only in sysfs, no character device (yet?)
#define DEV393_MT9X001        ("",               "elphel393-mt9x001", -1, -1, "0666", "c")  ///< Used only in sysfs, no character device (yet?)
#define DEV393_MT9F002        ("",               "elphel393-mt9f002", -1, -1, "0666", "c")  ///< Used only in sysfs, no character device (yet?)
#define DEV393_LEPTON         ("",                "elphel393-lepton", -1, -1, "0666", "c")  ///< Used only in sysfs, no character device (yet?)
#define DEV393_KLOGGER        ("klogger_393",          "klogger_393",144,  1, "0666", "c")  ///< kernel event logger to memory (no i/o)

#define _DEV393_DEVNAME(n, ...) n
#define _DEV393_PATH(n, ...) "/dev/"n
#define _DEV393_NAME(a,n, ...) n
#define _DEV393_MAJOR(a,b,n, ...) n
#define _DEV393_MINOR(a,b,c,n, ...) n
#define _DEV393_PERMISSIONS(a,b,c,d,n, ...) n
#define _DEV393_TYPE(a,b,c,d,e,n,...) n

#define DEV393_DEVNAME(LIST)      _DEV393_DEVNAME LIST       ///< @return the device node name as string ("somediviceN")
#define DEV393_PATH(LIST)         _DEV393_PATH LIST          ///< @return full path of the device node as string ("/dev/somedivice")
#define DEV393_NAME(LIST)         _DEV393_NAME LIST          ///< @return driver name
#define DEV393_MAJOR(LIST)        _DEV393_MAJOR LIST         ///< @return device major number
#define DEV393_MINOR(LIST)        _DEV393_MINOR LIST         ///< @return device minor number
#define DEV393_PERMISSIONS(LIST)  _DEV393_PERMISSIONS LIST   ///< @return device permissions as quoted string
#define DEV393_TYPE(LIST)         _DEV393_TYPE LIST          ///< @return device type: "b" for block devices, "c" - for character ones


