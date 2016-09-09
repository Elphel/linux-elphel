/** @file ahci_cmd.h
 *
 * @brief Elphel AHCI SATA platform driver for Elphel393 camera. This module provides
 * constants and data structures which are used to organize interaction between drivers
 * and user space applications during JPEG files recording.
 *
 * @copyright Copyright (C) 2016 Elphel, Inc
 *
 * @par <b>License</b>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _AHCI_CMD
#define _AHCI_CMD

#define DRV_CMD_WRITE             (1 << 0)
#define DRV_CMD_FINISH            (1 << 1)
#define DRV_CMD_EXIF              (1 << 2)

#define NAME_TO_STR(NAME)         #NAME
/** The path to Elphel AHCI driver sysfs entry. The trailing slash is mandatory. */
#define SYSFS_AHCI_ENTRY          "/sys/devices/soc0/amba@0/80000000.elphel-ahci/"
/** sysfs entry name, no double quotes. This macro is used to populate <em>struct attribute</em> in #ahci_elphel.c */
#define SYSFS_AHCI_FNAME_WRITE    write
/** sysfs entry name, no double quotes. This macro is used to populate <em>struct attribute</em> in #ahci_elphel.c */
#define SYSFS_AHCI_FNAME_START    lba_start
/** sysfs entry name, no double quotes. This macro is used to populate <em>struct attribute</em> in #ahci_elphel.c */
#define SYSFS_AHCI_FNAME_END      lba_end
/** sysfs entry name, no double quotes. This macro is used to populate <em>struct attribute</em> in #ahci_elphel.c */
#define SYSFS_AHCI_FNAME_CURR     lba_current
/** This file is used to send commands to AHCI driver from user space applications (camogm as for now). */
#define SYSFS_AHCI_WRITE          SYSFS_AHCI_ENTRY NAME_TO_STR(SYSFS_AHCI_FNAME_WRITE)
/** This file is used to control starting LBA of a disk buffer (R/W). */
#define SYSFS_AHCI_LBA_START      SYSFS_AHCI_ENTRY NAME_TO_STR(SYSFS_AHCI_FNAME_START)
/** This file is used to control ending LBA of a disk buffer (R/W). */
#define SYSFS_AHCI_LBA_END        SYSFS_AHCI_ENTRY NAME_TO_STR(SYSFS_AHCI_FNAME_END)
/** This file is used to control current LBA of a disk buffer (R/W). Use this file to set a pointer inside
 * [lba_start..lba_end] area where next write operation will begin. */
#define SYSFS_AHCI_LBA_CURRENT    SYSFS_AHCI_ENTRY NAME_TO_STR(SYSFS_AHCI_FNAME_CURR)

struct frame_data {
       unsigned int sensor_port;
       int cirbuf_ptr;
       int jpeg_len;
       int meta_index;
       int cmd;
};

#endif /* _AHCI_CMD */
