/*!********************************************************************************
 * @file   exif353.c
 * @brief  Drivers for Exif manipulation
 * @copyright Copyright (C) 2008 Elphel, Inc.

 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *  $Log: exif353.c,v $
 *  Revision 1.2  2009/02/18 06:27:34  elphel
 *  fixed system date calculation fro Exif (was 2 days ahead in the years after leap)
 *
 *  Revision 1.1.1.1  2008/11/27 20:04:00  elphel
 *
 *
 *  Revision 1.3  2008/09/20 00:29:49  elphel
 *  moved driver major/minor numbers to a single file - include/asm-cris/elphel/driver_numbers.h
 *
 *  Revision 1.2  2008/09/12 00:23:59  elphel
 *  removed cc353.c, cc353.h
 *
 *  Revision 1.4  2008/04/25 21:28:56  elphel
 *  added more functions for access from the IRQ, changed them to inline
 *
 *  Revision 1.3  2008/04/11 23:16:51  elphel
 *  removed unneeded local_irq_disable() after local_irq_save_flags()
 *
 *  Revision 1.2  2008/04/07 09:45:30  elphel
 *  removed unused variable
 *
 *  Revision 1.1  2008/04/07 09:09:07  elphel
 *  New driver to handle Exif
 *
 *
 */


//copied freom cxi2c.c - TODO:remove unneeded

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
//#include <linux/autoconf.h>
#include <linux/vmalloc.h>

//#include <asm/system.h>
//#include <asm/svinto.h>
#include <asm/byteorder.h> // endians
//#include <asm/io.h>

#include <asm/irq.h>

#include <asm/delay.h>
#include <asm/uaccess.h>

#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
#include <elphel/exifa.h>

//#include "fpgactrl.h"  // defines port_csp0_addr, port_csp4_addr
//
//#include "x3x3.h"
//#include "cc3x3.h"


#include "exif393.h"

#define D(x)
//#define D(x) printk("%s:%d:",__FILE__,__LINE__);x


//Major
//#define X3X3_EXIF 136
//Minors
//#define X3X3_EXIF_EXIF    0 // read encoded Exif data (SEEK_END,
//#define X3X3_EXIF_META    1 // write metadata, concurently opened files. All writes atomic
// control/setup devices
//#define X3X3_EXIF_TEMPLATE 2 // write Exif template
//#define X3X3_EXIF_METADIR  3 // write metadata to Exif header translation (dir_table[MAX_EXIF_FIELDS])
// those 2 files will disable exif_enabled and exif_valid, truncate file size to file pointer on release.
//#define X3X3_EXIF_TIME     4 // write today/tomorrow date (YYYY:MM:DD) and number of seconds at today/tomorrow
// midnight (00:00:00) in seconds from epoch (long, startting from LSB)



#define  X3X3_EXIF_DRIVER_NAME "Elphel (R) model 353 Exif device driver"
static DEFINE_SPINLOCK(lock);

//#define MAX_EXIF_FIELDS  256 // number of Exif tags in the header
//#define MAX_EXIF_SIZE   4096 // Exif data size 

static struct exif_dir_table_t dir_table[MAX_EXIF_FIELDS];
static int  exif_fields        = 0; // total number of the Exif fields in the header
static int  exif_template_size = 0; // size of Exif template
static char exif_template[MAX_EXIF_SIZE];

static int aexif_meta_size[SENSOR_PORTS] = {0,0,0,0}; // size of Exif meta data page (is it the same for all ports?) 393: set as individual
static int aexif_wp[SENSOR_PORTS]  =       {1,1,1,1}; // frame write pointer in the meta_buffer
static int aexif_enabled[SENSOR_PORTS] =   {0,0,0,0}; // enable storing of frame meta data, enable reading Exif data
static int aexif_valid[SENSOR_PORTS] =     {0,0,0,0}; // Exif tables and buffer are valid.
static char * ameta_buffer[SENSOR_PORTS]=  {NULL,NULL,NULL,NULL}; // dynamically allocated buffer to store frame meta data.

//static char * meta_buffer=NULL; // dynamically allocated buffer to store frame meta data.
// page 0 - temporary storage, 1..MAX_EXIF_FRAMES - buffer


// Common for all sensor ports
struct exif_time_t {
	char          tomorrow_date[10]; //!"YYYY:MM:DD"
	unsigned long tomorrow_sec;      //!seconds from epoch tomorrow at 00:00
	char          today_date[10];    //!"YYYY:MM:DD"
	unsigned long today_sec;         //!seconds from epoch today at 00:00
} exif_time;

static struct exif_datetime_t {
	char          datetime[20];      //!"YYYY:MM:DD HH:MM:SS\0"
	char          subsec[7];         //!ASCII microseconds (0-padded), ."\0"
} now_datetime;



static int        exif_open   (struct inode *inode, struct file *filp);
static int        exif_release(struct inode *inode, struct file *filp);
static loff_t     exif_lseek  (struct file * file, loff_t offset, int orig);
static ssize_t    exif_write  (struct file * file, const char * buf, size_t count, loff_t *off);
static ssize_t    exif_read   (struct file * file, char * buf, size_t count, loff_t *off);
static int __init exif_init(void);

static struct file_operations exif_fops = {
		owner:    THIS_MODULE,
		open:     exif_open,
		release:  exif_release,
		read:     exif_read,
		write:    exif_write,
		llseek:   exif_lseek
};
ssize_t minor_file_size(int minor) { //return current file size for different minors
	int sensor_port;
	switch (minor) {
	case X3X3_EXIF_TEMPLATE:
		return exif_template_size;
	case X3X3_EXIF_EXIF_CHN_0:
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
		sensor_port = minor - X3X3_EXIF_EXIF_CHN_0;
		return aexif_enabled[sensor_port]? (exif_template_size * (MAX_EXIF_FRAMES+1)):0;
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
		sensor_port = minor - X3X3_EXIF_META_CHN_0;
		return aexif_meta_size[sensor_port];
	case X3X3_EXIF_METADIR:
		return exif_fields * sizeof(struct exif_dir_table_t);
	case X3X3_EXIF_TIME:
		return sizeof(struct exif_time_t);
	default:return 0;
	}
}
ssize_t minor_max_size(int minor) { //return max file size for different minors
	switch (minor) {
	case X3X3_EXIF_TEMPLATE:
		return MAX_EXIF_SIZE;
	case X3X3_EXIF_EXIF_CHN_0:
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
		return MAX_EXIF_SIZE * (MAX_EXIF_FRAMES+1);
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
		return MAX_EXIF_SIZE;
	case X3X3_EXIF_METADIR:
		return MAX_EXIF_FIELDS * sizeof(struct exif_dir_table_t);
	case X3X3_EXIF_TIME:
		return sizeof(struct exif_time_t);
	default:
		return 0;
	}
}
void exif_invalidate(void) { // 393: OK, only invalidates all ayt once
	int sensor_port;
	for (sensor_port =0; sensor_port < SENSOR_PORTS; sensor_port++){
		aexif_enabled[sensor_port] = 0;
		aexif_valid[sensor_port]  = 0;
	}
}

//reallocate meta buffer to store per-frame meta data (later output as Exif)
// 393: Make both individual and all at once
int exif_rebuild(int frames) {
	int sensor_port,rslt;
	for (sensor_port =0; sensor_port < SENSOR_PORTS; sensor_port++){
		if ((rslt = exif_rebuild_chn(sensor_port, frames)) <0){
			return rslt;
		}
	}
	return 0;
}
int exif_rebuild_chn(int sensor_port, int frames) {
	int i,ml;
	char * meta_buffer = ameta_buffer[sensor_port];

	aexif_enabled[sensor_port] = 0;
	aexif_valid[sensor_port]  =  0;
	aexif_wp[sensor_port] =      1;
	// free buffer, if allocated
	if (meta_buffer) {
		vfree (meta_buffer);
		meta_buffer=NULL;
	}
	//  calculate page size
	if (exif_fields==0) return 0; // exif_valid==0;
	for (i=0; i < exif_fields; i++) {
		ml=dir_table[i].src+dir_table[i].len;
		if (ml > aexif_meta_size[sensor_port]) aexif_meta_size[sensor_port] = ml;
	}
	if (aexif_meta_size[sensor_port] > MAX_EXIF_SIZE) {
		printk ("%s:%d: Meta frame size (0x%x) is too big (>0x%x)\n",__FILE__,__LINE__, aexif_meta_size[sensor_port], MAX_EXIF_SIZE);
		return -1;
	}
	meta_buffer= vmalloc(aexif_meta_size[sensor_port] * (MAX_EXIF_FRAMES+1));
	if (!meta_buffer) {
		printk ("%s:%d: Failed to allocate memory (%d bytes)\n",__FILE__,__LINE__, aexif_meta_size[sensor_port] * (MAX_EXIF_FRAMES+1));
		return -1;
	}
	memset(meta_buffer, 0, aexif_meta_size[sensor_port] * (MAX_EXIF_FRAMES+1));
	ameta_buffer[sensor_port] = meta_buffer;
	aexif_valid[sensor_port]  = 1;
	return 0;
}

int exif_enable(int en) {
	int sensor_port,rslt;
	for (sensor_port =0; sensor_port < SENSOR_PORTS; sensor_port++){
		if ((rslt = exif_enable_chn(sensor_port, en)) <0){
			return rslt;
		}
	}
	return 0;
}

int exif_enable_chn(int sensor_port, int en) {
	int rslt;
	if (en) {
		if (!aexif_valid[sensor_port]) {
			if (((rslt=exif_rebuild_chn(sensor_port, MAX_EXIF_FRAMES))) <0) return rslt;
		}
		aexif_enabled[sensor_port] = 1;
	} else {
		aexif_enabled[sensor_port] = 0;
	}
	return 0;
}

int dir_find_tag (unsigned long tag) { //find location of the tag field in meta page 
	int indx;
	for (indx=0; indx < exif_fields; indx++) if (dir_table[indx].ltag==tag) return (int) dir_table[indx].src;
	return -1;
}

inline void write_meta_raw_irq(int sensor_port, char * data, int offset, int len) { //write data to meta, called from IRQ
	if (aexif_enabled[sensor_port])
		memcpy(&ameta_buffer[sensor_port][offset], data, len);
}

inline int write_meta_irq(int sensor_port, char * data, int * indx,  unsigned long ltag, int len) { //write data to meta, called from IRQ(len==0 => use field length)
	int i;
	if (!aexif_enabled[sensor_port]) return 0;
	if (indx && (dir_table[* indx].ltag==ltag)) i=*indx;
	else {
		for (i=0; i<exif_fields; i++) if (dir_table[i].ltag==ltag) break;
		if (i>=exif_fields) return -1; //ltag not found
	}
	if (len==0) len=dir_table[i].len;
	memcpy(&ameta_buffer[sensor_port][dir_table[i].src], data, len);
	if (indx)  * indx = i;
	return dir_table[i].src;
}

inline void putlong_meta_raw_irq(int sensor_port, unsigned long data, int offset) { //write data to meta (4 bytes, big endian), called from IRQ
	unsigned long bedata=__cpu_to_be32(data);
	if (aexif_enabled[sensor_port]) {
		memcpy(&ameta_buffer[sensor_port][ offset], &bedata, 4);
	}
}

inline int putlong_meta_irq(int sensor_port, unsigned long data, int * indx,  unsigned long ltag) { //write data to meta (4 bytes, big endian), from IRQ
	int i;
	unsigned long bedata=__cpu_to_be32(data);
	if (!aexif_enabled[sensor_port]) return -10;
	if (indx && (dir_table[* indx].ltag==ltag)) i=*indx;
	else {
		for (i=0; i<exif_fields; i++) if (dir_table[i].ltag==ltag) break;
		if (i>=exif_fields) return -1; //ltag not found
	}
	memcpy(&ameta_buffer[sensor_port][dir_table[i].src], &bedata, 4);
	if (indx)  * indx=i;
	return dir_table[i].src;
}



void write_meta_raw(int sensor_port, char * data, int offset, int len) { //write data to meta, called from outside IRQ (atomic)
	unsigned long flags;
	if (aexif_enabled[sensor_port]) {
		local_irq_save(flags);
		//local_irq_disable();
		memcpy(&ameta_buffer[sensor_port][ offset], data, len);
		local_irq_restore(flags);
	}
}
int write_meta(int sensor_port, char * data, int * indx,  unsigned long ltag, int len) { //write data to meta, from outside IRQ (atomic) (len==0 => use field length)
	int i;
	unsigned long flags;
	if (!aexif_enabled[sensor_port]) return 0;
	if (indx && (dir_table[* indx].ltag==ltag)) i=*indx;
	else {
		for (i=0; i<exif_fields; i++) if (dir_table[i].ltag==ltag) break;
		if (i>=exif_fields) return -1; //ltag not found
	}
	if (len==0) len=dir_table[i].len;
	local_irq_save(flags);
	memcpy(&ameta_buffer[sensor_port][dir_table[i].src], data, len);
	local_irq_restore(flags);
	if (indx)  * indx=i;
	return dir_table[i].src;
}

void putlong_meta_raw(int sensor_port, unsigned long data, int offset) { //write data to meta (4 bytes, big endian), called from outside IRQ (atomic)
	unsigned long flags;
	unsigned long bedata=__cpu_to_be32(data);
	if (aexif_enabled[sensor_port]) {
		local_irq_save(flags);
		//local_irq_disable();
		memcpy(&ameta_buffer[sensor_port][ offset], &bedata, 4);
		local_irq_restore(flags);
	}
}

int putlong_meta(int sensor_port, unsigned long data, int * indx,  unsigned long ltag) { //write data to meta (4 bytes, big endian), from outside IRQ (atomic)
	int i;
	unsigned long flags;
	unsigned long bedata=__cpu_to_be32(data);
	if (!aexif_enabled[sensor_port]) return -10;
	if (indx && (dir_table[* indx].ltag==ltag)) i=*indx;
	else {
		for (i=0; i<exif_fields; i++) if (dir_table[i].ltag==ltag) break;
		if (i>=exif_fields) return -1; //ltag not found
	}
	local_irq_save(flags);
	memcpy(&ameta_buffer[sensor_port][dir_table[i].src], &bedata, 4);
	local_irq_restore(flags);
	if (indx)  * indx=i;
	return dir_table[i].src;
}

// The next function is normally called from the interrupt service routine
// Encode time (epoch sec, usec) into static buffer, return pointer to the buffer 
// Uses struct exif_time that should be updated from the user space (once a day),
// calculates date/time ignoring leap seconds if not updated in time
/*
 * 393: Continue to use same static buffers for exif_time
 */
char * encode_time(char buf[27], unsigned long sec, unsigned long usec) {
	unsigned long s,d,m,y,y4,lp,h;
	spin_lock(&lock);
	if (((sec-exif_time.today_sec)>86400) || (sec < exif_time.today_sec)) {// today's time is not valid, try tomorrow:
		memcpy(&exif_time.today_date[0],&exif_time.tomorrow_date[0],sizeof(exif_time.today_date)+sizeof(exif_time.today_sec));
		if (((sec-exif_time.today_sec)>86400) || (sec < exif_time.today_sec)) {// today's time is _still_ not valid, has to do it itself :-(
			d=sec/86400;
			s=d*86400;
			y4=d/1461; // number of 4-year periods
			d-=1461*y4; //days after 1970, 1974, ...
			y=(d- ((d>=1095)?1:0))/365;
			d-=y*365+((y>2)?1:0);
			lp=(y==2);
			y+=4*y4+1970;
			if ((!lp) && (d>58)) d++;
			//      d+=(lp && (d>58))?1:0;
			D(printk("d=%ld, y=%ld, y4=%ld, lp=%ld\n",d, y, y4, lp));
			if (d>181) {
				if (d>273) {
					if (d>304) {
						if (d>334) {m=12;d-=334;} // December
						else       {m=11;d-=304;} // November
					} else       {m=10;d-=273;} // October
				} else {
					if (d>212) {
						if (d>243) {m= 9;d-=243;} // September
						else       {m= 8;d-=212;} // August
					} else       {m= 7;d-=181;} // July
				}
			} else {
				if (d>90) {
					if (d>120) {
						if (d>151) {m=6; d-=151;} // June
						else       {m=5; d-=120;} // May
					} else       {m=4; d-= 90;} // April
				} else {
					if (d>30) {
						if (d>59)  {m=3; d-= 59;} // March
						else       {m=2; d-= 30;} // February
					} else       {m=1; d++;   } // January
				}
			}
			D(printk("d=%ld, y=%ld, y4=%ld\n",d, y, y4));
			sprintf(exif_time.today_date,"%04ld:%02ld:%02ld",y,m,d);
			exif_time.today_sec=s;
		}
		memcpy (&now_datetime.datetime[0],exif_time.today_date,10);
		now_datetime.datetime[10]=' ';
		now_datetime.datetime[19]='\0';
		now_datetime.subsec[6]='\0';
	}
	// now we have valid exif_time.today_date, exif_time.today_sec;
	s=sec-exif_time.today_sec;
	h= s/3600;
	s-= 3600*h;
	m=  s/60;
	s-= 60*m;
	sprintf(&now_datetime.datetime[11],"%02ld:%02ld:%02ld",h,m,s);
	sprintf(&now_datetime.subsec[0],"%06ld",usec);
	memcpy(buf,&now_datetime.datetime[0],sizeof(now_datetime));
	//  return &now_datetime.datetime[0];
	spin_unlock(&lock);
	return buf;
}

int store_meta(int sensor_port) { //called from IRQ service - put current metadata to meta_buffer, return page index
	if (!aexif_enabled[sensor_port]) return 0;
	int retval=aexif_wp[sensor_port];
	memcpy(&ameta_buffer[sensor_port][aexif_wp[sensor_port] * aexif_meta_size[sensor_port]], ameta_buffer[sensor_port],aexif_meta_size[sensor_port]);
	aexif_wp[sensor_port]++;
	if (aexif_wp[sensor_port] > MAX_EXIF_FRAMES) aexif_wp[sensor_port] = 1;
	return retval;
}

//!++++++++++++++++++++++++++++++++++++ open() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static int exif_open(struct inode *inode, struct file *filp) {
	int p = MINOR(inode->i_rdev);
	int * pd= (int *) &(filp->private_data);
	switch (p) {
	case X3X3_EXIF_EXIF_CHN_0:
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
	case X3X3_EXIF_TEMPLATE:
	case X3X3_EXIF_METADIR:
	case X3X3_EXIF_TIME:
		break;
	default:return -EINVAL;
	}

	D(printk("exif_open, minor=%d\n",p));
	inode->i_size=minor_file_size(p);
	pd[0]=p; // just a minor number
	return 0;
}

//!++++++++++++++++++++++++++++++++++++ release() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static int exif_release(struct inode *inode, struct file *filp){
	int p = MINOR(inode->i_rdev);
	int * pd= (int *) &(filp->private_data);
	switch (p) {
	case X3X3_EXIF_EXIF_CHN_0:
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
		break;
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
		break;
	case X3X3_EXIF_TEMPLATE:
		break;
	case X3X3_EXIF_METADIR:
		break;
	case X3X3_EXIF_TIME:
		break;
	default:return -EINVAL;
	}

	D(printk("exif_open, minor=%d\n",p));
	inode->i_size=minor_file_size(p);
	pd[0]=p; // just a minor number
	return 0;
}

//!++++++++++++++++++++++++++++++++++++ lseek() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static loff_t exif_lseek  (struct file * file, loff_t offset, int orig) {
	int p=(int)file->private_data;
	int thissize=minor_file_size(p);
	int maxsize=minor_max_size(p);
	//   int sensor_port;
	int fp;
	switch (orig) {
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		file->f_pos += offset;
		break;
	case SEEK_END:
		//!overload
		if (offset<=0) {
			file->f_pos = thissize + offset;
		} else {

			switch (p) {
			case X3X3_EXIF_TEMPLATE: //enable/disable
				switch (offset) {
				case EXIF_LSEEK_DISABLE:
					exif_enable(0);
					break;
				case EXIF_LSEEK_ENABLE:
					if (exif_enable(1)<0) return -EOVERFLOW; //TODO: change code
					break;
				case EXIF_LSEEK_INVALIDATE:
					exif_invalidate();
					break;
				case EXIF_LSEEK_REBUILD:
					if (exif_rebuild(MAX_EXIF_FRAMES)<0) return -EOVERFLOW; //TODO: change code
					break;
				default:return -EINVAL;
				}
				break;
			case X3X3_EXIF_EXIF_CHN_0:
			case X3X3_EXIF_EXIF_CHN_1:
			case X3X3_EXIF_EXIF_CHN_2:
			case X3X3_EXIF_EXIF_CHN_3:
				//            sensor_port = p - X3X3_EXIF_EXIF_CHN_0;
				if (offset > MAX_EXIF_FRAMES) return -EOVERFLOW; //larger than buffer
				//            file->f_pos=exif_meta_size * offset;
				file->f_pos=exif_template_size * offset;
				break;
			case X3X3_EXIF_META: // iterate
				fp= dir_find_tag (offset);
				if (fp < 0) return -EOVERFLOW; // tag is not in the directory
				file->f_pos=fp;
				break;
			case X3X3_EXIF_META_CHN_0:
			case X3X3_EXIF_META_CHN_1:
			case X3X3_EXIF_META_CHN_2:
			case X3X3_EXIF_META_CHN_3:
				file->f_pos=offset*sizeof(struct exif_dir_table_t);
				break;
			case X3X3_EXIF_TIME:
				switch (offset) {
				case EXIF_LSEEK_TOMORROW_DATE:
					file->f_pos=exif_time.tomorrow_date - ((char *) &exif_time);
					break;
				case EXIF_LSEEK_TOMORROW_SEC:
					file->f_pos=((char *) &exif_time.tomorrow_sec) - ((char *) &exif_time);
					break;
				case EXIF_LSEEK_TODAY_DATE:
					file->f_pos=exif_time.today_date - ((char *) &exif_time);
					break;
				case EXIF_LSEEK_TODAY_SEC:
					file->f_pos=((char *) &exif_time.today_sec) - ((char *) &exif_time);
					break;
				default:return -EINVAL;
				}
				break;
				default:return -EINVAL;
			}
		}
		break;
	default:
		return -EINVAL;
	}
	/* truncate position */
	if (file->f_pos < 0) {
		file->f_pos = 0;
		return (-EOVERFLOW);
	}

	if (file->f_pos > maxsize) {
		file->f_pos = maxsize;
		return (-EOVERFLOW);
	}
	return (file->f_pos);
}

//!++++++++++++++++++++++++++++++++++++ write() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ssize_t    exif_write  (struct file * file, const char * buf, size_t count, loff_t *off) {
	int p=(int)file->private_data;
	int sensor_port;
	//  int thissize=minor_file_size(p);
	int maxsize=minor_max_size(p);
	char * cp;
	char tmp[MAX_EXIF_SIZE]; //! Or is it possible to disable IRQ while copy_from_user()?
	unsigned long flags;
	int disabled_err=0;
	if ((*off+count)>maxsize) {
		printk ("%s:%d: Data (0x%x) does not fit into 0x%x bytes\n",__FILE__,__LINE__, (int) (*off+count), maxsize);
		return -EOVERFLOW;
	}
	switch (p) {
	case X3X3_EXIF_TEMPLATE:
		exif_invalidate();
		if (copy_from_user(&exif_template[*off], buf, count)) return -EFAULT;
		exif_template_size=*off+count;
		break;
	case X3X3_EXIF_METADIR:
		exif_invalidate();
		cp= (char *) &dir_table;
		if (copy_from_user(&cp[*off], buf, count)) return -EFAULT;
		exif_fields=(*off+count)/sizeof(struct exif_dir_table_t);
		break;
	case X3X3_EXIF_TIME: //write date/time first, then - midnight seconds
		cp= (char *) &exif_time;
		if (copy_from_user(&cp[*off], buf, count)) return -EFAULT;
		break;
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
		sensor_port = p - X3X3_EXIF_META_CHN_0;
		if (copy_from_user(tmp, buf, count)) return -EFAULT;
		local_irq_save(flags);
		//local_irq_disable();
		if (aexif_enabled[sensor_port]) memcpy(&ameta_buffer[sensor_port][*off], tmp, count);
		else disabled_err=1;
		local_irq_restore(flags);
		if (disabled_err) {
			D(printk("tried to write meta channel %d while disabled\n",sensor_port));
			count=0;
		}
		break;
	case X3X3_EXIF_EXIF_CHN_0:
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
		return -EINVAL; // no writing - read only
		break;
	default:return -EINVAL;
	}
	*off+=count;
	D(printk("count= 0x%x, pos= 0x%x\n", (int) count, (int)*off));
	return count;
}

//!++++++++++++++++++++++++++++++++++++ read() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ssize_t    exif_read   (struct file * file, char * buf, size_t count, loff_t *off) {
	int p=(int)file->private_data;
	int thissize=minor_file_size(p);
	char * cp, * metap;
	int start_p, page_p,i;
	int sensor_port;
	char tmp[MAX_EXIF_SIZE]; //! Or is it possible to disable IRQ while copy_from_user()?
	/*
  Does not work with PHP - always read 8192 bytes
  if ((*off+count)>maxsize) {
    printk ("%s:%d: Data (0x%x) does not fit into 0x%x bytes\n",__FILE__,__LINE__, (int) (*off+count), maxsize);
    return -EOVERFLOW;
  }
	 */
	if (*off > thissize) {
		return 0; // nothing to read
	}
	if ((*off + count) > thissize) {
		count=thissize-*off;
	}

	switch (p) {
	case X3X3_EXIF_TEMPLATE:
		if (copy_to_user(buf,  &exif_template[*off], count)) return -EFAULT;
		break;
	case X3X3_EXIF_METADIR:
		cp= (char *) &dir_table;
		if (copy_to_user(buf,  &cp[*off], count)) return -EFAULT;
		break;
	case X3X3_EXIF_TIME:
		cp= (char *) &exif_time;
		if (copy_to_user(buf,  &cp[*off], count)) return -EFAULT;
		break;
	case X3X3_EXIF_META_CHN_0:
	case X3X3_EXIF_META_CHN_1:
	case X3X3_EXIF_META_CHN_2:
	case X3X3_EXIF_META_CHN_3:
		sensor_port = p - X3X3_EXIF_META_CHN_0;
		if (!aexif_enabled[sensor_port]) return 0;
		if (copy_to_user(buf,  &ameta_buffer[sensor_port][*off], count)) return -EFAULT;
		break;
	case X3X3_EXIF_EXIF_CHN_0:// generates exif data by merging exif_template with the selected meta_buffer page
	case X3X3_EXIF_EXIF_CHN_1:
	case X3X3_EXIF_EXIF_CHN_2:
	case X3X3_EXIF_EXIF_CHN_3:
		sensor_port = p - X3X3_EXIF_EXIF_CHN_0;
		//will truncate by the end of current page
		if (!aexif_enabled[sensor_port]) return 0;
		i=((int) *off) / exif_template_size;
		D(printk("count= 0x%x, *off= 0x%x, i=0x%x, exif_template_size=0x%x\n", (int) count, (int) *off, (int) i, (int) exif_template_size));
		//arch/cris/arch-v32/drivers/elphel/exif353.c:590:count= 0x2000, *off= 0x410, i=0x82, exif_template_size=0x208
		start_p=i*exif_template_size;
		page_p= *off - start_p;
		D(printk("count= 0x%x, pos= 0x%x, start_p=0x%x, page_p=0x%x, i=0x%x, exif_template_size=0x%x\n", (int) count, (int) *off, (int)start_p, (int)page_p,(int) i, (int) exif_template_size));
		//arch/cris/arch-v32/drivers/elphel/exif353.c:591:count= 0x2000, pos= 0x410, start_p=0x10810, page_p=0xfffefc00, i=0x82, exif_template_size=0x208
		metap= &ameta_buffer[sensor_port][i*aexif_meta_size[sensor_port]]; // pointer to the start of the selected page in frame meta_buffer
		if ((page_p+count) > exif_template_size) count=exif_template_size-page_p;
		memcpy(tmp,exif_template, exif_template_size);
		D(printk("count= 0x%x, pos= 0x%x, start_p=0x%x, page_p=0x%x\n", (int) count, (int) *off, (int)start_p, (int)page_p));
		for (i=0;i<exif_fields;i++){
			memcpy(&tmp[dir_table[i].dst],&metap[dir_table[i].src], dir_table[i].len);
		}
		if (copy_to_user(buf,  &tmp[page_p], count)) return -EFAULT;
		break;
	default:return -EINVAL;
	}
	*off+=count;
	D(printk("count= 0x%x, pos= 0x%x\n", (int) count, (int)*off));
	return count;
}


//!++++++++++++++++++++++++++++++++++++ _init() ++++++++++++++++++++++++++++++++++++++++++++++++++++++

static int __init exif_init(void) {
	int res;
	res = register_chrdev(X3X3_EXIF_MAJOR, "Exif", &exif_fops);
	if(res < 0) {
		printk(KERN_ERR "\nexif_init: couldn't get a major number  %d.\n",X3X3_EXIF_MAJOR);
		return res;
	}
	printk(X3X3_EXIF_DRIVER_NAME" - %d\n",X3X3_EXIF_MAJOR);
	return 0;
}

module_init(exif_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION(X3X3_EXIF_DRIVER_NAME);
