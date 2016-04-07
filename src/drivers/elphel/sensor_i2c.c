/*******************************************************************************
* FILE NAME  : sensor_i2c.c
* DESCRIPTION: Interface to FPGA-based i2c sequencer for sensor ports
* Copyright 2016 (C) Elphel, Inc.
* -----------------------------------------------------------------------------*
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
*******************************************************************************/
/****************** INCLUDE FILES SECTION ***********************************/

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
//#include <linux/poll.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/spinlock.h>
//#include <linux/spinlock_types.h>

#include <asm/io.h>
//#include <asm/system.h>
#include <asm/irq.h>

#include <elphel/driver_numbers.h>
#include <asm/uaccess.h>

#include "x393.h"
#include "sensor_i2c.h"

//------------------

#include <asm/page.h>
#include <linux/io.h>

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

#define DRV_NAME "elphel_sensor_i2c"

//------------------


// Number of channels is hard-wired to 4
static u32 free_i2c_groups[4];
static u32 free_i2c_pages [32];
static DEFINE_SPINLOCK(lock);
static u32 i2c_pages_shadow[1024]; // Mostly for debugging to analyze i2c pages allocation
static int sysfs_page[4]; // when positive - page locked for exclusive access
/* Mark all i2c pages for each channel as free */
void i2c_page_alloc_init(void){
	int i;
	for (i = 0; i < sizeof(free_i2c_groups)/sizeof(free_i2c_groups[0]); i++) free_i2c_groups[i] = 0xff000000;
	for (i = 0; i < sizeof(free_i2c_pages)/ sizeof(free_i2c_pages[0]);  i++) free_i2c_pages[i] =  0xffffffff;
	for (i = 0; i < sizeof(i2c_pages_shadow)/sizeof(i2c_pages_shadow[0]); i++) i2c_pages_shadow[i] = 0;
	for (i = 0; i < sizeof(sysfs_page)/sizeof(sysfs_page[0]); i++) sysfs_page[i] = -1;

}

/* Reserve i2c page (1 of 256 for a sensor port)*/
int i2c_page_alloc(int chn){
	int g, b;
	u32 * fb;
	spin_lock(&lock);
	g = __builtin_clz(free_i2c_groups[chn]);
	if (unlikely(g > 7)) {
		spin_unlock(&lock);
		return -ENOMEM; // no free i2c pages left
	}
	fb = free_i2c_pages + ((chn << 3) + g);
	b = __builtin_clz(*fb);
	*fb &= ~ (1 << (31-b));
	if (unlikely(*fb == 0)){
		free_i2c_groups[chn] &= ~(1 << (31 - g));
	}
	spin_unlock(&lock);
	return (g << 5) + b;
}

/* Free i2c page */
void i2c_page_free(int chn, int page){
	int g =    page >> 5;
	int b =    page & 0x1f;
	u32 * fb = free_i2c_pages + ((chn << 3) + g);
	spin_lock(&lock);
	free_i2c_groups[chn] |=  1 << (31 - g);
	*fb |= 1 << (31-b);
	spin_unlock(&lock);
}
/* Set i2c table entry to raw data (will just overwrite tbl_mode = 2)*/
void set_sensor_i2c_raw(int chn,
		                int page,   // index in lookup table
					    u32 data) { // Bit delay - number of mclk periods in 1/4 of the SCL period
	x393_i2c_ctltbl_t tb_data, tb_addr;
	tb_addr.d32 =      0;
	tb_addr.tbl_mode = 3;
	tb_data.d32 =      data;
	tb_data.tbl_mode = 2; //
	/* Table address and data should not interleave with others */
	spin_lock(&lock);
	x393_sensi2c_ctrl (tb_addr, chn);
	x393_sensi2c_ctrl (tb_data, chn);
	spin_unlock(&lock);
	i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
}

/* Set i2c table entry for write operation */
void set_sensor_i2c_wr(int chn,
		              int page,        // index in lookup table
					  int sa,          // slave address (7 bit)
					  int rah,         // High byte of the i2c register address
					  int num_bytes,   //Number of bytes to write (1..10)
					  int bit_delay) { // Bit delay - number of mclk periods in 1/4 of the SCL period
	x393_i2c_ctltbl_t tb_data, tb_addr;
	tb_addr.d32 =      0;
	tb_addr.tbl_mode = 3;
	tb_data.d32 =      0;
	tb_data.rah =      rah;
	tb_data.rnw =      0;
	tb_data.sa =       sa;
	tb_data.nbwr =     num_bytes;
	tb_data.dly =      bit_delay;
	tb_data.tbl_mode = 2;
	/* Table address and data should not interleave with others */
	spin_lock(&lock);
	x393_sensi2c_ctrl (tb_addr, chn);
	x393_sensi2c_ctrl (tb_data, chn);
	spin_unlock(&lock);
	i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
}

/* Set i2c table entry for read operation */
void set_sensor_i2c_rd(int chn,
		              int page,          // index in lookup table
					  int two_byte_addr, // Number of address bytes (0 - one byte, 1 - two bytes)
					  int num_bytes,     // Number of bytes to read (1..8, 0 means 8)
					  int bit_delay) { // Bit delay - number of mclk periods in 1/4 of the SCL period
	x393_i2c_ctltbl_t tb_data, tb_addr;
	tb_addr.d32 =      0;
	tb_addr.tbl_mode = 3;
	tb_data.d32 =      0;
	tb_data.rnw =      1;
	tb_data.nabrd =    two_byte_addr;
	tb_data.nbrd =     num_bytes;
	tb_data.dly =      bit_delay;
	tb_data.tbl_mode = 2;
	/* Table address and data should not interleave with others */
	spin_lock(&lock);
	x393_sensi2c_ctrl (tb_addr, chn);
	x393_sensi2c_ctrl (tb_data, chn);
	spin_unlock(&lock);
	i2c_pages_shadow[(chn << 8) + page] =tb_data.d32;
}

/*
// Write i2c command to the i2c command sequencer
// I2C command sequencer, block of 16 DWORD slots for absolute frame numbers (modulo 16) and 15 slots for relative ones
// 0 - ASAP, 1 next frame, 14 -14-th next.
// Data written depends on context:
// 1 - I2C register write: index page (MSB), 3 payload bytes. Payload bytes are used according to table and sent
//     after the slave address and optional high address byte. Other bytes are sent in descending order (LSB- last).
//     If less than 4 bytes are programmed in the table the high bytes (starting with the one from the table) are
//     skipped.
//     If more than 4 bytes are programmed in the table for the page (high byte), one or two next 32-bit words
//     bypass the index table and all 4 bytes are considered payload ones. If less than 4 extra bytes are to be
//     sent for such extra word, only the lower bytes are sent.
//
// 2 - I2C register read: index page, slave address (8-bit, with lower bit 0) and one or 2 address bytes (as programmed
//     in the table. Slave address is always in byte 2 (bits 23:16), byte1 (high register address) is skipped if
//     read address in the table is programmed to be a single-byte one
 */

/* Write one or multiple DWORDs to i2c relative (modulo16) address. Use offs = 0 for immediate (ASAP) command */
/* Length of data is determined by the page data already preset */
int write_sensor_i2c_rel (int chn,
                          int offs, // 4 bits
                          u32 * data){
	x393_i2c_ctltbl_t tb_data;
	int len;
	int i;
	tb_data.d32 = i2c_pages_shadow[(chn <<8) + (data[0] >> 24)];
	if (tb_data.tbl_mode !=2) return -1;
	len = (tb_data.rnw )? 1:((tb_data.nbwr + 5) >> 2); // read mode - always 1 DWORD, write - 1..3
	if (len > 1) {
		spin_lock(&lock);
		for (i = 0; i <len; i++){
			x393_sensi2c_rel (data[i], chn, offs);
		}
	} else {
		x393_sensi2c_rel (data[0], chn, offs);
	}
	spin_unlock(&lock);
	return 0;
}

int write_sensor_i2c_abs (int chn,
                          int offs, // 4 bits
                          u32 * data){
	x393_i2c_ctltbl_t tb_data;
	int len;
	int i;
	tb_data.d32 = i2c_pages_shadow[(chn <<8) + (data[0] >> 24)];
	if (tb_data.tbl_mode !=2) return -1;
	len = (tb_data.rnw )? 1:((tb_data.nbwr + 5) >> 2); // read mode - always 1 DWORD, write - 1..3
	if (len > 1) {
		spin_lock(&lock);
		for (i = 0; i <len; i++){
			x393_sensi2c_abs (data[i], chn, offs);
		}
	} else {
		x393_sensi2c_abs (data[0], chn, offs);
	}
	spin_unlock(&lock);
	return 0;
}

/* Write sensor 16 bit (or 8 bit as programmed in the table) data in immediate mode */
void  write_sensor_reg16  (int chn,
	 	                   int page, // page (8 bits)
                           int addr, // low 8 bits
	      				   u32 data){ // 16 or 8-bit data (LSB aligned)
    u32 dw = ((page & 0xff) << 24) | ((addr & 0xff) << 16) | (data & 0xffff);
	x393_sensi2c_rel (dw, chn, 0);
}

/* Initiate sensor i2c read in immediate mode (data itself has to be read from FIFO with read_sensor_i2c_fifo)*/
void  read_sensor_i2c  (int chn,
		               int page, // page (8 bits)
					   int sa7,  // 7-bit i2c slave address
                       int addr){ // 8/16 bit address
    u32 dw = ((page & 0xff) << 24)  | (sa7 << 17) | (addr & 0xffff);
	x393_sensi2c_rel (dw, chn, 0);
}

//void                         x393_sensi2c_rel                    (u32 d, int sens_num, int offset){writel(d, mmio_ptr + (0x1080 + 0x40 * sens_num + 0x1 * offset));} // Write sensor i2c sequencer

/* Read next byte from the channel i2c FIFO. Return byte or -1 if no data available */
/* Sensor channel status should be in auto update mode (3) */
int read_sensor_i2c_fifo(int chn){
	int fifo_lsb, rslt,i;
	x393_i2c_ctltbl_t i2c_cmd;
	x393_status_sens_i2c_t  status =  x393_sensi2c_status (chn);
	if (!status.i2c_fifo_nempty) return -1; // No data available
	fifo_lsb = status.i2c_fifo_lsb;
	rslt = status.i2c_fifo_dout;
	// Advance FIFO readout pointer
	i2c_cmd.d32 = 0;
	i2c_cmd.next_fifo_rd = 1; // tbl_mode is 0 already
	x393_sensi2c_ctrl (i2c_cmd, chn);
	for (i = 0; i < 10; i++) {
		status = x393_sensi2c_status(chn);
		if (likely(status.i2c_fifo_lsb != fifo_lsb)) break;
	}
	return rslt;
}
// ======================================
// SYSFS
/* Get channelo number from the last character of the attribute name*/
static int get_channel_from_name(struct device_attribute *attr){
	int reg = 0;
	sscanf(attr->attr.name + (strlen(attr->attr.name)-1), "%du", &reg);
	return reg;
}

static ssize_t get_i2c_page_alloc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn = get_channel_from_name(attr) ;
	int page;
	if (sysfs_page[chn]>=0)
		return -EBUSY;
	page= i2c_page_alloc(chn);
	if (page <0)
		return -ENOMEM;
	sysfs_page[chn] = page;
	return sprintf(buf,"%d\n",page);
}
static ssize_t free_i2c_page(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn = get_channel_from_name(attr) ;
	int page;
    sscanf(buf, "%i", &page);
    page &= 0xff;
	if (sysfs_page[chn] >= 0)
		i2c_page_free(chn, page);
	    sysfs_page[chn] = -1; // free
	return count;
}

static ssize_t get_i2c_page_inuse(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn = get_channel_from_name(attr) ;
	return sprintf(buf,"%d\n",sysfs_page[chn]);
}
// Get i2c table data as raw data (hex)
static ssize_t get_i2c_tbl_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn = get_channel_from_name(attr) ;
	int page = sysfs_page[chn]; // currently selected page for sysfs reads
	if (page < 0)
		return -ENXIO;   /* No such device or address */

	return sprintf(buf,"%0x08x\n",i2c_pages_shadow[(chn << 8) + (page &0xff)]);
}

static ssize_t set_i2c_tbl_raw(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn = get_channel_from_name(attr) ;
	int page, data;
    sscanf(buf, "%i %i", &page, &data);
    page &= 0xff;
    set_sensor_i2c_raw(chn,
    		           page,   // index in lookup table
    				   (u32) data); // Bit delay - number of mclk periods in 1/4 of the SCL period
	return count;
}

// Sysfs top

static DEVICE_ATTR(alloc0  ,       SYSFS_PERMISSIONS & SYSFS_READONLY,    get_i2c_page_alloc ,   NULL);
static DEVICE_ATTR(alloc1  ,       SYSFS_PERMISSIONS & SYSFS_READONLY,    get_i2c_page_alloc ,   NULL);
static DEVICE_ATTR(alloc2  ,       SYSFS_PERMISSIONS & SYSFS_READONLY,    get_i2c_page_alloc ,   NULL);
static DEVICE_ATTR(alloc3  ,       SYSFS_PERMISSIONS & SYSFS_READONLY,    get_i2c_page_alloc ,   NULL);
static DEVICE_ATTR(page_in_use0 ,  SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,   free_i2c_page);
static DEVICE_ATTR(page_in_use1 ,  SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,   free_i2c_page);
static DEVICE_ATTR(page_in_use2 ,  SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,   free_i2c_page);
static DEVICE_ATTR(page_in_use3 ,  SYSFS_PERMISSIONS                 ,    get_i2c_page_inuse,   free_i2c_page);
static DEVICE_ATTR(tbl_raw0 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,      set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw1 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,      set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw2 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,      set_i2c_tbl_raw);
static DEVICE_ATTR(tbl_raw3 ,      SYSFS_PERMISSIONS                 ,    get_i2c_tbl_raw,      set_i2c_tbl_raw);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_alloc0.attr,
        &dev_attr_alloc1.attr,
        &dev_attr_alloc2.attr,
        &dev_attr_alloc3.attr,
        &dev_attr_page_in_use0.attr,
        &dev_attr_page_in_use1.attr,
        &dev_attr_page_in_use2.attr,
        &dev_attr_page_in_use3.attr,
        &dev_attr_tbl_raw0.attr,
        &dev_attr_tbl_raw1.attr,
        &dev_attr_tbl_raw2.attr,
        &dev_attr_tbl_raw3.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static int elphel393_sens_i2c_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
	}
	return retval;
}


// =======================================
static void elphel393_sensor_i2c_init_of(struct platform_device *pdev)
{
	const __be32 * config_data;
    const   char * config_string;
	char str[40];
	int len,chn,pre_disabled,old_dis_por,rc,chn_bits;
	struct device_node *node = pdev->dev.of_node;
//	struct elphel393_pwr_data_t *clientdata = platform_get_drvdata(pdev);
//	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);

	if (node) {
		/*TODO: Configure some i2c devices here (slaves, formats, speeds) to be used by names*/
	}
	dev_info(&pdev->dev,"elphel393_sensor_i2c configuration done\n");
}


static int elphel393_sensor_i2c_probe(struct platform_device *pdev)
{
	/*
	struct gpio_chip *chip;
//	struct device * ltc3489_dev;
	int i,rc;
	int base[2];
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata = NULL;
	*/
	dev_info(&pdev->dev,"Probing elphel393-sensor-i2c\n");
	i2c_page_alloc_init();
#if 0
	clientdata = devm_kzalloc(&pdev->dev, sizeof(*clientdata), GFP_KERNEL);
#endif
	elphel393_sensor_i2c_init_of(pdev);
	elphel393_sens_i2c_sysfs_register(pdev);
	return 0;
}

static int elphel393_sensor_i2c_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,"Removing elphel393-sensor-i2c");
	return 0;
}

static struct of_device_id elphel393_sensor_i2c_of_match[] = {
	{ .compatible = "elphel,elphel393-sensor-i2c-1.00", },
	{ /* end of table */}
};

MODULE_DEVICE_TABLE(of, elphel393_sensor_i2c_of_match);

static struct platform_driver elphel393_pwr = {
	.probe   = elphel393_sensor_i2c_probe,
	.remove  = elphel393_sensor_i2c_remove,
	.driver  = {
		.name  = "elphel393-sensor-i2c",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_sensor_i2c_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_pwr);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("Elphel 10393 sensor ports i2c");
MODULE_LICENSE("GPL");
