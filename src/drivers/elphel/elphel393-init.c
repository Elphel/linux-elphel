/*!***************************************************************************
 *! FILE NAME  : elphel393-init.c
 *! DESCRIPTION: * Unlock rootfs NAND flash partition
 *!              * Read MAC and other useful info from NAND flash OTP area
 *!                and put to sysfs
 *!
 *! E-mail: oleg@elphel.com, support-list@elphel.com
 *!
 *! Copyright (C) 2016 Elphel, Inc.
 *! -----------------------------------------------------------------------------**
 *!
 *!  This program is free software: you can redistribute it and/or modify
 *!  it under the terms of the GNU General Public License as published by
 *!  the Free Software Foundation, either version 3 of the License, or
 *!  (at your option) any later version.
 *!
 *!  This program is distributed in the hope that it will be useful,
 *!  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *!  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *!  GNU General Public License for more details.
 *!
 *!  You should have received a copy of the GNU General Public License
 *!  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *!****************************************************************************/

#define DRV_NAME "elphel393-init"
#define pr_fmt(fmt) DRV_NAME": " fmt

#include <asm/page.h>

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <elphel/elphel393-init.h>

/* from elphel393-mem.c */
#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

/*
 * Read and parse bootargs parameter in the device tree
 * This driver is run in the last place - at least after NAND driver is probed
 */
static struct mtd_info *mtd;

//surprise size
static char *bootargs;
//known size
static char boardinfo[2048];
static char serial[13];
static char revision[8];

/*
static int __init elphel393_early_initialize(void){

	struct device_node *node;
	struct property *newproperty;
	u8 *macaddr;

	pr_info("VERY EARLY CALL TO UPDATE DEVICE TREE");

	node = of_find_node_by_name(NULL, "ps7-ethernet");
	if (!node){
		pr_err("Device tree node 'ps7-ethernet' not found.");
		return -ENODEV;
	}

	newproperty = kzalloc(sizeof(*newproperty) + 6, GFP_KERNEL);
	if (!newproperty)
		return -ENOMEM;
	newproperty->value = newproperty + 1;
	newproperty->length = 6;
	newproperty->name = kstrdup("local-mac-address", GFP_KERNEL);
	if (!newproperty->name) {
		kfree(newproperty);
		return -ENOMEM;
	}
	macaddr = newproperty->value;
	macaddr[0] = 0x02;
	macaddr[1] = 0x03;
	macaddr[2] = 0x04;
	macaddr[3] = 0x05;
	macaddr[4] = 0x06;
	macaddr[5] = 0x07;

	of_update_property(node,newproperty);
	return 0;
}
*/
static int __init elphel393_init_init (void)
{
	struct device_node *node;

	node = of_find_node_by_name(NULL, "chosen");
	//just throw an error
	if (!node){
		pr_err("Device tree node 'chosen' not found.");
		return -ENODEV;
	}
	of_property_read_string(node, "bootargs", &bootargs);
	if (bootargs!=NULL) {
		pr_debug("bootargs line from device tree is %s",bootargs);
	}
	return 0;
}

static void __exit elphel393_init_exit(void)
{
    printk("Exit\n");
}

// SYSFS

static ssize_t get_bootargs(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n",bootargs);
}
static ssize_t get_boardinfo(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n",boardinfo);
}
static ssize_t get_revision(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n",revision);
}
static ssize_t get_serial(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n",serial);
}

static int get_factory_info(void);

static ssize_t set_boardinfo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	loff_t pos=0;//second page
	loff_t *ppos=&pos;
	size_t retlen;

	char wbuf[2048];
	memset(wbuf,0xff,2048);

	if (!mtd){
		pr_err("Get MTD device error\n");
		return mtd;
	}
	//too much of a trouble to read from flash again
	if(!strnstr(boardinfo,"<board>",sizeof(boardinfo))){
		pr_info("Factory Info record is clean.\n");
		pr_info("Data to be written: %s",buf);
		// I got some buf, unknown size- should be limited to 2048? ok
		if (strlen(buf)>2048) {
			pr_err("Data > 2KiB. Abort.\n");
			return -1;
		}
		// Not too strict check, just look for opening tags.
		if(!strnstr(buf,"<board>",2048)||!strnstr(buf,"<serial>",2048)||!strnstr(buf,"<rev>",2048)){
			pr_err("Bad data format\n");
			return -2;
		}
		//tmp disabled.
		//ret = mtd_write_user_prot_reg(mtd, *ppos+4*2048, 2048, &retlen, buf);

		//copy to buf
		strncpy(wbuf,buf,strlen(buf));
		pr_info("BUFFER: %s\n",wbuf);
		ret = mtd_write_user_prot_reg(mtd, *ppos+4*2048, 2048, &retlen, wbuf);
		if (ret){
			pr_err("Flash page write, code %d",ret);
			return ret;
		}
		pr_info("Data is successfully written and cannot be overwritten anymore, record size: %d B\n",strlen(wbuf));
		get_factory_info();
	}else{
		pr_info("Factory Info record (serial='%s' revision='%s') can not be overwritten\n",serial,revision);
	}
    return count;
}

static DEVICE_ATTR(bootargs  ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_bootargs ,   NULL);
static DEVICE_ATTR(boardinfo ,  SYSFS_PERMISSIONS                 ,    get_boardinfo,   set_boardinfo);
static DEVICE_ATTR(revision  ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_revision ,   NULL);
static DEVICE_ATTR(serial    ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_serial ,     NULL);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_bootargs.attr,
        &dev_attr_boardinfo.attr,
        &dev_attr_revision.attr,
        &dev_attr_serial.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static int elphel393_init_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
	}
	return retval;
}

static int elphel393_init_probe(struct platform_device *pdev)
{
	char *bootargs_copy;

	char *token;
	char *token_copy;

	char *param;
	char *value;

	//mtd device number to unlock
	u8 devnum= 0;
	u8 unlock= 0;

	pr_debug("Probing");
	//copy bootargs string
	bootargs_copy = kstrdup(bootargs,GFP_KERNEL);
	//find out which partition to unlock if any
	//parse bootargs
	do {
		token = strsep(&bootargs_copy," ");
		if (token) {
			//if '=' is found - split by '='
			token_copy = token;
			if (strchr(token_copy,'=')){
				if (!strcmp(token_copy,"rootfstype=ubifs")){
					unlock=1;
				}
				param = strsep(&token_copy,"=");
	            //if "ubi.mtd" then get the partition number and unlock /dev/mtdN - if not found then don't care
				if (!strcmp(param,"ubi.mtd")){
					value = strsep(&token_copy,",");
					if (kstrtou8(value,10,&devnum)){
						pr_err("Partition number str to u8 conversion.");
					}
				}
			}
		}
	} while (token);

	kfree(bootargs_copy);

	// unlock /dev/mtdN partition
	// if there's no need to unlock, get /dev/mtd0
	if (devnum>=0){
		mtd = get_mtd_device(NULL,devnum);
		if (!mtd){
			pr_err("Get MTD device error\n");
			return mtd;
		}
		if (unlock){
			mtd_unlock(mtd,0,mtd->size);
			pr_info("/dev/mtd%d: unlocked",devnum);
		}
	}
	// read boardinfo record
	// * device number is not important
	// * no need to unlock

	// page size
	BUG_ON(mtd->writesize > 2048);

	get_factory_info();
	elphel393_init_sysfs_register(pdev);
	return 0;
}

static int get_factory_info(void){

	char regvalh[5];//char reg[]="0000";
	char regvall[9];
	u16 hwaddrh;
	u32 hwaddrl;

	size_t retlen;
	//size of nand flash page
	char kbuf[2048];
	size_t size = mtd->writesize;
	int ret;
	char *ps,*pe;

//	const u8 *mac_address;
	struct device_node *node;
	struct property *newproperty;

	u32 *mac32; //  = (u32*) mac_address;
	u8 *macaddr;

	// I expected to have null terminated strings
	// but cannot get it from the declaration. Don't know.
	memset(regvalh,0x00,5);
	memset(regvall,0x00,9);

	ret = mtd_read_user_prot_reg(mtd, 4*2048, size, &retlen, kbuf);
	if (ret){
		pr_err("Flash page read, code %d",ret);
		return ret;
	}
	// do whatever we like with the kbuf
	// search for "<board>"
	// expecting to find it somewhere...
	if(strnstr(kbuf,"<board>",size)){
		//...right in the beginning or error
		ps = strnstr(kbuf,"<serial>",size);
		pe = strnstr(kbuf,"</serial>",size);
		strncpy(serial,ps+sizeof("<serial>")-1,pe-ps-(sizeof("<serial>")-1));

		strncpy(regvalh,serial,4);
		strncpy(regvall,serial+4,8);

		//there is kstrtou64 but it doesn't work?
		kstrtou16(regvalh,16,&hwaddrh);
		kstrtou32(regvall,16,&hwaddrl);

		node = of_find_node_by_name(NULL, "ps7-ethernet");
		if (!node){
			pr_err("Device tree node 'ps7-ethernet' not found.");
			return -ENODEV;
		}

		pr_debug("MAC from flash: %02x:%02x:%02x:%02x:%02x:%02x\n",
			 (hwaddrl & 0xff), ((hwaddrl >> 8) & 0xff),
			((hwaddrl >> 16) & 0xff), (hwaddrl >> 24),
			 (hwaddrh & 0xff), (hwaddrh >> 8));

		newproperty = kzalloc(sizeof(*newproperty) + 6, GFP_KERNEL);
		if (!newproperty)
			return -ENOMEM;
		newproperty->value = newproperty + 1;
		newproperty->length = 6;
		newproperty->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newproperty->name) {
			kfree(newproperty);
			return -ENOMEM;
		}
		macaddr = newproperty->value;
		macaddr[0] = (hwaddrh >> 8) & 0xff;
		macaddr[1] =  hwaddrh & 0xff;
		macaddr[2] = (hwaddrl >> 24) & 0xff;
		macaddr[3] = (hwaddrl >> 16) & 0xff;
		macaddr[4] = (hwaddrl >> 8) & 0xff;
		macaddr[5] =  hwaddrl & 0xff;

		of_update_property(node,newproperty);

		mac32 = (u32 *) of_get_mac_address(node);
		hwaddrl = cpu_to_le32(mac32[0]);
		hwaddrh = cpu_to_le16(mac32[1]);

		pr_debug("MAC from device tree: %02x:%02x:%02x:%02x:%02x:%02x\n",
			(hwaddrl & 0xff), ((hwaddrl >> 8) & 0xff),
			((hwaddrl >> 16) & 0xff), (hwaddrl >> 24),
			(hwaddrh & 0xff), (hwaddrh >> 8));

		//write hwaddr to zynq reg
		//kstrtou16(serial,16,&regvalh);
		//serial

		ps = strnstr(kbuf,"<rev>",size);
		pe = strnstr(kbuf,"</rev>",size);
		strncpy(revision,ps+sizeof("<rev>")-1,pe-ps-(sizeof("<rev>")-1));
		ps = strnstr(kbuf,"<board>",size);
		pe = strnstr(kbuf,"</board>",size);
		strncpy(boardinfo,ps,pe-ps+(sizeof("</board>")-1));
	}
	return 0;
}

static int elphel393_init_remove(struct platform_device *pdev)
{
	pr_info("Remove");
	return 0;
}

static struct of_device_id elphel393_init_of_match[] = {
	{ .compatible = "elphel,elphel393-init-1.00", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, elphel393_init_of_match);

static struct platform_driver elphel393_initialize = {
	.probe   = elphel393_init_probe,
	.remove  = elphel393_init_remove,
	.driver  = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = elphel393_init_of_match,
		.pm = NULL, /* power management */
	},
};

//early_initcall(elphel393_early_initialize);

module_platform_driver(elphel393_initialize);
module_init(elphel393_init_init);
module_exit(elphel393_init_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Unlock rootfs flash partition and read/write board info: serial and revision");
