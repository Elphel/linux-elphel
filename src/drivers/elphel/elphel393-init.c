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
#define pr_fmt(fmt) "elphel393-init: " fmt

#include <asm/page.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
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
static char *bootargs;
static struct mtd_info *mtd;

static char *boardinfo;

static char serial[13];
//strncpy(serial,buf,sizeof(serial)-1)
//static char *serial;

static char revision[8];
//static char *revision;

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

static DEVICE_ATTR(bootargs  ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_bootargs ,   NULL);
static DEVICE_ATTR(board_info,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_boardinfo,   NULL);
static DEVICE_ATTR(revision  ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_revision ,   NULL);
static DEVICE_ATTR(serial    ,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_serial ,     NULL);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_bootargs.attr,
        &dev_attr_board_info.attr,
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
	u8 devnum=-1;
	u8 unlock= 0;

	size_t retlen;
	//size of nand flash page
	char kbuf[2048];
	int page_limit = 3;
	size_t count;
	size_t size;
	int len;
	loff_t *ppos;
	loff_t pos;
	int ret;
	char *ps,*pe;

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
	if (unlock&&(devnum>=0)){
		mtd = get_mtd_device(NULL,devnum);
		mtd_unlock(mtd,0,mtd->size);
		pr_info("/dev/mtd%d: unlocked",devnum);
	}else{
		mtd = get_mtd_device(NULL,0);
	}

	if (!mtd){
		pr_err("Get mtd device error\n");
		return mtd;
	}

	// read boardinfo record
	// * device number is not important
	// * no need to unlock

	// page size
	BUG_ON(mtd->writesize > 2048);

	size = mtd->writesize;
	// search within page_limit pages area
	count = page_limit*(mtd->writesize);
	// starting offset
	pos = 0;
	ppos = &pos;

	// memory for boardinfo
	boardinfo = kmalloc(size,GFP_KERNEL);
	if (!boardinfo)
		return -ENOMEM;

	while(count){
		len = min_t(size_t,count, size);
		ret = mtd_read_user_prot_reg(mtd, *ppos, len, &retlen, kbuf);
		if (ret){
			pr_err("flash page read, code %d",ret);
		}
		// do whatever we like with the kbuf
		// search for "<board>"
		// expecting to find it somewhere...
		if(strnstr(kbuf,"<board>",size)){
			//...right in the beginning or error
			ps = strnstr(kbuf,"<serial>",size);
			pe = strnstr(kbuf,"</serial>",size);
			strncpy(serial,ps+sizeof("<serial>")-1,pe-ps-(sizeof("<serial>")-1));
			ps = strnstr(kbuf,"<rev>",size);
			pe = strnstr(kbuf,"</rev>",size);
			strncpy(revision,ps+sizeof("<rev>")-1,pe-ps-(sizeof("<rev>")-1));
			strncpy(boardinfo,kbuf,retlen);
			break;
		}
		*ppos += retlen;
		count -= retlen;
	}
	elphel393_init_sysfs_register(pdev);
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
		.name  = "elphel393-init",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_init_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_initialize);
module_init(elphel393_init_init);
module_exit(elphel393_init_exit);
MODULE_LICENSE("GPL");
