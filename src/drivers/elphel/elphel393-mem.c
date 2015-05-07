#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
#include <asm/dma-mapping.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222


static ssize_t get_paddr(struct device *dev, struct device_attribute *attr, char *buf);

struct elphel_buf_t
{
	void *vaddr;
	dma_addr_t paddr;
	ssize_t size;
};

static struct elphel_buf_t elphel_buf = {
	.vaddr = NULL,
	.paddr = 0,
	.size = 0
};

static int __init elphelmem_init(void)
{
    struct device_node *node;
	const __be32 *bufsize_be;

	printk("======== Allocating memory buffer ========\n");

	node = of_find_node_by_name(NULL, "elphel393-mem");
	if (!node)
	{
		printk("ERROR: No node found\n");
		return -ENODEV;
	}

	bufsize_be = (__be32 *)of_get_property(node, "memsize", NULL);
	elphel_buf.size = be32_to_cpup(bufsize_be);

	elphel_buf.vaddr = dma_alloc_coherent(NULL,(elphel_buf.size*PAGE_SIZE),&(elphel_buf.paddr),GFP_KERNEL);

    if(elphel_buf.paddr)
    {
    	printk("Allocated %u pages at address %x\n", (u32)elphel_buf.size, (u32)elphel_buf.paddr);
    }
    else printk("ERROR allocating memory buffer");

    return 0;
}

static void __exit elphelmem_exit(void)
{
    printk("Goodbye Cruel World!\n");
}



// SYSFS

static ssize_t get_paddr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%x\n", (u32)elphel_buf.paddr);
}

static ssize_t get_size(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", elphel_buf.size);
}
static ssize_t get_cache(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write in this file to flush L1 and L2 caches.\n");
}
static ssize_t flush_cache(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	__cpuc_flush_dcache_area(elphel_buf.vaddr,elphel_buf.size);
	outer_clean_range(elphel_buf.vaddr,elphel_buf.size);
	outer_cache.inv_range(elphel_buf.vaddr,elphel_buf.size);
	return count;
}
static DEVICE_ATTR(buffer_address,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_paddr,          NULL);
static DEVICE_ATTR(buffer_pages,  SYSFS_PERMISSIONS & SYSFS_READONLY,    get_size,          NULL);
static DEVICE_ATTR(buffer_flush,  SYSFS_PERMISSIONS,    get_cache,          flush_cache);

static struct attribute *root_dev_attrs[] = {
		&dev_attr_buffer_address.attr,
		&dev_attr_buffer_pages.attr,
		&dev_attr_buffer_flush.attr,
	    NULL
};

static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static int elphel393_mem_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
	}
	return retval;
}

static int elphel393_mem_probe(struct platform_device *pdev)
{
	elphel393_mem_sysfs_register(pdev);
	dev_info(&pdev->dev,"Probing elphel393-mem\n");
	return 0;
}

static int elphel393_mem_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,"Removing elphel393-mem");
	return 0;
}

static struct of_device_id elphel393_mem_of_match[] = {
	{ .compatible = "elphel,elphel393-mem-1.00", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, elphel393_pwr_of_match);

static struct platform_driver elphel393_mem = {
	.probe   = elphel393_mem_probe,
	.remove  = elphel393_mem_remove,
	.driver  = {
		.name  = "elphel393-mem",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_mem_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_mem);
module_init(elphelmem_init);
module_exit(elphelmem_exit);
MODULE_LICENSE("GPL");


