/*!***********************************************************************//**
 * @file   elphel393-mem.c
 * @brief  Reserve large memory range at boot time (when it is available)
 *              to use as a circular video buffer
 * @copyright Copyright (C) 2015 Elphel, Inc.
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
 *****************************************************************************/

//#define DEBUG
#define DRV_NAME "elphel393-mem"
#define pr_fmt(fmt) DRV_NAME": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/slab.h> // kmalloc
#include <linux/vmalloc.h> // vmalloc
#include <linux/gfp.h> // __get_free_pages
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
#include <asm/dma-mapping.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

#include <elphel/elphel393-mem.h>

#include <uapi/elphel/c313a.h>

#include "x393_macro.h"

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

static ssize_t get_paddr(struct device *dev, struct device_attribute *attr, char *buf);

static struct elphel_buf_t _elphel_buf = {

	// Coherent DMA buffer
	.vaddr = NULL,
	.paddr = 0,
	.size = 0,

	// Host to device stream DMA buffer
	.h2d_vaddr = NULL,
	.h2d_paddr = 0,
	.h2d_size = 1024, // TODO: add to DT, learn how to allocate more

	// Device to host stream DMA buffer
	.d2h_vaddr = NULL,
	.d2h_paddr = 0,
	.d2h_size = 1024,

	// Bidirectional stream DMA buffer
	.bidir_vaddr = NULL,
	.bidir_paddr = 0,
	.bidir_size = 1024,

    // Device to host stream DMA buffer for histograms
    .histograms_vaddr = NULL,
    .histograms_paddr = 0,
    .histograms_size = 1024,

    // Device to host stream DMA buffer for the logger
    .logger_vaddr = NULL,
    .logger_paddr = 0,
    .logger_size = 1024, // should be 2**n !

	// circbuf channel 0 (in Coherent DMA buffer)
	.circbuf_chn0_vaddr = NULL,
	.circbuf_chn0_paddr = 0,
	.circbuf_chn0_size = 16384,

	// circbuf channel 1 (in Coherent DMA buffer)
	.circbuf_chn1_vaddr = NULL,
	.circbuf_chn1_paddr = 0,
	.circbuf_chn1_size = 16384,

	// circbuf channel 2 (in Coherent DMA buffer)
	.circbuf_chn2_vaddr = NULL,
	.circbuf_chn2_paddr = 0,
	.circbuf_chn2_size = 16384,

	// circbuf channel 3 (in Coherent DMA buffer)
	.circbuf_chn3_vaddr = NULL,
	.circbuf_chn3_paddr = 0,
	.circbuf_chn3_size = 16384,

	// raw channel 0 (in Coherent DMA buffer)
	.raw_chn0_vaddr = NULL,
	.raw_chn0_paddr = 0,
	.raw_chn0_size = 0,

	// raw channel 1 (in Coherent DMA buffer)
	.raw_chn1_vaddr = NULL,
	.raw_chn1_paddr = 0,
	.raw_chn1_size = 0,

	// raw channel 2 (in Coherent DMA buffer)
	.raw_chn2_vaddr = NULL,
	.raw_chn2_paddr = 0,
	.raw_chn2_size = 0,

	// raw channel 3 (in Coherent DMA buffer)
	.raw_chn3_vaddr = NULL,
	.raw_chn3_paddr = 0,
	.raw_chn3_size = 0

};

struct elphel_buf_t *pElphel_buf; // static can not be extern
EXPORT_SYMBOL_GPL(pElphel_buf);

static int elphelmem_of_get_init_data(void){

	struct device_node *node;

	int parts[4];

	node = of_find_node_by_name(NULL, "elphel393-mem");
	if (!node) {
		pr_err("DMA buffer allocation ERROR: No device tree node found\n");
		return -ENODEV;
	}

	of_property_read_u32(node, "memsize",&_elphel_buf.size);
	of_property_read_u32(node, "h2d_size",&_elphel_buf.h2d_size);
	of_property_read_u32(node, "d2h_size",&_elphel_buf.d2h_size);
	of_property_read_u32(node, "bidir_size",&_elphel_buf.bidir_size);
	of_property_read_u32(node, "histograms_size",&_elphel_buf.histograms_size);
	of_property_read_u32(node, "logger_size",&_elphel_buf.logger_size);

	if (of_property_read_u32_array(node,
			                       "memsize-partitions-circbuf",
								   parts,
								   ARRAY_SIZE(parts)) >= 0)
	{
		_elphel_buf.circbuf_chn0_size = parts[0];
		_elphel_buf.circbuf_chn1_size = parts[1];
		_elphel_buf.circbuf_chn2_size = parts[2];
		_elphel_buf.circbuf_chn3_size = parts[3];
	}else{
		pr_info("circbufs sizes are not defined in device tree, will use defaults\n");
	}

	if (of_property_read_u32_array(node,
			                       "memsize-partitions-raw",
								   parts,
								   ARRAY_SIZE(parts)) >= 0)
	{
		_elphel_buf.raw_chn0_size = parts[0];
		_elphel_buf.raw_chn1_size = parts[1];
		_elphel_buf.raw_chn2_size = parts[2];
		_elphel_buf.raw_chn3_size = parts[3];
	}else{
		pr_info("raw buffers sizes are not defined in device tree, will use defaults\n");
	}

	return 0;
}

int elphelmem_update_partitions(void){

	dma_addr_t tmpsize=0;

	////from circbuf.c
	//...
	////nobody knows what the 1st 1MB is for...
	//circbuf_priv[i].buf_ptr = dma_buf_ptr + BYTE2DW(CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE);
	//circbuf_priv[i].phys_addr = dma_handle + CIRCBUF_START_OFFSET + i * CCAM_DMA_SIZE;
	//circbuf_priv[i].buf_size = CCAM_DMA_SIZE;
	//...

	tmpsize = CIRCBUF_START_OFFSET;

	// fill out buffers info
	_elphel_buf.circbuf_chn0_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.circbuf_chn0_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.circbuf_chn0_size*PAGE_SIZE;

	_elphel_buf.circbuf_chn1_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.circbuf_chn1_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.circbuf_chn1_size*PAGE_SIZE;

	_elphel_buf.circbuf_chn2_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.circbuf_chn2_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.circbuf_chn2_size*PAGE_SIZE;

	_elphel_buf.circbuf_chn3_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.circbuf_chn3_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.circbuf_chn3_size*PAGE_SIZE;

	_elphel_buf.raw_chn0_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.raw_chn0_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.raw_chn0_size*PAGE_SIZE;

	_elphel_buf.raw_chn1_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.raw_chn1_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.raw_chn1_size*PAGE_SIZE;

	_elphel_buf.raw_chn2_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.raw_chn2_paddr = _elphel_buf.paddr+tmpsize;

	tmpsize += _elphel_buf.raw_chn2_size*PAGE_SIZE;

	_elphel_buf.raw_chn3_vaddr = _elphel_buf.vaddr+tmpsize;
	_elphel_buf.raw_chn3_paddr = _elphel_buf.paddr+tmpsize;

	return 0;
}
EXPORT_SYMBOL_GPL(elphelmem_update_partitions);

static int __init elphelmem_init(void)
{
    struct device_node *node;
	const __be32 *bufsize_be;
	pElphel_buf = &_elphel_buf;

	node = of_find_node_by_name(NULL, "elphel393-mem");
	if (!node)
	{
		pr_err("DMA buffer allocation ERROR: No device tree node found\n");
		return -ENODEV;
	}

	elphelmem_of_get_init_data();

	//bufsize_be = (__be32 *)of_get_property(node, "memsize", NULL);
	//_elphel_buf.size = be32_to_cpup(bufsize_be);

	_elphel_buf.vaddr = dma_alloc_coherent(NULL,(_elphel_buf.size*PAGE_SIZE),&(_elphel_buf.paddr),GFP_KERNEL);
    if(_elphel_buf.vaddr) {
    	pr_info("Allocated %u pages for DMA at address 0x%x\n", (u32)_elphel_buf.size, (u32)_elphel_buf.paddr);

    	elphelmem_update_partitions();

    } else {
    	pr_err("ERROR allocating coherent DMA memory buffer\n");
    }

    _elphel_buf.h2d_vaddr = kzalloc((_elphel_buf.h2d_size*PAGE_SIZE) ,GFP_KERNEL);
    if (!_elphel_buf.h2d_vaddr){
    	_elphel_buf.h2d_size = 0;
    	pr_err("ERROR allocating H2D DMA memory buffer\n");
    }

    _elphel_buf.d2h_vaddr = kzalloc((_elphel_buf.d2h_size*PAGE_SIZE) ,GFP_KERNEL);
    if (!_elphel_buf.d2h_vaddr){
    	_elphel_buf.d2h_size = 0;
    	pr_err("ERROR allocating D2H DMA memory buffer\n");
    }

    _elphel_buf.histograms_vaddr = kzalloc((_elphel_buf.histograms_size*PAGE_SIZE) ,GFP_KERNEL);
    if (!_elphel_buf.histograms_vaddr){
        _elphel_buf.histograms_size = 0;
        pr_err("ERROR allocating HISTOGRAMS memory buffer\n");
    }

    _elphel_buf.logger_vaddr = kzalloc((_elphel_buf.logger_size*PAGE_SIZE) ,GFP_KERNEL);
    if (!_elphel_buf.logger_vaddr){
        _elphel_buf.logger_size = 0;
        pr_err("ERROR allocating LOGGER memory buffer\n");
    }

    _elphel_buf.bidir_vaddr = kzalloc((_elphel_buf.bidir_size*PAGE_SIZE) ,GFP_KERNEL);
    if (!_elphel_buf.bidir_vaddr){
    	_elphel_buf.bidir_size = 0;
    	pr_err("ERROR allocating Bidirectional DMA memory buffer\n");
    }

    pr_info("Coherent buffer vaddr:              0x%08X\n",(u32) pElphel_buf -> vaddr);
    pr_info("Coherent buffer paddr:              0x%08X\n",(u32) pElphel_buf -> paddr);
    pr_info("Coherent buffer length:             0x%08X\n",(u32) pElphel_buf -> size * PAGE_SIZE);

    pr_info("    circbuf channel 0 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> circbuf_chn0_vaddr,
			(u32) pElphel_buf -> circbuf_chn0_paddr,
			(u32) pElphel_buf-> circbuf_chn0_size * PAGE_SIZE);

    pr_info("    circbuf channel 1 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> circbuf_chn1_vaddr,
			(u32) pElphel_buf -> circbuf_chn1_paddr,
			(u32) pElphel_buf-> circbuf_chn1_size * PAGE_SIZE);

    pr_info("    circbuf channel 2 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> circbuf_chn2_vaddr,
			(u32) pElphel_buf -> circbuf_chn2_paddr,
			(u32) pElphel_buf-> circbuf_chn2_size * PAGE_SIZE);

    pr_info("    circbuf channel 3 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> circbuf_chn3_vaddr,
			(u32) pElphel_buf -> circbuf_chn3_paddr,
			(u32) pElphel_buf-> circbuf_chn3_size * PAGE_SIZE);

    pr_info("        raw channel 0 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> raw_chn0_vaddr,
			(u32) pElphel_buf -> raw_chn0_paddr,
			(u32) pElphel_buf-> raw_chn0_size * PAGE_SIZE);

    pr_info("        raw channel 1 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> raw_chn1_vaddr,
			(u32) pElphel_buf -> raw_chn1_paddr,
			(u32) pElphel_buf-> raw_chn1_size * PAGE_SIZE);

    pr_info("        raw channel 2 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> raw_chn2_vaddr,
			(u32) pElphel_buf -> raw_chn2_paddr,
			(u32) pElphel_buf-> raw_chn2_size * PAGE_SIZE);

    pr_info("        raw channel 3 vaddr: 0x%08X paddr: 0x%08X length 0x%08X\n",
			(u32) pElphel_buf -> raw_chn3_vaddr,
			(u32) pElphel_buf -> raw_chn3_paddr,
			(u32) pElphel_buf-> raw_chn3_size * PAGE_SIZE);

    return 0;
}

static void __exit elphelmem_exit(void)
{
	pr_info("DMA buffer disabled\n");
}


static ssize_t get_paddr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"0x%x\n", (u32)_elphel_buf.paddr);
}

static ssize_t get_size(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", _elphel_buf.size);
}

static ssize_t get_paddr_h2d(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"0x%x\n", (u32)_elphel_buf.h2d_paddr);
}

static ssize_t get_size_h2d(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", _elphel_buf.h2d_size);
}

static ssize_t get_paddr_d2h(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"0x%x\n", (u32)_elphel_buf.d2h_paddr);
}

static ssize_t get_size_d2h(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", _elphel_buf.d2h_size);
}

static ssize_t get_paddr_bidir(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"0x%x\n", (u32)_elphel_buf.bidir_paddr);
}

static ssize_t get_size_bidir(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", _elphel_buf.bidir_size);
}

static ssize_t get_paddr_histograms(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.histograms_paddr);
}

static ssize_t get_size_histograms(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.histograms_size);
}
static ssize_t get_paddr_logger(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.logger_paddr);
}

static ssize_t get_size_logger(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.logger_size);
}

static ssize_t get_paddr_circbuf_chn0(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.circbuf_chn0_paddr);
}

static ssize_t get_size_circbuf_chn0(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.circbuf_chn0_size);
}

static ssize_t set_size_circbuf_chn0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.circbuf_chn0_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_circbuf_chn1(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.circbuf_chn1_paddr);
}

static ssize_t get_size_circbuf_chn1(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.circbuf_chn1_size);
}

static ssize_t set_size_circbuf_chn1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.circbuf_chn1_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_circbuf_chn2(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.circbuf_chn2_paddr);
}

static ssize_t get_size_circbuf_chn2(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.circbuf_chn2_size);
}

static ssize_t set_size_circbuf_chn2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.circbuf_chn2_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_circbuf_chn3(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.circbuf_chn3_paddr);
}

static ssize_t get_size_circbuf_chn3(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.circbuf_chn3_size);
}

static ssize_t set_size_circbuf_chn3(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.circbuf_chn3_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_raw_chn0(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.raw_chn0_paddr);
}

static ssize_t get_size_raw_chn0(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.raw_chn0_size);
}

static ssize_t set_size_raw_chn0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.raw_chn0_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_raw_chn1(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.raw_chn1_paddr);
}

static ssize_t get_size_raw_chn1(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.raw_chn1_size);
}

static ssize_t set_size_raw_chn1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.raw_chn1_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_raw_chn2(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.raw_chn2_paddr);
}

static ssize_t get_size_raw_chn2(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.raw_chn2_size);
}

static ssize_t set_size_raw_chn2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.raw_chn2_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t get_paddr_raw_chn3(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", (u32)_elphel_buf.raw_chn3_paddr);
}

static ssize_t get_size_raw_chn3(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n", _elphel_buf.raw_chn3_size);
}

static ssize_t set_size_raw_chn3(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (sscanf(buf, "%u", &val)>0){
		_elphel_buf.raw_chn3_size = val;
	    elphelmem_update_partitions();
	}
    return count;
}

static ssize_t sync_for_cpu_h2d(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.h2d_paddr;
		len = _elphel_buf.h2d_size * PAGE_SIZE;
	}
    printk("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_cpu(dev, paddr, len, DMA_TO_DEVICE);

    return count;

}
static ssize_t sync_for_device_h2d(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.h2d_paddr;
		len = _elphel_buf.h2d_size * PAGE_SIZE;
	}
    pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_device(dev, paddr, len, DMA_TO_DEVICE);

    return count;
}
static ssize_t sync_for_cpu_d2h(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.d2h_paddr;
		len = _elphel_buf.d2h_size * PAGE_SIZE;
	}
	pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_cpu(dev, paddr, len, DMA_FROM_DEVICE);

    return count;

}
static ssize_t sync_for_device_d2h(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.d2h_paddr;
		len = _elphel_buf.d2h_size * PAGE_SIZE;
	}
	pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_device(dev, paddr, len, DMA_FROM_DEVICE);

    return count;
}

static ssize_t sync_for_cpu_bidir(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.bidir_paddr;
		len = _elphel_buf.bidir_size * PAGE_SIZE;
	}
	pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_cpu(dev, paddr, len, DMA_BIDIRECTIONAL);

    return count;

}
static ssize_t sync_for_device_bidir(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	dma_addr_t paddr;
	size_t len;
	int num_items;
	num_items=sscanf(buf, "%u %u", &paddr, &len);
	if (num_items<2) {
		paddr =  _elphel_buf.bidir_paddr;
		len = _elphel_buf.bidir_size * PAGE_SIZE;
	}
	pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_device(dev, paddr, len, DMA_BIDIRECTIONAL);

    return count;
}



static ssize_t sync_for_cpu_histograms(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    dma_addr_t paddr;
    size_t len;
    int num_items;
    num_items=sscanf(buf, "%u %u", &paddr, &len);
    if (num_items<2) {
        paddr =  _elphel_buf.histograms_paddr;
        len = _elphel_buf.histograms_size * PAGE_SIZE;
    }
    pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_cpu(dev, paddr, len, DMA_FROM_DEVICE);

    return count;

}
static ssize_t sync_for_device_histograms(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    dma_addr_t paddr;
    size_t len;
    int num_items;
    num_items=sscanf(buf, "%u %u", &paddr, &len);
    if (num_items<2) {
        paddr =  _elphel_buf.histograms_paddr;
        len = _elphel_buf.histograms_size * PAGE_SIZE;
    }
    pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_device(dev, paddr, len, DMA_FROM_DEVICE);

    return count;
}
static ssize_t sync_for_cpu_logger(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    dma_addr_t paddr;
    size_t len;
    int num_items;
    num_items=sscanf(buf, "%u %u", &paddr, &len);
    if (num_items<2) {
        paddr =  _elphel_buf.logger_paddr;
        len = _elphel_buf.logger_size * PAGE_SIZE;
    }
    pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_cpu(dev, paddr, len, DMA_FROM_DEVICE);

    return count;

}
static ssize_t sync_for_device_logger(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    dma_addr_t paddr;
    size_t len;
    int num_items;
    num_items=sscanf(buf, "%u %u", &paddr, &len);
    if (num_items<2) {
        paddr =  _elphel_buf.logger_paddr;
        len = _elphel_buf.logger_size * PAGE_SIZE;
    }
    pr_info("\naddr=0x%08x, size = 0x%08x\n", paddr, len);
    dma_sync_single_for_device(dev, paddr, len, DMA_FROM_DEVICE);

    return count;
}


static ssize_t get_sync_for_device_h2d(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the host to device DMA buffer to device (after CPU writes).\n");
}
static ssize_t get_sync_for_cpu_h2d(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the host to device DMA buffer to CPU (before CPU reads).\n");
}
static ssize_t get_sync_for_device_d2h(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the device to host DMA buffer to device (after CPU writes).\n");
}
static ssize_t get_sync_for_cpu_d2h(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the device to host DMA buffer to CPU (before CPU reads).\n");
}
static ssize_t get_sync_for_device_bidir(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the bidirectional DMA buffer to device (after CPU writes).\n");
}
static ssize_t get_sync_for_cpu_bidir(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Write address/length pair into this file to hand this region of the bidirectional DMA buffer to CPU (before CPU reads).\n");
}


static ssize_t get_sync_for_device_histograms(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Write address/length pair into this file to hand this region of the histograms buffer to device (after CPU writes).\n");
}
static ssize_t get_sync_for_cpu_histograms(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Write address/length pair into this file to hand this region of the histograms buffer to CPU (before CPU reads).\n");
}
static ssize_t get_sync_for_device_logger(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Write address/length pair into this file to hand this region of the logger buffer to device (after CPU writes).\n");
}
static ssize_t get_sync_for_cpu_logger(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Write address/length pair into this file to hand this region of the logger buffer to CPU (before CPU reads).\n");
}


static DEVICE_ATTR(buffer_address,            SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr,                      NULL);
static DEVICE_ATTR(buffer_pages,              SYSFS_PERMISSIONS & SYSFS_READONLY, get_size,                       NULL);
static DEVICE_ATTR(buffer_address_h2d,        SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_h2d,                  NULL);
static DEVICE_ATTR(buffer_pages_h2d,          SYSFS_PERMISSIONS & SYSFS_READONLY, get_size_h2d,                   NULL);
static DEVICE_ATTR(buffer_address_d2h,        SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_d2h,                  NULL);
static DEVICE_ATTR(buffer_pages_d2h,          SYSFS_PERMISSIONS & SYSFS_READONLY, get_size_d2h,                   NULL);
static DEVICE_ATTR(buffer_address_bidir,      SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_bidir,                NULL);
static DEVICE_ATTR(buffer_pages_bidir,        SYSFS_PERMISSIONS & SYSFS_READONLY, get_size_bidir,                 NULL);
static DEVICE_ATTR(buffer_address_histograms, SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_histograms,           NULL);
static DEVICE_ATTR(buffer_pages_histograms,   SYSFS_PERMISSIONS & SYSFS_READONLY, get_size_histograms,            NULL);
static DEVICE_ATTR(buffer_address_logger,     SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_logger,               NULL);
static DEVICE_ATTR(buffer_pages_logger,       SYSFS_PERMISSIONS & SYSFS_READONLY, get_size_logger,                NULL);
static DEVICE_ATTR(buffer_address_circbuf_chn0, SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_circbuf_chn0,       NULL);
static DEVICE_ATTR(  buffer_pages_circbuf_chn0, SYSFS_PERMISSIONS,                   get_size_circbuf_chn0,       set_size_circbuf_chn0);
static DEVICE_ATTR(buffer_address_circbuf_chn1, SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_circbuf_chn1,       NULL);
static DEVICE_ATTR(  buffer_pages_circbuf_chn1, SYSFS_PERMISSIONS,                   get_size_circbuf_chn1,       set_size_circbuf_chn1);
static DEVICE_ATTR(buffer_address_circbuf_chn2, SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_circbuf_chn2,       NULL);
static DEVICE_ATTR(  buffer_pages_circbuf_chn2, SYSFS_PERMISSIONS,                   get_size_circbuf_chn2,       set_size_circbuf_chn2);
static DEVICE_ATTR(buffer_address_circbuf_chn3, SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_circbuf_chn3,       NULL);
static DEVICE_ATTR(  buffer_pages_circbuf_chn3, SYSFS_PERMISSIONS,                   get_size_circbuf_chn3,       set_size_circbuf_chn3);
static DEVICE_ATTR(buffer_address_raw_chn0,     SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_raw_chn0,           NULL);
static DEVICE_ATTR(  buffer_pages_raw_chn0,     SYSFS_PERMISSIONS,                   get_size_raw_chn0,           set_size_raw_chn0);
static DEVICE_ATTR(buffer_address_raw_chn1,     SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_raw_chn1,           NULL);
static DEVICE_ATTR(  buffer_pages_raw_chn1,     SYSFS_PERMISSIONS,                   get_size_raw_chn1,           set_size_raw_chn1);
static DEVICE_ATTR(buffer_address_raw_chn2,     SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_raw_chn2,           NULL);
static DEVICE_ATTR(  buffer_pages_raw_chn2,     SYSFS_PERMISSIONS,                   get_size_raw_chn2,           set_size_raw_chn2);
static DEVICE_ATTR(buffer_address_raw_chn3,     SYSFS_PERMISSIONS & SYSFS_READONLY, get_paddr_raw_chn3,           NULL);
static DEVICE_ATTR(  buffer_pages_raw_chn3,     SYSFS_PERMISSIONS,                   get_size_raw_chn3,           set_size_raw_chn3);
static DEVICE_ATTR(sync_for_cpu_h2d,          SYSFS_PERMISSIONS,                  get_sync_for_cpu_h2d,           sync_for_cpu_h2d);
static DEVICE_ATTR(sync_for_device_h2d,       SYSFS_PERMISSIONS,                  get_sync_for_device_h2d,        sync_for_device_h2d);
static DEVICE_ATTR(sync_for_cpu_d2h,          SYSFS_PERMISSIONS,                  get_sync_for_cpu_d2h,           sync_for_cpu_d2h);
static DEVICE_ATTR(sync_for_device_d2h,       SYSFS_PERMISSIONS,                  get_sync_for_device_d2h,        sync_for_device_d2h);
static DEVICE_ATTR(sync_for_cpu_bidir,        SYSFS_PERMISSIONS,                  get_sync_for_cpu_bidir,         sync_for_cpu_bidir);
static DEVICE_ATTR(sync_for_device_bidir,     SYSFS_PERMISSIONS,                  get_sync_for_device_bidir,      sync_for_device_bidir);
static DEVICE_ATTR(sync_for_cpu_histograms,   SYSFS_PERMISSIONS,                  get_sync_for_cpu_histograms,    sync_for_cpu_histograms);
static DEVICE_ATTR(sync_for_device_histograms,SYSFS_PERMISSIONS,                  get_sync_for_device_histograms, sync_for_device_histograms);
static DEVICE_ATTR(sync_for_cpu_logger,       SYSFS_PERMISSIONS,                  get_sync_for_cpu_logger,        sync_for_cpu_logger);
static DEVICE_ATTR(sync_for_device_logger,    SYSFS_PERMISSIONS,                  get_sync_for_device_logger,     sync_for_device_logger);

static struct attribute *root_dev_attrs[] = {
		&dev_attr_buffer_address.attr,
		&dev_attr_buffer_pages.attr,
		&dev_attr_buffer_address_h2d.attr,
		&dev_attr_buffer_pages_h2d.attr,
		&dev_attr_buffer_address_d2h.attr,
		&dev_attr_buffer_pages_d2h.attr,
		&dev_attr_buffer_address_bidir.attr,
		&dev_attr_buffer_pages_bidir.attr,
        &dev_attr_buffer_address_histograms.attr,
        &dev_attr_buffer_pages_histograms.attr,
        &dev_attr_buffer_address_logger.attr,
        &dev_attr_buffer_pages_logger.attr,
		&dev_attr_buffer_address_circbuf_chn0.attr,
		  &dev_attr_buffer_pages_circbuf_chn0.attr,
		&dev_attr_buffer_address_circbuf_chn1.attr,
		  &dev_attr_buffer_pages_circbuf_chn1.attr,
		&dev_attr_buffer_address_circbuf_chn2.attr,
		  &dev_attr_buffer_pages_circbuf_chn2.attr,
		&dev_attr_buffer_address_circbuf_chn3.attr,
		  &dev_attr_buffer_pages_circbuf_chn3.attr,
		&dev_attr_buffer_address_raw_chn0.attr,
		  &dev_attr_buffer_pages_raw_chn0.attr,
		&dev_attr_buffer_address_raw_chn1.attr,
		  &dev_attr_buffer_pages_raw_chn1.attr,
		&dev_attr_buffer_address_raw_chn2.attr,
		  &dev_attr_buffer_pages_raw_chn2.attr,
		&dev_attr_buffer_address_raw_chn3.attr,
		  &dev_attr_buffer_pages_raw_chn3.attr,
		&dev_attr_sync_for_cpu_h2d.attr,
		&dev_attr_sync_for_device_h2d.attr,
		&dev_attr_sync_for_cpu_d2h.attr,
		&dev_attr_sync_for_device_d2h.attr,
		&dev_attr_sync_for_cpu_bidir.attr,
		&dev_attr_sync_for_device_bidir.attr,
        &dev_attr_sync_for_cpu_histograms.attr,
        &dev_attr_sync_for_device_histograms.attr,
        &dev_attr_sync_for_cpu_logger.attr,
        &dev_attr_sync_for_device_logger.attr,
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
	pr_info("Probing elphel393-mem\n");

	if (_elphel_buf.h2d_vaddr){
		// mapped as DMA_BIDIRECTIONAL, each time will be synchronized when passing control from soft to hard and back
		pElphel_buf->h2d_paddr = dma_map_single(&pdev->dev, _elphel_buf.h2d_vaddr, (_elphel_buf.h2d_size*PAGE_SIZE), DMA_TO_DEVICE);
		if (!pElphel_buf->h2d_paddr){
			pr_err("ERROR in dma_map_single() for h2d buffer\n");
			return 0;
		}
//	    printk("H2D DMA buffer location:\t\t0x%08X\n", pElphel_buf->h2d_paddr);
	}

	if (_elphel_buf.d2h_vaddr){
		// mapped as DMA_BIDIRECTIONAL, each time will be synchronized when passing control from soft to hard and back
		pElphel_buf->d2h_paddr = dma_map_single(&pdev->dev, _elphel_buf.d2h_vaddr, (_elphel_buf.d2h_size*PAGE_SIZE), DMA_FROM_DEVICE);
		if (!pElphel_buf->d2h_paddr){
			pr_err("ERROR in dma_map_single() for d2h buffer\n");
			return 0;
		}
	}

    if (_elphel_buf.histograms_vaddr){
        pElphel_buf->histograms_paddr = dma_map_single(&pdev->dev, _elphel_buf.histograms_vaddr, (_elphel_buf.histograms_size*PAGE_SIZE), DMA_FROM_DEVICE);
        if (!pElphel_buf->histograms_paddr){
            pr_err("ERROR in dma_map_single() for histograms buffer\n");
            return 0;
        }
    }
    if (_elphel_buf.logger_vaddr){
        pElphel_buf->logger_paddr = dma_map_single(&pdev->dev, _elphel_buf.logger_vaddr, (_elphel_buf.logger_size*PAGE_SIZE), DMA_FROM_DEVICE);
        if (!pElphel_buf->logger_paddr){
            pr_err("ERROR in dma_map_single() for the logger buffer\n");
            return 0;
        }
    }

	if (_elphel_buf.bidir_vaddr){
		pElphel_buf->bidir_paddr = dma_map_single(&pdev->dev, _elphel_buf.bidir_vaddr, (_elphel_buf.bidir_size*PAGE_SIZE), DMA_BIDIRECTIONAL);
		if (!pElphel_buf->bidir_paddr){
			pr_err("ERROR in dma_map_single() for bidirectional buffer\n");
			return 0;
		}
//	    printk("Bidirectional DMA buffer location:\t0x%08X\n", pElphel_buf->bidir_paddr);
	}

    printk("H2D stream buffer vaddr:            0x%08X\n",(u32) pElphel_buf -> h2d_vaddr);
    printk("H2D stream buffer paddr:            0x%08X\n",(u32) pElphel_buf -> h2d_paddr);
    printk("H2D stream buffer length:           0x%08lX\n",(u32) pElphel_buf -> h2d_size * PAGE_SIZE);

    printk("D2H stream buffer vaddr:            0x%08X\n",(u32) pElphel_buf -> d2h_vaddr);
    printk("D2H stream buffer paddr:            0x%08X\n",(u32) pElphel_buf -> d2h_paddr);
    printk("D2H stream buffer length:           0x%08lX\n",(u32) pElphel_buf -> d2h_size * PAGE_SIZE);

    printk("Bidirectional stream buffer vaddr:  0x%08X\n",(u32) pElphel_buf -> bidir_vaddr);
    printk("Bidirectional stream buffer paddr:  0x%08X\n",(u32) pElphel_buf -> bidir_paddr);
    printk("Bidirectional stream buffer length: 0x%08lX\n",(u32) pElphel_buf -> bidir_size * PAGE_SIZE);

    printk("HISTOGRAMS stream buffer vaddr:     0x%08X\n",(u32) pElphel_buf -> histograms_vaddr);
    printk("HISTOGRAMS stream buffer paddr:     0x%08X\n",(u32) pElphel_buf -> histograms_paddr);
    printk("HISTOGRAMS stream buffer length:    0x%08lX\n",(u32) pElphel_buf -> histograms_size * PAGE_SIZE);

    printk("LOGGER stream buffer vaddr:         0x%08X\n",(u32) pElphel_buf -> logger_vaddr);
    printk("LOGGER stream buffer paddr:         0x%08X\n",(u32) pElphel_buf -> logger_paddr);
    printk("LOGGER stream buffer length:        0x%08lX\n",(u32) pElphel_buf -> logger_size * PAGE_SIZE);

	return 0;
}

static int elphel393_mem_remove(struct platform_device *pdev)
{
	pr_info("Removing elphel393-mem");
	return 0;
}

static struct of_device_id elphel393_mem_of_match[] = {
	{ .compatible = "elphel,elphel393-mem-1.00", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, elphel393_mem_of_match);

static struct platform_driver elphel393_mem = {
	.probe   = elphel393_mem_probe,
	.remove  = elphel393_mem_remove,
	.driver  = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = elphel393_mem_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_mem);
module_init(elphelmem_init);
module_exit(elphelmem_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Reserve a large chunk of contiguous memory at boot");
