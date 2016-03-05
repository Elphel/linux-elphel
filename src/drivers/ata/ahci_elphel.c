/*
 * Elphel AHCI SATA platform driver for elphel393 camera
 *
 * Based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ahci_platform.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include "ahci.h"
#include "libahci_debug.h"

#define DRV_NAME "elphel-ahci"

/* Property names from device tree, these are specific for the controller */
#define PROP_NAME_CLB_OFFS "clb_offs"
#define PROP_NAME_FB_OFFS "fb_offs"

static struct ata_port_operations ahci_elphel_ops;
static const struct ata_port_info ahci_elphel_port_info;
static struct scsi_host_template ahci_platform_sht;
static const struct of_device_id ahci_elphel_of_match[];

struct elphel_ahci_priv {
	u32 clb_offs;
	u32 fb_offs;
	u32 base_addr;
};


// What about port_stop and freeing/unmapping ?
// Or at least check if it is re-started and memory is already allocated/mapped
static int elphel_port_start(struct ata_port *ap)
{
	void *mem;
	dma_addr_t mem_dma;
	struct device *dev = ap->host->dev;
	struct ahci_port_priv *pp;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	const struct elphel_ahci_priv *dpriv = hpriv->plat_data;

//	const ssize_t align_cdt = 128;
	const ssize_t align_cdt = 4096; // just trying - page align
    u32 * dbg_p;
    int   dbg_i;
	libahci_debug_init(ap->host);

	dev_dbg(dev, "starting port %d", ap->port_no);
#ifdef DEBUG_EVENT_ELPHEL
	libahci_debug_wait_flag();
#endif
	pp = devm_kzalloc(dev, sizeof(struct ahci_port_priv), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	// Seems that dmam_alloc_coherent() in Zynq does not really make it "coherent" (write buffers), but stream functions work
	/*
Command Table Descriptor Base Address (CTBA): Indicates the 32-bit physical address of
the command table, which contains the command FIS, ATAPI Command, and PRD table. This
address must be aligned to a 128-byte cache line, indicated by bits 06:00 being reserved.

	mem = dmam_alloc_coherent(dev, AHCI_CMD_TBL_AR_SZ, &mem_dma, GFP_KERNEL);
	if (!mem)
		return -ENOMEM;
	memset(mem, 0, AHCI_CMD_TBL_AR_SZ); // dmam_alloc_coherent() does this
	void *dma_alloc_coherent(struct device *dev, size_t size, dma_addr_t *dma_handle, int flag);
    dma_addr_t dma_map_single(struct device *dev, void *buffer, size_t size, enum dma_data_direction direction);
    void dma_unmap_single(struct device *dev, dma_addr_t dma_addr, size_t size, enum dma_data_direction direction);
	*/
//	mem = devm_kzalloc(dev, AHCI_CMD_TBL_AR_SZ + align_cdt - 1, GFP_KERNEL);
//	mem = devm_kmalloc(dev, AHCI_CMD_TBL_AR_SZ + align_cdt - 1, GFP_KERNEL); // let some junk be there
	mem = devm_kmalloc(dev, 0x100000, GFP_KERNEL); // AHCI_CMD_TBL_AR_SZ = 0x16000
	dbg_p = (u32*) mem;
	for (dbg_i=0; dbg_i < ((AHCI_CMD_TBL_AR_SZ + align_cdt)>>2); dbg_i++) {
		dbg_p[dbg_i] = dbg_i;
	}
	dbg_i = 0;
	/*
	if (((u32) mem) & (align_cdt - 1)) {
//		mem += align_cdt - (((u32) mem) & (align_cdt - 1));
		dbg_i = align_cdt - (((u32) mem) & (align_cdt - 1));
	}
*/
	mem_dma = dma_map_single(dev, mem, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE); // maybe DMA_BIDIRECTIONAL, but currently we do not use DMA for received FISes

	dev_dbg(dev, "ahci_elphel.c: dbg_i= 0x%08x, mem= 0x%08x, mem_dma= 0x%08x", dbg_i, (u32) mem, (u32) mem_dma);
	pp->cmd_tbl = mem + dbg_i;
	pp->cmd_tbl_dma = mem_dma + dbg_i;
	dev_dbg(dev, "ahci_elphel.c: dbg_i= 0x%08x, pp->cmd_tbl= 0x%08x, pp->cmd_tbl_dma= 0x%08x", dbg_i, (u32) pp->cmd_tbl, (u32) pp->cmd_tbl_dma);

	/*
	 * Set predefined addresses
	 */
	pp->cmd_slot = hpriv->mmio + dpriv->clb_offs;
	//pp->cmd_slot_dma = virt_to_phys(pp->cmd_slot);
	pp->cmd_slot_dma = 0x80000000 + dpriv->clb_offs;

	pp->rx_fis = hpriv->mmio + dpriv->fb_offs;
	//pp->rx_fis_dma = virt_to_phys(pp->rx_fis);
	pp->rx_fis_dma = 0x80000000 + dpriv->fb_offs;

	/*dev_info(dev, "cmd_slot: 0x%p", pp->cmd_slot);
	dev_info(dev, "cmd_slot_dma: 0x%08u", pp->cmd_slot_dma);
	dev_info(dev, "rx_fis: 0x%p", pp->rx_fis);
	dev_info(dev, "rx_fis_dma: 0x%08u", pp->rx_fis_dma);
	dev_info(dev, "base_addr: 0x%08u", dpriv->base_addr);*/

	/*
	 * Save off initial list of interrupts to be enabled.
	 * This could be changed later
	 */
	pp->intr_mask = DEF_PORT_IRQ;

	ap->private_data = pp;

//	libahci_debug_state_dump(ap);
//	libahci_debug_state_dump(ap);

	//libahci_debug_saxigp1_save(ap, 0x3000);
	//libahci_debug_saxigp1_save(ap, 0x3000);

	dev_dbg(dev, "flags (ATA_FLAG_xxx): %u", ap->flags);
	dev_dbg(dev, "pflags (ATA_PFLAG_xxx): %u", ap->pflags);

	dev_dbg(dev, "ahci_elphel.c: Calling  ahci_port_resume()");
	return ahci_port_resume(ap);
}

static int elphel_parse_prop(const struct device_node *devn,
		struct device *dev,
		struct elphel_ahci_priv *dpriv)
{
	u64 size;
	unsigned int flags;
	const __be32 *val;
	struct resource res;

	if (!devn) {
		dev_err(dev, "device tree node is not found");
		return -EINVAL;
	}

	val = of_get_property(devn, PROP_NAME_CLB_OFFS, NULL);
	dpriv->clb_offs = be32_to_cpup(val);
	val = of_get_property(devn, PROP_NAME_FB_OFFS, NULL);
	dpriv->fb_offs = be32_to_cpup(val);
	val = of_get_address(devn, 0, NULL, NULL);
	if (val != NULL) {
		dpriv->base_addr = be32_to_cpu(val);
		dev_dbg(dev, "base_addr: 0x%08u", dpriv->base_addr);
	} else {
		dev_err(dev, "can not get register address");
	}
	//of_address_to_resource(devn, 0, &res);

	return 0;
}

static int elphel_drv_probe(struct platform_device *pdev)
{
	int ret;
	struct ahci_host_priv *hpriv;
	struct elphel_ahci_priv *drv_priv;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;

	struct resource *res;
	unsigned int reg_val;

	dev_info(&pdev->dev, "probing Elphel AHCI driver");

	drv_priv = devm_kzalloc(dev, sizeof(struct elphel_ahci_priv), GFP_KERNEL);
	if (!drv_priv)
		return -ENOMEM;

	match = of_match_device(ahci_elphel_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	ret = elphel_parse_prop(dev->of_node, dev, drv_priv);
	if (ret != 0)
		return ret;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	hpriv->plat_data = drv_priv;

	reg_val = readl(hpriv->mmio + HOST_CAP);
	dev_dbg(dev, "HOST CAP register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_CTL);
	dev_dbg(dev, "HOST GHC register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_IRQ_STAT);
	dev_dbg(dev, "HOST IS register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_PORTS_IMPL);
	dev_dbg(dev, "HOST PI register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_VERSION);
	dev_dbg(dev, "HOST VS register: 0x%08x", reg_val);

	phys_addr_t paddr = virt_to_phys(hpriv->mmio);
	void *vaddr = phys_to_virt(paddr);
	dev_err(dev, "current mmio virt addr: 0x%p\n", hpriv->mmio);
	dev_err(dev, "current mmio virt addr as uint: 0x%08x\n", hpriv->mmio);
	dev_err(dev, "mmio phys addr: 0x%08x\n", paddr);
	dev_err(dev, "mmio phys addr as tr: 0x%p\n", paddr);
	dev_err(dev, "back converted mmio virt addr: 0x%p\n", vaddr);
	//printk(KERN_DEBUG, "current mmio virt addr: %p\n", hpriv->mmio);
	//printk(KERN_DEBUG, "mmio phys addr: %u\n", paddr);
	//printk(KERN_DEBUG, "back converted mmio virt addr: %p\n", vaddr);
	printk(KERN_DEBUG "======");

	ret = ahci_platform_init_host(pdev, hpriv, &ahci_elphel_port_info,
			&ahci_platform_sht);
	if (ret) {
		dev_err(dev, "can not initialize platform host");
		ahci_platform_disable_resources(hpriv);
		return ret;
	}
	dev_dbg(dev, "ahci platform host initialized");

	return 0;
}

static int elphel_drv_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "removing Elphel AHCI driver");
	ata_platform_remove_one(pdev);
	libahci_debug_exit();

	return 0;
}

static unsigned int elphel_read_id(struct ata_device *dev, struct ata_taskfile *tf, u16 *id)
{
	u32 err_mask;
	struct device *d = &dev->tdev;
	int i, len;
	char *msg_str;

	err_mask = ata_do_dev_read_id(dev, tf, id);
	if (err_mask)
		return err_mask;

	dev_dbg(d, "elphel_read_id(): issue identify command finished\n");
	/*dev_info(d, "dump IDENTIFY:\n");
	msg_str = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!msg_str)
		return 0;
	len = 0;
	for (i = 0; i < ATA_ID_WORDS; i++) {
		if ((i % 16) == 0 && i != 0) {
			dev_info(d, "%s\n", msg_str);
			len = 0;
		}
		len += snprintf(msg_str + len, PAGE_SIZE - len, "0x%04x ", id[i]);
	}
	// print last string
	dev_info(d, "%s\n", msg_str);*/

	return 0;
}

static struct ata_port_operations ahci_elphel_ops = {
		.inherits		= &ahci_ops,
		.port_start		= elphel_port_start,
		.read_id		= elphel_read_id,
};

static const struct ata_port_info ahci_elphel_port_info = {
		AHCI_HFLAGS(AHCI_HFLAG_NO_NCQ),
		.flags			= AHCI_FLAG_COMMON,
		.pio_mask		= ATA_PIO4,
		.udma_mask		= ATA_UDMA6,
		.port_ops		= &ahci_elphel_ops,
};

static struct scsi_host_template ahci_platform_sht = {
		AHCI_SHT(DRV_NAME),
		.can_queue			= 1,
		.sg_tablesize		= AHCI_MAX_SG,
		.dma_boundary		= AHCI_DMA_BOUNDARY,
		.shost_attrs		= ahci_shost_attrs,
		.sdev_attrs			= ahci_sdev_attrs,
};

static const struct of_device_id ahci_elphel_of_match[] = {
		{ .compatible = "elphel,elphel-ahci", },
		{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, ahci_elphel_of_match);

static struct platform_driver ahci_elphel_driver = {
		.probe			= elphel_drv_probe,
		/*.remove			= ata_platform_remove_one,*/
		.remove			= elphel_drv_remove,
		.driver	= {
				.name	= DRV_NAME,
				.owner	= THIS_MODULE,
				.of_match_table	= ahci_elphel_of_match,
		},
};
module_platform_driver(ahci_elphel_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Elphel AHCI SATA platform driver for elphel393 camera");
