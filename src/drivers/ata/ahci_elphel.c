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

static int elphel_port_start(struct ata_port *ap)
{
	void *mem;
	dma_addr_t mem_dma;
	struct device *dev = ap->host->dev;
	struct ahci_port_priv *pp;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	const struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	libahci_debug_init(ap->host);

	dev_info(dev, "starting port %d", ap->port_no);
	libahci_debug_wait_flag();

	pp = devm_kzalloc(dev, sizeof(struct ahci_port_priv), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	mem = dmam_alloc_coherent(dev, AHCI_CMD_TBL_AR_SZ, &mem_dma, GFP_KERNEL);
	if (!mem)
		return -ENOMEM;
	memset(mem, 0, AHCI_CMD_TBL_AR_SZ);
	pp->cmd_tbl = mem;
	pp->cmd_tbl_dma = mem_dma;

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

	libahci_debug_saxigp1_save(ap, 0x3000);
	libahci_debug_saxigp1_save(ap, 0x3000);


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
		dev_info(dev, "base_addr: 0x%08u", dpriv->base_addr);
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
	dev_info(dev, "HOST CAP register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_CTL);
	dev_info(dev, "HOST GHC register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_IRQ_STAT);
	dev_info(dev, "HOST IS register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_PORTS_IMPL);
	dev_info(dev, "HOST PI register: 0x%08x", reg_val);
	reg_val = readl(hpriv->mmio + HOST_VERSION);
	dev_info(dev, "HOST VS register: 0x%08x", reg_val);

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
	dev_info(dev, "ahci platform host initialized");

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

	err_mask = ata_do_dev_read_id(dev, tf, id);
	if (err_mask)
		return err_mask;

	dev_info(d, "issue identify command");

	return 0;
}

static struct ata_port_operations ahci_elphel_ops = {
		.inherits		= &ahci_ops,
		.port_start		= elphel_port_start,
		.read_id		= elphel_read_id,
};

static const struct ata_port_info ahci_elphel_port_info = {
		.flags			= AHCI_FLAG_COMMON,
		.pio_mask		= ATA_PIO4,
		.udma_mask		= ATA_UDMA6,
		.port_ops		= &ahci_elphel_ops,
};

static struct scsi_host_template ahci_platform_sht = {
		AHCI_SHT(DRV_NAME),
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
