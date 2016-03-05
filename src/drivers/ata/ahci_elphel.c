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

	libahci_debug_init(ap->host);

	dev_dbg(dev, "starting port %d", ap->port_no);
#ifdef DEBUG_EVENT_ELPHEL
	libahci_debug_wait_flag();
#endif
	pp = devm_kzalloc(dev, sizeof(struct ahci_port_priv), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	mem = devm_kmalloc(dev, 0x100000, GFP_KERNEL); // AHCI_CMD_TBL_AR_SZ = 0x16000
	if (!mem)
		return -ENOMEM;
	mem_dma = dma_map_single(dev, mem, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE); // maybe DMA_BIDIRECTIONAL, but currently we do not use DMA for received FISes

	pp->cmd_tbl = mem;
	pp->cmd_tbl_dma = mem_dma;

	/*
	 * Set predefined addresses
	 */
	pp->cmd_slot = hpriv->mmio + dpriv->clb_offs;
	pp->cmd_slot_dma = dpriv->base_addr + dpriv->clb_offs;

	pp->rx_fis = hpriv->mmio + dpriv->fb_offs;
	pp->rx_fis_dma = dpriv->base_addr + dpriv->fb_offs;

	/*
	 * Save off initial list of interrupts to be enabled.
	 * This could be changed later
	 */
	pp->intr_mask = DEF_PORT_IRQ;

	ap->private_data = pp;

	dev_dbg(dev, "flags (ATA_FLAG_xxx): %lu", ap->flags);
	dev_dbg(dev, "pflags (ATA_PFLAG_xxx): %u", ap->pflags);

	dev_dbg(dev, "ahci_elphel.c: Calling  ahci_port_resume()");
	return ahci_port_resume(ap);
}

static int elphel_parse_prop(const struct device_node *devn,
		struct device *dev,
		struct elphel_ahci_priv *dpriv)
{
	int rc = 0;
	const __be32 *val;
	struct resource res;

	if (!devn) {
		dev_err(dev, "elphel-ahci device tree node is not found");
		return -EINVAL;
	}

	val = of_get_property(devn, PROP_NAME_CLB_OFFS, NULL);
	if (!val) {
		dev_err(dev, "can not find clb_offs in device tree");
		return -EINVAL;
	}
	dpriv->clb_offs = be32_to_cpup(val);

	val = of_get_property(devn, PROP_NAME_FB_OFFS, NULL);
	if (!val) {
		dev_err(dev, "can not find fb_offs in device tree");
		return -EINVAL;
	}
	dpriv->fb_offs = be32_to_cpup(val);

	rc = of_address_to_resource((struct device_node *)devn, 0, &res);
	if (rc < 0) {
		dev_err(dev, "can not find address in device tree");
		return -EINVAL;
	}
	dpriv->base_addr = (u32)res.start;

	return 0;
}

static int elphel_drv_probe(struct platform_device *pdev)
{
	int ret;
	struct ahci_host_priv *hpriv;
	struct elphel_ahci_priv *dpriv;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	unsigned int reg_val;

	dev_info(&pdev->dev, "probing Elphel AHCI driver");

	dpriv = devm_kzalloc(dev, sizeof(struct elphel_ahci_priv), GFP_KERNEL);
	if (!dpriv)
		return -ENOMEM;

	match = of_match_device(ahci_elphel_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	ret = elphel_parse_prop(dev->of_node, dev, dpriv);
	if (ret != 0)
		return ret;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	hpriv->plat_data = dpriv;

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

	ret = ahci_platform_init_host(pdev, hpriv, &ahci_elphel_port_info,
			&ahci_platform_sht);
	if (ret) {
		dev_err(dev, "can not initialize platform host");
		ahci_platform_disable_resources(hpriv);
		return ret;
	}

	return 0;
}

static int elphel_drv_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "removing Elphel AHCI driver");
	ata_platform_remove_one(pdev);
	libahci_debug_exit();

	return 0;
}

static void elphel_qc_prep(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ahci_port_priv *pp = ap->private_data;
	int is_atapi = ata_is_atapi(qc->tf.protocol);
	void *cmd_tbl;
	u32 opts;
	const u32 cmd_fis_len = 5; /* five dwords */
	unsigned int n_elem;
	struct scatterlist *sg;
	struct ahci_sg *ahci_sg;

	/*
	 * Fill in command table information.  First, the header,
	 * a SATA Register - Host to Device command FIS.
	 */
	dma_sync_single_for_cpu(&qc->dev->tdev, pp->cmd_tbl_dma,
			AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);
	cmd_tbl = pp->cmd_tbl + qc->tag * AHCI_CMD_TBL_SZ;

	ata_tf_to_fis(&qc->tf, qc->dev->link->pmp, 1, cmd_tbl);

	if (is_atapi) {
		memset(cmd_tbl + AHCI_CMD_TBL_CDB, 0, 32);
		memcpy(cmd_tbl + AHCI_CMD_TBL_CDB, qc->cdb, qc->dev->cdb_len);
	}

	/*
	 * Next, the S/G list.
	 */
	n_elem = 0;
	ahci_sg = cmd_tbl + AHCI_CMD_TBL_HDR_SZ;
	if (qc->flags & ATA_QCFLAG_DMAMAP) {
		for_each_sg(qc->sg, sg, qc->n_elem, n_elem) {
			dma_addr_t addr = sg_dma_address(sg);
			u32 sg_len = sg_dma_len(sg);

			ahci_sg[n_elem].addr = cpu_to_le32(addr & 0xffffffff);
			ahci_sg[n_elem].addr_hi = cpu_to_le32((addr >> 16) >> 16);
			ahci_sg[n_elem].flags_size = cpu_to_le32(sg_len - 1);
		}

		if (qc->dma_dir == DMA_TO_DEVICE) {
			dma_sync_sg_for_device(&qc->dev->tdev, qc->sg, qc->n_elem, qc->dma_dir);
		} else if (qc->dma_dir == DMA_FROM_DEVICE) {
			dma_sync_sg_for_cpu(&qc->dev->tdev, qc->sg, qc->n_elem, qc->dma_dir);
		}
	}

	/*
	 * Fill in command slot information.
	 */
	opts = cmd_fis_len | n_elem << 16 | (qc->dev->link->pmp << 12);
	if (qc->tf.flags & ATA_TFLAG_WRITE)
		opts |= AHCI_CMD_WRITE;
	if (is_atapi)
		opts |= AHCI_CMD_ATAPI | AHCI_CMD_PREFETCH;

	ahci_fill_cmd_slot(pp, qc->tag, opts);
	dma_sync_single_for_device(&qc->dev->tdev, pp->cmd_tbl_dma,
			AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);
}

static struct ata_port_operations ahci_elphel_ops = {
		.inherits		= &ahci_ops,
		.port_start		= elphel_port_start,
		.qc_prep		= elphel_qc_prep,
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
		.can_queue		= 1,
		.sg_tablesize	= AHCI_MAX_SG,
		.dma_boundary	= AHCI_DMA_BOUNDARY,
		.shost_attrs	= ahci_shost_attrs,
		.sdev_attrs		= ahci_sdev_attrs,
};

static const struct of_device_id ahci_elphel_of_match[] = {
		{ .compatible = "elphel,elphel-ahci", },
		{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, ahci_elphel_of_match);

static struct platform_driver ahci_elphel_driver = {
		.probe			= elphel_drv_probe,
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
