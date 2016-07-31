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
#include <linux/sysfs.h>
#include "ahci.h"

#include <elphel/elphel393-mem.h>

#define DRV_NAME "elphel-ahci"
/*
 * FPGA bitstream control address and bit mask. These are used to check whether
 * bitstream is loaded or not.
 */
#define BITSTREAM_CTRL_ADDR	0xf800700c
#define BITSTREAM_CTRL_BIT	0x4

/* Property names from device tree, these are specific for the controller */
#define PROP_NAME_CLB_OFFS "clb_offs"
#define PROP_NAME_FB_OFFS "fb_offs"
/** Flag indicating that IRQ should not be processed in ahci_port_interrupt */
#define IRQ_SIMPLE                (1 << 0)
/** The length of a command FIS in double words */
#define CMD_FIS_LEN               5
/** This is used to get 28-bit address from 64-bit value */
#define ADDR_MASK_28_BIT          ((u64)0xfffffff)

static struct ata_port_operations ahci_elphel_ops;
static const struct ata_port_info ahci_elphel_port_info;
static struct scsi_host_template ahci_platform_sht;
static const struct of_device_id ahci_elphel_of_match[];
static const struct attribute_group dev_attr_root_group;

static bool load_driver = false;

struct elphel_ahci_priv {
	u32 clb_offs;
	u32 fb_offs;
	u32 base_addr;
	u32 flags;
};

static struct platform_device *g_pdev;
static ssize_t elphel_test_write(struct device *dev, struct device_attribute *attr,
		const char *buff, size_t buff_sz);
static irqreturn_t elphel_irq_handler(int irq, void * dev_instance);
static int elphel_write_dma(struct ata_port *ap, u64 start, u16 count, struct scatterlist *sg, unsigned int elem);
static int elphel_read_dma(struct ata_port *ap, u64 start, u16 count, struct scatterlist *sgl, unsigned int elem);
void prep_cfis(u8 *cmd_tbl, u8 cmd, u64 start_addr, u16 count);

static ssize_t set_load_flag(struct device *dev, struct device_attribute *attr,
		const char *buff, size_t buff_sz)
{
	load_driver = true;

	return buff_sz;
}

static int bitstream_loaded(u32 *ptr)
{
	u32 val = ioread32(ptr);

	if (val & BITSTREAM_CTRL_BIT)
		return 1;
	else
		return 0;
}

static void elphel_defer_load(struct device *dev)
{
	bool check_flag = true;
	u32 *ctrl_ptr = ioremap_nocache(BITSTREAM_CTRL_ADDR, 4);

	dev_info(dev, "AHCI driver loading is deferred. Load bitstream and write 1 into "
			"/sys/devices/soc0/amba@0/80000000.elphel-ahci/load_module to continue\n");
	while (check_flag) {
		if (load_driver) {
			if (bitstream_loaded(ctrl_ptr)) {
				check_flag = false;
			} else {
				dev_err(dev, "FPGA bitstream is not loaded or bitstream "
						"does not contain AHCI controller\n");
				load_driver = false;
			}
		} else {
			msleep(1000);
		}
	}
	load_driver = false;
	iounmap(ctrl_ptr);
}

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

	dev_dbg(dev, "starting port %d", ap->port_no);
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

	if (&dev->kobj) {
		ret = sysfs_create_group(&dev->kobj, &dev_attr_root_group);
		if (ret < 0)
			return ret;
	}
//	elphel_defer_load(dev);

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

	ret = ahci_platform_init_host(pdev, hpriv, &ahci_elphel_port_info,
			&ahci_platform_sht);
	if (ret) {
		dev_err(dev, "can not initialize platform host");
		ahci_platform_disable_resources(hpriv);
		return ret;
	}
	g_pdev = pdev;
	/* reassign interrupt handler*/
	int rc;
	unsigned int irq_flags = IRQF_SHARED;
	int irq = platform_get_irq(pdev, 0);
	struct ata_host *ahost = platform_get_drvdata(pdev);
	printk(KERN_DEBUG ">>> removing automatically assigned irq handler\n");
	devm_free_irq(dev, irq, ahost);
	rc = devm_request_irq(dev, irq, elphel_irq_handler, irq_flags, dev_name(dev), ahost);
	if (rc) {
		printk(KERN_DEBUG ">>> failed to request irq\n");
		return rc;
	}
	printk(KERN_DEBUG ">>> irq handler reassigned\n");

	return 0;
}

static int elphel_drv_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "removing Elphel AHCI driver");
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_root_group);
	ata_platform_remove_one(pdev);

	return 0;
}

void dump_tf_addr(struct ata_taskfile *tf)
{
	printk(KERN_DEBUG ">>> taskfile dump: lbal = 0x%x, lbam = 0x%x, lbah = 0x%x, "
			"hob_lbal = 0x%x, hod_lbam = 0x%x, hob_lbah = 0x%x\n",
			tf->lbal, tf->lbam, tf->lbah,
			tf->hob_lbal, tf->hob_lbam, tf->hob_lbah);
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

	/* There is only one slot in controller thus we need to change tag*/
	qc->tag = 0;

	/*
	 * Fill in command table information.  First, the header,
	 * a SATA Register - Host to Device command FIS.
	 */
	dma_sync_single_for_cpu(&qc->dev->tdev, pp->cmd_tbl_dma,
			AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);
	cmd_tbl = pp->cmd_tbl + qc->tag * AHCI_CMD_TBL_SZ;

	ata_tf_to_fis(&qc->tf, qc->dev->link->pmp, 1, cmd_tbl);

	dev_dbg(ap->dev, ">>> CFIS dump, data from libahci:\n");
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, cmd_tbl, 20);

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

static DEVICE_ATTR(load_module, S_IWUSR | S_IWGRP, NULL, set_load_flag);
static DEVICE_ATTR(test_write, S_IWUSR | S_IWGRP, NULL, elphel_test_write);
static struct attribute *root_dev_attrs[] = {
		&dev_attr_load_module.attr,
		&dev_attr_test_write.attr,
		NULL
};
static const struct attribute_group dev_attr_root_group = {
		.attrs			= root_dev_attrs,
		.name			= NULL,
};

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

#define TEST_BUFF_SZ              512
//#define SDA2_LBA_ADDR             124963848
#define SDA2_LBA_ADDR             1
#define SG_TBL_SZ                 3
static ssize_t elphel_test_write(struct device *dev, struct device_attribute *attr,
		const char *buff, size_t buff_sz)
{
	int i, n_elem;
	int sg_elems = 0;
	struct ata_host *host;
	struct ata_port *port;
	struct ahci_host_priv *hpriv;
	struct elphel_ahci_priv *dpriv;
	struct scatterlist *sgl;
	struct scatterlist *sg_ptr;
//	u8 *test_buff = pElphel_buf->d2h_vaddr;
	u8 *test_buff;
	unsigned int lba_addr;

	if (sscanf(buff, "%u", &lba_addr) == 1) {
		printk(KERN_DEBUG ">>> got LBA address: %d\n", lba_addr);
	} else {
		lba_addr = SDA2_LBA_ADDR;
	}

	host = platform_get_drvdata(g_pdev);
	port = host->ports[0];
	hpriv = port->host->private_data;
	dpriv = hpriv->plat_data;

	/* prepare buffer and fill it with markers */
	sgl = kmalloc(sizeof(struct scatterlist) * SG_TBL_SZ, GFP_KERNEL);
	if (!sgl)
		return ENOMEM;
	sg_init_table(sgl, SG_TBL_SZ);
	for_each_sg(sgl, sg_ptr, SG_TBL_SZ, n_elem) {
		test_buff = kmalloc(TEST_BUFF_SZ, GFP_KERNEL);
		if (!test_buff)
			return ENOMEM;
		memset(test_buff, 0xa5, TEST_BUFF_SZ);
		sg_set_buf(sg_ptr, (void *)test_buff, TEST_BUFF_SZ);
		sg_elems++;
	}

	printk(KERN_DEBUG ">>> mapped %d SG elemets\n", sg_elems);
	printk(KERN_DEBUG ">>>\n");

	/* read test */
	dma_map_sg(dev, sgl, sg_elems, DMA_FROM_DEVICE);

	printk(KERN_DEBUG ">>> trying to read data to sg list\n");
	elphel_read_dma(port, lba_addr, sg_elems, sgl, sg_elems);
	printk(KERN_DEBUG ">>> command has been issued\n");

	while (dpriv->flags & IRQ_SIMPLE) {
		printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
		msleep(1);
	}

//	printk(KERN_DEBUG ">>> dump test buffer after reading: %d bytes\n", TEST_BUFF_SZ);
//	dma_sync_single_for_cpu(dev, pElphel_buf->d2h_paddr, pElphel_buf->d2h_size, DMA_FROM_DEVICE);
//	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, test_buff, TEST_BUFF_SZ);
//	dma_sync_single_for_device(dev, pElphel_buf->d2h_paddr, pElphel_buf->d2h_size, DMA_FROM_DEVICE);
//	printk(KERN_DEBUG ">>> buffer has been mapped for device\n");

	printk(KERN_DEBUG ">>> dump test buffer after reading: %d bytes\n", TEST_BUFF_SZ);
	dma_unmap_sg(dev, sgl, sg_elems, DMA_FROM_DEVICE);
	for (i = 0; i < sg_elems; i++) {
		dev_dbg(dev, ">>> sector %i\n", i);
		u8 buff[TEST_BUFF_SZ];
		sg_copy_to_buffer(&sgl[i], 1, buff, TEST_BUFF_SZ);
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buff, TEST_BUFF_SZ);
	}

	/* end of read test */

//	printk(KERN_DEBUG ">>> interrupt flag has been cleared\n");
//	printk(KERN_DEBUG ">>> dump of SG list area\n");
//	dma_sync_single_for_cpu(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);
//	print_hex_dump_bytes("", DUMP_PREFIX_NONE, test_buff, TEST_BUFF_SZ);
//	dma_sync_single_for_device(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);
//	/* end of read test */
//
//	/* write test */
//	printk(KERN_DEBUG ">>> filling test buffer: %d bytes\n", TEST_BUFF_SZ);
//	dma_sync_single_for_cpu(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);
//	for (i = 0; i < TEST_BUFF_SZ - 1; i += 2) {
//		test_buff[i] = 0xaa;
//		test_buff[i + 1] = 0x55;
//	}
//	print_hex_dump_bytes("", DUMP_PREFIX_NONE, test_buff, TEST_BUFF_SZ);
//	dma_sync_single_for_device(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);
//	printk(KERN_DEBUG ">>> buffer has been mapped\n");
//
//	sg_init_one(&sg, pElphel_buf->h2d_vaddr, TEST_BUFF_SZ);
//
//	printk(KERN_DEBUG ">>> trying to read data to sg list\n");
//	elphel_read_dma(port, SDA2_LBA_ADDR, 1, &sg, 1);
//	printk(KERN_DEBUG ">>> command has been issued\n");
//	/* end of write test */
//
////	while (dpriv->flags & IRQ_SIMPLE) {
////		msleep(1);
////	}
//	printk(KERN_DEBUG ">>> interrupt flag has been cleared\n");
//	printk(KERN_DEBUG ">>> dump of SG list area\n");
//	dma_sync_single_for_cpu(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);
//	print_hex_dump_bytes("", DUMP_PREFIX_NONE, test_buff, TEST_BUFF_SZ);
//	dma_sync_single_for_device(dev, pElphel_buf->h2d_paddr, pElphel_buf->h2d_size, DMA_TO_DEVICE);

	return buff_sz;
}

/** Prepare software constructed command FIS in command table area. The structure of the
 * command FIS is described in Transport Layer chapter of Serial ATA revision 3.1 documentation.
 */
inline void prep_cfis(u8 *cmd_tbl,               ///< pointer to the beginning of command table
		u8 cmd,                                  ///< ATA command as described in ATA/ATAPI command set
		u64 start_addr,                          ///< LBA start address
		u16 count)                               ///< sector count, the number of 512 byte sectors to read or write
		                                         ///< @return None
{
	u8 device, ctrl;

	/* select the content of Device and Control registers based on command, read the description of
	 * a command in ATA/ATAPI command set documentation
	 */
	switch (cmd) {
	case ATA_CMD_WRITE_EXT:
		// not verified yet
		device = 0x00;
		ctrl = 0x00;
		break;
	case ATA_CMD_READ:
		device = 0xe0 | ((start_addr >> 24) & 0x0f);
		ctrl = 0x08;
		/* this is 28-bit command; 4 bits of the address have already been
		 * placed to Device register, invalidate the remaining (if any) upper
		 * bits of the address and leave only 24 significant bits (just in case)
		 */
		start_addr &= 0xffffff;
		break;
	case ATA_CMD_READ_EXT:
		device = 0xe0;
		ctrl = 0x08;
		break;
	default:
		device = 0x00;
		ctrl = 0x00;
	}

	cmd_tbl[0] = 0x27;                       // H2D register FIS
	cmd_tbl[1] = 0x80;                       // set C = 1
	cmd_tbl[2] = cmd;                        // ATA READ or WRITE DMA command as described in ATA/ATAPI command set
	cmd_tbl[3] = 0;                          // features(7:0)
	cmd_tbl[4] = start_addr & 0xff;          // LBA(7:0)
	cmd_tbl[5] = (start_addr >> 8)  & 0xff;  // LBA(15:8)
	cmd_tbl[6] = (start_addr >> 16) & 0xff;  // LBA(23:16)
	cmd_tbl[7] = device;                     // device
	cmd_tbl[8] = (start_addr >> 24)  & 0xff; // LBA(31:24)
	cmd_tbl[9] = (start_addr >> 32)  & 0xff; // LBA(39:32)
	cmd_tbl[10] = (start_addr >> 40) & 0xff; // LBA(47:40)
	cmd_tbl[11] = 0;                         // features(15:8)
	cmd_tbl[12] = count & 0xff;              // count(7:0)
	cmd_tbl[13] = (count >> 8) & 0xff;       // count(15:8)
	cmd_tbl[14] = 0;                         // ICC (isochronous command completion)
	cmd_tbl[15] = ctrl;                      // control
}

/** Map S/G list to physical region descriptor table in AHCI controller command table */
inline void prep_prdt(struct scatterlist *sgl,   ///< pointer to S/G list which should be mapped to physical
		                                         ///< region description table
		unsigned int n_elem,                     ///< the number of elements in @e sgl
		struct ahci_sg *ahci_sgl)                ///< pointer to physical region description table
		                                         ///< @return None
{
	unsigned int num = 0;
	struct scatterlist *sg_ptr;

	for_each_sg(sgl, sg_ptr, n_elem, num) {
		dma_addr_t addr = sg_dma_address(sg_ptr);
		u32 sg_len = sg_dma_len(sg_ptr);

		ahci_sgl[num].addr = cpu_to_le32(addr & 0xffffffff);
		ahci_sgl[num].addr_hi = cpu_to_le32((addr >> 16) >> 16);
		ahci_sgl[num].flags_size = cpu_to_le32(sg_len - 1);
	}
}

static int elphel_write_dma(struct ata_port *ap, u64 start, u16 count, struct scatterlist *sg, unsigned int elem)
{
	u32 opts;
	const u32 cmd_fis_len = 5;
	unsigned int n_elem;
//	void *cmd_tbl;
	u8 *cmd_tbl;
	unsigned int slot_num = 0;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;
	struct scatterlist *sg_ptr;
	struct ahci_sg *ahci_sg;
	void __iomem *port_mmio = ahci_port_base(ap);

	dpriv->flags |= IRQ_SIMPLE;

	/* prepare command FIS */
	dma_sync_single_for_cpu(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);
	cmd_tbl = pp->cmd_tbl + slot_num * AHCI_CMD_TBL_SZ;
	cmd_tbl[0] = 0x27;                  // H2D register FIS
	cmd_tbl[1] = 0x80;                  // set C = 1
	cmd_tbl[2] = ATA_CMD_WRITE;         // ATA WRITE DMA command as described in ATA/ATAPI command set
	cmd_tbl[3] = 0;                     // features(7:0)
	cmd_tbl[4] = (start >> 0)  & 0xff;  // LBA(7:0)
	cmd_tbl[5] = (start >> 8)  & 0xff;  // LBA(15:8)
	cmd_tbl[6] = (start >> 16) & 0xff;  // LBA(23:16)
	cmd_tbl[7] = 0;                     // device
	cmd_tbl[8] = (start >> 24)  & 0xff; // LBA(31:24)
	cmd_tbl[9] = (start >> 32)  & 0xff; // LBA(39:32)
	cmd_tbl[10] = (start >> 40) & 0xff; // LBA(47:40)
	cmd_tbl[11] = 0;                    // features(15:8)
	cmd_tbl[12] = (count >> 0) & 0xff;  // count(7:0)
	cmd_tbl[13] = (count >> 8) & 0xff;  // count(15:8)
	cmd_tbl[14] = 0;                    // ICC (isochronous command completion)
	cmd_tbl[15] = 0;                    // control

	/* prepare physical region descriptor table */
	n_elem = 0;
	ahci_sg = pp->cmd_tbl + slot_num * AHCI_CMD_TBL_SZ + AHCI_CMD_TBL_HDR_SZ;
	prep_prdt(sg, elem, ahci_sg);
//	for_each_sg(sg, sg_ptr, elem, n_elem) {
//		dma_addr_t addr = sg_dma_address(sg_ptr);
//		u32 sg_len = sg_dma_len(sg_ptr);
//
//		ahci_sg[n_elem].addr = cpu_to_le32(addr & 0xffffffff);
//		ahci_sg[n_elem].addr_hi = cpu_to_le32((addr >> 16) >> 16);
//		ahci_sg[n_elem].flags_size = cpu_to_le32(sg_len - 1);
//	}

	/* prepare command header */
	opts = cmd_fis_len | (n_elem << 16) | AHCI_CMD_WRITE | AHCI_CMD_PREFETCH | AHCI_CMD_CLR_BUSY;
	ahci_fill_cmd_slot(pp, slot_num, opts);
	dma_sync_single_for_device(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);

	/* issue command */
	writel(1 << slot_num, port_mmio + PORT_CMD_ISSUE);

	return 0;
}

static int elphel_read_dma(struct ata_port *ap, u64 start, u16 count, struct scatterlist *sgl, unsigned int elem)
{
	u32 opts;
	u8 *cmd_tbl;
	u8 cmd;
	unsigned int slot_num = 0;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;
	struct ahci_sg *ahci_sg;
	void __iomem *port_mmio = ahci_port_base(ap);

	dpriv->flags |= IRQ_SIMPLE;

	/* prepare command FIS */
	dma_sync_single_for_cpu(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);
	cmd_tbl = pp->cmd_tbl + slot_num * AHCI_CMD_TBL_SZ;
	if (start & ~ADDR_MASK_28_BIT)
		cmd = ATA_CMD_READ_EXT;
	else
		cmd = ATA_CMD_READ;
	prep_cfis(cmd_tbl, cmd, start, count);

	/* prepare physical region descriptor table */
	ahci_sg = pp->cmd_tbl + slot_num * AHCI_CMD_TBL_SZ + AHCI_CMD_TBL_HDR_SZ;
	prep_prdt(sgl, elem, ahci_sg);

	/* prepare command header */
	opts = CMD_FIS_LEN | (elem << 16) | AHCI_CMD_PREFETCH | AHCI_CMD_CLR_BUSY;
	ahci_fill_cmd_slot(pp, slot_num, opts);

	printk(KERN_DEBUG ">>> dump command table content, first %d bytes, phys addr = 0x%x:\n", TEST_BUFF_SZ, pp->cmd_tbl_dma);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, pp->cmd_tbl, TEST_BUFF_SZ);

	dma_sync_single_for_device(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);

	/* issue command */
	writel(0x11, port_mmio + PORT_CMD);
	writel(1 << slot_num, port_mmio + PORT_CMD_ISSUE);

	return 0;
}

static irqreturn_t elphel_irq_handler(int irq, void * dev_instance)
{
	irqreturn_t handled;
	struct ata_host *host = dev_instance;
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;
	u32 irq_stat, irq_masked;

	if (dpriv->flags & IRQ_SIMPLE) {
		/* handle interrupt */
		printk(KERN_DEBUG ">>> handling interrupt\n");
		dpriv->flags &= ~IRQ_SIMPLE;
//		clear_bit(IRQ_SIMPLE, &dpriv->flags);
		irq_stat = readl(hpriv->mmio + HOST_IRQ_STAT);
		if (!irq_stat)
			return IRQ_NONE;
//		irq_masked = irq_stat & hpriv->port_map;
		writel(irq_stat, hpriv->mmio + HOST_IRQ_STAT);
		handled = IRQ_HANDLED;
	} else {
		/* pass handling to AHCI level*/
		handled = ahci_single_irq_intr(irq, dev_instance);
	}

	return handled;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Elphel AHCI SATA platform driver for elphel393 camera");
