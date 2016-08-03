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
#include <linux/uio.h>
//#include <asm/uaccess.h>

#include "ahci.h"
#include "ahci_elphel.h"
#include "../elphel/exif393.h"
#include "../elphel/exifa.h"
#include "../elphel/jpeghead.h"
//#include "../elphel/circbuf.h"

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

/** Maximum number of sectors for READ DMA or WRITE DMA commands */
#define MAX_LBA_COUNT             0xff
/** Maximum number of sectors for READ DMA EXT or WRITE_DMA EXT commands */
#define MAX_LBA_COUNT_EXT         0xffff

static struct ata_port_operations ahci_elphel_ops;
static const struct ata_port_info ahci_elphel_port_info;
static struct scsi_host_template ahci_platform_sht;
static const struct of_device_id ahci_elphel_of_match[];
static const struct attribute_group dev_attr_root_group;

static bool load_driver = false;

static void elphel_cmd_issue(struct ata_port *ap, uint64_t start, uint16_t count, struct fvec *sgl, unsigned int elem, uint8_t cmd);
static int init_buffers(struct device *dev, struct frame_buffers *buffs);
static void init_vectors(struct elphel_ahci_priv *dpriv);
static void deinit_buffers(struct device *dev, struct frame_buffers *buffs);
static inline struct elphel_ahci_priv *dev_get_dpriv(struct device *dev);
static void finish_cmd(struct device *dev, struct elphel_ahci_priv *dpriv);
static int process_cmd(struct device *dev, struct elphel_ahci_priv *dpriv, struct ata_port *port);
//static void start_cmd(struct device *dev, struct elphel_ahci_priv *dpriv, struct ata_port *port);
static inline size_t get_size_from(const struct fvec *vects, int index, size_t offset, int all);

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

static irqreturn_t elphel_irq_handler(int irq, void * dev_instance)
{
	irqreturn_t handled;
	struct ata_host *host = dev_instance;
	struct ahci_host_priv *hpriv = host->private_data;
	struct ata_port *port = host->ports[DEFAULT_PORT_NUM];
	void __iomem *port_mmio = ahci_port_base(port);
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;
	uint32_t irq_stat, host_irq_stat;


	if (dpriv->flags & IRQ_SIMPLE) {
		/* handle interrupt */
		host_irq_stat = readl(hpriv->mmio + HOST_IRQ_STAT);
		if (!host_irq_stat)
			return IRQ_NONE;
		dpriv->flags &= ~IRQ_SIMPLE;
		irq_stat = readl(port_mmio + PORT_IRQ_STAT);

		printk(KERN_DEBUG "irq_stat = 0x%x, host irq_stat = 0x%x\n", irq_stat, host_irq_stat);

//		writel(irq_stat, port_mmio + PORT_IRQ_STAT);
		writel(0xffffffff, port_mmio + PORT_IRQ_STAT);

		writel(host_irq_stat, hpriv->mmio + HOST_IRQ_STAT);
		handled = IRQ_HANDLED;

//		if (proc_cmd(host->dev, dpriv, host->ports[0]) == 0)
//			finish_cmd(host->dev, dpriv);
	} else {
		/* pass handling to AHCI level */
		handled = ahci_single_irq_intr(irq, dev_instance);
	}

	return handled;
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

	ret = init_buffers(dev, &dpriv->fbuffs);
	if (ret != 0)
		return ret;
//	sg_init_table(dpriv->sgl, MAX_DATA_CHUNKS);
	init_vectors(dpriv);

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
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(&pdev->dev);

	dev_info(&pdev->dev, "removing Elphel AHCI driver");
	deinit_buffers(&pdev->dev, &dpriv->fbuffs);
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

	dev_dbg(ap->dev, ">>> CFIS dump, data from libahci, phys addr = 0x%x:\n", pp->cmd_tbl_dma);
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

/* ============================================== */
#define TEST_BUFF_SZ              512
#define MAX_IOVECTORS             10
/** The position of size field in copy buffer */
#define VECTOR_SZ_POS             0
/** The position of vector pointer field in copy buffer */
#define POINTER_POS               1
/** Physical disk block size */
#define PHY_BLOCK_SIZE            512
//#define PHY_BLOCK_SIZE            4096
#define JPEG_MARKER_LEN           2
#define SG_TBL_SZ                 256
/** Include REM buffer to total size calculation */
#define INCLUDE_REM               1
/** Exclude REM buffer from total size calculation */
#define EXCLUDE_REM               0

#define DEBUG_DONT_WRITE

unsigned char app15[ALIGNMENT_SIZE] = {0xff, 0xef};

/* this should be placed to system includes directory*/
struct frame_data {
	unsigned int sensor_port;
	int cirbuf_ptr;
	int jpeg_len;
	int meta_index;
};
/* end of system includes */

/* Debug functions */
#define DATA_BUFF_SIZE            500000
unsigned char *g_jpg_data_0;
unsigned char *g_jpg_data_1;
int use_preset;
size_t g_jpg_0_sz;
size_t g_jpg_1_sz;
ssize_t g_exif_sz;
ssize_t g_jpg_hdr_sz;
static size_t exif_get_data_tst(int sensor_port, unsigned short meta_index, void *buff, size_t buff_sz, int enable)
{
	int i;
	const int default_exif_sz = 774;
	int exif_sz;
	unsigned char *dest = buff;

	if (g_exif_sz >= 0 && g_exif_sz < MAX_EXIF_SIZE)
		exif_sz = g_exif_sz;
	else
		exif_sz = default_exif_sz;

	if (buff_sz < exif_sz || enable == 0)
		return 0;

	dest[0] = 0xff;
	dest[1] = 0xe1;
	for (i = 2; i < exif_sz; i++) {
		dest[i] = 0xa1;
	}

	return exif_sz;
}
static size_t jpeghead_get_data_tst(int sensor_port, void *buff, size_t buff_sz, size_t offs)
{
	int i;
	const int default_jpeghdr_sz = 623;
	int jpeghdr_sz;
	unsigned char *dest = buff;

	if (g_jpg_hdr_sz >=0 && g_jpg_hdr_sz < JPEG_HEADER_MAXSIZE)
		jpeghdr_sz = g_jpg_hdr_sz;
	else
		jpeghdr_sz = default_jpeghdr_sz;

	if (buff_sz < jpeghdr_sz)
		return 0;

	dest[0] = 0xff;
	dest[1] = 0xd8;
	dest[2] = 0xff;
	dest[3] = 0xe0;
	for (i = 4; i < jpeghdr_sz; i++) {
		dest[i] = 0xb2;
	}

	return jpeghdr_sz;
}
#include <linux/random.h>
static int circbuf_get_ptr_tst(int sensor_port, size_t offset, size_t len, struct fvec *vect_0, struct fvec *vect_1)
{
	int ret = 1;
	size_t jpg_0_sz;
	size_t jpg_1_sz;

	get_random_bytes(&jpg_0_sz, sizeof(size_t));
	get_random_bytes(&jpg_1_sz, sizeof(size_t));
	if (use_preset == 0) {
		if (jpg_0_sz != 0)
			jpg_0_sz = jpg_0_sz % (DATA_BUFF_SIZE - 1);
		if (jpg_1_sz != 0)
			jpg_1_sz = jpg_1_sz % (DATA_BUFF_SIZE - 1);
	} else if (use_preset == 1) {
		if (g_jpg_0_sz != 0)
			jpg_0_sz = jpg_0_sz % g_jpg_0_sz;
		if (g_jpg_1_sz != 0)
			jpg_1_sz = jpg_1_sz % g_jpg_1_sz;
	} else if (use_preset == 2) {
		jpg_0_sz = g_jpg_0_sz;
		jpg_1_sz = g_jpg_1_sz;
	}
	if (g_jpg_0_sz != 0)
		memset(g_jpg_data_0, 0xc3, jpg_0_sz);
	if (g_jpg_1_sz != 0)
		memset(g_jpg_data_1, 0xd4, jpg_1_sz);

	if (g_jpg_0_sz != 0) {
		vect_0->iov_base = g_jpg_data_0;
//		vect_0->iov_dma = 0;
		vect_0->iov_len = jpg_0_sz;
	} else {
		vect_0->iov_base = NULL;
		vect_0->iov_dma = 0;
		vect_0->iov_len = 0;
	}

	if (g_jpg_1_sz != 0) {
		vect_1->iov_base = g_jpg_data_1;
//		vect_1->iov_dma = 0;
		vect_1->iov_len = jpg_1_sz;
		ret = 2;
	} else {
		vect_1->iov_base = NULL;
		vect_1->iov_dma = 0;
		vect_1->iov_len = 0;
	}

	return ret;
}
static void dump_frame(struct fvec *vects)
{
	int i;
	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
		printk(KERN_DEBUG ">>> dump data chunk %d, size %u\n", i, vects[i].iov_len);
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, vects[i].iov_base, vects[i].iov_len);
	}
}
static int check_chunks(struct fvec *vects)
{
	int i;
	int ret = 0;
	size_t sz = 0;
	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
		if (i != CHUNK_REM) {
			sz += vects[i].iov_len;
			if ((vects[i].iov_len % ALIGNMENT_SIZE) != 0) {
				dev_err(NULL, "ERROR: unaligned write from slot %d, length %u\n", i, vects[i].iov_len);
				ret = -1;
			}
			if ((vects[i].iov_dma % ALIGNMENT_ADDR) != 0) {
				dev_err(NULL, "ERROR: unaligned DMA address in slot %d: 0x%x\n", i, vects[i].iov_dma);
				ret = -1;
			}
		}
		if ((i == CHUNK_ALIGN_0 || i == CHUNK_ALIGN_1) && vects[i].iov_len > 2 * ALIGNMENT_SIZE) {
			dev_err(NULL, "ERROR: alignment buffer %d overflow\n", i);
			ret = -1;
		}
	}
	if ((sz % PHY_BLOCK_SIZE) != 0) {
		dev_err(NULL, "ERROR: total length of the transaction is not aligned to sector boundary, total length %u\n", sz);
		ret = -1;
	} else {
		dev_err(NULL, ">>> +++ frame is OK +++\n");
	}
	return ret;
}
static ssize_t data_0_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	if (kstrtoul(buff, 10, &g_jpg_0_sz) != 0)
		return -EINVAL;
	printk(KERN_DEBUG ">>> preset DATA_0 length: %u\n", g_jpg_0_sz);
	return buff_sz;
}
static ssize_t data_1_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	if (kstrtoul(buff, 10, &g_jpg_1_sz) != 0)
		return -EINVAL;
	printk(KERN_DEBUG ">>> preset DATA_1 length: %u\n", g_jpg_1_sz);
	return buff_sz;
}
static ssize_t data_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	if (kstrtoul(buff, 10, &use_preset) != 0)
		return -EINVAL;

	return buff_sz;
}
static ssize_t exif_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	if (kstrtol(buff, 10, &g_exif_sz) != 0)
		return -EINVAL;
	printk(KERN_DEBUG ">>> preset EXIF length: %u\n", g_exif_sz);

	return buff_sz;
}
static ssize_t hdr_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	if (kstrtol(buff, 10, &g_jpg_hdr_sz) != 0)
		return -EINVAL;
	printk(KERN_DEBUG ">>> preset JPEGHEADER length: %u\n", g_jpg_hdr_sz);

	return buff_sz;
}
static DEVICE_ATTR(data_0_sz, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, NULL, data_0_write);
static DEVICE_ATTR(data_1_sz, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, NULL, data_1_write);
static DEVICE_ATTR(data_proc, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, NULL, data_write);
static DEVICE_ATTR(exif_sz, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, NULL, exif_write);
static DEVICE_ATTR(jpg_hdr_sz, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, NULL, hdr_write);
/* End of debug functions*/

/** Map buffer vectors to S/G list and return the number of vectors mapped */
static int map_vectors(struct elphel_ahci_priv *dpriv)
{
	int i;
	int index = 0;
	int finish = 0;
	size_t total_sz = 0;
	size_t tail;
	struct fvec *chunks = dpriv->data_chunks;
	struct fvec vect;

	for (i = dpriv->curr_data_chunk; i < MAX_DATA_CHUNKS; i++) {
		if (i == CHUNK_REM)
			/* remainder should never be processed */
			continue;
		if (i == dpriv->curr_data_chunk) {
			total_sz = chunks[i].iov_len - dpriv->curr_data_offset;
			vect.iov_base = (unsigned char *)chunks[i].iov_base + dpriv->curr_data_offset;
			vect.iov_dma = chunks[i].iov_dma + dpriv->curr_data_offset;
			vect.iov_len = chunks[i].iov_len - dpriv->curr_data_offset;
		} else {
			total_sz += chunks[i].iov_len;
//			vect.iov_base = chunks[i].iov_base;
//			vect.iov_len = chunks[i].iov_len;
			vect = chunks[i];
		}
		if (total_sz > dpriv->max_data_sz) {
			// truncate current buffer and finish mapping
			tail = total_sz - dpriv->max_data_sz;
			vect.iov_len -= tail;
			dpriv->curr_data_chunk = i;
			dpriv->curr_data_offset = chunks[i].iov_len - tail;
			finish = 1;
		} else if (unlikely(total_sz == dpriv->max_data_sz)) {
			dpriv->curr_data_chunk = i;
			dpriv->curr_data_offset = chunks[i].iov_len;
			finish = 1;
		}
		printk(KERN_DEBUG "mapping data chunk number %d: total_sz = %u, vect.iov_len = %u\n", i, total_sz, vect.iov_len);
		if (vect.iov_len != 0) {
//			sg_set_buf(&dpriv->sgl[index++], vect.iov_base, vect.iov_len);
			dpriv->sgl[index++] = vect;
		}
		if (finish)
			break;
	}
	if (finish == 0) {
		// frame vectors have been fully processed, stop calling me
		dpriv->curr_data_chunk = MAX_DATA_CHUNKS;
		dpriv->curr_data_offset = 0;
	}

	return index;
}

static inline void vectcpy(struct fvec *dest, void *src, size_t len)
{
	unsigned char *d = (unsigned char *)dest->iov_base;

	memcpy(d + dest->iov_len, src, len);
	dest->iov_len += len;
}
static inline void vectmov(struct fvec *vec, size_t len)
{
	if (vec->iov_len >= len) {
		vec->iov_base = (unsigned char *)vec->iov_base + len;
		vec->iov_dma += len;
		vec->iov_len -= len;
	}
}
static inline void vectshrink(struct fvec *vec, size_t len)
{
	if (vec->iov_len >= len) {
		vec->iov_len -= len;
	}
}
static inline size_t align_bytes_num(size_t data_len, size_t align_len)
{
	size_t rem = data_len % align_len;
	if (rem == 0)
		return 0;
	else
		return align_len - rem;
}
/** This helper function is used to position a pointer @e offset bytes from the end
 * of a buffer. DMA handle is not updated intentionally as it is not needed during copying */
static inline unsigned char *vectrpos(struct fvec *vec, size_t offset)
{
	return (unsigned char *)vec->iov_base + (vec->iov_len - offset);
}
/** Check if bus address will be properly aligned after moving vector forward by @e len bytes */
static inline int is_addr_aligned(const struct fvec *vec, size_t len)
{
	int ret = 0;

	if (vec->iov_len >= len) {
		if (((vec->iov_dma + len) % ALIGNMENT_ADDR) == 0)
			ret = 1;
	}

	return ret;
}
static size_t align_address(struct fvec *dest, struct fvec *src, size_t len)
{
	size_t cut_bytes = (src->iov_dma + len) % ALIGNMENT_ADDR;
	size_t pad_len = ALIGNMENT_SIZE - cut_bytes;
	if (pad_len <= len) {
		len -= pad_len;
	}
	if (len != 0) {
		len += cut_bytes;
	}
	/* set marker length */
	app15[3] = pad_len - JPEG_MARKER_LEN;
	printk(KERN_DEBUG ">>> copy %u bytes from APP15 to other buffer\n", pad_len);
	vectcpy(dest, app15, pad_len);

	return len;
}
static void align_frame(struct device *dev, struct elphel_ahci_priv *dpriv)
{
	int is_delayed = 0;
	unsigned char *src;
	size_t len, total_sz, data_len;
	size_t max_len = dpriv->fbuffs.common_buff.iov_len;
	struct frame_buffers *fbuffs = &dpriv->fbuffs;
	struct fvec *chunks = dpriv->data_chunks;
	struct fvec *cbuff = &chunks[CHUNK_COMMON];

	total_sz = get_size_from(chunks, 0, 0, INCLUDE_REM);
	if (total_sz < PHY_BLOCK_SIZE) {
		/* the frame length is less than sector size, delay this frame */
		dev_dbg(dev, "frame size is less than sector size: %u bytes; delay recording\n", total_sz);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_LEADER].iov_base, chunks[CHUNK_LEADER].iov_len);
		vectshrink(&chunks[CHUNK_LEADER], chunks[CHUNK_LEADER].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_EXIF].iov_base, chunks[CHUNK_EXIF].iov_len);
		vectshrink(&chunks[CHUNK_EXIF], chunks[CHUNK_EXIF].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_HEADER].iov_base, chunks[CHUNK_HEADER].iov_len);
		vectshrink(&chunks[CHUNK_HEADER], chunks[CHUNK_HEADER].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
		vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
		vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
		vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		return;
	}

	dma_sync_single_for_cpu(dev, fbuffs->common_buff.iov_dma, fbuffs->common_buff.iov_len, DMA_TO_DEVICE);
	dma_sync_single_for_cpu(dev, chunks[CHUNK_DATA_0].iov_dma, chunks[CHUNK_DATA_0].iov_len, DMA_TO_DEVICE);
	dma_sync_single_for_cpu(dev, chunks[CHUNK_DATA_1].iov_dma, chunks[CHUNK_DATA_1].iov_len, DMA_TO_DEVICE);

	/* copy remainder of previous frame to the beginning of common buffer */
	if (likely(chunks[CHUNK_REM].iov_len != 0)) {
		len = chunks[CHUNK_REM].iov_len;
	printk(KERN_DEBUG ">>> copy %u bytes from REM to common buffer\n", len);
		vectcpy(cbuff, chunks[CHUNK_REM].iov_base, len);
		vectshrink(&chunks[CHUNK_REM], chunks[CHUNK_REM].iov_len);
	}

	/* copy JPEG marker */
	len = chunks[CHUNK_LEADER].iov_len;
	printk(KERN_DEBUG ">>> copy %u bytes from LEADER to common buffer\n", len);
	vectcpy(cbuff, chunks[CHUNK_LEADER].iov_base, len);
	vectshrink(&chunks[CHUNK_LEADER], chunks[CHUNK_LEADER].iov_len);

	/* copy Exif if present */
	if (chunks[CHUNK_EXIF].iov_len != 0) {
		len = chunks[CHUNK_EXIF].iov_len;
	printk(KERN_DEBUG ">>> copy %u bytes from EXIF to common buffer\n", len);
		vectcpy(cbuff, chunks[CHUNK_EXIF].iov_base, len);
		vectshrink(&chunks[CHUNK_EXIF], chunks[CHUNK_EXIF].iov_len);
	}

	/* copy JPEG header */
	len = chunks[CHUNK_HEADER].iov_len;
	printk(KERN_DEBUG ">>> copy %u bytes from HEADER to common buffer\n", len);
	vectcpy(cbuff, chunks[CHUNK_HEADER].iov_base, len);
	vectshrink(&chunks[CHUNK_HEADER], chunks[CHUNK_HEADER].iov_len);

	/* check if we have enough data for further processing */
	len = chunks[CHUNK_DATA_0].iov_len + chunks[CHUNK_DATA_1].iov_len + chunks[CHUNK_TRAILER].iov_len;
	if (len < PHY_BLOCK_SIZE) {
		data_len = cbuff->iov_len % PHY_BLOCK_SIZE;
		dev_dbg(dev, "JPEG data size is too small: %u; unaligned data length: %u\n", len, data_len);
		if (data_len + len <= fbuffs->rem_buff.iov_len) {
			/* place data to REM buffer and continue */
			src = vectrpos(cbuff, data_len);
	printk(KERN_DEBUG ">>> 5.0.1 copy %u bytes from COMMON to REM buffer \n", len);
			vectcpy(&chunks[CHUNK_REM], src, data_len);
			vectshrink(cbuff, data_len);
	printk(KERN_DEBUG ">>> 5.0.1 copy %u bytes from DATA_0 to REM buffer \n", len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
			vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
	printk(KERN_DEBUG ">>> 5.0.1 copy %u bytes from DATA_1 to REM buffer \n", len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
			vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
	printk(KERN_DEBUG ">>> 5.0.1 copy %u bytes from TRAILER to REM buffer \n", len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
			vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		} else {
			/* the REM buffer is too short, process some buffers to align COMMON buffer to sector boundary */
			dev_dbg(dev, "REM buffer is too short, align COMMON to sector boundary\n");
			data_len = PHY_BLOCK_SIZE - data_len;
			if (data_len >= chunks[CHUNK_DATA_0].iov_len) {
				/* copy whole DATA_0 to COMMON */
	printk(KERN_DEBUG ">>> 5.0.2 copy %u bytes from DATA_0 to COMMON buffer \n", chunks[CHUNK_DATA_0].iov_len);
				vectcpy(cbuff, chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				data_len -= chunks[CHUNK_DATA_0].iov_len;
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
			} else {
				/* copy a part of DATA_0 to COMMON */
	printk(KERN_DEBUG ">>> 5.0.2 copy %u bytes from DATA_0 to COMMON buffer \n", data_len);
				src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
				vectcpy(cbuff, src, data_len);
				vectshrink(&chunks[CHUNK_DATA_0], data_len);
				data_len = 0;
			}
			if (data_len != 0) {
				if (data_len >= chunks[CHUNK_DATA_1].iov_len) {
					/* copy whole DATA_1 to COMMON */
	printk(KERN_DEBUG ">>> 5.0.2 copy %u bytes from DATA_1 to COMMON buffer \n", chunks[CHUNK_DATA_1].iov_len);
					vectcpy(cbuff, chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
					data_len -= chunks[CHUNK_DATA_1].iov_len;
					vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
				} else {
					/* copy a part of DATA_1 to COMMON */
	printk(KERN_DEBUG ">>> 5.0.2 copy %u bytes from DATA_1 to COMMON buffer \n", data_len);
					src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
					vectcpy(cbuff, src, data_len);
					vectshrink(&chunks[CHUNK_DATA_1], data_len);
					data_len = 0;
				}
				if (data_len != 0) {
					/* copy a part of TRAILER to COMMON */
	printk(KERN_DEBUG ">>> 5.0.2 copy %u bytes from TRAILER to COMMON buffer \n", data_len);
					src = vectrpos(&chunks[CHUNK_TRAILER], data_len);
					vectcpy(cbuff, src, data_len);
					vectshrink(&chunks[CHUNK_TRAILER], data_len);
					data_len = 0;
				}
			}
			/* COMMON buffer is aligned to sector boundary, copy all other data to REM buffer */
			if (chunks[CHUNK_DATA_0].iov_len != 0) {
	printk(KERN_DEBUG ">>> 5.0.3 copy %u bytes from DATA_0 to REM buffer \n", chunks[CHUNK_DATA_0].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
			}
			if (chunks[CHUNK_DATA_1].iov_len != 0) {
	printk(KERN_DEBUG ">>> 5.0.3 copy %u bytes from DATA_1 to REM buffer \n", chunks[CHUNK_DATA_1].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
				vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
			}
			if (chunks[CHUNK_TRAILER].iov_len != 0) {
	printk(KERN_DEBUG ">>> 5.0.3 copy %u bytes from TRAILER to REM buffer \n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			}
		}

		return;
	}

	/* there is enough data in data buffers, align common buffer */
	len = align_bytes_num(cbuff->iov_len, ALIGNMENT_SIZE);
	if (likely(len != 0)) {
		dev_dbg(dev, "align COMMON buffer, add %u bytes from other buffers\n", len);
		if ((len + ALIGNMENT_ADDR) <= chunks[CHUNK_DATA_0].iov_len) {
	printk(KERN_DEBUG ">>> check alignment of DMA address: 0x%x\n", chunks[CHUNK_DATA_0].iov_dma);
			if (is_addr_aligned(&chunks[CHUNK_DATA_0], len) == 0) {
				/* we are going to copy =len= bytes from DATA_0 buffer to align COMMON buffer to
				 * ALIGNMNET_SIZE boundary which will make the starting address of DATA_0 buffer
				 * unaligned to ALIGNMENT_ADDR boundary, fix this by moving additional bytes from
				 * the beginning of DATA_0 buffer and padding COMMON buffer with APP15 marker */
				len = align_address(cbuff, &chunks[CHUNK_DATA_0], len);
			}
	printk(KERN_DEBUG ">>> 5.1.1 copy %u bytes from DATA_0 to common buffer \n", len);
			vectcpy(cbuff, chunks[CHUNK_DATA_0].iov_base, len);
			vectmov(&chunks[CHUNK_DATA_0], len);
		} else {
			/* handle special case: JPEG data rolls over circular buffer boundary and the
			 * data chunk in the end of the buffer is less than we need for alignment; copy
			 * all data from first chunk, discard it, copy remainder from second chunk
			 * and from trailer if needed
			 */
			if (len <= (chunks[CHUNK_DATA_0].iov_len + chunks[CHUNK_DATA_1].iov_len + chunks[CHUNK_TRAILER].iov_len + ALIGNMENT_ADDR)) {
	printk(KERN_DEBUG ">>> 5.2 copy %u bytes from DATA_0 to common buffer\n", chunks[CHUNK_DATA_0].iov_len);
				vectcpy(cbuff, chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				len -= chunks[CHUNK_DATA_0].iov_len;
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
				if (chunks[CHUNK_DATA_1].iov_len >= len) {
	printk(KERN_DEBUG ">>> check alignment of DMA address: 0x%x\n", chunks[CHUNK_DATA_1].iov_dma);
					if (is_addr_aligned(&chunks[CHUNK_DATA_1], len) == 0) {
						len = align_address(cbuff, &chunks[CHUNK_DATA_1], len);
					}
	printk(KERN_DEBUG ">>> 5.2.4 copy %u bytes from DATA_1 to common buffer\n", len);
					vectcpy(cbuff, chunks[CHUNK_DATA_1].iov_base, len);
					vectmov(&chunks[CHUNK_DATA_1], len);
					len = 0;
				} else {
	printk(KERN_DEBUG ">>> 5.2.5 copy %u bytes from DATA_1 to common buffer\n", chunks[CHUNK_DATA_1].iov_len);
					vectcpy(cbuff, chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
					len -= chunks[CHUNK_DATA_1].iov_len;
					vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
				}
				if (chunks[CHUNK_TRAILER].iov_len >= len) {
	printk(KERN_DEBUG ">>> 5.2.6 copy %u bytes from TRAILER to common buffer\n", len);
					vectcpy(cbuff, chunks[CHUNK_TRAILER].iov_base, len);
					vectmov(&chunks[CHUNK_TRAILER], len);
					len = 0;
				} else {
	printk(KERN_DEBUG ">>> 5.2.7 copy %u bytes from TRAILER to common buffer\n", chunks[CHUNK_TRAILER].iov_len);
					vectcpy(cbuff, chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
					len -= chunks[CHUNK_TRAILER].iov_len;
					vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
				}
			} else {
				/* JPEG data is too short, place all to remainder buffer */
				data_len = cbuff->iov_len % PHY_BLOCK_SIZE;
				src = vectrpos(cbuff, data_len);
	printk(KERN_DEBUG ">>> 5.2.1 copy %u bytes from COMMON to REM buffer\n", data_len);
				vectcpy(&chunks[CHUNK_REM], src, data_len);
				vectshrink(cbuff, data_len);
	printk(KERN_DEBUG ">>> 5.2.1 copy %u bytes from DATA_0 to REM buffer\n", chunks[CHUNK_DATA_0].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
	printk(KERN_DEBUG ">>> 5.2.1 copy %u bytes from DATA_1 to REM buffer\n", chunks[CHUNK_DATA_1].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
				vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
	printk(KERN_DEBUG ">>> 5.2.1 copy %u bytes from TRAILER to REM buffer\n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			}
		}
	}

	/* JPEG header and Exif data is aligned to ALIGNMENT_SIZE boundary in a common buffer now,
	 * the entire frame should be aligned to physical sector boundary and remainder should be
	 * copied to a buffer for recording during next frame */
	total_sz = get_size_from(chunks, 0, 0, INCLUDE_REM);
	len = total_sz % PHY_BLOCK_SIZE;
	chunks[CHUNK_ALIGN_0].iov_base = (unsigned char *)cbuff->iov_base + cbuff->iov_len;
	chunks[CHUNK_ALIGN_0].iov_len = 0;
	dev_dbg(dev, "total frame size: %u, unaligned bytes to sector boundary: %u\n", total_sz, len);
	if (likely(chunks[CHUNK_DATA_1].iov_len == 0)) {
		/* JPEG data is not split, align just one buffer to physical sector boundary if it is not already aligned */
		if (chunks[CHUNK_DATA_0].iov_len + chunks[CHUNK_TRAILER].iov_len >= len) {
			if (likely(len >= chunks[CHUNK_TRAILER].iov_len)) {
				data_len = len - chunks[CHUNK_TRAILER].iov_len;
				src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
		printk(KERN_DEBUG ">>> 6.1 copy %u bytes from DATA_0 to remainder buffer\n", data_len);
				vectcpy(&chunks[CHUNK_REM], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_0], data_len);
		printk(KERN_DEBUG ">>> 6.1 copy %u bytes from TRAILER to remainder buffer\n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			} else {
				/* align DATA_0 buffer, break trailing marker and place first part to ALIGN_0 buffer and second part to REM */
				data_len = chunks[CHUNK_DATA_0].iov_len % ALIGNMENT_SIZE;
		printk(KERN_DEBUG ">>> 6.1.2 copy %u bytes from DATA_0 to ALIGN_0 buffer\n", data_len);
				src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
				vectcpy(&chunks[CHUNK_ALIGN_0], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_0], data_len);
				if (len == 0)
					len = chunks[CHUNK_TRAILER].iov_len;
		printk(KERN_DEBUG ">>> 6.1.2 copy %u bytes from TRAILER to ALIGN_0 buffer\n", len);
				vectcpy(&chunks[CHUNK_ALIGN_0], chunks[CHUNK_TRAILER].iov_base, len);
				vectmov(&chunks[CHUNK_TRAILER], len);
		printk(KERN_DEBUG ">>> 6.1.2 copy %u bytes from TRAILER to remainder buffer\n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			}
		} else {
			/* the number of bytes needed to align to sector boundary is greater than the number of bytes left
			 * in DATA_0 and TRAILER buffers, realign COMMON buffer and delay DATA_0 and TRAILING buffers recording */
			data_len = cbuff->iov_len % PHY_BLOCK_SIZE;
		printk(KERN_DEBUG ">>> 6.1.3 copy %u bytes from COMMON to remainder buffer\n", data_len);
			vectcpy(&chunks[CHUNK_REM], cbuff->iov_base, data_len);
			vectshrink(cbuff, data_len);
		printk(KERN_DEBUG ">>> 6.1.3 copy %u bytes from DATA_0 to remainder buffer\n", chunks[CHUNK_DATA_0].iov_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
			vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
		printk(KERN_DEBUG ">>> 6.1.3 copy %u bytes from TRAILER to remainder buffer\n", chunks[CHUNK_TRAILER].iov_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
			vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		}
	} else {
	printk(KERN_DEBUG ">>> 6.2 <<<\n");
		/* JPEG data rolls over circular buffer boundary */
		struct kvec data_rem;
		data_len = chunks[CHUNK_DATA_0].iov_len % ALIGNMENT_SIZE;
		size_t align_len = ALIGNMENT_SIZE - data_len;

		chunks[CHUNK_ALIGN_1].iov_base = (unsigned char *)chunks[CHUNK_ALIGN_0].iov_base + chunks[CHUNK_ALIGN_0].iov_len;
		chunks[CHUNK_ALIGN_1].iov_len = 0;

		if (chunks[CHUNK_DATA_1].iov_len >= len + align_len) {
			/* DATA_1 buffer contains enough bytes to align frame to sector boundary,
			 * align DATA_0 buffer to ALIGNMENT_SIZE boundary only */
			if (data_len != 0) {
	printk(KERN_DEBUG ">>> 6.2 copy %u bytes from DATA_0 to ALIGN_0 buffer\n", data_len);
				src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
				vectcpy(&chunks[CHUNK_ALIGN_0], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_0], data_len);
	printk(KERN_DEBUG ">>> check alignment of DMA address: 0x%x\n", chunks[CHUNK_DATA_1].iov_dma);
				if (is_addr_aligned(&chunks[CHUNK_DATA_1], align_len) == 0) {
					align_len = align_address(&chunks[CHUNK_ALIGN_0], &chunks[CHUNK_DATA_1], align_len);
				}
	printk(KERN_DEBUG ">>> 6.2 copy %u bytes from DATA_1 to ALIGN_0 buffer\n", align_len);
				vectcpy(&chunks[CHUNK_ALIGN_0], chunks[CHUNK_DATA_1].iov_base, align_len);
				vectmov(&chunks[CHUNK_DATA_1], align_len);
				/* adjust ALIGN_1 position */
				chunks[CHUNK_ALIGN_1].iov_base = (unsigned char *)chunks[CHUNK_ALIGN_0].iov_base + chunks[CHUNK_ALIGN_0].iov_len;
				chunks[CHUNK_ALIGN_1].iov_len = 0;
			}
		} else {
			/* there is no enough data in the second JPEG data buffer, delay its recording and
			 * align first buffer to physical sector boundary */
			data_len = (cbuff->iov_len + chunks[CHUNK_DATA_0].iov_len) % PHY_BLOCK_SIZE;
	printk(KERN_DEBUG ">>> 6.2.2 align to data_len: %u\n", data_len);
			if (data_len <= chunks[CHUNK_DATA_0].iov_len) {
				src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
	printk(KERN_DEBUG ">>> 6.2.2 copy %u bytes from DATA_0 to REM buffer\n", data_len);
				vectcpy(&chunks[CHUNK_REM], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_0], data_len);
			} else {
				data_len -= chunks[CHUNK_DATA_0].iov_len;
				src = vectrpos(cbuff, data_len);
	printk(KERN_DEBUG ">>> 6.2.2 copy %u bytes from COMMON to REM buffer\n", data_len);
				vectcpy(&chunks[CHUNK_REM], cbuff->iov_base, data_len);
				vectshrink(cbuff, data_len);
	printk(KERN_DEBUG ">>> 6.2.2 copy %u bytes from DATA_0 to REM buffer\n", chunks[CHUNK_DATA_0].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
				/* COMMON buffer has been updated, adjust ALIGN_0 and ALIGN_1 position */
				chunks[CHUNK_ALIGN_1].iov_base = (unsigned char *)chunks[CHUNK_ALIGN_0].iov_base + chunks[CHUNK_ALIGN_0].iov_len;
				chunks[CHUNK_ALIGN_1].iov_len = 0;
				chunks[CHUNK_ALIGN_1].iov_base = (unsigned char *)chunks[CHUNK_ALIGN_0].iov_base + chunks[CHUNK_ALIGN_0].iov_len;
				chunks[CHUNK_ALIGN_1].iov_len = 0;
			}
	printk(KERN_DEBUG ">>> 6.2.2 copy %u bytes from DATA_1 to REM buffer\n", chunks[CHUNK_DATA_1].iov_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
			vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
	printk(KERN_DEBUG ">>> 6.2.2 copy %u bytes from TRAILER to REM buffer\n", chunks[CHUNK_TRAILER].iov_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
			vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		}

	printk(KERN_DEBUG ">>> 6.3 <<<\n");
		/* the total length could have changed by the moment, recalculate */
		total_sz = get_size_from(chunks, 0, 0, INCLUDE_REM);
		len = total_sz % PHY_BLOCK_SIZE;
		dev_dbg(dev, "total frame size: %u, unaligned bytes to sector boundary: %u\n", total_sz, len);
		if (chunks[CHUNK_DATA_1].iov_len + chunks[CHUNK_TRAILER].iov_len > len) {
			if (len >= chunks[CHUNK_TRAILER].iov_len) {
				/* DATA_1 and TRAILER buffers contain enough data to align the current frame to sector boundary */
				data_len = len - chunks[CHUNK_TRAILER].iov_len;
	printk(KERN_DEBUG ">>> 6.3.1 copy %u bytes from DATA_1 to REM buffer\n", data_len);
				src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
				vectcpy(&chunks[CHUNK_REM], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_1], data_len);
	printk(KERN_DEBUG ">>> 6.3.1 copy %u bytes from TRAILER to REM buffer\n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			} else if (len != 0) {
				/* break trailing marker and copy the remaining data to REM buffer, align DATA_1 and TRAILER parts in
				 * ALIGN_1 buffer */
				data_len = chunks[CHUNK_DATA_1].iov_len % ALIGNMENT_SIZE;
	printk(KERN_DEBUG ">>> 6.3.2 copy %u bytes from DATA_1 to ALIGN_1 buffer\n", data_len);
				src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
				vectcpy(&chunks[CHUNK_ALIGN_1], chunks[CHUNK_DATA_1].iov_base, data_len);
				vectshrink(&chunks[CHUNK_DATA_1], data_len);
	printk(KERN_DEBUG ">>> 6.3.2 copy %u bytes from TRAILER to ALIGN_1 buffer\n", len);
				vectcpy(&chunks[CHUNK_ALIGN_1], chunks[CHUNK_TRAILER].iov_base, len);
				vectmov(&chunks[CHUNK_TRAILER], len);
	printk(KERN_DEBUG ">>> 6.3.2 copy %u bytes from TRAILER to REM buffer\n", chunks[CHUNK_TRAILER].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			} else {
				/* the frame is aligned to sector boundary, but DATA_1 and TRAILER buffers may be
				 * unaligned to ALIGNMENT_SIZE boundary */
				data_len = chunks[CHUNK_DATA_1].iov_len % ALIGNMENT_SIZE;
				if (data_len >= chunks[CHUNK_TRAILER].iov_len) {
		printk(KERN_DEBUG ">>> 6.3.3 copy %u bytes from DATA_1 to ALIGN_1 buffer\n", data_len);
					src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
					vectcpy(&chunks[CHUNK_ALIGN_1], src, data_len);
					vectshrink(&chunks[CHUNK_DATA_1], data_len);
		printk(KERN_DEBUG ">>> 6.3.3 copy %u bytes from TRAILER to ALIGN_1 buffer\n", chunks[CHUNK_TRAILER].iov_len);
					vectcpy(&chunks[CHUNK_ALIGN_1], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
					vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
				} else {
					/* break trailing marker and copy the remaining data to REM buffer */
					data_len = chunks[CHUNK_DATA_1].iov_len % ALIGNMENT_SIZE - len;
		printk(KERN_DEBUG ">>> 6.3.4 copy %u bytes from DATA_1 to ALIGN_1 buffer\n", data_len);
					src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
					vectcpy(&chunks[CHUNK_ALIGN_1], chunks[CHUNK_DATA_1].iov_base, data_len);
					vectshrink(&chunks[CHUNK_DATA_1], data_len);
		printk(KERN_DEBUG ">>> 6.3.4 copy %u bytes from TRAILER to ALIGN_1 buffer\n", len);
					vectcpy(&chunks[CHUNK_ALIGN_1], chunks[CHUNK_TRAILER].iov_base, len);
					vectmov(&chunks[CHUNK_TRAILER], len);
		printk(KERN_DEBUG ">>> 6.3.4 copy %u bytes from TRAILER to REM buffer\n", chunks[CHUNK_TRAILER].iov_len);
					vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
					vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
				}
			}
		}
	}

	/* debug sanity check, should not happen */
	if (cbuff->iov_len >= max_len) {
		dev_err(NULL, "ERROR: the number of bytes copied to common buffer exceeds its size\n");
	}
}

/** TEST FUNCTION: stuff frame data to align the frame to disk block boundary */
//static void stuff_frame(struct fvec *vects)
//{
//	int i;
//	size_t total = 0;
//	size_t stuffing = 0;
//
//	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
//		total += vects[i].iov_len;
//	}
//
//	stuffing = PHY_BLOCK_SIZE - total % PHY_BLOCK_SIZE;
//
//	printk(KERN_DEBUG "%s: total = %u, stuffing = %u\n", __func__, total, stuffing);
//	if (stuffing == PHY_BLOCK_SIZE)
//		return;
//
//	if (stuffing < 3) {
//		// the number of stuffing bytes is less then marker plus one byte, add one more sector
//		stuffing += PHY_BLOCK_SIZE;
//	}
//	vects[CHUNK_STUFFING].iov_len = stuffing;
//}

static void dump_sg_list(const struct fvec *sgl, size_t elems)
{
	int i;

	printk(KERN_DEBUG "dump S/G list, %u elements:\n", elems);
	for (i = 0; i < elems; i++) {
		printk(KERN_DEBUG "dma address: 0x%x, len: %u\n", sgl[i].iov_dma, sgl[i].iov_len);
	}
	printk(KERN_DEBUG "===== end of S/G list =====\n");
}

/** Calculate the number of blocks this frame will occupy. The frame must be aligned to block size */
static inline size_t get_blocks_num(struct fvec *sgl, size_t n_elem)
{
	int num;
	size_t total = 0;

	for (num = 0; num < n_elem; num++) {
		total += sgl[num].iov_len;
	}

	return total / PHY_BLOCK_SIZE;
}

/** Calculate the size of current frame in bytes starting from vector and offset given */
static inline size_t get_size_from(const struct fvec *vects, int index, size_t offset, int all)
{
	int i;
	size_t total = 0;

	if (index >= MAX_DATA_CHUNKS || offset > vects[index].iov_len) {
		dev_dbg(NULL, "nothing to process, index or offset is out of vector range: vector %d, offset %u\n", index, offset);
		return 0;
	}

	for (i = index; i < MAX_DATA_CHUNKS; i++) {
		if (i == CHUNK_REM && all == EXCLUDE_REM)
			/* remainder should not be processed */
			continue;
		if (i == index)
			total += vects[i].iov_len - offset;
		else
			total += vects[i].iov_len;
	}

	return total;
}

/** Set vectors pointing to data buffers except for JPEG data - those are set in circbuf driver */
static void init_vectors(struct elphel_ahci_priv *dpriv)
{
	struct frame_buffers *buffs = &dpriv->fbuffs;
	struct fvec *chunks = dpriv->data_chunks;

	chunks[CHUNK_EXIF].iov_base = buffs->exif_buff.iov_base;
	chunks[CHUNK_EXIF].iov_len = 0;

	chunks[CHUNK_LEADER].iov_base = buffs->jpheader_buff.iov_base;
	chunks[CHUNK_LEADER].iov_len = JPEG_MARKER_LEN;
	chunks[CHUNK_HEADER].iov_base = (unsigned char *)chunks[CHUNK_LEADER].iov_base + chunks[CHUNK_LEADER].iov_len;
	chunks[CHUNK_HEADER].iov_len = 0;

	chunks[CHUNK_TRAILER].iov_base = buffs->trailer_buff.iov_base;
	chunks[CHUNK_TRAILER].iov_len = JPEG_MARKER_LEN;

	chunks[CHUNK_REM].iov_base = buffs->rem_buff.iov_base;
	chunks[CHUNK_REM].iov_len = 0;

	/* this is the only DMA mapped buffer and its DMA address should be set */
	chunks[CHUNK_COMMON].iov_base = buffs->common_buff.iov_base;
	chunks[CHUNK_COMMON].iov_dma = buffs->common_buff.iov_dma;
	chunks[CHUNK_COMMON].iov_len = 0;
}

static int init_buffers(struct device *dev, struct frame_buffers *buffs)
{
	int mult;
	int total_sz;
	unsigned char *ptr;

	buffs->exif_buff.iov_base = kmalloc(MAX_EXIF_SIZE, GFP_KERNEL);
	if (!buffs->exif_buff.iov_base)
		return -ENOMEM;
	buffs->exif_buff.iov_len = MAX_EXIF_SIZE;

	buffs->jpheader_buff.iov_base = kmalloc(JPEG_HEADER_MAXSIZE, GFP_KERNEL);
	if (!buffs->jpheader_buff.iov_base)
		goto err_header;
	buffs->jpheader_buff.iov_len = JPEG_HEADER_MAXSIZE;

	buffs->trailer_buff.iov_base = kmalloc(JPEG_MARKER_LEN, GFP_KERNEL);
	if (!buffs->trailer_buff.iov_base)
		goto err_trailer;
	buffs->trailer_buff.iov_len = JPEG_MARKER_LEN;
	ptr = buffs->trailer_buff.iov_base;
	ptr[0] = 0xff;
	ptr[1] = 0xd9;

	/* 3 * ALIGMENT_SIZE here means 2 buffers for JPEG data alignment plus one buffer for
	 * DATA_0 address alignment - this one is padded with APP15 marker */
	total_sz = MAX_EXIF_SIZE + JPEG_HEADER_MAXSIZE + 4 * ALIGNMENT_SIZE + PHY_BLOCK_SIZE;
	if (total_sz > PAGE_SIZE) {
		mult = total_sz / PAGE_SIZE + 1;
		total_sz = mult * PAGE_SIZE;
	} else {
		total_sz = PAGE_SIZE;
	}
	buffs->common_buff.iov_base = kmalloc(total_sz, GFP_KERNEL);
	if (!buffs->common_buff.iov_base)
		goto err_common;
	buffs->common_buff.iov_len = total_sz;
	/* this is the only buffer which needs DMA mapping as all other data will be collected in it */
	buffs->common_buff.iov_dma = dma_map_single(dev, buffs->common_buff.iov_base, buffs->common_buff.iov_len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, buffs->common_buff.iov_dma))
		goto err_common_dma;

	buffs->rem_buff.iov_base = kmalloc(PHY_BLOCK_SIZE, GFP_KERNEL);
	if (!buffs->rem_buff.iov_base)
		goto err_remainder;
	buffs->rem_buff.iov_len = PHY_BLOCK_SIZE;

	/* debug code follows */
	g_jpg_data_0 = kzalloc(DATA_BUFF_SIZE, GFP_KERNEL);
	g_jpg_data_1 = kzalloc(DATA_BUFF_SIZE, GFP_KERNEL);
	if (!g_jpg_data_0 || !g_jpg_data_1)
		return -ENOMEM;
	/* end of debug code */

	return 0;

err_remainder:
	dma_unmap_single(dev, buffs->common_buff.iov_dma, buffs->common_buff.iov_len, DMA_TO_DEVICE);
err_common_dma:
	kfree(buffs->common_buff.iov_base);
err_common:
	kfree(buffs->trailer_buff.iov_base);
err_trailer:
	kfree(buffs->jpheader_buff.iov_base);
err_header:
	kfree(buffs->exif_buff.iov_base);
	return -ENOMEM;
}

static void deinit_buffers(struct device *dev, struct frame_buffers *buffs)
{
	kfree(buffs->jpheader_buff.iov_base);
	kfree(buffs->exif_buff.iov_base);
	kfree(buffs->trailer_buff.iov_base);
	dma_unmap_single(dev, buffs->common_buff.iov_dma, buffs->common_buff.iov_len, DMA_TO_DEVICE);
	kfree(buffs->common_buff.iov_base);
	kfree(buffs->rem_buff.iov_base);
}

static inline void reset_chunks(struct fvec *vects, int all)
{
	int i;

	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
		if (i != CHUNK_REM)
			vects[i].iov_len = 0;
	}
	if (all) {
		vects[CHUNK_REM].iov_len = 0;
	}
}

static inline struct elphel_ahci_priv *dev_get_dpriv(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return dpriv;
}

//static void start_cmd(struct device *dev, struct elphel_ahci_priv *dpriv, struct ata_port *port)
//{
//	int num;
//	size_t max_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
//	size_t total_sz = get_total_size(dpriv->data_chunks);
//
//	if ((dpriv->lba_ptr.lba_write & ~ADDR_MASK_28_BIT) || total_sz > max_sz) {
////	if (dpriv->lba_ptr.lba_write & ~ADDR_MASK_28_BIT) {
//		dpriv->curr_cmd = ATA_CMD_WRITE_EXT;
//		dpriv->max_data_sz = (MAX_LBA_COUNT_EXT + 1) * PHY_BLOCK_SIZE;
//	} else {
//		dpriv->curr_cmd = ATA_CMD_WRITE;
//		dpriv->max_data_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
//	}
//	dpriv->flags |= PROC_CMD;
//	dpriv->sg_elems = map_vectors(dpriv);
//
//	num = dma_map_sg(dev, dpriv->sgl, dpriv->sg_elems, DMA_TO_DEVICE);
//	printk(KERN_DEBUG ">>> %d entries dma mapped\n", num);
//	dump_sg_list(dpriv->sgl, dpriv->sg_elems);
//
//	dpriv->lba_ptr.wr_count = get_blocks_num(dpriv->sgl, dpriv->sg_elems);
//	printk(KERN_DEBUG ">>> trying to write data from sg list %u blocks, LBA: %llu\n", dpriv->lba_ptr.wr_count, dpriv->lba_ptr.lba_write);
//	elphel_cmd_issue(port, dpriv->lba_ptr.lba_write, dpriv->lba_ptr.wr_count, dpriv->sgl, dpriv->sg_elems, dpriv->curr_cmd);
//}

/** Process command and return the number of S/G entries mapped */
static int process_cmd(struct device *dev, struct elphel_ahci_priv *dpriv, struct ata_port *port)
{
	int num;
	struct fvec *cbuff = &dpriv->data_chunks[CHUNK_COMMON];
	size_t max_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
	size_t rem_sz = get_size_from(dpriv->data_chunks, dpriv->curr_data_chunk, dpriv->curr_data_offset, EXCLUDE_REM);

	/* define ATA command to use for current transaction */
	if ((dpriv->lba_ptr.lba_write & ~ADDR_MASK_28_BIT) || rem_sz > max_sz) {
		dpriv->curr_cmd = ATA_CMD_WRITE_EXT;
		dpriv->max_data_sz = (MAX_LBA_COUNT_EXT + 1) * PHY_BLOCK_SIZE;
	} else {
		dpriv->curr_cmd = ATA_CMD_WRITE;
		dpriv->max_data_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
	}

	if (dpriv->flags & PROC_CMD)
		dpriv->lba_ptr.lba_write += dpriv->lba_ptr.wr_count;
	dpriv->flags |= PROC_CMD;
	dpriv->sg_elems = map_vectors(dpriv);
	if (dpriv->sg_elems != 0) {
		dump_sg_list(dpriv->sgl, dpriv->sg_elems);

		dpriv->lba_ptr.wr_count = get_blocks_num(dpriv->sgl, dpriv->sg_elems);
		printk(KERN_DEBUG ">>> trying to write data from sg list %u blocks, LBA: %llu\n", dpriv->lba_ptr.wr_count, dpriv->lba_ptr.lba_write);
		dma_sync_single_for_device(dev, cbuff->iov_dma, cbuff->iov_len, DMA_TO_DEVICE);
		if (dpriv->data_chunks[CHUNK_DATA_0].iov_len != 0)
			dma_sync_single_for_device(dev, dpriv->data_chunks[CHUNK_DATA_0].iov_dma, dpriv->data_chunks[CHUNK_DATA_0].iov_len, DMA_TO_DEVICE);
		if (dpriv->data_chunks[CHUNK_DATA_1].iov_len != 0)
			dma_sync_single_for_device(dev, dpriv->data_chunks[CHUNK_DATA_1].iov_dma, dpriv->data_chunks[CHUNK_DATA_1].iov_len, DMA_TO_DEVICE);
		elphel_cmd_issue(port, dpriv->lba_ptr.lba_write, dpriv->lba_ptr.wr_count, dpriv->sgl, dpriv->sg_elems, dpriv->curr_cmd);
	}

	return dpriv->sg_elems;
}

static void finish_cmd(struct device *dev, struct elphel_ahci_priv *dpriv)
{
	int all;

	dpriv->lba_ptr.wr_count = 0;
	if ((dpriv->flags & LAST_BLOCK) == 0) {
		all = 0;
	} else {
		all = 1;
		dpriv->flags &= ~LAST_BLOCK;
	}
	reset_chunks(dpriv->data_chunks, all);
	dpriv->flags &= ~PROC_CMD;
	dpriv->curr_cmd = 0;
	dpriv->max_data_sz = 0;
	dpriv->curr_data_chunk = 0;
	dpriv->curr_data_offset = 0;
}

/** Fill free space in REM buffer with 0 and save the reaming data chunk */
static void finish_rec(struct device *dev, struct elphel_ahci_priv *dpriv, struct ata_port *port)
{
	size_t stuff_len;
	struct fvec *src;
	struct fvec *cbuff = &dpriv->data_chunks[CHUNK_COMMON];
	struct fvec *rbuff = &dpriv->data_chunks[CHUNK_REM];

	if (rbuff->iov_len == 0)
		return;

	dev_dbg(dev, "write last chunk of data, size: %u\n", rbuff->iov_len);
	stuff_len = PHY_BLOCK_SIZE - rbuff->iov_len;
	src = vectrpos(rbuff, stuff_len);
	memset(src->iov_base, 0, stuff_len);
	vectcpy(cbuff, rbuff->iov_base, rbuff->iov_len);
	vectshrink(rbuff, rbuff->iov_len);

	dpriv->flags |= LAST_BLOCK;
	process_cmd(dev, dpriv, port);
}

static ssize_t rawdev_write(struct device *dev,  ///<
		struct device_attribute *attr,           ///<
		const char *buff,                        ///<
		size_t buff_sz)                          ///<
{
	int i, n_elem;
	int sg_elems = 0;
	struct ata_host *host = dev_get_drvdata(dev);
	struct ata_port *port = host->ports[DEFAULT_PORT_NUM];
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);
	struct scatterlist *sgl;
	struct scatterlist *sg_ptr;
//	u8 *test_buff = pElphel_buf->d2h_vaddr;
	u8 *test_buff;
	uint8_t *buffers[SG_TBL_SZ] = {0};
	uint64_t lba_addr;
	struct frame_data fdata;
	size_t rcvd = 0;
	struct frame_buffers *buffs = &dpriv->fbuffs;
	struct fvec *chunks = dpriv->data_chunks;
	size_t blocks_num;
	static int dont_process = 0;

	if ((dpriv->flags & PROC_CMD) || dont_process)
		// we are not ready yet
		return -EAGAIN;

	if (buff_sz != sizeof(struct frame_data)) {
		dev_err(dev, "the size of the data buffer is incorrect, should be equal to sizeof(struct data_page)\n");
		return -EINVAL;
	}
	memcpy(&fdata, buff, sizeof(struct frame_data));

	/* debug code follows */
	printk(KERN_DEBUG ">>> data pointers received:\n");
	printk(KERN_DEBUG ">>> sensor port: %u\n", fdata.sensor_port);
	printk(KERN_DEBUG ">>> cirbuf ptr: %d, cirbuf data len: %d\n", fdata.cirbuf_ptr, fdata.jpeg_len);
	printk(KERN_DEBUG ">>> meta_index: %d\n", fdata.meta_index);
	printk(KERN_DEBUG "\n");

//	rcvd = exif_get_data(fdata.sensor_port, fdata.meta_index, buffs->exif_buff.iov_base, buffs->exif_buff.iov_len);
	rcvd = exif_get_data_tst(fdata.sensor_port, fdata.meta_index, buffs->exif_buff.iov_base, buffs->exif_buff.iov_len, 1);
	printk(KERN_DEBUG ">>> bytes received from exif driver: %u\n", rcvd);
	if (rcvd > 0 && rcvd < buffs->exif_buff.iov_len)
//		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buffs->exif_buff.iov_base, rcvd);
		printk(KERN_DEBUG ">>>\tskipped");
	chunks[CHUNK_EXIF].iov_len = rcvd;

//	rcvd = jpeghead_get_data(fdata.sensor_port, buffs->jpheader_buff.iov_base, buffs->jpheader_buff.iov_len, 0);
	rcvd = jpeghead_get_data_tst(fdata.sensor_port, buffs->jpheader_buff.iov_base, buffs->jpheader_buff.iov_len, 0);
	printk(KERN_DEBUG ">>> bytes received from jpeghead driver: %u\n", rcvd);
	if (rcvd > 0 && rcvd < buffs->jpheader_buff.iov_len) {
//		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buffs->jpheader_buff.iov_base, rcvd);
		chunks[CHUNK_LEADER].iov_len = JPEG_MARKER_LEN;
		chunks[CHUNK_TRAILER].iov_len = JPEG_MARKER_LEN;
		chunks[CHUNK_HEADER].iov_len = rcvd - chunks[CHUNK_LEADER].iov_len;
	} else {
		// we don't want these buffers for test purposes
		chunks[CHUNK_LEADER].iov_len = 0;
		chunks[CHUNK_TRAILER].iov_len = 0;
		chunks[CHUNK_HEADER].iov_len = 0;
	}

	rcvd = 0;
//	rcvd = circbuf_get_ptr(fdata.sensor_port, fdata.cirbuf_ptr, fdata.jpeg_len, &chunks[CHUNK_DATA_0], &chunks[CHUNK_DATA_1]);
	rcvd = circbuf_get_ptr_tst(fdata.sensor_port, fdata.cirbuf_ptr, fdata.jpeg_len, &chunks[CHUNK_DATA_0], &chunks[CHUNK_DATA_1]);
	if (rcvd > 0) {
		printk(KERN_DEBUG ">>> number of jpeg data pointers: %d\n", rcvd);
		printk(KERN_DEBUG ">>> bytes received from circbuf driver, chunk 0: %u\n", chunks[CHUNK_DATA_0].iov_len);
		if (rcvd == 2)
			printk(KERN_DEBUG ">>> bytes received from circbuf driver, chunk 1: %u\n", chunks[CHUNK_DATA_1].iov_len);
	}
	if (chunks[CHUNK_DATA_0].iov_len != 0)
		chunks[CHUNK_DATA_0].iov_dma = dma_map_single(dev, chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len, DMA_TO_DEVICE);
	if (chunks[CHUNK_DATA_1].iov_len != 0)
		chunks[CHUNK_DATA_1].iov_dma = dma_map_single(dev, chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len, DMA_TO_DEVICE);

	printk(KERN_DEBUG ">>> unaligned frame dump:\n");
	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
		printk(KERN_DEBUG ">>>\tslot: %i; len: %u\n", i, dpriv->data_chunks[i].iov_len);
	}
	align_frame(dev, dpriv);
	printk(KERN_DEBUG ">>> aligned frame dump:\n");
	for (i = 0; i < MAX_DATA_CHUNKS; i++) {
		printk(KERN_DEBUG ">>>\tslot: %i; len: %u\n", i, dpriv->data_chunks[i].iov_len);
	}
	if (check_chunks(dpriv->data_chunks) != 0) {
		dont_process = 1;
		return -EINVAL;
	}

	process_cmd(dev, dpriv, port);
	while (dpriv->flags & PROC_CMD) {
#ifndef DEBUG_DONT_WRITE
		while (dpriv->flags & IRQ_SIMPLE) {
			printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
			msleep_interruptible(1);
		}
#endif
		printk(KERN_DEBUG ">>> proceeding to next cmd chunk\n");
		sg_elems = process_cmd(dev, dpriv, port);
		if (sg_elems == 0)
			finish_cmd(dev, dpriv);
	}
	if (chunks[CHUNK_DATA_0].iov_len != 0)
		dma_unmap_single(dev, chunks[CHUNK_DATA_0].iov_dma, chunks[CHUNK_DATA_0].iov_len, DMA_TO_DEVICE);
	if (chunks[CHUNK_DATA_1].iov_len != 0)
		dma_unmap_single(dev, chunks[CHUNK_DATA_1].iov_dma, chunks[CHUNK_DATA_1].iov_len, DMA_TO_DEVICE);

//	chunks[CHUNK_STUFFING].iov_base = buffs->stuff_buff.iov_base;
//	stuff_frame(chunks);
//
//	/* copy data to common buffer */
//	struct fvec vec = {0};
//	vec.iov_base = buffs->common_buff.iov_base;
//	memcpy(vec.iov_base, chunks[CHUNK_LEADER].iov_base, chunks[CHUNK_LEADER].iov_len);
//	vec.iov_len += chunks[CHUNK_LEADER].iov_len;
//	chunks[CHUNK_LEADER].iov_len = 0;
//
//	memcpy(vec.iov_base + vec.iov_len, chunks[CHUNK_EXIF].iov_base, chunks[CHUNK_EXIF].iov_len);
//	vec.iov_len += chunks[CHUNK_EXIF].iov_len;
//	chunks[CHUNK_EXIF].iov_len = 0;
//
//	memcpy(vec.iov_base + vec.iov_len, chunks[CHUNK_HEADER].iov_base, chunks[CHUNK_HEADER].iov_len);
//	vec.iov_len += chunks[CHUNK_HEADER].iov_len;
//	chunks[CHUNK_HEADER].iov_len = 0;
//
//	memcpy(vec.iov_base + vec.iov_len, chunks[CHUNK_STUFFING].iov_base, chunks[CHUNK_STUFFING].iov_len);
//	vec.iov_len += chunks[CHUNK_STUFFING].iov_len;
//	chunks[CHUNK_STUFFING].iov_len = 0;
//
//	chunks[CHUNK_HEADER] = vec;

	/* end of debug code */

//	start_cmd(dev, dpriv, port);
//	while (dpriv->flags & PROC_CMD) {
//		while (dpriv->flags & IRQ_SIMPLE) {
//			printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
//			msleep_interruptible(1);
//		}
//		printk(KERN_DEBUG ">>> proceeding to next cmd chunk\n");
////		if (proc_cmd(dev, dpriv, port) == 0)
//			finish_cmd(dev, dpriv);
//	}



//	/* prepare buffer and fill it with markers */
//	sgl = kmalloc(sizeof(struct scatterlist) * SG_TBL_SZ, GFP_KERNEL);
//	if (!sgl)
//		return ENOMEM;
//	sg_init_table(sgl, SG_TBL_SZ);
//	for_each_sg(sgl, sg_ptr, SG_TBL_SZ, n_elem) {
//		test_buff = kmalloc(TEST_BUFF_SZ, GFP_KERNEL);
//		if (!test_buff)
//			return ENOMEM;
//		buffers[n_elem] = test_buff;
//		memset(test_buff, 0xa5, TEST_BUFF_SZ);
//		sg_set_buf(sg_ptr, (void *)test_buff, TEST_BUFF_SZ);
//		sg_elems++;
//	}
//
//	printk(KERN_DEBUG ">>> mapped %d SG elemets\n", sg_elems);
//	printk(KERN_DEBUG ">>>\n");
//
//	/* write test #2 */
//	if (dpriv->lba_ptr.lba_write & ~ADDR_MASK_28_BIT) {
//		dpriv->curr_cmd = ATA_CMD_WRITE_EXT;
//		dpriv->max_data_sz = (0xffff + 1) * PHY_BLOCK_SIZE;
//	} else {
//		dpriv->curr_cmd = ATA_CMD_WRITE;
//		dpriv->max_data_sz = (0xff + 1) * PHY_BLOCK_SIZE;
//	}
//	dpriv->flags |= PROC_CMD;
//	blocks_num = get_blocks_num(sgl, sg_elems);
//	i = dma_map_sg(dev, sgl, sg_elems, DMA_TO_DEVICE);
//	printk(KERN_DEBUG ">>> dma mapped %d elements\n", i);
//	printk(KERN_DEBUG ">>> trying to write data from sg list, %u blocks, LBA: %llu\n", blocks_num, dpriv->lba_ptr.lba_write);
//	elphel_cmd_issue(port, dpriv->lba_ptr.lba_write, blocks_num, sgl, sg_elems, dpriv->curr_cmd);
//
//	while (dpriv->flags & IRQ_SIMPLE) {
//		printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
//		msleep_interruptible(1);
//	}
//	dma_unmap_sg(dev, sgl, sg_elems, DMA_TO_DEVICE);
//	dpriv->lba_ptr.lba_write += blocks_num;
//	dpriv->flags &= ~PROC_CMD;
//	/* end of write test #2 */
//
//	for (i = 0; i < sg_elems; i++) {
//		kfree(buffers[i]);
//	}
//	kfree(sgl);


//
//	/* read test */
//	dma_map_sg(dev, sgl, sg_elems, DMA_FROM_DEVICE);
//
//	printk(KERN_DEBUG ">>> trying to read data to sg list\n");
//	if (lba_addr & ~ADDR_MASK_28_BIT)
//		cmd = ATA_CMD_READ_EXT;
//	else
//		cmd = ATA_CMD_READ;
//	elphel_cmd_issue(port, lba_addr, sg_elems, sgl, sg_elems, cmd);
//	printk(KERN_DEBUG ">>> command has been issued\n");
//
//	while (dpriv->flags & IRQ_SIMPLE) {
//		printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
//		msleep(1);
//	}
//
//	printk(KERN_DEBUG ">>> dump test buffer after reading: %d bytes\n", TEST_BUFF_SZ);
//	dma_unmap_sg(dev, sgl, sg_elems, DMA_FROM_DEVICE);
//	for (i = 0; i < sg_elems; i++) {
//		dev_dbg(dev, ">>> sector %i\n", i);
//		u8 buff[TEST_BUFF_SZ];
//		sg_copy_to_buffer(&sgl[i], 1, buff, TEST_BUFF_SZ);
//		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buff, TEST_BUFF_SZ);
//	}
//	/* end of read test */
//
//	printk(KERN_DEBUG ">>> *** proceeding to write test *** <<<\n");
//
//	/* write test */
//	for_each_sg(sgl, sg_ptr, SG_TBL_SZ, n_elem) {
//		uint8_t pattern_buff[TEST_BUFF_SZ];
//		memset(pattern_buff, 0xb6, TEST_BUFF_SZ);
//		sg_copy_from_buffer(sg_ptr, 1, pattern_buff, TEST_BUFF_SZ);
//	}

//	if (lba_addr & ~ADDR_MASK_28_BIT) {
//		dpriv->curr_cmd = ATA_CMD_WRITE_EXT;
//		dpriv->max_data_sz = (0xffff + 1) * PHY_BLOCK_SIZE;
//	} else {
//		dpriv->curr_cmd = ATA_CMD_WRITE;
//		dpriv->max_data_sz = (0xff + 1) * PHY_BLOCK_SIZE;
//	}
//	sg_elems = map_vectors(sgl, dpriv, dpriv->max_data_sz);
//	dump_sg_list(sgl, sg_elems);
//	while (sg_elems != 0) {
//		dma_map_sg(dev, sgl, sg_elems, DMA_TO_DEVICE);
//		lba_addr = dpriv->lba_ptr.lba_write;
//		blocks_num = get_blocks_num(sgl, sg_elems);
//		printk(KERN_DEBUG ">>> trying to write data from sg list %u blocks, LBA: %llu\n", blocks_num, lba_addr);
//		elphel_cmd_issue(port, lba_addr, blocks_num, sgl, sg_elems, cmd);
//		printk(KERN_DEBUG ">>> command has been issued, wrting %u LBAs\n", blocks_num);
//
//		while (dpriv->flags & IRQ_SIMPLE) {
//			printk_once(KERN_DEBUG ">>> waiting for interrupt\n");
//			msleep(1);
//		}
//		dma_unmap_sg(dev, sgl, sg_elems, DMA_TO_DEVICE);
//		dpriv->lba_ptr.lba_write += blocks_num;
//
//		sg_elems = map_vectors(sgl, dpriv, dpriv->max_data_sz);
//		dump_sg_list(sgl, sg_elems);
//	}
//	finish_cmd(dpriv);

//	printk(KERN_DEBUG ">>> dump test buffer after writing: %d bytes\n", TEST_BUFF_SZ);
//	for (i = 0; i < sg_elems; i++) {
//		dev_dbg(dev, ">>> sector %i\n", i);
//		u8 buff[TEST_BUFF_SZ];
//		sg_copy_to_buffer(&sgl[i], 1, buff, TEST_BUFF_SZ);
//		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buff, TEST_BUFF_SZ);
//	}
	/* end of write test */

	return buff_sz;
}

/** Prepare software constructed command FIS in command table area. The structure of the
 * command FIS is described in Transport Layer chapter of Serial ATA revision 3.1 documentation.
 */
static inline void prep_cfis(uint8_t *cmd_tbl,   ///< pointer to the beginning of command table
		uint8_t cmd,                             ///< ATA command as described in ATA/ATAPI command set
		uint64_t start_addr,                     ///< LBA start address
		uint16_t count)                          ///< sector count, the number of 512 byte sectors to read or write
		                                         ///< @return None
{
	uint8_t device, ctrl;

	/* select the content of Device and Control registers based on command, read the description of
	 * a command in ATA/ATAPI command set documentation
	 */
	switch (cmd) {
	case ATA_CMD_WRITE:
	case ATA_CMD_READ:
		device = 0xe0 | ((start_addr >> 24) & 0x0f);
		ctrl = 0x08;
		/* this is 28-bit command; 4 bits of the address have already been
		 * placed to Device register, invalidate the remaining (if any) upper
		 * bits of the address and leave only 24 significant bits (just in case)
		 */
		start_addr &= 0xffffff;
		count &= 0xff;
		break;
	case ATA_CMD_WRITE_EXT:
	case ATA_CMD_READ_EXT:
		device = 0xe0;
		ctrl = 0x08;
		break;
	default:
		device = 0xe0;
		ctrl = 0x08;
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
static inline void prep_prdt(struct fvec *sgl,   ///< pointer to S/G list which should be mapped to physical
		                                         ///< region description table
		unsigned int n_elem,                     ///< the number of elements in @e sgl
		struct ahci_sg *ahci_sgl)                ///< pointer to physical region description table
		                                         ///< @return None
{
	unsigned int num = 0;

	for (num = 0; num < n_elem; num++) {
		ahci_sgl[num].addr = cpu_to_le32(sgl[num].iov_dma & 0xffffffff);
		ahci_sgl[num].addr_hi = cpu_to_le32((sgl[num].iov_dma >> 16) >> 16);
		ahci_sgl[num].flags_size = cpu_to_le32(sgl[num].iov_len - 1);
	}
}

/** Prepare and issue read or write command */
static void elphel_cmd_issue(struct ata_port *ap,///< device port for which the command should be issued
		uint64_t start,                          ///< LBA start address
		uint16_t count,                          ///< the number of sectors to read or write
		struct fvec *sgl,                 ///< S/G list pointing to data buffers
		unsigned int elem,                       ///< the number of elements in @e sgl
		uint8_t cmd)                             ///< the command to be issued; should be ATA_CMD_READ, ATA_CMD_READ_EXT,
		                                         ///< ATA_CMD_WRITE or ATA_CMD_WRITE_EXT, other commands are not tested
		                                         ///< @return None
{
	uint32_t opts;
	uint8_t *cmd_tbl;
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
	prep_cfis(cmd_tbl, cmd, start, count);

	/* prepare physical region descriptor table */
	ahci_sg = pp->cmd_tbl + slot_num * AHCI_CMD_TBL_SZ + AHCI_CMD_TBL_HDR_SZ;
	prep_prdt(sgl, elem, ahci_sg);

	/* prepare command header */
	opts = CMD_FIS_LEN | (elem << 16) | AHCI_CMD_PREFETCH | AHCI_CMD_CLR_BUSY;
	if (cmd == ATA_CMD_WRITE || cmd == ATA_CMD_WRITE_EXT)
		opts |= AHCI_CMD_WRITE;
	ahci_fill_cmd_slot(pp, slot_num, opts);

	dev_dbg(ap->dev, ">>> dump command table content, first %d bytes, phys addr = 0x%x:\n", 20, pp->cmd_tbl_dma);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, pp->cmd_tbl, 20);

	dma_sync_single_for_device(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);

	/* debug code follows */
#ifdef DEBUG_DONT_WRITE
	return;
#endif
	/* end of debug code */

	/* issue command */
	writel(0x11, port_mmio + PORT_CMD);
	writel(1 << slot_num, port_mmio + PORT_CMD_ISSUE);
}

static ssize_t lba_start_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_start);
}

static ssize_t lba_start_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	if (kstrtoull(buff, 10, &dpriv->lba_ptr.lba_start) != 0)
		return -EINVAL;

	if (dpriv->lba_ptr.lba_write < dpriv->lba_ptr.lba_start)
		dpriv->lba_ptr.lba_write = dpriv->lba_ptr.lba_start;

	return buff_sz;
}

static ssize_t lba_end_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_end);
}

static ssize_t lba_end_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	if (kstrtoull(buff, 10, &dpriv->lba_ptr.lba_end) != 0)
		return -EINVAL;

	if (dpriv->lba_ptr.lba_write > dpriv->lba_ptr.lba_end)
		dpriv->lba_ptr.lba_write = dpriv->lba_ptr.lba_end;

	return buff_sz;
}

static ssize_t lba_current_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_write);
}

static ssize_t lba_current_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	if (kstrtoull(buff, 10, &dpriv->lba_ptr.lba_write) != 0)
		return -EINVAL;

	return buff_sz;
}

static DEVICE_ATTR(load_module, S_IWUSR | S_IWGRP, NULL, set_load_flag);
static DEVICE_ATTR(write, S_IWUSR | S_IWGRP, NULL, rawdev_write);
static DEVICE_ATTR(lba_start, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, lba_start_read, lba_start_write);
static DEVICE_ATTR(lba_end, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, lba_end_read, lba_end_write);
static DEVICE_ATTR(lba_current, S_IRUSR | S_IRGRP | S_IWUSR | S_IRGRP, lba_current_read, lba_current_write);
static struct attribute *root_dev_attrs[] = {
		&dev_attr_load_module.attr,
		&dev_attr_write.attr,
		&dev_attr_lba_start.attr,
		&dev_attr_lba_end.attr,
		&dev_attr_lba_current.attr,
		&dev_attr_data_0_sz.attr,
		&dev_attr_data_1_sz.attr,
		&dev_attr_data_proc.attr,
		&dev_attr_exif_sz.attr,
		&dev_attr_jpg_hdr_sz.attr,
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Elphel AHCI SATA platform driver for elphel393 camera");
