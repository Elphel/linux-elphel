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

/* this one is required for printk_ratelimited */
#define CONFIG_PRINK

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
#include <elphel/exifa.h>
#include <elphel/elphel393-mem.h>

#include "ahci.h"
#include "ahci_elphel.h"
#include "../elphel/exif393.h"
#include "../elphel/jpeghead.h"

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

#define DATASCOPE_OFFSET    0x1000
#define DATASCOPE_SIZE      0x200

static struct ata_port_operations ahci_elphel_ops;
static const struct ata_port_info ahci_elphel_port_info;
static struct scsi_host_template ahci_platform_sht;
static const struct of_device_id ahci_elphel_of_match[];
static const struct attribute_group dev_attr_root_group;

static bool load_driver = false;
static unsigned char app15[ALIGNMENT_SIZE] = {0xff, 0xef};

static void elphel_cmd_issue(struct ata_port *ap, uint64_t start, uint16_t count, struct fvec *sgl, unsigned int elem, uint8_t cmd);
static int init_buffers(struct device *dev, struct frame_buffers *buffs);
static void init_vectors(struct frame_buffers *buffs, struct fvec *chunks);
static void deinit_buffers(struct device *dev, struct frame_buffers *buffs);
static inline struct elphel_ahci_priv *dev_get_dpriv(struct device *dev);
static void finish_cmd(struct elphel_ahci_priv *dpriv);
static void finish_rec(struct elphel_ahci_priv *dpriv);
static int process_cmd(struct elphel_ahci_priv *dpriv);
static inline size_t get_size_from(const struct fvec *vects, int index, size_t offset, int all);
static inline void vectmov(struct fvec *vec, size_t len);
static inline void vectsplit(struct fvec *vect, struct fvec *parts, size_t *n_elem);
static int move_tail(struct elphel_ahci_priv *dpriv);
static int move_head(struct elphel_ahci_priv *dpriv);
static size_t get_prev_slot(const struct elphel_ahci_priv *dpriv);
static int is_cmdq_empty(const struct elphel_ahci_priv *dpriv);
void process_queue(unsigned long data);
static void set_flag(struct elphel_ahci_priv *drpiv, uint32_t flag);
static void reset_flag(struct elphel_ahci_priv *dpriv, uint32_t flag);
static inline void reset_chunks(struct fvec *vects, int all);
/* debug functions */
static int check_chunks(struct fvec *vects);
static void dump_sg_list(const struct device *dev, const struct fvec *sgl, size_t elems);
static void dump_iomem(const struct device *dev, void __iomem *mmio, size_t sz, dma_addr_t dma_addr);
static void dump_dpriv_fields(struct elphel_ahci_priv *dpriv);
static void dump_current_state(struct elphel_ahci_priv *dpriv);
static void set_dscope_tstamp(struct elphel_ahci_priv *dpriv, unsigned int cmd);

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

/** Set command execution timer. This function is called before every internal command and
 * the command is considered stalled if the timer has expired. */
static void set_timer(struct elphel_ahci_priv *dpriv)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(dpriv->cmd_timeout);
	mod_timer(&dpriv->cmd_timer, timeout);
}

/** Remove command execution timer if it is still running */
static void remove_timer(struct elphel_ahci_priv *dpriv)
{
	int ret;

	if (timer_pending(&dpriv->cmd_timer)) {
		ret = del_timer_sync(&dpriv->cmd_timer);
		if (ret < 0)
			dev_err(dpriv->dev, "can not remove command guard timer\n");
		reset_flag(dpriv, IRQ_PROCESSED);
	}
}

/** This command resets all commands and flags indicating current state of the driver */
static void reset_all_commands(struct elphel_ahci_priv *dpriv)
{
	int i;
	unsigned long irq_flags;

	if (is_cmdq_empty(dpriv))
		return;

	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	for (i = 0; i < MAX_CMD_SLOTS; i++) {
		reset_chunks(dpriv->data_chunks[i], 1);
	}
	dpriv->flags = 0;
	dpriv->head_ptr = 0;
	dpriv->tail_ptr = 0;

	dpriv->lba_ptr.wr_count = 0;
	dpriv->curr_cmd = 0;
	dpriv->max_data_sz = 0;
	dpriv->curr_data_chunk = 0;
	dpriv->curr_data_offset = 0;
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
}

/** Command execution timer has expired, set flag indicating that recording should be restarted */
//static void process_timeout(unsigned long data)
static void process_timeout(struct timer_list *t)
{
	int report = 0;
	unsigned long irq_flags;
	struct elphel_ahci_priv *dpriv = from_timer(dpriv,t,cmd_timer);

	/* we are racing with interrupt here, IRQ handler sets IRQ_PROCESSED flag to prevent timer from
	 * triggering error handler before this timer is removed
	 */
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if (!(dpriv->flags & IRQ_PROCESSED)) {
		dpriv->flags |= START_EH;
		report = 1;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
	if (report) {
		set_dscope_tstamp(dpriv, TSTMP_CMD_SYS);
	}
}

/** Calculate the difference between two time stamps and return it in microseconds */
static unsigned long time_diff_usec(sec_usec_t *start_time, sec_usec_t *end_time)
{
	unsigned long time_us;
	const unsigned long scale = 1000000;

	if (start_time->sec <= end_time->sec) {
		time_us = (end_time->sec - start_time->sec) * scale;
	} else {
		// time counter has rolled over
		time_us = (ULONG_MAX - start_time->sec + end_time->sec) * scale;
	}
	if (start_time->usec <= end_time->usec)
		time_us += end_time->usec - start_time->usec;
	else
		time_us += scale - start_time->usec + end_time->usec;

	return time_us;
}

/** Add new recording speed sample to the list of samples */
static void add_sample(unsigned int sample, struct rec_stat *stat)
{
	stat->samples[stat->samples_ptr] = sample;
	stat->samples_ptr++;
	if (stat->samples_ptr >= SPEED_SAMPLES_NUM) {
		stat->samples_ptr = 0;
	}
}

static irqreturn_t elphel_irq_handler(int irq, void * dev_instance)
{
	unsigned long irq_flags;
	irqreturn_t handled;
	struct ata_host *host = dev_instance;
	struct ahci_host_priv *hpriv = host->private_data;
	struct ata_port *port = host->ports[DEFAULT_PORT_NUM];
	void __iomem *port_mmio = ahci_port_base(port);
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;
	uint32_t irq_stat, host_irq_stat, serror;

	if (dpriv->flags & IRQ_SIMPLE) {

		set_dscope_tstamp(dpriv, TSTMP_IRQ_DRV);

		/* handle interrupt from internal command */
		host_irq_stat = readl(hpriv->mmio + HOST_IRQ_STAT);
		if (!host_irq_stat) {
			dev_err(dpriv->dev, "Spurious IRQ received, host_irq_stat = 0x%x\n", host_irq_stat);
			return IRQ_NONE;
		}

		reset_flag(dpriv, IRQ_SIMPLE);
		irq_stat = readl(port_mmio + PORT_IRQ_STAT);
		serror =  readl(port_mmio + PORT_SCR_ERR);

		/* debug feature, save current state for analysis */
		dpriv->datascope.reg_stat[REG_HOST_IS] = host_irq_stat;
		dpriv->datascope.reg_stat[REG_PxIS] = irq_stat;
		dpriv->datascope.reg_stat[REG_PxSERR] = serror;
		dpriv->datascope.reg_stat[REG_PxIE] = readl(port_mmio + PORT_IRQ_MASK);
		dpriv->datascope.reg_stat[IRQ_COUNTER] = dpriv->datascope.reg_stat[IRQ_COUNTER] + 1;

		if (unlikely(irq_stat & PORT_IRQ_ERROR)) {
			dpriv->eh.irq_stat = irq_stat;
			dpriv->eh.s_error = serror;
			set_flag(dpriv, START_EH);
		}

		writel(irq_stat, port_mmio + PORT_IRQ_STAT);
		writel(host_irq_stat, hpriv->mmio + HOST_IRQ_STAT);
		handled = IRQ_HANDLED;
		set_dscope_tstamp(dpriv, TSTMP_IRQ_MARK_1);

		/*
		 * check if this command timed out and charged error handler, do not schedule tasklet in this case as
		 * the processing will restart from error handler
		 */
		spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
		if (dpriv->flags & (START_EH | WAIT_EH)) {
			// tell to EH that the command will be processed in normal way
			set_dscope_tstamp(dpriv, TSTMP_IRQ_MARK_1);
			dpriv->flags |= DELAYED_IRQ_RCVD;
			wake_up_interruptible(&dpriv->eh.wait);
		} else {
			dpriv->flags |= IRQ_PROCESSED;
			tasklet_schedule(&dpriv->bh);
		}
		spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
	} else {

		set_dscope_tstamp(dpriv, TSTMP_IRQ_SYS);
		dpriv->datascope.reg_stat[IRQ_COUNTER_SYS] = dpriv->datascope.reg_stat[IRQ_COUNTER_SYS] + 1;

		/* pass handling to AHCI level and then decide if the resource should be freed */
		// Elphel, Rocko migration
		//handled = ahci_single_irq_intr(irq, dev_instance);
		handled = ahci_single_level_irq_intr(irq, dev_instance);
		spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
		if (is_cmdq_empty(dpriv)) {
			dpriv->flags &= ~DISK_BUSY;
		} else {
			tasklet_schedule(&dpriv->bh);
		}
		spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
	}

	return handled;
}

/** Command queue processing tasklet */
void process_queue(unsigned long data)
{
	int i;
	size_t total_sz = 0;
	unsigned long time_usec;
	sec_usec_t end_time = {0};
	struct elphel_ahci_priv *dpriv = (struct elphel_ahci_priv *)data;

	// calculate the speed this frame has been written with
//	get_fpga_rtc(&end_time);
	time_usec = time_diff_usec(&dpriv->stat.start_time, &end_time);
	if (time_usec != 0) {
		for (i = 0; i < dpriv->sg_elems; i++)
			total_sz += dpriv->sgl[i].iov_len;
		add_sample(total_sz / time_usec, &dpriv->stat);
	}

//	// check if we can proceed - this was debug feature to fully stop cmd processing
//	if (dpriv->flags & (START_EH | WAIT_EH))
//		return;

	if (process_cmd(dpriv) == 0) {
		finish_cmd(dpriv);
		if (move_head(dpriv) != -1) {
			process_cmd(dpriv);
		} else {
			if (dpriv->flags & DELAYED_FINISH) {
				dpriv->flags &= ~DELAYED_FINISH;
				finish_rec(dpriv);
			} else {
				/* all commands have been processed */
				reset_flag(dpriv, DISK_BUSY);
			}
		}
	}
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
	int ret, i, irq_num;
	struct ahci_host_priv *hpriv;
	struct elphel_ahci_priv *dpriv;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct ata_host *host;

	if (&dev->kobj) {
		ret = sysfs_create_group(&dev->kobj, &dev_attr_root_group);
		if (ret < 0)
			return ret;
	}
	elphel_defer_load(dev);

	dev_info(&pdev->dev, "probing Elphel AHCI driver");

	dpriv = devm_kzalloc(dev, sizeof(struct elphel_ahci_priv), GFP_KERNEL);
	if (!dpriv)
		return -ENOMEM;

	dpriv->dev = dev;
	spin_lock_init(&dpriv->flags_lock);
	tasklet_init(&dpriv->bh, process_queue, (unsigned long)dpriv);
	// function has been changed
	//setup_timer(&dpriv->cmd_timer, process_timeout, (unsigned long)dpriv);
	timer_setup(&dpriv->cmd_timer, process_timeout, 0);
	dpriv->cmd_timeout = DEFAULT_CMD_TIMEOUT;
	init_waitqueue_head(&dpriv->eh.wait);

	for (i = 0; i < MAX_CMD_SLOTS; i++) {
		ret = init_buffers(dev, &dpriv->fbuffs[i]);
		if (ret != 0)
			return ret;
		init_vectors(&dpriv->fbuffs[i], dpriv->data_chunks[i]);
	}

	match = of_match_device(ahci_elphel_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	ret = elphel_parse_prop(dev->of_node, dev, dpriv);
	if (ret != 0)
		return ret;

	hpriv = ahci_platform_get_resources(pdev,0);
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

	/* reassign automatically assigned interrupt handler */
	/* this is disabled because recording in both modes goes through system */
//	irq_num = platform_get_irq(pdev, 0);
//	host = platform_get_drvdata(pdev);
//	devm_free_irq(dev, irq_num, host);
//	ret = devm_request_irq(dev, irq_num, elphel_irq_handler, IRQF_SHARED, dev_name(dev), host);
//	if (ret) {
//		dev_err(dev, "failed to reassign default IRQ handler to Elphel handler\n");
//		return ret;
//	}

	return 0;
}

static int elphel_drv_remove(struct platform_device *pdev)
{
	int i;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(&pdev->dev);

	dev_info(&pdev->dev, "removing Elphel AHCI driver");
	tasklet_kill(&dpriv->bh);
	remove_timer(dpriv);
	for (i = 0; i < MAX_CMD_SLOTS; i++)
		deinit_buffers(&pdev->dev, &dpriv->fbuffs[i]);
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_root_group);
	ata_platform_remove_one(pdev);

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

	dev_dbg(ap->dev, "cmd = 0x%x, qc->tag = %u, ap->qc_active = %u, link->sactive = %u, link->active_tag = %u\n",
			qc->tf.command, qc->tag, ap->qc_active, ap->link.sactive, ap->link.active_tag);

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

	dev_dbg(ap->dev, "dump command table content, first %d bytes, phys addr = 0x%x:\n", 16, pp->cmd_tbl_dma);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, pp->cmd_tbl, 16);

	dma_sync_single_for_device(&qc->dev->tdev, pp->cmd_tbl_dma,
			AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);

	struct elphel_ahci_priv *dpriv = dev_get_dpriv(ap->dev);
	set_dscope_tstamp(dpriv, TSTMP_CMD_SYS);
}

/** Set flag @e flag in driver private structure. This function uses spin lock to access the flags variable. */
static void set_flag(struct elphel_ahci_priv *dpriv, uint32_t flag)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	dpriv->flags |= flag;
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
}

/** Reset flag @e flag in driver private structure. This function uses spin lock to access the flags variable. */
static void reset_flag(struct elphel_ahci_priv *dpriv, uint32_t flag)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	dpriv->flags &= ~flag;
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
}

/** Map buffer vectors to S/G list and return the number of vectors mapped */
static int map_vectors(struct elphel_ahci_priv *dpriv)
{
	int i;
	int index = 0;
	int finish = 0;
	size_t total_sz = 0;
	size_t tail;
	struct fvec *chunks;
	struct fvec vect;

	chunks = dpriv->data_chunks[dpriv->head_ptr];
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
			vect = chunks[i];
		}
		if (total_sz > dpriv->max_data_sz) {
			/* truncate current buffer and finish mapping */
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
		if (vect.iov_len != 0) {
			if (vect.iov_len < MAX_PRDT_LEN) {
				dpriv->sgl[index++] = vect;
			} else {
				/* current vector is too long and can not be mapped to a single PRDT entry, split it */
				vectsplit(&vect, dpriv->sgl, &index);
				if (vect.iov_len < MAX_PRDT_LEN) {
					dpriv->sgl[index++] = vect;
				} else {
					/* free slots in PRDT table have ended */
					dpriv->curr_data_chunk = i;
					dpriv->curr_data_offset = (unsigned char *)vect.iov_base - (unsigned char *)chunks[i].iov_base;
					finish = 1;
				}
			}
			if (index == (MAX_SGL_LEN - 1))
				finish = 1;
		}
		if (finish)
			break;
	}
	if (finish == 0) {
		/* frame vectors have been fully processed, stop calling me */
		dpriv->curr_data_chunk = MAX_DATA_CHUNKS;
		dpriv->curr_data_offset = 0;
	}

	return index;
}

/** Split buffer pointed by vector @e vect into several smaller buffer. Each part will be less than #MAX_PRDT_LEN bytes */
static inline void vectsplit(struct fvec *vect, struct fvec *parts, size_t *n_elem)
{
	size_t len;
	struct fvec split;

	while (vect->iov_len > MAX_PRDT_LEN && *n_elem < MAX_SGL_LEN) {
		len = MAX_PRDT_LEN - MAX_PRDT_LEN % PHY_BLOCK_SIZE;
		split.iov_base = vect->iov_base;
		split.iov_dma = vect->iov_dma;
		split.iov_len = len;
		vectmov(vect, len);
		parts[*n_elem] = split;
		*n_elem = *n_elem + 1;
	}
}

/** Copy @e len bytes from buffer pointed by @e src vector to buffer pointed by @e dest vector */
static inline void vectcpy(struct fvec *dest, void *src, size_t len)
{
	unsigned char *d = (unsigned char *)dest->iov_base;

	memcpy(d + dest->iov_len, src, len);
	dest->iov_len += len;
}

/** Move vector forward by @e len bytes decreasing its length */
static inline void vectmov(struct fvec *vec, size_t len)
{
	if (vec->iov_len >= len) {
		vec->iov_base = (unsigned char *)vec->iov_base + len;
		vec->iov_dma += len;
		vec->iov_len -= len;
	}
}

/** Shrink vector length by @len bytes */
static inline void vectshrink(struct fvec *vec, size_t len)
{
	if (vec->iov_len >= len) {
		vec->iov_len -= len;
	}
}

/** Return the number of bytes needed to align @e data_len to @e align_len boundary */
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

/** Align current frame to disk sector boundary and each individual buffer to #ALIGNMENT_SIZE boundary */
static void align_frame(struct elphel_ahci_priv *dpriv)
{
	unsigned char *src;
	size_t len, total_sz, data_len;
	size_t cmd_slot = dpriv->tail_ptr;
	size_t prev_slot = get_prev_slot(dpriv);
	size_t max_len = dpriv->fbuffs[cmd_slot].common_buff.iov_len;
	struct device *dev = dpriv->dev;
	struct frame_buffers *fbuffs = &dpriv->fbuffs[cmd_slot];
	struct fvec *chunks = dpriv->data_chunks[cmd_slot];
	struct fvec *cbuff = &chunks[CHUNK_COMMON];
	struct fvec *rbuff = &dpriv->data_chunks[prev_slot][CHUNK_REM];

	total_sz = get_size_from(chunks, 0, 0, INCLUDE_REM) + rbuff->iov_len;
	if (total_sz < PHY_BLOCK_SIZE) {
		/* the frame length is less than sector size, delay this frame */
		if (prev_slot != cmd_slot) {
			/* some data may be left from previous frame */
			vectcpy(&chunks[CHUNK_REM], rbuff->iov_base, rbuff->iov_len);
			vectshrink(rbuff, rbuff->iov_len);
		}
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

	/* copy remainder of previous frame to the beginning of common buffer */
	if (likely(rbuff->iov_len != 0)) {
		len = rbuff->iov_len;
		dev_dbg(dev, "copy %u bytes from REM #%u to common buffer\n", len, prev_slot);
		vectcpy(cbuff, rbuff->iov_base, len);
		vectshrink(rbuff, rbuff->iov_len);
	}

	/* copy JPEG marker */
	len = chunks[CHUNK_LEADER].iov_len;
	vectcpy(cbuff, chunks[CHUNK_LEADER].iov_base, len);
	vectshrink(&chunks[CHUNK_LEADER], chunks[CHUNK_LEADER].iov_len);

	/* copy Exif if present */
	if (chunks[CHUNK_EXIF].iov_len != 0) {
		len = chunks[CHUNK_EXIF].iov_len;
		dev_dbg(dev, "copy %u bytes from EXIF to common buffer\n", len);
		vectcpy(cbuff, chunks[CHUNK_EXIF].iov_base, len);
		vectshrink(&chunks[CHUNK_EXIF], chunks[CHUNK_EXIF].iov_len);
	}

	/* align common buffer to ALIGNMENT boundary, APP15 marker should be placed before header data */
	data_len = cbuff->iov_len + chunks[CHUNK_HEADER].iov_len;
	len = align_bytes_num(data_len, ALIGNMENT_SIZE);
	if (len < JPEG_MARKER_LEN + JPEG_SIZE_LEN && len != 0) {
		/* the number of bytes needed for alignment is less than the length of the marker itself, increase the number of stuffing bytes */
		len += ALIGNMENT_SIZE;
	}
	dev_dbg(dev, "total number of stuffing bytes in APP15 marker: %u\n", len);
	app15[3] = len - JPEG_MARKER_LEN;
	vectcpy(cbuff, app15, len);

	/* copy JPEG header */
	len = chunks[CHUNK_HEADER].iov_len;
	dev_dbg(dev, "copy %u bytes from HEADER to common buffer\n", len);
	vectcpy(cbuff, chunks[CHUNK_HEADER].iov_base, len);
	vectshrink(&chunks[CHUNK_HEADER], chunks[CHUNK_HEADER].iov_len);

	/* check if there is enough data to continue - JPEG data length can be too short */
	len = get_size_from(chunks, CHUNK_DATA_0, 0, EXCLUDE_REM);
	if (len < PHY_BLOCK_SIZE) {
		size_t num = align_bytes_num(cbuff->iov_len, PHY_BLOCK_SIZE);
		dev_dbg(dev, "jpeg data is too short, delay this frame\n");
		if (len >= num) {
			/* there is enough data to align common buffer to sector boundary */
			if (num >= chunks[CHUNK_DATA_0].iov_len) {
				vectcpy(cbuff, chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
				num -= chunks[CHUNK_DATA_0].iov_len;
				vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
			} else {
				src = vectrpos(&chunks[CHUNK_DATA_0], num);
				vectcpy(cbuff, chunks[CHUNK_DATA_0].iov_base, num);
				vectshrink(&chunks[CHUNK_DATA_0], num);
				num = 0;
			}
			if (num >= chunks[CHUNK_DATA_1].iov_len) {
				vectcpy(cbuff, chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
				num -= chunks[CHUNK_DATA_1].iov_len;
				vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
			} else {
				src = vectrpos(&chunks[CHUNK_DATA_1], num);
				vectcpy(cbuff, chunks[CHUNK_DATA_1].iov_base, num);
				vectshrink(&chunks[CHUNK_DATA_1], num);
				num = 0;
			}
			if (num >= chunks[CHUNK_TRAILER].iov_len) {
				vectcpy(cbuff, chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				num -= chunks[CHUNK_TRAILER].iov_len;
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			} else {
				src = vectrpos(&chunks[CHUNK_TRAILER], num);
				vectcpy(cbuff, chunks[CHUNK_TRAILER].iov_base, num);
				vectshrink(&chunks[CHUNK_TRAILER], num);
				num = 0;
			}
		} else {
			/* there is not enough data to align common buffer to sector boundary, truncate common buffer */
			data_len = cbuff->iov_len % PHY_BLOCK_SIZE;
			src = vectrpos(cbuff, data_len);
			vectcpy(&chunks[CHUNK_REM], src, data_len);
			vectshrink(cbuff, data_len);
		}
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_0].iov_base, chunks[CHUNK_DATA_0].iov_len);
		vectshrink(&chunks[CHUNK_DATA_0], chunks[CHUNK_DATA_0].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
		vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
		vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
		vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);

		return;
	}

	/* align frame to sector size boundary; total size could have changed by the moment - recalculate */
	total_sz = get_size_from(chunks, 0, 0, INCLUDE_REM);
	len = total_sz % PHY_BLOCK_SIZE;
	dev_dbg(dev, "number of bytes crossing sector boundary: %u\n", len);
	if (len != 0) {
		if (len >= (chunks[CHUNK_DATA_1].iov_len + chunks[CHUNK_TRAILER].iov_len)) {
			/* current frame is not split or the second part of JPEG data is too short */
			data_len = len - chunks[CHUNK_DATA_1].iov_len - chunks[CHUNK_TRAILER].iov_len;
			src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
			vectcpy(&chunks[CHUNK_REM], src, data_len);
			vectshrink(&chunks[CHUNK_DATA_0], data_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
			vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
			vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		} else if (len >= chunks[CHUNK_TRAILER].iov_len) {
			/* there is enough data in second part to align the frame */
			data_len = len - chunks[CHUNK_TRAILER].iov_len;
			src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
			vectcpy(&chunks[CHUNK_REM], src, data_len);
			vectshrink(&chunks[CHUNK_DATA_1], data_len);
			vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
			vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
		} else {
			/* the trailing marker is split by sector boundary, copy (PHY_BLOCK_SIZE - 1) bytes from
			 * JPEG data block(s) to remainder buffer and then add trailing marker */
			data_len = PHY_BLOCK_SIZE - (chunks[CHUNK_TRAILER].iov_len - len);
			if (data_len >= chunks[CHUNK_DATA_1].iov_len) {
				size_t cut_len = data_len - chunks[CHUNK_DATA_1].iov_len;
				src = vectrpos(&chunks[CHUNK_DATA_0], cut_len);
				vectcpy(&chunks[CHUNK_REM], src, cut_len);
				vectshrink(&chunks[CHUNK_DATA_0], cut_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_DATA_1].iov_base, chunks[CHUNK_DATA_1].iov_len);
				vectshrink(&chunks[CHUNK_DATA_1], chunks[CHUNK_DATA_1].iov_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			} else {
				src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
				vectcpy(&chunks[CHUNK_REM], src, data_len);
				vectshrink(&chunks[CHUNK_DATA_1], data_len);
				vectcpy(&chunks[CHUNK_REM], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
				vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
			}
		}
	} else {
		/* the frame is aligned to sector boundary but some buffers may be not */
		chunks[CHUNK_ALIGN].iov_base = vectrpos(cbuff, 0);
		chunks[CHUNK_ALIGN].iov_dma = cbuff->iov_dma + cbuff->iov_len;
		chunks[CHUNK_ALIGN].iov_len = 0;
		if (chunks[CHUNK_DATA_1].iov_len == 0) {
			data_len = chunks[CHUNK_DATA_0].iov_len % ALIGNMENT_SIZE;
			src = vectrpos(&chunks[CHUNK_DATA_0], data_len);
			vectcpy(&chunks[CHUNK_ALIGN], src, data_len);
			vectshrink(&chunks[CHUNK_DATA_0], data_len);
		} else {
			data_len = chunks[CHUNK_DATA_1].iov_len % ALIGNMENT_SIZE;
			src = vectrpos(&chunks[CHUNK_DATA_1], data_len);
			vectcpy(&chunks[CHUNK_ALIGN], src, data_len);
			vectshrink(&chunks[CHUNK_DATA_1], data_len);
		}
		vectcpy(&chunks[CHUNK_ALIGN], chunks[CHUNK_TRAILER].iov_base, chunks[CHUNK_TRAILER].iov_len);
		vectshrink(&chunks[CHUNK_TRAILER], chunks[CHUNK_TRAILER].iov_len);
	}

	/* debug sanity check, should not happen */
	if (cbuff->iov_len >= max_len) {
		dev_err(NULL, "ERROR: the number of bytes copied to common buffer exceeds its size\n");
	}
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
static void init_vectors(struct frame_buffers *buffs, struct fvec *chunks)
{
	chunks[CHUNK_EXIF].iov_base = buffs->exif_buff.iov_base;
	chunks[CHUNK_EXIF].iov_len = 0;

	chunks[CHUNK_LEADER].iov_base = buffs->jpheader_buff.iov_base;
	chunks[CHUNK_LEADER].iov_len = 0;
	chunks[CHUNK_HEADER].iov_base = (unsigned char *)chunks[CHUNK_LEADER].iov_base + JPEG_MARKER_LEN;
	chunks[CHUNK_HEADER].iov_len = 0;

	chunks[CHUNK_TRAILER].iov_base = buffs->trailer_buff.iov_base;
	chunks[CHUNK_TRAILER].iov_len = 0;

	chunks[CHUNK_REM].iov_base = buffs->rem_buff.iov_base;
	chunks[CHUNK_REM].iov_len = 0;

	/* this is the only DMA mapped buffer and its DMA address should be set */
	chunks[CHUNK_COMMON].iov_base = buffs->common_buff.iov_base;
	chunks[CHUNK_COMMON].iov_dma = buffs->common_buff.iov_dma;
	chunks[CHUNK_COMMON].iov_len = 0;
}

/** Allocate memory for frame buffers */
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

	/* common buffer should be large enough to contain JPEG header, Exif, some alignment bytes and
	 * remainder from previous frame */
	total_sz = MAX_EXIF_SIZE + JPEG_HEADER_MAXSIZE + ALIGNMENT_SIZE + 2 * PHY_BLOCK_SIZE;
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

	buffs->rem_buff.iov_base = kmalloc(2 * PHY_BLOCK_SIZE, GFP_KERNEL);
	if (!buffs->rem_buff.iov_base)
		goto err_remainder;
	buffs->rem_buff.iov_len = 2 * PHY_BLOCK_SIZE;

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

/** Free allocated frame buffers */
static void deinit_buffers(struct device *dev, struct frame_buffers *buffs)
{
	kfree(buffs->jpheader_buff.iov_base);
	kfree(buffs->exif_buff.iov_base);
	kfree(buffs->trailer_buff.iov_base);
	dma_unmap_single(dev, buffs->common_buff.iov_dma, buffs->common_buff.iov_len, DMA_TO_DEVICE);
	kfree(buffs->common_buff.iov_base);
	kfree(buffs->rem_buff.iov_base);
}

/** Discard buffer pointers which makes the command slot marked as empty */
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

/** Get driver private structure from pointer to device structure */
static inline struct elphel_ahci_priv *dev_get_dpriv(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return dpriv;
}

/** Process command and return the number of S/G entries mapped */
static int process_cmd(struct elphel_ahci_priv *dpriv)
{
	unsigned long irq_flags;
	struct fvec *cbuff;
	struct ata_host *host = dev_get_drvdata(dpriv->dev);
	struct ata_port *port = host->ports[DEFAULT_PORT_NUM];
	size_t max_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
	size_t rem_sz = get_size_from(dpriv->data_chunks[dpriv->head_ptr], dpriv->curr_data_chunk, dpriv->curr_data_offset, EXCLUDE_REM);

	dev_dbg(dpriv->dev, "flags = 0x%x\n", dpriv->flags);
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if (dpriv->flags & PROC_CMD)
		dpriv->lba_ptr.lba_write += dpriv->lba_ptr.wr_count;
	dpriv->flags |= PROC_CMD;
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);

	/* define ATA command to use for current transaction */
	if ((dpriv->lba_ptr.lba_write & ~ADDR_MASK_28_BIT) || rem_sz > max_sz) {
		dpriv->curr_cmd = ATA_CMD_WRITE_EXT;
		dpriv->max_data_sz = (MAX_LBA_COUNT_EXT + 1) * PHY_BLOCK_SIZE;
	} else {
		dpriv->curr_cmd = ATA_CMD_WRITE;
		dpriv->max_data_sz = (MAX_LBA_COUNT + 1) * PHY_BLOCK_SIZE;
	}

	dpriv->sg_elems = map_vectors(dpriv);

	if (dpriv->sg_elems != 0) {
		dump_sg_list(dpriv->dev, dpriv->sgl, dpriv->sg_elems);
//		get_fpga_rtc(&dpriv->stat.start_time);
		set_timer(dpriv);

		dpriv->lba_ptr.wr_count = get_blocks_num(dpriv->sgl, dpriv->sg_elems);
		if (dpriv->lba_ptr.lba_write + dpriv->lba_ptr.wr_count > dpriv->lba_ptr.lba_end) {
			/* the frame rolls over the buffer boundary, don't split it and start writing from the beginning */
			dpriv->lba_ptr.lba_write = dpriv->lba_ptr.lba_start;
		}
		cbuff = &dpriv->fbuffs[dpriv->head_ptr].common_buff;
		dma_sync_single_for_device(dpriv->dev, cbuff->iov_dma, cbuff->iov_len, DMA_TO_DEVICE);
		elphel_cmd_issue(port, dpriv->lba_ptr.lba_write, dpriv->lba_ptr.wr_count, dpriv->sgl, dpriv->sg_elems, dpriv->curr_cmd);
	}
	return dpriv->sg_elems;
}

/** Finish currently running command */
static void finish_cmd(struct elphel_ahci_priv *dpriv)
{
	int all;
	unsigned long irq_flags;

	remove_timer(dpriv);
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	dpriv->lba_ptr.wr_count = 0;
	if ((dpriv->flags & LAST_BLOCK) == 0) {
		all = 0;
	} else {
		all = 1;
		dpriv->flags &= ~LAST_BLOCK;
	}
	reset_chunks(dpriv->data_chunks[dpriv->head_ptr], all);
	dpriv->curr_cmd = 0;
	dpriv->max_data_sz = 0;
	dpriv->curr_data_chunk = 0;
	dpriv->curr_data_offset = 0;
	dpriv->flags &= ~PROC_CMD;
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
}

/** Fill free space in REM buffer with 0 and save the remaining data chunk */
static void finish_rec(struct elphel_ahci_priv *dpriv)
{
	size_t stuff_len;
	unsigned char *src;
	struct fvec *cvect = &dpriv->data_chunks[dpriv->head_ptr][CHUNK_COMMON];
	struct fvec *rvect = &dpriv->data_chunks[dpriv->head_ptr][CHUNK_REM];

	if (rvect->iov_len == 0) {
		finish_cmd(dpriv);
		reset_flag(dpriv, DISK_BUSY);
		return;
	}

	dev_dbg(dpriv->dev, "write last chunk of data from slot %u, size: %u\n", dpriv->head_ptr, rvect->iov_len);
	stuff_len = PHY_BLOCK_SIZE - rvect->iov_len;
	src = vectrpos(rvect, 0);
	memset(src, 0, stuff_len);
	rvect->iov_len += stuff_len;
	dma_sync_single_for_cpu(dpriv->dev, dpriv->fbuffs[dpriv->head_ptr].common_buff.iov_dma, dpriv->fbuffs[dpriv->head_ptr].common_buff.iov_len, DMA_TO_DEVICE);
	vectcpy(cvect, rvect->iov_base, rvect->iov_len);
	vectshrink(rvect, rvect->iov_len);

	dpriv->flags |= LAST_BLOCK;
	if (process_cmd(dpriv) == 0) {
		finish_cmd(dpriv);
		reset_flag(dpriv, DISK_BUSY);
	}
}

/** Move a pointer to free command slot one step forward. This function holds spin lock #elphel_ahci_priv::flags_lock */
static int move_tail(struct elphel_ahci_priv *dpriv)
{
	int ret;
	unsigned long irq_flags;
	size_t slot = (dpriv->tail_ptr + 1) % MAX_CMD_SLOTS;

	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if (slot != dpriv->head_ptr) {
		dpriv->flags |= LOCK_TAIL;
		dpriv->tail_ptr = slot;
		ret = 0;
	} else {
		/* no more free command slots */
		ret = -1;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);

	if (ret == 0)
		dev_dbg(dpriv->dev, "move tail pointer to slot: %u\n", slot);

	return ret;
}

/** Move a pointer to next ready command. This function holds spin lock #elphel_ahci_priv::flags_lock*/
static int move_head(struct elphel_ahci_priv *dpriv)
{
	int ret;
	size_t use_tail;
	unsigned long irq_flags;
	size_t slot = (dpriv->head_ptr + 1) % MAX_CMD_SLOTS;

	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if (dpriv->flags & LOCK_TAIL) {
		/* current command slot is not ready yet, use previous */
		use_tail = get_prev_slot(dpriv);
	} else {
		use_tail = dpriv->tail_ptr;
	}
	if (dpriv->head_ptr != use_tail) {
		dpriv->head_ptr = slot;
		ret = 0;
	} else {
		/* no more commands in queue */
		ret = -1;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);

	if (ret == 0)
		dev_dbg(dpriv->dev, "move head pointer to slot: %u\n", slot);

	return ret;
}

/** Check if command queue is empty */
static int is_cmdq_empty(const struct elphel_ahci_priv *dpriv)
{
	size_t use_tail;

	if (dpriv->flags & LOCK_TAIL) {
		/* current command slot is not ready yet, use previous */
		use_tail = get_prev_slot(dpriv);
	} else {
		use_tail = dpriv->tail_ptr;
	}
	if (dpriv->head_ptr != use_tail)
		return 0;
	else
		return 1;
}

/** Get command slot before the last one filled in */
static size_t get_prev_slot(const struct elphel_ahci_priv *dpriv)
{
	size_t slot;

	if (dpriv->tail_ptr == dpriv->head_ptr)
		return dpriv->tail_ptr;

	if (dpriv->tail_ptr != 0) {
		slot = dpriv->tail_ptr - 1;
	} else {
		slot = MAX_CMD_SLOTS - 1;
	}
	return slot;
}

/** Handle delayed interrupt or other errors */
void error_handler(struct elphel_ahci_priv *dpriv)
{
	long rc;
	unsigned int class;
	unsigned long delay;
	unsigned long start_wait = jiffies;
	unsigned long deadline = msecs_to_jiffies(WAIT_IRQ_TIMEOUT);
	struct ata_host *host = dev_get_drvdata(dpriv->dev);
	struct ata_port *ap = host->ports[DEFAULT_PORT_NUM];
	struct ata_link *link = &ap->link;

	set_flag(dpriv, WAIT_EH);

	if (dpriv->eh.s_error || dpriv->eh.irq_stat) {
		// stub
	}

	rc = wait_event_interruptible_timeout(dpriv->eh.wait, dpriv->flags & DELAYED_IRQ_RCVD, deadline);
	dev_dbg(dpriv->dev, "wait_event_interruptible_timeout returned %ld\n", rc);
	if (!rc) {
		reset_flag(dpriv, IRQ_SIMPLE | DISK_BUSY | PROC_CMD | START_EH);
		/* interrupt has not been received after more than (DEFAULT_CMD_TIMEOUT + WAIT_IRQ_TIMEOUT) ms,
		 * try to recover the link using WAIT_IRQ_TIMEOUT value for next deadline
		 */
		deadline = jiffies + msecs_to_jiffies(WAIT_IRQ_TIMEOUT);
		rc = ap->ops->hardreset(link, &class, deadline);
		if (!rc)
			dev_err(dpriv->dev, "error: ahci_hardreset returned %ld\n", rc);
		ata_port_wait_eh(ap);

		// here we assume that circbuf has not been overrun yet, prepare to repeat current command
		dpriv->lba_ptr.wr_count = 0;
		dpriv->curr_cmd = 0;
		dpriv->max_data_sz = 0;
		dpriv->curr_data_chunk = 0;
		dpriv->curr_data_offset = 0;
		dev_dbg(dpriv->dev, "repeat command from slot (head_ptr): %u\n", dpriv->head_ptr);
	} else {
		// late interrupt received or the queue was woken up, proceed to normal operation
		if (rc < 0) {
			delay = jiffies - start_wait;
		} else {
			delay = msecs_to_jiffies(WAIT_IRQ_TIMEOUT) - rc;
		}
		dpriv->stat.last_irq_delay = jiffies_to_msecs(delay) + DEFAULT_CMD_TIMEOUT;
		dpriv->stat.stat_ready = 1;
		dev_dbg(dpriv->dev, "late interrupt received, delay is %lu ms\n", dpriv->stat.last_irq_delay);
		reset_flag(dpriv, DELAYED_IRQ_RCVD | START_EH);
	}
	reset_flag(dpriv, WAIT_EH);

	// resume command processing
	if (process_cmd(dpriv) == 0) {
		finish_cmd(dpriv);
		if (move_head(dpriv) != -1)
			process_cmd(dpriv);
	}
}

/** This function is called during write test only */
static void invalidate_cache(struct fvec *vect)
{
	__cpuc_flush_dcache_area(vect->iov_base, vect->iov_len);
	outer_inv_range(vect->iov_dma, vect->iov_dma + (vect->iov_len - 1));
}

/** Get and enqueue new command */
static ssize_t rawdev_write(struct device *dev,  ///< device structure associated with the driver
		struct device_attribute *attr,           ///< interface for device attributes
		const char *buff,                        ///< buffer containing new command
		size_t buff_sz)                          ///< the size of the command buffer
{
	ssize_t rcvd = 0;
	bool proceed = false;
	bool start_eh = false;
	unsigned long irq_flags;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);
	struct frame_data fdata;
	struct frame_buffers *buffs;
	struct fvec *chunks;

	/* disable recording using direct driver commands */
	return -EINVAL;

	/* simple check if we've got the right command */
	if (buff_sz != sizeof(struct frame_data)) {
		dev_err(dev, "the size of the data buffer is incorrect, should be equal to sizeof(struct frame_data)\n");
		return -EINVAL;
	}
	memcpy(&fdata, buff, sizeof(struct frame_data));

	/* error recovery in progress, do not continue recording and wait while error handler has finished */
	if (dpriv->flags & WAIT_EH) {
		if (dpriv->stat.stat_ready) {
			// return this code to indicate that some stats are ready
			return -EIO;
		} else {
			return -EAGAIN;
		}
	}
	/* check if we should reset controller */
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if (dpriv->flags & START_EH) {
		start_eh = true;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
	if (start_eh) {
		dpriv->io_error_flag = 1;
		error_handler(dpriv);
		return -EAGAIN;
	}

	/* lock disk resource as soon as possible */
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if ((dpriv->flags & DISK_BUSY) == 0) {
		dpriv->flags |= DISK_BUSY;
		proceed = true;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);

	if (fdata.cmd & DRV_CMD_FINISH) {
		if ((dpriv->flags & PROC_CMD) == 0 && proceed) {
			finish_rec(dpriv);
		} else {
			dpriv->flags |= DELAYED_FINISH;
		}
		dpriv->datascope.reg_stat[CMD_RCVD] = dpriv->datascope.reg_stat[CMD_RCVD] + 1;
		return buff_sz;
	}

	if (move_tail(dpriv) == -1) {
		/* we are not ready yet because command queue is full */
		dev_dbg_ratelimited(dpriv->dev, "command queue is full, flags = %u, proceed = %d\n", dpriv->flags, proceed);
		return -EAGAIN;
	}
	chunks = dpriv->data_chunks[dpriv->tail_ptr];
	buffs = &dpriv->fbuffs[dpriv->tail_ptr];

	dev_dbg(dev, "process frame from sensor port: %u, command = %d, flags = %u\n", fdata.sensor_port, fdata.cmd, dpriv->flags);
	if (fdata.cmd & DRV_CMD_EXIF) {
		rcvd = exif_get_data(fdata.sensor_port, fdata.meta_index, buffs->exif_buff.iov_base, buffs->exif_buff.iov_len);
		chunks[CHUNK_EXIF].iov_len = rcvd;
	}

	if (!(fdata.cmd & DRV_CMD_WRITE_TEST)) {
		rcvd = jpeghead_get_data(fdata.sensor_port, buffs->jpheader_buff.iov_base, buffs->jpheader_buff.iov_len, 0);
		if (rcvd < 0) {
			/* free resource lock and current command slot */
			if (proceed) {
				spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
				dpriv->flags &= ~DISK_BUSY;
				spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
			}
			reset_chunks(chunks, 0);
			dpriv->tail_ptr = get_prev_slot(dpriv);
			dpriv->flags &= ~LOCK_TAIL;
			dev_err(dev, "could not get JPEG header, error %d\n", rcvd);
			return -EINVAL;
		}
		chunks[CHUNK_LEADER].iov_len = JPEG_MARKER_LEN;
		chunks[CHUNK_TRAILER].iov_len = JPEG_MARKER_LEN;
		chunks[CHUNK_HEADER].iov_len = rcvd - chunks[CHUNK_LEADER].iov_len;
	}

	rcvd = circbuf_get_ptr(fdata.sensor_port, fdata.cirbuf_ptr, fdata.jpeg_len, &chunks[CHUNK_DATA_0], &chunks[CHUNK_DATA_1]);
	if (rcvd < 0) {
		/* free resource lock and current command slot */
		if (proceed) {
			spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
			dpriv->flags &= ~DISK_BUSY;
			spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
		}
		reset_chunks(chunks, 0);
		dpriv->tail_ptr = get_prev_slot(dpriv);
		dpriv->flags &= ~LOCK_TAIL;
		dev_err(dev, "could not get JPEG data, error %d\n", rcvd);
		return -EINVAL;
	}
	if (fdata.cmd & DRV_CMD_WRITE_TEST)
		invalidate_cache(&chunks[CHUNK_DATA_0]);

	align_frame(dpriv);
	/* new command slot is ready now and can be unlocked */
	reset_flag(dpriv, LOCK_TAIL);

	dpriv->datascope.reg_stat[CMD_RCVD] = dpriv->datascope.reg_stat[CMD_RCVD] + 1;

	if (!proceed) {
		/* disk may be free by the moment, try to grab it */
		spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
		if ((dpriv->flags & DISK_BUSY) == 0) {
			dpriv->flags |= DISK_BUSY;
			proceed = true;
		}
		spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);
	}
	if ((dpriv->flags & PROC_CMD) == 0 && proceed) {
		if (get_size_from(dpriv->data_chunks[dpriv->head_ptr], 0, 0, EXCLUDE_REM) == 0)
			move_head(dpriv);
		process_cmd(dpriv);
	}

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
		struct fvec *sgl,                        ///< S/G list pointing to data buffers
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

	dev_dbg(ap->dev, "dump command table content, first %d bytes, phys addr = 0x%x:\n", 16, pp->cmd_tbl_dma);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, pp->cmd_tbl, 16);

	dma_sync_single_for_device(ap->dev, pp->cmd_tbl_dma, AHCI_CMD_TBL_AR_SZ, DMA_TO_DEVICE);

	set_dscope_tstamp(dpriv, TSTMP_CMD_DRV);
	dpriv->datascope.reg_stat[CMD_SENT] = dpriv->datascope.reg_stat[CMD_SENT] + 1;

	/* issue command */
	writel(0x11, port_mmio + PORT_CMD);
	writel(1 << slot_num, port_mmio + PORT_CMD_ISSUE);
}

/** Defer system command if internal command queue is not empty */
static int elphel_qc_defer(struct ata_queued_cmd *qc)
{
	int ret;
	unsigned long irq_flags;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(qc->ap->dev);

	/* First apply the usual rules */
	ret = ata_std_qc_defer(qc);
	if (ret != 0)
		return ret;

	/* And now check if internal command is in progress */
	spin_lock_irqsave(&dpriv->flags_lock, irq_flags);
	if ((dpriv->flags & DISK_BUSY) || is_cmdq_empty(dpriv) == 0) {
		ret = ATA_DEFER_LINK;
	} else {
		dpriv->flags |= DISK_BUSY;
	}
	spin_unlock_irqrestore(&dpriv->flags_lock, irq_flags);

	return ret;
}

/** Return the stating position of disk buffer (in LBA) */
static ssize_t lba_start_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_start);
}

/** Set the starting position of disk buffer (in LBA) */
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

/** Return the ending position of disk buffer (in LBA) */
static ssize_t lba_end_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_end);
}

/** Set the ending position of disk buffer (in LBA) */
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

/** Return the current position of write pointer (in LBA) */
static ssize_t lba_current_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	return snprintf(buff, 20, "%llu\n", dpriv->lba_ptr.lba_write);
}

/** Set the current position of write pointer (in LBA) */
static ssize_t lba_current_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct elphel_ahci_priv *dpriv = hpriv->plat_data;

	if (kstrtoull(buff, 10, &dpriv->lba_ptr.lba_write) != 0)
		return -EINVAL;

	return buff_sz;
}

/** Return the current disk write speed in MB/s */
static ssize_t wr_speed_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	int i;
	unsigned int val, median;
	unsigned int *l, *r;
	unsigned int samples[SPEED_SAMPLES_NUM];
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	// sort recording speed samples and get median
	memcpy(samples, dpriv->stat.samples, SPEED_SAMPLES_NUM * sizeof(dpriv->stat.samples[0]));
	l = samples;
	for (i = 1; i < SPEED_SAMPLES_NUM; i++) {
		val = samples[i];
		r = &samples[i- 1];
		while (r >= l && *r > val) {
			*(r + 1) = *r;
			r--;
		}
		*(r + 1) = val;
	}
	median = samples[SPEED_SAMPLES_NUM / 2];

	return snprintf(buff, 32, "Write speed: %u MB/s\n", median);
}

static ssize_t timeout_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	if (kstrtouint(buff, 10, &dpriv->cmd_timeout) != 0)
		return -EINVAL;

	return buff_sz;
}

static ssize_t timeout_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	return snprintf(buff, 64, "Command execution timeout: %u ms\n", dpriv->cmd_timeout);
}

static ssize_t io_error_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	return sprintf(buff, "%u\n", dpriv->io_error_flag);
}

/** Reset statistics availability flag. This will indicate that the current sample is processed */
static ssize_t stat_delay_write(struct device *dev, struct device_attribute *attr, const char *buff, size_t buff_sz)
{
	unsigned long data;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	if (kstrtoul(buff, 10, &data) != 0)
		return -EINVAL;

	if (data == dpriv->stat.last_irq_delay)
		dpriv->stat.stat_ready = 0;

	return buff_sz;
}

static ssize_t stat_delay_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	unsigned int data;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	if (dpriv->stat.stat_ready)
		data = dpriv->stat.last_irq_delay;
	else
		data = 0;

	return scnprintf(buff, 20, "%u\n", data);
}

static ssize_t reg_status_read(struct device *dev, struct device_attribute *attr, char *buff)
{
	int i;
	ssize_t cntr;
	unsigned int data;
	struct elphel_ahci_priv *dpriv = dev_get_dpriv(dev);

	for (i = 0, cntr = 0; i < REG_NUM; i++) {
		if ((i % 8) == 0 && i != 0)
			cntr += scnprintf(&buff[cntr], PAGE_SIZE - cntr - 1, "\n");
		cntr += scnprintf(&buff[cntr], PAGE_SIZE - cntr - 1, "0x%08x ", dpriv->datascope.reg_stat[i]);
	}
	cntr += scnprintf(&buff[cntr], PAGE_SIZE - cntr - 1, "\n");

	return cntr;
}

extern int enable_panic;
static ssize_t enable_panic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%i", enable_panic);
}

static ssize_t enable_panic_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int data;

	sscanf(buf, "%i", &data);
	if (data != 0)
		enable_panic = 1;
	else
		enable_panic = 0;

	return count;
}

static DEVICE_ATTR(load_module, S_IWUSR | S_IWGRP, NULL, set_load_flag);
static DEVICE_ATTR(io_error, S_IRUSR | S_IRGRP | S_IROTH, io_error_read, NULL);
static DEVICE_ATTR(stat_irq_delay, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, stat_delay_read, stat_delay_write);
static DEVICE_ATTR(reg_status, S_IRUSR | S_IRGRP | S_IROTH, reg_status_read, NULL);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_WRITE, S_IWUSR | S_IWGRP, NULL, rawdev_write);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_START, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, lba_start_read, lba_start_write);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_END, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, lba_end_read, lba_end_write);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_CURR, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, lba_current_read, lba_current_write);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_WRSPEED, S_IRUSR | S_IRGRP | S_IROTH, wr_speed_read, NULL);
static DEVICE_ATTR(SYSFS_AHCI_FNAME_TIMEOUT, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, timeout_read, timeout_write);
static DEVICE_ATTR(enable_panic_flag, S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, enable_panic_show, enable_panic_store);
static struct attribute *root_dev_attrs[] = {
		&dev_attr_load_module.attr,
		&dev_attr_io_error.attr,
		&dev_attr_stat_irq_delay.attr,
		&dev_attr_reg_status.attr,
		&dev_attr_SYSFS_AHCI_FNAME_WRITE.attr,
		&dev_attr_SYSFS_AHCI_FNAME_START.attr,
		&dev_attr_SYSFS_AHCI_FNAME_END.attr,
		&dev_attr_SYSFS_AHCI_FNAME_CURR.attr,
		&dev_attr_SYSFS_AHCI_FNAME_WRSPEED.attr,
		&dev_attr_SYSFS_AHCI_FNAME_TIMEOUT.attr,
		&dev_attr_enable_panic_flag.attr,
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
		/* .qc_defer       = elphel_qc_defer, */
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

/** Debug function, checks frame alignment */
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
		}
	}
	if ((sz % PHY_BLOCK_SIZE) != 0) {
		dev_err(NULL, "ERROR: total length of the transaction is not aligned to sector boundary, total length %u\n", sz);
		ret = -1;
	} else {
		dev_err(NULL, "===== frame is OK =====\n");
	}
	return ret;
}

/** Debug function, prints the S/G list of current command */
static void dump_sg_list(const struct device *dev, const struct fvec *sgl, size_t elems)
{
	int i;

	dev_dbg(dev, "===== dump S/G list, %u elements:\n", elems);
	for (i = 0; i < elems; i++) {
		dev_dbg(dev, "dma address: 0x%x, len: %u\n", sgl[i].iov_dma, sgl[i].iov_len);
	}
	dev_dbg(dev, "===== end of S/G list =====\n");
}

/** Debug function, make dump of controller's memory, sz is in DW */
static void dump_iomem(const struct device *dev, void __iomem *mmio, size_t sz, dma_addr_t dma_addr)
{
	int i, col = 0, pos = 0;
	u32 val;
	char str[512] = {0};

	if (dma_addr)
		dev_dbg(dev, "DMA address: 0x%08x\n", dma_addr);
	for (i = 0; i < sz; i++) {
		val = ioread32(mmio + 4 * i);
		pos += snprintf(&str[pos], 16, "0x%08x ", val);
		col++;
		if (col == 16) {
			col = 0;
			pos = 0;
			dev_dbg(dev, "%s\n", str);
		}
	}
	if (pos != 0)
		dev_dbg(dev, "%s\n", str);
}

static void dump_dpriv_fields(struct elphel_ahci_priv *dpriv)
{
	dev_dbg(dpriv->dev, "head ptr: %u, tail ptr: %u\n", dpriv->head_ptr, dpriv->tail_ptr);
	dev_dbg(dpriv->dev, "flags: 0x%x\n", dpriv->flags);

	dev_dbg(dpriv->dev, "lba_start = %llu, lba_current = %llu, lba_end = %llu\n",
			dpriv->lba_ptr.lba_start, dpriv->lba_ptr.lba_write, dpriv->lba_ptr.lba_end);
	dev_dbg(dpriv->dev, "lba_ptr.wr_count = %u, curr_cmd = 0x%x, max_data_sz = %u, curr_data_chunk = %d, curr_data_offset = %u\n",
			dpriv->lba_ptr.wr_count, dpriv->curr_cmd, dpriv->max_data_sz, dpriv->curr_data_chunk, dpriv->curr_data_offset);
}

static void dump_current_state(struct elphel_ahci_priv *dpriv)
{
	struct ata_host *host = dev_get_drvdata(dpriv->dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct ata_port *ap = host->ports[DEFAULT_PORT_NUM];
	struct ata_link *link = &ap->link;
	struct ahci_port_priv *pp = ap->private_data;
	uint32_t *datascope_ptr;

	dev_dbg(dpriv->dev, "===== HBA memory dump ======\n");
	dump_iomem(dpriv->dev, hpriv->mmio, 100, dpriv->base_addr);
	dev_dbg(dpriv->dev, "===== command list structure dump =====\n");
	dump_iomem(dpriv->dev, pp->cmd_slot, 8, pp->cmd_slot_dma);
	dev_dbg(dpriv->dev, "===== command FIS dump =====\n");
	dump_iomem(dpriv->dev, pp->cmd_tbl, 16, pp->cmd_tbl_dma);
	dev_dbg(dpriv->dev, "===== received FIS memory (last FIS received) dump =====\n");
	dump_iomem(dpriv->dev, pp->rx_fis, 64, pp->rx_fis_dma);

//	datascope_ptr = ioremap_nocache(dpriv->base_addr + DATASCOPE_OFFSET, DATASCOPE_SIZE);
//	if (datascope_ptr) {
//		dev_dbg(dpriv->dev, "===== datascope (debug) data dump =====\n");
//		dump_iomem(dpriv->dev, datascope_ptr, 512, dpriv->base_addr + DATASCOPE_OFFSET);
//		iounmap(datascope_ptr);
//	} else {
//		dev_dbg(dpriv->dev, "unable to ioremap datascope region\n");
//	}

	dump_dpriv_fields(dpriv);
	dump_sg_list(dpriv->dev, dpriv->sgl, dpriv->sg_elems);
}

static void set_dscope_tstamp(struct elphel_ahci_priv *dpriv, unsigned int cmd)
{
        struct ata_host *host;
        struct ahci_host_priv *hpriv;
        struct ata_port *ap;
        void __iomem *port_mmio;
        uint32_t data = 0;

        host = dev_get_drvdata(dpriv->dev);
        hpriv = host->private_data;
        ap = host->ports[DEFAULT_PORT_NUM];
        port_mmio = ahci_port_base(ap);

        // three bits command flag (TSTMP_*)
        data |= (cmd & 0x7);
		iowrite32(data, port_mmio + PORT_TIMESTAMP_ADDR);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elphel, Inc.");
MODULE_DESCRIPTION("Elphel AHCI SATA platform driver for elphel393 camera");
