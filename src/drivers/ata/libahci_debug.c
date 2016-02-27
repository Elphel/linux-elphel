/*
 * libahci_debug.c
 *
 *  Created on: Jan 20, 2016
 *      Author: mk
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>
#include <elphel/elphel393-mem.h>
#include "libahci_debug.h"
#include <asm/io.h> // ioremap(), copy_fromio
/*
 struct elphel_buf_t
{
	void *vaddr;
	dma_addr_t paddr;
	ssize_t size;
	memcpy(to,from,sizeof())
};
extern struct elphel_buf_t *pElphel_buf;
*/
static u32              page_cntr = 0;
static struct dentry	*debug_root = NULL;
static struct libahci_debug_list debug_list = {.debug = 0};
static struct ahci_cmd	cmd;
static bool load_flag = false;
const size_t maxigp1_start = 0x80000000; // start of MAXIGP1 physical address range
const size_t maxigp1_size =  0x3000; // size of register memory to save
const size_t buffer_offset = 0x40000; // start of dumping area (0xxxx, 1xxxx and 2xxxx are used for dma buffers
const size_t counter_offset = 0x3fff8; // save page counter and page size with this offset in the buffer
const size_t fsm_state_offset = 0xffc;
static void * ioptr = 0; // keep iomemory mapped forever

char *early_buff;

static inline u32 libahci_debug_get_fsm_state(void);

//static struct mem_buffer mem_buff;

/*
 * Print PxIS (0x10) analysis
 */
void libahci_debug_dump_irq(u32 status)
{
	int len = 0;
	int pos;
	char *str = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);

	if (!str)
		return;

	len = snprintf(str, LIBAHCI_DEBUG_BUFSZ, "\tinterrupt analysis: ");
	pos = len;
	if (status & PORT_IRQ_D2H_REG_FIS) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "D2H Register FIS * ");
		pos += len;
	}
	if (status & PORT_IRQ_PIOS_FIS) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, " PIO Setup FIS * ");
		pos += len;
	}
	if (status & PORT_IRQ_DMAS_FIS) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "DMA Setup FIS * ");
		pos += len;
	}
	if (status & PORT_IRQ_SDB_FIS) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Set Device Bits FIS * ");
		pos += len;
	}
	if (status & PORT_IRQ_UNK_FIS) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Unknown FIS * ");
		pos += len;
	}
	if (status & PORT_IRQ_SG_DONE) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Descriptor processed * ");
		pos += len;
	}
	if (status & PORT_IRQ_CONNECT) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Port connect change status * ");
		pos += len;
	}
	if (status & PORT_IRQ_DEV_ILCK) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Device interlock * ");
		pos += len;
	}
	if (status & PORT_IRQ_PHYRDY) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "PhyRdy change status * ");
		pos += len;
	}
	if (status & PORT_IRQ_BAD_PMP) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Incorrect port multiplier * ");
		pos += len;
	}
	if (status & PORT_IRQ_OVERFLOW) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Overflow * ");
		pos += len;
	}
	if (status & PORT_IRQ_IF_NONFATAL) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Iface nonfatal error * ");
		pos += len;
	}
	if (status & PORT_IRQ_IF_ERR) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Iface fatal error * ");
		pos += len;
	}
	if (status & PORT_IRQ_HBUS_DATA_ERR) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Host bus data error * ");
		pos += len;
	}
	if (status & PORT_IRQ_HBUS_ERR) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Host bus fatal error * ");
		pos += len;
	}
	if (status & PORT_IRQ_TF_ERR) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Task file error * ");
		pos += len;
	}
	if (status & PORT_IRQ_COLD_PRES) {
		len = snprintf(&str[pos], LIBAHCI_DEBUG_BUFSZ - pos, "Cold port detect * ");
		pos += len;
	}
	libahci_debug_event(NULL, str, pos);
	kfree(str);
}
EXPORT_SYMBOL_GPL(libahci_debug_dump_irq);

/*
 * Read memory region pointed to by buff and dump its content
 */
void libahci_debug_dump_region(const struct ata_port *ap, const u32 *buff, size_t buff_sz, const char *prefix)
{
	int i;
	int len, sz;
	char *str = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);

	if (!str)
		return;

	len = strlen(prefix);
	if (len < LIBAHCI_DEBUG_BUFSZ) {
		strncpy(str, prefix, len);
	} else {
		len = 0;
	}

	for (i = 0; i < buff_sz; i++) {
		sz = snprintf(&str[len], LIBAHCI_DEBUG_BUFSZ - len, "0x%08x ", buff[i]);
		len += sz;
	}
	libahci_debug_event(ap, str, len);
	kfree(str);
}
EXPORT_SYMBOL_GPL(libahci_debug_dump_region);

/*
 * Copy data from S/G list to linear buffer and dump the data
 */
void libahci_debug_dump_sg(const struct ata_queued_cmd *qc, const char *prefix)
{
	struct scatterlist	*sg;
	int					si;
	int					i;
	int					len;
	int					sz;
	int					line_brk;
	u32					buff_sz = 0;
	u32					buff_ptr = 0;
	char				*buff;
	char				*str;
	u32					*buff_map;

	// Calculate the amount of memory needed
	for_each_sg(qc->sg, sg, qc->n_elem, si) {
		buff_sz += sg_dma_len(sg);
	}
	buff = kzalloc(buff_sz, GFP_KERNEL);
	if (!buff) {
		return;
	}
	str = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);
	if (!str) {
		kfree(buff);
		return;
	}

	// Copy data from all DMA buffers
	dma_sync_sg_for_cpu(&qc->dev->tdev, qc->sg, qc->n_elem, qc->dma_dir);
	for_each_sg(qc->sg, sg, qc->n_elem, si) {
		u32 sg_len = sg_dma_len(sg);

		sz = sg_copy_to_buffer(sg, 1, buff + buff_ptr, sg_len);
		buff_ptr += sz;
	}
	dma_sync_sg_for_device(&qc->dev->tdev, qc->sg, qc->n_elem, qc->dma_dir);

	// Print the content of DMA buffers
	buff_map = (u32 *)buff;
	len = snprintf(str, LIBAHCI_DEBUG_BUFSZ, "\t%s\t%u bytes\n\t", prefix, buff_ptr);
	for (i = 0, line_brk = 0; i < buff_ptr / 4; i++) {
		sz = snprintf(&str[len], LIBAHCI_DEBUG_BUFSZ - len, "0x%08x ", buff_map[i]);
		len += sz;
		line_brk++;
		if (line_brk >= 8) {
			libahci_debug_event(qc->ap, str, len);
			line_brk = 0;
			len = snprintf(str, LIBAHCI_DEBUG_BUFSZ, "\t");
		}
	}
	if (line_brk != 0) {
		libahci_debug_event(qc->ap, str, len);
	}

	//printk(KERN_DEBUG "%s\tdump S/G list", MARKER);

	kfree(buff);
	kfree(str);
}
EXPORT_SYMBOL_GPL(libahci_debug_dump_sg);

static void libahci_debug_read_host_regs(struct ata_host *host, struct host_regs *regs)
{
	struct ahci_host_priv *hpriv = host->private_data;
	void __iomem *host_mmio = hpriv->mmio;

	regs->CAP = readl(host_mmio + HOST_CAP);
	regs->CAP2 = readl(host_mmio + HOST_CAP2);
	regs->GHC = readl(host_mmio + HOST_CTL);
	regs->IS = readl(host_mmio + HOST_IRQ_STAT);
	regs->PI = readl(host_mmio + HOST_PORTS_IMPL);
	regs->VS = readl(host_mmio + HOST_VERSION);
	regs->CCC_CTL = readl(host_mmio + 0x14);
	regs->CCC_PORTS = readl(host_mmio + 0x18);
	regs->EM_CTL = readl(host_mmio + HOST_EM_CTL);
	regs->EM_LOC = readl(host_mmio + HOST_EM_LOC);
	regs->BOHC = readl(host_mmio + 0x28);
}

static void libahci_debug_read_port_regs(struct ata_port *ap, struct port_regs *pr)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	int					i;

	pr->PxCLB = readl(port_mmio + PORT_LST_ADDR);
	pr->PxCLBU = readl(port_mmio + PORT_LST_ADDR_HI);
	pr->PxFB = readl(port_mmio + PORT_FIS_ADDR);
	pr->PxFBU = readl(port_mmio + PORT_FIS_ADDR_HI);
	pr->PxIS = readl(port_mmio + PORT_IRQ_STAT);
	pr->PxIE = readl(port_mmio + PORT_IRQ_MASK);
	pr->PxCMD = readl(port_mmio + PORT_CMD);
	//pr->reserved_1 = readl(port_mmio + PORT_RESERVED_1);
	pr->PxTFD = readl(port_mmio + PORT_TFDATA);
	pr->PxSIG = readl(port_mmio + PORT_SIG);
	pr->PxSSTS = readl(port_mmio + PORT_SCR_STAT);
	pr->PxSCTL = readl(port_mmio + PORT_SCR_CTL);
	pr->PxSERR = readl(port_mmio + PORT_SCR_ERR);
	pr->PxSACT = readl(port_mmio + PORT_SCR_ACT);
	pr->PxCI = readl(port_mmio + PORT_CMD_ISSUE);
	pr->PxSNTF = readl(port_mmio + PORT_SCR_NTF);
	pr->PxFBS = readl(port_mmio + PORT_FBS);
	pr->PxDEVSLP = readl(port_mmio + PORT_DEVSLP);
	for (i = 0; i < PORT_VENDOR_BYTES; i++) {
		pr->reserved_2[i] = readb(port_mmio + 0x70 + i);
	}
}

static int libahci_debug_host_show(struct seq_file *f, void *p)
{
	struct ata_host		*host = f->private;
	struct host_regs	hr = {0};

	libahci_debug_read_host_regs(host, &hr);
	seq_printf(f, "CAP:\t\t0x%08X\n", hr.CAP);
	seq_printf(f, "CAP2:\t\t0x%08X\n", hr.CAP2);
	seq_printf(f, "GHC:\t\t0x%08X\n", hr.GHC);
	seq_printf(f, "IS:\t\t0x%08X\n", hr.IS);
	seq_printf(f, "PI:\t\t0x%08X\n", hr.PI);
	seq_printf(f, "VS:\t\t0x%08X\n", hr.VS);
	seq_printf(f, "CCC_CTL:\t0x%08X\n", hr.CCC_CTL);
	seq_printf(f, "CCC_PORTS:\t0x%08X\n", hr.CCC_PORTS);
	seq_printf(f, "EM_LOC:\t\t0x%08X\n", hr.EM_LOC);
	seq_printf(f, "EM_CTL:\t\t0x%08X\n", hr.EM_CTL);
	seq_printf(f, "BOHC:\t\t0x%08X\n", hr.BOHC);

	seq_printf(f, "\nbuffer location:\t\t0x%08X\n", pElphel_buf->paddr);

	return 0;
}

static int libahci_debug_rdesc_show(struct seq_file *f, void *p)
{
	struct ata_port		*ap = f->private;
	struct port_regs	pr = {0};
	int					i;

	libahci_debug_read_port_regs(ap, &pr);
	seq_printf(f, "PxCLB:\t\t0x%08X\n", pr.PxCLB);
	seq_printf(f, "PxCLBU:\t\t0x%08X\n", pr.PxCLBU);
	seq_printf(f, "PxFB:\t\tx0%08X\n", pr.PxFB);
	seq_printf(f, "PxFBU:\t\t0x%08X\n", pr.PxFBU);
	seq_printf(f, "PxIS:\t\t0x%08X\n", pr.PxIS);
	seq_printf(f, "PxIE:\t\t0x%08X\n", pr.PxIE);
	seq_printf(f, "PxCMD:\t\t0x%08X\n", pr.PxCMD);
	seq_printf(f, "reserved:\t0x%08X\n", pr.reserved_1);
	seq_printf(f, "PxTFD:\t\t0x%08X\n", pr.PxTFD);
	seq_printf(f, "PxSIG:\t\t0x%08X\n", pr.PxSIG);
	seq_printf(f, "PxSSTS:\t\t0x%08X\n", pr.PxSSTS);
	seq_printf(f, "PxSCTL:\t\t0x%08X\n", pr.PxSCTL);
	seq_printf(f, "PxSERR:\t\t0x%08X\n", pr.PxSERR);
	seq_printf(f, "PxSACT:\t\t0x%08X\n", pr.PxSACT);
	seq_printf(f, "PxCI:\t\t0x%08X\n", pr.PxCI);
	seq_printf(f, "PxSNTF:\t\t0x%08X\n", pr.PxSNTF);
	seq_printf(f, "PxFBS:\t\t0x%08X\n", pr.PxFBS);
	seq_printf(f, "PxDEVSLP:\t0x%08X\n", pr.PxDEVSLP);
	seq_printf(f, "reserved area:\t");
	for (i = 0; i < PORT_RESERVED_2; i++) {
		seq_printf(f, "0x%02X ", pr.reserved_2[i]);
	}
	seq_printf(f, "\nVendor specific bytes:\t");
	for (i = 0; i < PORT_VENDOR_BYTES; i++) {
		seq_printf(f, "0x%02X ", pr.PxVS[i]);
	}
	seq_printf(f, "\n");

	return 0;
}

static int libahci_debug_host_open(struct inode *i_node, struct file *f)
{
	return single_open(f, libahci_debug_host_show, i_node->i_private);
}

static int libahci_debug_rdesc_open(struct inode *i_node, struct file *f)
{
	return single_open(f, libahci_debug_rdesc_show, i_node->i_private);
}

static int libahci_debug_events_open(struct inode *i_node, struct file *f)
{
	int					err = 0;
	unsigned long		flags;
	struct libahci_debug_list *list;
	struct ata_port		*port = i_node->i_private;

	debug_list.debug = 1;

	//Create event buffer for current port
	list = kzalloc(sizeof(struct libahci_debug_list), GFP_KERNEL);
	if (!list) {
		err = -ENOMEM;
		goto fail_handler;
	}
	if (!(list->libahci_debug_buf = kzalloc(sizeof(char) * LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL))) {
		err = -ENOMEM;
		kfree(list);
		goto fail_handler;
	}
	list->port_n = port->port_no;
	mutex_init(&list->read_mutex);
	init_waitqueue_head(&list->debug_wait);

	spin_lock_irqsave(&list->debug_list_lock, flags);
	list_add_tail(&list->node, &debug_list.node);
	spin_unlock_irqrestore(&list->debug_list_lock, flags);

	// Associate debug list entry with corresponding file
	f->private_data = list;

fail_handler:
	return err;
}

static ssize_t libahci_debug_events_read(struct file *f, char __user *buff, size_t sz, loff_t *pos)
{
	int					ret = 0, len;
	struct libahci_debug_list *list = f->private_data;

	DECLARE_WAITQUEUE(wait, current);
	mutex_lock(&list->read_mutex);

	while (ret == 0) {
		if (list->head == list->tail) {
			// Buffer is empty, put the queue in sleep mode
			add_wait_queue(&list->debug_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			while (list->head == list->tail) {
				if (f->f_flags & O_NONBLOCK) {
					ret = -EAGAIN;
					break;
				}
				if (signal_pending(current)) {
					ret = -ERESTARTSYS;
					break;
				}

				mutex_unlock(&list->read_mutex);
				schedule();
				mutex_lock(&list->read_mutex);
				set_current_state(TASK_INTERRUPTIBLE);
			}
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&list->debug_wait, &wait);
		}

copy_rest:
		//printk(KERN_DEBUG "%s Read event", MARKER);
		if (list->head != list->tail && ret == 0) {
			if (list->tail > list->head) {
				len = list->tail - list->head;
				if (copy_to_user(buff + ret, &list->libahci_debug_buf[list->head], len)) {
					ret = -EFAULT;
				} else {
					ret += len;
					list->head += len;
				}
			} else {
				len = LIBAHCI_DEBUG_BUFSZ - list->head;
				if (copy_to_user(buff, &list->libahci_debug_buf[list->head], len)) {
					ret = -EFAULT;
				} else {
					ret += len;
					list->head = 0;
					goto copy_rest;
				}
			}
		}
		//printk(KERN_DEBUG "%s\thead now is %u", MARKER, list->head);
	}

	mutex_unlock(&list->read_mutex);
	return ret;
}

static unsigned int libahci_debug_events_poll(struct file *f, struct poll_table_struct *wait)
{
	struct libahci_debug_list *list = f->private_data;

	poll_wait(f, &list->debug_wait, wait);
	if (list->head != list->tail) {
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int libahci_debug_events_release(struct inode *i_node, struct file *f)
{
	struct libahci_debug_list *list = f->private_data;
	unsigned long		flags;

	debug_list.debug = 0;

	spin_lock_irqsave(&list->debug_list_lock, flags);
	list_del(&list->node);
	spin_unlock_irqrestore(&list->debug_list_lock, flags);
	kfree(list->libahci_debug_buf);
	kfree(list);

	return 0;
}

void libahci_debug_event(const struct ata_port *port, char *msg, size_t msg_sz)
{
	int					len;
	int					i;
	u32					tmp;
	char				*format_msg = NULL;
	unsigned long		flags;
	unsigned int		port_index = (port == NULL) ? 0 : port->port_no;
	struct libahci_debug_list *list = NULL, *pos = NULL;

	//printk(KERN_DEBUG "%s Write event: %s", MARKER, msg);
	if (debug_list.debug) {
		format_msg = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);
		if (format_msg != NULL) {
			// Find buffer which this event is addressed to
			list_for_each_entry(list, &debug_list.node, node) {
				if (list->port_n == port_index) {
					pos = list;
				}
			}

			if (pos != NULL) {
				//i = libahci_debug_state_dump(port);
				i = libahci_debug_saxigp1_save(port, 0x3000);
				tmp = libahci_debug_get_fsm_state();

				len = snprintf(format_msg, LIBAHCI_DEBUG_BUFSZ, "%s [%08u; fsm: 0x%08x] %s\n", EVT_MARKER, i, tmp, msg);
				spin_lock_irqsave(&pos->debug_list_lock, flags);
				for (i = 0; i < len; i++) {
					pos->libahci_debug_buf[(pos->tail+ i) % LIBAHCI_DEBUG_BUFSZ] = format_msg[i];
				}
				pos->tail = (pos->tail + len) % LIBAHCI_DEBUG_BUFSZ;
				spin_unlock_irqrestore(&pos->debug_list_lock, flags);

				//printk(KERN_DEBUG "%s\ttail is now %u", MARKER, pos->tail);

				// Wake up the queue which should be sleeping
				wake_up_interruptible(&pos->debug_wait);
			}
			kfree(format_msg);
		}
	} else {
		if (early_buff) {

		}
	}
}
EXPORT_SYMBOL_GPL(libahci_debug_event);

static void libahci_debug_prep_cfis(struct ahci_cmd_fis *cmd, u32 *fis, u8 pmp)
{
	fis[0] = cmd->dw0;
	fis[1] = cmd->dw1;
	fis[2] = cmd->dw2;
	fis[3] = cmd->dw3;
	fis[4] = cmd->dw4;
}

void libahci_debug_exec_cmd(struct ata_port *ap)
{
	void				*cmd_tbl;
	dma_addr_t			cmd_tbl_dma;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_sg		*ahci_sg;
	struct ahci_cmd_hdr	*data;
	void __iomem		*port_mmio = ahci_port_base(ap);
	unsigned int		slot;

	printk(KERN_DEBUG "%s Executing command for port %u", MARKER, ap->port_no);

	slot = readl(port_mmio + PORT_CMD_ISSUE);
	printk(KERN_DEBUG "%s PxCI: 0x%08x", MARKER, slot);
	slot = ffz(slot);

	printk(KERN_DEBUG "%s Preparing command; using slot %u", MARKER, slot);
	cmd_tbl = pp->cmd_tbl + slot * AHCI_CMD_TBL_SZ;
	libahci_debug_prep_cfis(&cmd.fis, cmd_tbl, ap->link.pmp);
	libahci_debug_dump_region(ap, (const u32 *)cmd_tbl, 5, "\tcfis data dump: ");
	/*printk(KERN_DEBUG "%s\tcfis data: DW0 = 0x%08x DW1 = 0x%08x DW2 = 0x%08x DW3 = 0x%08x DW4 = 0x%08x",
			MARKER, cmd.fis.dw0, cmd.fis.dw1, cmd.fis.dw2, cmd.fis.dw3, cmd.fis.dw4);*/

	printk(KERN_DEBUG "%s Preparing one S/G list", MARKER);
	ahci_sg = cmd_tbl + AHCI_CMD_TBL_HDR_SZ;
	ahci_sg->addr = cpu_to_le32(sg_dma_address(&cmd.sg) & 0xffffffff);
	ahci_sg->addr_hi = cpu_to_le32((sg_dma_address(&cmd.sg) >> 16) >> 16);
	ahci_sg->flags_size = cpu_to_le32(sg_dma_len(&cmd.sg));

	printk(KERN_DEBUG "%s Preparing command header", MARKER);
	cmd_tbl_dma = pp->cmd_tbl_dma + slot * AHCI_CMD_TBL_SZ;
	pp->cmd_slot[slot].opts = cpu_to_le32(cmd.hdr.opts);
	pp->cmd_slot[slot].status = 0;
	pp->cmd_slot[slot].tbl_addr = cpu_to_le32(cmd_tbl_dma & 0xffffffff);
	pp->cmd_slot[slot].tbl_addr_hi = cpu_to_le32((cmd_tbl_dma >> 16) >> 16);
	data = &pp->cmd_slot[slot];
	printk(KERN_DEBUG "%s\tchdr data: DW0 = 0x%08x DW1 = 0x%08x DW2 = 0x%08x DW3 = 0x%08x", MARKER, data->opts,
			data->status, data->tbl_addr, data->tbl_addr_hi);

	printk(KERN_DEBUG "%s Issuing command", MARKER);
	writel(1 << slot, port_mmio + PORT_CMD_ISSUE);
	cmd.cmd_sent = true;
}
EXPORT_SYMBOL_GPL(libahci_debug_exec_cmd);

void libahci_debug_irq_notify(const struct ata_port *ap)
{
	int					i, sz, line_brk, ptr;
	size_t				len;
	char				*buff = kzalloc(CMD_DMA_BUFSZ, GFP_KERNEL);
	char				*str = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);
	u32					*buff_map;
	struct ahci_port_priv *pp = ap->private_data;
	u32					*rx_fis = pp->rx_fis;
	void __iomem		*port_mmio = ahci_port_base((struct ata_port *)ap);
	u32					tmp;

	if (!buff || !str)
		return;

	printk(KERN_DEBUG "%s IRQ notify event, flag: %d", MARKER, cmd.cmd_sent);
	if (cmd.cmd_sent) {
		tmp = readl(port_mmio + PORT_SCR_ERR);
		printk(KERN_DEBUG "%s PxSERR: 0x%08x", MARKER, tmp);

		// Read RX FIS memory
		libahci_debug_dump_region(ap, rx_fis + RX_FIS_PIO_SETUP, 5, "\tread PIO SETUP FIS region; dump: ");
		libahci_debug_dump_region(ap, rx_fis + RX_FIS_D2H_REG, 5,  "\tread D2H Register FIS; dump: ");

		dma_sync_sg_for_cpu(ap->dev, &cmd.sg, 1, DMA_BIDIRECTIONAL);
		len = sg_copy_to_buffer(&cmd.sg, 1, buff, CMD_DMA_BUFSZ);
		dma_sync_sg_for_device(ap->dev, &cmd.sg, 1, DMA_BIDIRECTIONAL);

		printk(KERN_DEBUG "%s %u bytes copied from DMA buffer", MARKER, len);

		// Print the content of DMA buffers
		buff_map = (u32 *)buff;
		for (i = 0, line_brk = 0, ptr = 0; i < len / 4; i++) {
			sz = snprintf(&str[ptr], LIBAHCI_DEBUG_BUFSZ - ptr, "0x%08x ", buff_map[i]);
			ptr += sz;
			line_brk++;
			if (line_brk >= 8) {
				libahci_debug_event(ap, str, ptr);
				line_brk = 0;
				ptr = snprintf(str, LIBAHCI_DEBUG_BUFSZ, "\t");
			}
		}
		if (line_brk != 0) {
			libahci_debug_event(ap, str, ptr);
		}

		// Command processed, clear flag
		cmd.cmd_sent = false;
	}
}
EXPORT_SYMBOL_GPL(libahci_debug_irq_notify);

static int libahci_debug_fis_open(struct inode *i_node, struct file *f)
{
	struct ata_port		*ap = i_node->i_private;
//	const char			*name = f->f_path.dentry->d_name.name;
//	void				*buff = NULL;

	/*if (strncmp(name, FILE_NAME_CFIS, 5) == 0) {
		buff = kzalloc(sizeof(struct ahci_cmd_fis), GFP_KERNEL);
	} else if (strncmp(name, FILE_NAME_CHDR, 5) == 0) {
		buff = kzalloc(sizeof(struct ahci_cmd_hdr), GFP_KERNEL);
	}
	if (!buff)
		return -ENOMEM;*/

	f->private_data = ap;

	return 0;
}

static ssize_t libahci_debug_cfis_write(struct file *f, const char __user *buff, size_t buff_sz, loff_t *ppos)
{
	struct ahci_cmd_fis	*data = &cmd.fis;

	if (buff_sz != CMD_FIS_SZ) {
		return -EINVAL;
	}

	data->dw0 = (((buff[3] & 0xff) << 24) | ((buff[2] & 0xff) << 16) | ((buff[1] & 0xff) << 8) | (buff[0] & 0xff));
	data->dw1 = (((buff[7] & 0xff) << 24) | ((buff[6] & 0xff) << 16) | ((buff[5] & 0xff) << 8) | (buff[4] & 0xff));
	data->dw2 = (((buff[11] & 0xff) << 24) | ((buff[10] & 0xff) << 16) | ((buff[9] & 0xff) << 8) | (buff[8] & 0xff));
	data->dw3 = (((buff[15] & 0xff) << 24) | ((buff[14] & 0xff) << 16) | ((buff[13] & 0xff) << 8) | (buff[12] & 0xff));
	data->dw4 = (((buff[19] & 0xff) << 24) | ((buff[18] & 0xff) << 16) | ((buff[17] & 0xff) << 8) | (buff[16] & 0xff));

	//printk(KERN_DEBUG "%s cfis data: DW0 = 0x%08x DW1 = 0x%08x DW2 = 0x%08x DW3 = 0x%08x DW4 = 0x%08x, pos = %lld",
	//		MARKER, data->dw0, data->dw1, data->dw2, data->dw3, data->dw4, *ppos);

	return buff_sz;
}

static ssize_t libahci_debug_chdr_write(struct file *f, const char __user *buff, size_t buff_sz, loff_t *ppos)
{
	struct ahci_cmd_hdr	*data = &cmd.hdr;

	if (buff_sz != CMD_HDR_SZ) {
		return -EINVAL;
	}

	data->opts = (((buff[3] & 0xff) << 24) | ((buff[2] & 0xff) << 16) | ((buff[1] & 0xff) << 8) | (buff[0] & 0xff));
	data->status = (((buff[7] & 0xff) << 24) | ((buff[6] & 0xff) << 16) | ((buff[5] & 0xff) << 8) | (buff[4] & 0xff));
	data->tbl_addr = (((buff[11] & 0xff) << 24) | ((buff[10] & 0xff) << 16) | ((buff[9] & 0xff) << 8) | (buff[8] & 0xff));
	data->tbl_addr_hi = (((buff[15] & 0xff) << 24) | ((buff[14] & 0xff) << 16) | ((buff[13] & 0xff) << 8) | (buff[12] & 0xff));

	//printk(KERN_DEBUG "%s chdr data: DW0 = 0x%08x DW1 = 0x%08x DW2 = 0x%08x DW3 = 0x%08x, pos = %lld", MARKER, data->opts,
	//		data->status, data->tbl_addr, data->tbl_addr_hi, *ppos);

	libahci_debug_exec_cmd(f->private_data);

	return buff_sz;
}

static int libahci_debug_fis_release(struct inode *i_node, struct file *f)
{

	return 0;
}

static ssize_t libahci_debug_load(struct file *f, const char __user *buff, size_t buff_sz, loff_t *ppos)
{
	load_flag = true;

	return buff_sz;
}

/*
 * This function waits until the loading flag is set through debugfs file.
 * The state of the flag is checked every 100ms.
 */
void libahci_debug_wait_flag(void)
{
	printk(KERN_DEBUG "%s Waiting for flag to be written to debugfs", MARKER);
	while (load_flag == false) {
		/*set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(100));*/
		msleep(500);
	}
	load_flag = false;
}
EXPORT_SYMBOL_GPL(libahci_debug_wait_flag);

static void libahci_debug_buff_line(void *mem, u32 cntr)
{
	int i;
	u32 *mem_ptr = mem;

	mem_ptr[0] = cntr;
	for (i = 1; i < MARKER_LEN; i++) {
		mem_ptr[i] = 0xa5a5a5a5;
	}
}

/*
 * Copy controller registers to buffer memory and return the record number
 */
unsigned int libahci_debug_state_dump(struct ata_port *ap)
{
//	static u32 page_cntr;
	int i;
	u32 tmp;
	u32 ptr;
	u32 * buf = (u32 *) (pElphel_buf->vaddr);
	struct device *dev = ap->dev;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	void __iomem *host_mmio = hpriv->mmio;
	struct ahci_port_priv *ppriv = ap->private_data;
	void __iomem *port_mmio = ahci_port_base(ap);

	if (!ap)
		return 0;

	if (!pElphel_buf->vaddr) {
		dev_err(dev, "dump buffer has not been allocated");
		return 0;
	}

	dev_info(dev, "dump page num: %u", page_cntr);
	ptr = page_cntr * DUMP_LEN;
	if (ptr + DUMP_LEN > SEGMENT_SIZE)
		ptr = 0;
	dev_info(dev, "current ptr: %u", ptr);
	for (i = 0; i < GHC_SZ; i++) {
		tmp = ioread32(host_mmio + 4 * i);
		buf[ptr++] = tmp;
	}
	for (i = 0; i < PORT_REG_SZ; i++) {
		tmp = ioread32(port_mmio + 4 * i);
		buf[ptr++] = tmp;
	}
	for (i = 0; i < CLB_SZ; i++) {
		tmp = ioread32(ppriv->cmd_slot);
		buf[ptr++] = tmp;
	}
	for (i = 0; i < FIS_SZ; i++) {
		tmp = ioread32(ppriv->rx_fis);
		buf[ptr++] = tmp;
	}
	libahci_debug_buff_line(pElphel_buf->vaddr +ptr + ALIGN_OFFSET, page_cntr);
	//__cpuc_flush_kern_all();
	//outer_flush_all();
	page_cntr++;

	return page_cntr;
}
EXPORT_SYMBOL_GPL(libahci_debug_state_dump);

unsigned int libahci_debug_saxigp1_save(struct ata_port *ap, size_t dump_size)
{
	struct device *dev;
	u32 * counter_save;
	void *current_ptr;
	static size_t bytes_copied;
	const size_t end = 0x1000000;
	u32 *start_ptr;

	// ap pointer is not initialized on early loading stages, skip debug output
	if (ap) {
		dev = ap->dev;
		if (!ioptr) {
			dev_err(dev, "saxigp1 memory is not mapped");
			return 0; // should be non-zero when error, 0 is OK usually
		}
		if (!pElphel_buf->vaddr) {
			dev_err(dev, "elphel_buf has not been allocated");
			return 0; // should be non-zero when error, 0 is OK usually
		}
	}

	//if (bytes_copied < pElphel_buf->size * PAGE_SIZE - dump_size) {
	if (bytes_copied < end) {
		counter_save = (u32*) (pElphel_buf->vaddr + counter_offset);
		start_ptr = (u32 *)(pElphel_buf->vaddr + buffer_offset + (page_cntr * dump_size));
		//dev_err(dev, "Copying 0x%x bytes of data from saxigp1 to memory",dump_size);
		memcpy_fromio(pElphel_buf->vaddr  + buffer_offset + (page_cntr * dump_size), ioptr, dump_size);
		counter_save[0] = page_cntr;
		counter_save[1] = dump_size;

		page_cntr++;
		bytes_copied += dump_size;
	} else {
		page_cntr = 0;
	}

	return page_cntr;
}
EXPORT_SYMBOL_GPL(libahci_debug_saxigp1_save);

static void libahci_debug_buff_init(struct device *dev)
{
	dev_info(dev, "Nothing to allocate - using elphel_buf allocated at startup");
/*
	mem_buff.vaddr = dmam_alloc_coherent(dev, SEGMENT_SIZE, &mem_buff.paddr, GFP_KERNEL);
	if (!mem_buff.vaddr)
		dev_err(dev, "unable to allocate memory");
	else
		dev_info(dev, "dump buffer allocated");
*/
/*
 const size_t maxigp1_start = 0x80000000; // start of MAXIGP1 physical address range
 const size_t maxigp1_size = 0x3000; // size of register memory to save
 */
	ioptr =  ioremap_nocache(maxigp1_start, maxigp1_size);
	dev_info(dev, "Mapped 0x%08x bytes from physical address 0x%08x to 0x%08x", maxigp1_size, maxigp1_start, (size_t) ioptr);
	page_cntr = 0;
}

static inline u32 libahci_debug_get_fsm_state(void)
{
	return ioread32(ioptr + fsm_state_offset);
}

static const struct file_operations libahci_debug_host_ops = {
	.open				= libahci_debug_host_open,
	.read				= seq_read,
	.llseek				= seq_lseek,
	.release			= single_release,
};

static const struct file_operations libahci_debug_rdesc_ops = {
	.open				= libahci_debug_rdesc_open,
	.read				= seq_read,
	.llseek				= seq_lseek,
	.release			= single_release,
};

static const struct file_operations libahci_debug_events_fops = {
	.owner				= THIS_MODULE,
	.open				= libahci_debug_events_open,
	.read				= libahci_debug_events_read,
	.poll				= libahci_debug_events_poll,
	.release			= libahci_debug_events_release,
	.llseek				= noop_llseek,
};

static const struct file_operations libahci_debug_cfis_ops = {
	.open				= libahci_debug_fis_open,
	.write				= libahci_debug_cfis_write,
	.release			= libahci_debug_fis_release,
};

static const struct file_operations libahci_debug_chdr_ops = {
	.open				= libahci_debug_fis_open,
	.write				= libahci_debug_chdr_write,
	.release			= libahci_debug_fis_release,
};

static const struct file_operations libahci_debug_load_ops= {
		.write			= libahci_debug_load,
};

static int libahci_debug_init_sg(void)
{
	cmd.sg_buff = kzalloc(CMD_DMA_BUFSZ, GFP_KERNEL);
	// mark the area
	memset(cmd.sg_buff, 0xa5, CMD_DMA_BUFSZ);
	if (!cmd.sg_buff) {
		return -ENOMEM;
	} else {
		sg_init_one(&cmd.sg, cmd.sg_buff, CMD_DMA_BUFSZ);
	}

	return 0;
}

int libahci_debug_init_early(struct device *dev)
{
	early_buff = kzalloc(LIBAHCI_DEBUG_BUFSZ, GFP_KERNEL);
	if (!early_buff) {
		dev_err(dev, "unable to allocate mem for early buffer");
		return -ENOMEM;
	} else {
		dev_info(dev, "early buffer allocated");
	}
}

int libahci_debug_init(struct ata_host *host)
{
	int					i;
	char				port_n[] = "port00";
	struct dentry		*node;

	INIT_LIST_HEAD(&debug_list.node);
	printk(KERN_DEBUG "%s Loading debug AHCI driver", MARKER);
	debug_root = debugfs_create_dir(ROOT_DIR_NAME, NULL);
	if (!debug_root) {
		goto fail_handler;
	}
	debugfs_create_file("rdesc_host", 0644,
			debug_root, host, &libahci_debug_host_ops);
	debugfs_create_file("loading", 0222,
			debug_root, host, &libahci_debug_load_ops);

	/* Create subdir for each port and add there several files:
	 * one for port registers, one for port events log and
	 * two files for working with FISes
	 */
	for (i = 0; i < host->n_ports; i++) {
		snprintf(port_n, 7, "port%02d", i);
		node = debugfs_create_dir(port_n, debug_root);
		debugfs_create_file("rdesc_port", 0644,
				node, host->ports[i], &libahci_debug_rdesc_ops);
		debugfs_create_file("events", 0644,
				node, host->ports[i], &libahci_debug_events_fops);
		debugfs_create_file(FILE_NAME_CFIS, 0222,
				node, host->ports[i], &libahci_debug_cfis_ops);
		debugfs_create_file(FILE_NAME_CHDR, 0222,
				node, host->ports[i], &libahci_debug_chdr_ops);
	}

	if (libahci_debug_init_sg() != 0) {
		goto fail_handler;
	}

	libahci_debug_buff_init(host->dev);
	return 0;

fail_handler:
	debugfs_remove_recursive(debug_root);
	printk(KERN_DEBUG "%s Unable to create debugfs file structure", MARKER);
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(libahci_debug_init);

void libahci_debug_exit(void)
{
	kfree(cmd.sg_buff);
	kfree(early_buff);
	debugfs_remove_recursive(debug_root);
}
EXPORT_SYMBOL_GPL(libahci_debug_exit);

MODULE_AUTHOR("Jeff Garzik");
MODULE_DESCRIPTION("Debug AHCI SATA low-level routines");
MODULE_LICENSE("GPL");

