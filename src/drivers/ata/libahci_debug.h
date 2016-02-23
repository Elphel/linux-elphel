/*
 * libahci_debug.h
 *
 *  Created on: Jan 20, 2016
 *      Author: mk
 */
#include "ahci.h"

#ifndef _LIBAHCI_DEBUG_H_
#define _LIBAHCI_DEBUG_H_

#define ROOT_DIR_NAME	"ahci_exp"
#define FILE_NAME_CFIS	"cfis"
#define FILE_NAME_CHDR	"chdr"
#define MARKER			"++"
#define EVT_MARKER		">"
#define CMD_FIS_SZ		20
#define CMD_HDR_SZ		16
#define CMD_DMA_BUFSZ	512
#define PORT_RESERVED_2		40
#define PORT_VENDOR_BYTES	16
#define LIBAHCI_DEBUG_BUFSZ	16384

struct libahci_debug_list {
	unsigned int		debug;
	unsigned int		port_n;
	char				*libahci_debug_buf;
	int					head;
	int					tail;
	struct list_head	node;
	struct mutex		read_mutex;
	wait_queue_head_t	debug_wait;
	spinlock_t			debug_list_lock;
};

struct simple_buff {
	char				*buff;
	int					head;
	int					tail;
	bool				initialized;
};

struct ahci_cmd_fis {
	__le32				dw0;
	__le32				dw1;
	__le32				dw2;
	__le32				dw3;
	__le32				dw4;
};

struct ahci_cmd {
	struct ahci_cmd_hdr	hdr;
	struct ahci_cmd_fis	fis;
	struct scatterlist	sg;
	char				*sg_buff;
	int					cmd_sent;
};

// AHCI Port registers
struct port_regs {
	// Port command list base address
	u32					PxCLB;
	// Port command list based address upper 32-bits
	u32					PxCLBU;
	// Port FIS base address
	u32					PxFB;
	// Port FIS base address upper 32-bits
	u32					PxFBU;
	u32					PxIS;
	u32					PxIE;
	u32					PxCMD;
	u32					reserved_1;
	u32					PxTFD;
	u32					PxSIG;
	u32					PxSSTS;
	u32					PxSCTL;
	u32					PxSERR;
	u32					PxSACT;
	u32					PxCI;
	u32					PxSNTF;
	u32					PxFBS;
	u32					PxDEVSLP;
	char				reserved_2[PORT_RESERVED_2];
	char				PxVS[PORT_VENDOR_BYTES];
};

struct host_regs {
	u32					CAP;
	u32					GHC;
	u32					IS;
	u32					PI;
	u32					VS;
	u32					CCC_CTL;
	u32					CCC_PORTS;
	u32					EM_LOC;
	u32					EM_CTL;
	u32					CAP2;
	u32					BOHC;
};

int libahci_debug_init(struct ata_host *host);
void libahci_debug_exit(void);
void libahci_debug_event(const struct ata_port *port ,char *msg, size_t msg_sz);
void libahci_debug_dump_region(const struct ata_port *ap, const u32 *buf, size_t buff_sz, const char* prefix);
void libahci_debug_dump_irq(u32 status);
void libahci_debug_dump_sg(const struct ata_queued_cmd *qc, const char *prefix);
void libahci_debug_irq_notify(const struct ata_port *ap);
void libahci_debug_exec_cmd(struct ata_port *ap);

#endif /* _LIBAHCI_DEBUG_H_ */
