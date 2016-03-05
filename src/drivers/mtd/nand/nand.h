#include <linux/mtd/mtd.h>

#define NOTALIGNED(x)	((x & (chip->subpagesize - 1)) != 0)

int nand_get_device(struct mtd_info *mtd, int new_state);
void nand_release_device(struct mtd_info *mtd);

int nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops);
int nand_do_write_ops(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops);

void nandchip_micron_init(struct mtd_info *mtd, int dev_id);
