#include <linux/mtd/mtd.h>

#define NOTALIGNED(x)	((x & (chip->subpagesize - 1)) != 0)

int nand_check_wp(struct mtd_info *mtd);
int check_offs_len(struct mtd_info *mtd, loff_t ofs, uint64_t len);
int nand_get_device(struct mtd_info *mtd, int new_state);
void nand_release_device(struct mtd_info *mtd);
int nand_do_read_ops(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops);
int nand_do_write_ops(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops);

void nand_micron_mt29f_init(struct mtd_info *mtd, int dev_id);
