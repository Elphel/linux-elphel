#include <linux/types.h>
#include <linux/delay.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/flashchip.h>
#include <linux/io.h>

#include "nand.h"

#define MICRON_SETFEATURE_ARRAYOP	0x90
#define MICRON_SETFEATURE_ARRAYOP_NORMAL	((uint8_t[]){0,0,0,0})
#define MICRON_SETFEATURE_ARRAYOP_OTP		((uint8_t[]){1,0,0,0})
#define MICRON_SETFEATURE_ARRAYOP_OTPPROTECT	((uint8_t[]){3,0,0,0})

#define MICRON_NUM_OTP_FIRSTPAGE	2
#define MICRON_NUM_OTP_PAGES		30

// Elphel, Rocko
//static int mt29f_get_user_prot_info(struct mtd_info *mtd, struct otp_info *buf, size_t len)
static int mt29f_get_user_prot_info(struct mtd_info *mtd, size_t len, size_t *retlen, struct otp_info *buf)
{
	int i;

	// Elphel, Rocko
	*retlen = 0;

	if (len < MICRON_NUM_OTP_PAGES * sizeof(*buf))
		return -ENOSPC;

	for (i = 0; i < MICRON_NUM_OTP_PAGES; ++i) {

		buf->start = i * mtd->writesize;
		buf->length = mtd->writesize;

		/*
		 * XXX: don't know how to find out, if a page is locked
		 */
		buf->locked = 0;

		buf++;
	}

	return MICRON_NUM_OTP_PAGES * sizeof(*buf);
}

static int mt29f_read_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops;
	int ret;
	u8 get_feature;

	/* Valid pages in otp are 02h-1Fh. */
	if (from > MICRON_NUM_OTP_PAGES << chip->page_shift)
		return -EIO;

	if (from + len > MICRON_NUM_OTP_PAGES << chip->page_shift)
		len = (MICRON_NUM_OTP_PAGES << chip->page_shift) - from;

	from += MICRON_NUM_OTP_FIRSTPAGE << chip->page_shift;

	/* XXX: FL_READING? */
	nand_get_device(mtd, FL_READING);
	chip->select_chip(mtd, 0);

	ret = chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,MICRON_SETFEATURE_ARRAYOP_OTP);
	ndelay(1000);
	chip->cmdfunc(mtd, NAND_CMD_GET_FEATURES, MICRON_SETFEATURE_ARRAYOP, -1);
	get_feature = readb(chip->IO_ADDR_R);
	pr_debug("Feature on: 0x%02x\n",get_feature);

	if (ret)
		goto out;

	ops.len = len;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.mode = 0;
	/*
	 * XXX: some things in nand_do_read_ops might be wrong for OTP. e.g.
	 * chip->pagemask, chip->pagebuf handling, caching
	 */
	ret = nand_do_read_ops(mtd, from, &ops);
	*retlen = ops.retlen;

	/* nand_do_read_ops deselects the chip so reselect here */
	chip->select_chip(mtd, 0);

	chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,MICRON_SETFEATURE_ARRAYOP_NORMAL);
	ndelay(1000);
	chip->cmdfunc(mtd, NAND_CMD_GET_FEATURES, MICRON_SETFEATURE_ARRAYOP, -1);
	get_feature = readb(chip->IO_ADDR_R);
	pr_debug("Feature off: 0x%02x\n",get_feature);

out:
	nand_release_device(mtd);
	return ret;
}

static int mt29f_write_user_prot_reg(struct mtd_info *mtd, loff_t to,
		size_t len, size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops;
	int ret;

	pr_debug("mt29f_write_user_prot_reg start!!!");

	/* Valid pages in otp are 02h-1Fh. */
	if (to > MICRON_NUM_OTP_PAGES << chip->page_shift)
		return -EIO;

	if (to + len > MICRON_NUM_OTP_PAGES << chip->page_shift)
		len = (MICRON_NUM_OTP_PAGES << chip->page_shift) - to;

	to += MICRON_NUM_OTP_FIRSTPAGE << chip->page_shift;

	nand_get_device(mtd, FL_WRITING);

	chip->select_chip(mtd, 0);

	ret = chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,
			MICRON_SETFEATURE_ARRAYOP_OTP);

	if (ret)
		goto out;

	ops.len = len;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.mode = 0;
	/*
	 * some things in nand_do_write_ops might be wrong for OTP. e.g.
	 * chip->pagemask, chip->pagebuf handling
	 */
	ret = nand_do_write_ops(mtd, to, &ops);
	*retlen = ops.retlen;

	/* nand_do_write_ops deselects the chip so reselect here */
	chip->select_chip(mtd, 0);

	chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,
			MICRON_SETFEATURE_ARRAYOP_NORMAL);

out:
	nand_release_device(mtd);
	return ret;
}

static int mt29f_lock_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len)
{
	struct nand_chip *chip = mtd->priv;
	int ret;
	int i;

	/* assert from and len are aligned */
	if (NOTALIGNED(from) || NOTALIGNED(len)) {
		pr_notice("%s: attempt to lock non page aligned data\n",
				__func__);
		return -EINVAL;
	}

	if (!len)
		return 0;

	if (from > MICRON_NUM_OTP_PAGES << chip->page_shift)
		return -EINVAL;

	if (from + len > MICRON_NUM_OTP_PAGES << chip->page_shift)
		return -EINVAL;

	from += MICRON_NUM_OTP_FIRSTPAGE << chip->page_shift;

	nand_get_device(mtd, FL_WRITING);

	chip->select_chip(mtd, 0);

	ret = chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,
			MICRON_SETFEATURE_ARRAYOP_OTPPROTECT);
	if (ret)
		goto out;

	for (i = 0; i < len << chip->page_shift; ++i) {
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0,
				(from << chip->page_shift) + i);

		chip->write_byte(mtd, 0);

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	}

	chip->onfi_set_features(mtd, chip, MICRON_SETFEATURE_ARRAYOP,
			MICRON_SETFEATURE_ARRAYOP_NORMAL);
out:
	nand_release_device(mtd);
	return ret;
}

void nandchip_micron_init(struct mtd_info *mtd, int dev_id)
{
	/*
	 * OTP is available on (at least) Micron's MT29F2G{08,16}AB[AB]EA,
	 * MT29F[48]G{08,16}AB[AB]DA, MT29F16G08AJADA having device IDs:
	 *      0xda, 0xca, 0xaa, 0xba;
	 *      0xdc, 0xcc, 0xac, 0xbc, 0xa3, 0xb3, 0xd3, 0xc3;
	 *      0xd3
	 */
	if (IS_ENABLED(CONFIG_MTD_NAND_OTP) &&
			((dev_id + 0x20) & 0xc0) == 0xc0 &&
			((dev_id & 0x09) == 8 || (dev_id & 0x0f) == 3)) {
		mtd->_get_user_prot_info = mt29f_get_user_prot_info;
		mtd->_read_user_prot_reg = mt29f_read_user_prot_reg;
		mtd->_write_user_prot_reg = mt29f_write_user_prot_reg;
		mtd->_lock_user_prot_reg = mt29f_lock_user_prot_reg;
	}
} 
