/*!***************************************************************************
 * @file   ltc3589.c
 * @brief  control of the Linear Technology LTC3589 8-channel voltage regulator 
 * @copyright Copyright (C) 2013 Elphel, Inc.

 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#undef DEBUG /* should be before linux/module.h - enables dev_dbg at boot in this file */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/i2c/ltc3589.h>
 
#define DRV_VERSION "1.0"
#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

#define CACHE_INIT                      1
#define CACHE_VOLAT                     2

#define LAST_REG                     0x33



struct ltc3589_cache_t {
	u8 flags;
	u8 data;
};
struct ltc3589_data_t {
	int reg_addr; /* used for raw register r/w */
	int simulate; /* do not perform actual i2c writes */
	struct mutex lock;
	struct ltc3589_cache_t cache[LAST_REG+1];
};

static struct i2c_device_id ltc3589_id[] = {
	{ "ltc3589", 0 },
	{ }
};

struct named_fields_t {
	char *name;
	u32 awe;
};

static const struct named_fields_t status_fields[]={
		{"power_good",           LTC3589_AWE_PGSTAT},
		{"pgood_ldo1",           LTC3589_AWE_PGSTAT_LDO1},
		{"pgood_sd1",            LTC3589_AWE_PGSTAT_SD1},
		{"pgood_sd2",            LTC3589_AWE_PGSTAT_SD2},
		{"pgood_sd3",            LTC3589_AWE_PGSTAT_SD3},
		{"pgood_bb",             LTC3589_AWE_PGSTAT_BB},
		{"pgood_ldo2",           LTC3589_AWE_PGSTAT_LDO2},
		{"pgood_ldo3",           LTC3589_AWE_PGSTAT_LDO3},
		{"pgood_ldo4",           LTC3589_AWE_PGSTAT_LDO4},
		{"irqstat",              LTC3589_AWE_IRQSTAT},
		{"irqstat_pgoot_timeout",LTC3589_AWE_IRQSTAT_PGOOD_TIMOUT},
		{"irqstat_near_uv",      LTC3589_AWE_IRQSTAT_NEAR_UV},
		{"irqstat_hard_uv",      LTC3589_AWE_IRQSTAT_HARD_UV},
		{"irqstat_near_therm",   LTC3589_AWE_IRQSTAT_NEAR_THERM},
		{"irqstat_hard_therm",   LTC3589_AWE_IRQSTAT_HARD_THERM},
};
static const struct named_fields_t named_fields[]={
		{"ref1_sd1",        LTC3589_AWE_B1DTV1_REF},
		{"ref2_sd1",        LTC3589_AWE_B1DTV2_REF},
		{"ref1_sd2",        LTC3589_AWE_B2DTV1_REF},
		{"ref2_sd2",        LTC3589_AWE_B2DTV2_REF},
		{"ref1_sd3",        LTC3589_AWE_B3DTV1_REF},
		{"ref2_sd3",        LTC3589_AWE_B3DTV2_REF},
		{"ref1_ldo2",       LTC3589_AWE_L2DTV1_REF},
		{"ref2_ldo2",       LTC3589_AWE_L2DTV2_REF},
		{"ref_ldo4",        LTC3589_AWE_B1DTV1_REF},
		{"dv_dt_sd1",       LTC3589_AWE_B1DTV1_DVDT},
		{"pgood_mask_sd1",  LTC3589_AWE_B1DTV1_PGMASK},
		{"pgood_mask_sd2",  LTC3589_AWE_B2DTV1_PGMASK},
		{"pgood_mask_sd3",  LTC3589_AWE_B3DTV1_PGMASK},
		{"pgood_mask_ldo21",LTC3589_AWE_L2DTV1_PGMASK},
		{"clock_rate_sd1",  LTC3589_AWE_B1DTV2_CLKRATE},
		{"clock_rate_sd2",  LTC3589_AWE_B2DTV2_CLKRATE},
		{"clock_rate_sd3",  LTC3589_AWE_B3DTV2_CLKRATE},
		{"clock_phase_sd1", LTC3589_AWE_B1DTV2_PHASE},
		{"clock_phase_sd2", LTC3589_AWE_B2DTV2_PHASE},
		{"clock_phase_sd3", LTC3589_AWE_B3DTV2_PHASE},
		{"keep_alive_sd1",  LTC3589_AWE_B1DTV2_KEEP_ALIVE},
		{"keep_alive_sd2",  LTC3589_AWE_B2DTV2_KEEP_ALIVE},
		{"keep_alive_sd3",  LTC3589_AWE_B3DTV2_KEEP_ALIVE},
		{"keep_alive_ldo2", LTC3589_AWE_L2DTV1_KEEP_ALIVE},
		{"slew_rate_sd1",   LTC3589_AWE_VRRCR_SD1},
		{"slew_rate_sd2",   LTC3589_AWE_VRRCR_SD2},
		{"slew_rate_sd3",   LTC3589_AWE_VRRCR_SD3},
		{"slew_rate_ldo2",  LTC3589_AWE_VRRCR_LDO2},
		{"oven_ldo4",       LTC3589_AWE_L2DTV2_MODE_LDO4},
		{"oven_only",       LTC3589_AWE_OVEN_ONLY},
};

static const int volatile_registers[]={LTC3589_AWE_IRQSTAT_PGOOD_TIMOUT, LTC3589_AWE_PGSTAT_LDO1, LTC3589_AWE_VCCR,-1};
static const char * chn_names[]={"SD1","SD2","SD3","BB","LDO1","LDO2","LDO3","LDO4"};
static const char * modes[]={"continuous","burst","pulse_skip","invalid"};
static const char * pwr_states[]={"power_off","power_on"};
static const char * wait_states[]={"no_wait",  "wait"};
static const char * reference_sel[]={"reference_sel1",  "reference_sel2"};
/* (register_address << 8) | mask  */
static const u32 register_masks[]= {
		0x07ff,0x12ff,0x20ff,0x24ff,0x25ff,0x26ff,0x27ff,
		0x29ff,0x2aff,0x32ff,0x33ff,0x10ff};
static int make_status_fields(struct device *dev);
static int make_bit_fields(struct device *dev);
static ssize_t invalidate_cache_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t simulate_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_address_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_data_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_address_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_data_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t raw_hex_all_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_help_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_hex_adwe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t power_wait_on_off_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t power_wait_on_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t pgood_timeout_inhibit_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pgood_timeout_inhibit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mode_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t reference_select_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t reference_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t reference_select_go_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t reference_select_go_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t field_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t field_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_field_value (struct device *dev, const char* name, char *buf, int newline);

static ssize_t irq_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t irq_show_txt (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t pwr_bad_good_show (struct device *dev, struct device_attribute *attr, char *buf);

static int get_chn_mode(struct device *dev, char *buf, int chn); /* 0..3 */
static int get_chn_pwr(struct device *dev, char *buf, int chn); /* 0..7 */
static int get_chn_wait(struct device *dev, char *buf, int chn); /* 0..7 */
static int get_ref_sel_go(struct device *dev, char *buf, int chn); /* 0..7 */

static int no_off(const char *str);
static int read_channel_mask(const char * str);
static int read_field (struct i2c_client *client, u32 awe);
static int write_field (struct i2c_client *client, u8 data, u32 awe);
static int write_adwe(struct i2c_client *client, u32 adwe);
static int write_reg(struct i2c_client *client, u8 reg, u8 val, u8 mask);
static int read_reg(struct i2c_client *client, u8 reg);
static void invalidate_cache(struct i2c_client *client);
static int ltc3589_sysfs_register(struct device *dev);
static void ltc3589_init_of(struct i2c_client *client);
static int ltc3589_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltc3589_i2c_remove(struct i2c_client *client);

int ltc3589_read_field (struct i2c_client *client, u32 awe)
{
	return read_field (client, awe);
}
EXPORT_SYMBOL_GPL(ltc3589_read_field);

int ltc3589_write_field (struct i2c_client *client, u8 data, u32 awe)
{
	return write_field (client, data, awe);
}
EXPORT_SYMBOL_GPL(ltc3589_write_field);

int ltc3589_write_adwe(struct i2c_client *client, u32 adwe)
{
	return write_adwe(client, adwe);
}
EXPORT_SYMBOL_GPL(ltc3589_write_adwe);


void ltc3589_set_simulate(struct i2c_client *client, int simulate)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(client);
	clientdata->simulate=simulate;
}
EXPORT_SYMBOL_GPL(ltc3589_set_simulate);


/* raw access to i2c registers, need to set address (9 bits) first, then r/w data */

static DEVICE_ATTR(invalidate_cache, SYSFS_PERMISSIONS & SYSFS_WRITEONLY,   NULL,                invalidate_cache_store);
static DEVICE_ATTR(simulate,         SYSFS_PERMISSIONS,                     simulate_show,       simulate_store);
static DEVICE_ATTR(address,          SYSFS_PERMISSIONS,                     raw_address_show,    raw_address_store);
static DEVICE_ATTR(data,             SYSFS_PERMISSIONS,                     raw_data_show,       raw_data_store);
static DEVICE_ATTR(hex_address,      SYSFS_PERMISSIONS,                     raw_hex_address_show,raw_hex_address_store);
static DEVICE_ATTR(hex_data,         SYSFS_PERMISSIONS,                     raw_hex_data_show,   raw_hex_data_store);
static DEVICE_ATTR(hex_all,          SYSFS_PERMISSIONS & SYSFS_READONLY,    raw_hex_all_show,    NULL);
static DEVICE_ATTR(hex_adwe,         SYSFS_PERMISSIONS,                     raw_hex_adwe_show,   raw_hex_adwe_store);
static DEVICE_ATTR(hex_adwe_help,    SYSFS_PERMISSIONS & SYSFS_READONLY,    raw_hex_adwe_help_show,  NULL);


static struct attribute *raw_dev_attrs[] = {
		&dev_attr_invalidate_cache.attr,
		&dev_attr_simulate.attr,
		&dev_attr_address.attr,
		&dev_attr_data.attr,
		&dev_attr_hex_address.attr,
		&dev_attr_hex_data.attr,
		&dev_attr_hex_all.attr,
		&dev_attr_hex_adwe.attr,
		&dev_attr_hex_adwe_help.attr,
		NULL
};

static const struct attribute_group dev_attr_raw_group = {
	.attrs = raw_dev_attrs,
	.name  = "raw",
};
//static ssize_t irq_show (struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t irq_show_txt (struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
//static ssize_t pwr_bad_good_show (struct device *dev, struct device_attribute *attr, char *buf)

static DEVICE_ATTR(power_good,       SYSFS_PERMISSIONS & SYSFS_READONLY,  pwr_bad_good_show,            NULL);
static DEVICE_ATTR(power_bad,        SYSFS_PERMISSIONS & SYSFS_READONLY,  pwr_bad_good_show,            NULL);
static DEVICE_ATTR(irq,              SYSFS_PERMISSIONS,                   irq_show,                     irq_store);
static DEVICE_ATTR(irq_txt,          SYSFS_PERMISSIONS,                   irq_show_txt,                 irq_store);
static DEVICE_ATTR(power_off,        SYSFS_PERMISSIONS,                   power_wait_on_off_show,       power_wait_on_off_store);
static DEVICE_ATTR(power_on,         SYSFS_PERMISSIONS,                   power_wait_on_off_show,       power_wait_on_off_store);
static DEVICE_ATTR(wait,             SYSFS_PERMISSIONS,                   power_wait_on_off_show,       power_wait_on_off_store);
static DEVICE_ATTR(no_wait,          SYSFS_PERMISSIONS,                   power_wait_on_off_show,       power_wait_on_off_store);
static DEVICE_ATTR(reference_sel1,   SYSFS_PERMISSIONS,                   reference_select_show,        reference_select_store);
static DEVICE_ATTR(reference_sel2,   SYSFS_PERMISSIONS,                   reference_select_show,        reference_select_store);
static DEVICE_ATTR(reference_sel_go, SYSFS_PERMISSIONS,                   reference_select_go_show,     reference_select_go_store);
static DEVICE_ATTR(continuous,       SYSFS_PERMISSIONS,                   mode_show,               mode_store);
static DEVICE_ATTR(burst,            SYSFS_PERMISSIONS,                   mode_show,               mode_store);
static DEVICE_ATTR(pulse_skip,       SYSFS_PERMISSIONS,                   mode_show,               mode_store);
static DEVICE_ATTR(pgood_timeout_inhibit, SYSFS_PERMISSIONS,              pgood_timeout_inhibit_show,   pgood_timeout_inhibit_store);

static struct attribute *control_dev_attrs[] = {
		&dev_attr_power_good.attr,
		&dev_attr_power_bad.attr,
		&dev_attr_irq.attr,
		&dev_attr_irq_txt.attr,
		&dev_attr_power_off.attr,
		&dev_attr_power_on.attr,
		&dev_attr_wait.attr,
		&dev_attr_no_wait.attr,
		&dev_attr_reference_sel1.attr,
		&dev_attr_reference_sel2.attr,
		&dev_attr_reference_sel_go.attr,
		&dev_attr_continuous.attr,
		&dev_attr_burst.attr,
		&dev_attr_pulse_skip.attr,
		&dev_attr_pgood_timeout_inhibit.attr,
		NULL
};

static const struct attribute_group dev_attr_control_group = {
	.attrs = control_dev_attrs,
	.name  = "control",
};
//status_fields[]
static int make_status_fields(struct device *dev)
{
	int retval=-1;
	int index;
	struct attribute **pattrs; /* array of pointers to attibutes */
	struct device_attribute *dev_attrs;
	struct attribute_group *attr_group;
	pattrs = devm_kzalloc(dev,(ARRAY_SIZE(status_fields)+1)*sizeof(pattrs[0]), GFP_KERNEL);
	if (!pattrs) return -ENOMEM;
	dev_attrs = devm_kzalloc(dev, ARRAY_SIZE(status_fields)*sizeof(dev_attrs[0]), GFP_KERNEL);
	if (!dev_attrs) return -ENOMEM;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group) return -ENOMEM;
	memset(dev_attrs,  0, ARRAY_SIZE(status_fields)*sizeof(dev_attrs[0]));
	memset(attr_group, 0, sizeof(*attr_group));
	for (index=0;index<ARRAY_SIZE(status_fields);index++) {
		dev_attrs[index].attr.name=status_fields[index].name;
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS & SYSFS_READONLY;
		dev_attrs[index].show= field_show;
		dev_attrs[index].store=NULL;
		pattrs[index]=&(dev_attrs[index].attr);
	}
	pattrs[index]=NULL;
	attr_group->name  = "status";
	attr_group->attrs =pattrs;
	dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
	if (&dev->kobj) {
		retval = sysfs_create_group(&dev->kobj, attr_group);
	}
	return retval;
}

static int make_bit_fields(struct device *dev)
{
	int retval=-1;
	int index;
	struct attribute **pattrs; /* array of pointers to attibutes */
	struct device_attribute *dev_attrs;
	struct attribute_group *attr_group;
	pattrs = devm_kzalloc(dev,(ARRAY_SIZE(named_fields)+1)*sizeof(pattrs[0]), GFP_KERNEL);
	if (!pattrs) return -ENOMEM;
	dev_attrs = devm_kzalloc(dev, ARRAY_SIZE(named_fields)*sizeof(dev_attrs[0]), GFP_KERNEL);
	if (!dev_attrs) return -ENOMEM;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group) return -ENOMEM;
	memset(dev_attrs,  0, ARRAY_SIZE(named_fields)*sizeof(dev_attrs[0]));
	memset(attr_group, 0, sizeof(*attr_group));
	for (index=0;index<ARRAY_SIZE(named_fields);index++) {
		dev_attrs[index].attr.name=named_fields[index].name;
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		dev_attrs[index].show= field_show;
		dev_attrs[index].store=field_store;
		pattrs[index]=&(dev_attrs[index].attr);
	}
	pattrs[index]=NULL;
	attr_group->name  = "bit_fields";
	attr_group->attrs =pattrs;
	dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
	if (&dev->kobj) {
		retval = sysfs_create_group(&dev->kobj, attr_group);
	}
	return retval;
}


static ssize_t invalidate_cache_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	invalidate_cache(client);
    return count;
}

static ssize_t simulate_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    return sprintf(buf, "%d\n",clientdata->simulate);
}
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    sscanf(buf, "%du", &clientdata->simulate);
    return count;
}


//clientdata->simulate
static ssize_t raw_address_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    return sprintf(buf, "%d\n",clientdata->reg_addr);
}
static ssize_t raw_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    sscanf(buf, "%du", &clientdata->reg_addr);
    return count;
}

static ssize_t raw_data_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	int data= read_reg(client, clientdata->reg_addr);
    return sprintf(buf, "%d\n",data);
}
static ssize_t raw_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	int data;
    sscanf(buf, "%du", &data);
    write_reg(client, clientdata->reg_addr, data, 0xff); /* write all register, it is up to user to do R-mod-W */
    return count;
}

static ssize_t raw_hex_address_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    return sprintf(buf, "0x%02x\n",clientdata->reg_addr);
}
static ssize_t raw_hex_address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ltc3589_data_t *clientdata=i2c_get_clientdata(to_i2c_client(dev));
    sscanf(buf, "%x", &clientdata->reg_addr);
    return count;
}

static ssize_t raw_hex_data_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	int data= read_reg(client, clientdata->reg_addr);
    return sprintf(buf, "0x%02x\n",data);
}
static ssize_t raw_hex_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	int data;
    sscanf(buf, "%x", &data);
    write_reg(client, clientdata->reg_addr, data, 0xff); /* write all register, it is up to user to do R-mod-W */
    return count;
}

static ssize_t raw_hex_all_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int low_addr=0,reg,data,rc,len=0, count=PAGE_SIZE;
	struct i2c_client *client = to_i2c_client(dev);
//	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	for (reg=low_addr;reg<=LAST_REG;reg++) if (count>10){
		if ((reg & 0xf) ==0){
			rc=sprintf(buf, "%02x: ",reg);
		    buf+=rc;
		    len+=rc;
		    count-=rc;
		}
		data= read_reg(client, reg); //ignore errors
		if (data<0) rc=sprintf(buf, "??");
		else        rc=sprintf(buf, "%02x",data);
	    buf+=rc;
	    len+=rc;
	    count-=rc;
		if (((reg & 0xf) == 0xf) || (reg==LAST_REG)){
			rc=sprintf(buf, "\n");
		} else {
			rc=sprintf(buf, " ");
		}
	    buf+=rc;
	    len+=rc;
	    count-=rc;
	}
	return len;
}
static ssize_t raw_hex_adwe_help_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Setting one/multiple registers with masks in the form [0x]AADDWW, where AA is register address\n" \
			           "DD - data byte and WW - write enable bits ( 1 - write, 0 - keep old)\n" \
			           "When read, provides current register data that can be used in device tree.\n");

}

//static const u32 register_masks[]= {
static ssize_t raw_hex_adwe_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int i,data;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0;i<ARRAY_SIZE(register_masks);i++){
		if (((data=read_reg(client, register_masks[i]>>8)))<0) return data;
		buf+=sprintf(buf," 0x%x",((register_masks[i] & 0x1ff00)<<8) | (register_masks[i] & 0xff) | ((data & 0xff)<<8));
		if (((i+1) & 0x7)==0) buf+=sprintf(buf,"\n");  
	}
	buf+=sprintf(buf,"\n");
	return buf-cp;
}

/*
 *  accepts single or multiple data, each [0x]AAADDWW - AAA - register address, DD - data byte, WW - write enable mask (1 - write, 0 - keep).
 *  Ignores any other characters, so same format as in dts with hex data is OK
 */
static ssize_t raw_hex_adwe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const char hex_digits[]="0123456789abcdefABCDEF";
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3589_data_t *clientdata= i2c_get_clientdata(client);
	int adwe,rc=0;
	int left=count,num_bytes;
	char * cp;
 	mutex_lock(&clientdata->lock);
	while ((left>0) && ((cp=strpbrk(buf,hex_digits))) && cp[0]){
		left -= (cp-buf);
		buf = cp;
		dev_dbg(dev,"left=%d", left);
	    sscanf(buf, "%x%n", &adwe,&num_bytes);
	    left-=num_bytes;
	    buf+=num_bytes;
	    dev_dbg(dev,"left=%d num_bytes=%d, adwe=0x%08x", left,num_bytes,adwe);
	    if (((rc=write_adwe(client, adwe)))<0) {
	    	mutex_unlock(&clientdata->lock);
	    	return rc;
	    }
	}
	mutex_unlock(&clientdata->lock);
    return count;
}

//static const char * chn_names[]={"SD1","SD2","SD3","BB","LDO1","LDO2","LDO3","LDO4"}; 
//static const char * pwr_states[]={"power_off","power_on"};
//static const char * wait_states[]={"no_wait",  "wait"};

static ssize_t power_wait_on_off_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,i,invert;
	char * cp=buf;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	awe=   ((strcmp(attr->attr.name,wait_states[0])==0) || (strcmp(attr->attr.name,wait_states[1])==0))?LTC3589_AWE_SCR2:LTC3589_AWE_OVEN;
	invert=((strcmp(attr->attr.name,pwr_states[0])==0) || (strcmp(attr->attr.name,wait_states[0])==0))?0xff:0;
	if (((rc=read_field(client,awe)))<0) return rc;
	rc=((rc & 0xf) | 0x10 | ((rc & 0x70)<<1)) ^ invert;
	for (i=0;i<8;i++) if (rc & (1<<i)) {
		if (buf!=cp) buf+=sprintf(buf," ");
		buf+=sprintf(buf,"%s",chn_names[i]);
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
    
}
static ssize_t power_wait_on_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,mask,data;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	awe= ((strcmp(attr->attr.name,wait_states[0])==0) || (strcmp(attr->attr.name,wait_states[1])==0))?LTC3589_AWE_SCR2:LTC3589_AWE_OVEN;
	data=((strcmp(attr->attr.name,pwr_states[0])==0) || (strcmp(attr->attr.name,wait_states[0])==0))?0:0xff;
	mask=read_channel_mask(buf);
	mask=(mask & 0xf) | ((mask >> 1) & 0x70);
	awe = (awe & 0xff00) | mask;
	if (((rc=write_field (client, data, awe)))<0) return rc;
    return count;
}

static ssize_t pgood_timeout_inhibit_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_SCR2_PGOOD_SHTDN_INH)))<0) return rc;
	return sprintf(buf, "%d\n",rc);
}
static ssize_t pgood_timeout_inhibit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,data;
	struct i2c_client *client = to_i2c_client(dev);
    sscanf(buf, "%d", &data);
	if (((rc=write_field(client,data?1:0,LTC3589_AWE_SCR2_PGOOD_SHTDN_INH)))<0) return rc;
    return count;
}

static ssize_t mode_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,m,i;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_SCR1)))<0) return rc;
	for (m=0;m<ARRAY_SIZE(modes);m++) if (strcmp(attr->attr.name,modes[m])==0) break;
	if (m>=ARRAY_SIZE(modes)) return -EINVAL;
	for (i=0;i<4;i++) if (((rc>>(2*i)) & 3) == m) {
		if (buf!=cp) buf+=sprintf(buf," ");
		buf+=sprintf(buf,"%s",chn_names[i]);
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
    
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,mask,m,data;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	for (m=0;m<ARRAY_SIZE(modes);m++) if (strcmp(attr->attr.name,modes[m])==0) break;
	if (m>=ARRAY_SIZE(modes)) return -EINVAL;
	mask=read_channel_mask(buf);
	mask=((mask & 1)? 3:0) | ((mask & 2)? 0xc:0) | ((mask & 4)? 0x30:0) | ((mask & 8)? 0x40:0);
	awe = (LTC3589_AWE_SCR1 & 0xff00) | mask;
	data= m | (m<<2) | (m<<4) | (m<<6);
	if (((rc=write_field (client, data, awe)))<0) return rc;
    return count;
}

static ssize_t reference_select_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,m,i,chn;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_VCCR)))<0) return rc;
	for (m=0;m<ARRAY_SIZE(reference_sel);m++) if (strcmp(attr->attr.name,reference_sel[m])==0) break;
	if (m>=ARRAY_SIZE(reference_sel)) return -EINVAL;
	for (i=0;i<4;i++) if (((rc>>(2*i+1)) & 1)==m){
		chn=i;
		if (i==3) chn=5 ; /* LDO2 */
		if (buf!=cp) buf+=sprintf(buf," ");
		buf+=sprintf(buf,"%s",chn_names[chn]);
/*		if ((rc>>(2*i)) & 1){
			buf+=sprintf(buf,"(slewing)");
		}
*/		
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

/* not raising go */
static ssize_t reference_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,mask,m,data;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	for (m=0;m<ARRAY_SIZE(reference_sel);m++) if (strcmp(attr->attr.name,reference_sel[m])==0) break;
	if (m>=ARRAY_SIZE(reference_sel)) return -EINVAL;
	mask=read_channel_mask(buf);
	mask=((mask & 1)? 2:0) | ((mask & 2)? 0x8:0) | ((mask & 4)? 0x20:0) | ((mask & 0x20)? 0x80:0);
	
	awe = (LTC3589_AWE_VCCR & 0xff00) | mask;
	data= m?0xff:0;
	if (((rc=write_field (client, data, awe)))<0) return rc;
    return count;
}

static ssize_t reference_select_go_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,i,chn;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_VCCR)))<0) return rc;
	for (i=0;i<4;i++) if ((rc>>(2*i)) & 1){
		chn=i;
		if (i==3) chn=5 ; /* LDO2 */
		if (buf!=cp) buf+=sprintf(buf," ");
		buf+=sprintf(buf,"%s",chn_names[chn]);
	}
	buf+=sprintf(buf,"\n");
    return buf-cp;
}

static ssize_t reference_select_go_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,mask,data;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	mask=read_channel_mask(buf);
	mask=((mask & 1)? 1:0) | ((mask & 2)? 0x4:0) | ((mask & 4)? 0x10:0) | ((mask & 0x20)? 0x40:0);
	awe = (LTC3589_AWE_VCCR & 0xff00) | mask;
	data= 0xff;
	if (((rc=write_field (client, data, awe)))<0) return rc;
    return count;
}
//--------------------
static ssize_t field_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	return get_field_value (dev, attr->attr.name, buf, 1); /* with newline */
}

static ssize_t field_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc,i;
	int data;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0;i<ARRAY_SIZE(named_fields);i++) if (strcmp(attr->attr.name,named_fields[i].name)==0) {
		awe=named_fields[i].awe;
		dev_dbg(dev,"i=%d, field name=%s awe=0x%04x\n", i, named_fields[i].name, (int) awe);
		break;
	}
	if (i>=ARRAY_SIZE(named_fields)) {
		for (i=0;i<ARRAY_SIZE(status_fields);i++) if (strcmp(attr->attr.name,status_fields[i].name)==0) {
			awe=status_fields[i].awe;
			dev_dbg(dev,"i=%d, status field name=%s awe=0x%04x\n", i, status_fields[i].name, (int) awe);
			break;
		}
		if (i>=ARRAY_SIZE(status_fields)) return -EINVAL;
	}
    sscanf(buf, "%du", &data);
	if (((rc=write_field (client, data, awe)))<0) return rc;
    return count;
}

static ssize_t get_field_value (struct device *dev, const char* name, char *buf, int newline)
{
	int rc,i;
	char * cp=buf;
	u32 awe;
	struct i2c_client *client = to_i2c_client(dev);
	for (i=0;i<ARRAY_SIZE(named_fields);i++) if (strcmp(name,named_fields[i].name)==0){
		awe=named_fields[i].awe;
		dev_dbg(dev,"i=%d, field name=%s awe=0x%04x\n", i, named_fields[i].name, (int) awe);
		break;
	}
	if (i>=ARRAY_SIZE(named_fields)) {
		for (i=0;i<ARRAY_SIZE(status_fields);i++) if (strcmp(name,status_fields[i].name)==0){
			awe=status_fields[i].awe;
			dev_dbg(dev,"i=%d, status field name=%s awe=0x%04x\n", i, status_fields[i].name, (int) awe);
			break;
		}
		if (i>=ARRAY_SIZE(status_fields)) return -EINVAL;
	}
	if (((rc=read_field(client,awe)))<0) return rc;
	buf+=sprintf(buf,"%d",rc);
	if (newline) buf+=sprintf(buf,"\n");
	return buf-cp;
}

static ssize_t irq_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_IRQSTAT)))<0) return rc;
	return sprintf(buf,"%d\n",rc);
}

static ssize_t irq_show_txt (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_IRQSTAT)))<0) return rc;
	buf += sprintf(buf,"0x%02x",rc);
	if (rc & LTC3589_AWE_IRQSTAT_PGOOD_TIMOUT) buf += sprintf(buf,"PGOOD timeout");
	if (rc & LTC3589_AWE_IRQSTAT_NEAR_UV)      buf += sprintf(buf,", Near undervoltage");
	if (rc & LTC3589_AWE_IRQSTAT_HARD_UV)      buf += sprintf(buf,", Hard undervoltage");
	if (rc & LTC3589_AWE_IRQSTAT_NEAR_THERM)   buf += sprintf(buf,", Near undervoltage");
	if (rc & LTC3589_AWE_IRQSTAT_HARD_THERM)   buf += sprintf(buf,", Hard undervoltage");
	buf += sprintf(buf,"\n");
	return buf-cp;
}

static ssize_t irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=write_field (client, 0, LTC3589_AWE_CLIRQ)))<0) return rc;
    return count;
}
static ssize_t pwr_bad_good_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc,i,pg;
	char * cp=buf;
	struct i2c_client *client = to_i2c_client(dev);
	if (((rc=read_field(client,LTC3589_AWE_PGSTAT)))<0) return rc;
	pg=(rc & 0xe0) | ((rc>>1) & 0xff) | ((rc << 4) & 0x10);
	
	if (strstr(attr->attr.name,"bad")){
		pg^=0xff;
	}
	for (i=0;i<8;i++) if (pg & (1<<i)) buf+=sprintf(buf," %s",chn_names[i]);
	buf += sprintf(buf,"\n");
	return buf-cp;
}
static int get_chn_mode(struct device *dev, char *buf, int chn) /* 0..3 */
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if ((chn<0) || (chn>3)) return 0;
	if (((rc=read_field(client,LTC3589_AWE_SCR1)))<0) return rc;
	return sprintf (buf,"%s",modes[(rc >> (chn<<1)) &3]);
}
static int get_chn_pwr(struct device *dev, char *buf, int chn) /* 0..7 */
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if ((chn<0) || (chn>7)) return 0;
	if (((rc=read_field(client,LTC3589_AWE_OVEN)))<0) return rc;
	rc=(rc & 0xf) | 0x10 | ((rc & 0x70)<<1);
	return sprintf (buf,"%s",pwr_states[(rc >> chn) & 1]);
}
static int get_chn_wait(struct device *dev, char *buf, int chn) /* 0..7 */
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if ((chn<0) || (chn>7)) return 0;
	if (((rc=read_field(client,LTC3589_AWE_SCR2)))<0) return rc;
	rc=(rc & 0xf) | 0x10 | ((rc & 0x70)<<1);
	return sprintf (buf,"%s",wait_states[(rc >> chn) & 1]);
}

static int get_ref_sel_go(struct device *dev, char *buf, int chn) /* 0..7 */
{
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	if ((chn<0) || ((chn>2) && (chn!=5))) return 0;
	if (chn==5) chn=3;
	if (((rc=read_field(client,LTC3589_AWE_VCCR)))<0) return rc;
	rc =(rc >> (2*chn)) & 3;
	return sprintf (buf,"%s%s",reference_sel[(rc>>1)&1],(rc&1)?" (slewing)":"");
}

static int no_off(const char *str)
{
	return strstr(str,"_off")?0:1; 	
}

static int read_channel_mask(const char * str)
{
	int mask =0, i;
	for (i=0;i<ARRAY_SIZE(chn_names);i++) if (strstr(str,chn_names[i])) mask |= 1<<i;
	return mask;
}

static int read_field (struct i2c_client *client, u32 awe)
{
	int rc,nshift;
	u8 mask;
	u8 reg;
	reg=awe>>8;
	mask=awe&0xff;
	if (mask!=0){
		nshift=0;
		while (((1<<nshift) & mask)==0) nshift++;
		if (((rc=read_reg(client, reg)))<0) return rc;
		return (rc & mask) >> nshift;
	}
	return 0;
}


static int write_field (struct i2c_client *client, u8 data, u32 awe)
{
	int rc,nshift;
	u8 mask,reg_data;
	u8 reg;
	reg=awe>>8;
	mask=awe&0xff;
	if (mask!=0){
		nshift=0;
		while (((1<<nshift) & mask)==0) nshift++;
		reg_data=(data & 0xff) << nshift;
		if (((rc=write_reg(client, reg, reg_data, mask)))<0) return rc;
	}
	return 0;
}

static int write_adwe(struct i2c_client *client, u32 adwe)
{
	u8 we=   adwe & 0xff;
	u8 data= (adwe>>8) & 0xff;
	u16 reg=  (adwe>>16) & 0xff;
	return write_reg(client, reg, data, we);
}

static int write_reg(struct i2c_client *client, u8 reg, u8 val, u8 mask)
{
	int rc;
	struct ltc3589_data_t *clientdata = i2c_get_clientdata(client);
	if (reg>LAST_REG) return -EINVAL;
	if (mask==0) return 0;
	dev_dbg(&client->dev,"reg=0x%x, val=0x%x, mask=0x%x\n", (int) reg, (int) val, (int) mask);
	if (mask !=0xff){
	    if (((rc=read_reg(client, reg)))<0) return rc;
	    val=((val ^ rc) & mask)^ rc;
	    if ((val==rc) && !(clientdata->cache[reg].flags & CACHE_VOLAT)) {
	    	dev_dbg(&client->dev,"No change and not volatile -> no write\n");
	    	return 0;
	    }
	}
	clientdata->cache[reg].data= val;
	clientdata->cache[reg].flags |= CACHE_INIT;
	if (clientdata->simulate){
		dev_info(&client->dev,">>> Simulating LTC3589 register write: 0x%02x->[0x%02x]\n",(int) reg, (int) val);
		return 0;
	}
	return i2c_smbus_write_byte_data(client, reg, val);
}


static int read_reg(struct i2c_client *client, u8 reg)
{
	int rc;
	struct ltc3589_data_t *clientdata = i2c_get_clientdata(client);
	if (reg>LAST_REG) return -EINVAL;
	if (clientdata && (clientdata->cache[reg].flags & CACHE_INIT) && !(clientdata->cache[reg].flags & CACHE_VOLAT)){
		dev_dbg(&client->dev,"Using cached register: reg=0x%x -> 0x%x\n",reg,(int) clientdata->cache[reg].data);
		return clientdata->cache[reg].data;
	}
	rc= i2c_smbus_read_byte_data(client, reg & 0xff);
	dev_dbg(&client->dev,"reading i2c device : slave=0x%x, reg=0x%x -> 0x%x\n",(int) (client->addr),reg,rc);
	if (rc<0) return rc;
	if (clientdata){
		clientdata->cache[reg].data= (u8) rc;
		clientdata->cache[reg].flags |= CACHE_INIT;
	}
	return rc;
}
static void invalidate_cache(struct i2c_client *client)
{
	int i;
	struct ltc3589_data_t *clientdata = i2c_get_clientdata(client);
	for (i=0;i<=LAST_REG;i++){
		clientdata->cache[i].flags&= ~CACHE_INIT;
	}
}

static int ltc3589_sysfs_register(struct device *dev)
{
	int retval=0;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_raw_group)))<0) return retval;
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_control_group)))<0) return retval;
		if (((retval = make_status_fields (dev)))<0) return retval;
		if (((retval = make_bit_fields (dev)))<0) return retval;
	}
	return retval;
}

static void ltc3589_init_of(struct i2c_client *client)
{
	//	struct device *dev=&client->dev;
	const __be32 * config_data;
    const char *   init_type_string;
    int init_type=0; /* 0 - none, 1 - always, 2 - if not running (TODO) */
	struct device_node *node = client->dev.of_node;
	int len,i,n;

	char buf[40];

	
	struct ltc3589_setup_data  {
		u8		page;
		u8		reg;
		u8		data;
		u8		mask;
	};
	struct ltc3589_setup_data setup_data;
	__be32 * setup_data_be32= (__be32 *) &setup_data;
/* add stuff */	
#if 0	
	if (node) {
		init_type_string = of_get_property(client->dev.of_node, "ltc3589,init", &len);
		if (init_type_string){
			if      (strcmp(init_type_string,"always")==0) init_type=1;
			else if (strcmp(init_type_string,"if off")==0) init_type=2;
			else {
				dev_err(&client->dev,"Unrecognized ltc3589 initialization type '%s'. Only 'always' and 'if off' are permitted\n",init_type_string);
			}
		}
		switch (init_type){
		case 2:
			// static int is_set_up(struct i2c_client *client);
			i=is_set_up(client);
			if (i<0){
				dev_err(&client->dev,"Error reading i2c register, aborting initialization\n");
				return;
			} else if (i>0){
				init_type=0;
				dev_dbg(&client->dev,"Skipping conditional initialization (some driver variables will not be initialized)\n");
				return;
			}
			init_type=1;
			/* falling to initialization */
		case 1:
			pre_init(client,1); // clear outputs and muxes - they will be programmed later
			break;
		}

		config_data = of_get_property(client->dev.of_node, "ltc3589,configuration_data", &len);
		if (config_data){
			len /= sizeof(*config_data);
			dev_dbg(&client->dev,"Read %d values\n",len);
			dev_dbg(&client->dev,"Found %d items in 'ltc3589,configuration_data' in the Device Tree\n",len);
			for (i=0;i<len;i++){
				*setup_data_be32=config_data[i];
				page_reg=setup_data.reg+(setup_data.page<<8);
				dev_dbg(&client->dev,"page_reg=0x%03x, data=0x%02x, mask=0x%02x \n",
						(int) page_reg,(int)setup_data.data,(int)setup_data.mask);
				if (write_reg(client, page_reg, setup_data.data, setup_data.mask)<0) return;
			}
		}
		/* input section */
		/* setting input frequency here divides (if needed) and feeds it to the PLL reference. Other variants can use raw register writes */
		for (n=0;in_freq_names[n];n++){
			sprintf(buf,"ltc3589,%s",in_freq_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				dev_dbg(&client->dev,"Found '%s', value = %d (0x%x)\n",buf,(int)(be32_to_cpup(config_data)),(int)(be32_to_cpup(config_data)));
				if (set_in_frequency(client, be32_to_cpup(config_data),n)<0) return; /* 32 bits are sufficient here */
			}
		}
		/* setting PLL for the most important output frequency, sets analog parameters accordingly. Assumes input frequency set above */
		for (n=0;pll_setup_names[n];n++){
			sprintf(buf,"ltc3589,%s",pll_setup_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				freq[0]=be32_to_cpup(config_data);
				if (len<3){
					freq[1]=0;
					freq[2]=1;
				} else {
					freq[1]=be32_to_cpup(&config_data[1]);
					freq[2]=be32_to_cpup(&config_data[2]);
				}
				dev_dbg(&client->dev,"Found '%s', value = %lld+(%lld/%lld)\n",buf,freq[0],freq[1],freq[2]);
				if (n & 2){ /* by output */
					if (set_pll_freq_by_out(client, freq, n & 1)<0) return;
				} else { /* directly set  PLL frequency */
					if (set_pll_freq       (client, freq, n & 1)<0) return;
				}
				if (set_pll_paremeters(client)<0) return;
/*				if (set_misc_registers(client)<0) return; */ /* moved to pre_init() */ 
			}
		}
		/* setting MSn dividers (same channel as output), powering them up, setting output dividers and routing outputs */
		for (n=0;out_freq_setup_names[n];n++){
			sprintf(buf,"ltc3589,%s",out_freq_setup_names[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				freq[0]=be32_to_cpup(config_data);
				if (len<3){
					freq[1]=0;
					freq[2]=1;
				} else {
					freq[1]=be32_to_cpup(&config_data[1]);
					freq[2]=be32_to_cpup(&config_data[2]);
				}
				dev_dbg(&client->dev,"Found '%s', value = %lld+(%lld/%lld)\n",buf,freq[0],freq[1],freq[2]);
				if (set_out_frequency_and_route(client, freq, n&3, n>>2)<0) return;
			}
		}
		/* configure output driver standard */
		for (n=0;drv_configs[n].description;n++){
			sprintf(buf,"ltc3589,%s",drv_configs[n].description);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (configure_output_driver(&client->dev, drv_configs[n].description, setup_data.mask)<0) return;
				}
			}
		}

		/* configure disabled state of the output(s) */
		for (n=0;out_dis_states[n];n++){
			sprintf(buf,"ltc3589,%s",out_dis_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_disable(client, n, setup_data.mask)<0) return;
				}
			}
		}

		/* configure powerdown state of the output(s) */
		for (n=0;out_pwr_states[n];n++){
			sprintf(buf,"ltc3589,%s",out_pwr_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_powerdown(client, n, setup_data.mask)<0) return;
				}
			}
		}

		/* configure output enable state of the output(s) */
		for (n=0;out_en_states[n];n++){
			sprintf(buf,"ltc3589,%s",out_en_states[n]);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data){
				len /= sizeof(*config_data);
				for (i=0;i<len;i++){
					*setup_data_be32=config_data[i];
					dev_dbg(&client->dev,"Setting '%s', channel %d",buf,setup_data.mask);
					if (set_drv_disable(client, n, setup_data.mask)<0) return;
				}
			}
		}
		/* setting spread spectrum parameters */
		for (n=0;n<4;n++){
			sprintf(buf,"ltc3589,spread_spectrum_%d",n);
			config_data = of_get_property(client->dev.of_node, buf, &len);
			if (config_data && (len>0)){
				len /= sizeof(*config_data);
				rate=get_ss_down_rate(client, n);
				amp= get_ss_down_amplitude(client, n);
				if (len>1) amp =  be32_to_cpup(&config_data[1]);
				if (len>2) rate = be32_to_cpup(&config_data[2]);
				if (store_ss_down_parameters(client, rate, amp, n)==0){ 
					dev_dbg(&client->dev,"Set spread spectrum parameters for MS%d, amplitude=%d (*0.01%%), rate=%d Hz, %s\n",
							n,amp,rate,config_data[0]?"ON":"OFF");
				} else {
					dev_err(&client->dev,"Failed to set spread spectrum parameters for MS%d, amplitude=%d (*0.01%%), rate=%d Hz, %s\n",
							n,amp,rate,config_data[0]?"ON":"OFF");
					continue;
				}
				if (config_data[0]){ /* enable SS */
					if ((set_ss_down(client, n)==0) && /* calculate and set SS registers */
							(set_ss_state(client, 1, n)==0)){ // enable SS. Not using enable_spread_spectrum() as we'll reset MS later anyway
						dev_dbg(&client->dev,"Spread spectrum enabled for MS%d\n",n);
					} else {
						dev_dbg(&client->dev,"Fail to enable spread spectrum for MS%d\n",n);
					}
				}
			}
		}
	} else {
		dev_info(&client->dev,"Device tree data not found for %s\n",client->name);
	}
	if (init_type){
		if (post_init(client,INIT_TIMEOUT)<0) dev_err(&client->dev,"ltc3589 initialization failed\n");
		else dev_info(&client->dev,"ltc3589 initialized\n");
	}
#endif
}

static int ltc3589_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i,rc=0;
	struct ltc3589_data_t *clientdata = NULL;
	/* initialize i2c ... */
	if (read_field (client, LTC3589_AWE_IRQSTAT_PGOOD_TIMOUT)<0) {
		dev_err(&client->dev, "%s: chip not detected\n",id->name);
		return -EIO;
	}
	dev_info(&client->dev,
		 "Chip %s is found, driver version %s\n", id->name, DRV_VERSION);
	clientdata = devm_kzalloc(&client->dev, sizeof(*clientdata), GFP_KERNEL);
	for (i=0;i<=LAST_REG;i++){
		clientdata->cache[i].flags=0;
		clientdata->cache[i].data=0;
	}
	for (i=0;volatile_registers[i]>=0;i++){
		clientdata->cache[volatile_registers[i]>>8].flags |= CACHE_VOLAT;
	}
	clientdata->simulate=0;
	clientdata->reg_addr=0;
	//volatile_registers[]
	i2c_set_clientdata(client, clientdata);
	ltc3589_sysfs_register(&client->dev);
	mutex_init(&clientdata->lock);
	ltc3589_init_of(client);
	return 0;
}

static int ltc3589_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static struct i2c_driver ltc3589_i2c_driver = {
	.driver = {
		.name	= "ltc3589",
		.owner	= THIS_MODULE,
	},
	.probe		= ltc3589_i2c_probe,
	.remove		= ltc3589_i2c_remove,
	.id_table	= ltc3589_id,
};
module_i2c_driver(ltc3589_i2c_driver);
MODULE_DEVICE_TABLE(i2c, ltc3589_id);
MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("LTC3589 I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ltc3589");
