/*!***************************************************************************
 *! FILE NAME  : elphel393-pwr.c
 *! DESCRIPTION: power supplies control on Elphel 10393 board 
 *! Copyright (C) 2013-2016 Elphel, Inc.
 *! -----------------------------------------------------------------------------**
 *!
 *!  This program is free software: you can redistribute it and/or modify
 *!  it under the terms of the GNU General Public License as published by
 *!  the Free Software Foundation, either version 2 of the License, or
 *!  (at your option) any later version.
 *!
 *!  This program is distributed in the hope that it will be useful,
 *!  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *!  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *!  GNU General Public License for more details.
 *!
 *!  You should have received a copy of the GNU General Public License
 *!  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#undef DEBUG /* should be before linux/module.h - enables dev_dbg at boot in this file */
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/i2c/ltc3589.h>
#include <linux/math64.h>

#define DRIVER_DESCRIPTION	"Elphel 10393 power supply control"
#define DRIVER_VERSION		"1.00"

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222


#define GPIO_CHIP1_ADDR 0x20
#define GPIO_CHIP2_ADDR 0x21
#define GPIO_10389_U4_ADDR 0x25
#define LTC3589_ADDR    0x34

/* TODO: set resistors in device tree to accommodate different revisions ( elphel393_pwr,vp15_r1 = <357000>)*/
#define VP15_R1       357000
#define VP15_R2       287000
#define VCC_SENS01_R1 787000
#define VCC_SENS01_R2 287000
#define VCC_SENS23_R1 787000
#define VCC_SENS23_R2 287000
#define VP5_R1        523000
#define VP5_R2        100000
#define VLDO18_R1     357000
#define VLDO18_R2     287000

#define PINSTRAPPED_OVEN         1
#define REF_FIXED_TENTH_MV    8000
#define REF_VAR_0_TENTH_MV    3625
#define REF_VAR_STEP_TENTH_MV  125
#define DEAFULT_TIMEOUT        300 /* number of retries testing pgood before giving up */


struct pwr_gpio_t {
	const char * label;
	int pin;
	int dir;     /* direction: 0 - in, 1 - out*/
	int out_val; /* output value */
};

struct elphel393_pwr_data_t {
	int chip_i2c_addr[4];
	struct device * ltc3489_dev;
	struct pwr_gpio_t pwr_gpio [16];
	int simulate; /* do not perform actual i2c writes */
	struct mutex lock;
	int pgoot_timeout;
	int pinstrapped_oven;
};

struct voltage_reg_t {
	const char * name;
	int r1;         /* resistor in ohms, if <=0 - r2 is voltage in mv */
	int r2;         /* resistor in ohms, if r1<=0 - voltage in mv */
	int awe_ref;    /* 0 - no control, -1 - margining VP10, -2 - margining VP18 */ 
	int awe_en;     /* 0 - no control, negative - -1-gpio_index */
	int awe_pgood;  /* 0 - no status , negative - -1-gpio_index */
	int mask_pgood; /* 1 - temporarily disable pgood when turning on/changing voltage */
	int awe_slew;
};

static struct voltage_reg_t voltage_reg[]={
		{
				.name="vp15",
				.r1=VP15_R1,
				.r2=VP15_R2,
				.awe_ref=LTC3589_AWE_B1DTV1_REF,
				.awe_en=0,
				.awe_pgood=LTC3589_AWE_PGSTAT_SD1,
				.mask_pgood=1,
				.awe_slew=LTC3589_AWE_VCCR_SLEW_SD1
		},
		{
				.name="vcc_sens01",
				.r1=VCC_SENS01_R1,
				.r2=VCC_SENS01_R2,
				.awe_ref=LTC3589_AWE_B2DTV1_REF,
				.awe_en=LTC3589_AWE_OVEN_EN_SD2,
				.awe_pgood=LTC3589_AWE_PGSTAT_SD2,
				.mask_pgood=1,
				.awe_slew=LTC3589_AWE_VCCR_SLEW_SD2
		},
		{
				.name="vcc_sens23",
				.r1=VCC_SENS23_R1,
				.r2=VCC_SENS23_R2,
				.awe_ref=LTC3589_AWE_B3DTV1_REF,
				.awe_en=LTC3589_AWE_OVEN_EN_SD3,
				.awe_pgood=LTC3589_AWE_PGSTAT_SD3,
				.mask_pgood=1,
				.awe_slew=LTC3589_AWE_VCCR_SLEW_SD3
		},
		{
				.name="vp5",
				.r1=VP5_R1,
				.r2=VP5_R2, 
				.awe_ref=0,
				.awe_en=LTC3589_AWE_OVEN_EN_BB,
				.awe_pgood=LTC3589_AWE_PGSTAT_BB,
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="vldo18",
				.r1=VLDO18_R1,
				.r2=VLDO18_R2,
				.awe_ref=0,
				.awe_en= 0,
				.awe_pgood=LTC3589_AWE_PGSTAT_LDO1,
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="vp33sens01",
				.r1=-1,
				.r2=33000,
				.awe_ref=0,
				.awe_en= -7, /* SENSPWREN0 */
				.awe_pgood=0,
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="vp33sens23",
				.r1=-1,
				.r2=33000,
				.awe_ref=0,
				.awe_en= -8, /* SENSPWREN1 */
				.awe_pgood=0,
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="mmtavcc10",
				.r1=-1,
				.r2=10000,
				.awe_ref=0,
				.awe_en= 0,
				.awe_pgood=-15, /* MGTAVTTGOOD */
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="mmtavtt12",
				.r1=-1,
				.r2=12000,
				.awe_ref=0,
				.awe_en= 0,
				.awe_pgood=-15, /* MGTAVTTGOOD */
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="vp10",
				.r1=-1,
				.r2=10000,
				.awe_ref=-1,
				.awe_en= 0,
				.awe_pgood=-16, /* PGOOD18 */
				.mask_pgood=1,
				.awe_slew=0
		},
		{
				.name="vp18",
				.r1=-1,
				.r2=18000,
				.awe_ref=-2,
				.awe_en= 0,
				.awe_pgood=-16, /* PGOOD18 */
				.mask_pgood=1,
				.awe_slew=0
		},
}; 

static struct pwr_gpio_t pwr_gpio[16]={
/* 0x20: */
		{"PWR_MGB1",    0, 0, 0}, /* 1.8V margining magnitude (0 - 5%, 1 - 10%, float - 15%) */
		{"PWR_MG1",     1, 0, 0}, /* 1.8V margining enable 0 - negative margining, 1 - positive margining, float - no margining */
		{"PWR_MGB0",    2, 0, 0}, /* 1.0V margining magnitude (0 - 5%, 1 - 10%, float - 15%) */
		{"PWR_MG0",     3, 0, 0},  /* 1.0V margining enable 0 - negative margining, 1 - positive margining, float - no margining */
		{"PWR_FQ0",     4, 0, 0}, /*   float - nominal frequency (should float for SS), 0 - 0.67 nominal frequency, 1 - 1.5 nominal frequency */ 
		{"PWR_SS",      5, 0, 0}, /*  Spread spectrum, 0 or float - spread spectrum disabled  */
		{"SENSPWREN0",  6, 0, 0}, /*  1 - enable 3.3 power to sensor connectors J6 and J7 (0 or float - disable) */
		{"SENSPWREN1",  7, 0, 0}, /*  1 - enable 3.3 power to sensor connectors J8 and J9 (0 or float - disable) */
/* 0x21: */
		{"NSHUTDOWN",   8, 0, 0}, /*  (pulled up). 0 - shutdown, 1 normal */
		{"DIS_POR",     9, 0, 0}, /* (pulled down). 0 - normal, 1 - disable POR generation on PGOOD deassertion (needed whil changing voltages) */
		{ NULL,        10, 0, 0}, /* Not connected */
		{ NULL,        11, 0, 0}, /* Not connected */
		{ NULL,        12, 0, 0}, /* Not connected */
		{ NULL,        13, 0, 0}, /* Not connected */
		{"MGTAVTTGOOD",14, 0, 0}, /*  (input) 1.2V linear regulator status (generated from 1.8V) */
		{"PGOOD18",    15, 0, 0}, /*  (input). Combines other voltages, can be monitored when DIS_POR is activated */
/* 0x25: */
		{ "FAN_CTL",   16, 1, 0}, /* Fan Control, 1 - on, 0 - off */
		{ "DAS_DSS",   17, 1, 0}, /* ? */
		{ "DEVSLP",    18, 1, 0}, /* ? */
		{ "EPGMA",     19, 1, 0}, /* ? */
		{ NULL,        20, 1, 0}, /* Not connected */
		{ NULL,        21, 1, 0}, /* Not connected */
		{ NULL,        22, 1, 0}, /* Not connected */
		{ NULL,        23, 1, 0}  /* Not connected */
};

static struct device * shutdown_dev;

static int make_group (struct device *dev, const char * name,
		ssize_t (*show)(struct device *dev, struct device_attribute *attr,
				char *buf),
		ssize_t (*store)(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count));
static ssize_t simulate_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t outputs_all_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t configs_all_show(struct device *dev, struct device_attribute *attr, char *buf);

#if 0
static ssize_t output_state_show(struct device *dev, struct device_attribute *attr, char *buf);
#endif
static ssize_t output_en_output_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_en_output_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t outputs_pgood_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t channels_en_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t channels_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t channels_dis_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t channels_dis_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_ref_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_ref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t pgood_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pbad_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t enable_por_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t enable_por_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int por_ctrl(struct device *dev, int disable_por);
static int gpio_shutdown(struct device *dev);
static int get_and_disable_por(struct device *dev, int chn_bits, int * old_dis_por);
static int reenable_por(struct device *dev);
static int wait_all_pgood(struct device *dev);
static int list_chn_bits(char * buf, int chn_bits);
static int parse_chn_bits(const char * buf);
static int get_enabled_mask(struct device *dev);
static int set_enabled_by_mask(struct device *dev, int chn_bits, int enable);
static int slew_by_mask(struct device *dev, int chn_bits);
static int get_voltage_channel(const char * name);
static int get_gpio_index_by_name(const char * name);
static int gpio_conf_by_index(struct device *dev,int gpio_index, int dir, int val);
static int get_gpio_pwr_mgx_indices(int chn, int * indices); /* chn = 0 (VP10) or 1 (VP18) */
static int get_volt_mv(struct device *dev, int chn);
static int set_volt_mv(struct device *dev, int chn, int v_mv);
static int get_enable(struct device *dev, int chn);
static int set_enable(struct device *dev, int chn, int enable);
static int get_pgood(struct device *dev, int chn);


/*
 Voltages:
  VP10 (on at power up, nominal 1.0V)
  VP18 (on at power up, nomianl 1.8V)
  VP15 (SW1, on by pinstrap, nominal 1.5V - may be reduced to 1.35 later) 
  VCC_SENS01 (SW2, nominal 1.8V, max 2.8V)
  VCC_SENS23 (SW3, nominal 1.8V, max 2.8V)
  VP5  (nominal 5.0V, not software programmed)
  VLDO18 (LDO1 - always on)
  VP33SENS0 - 3.3V to sensors J6,J7
  VP33SESN1 - 3.3V to sensors J8,J9
  MGTAVCC10 - 1.0 V, linear from VP18 (pgood controls MGTAVTT12)
  MGTAVTT12 - 1.2 V, linear from VP18 (pgood available, means both)
  LTC3589 used channels : LDO1, SW1, SW2, SW3, BB
  
  TODO: Change VCC_SENS01_R1, VCC_SENS23_R1 to 787K (now 487)
 */

/* root directory */
static DEVICE_ATTR(simulate,     SYSFS_PERMISSIONS,                     simulate_show,       simulate_store);
static DEVICE_ATTR(output_state, SYSFS_PERMISSIONS & SYSFS_READONLY,    outputs_all_show,    NULL);
static DEVICE_ATTR(configs,      SYSFS_PERMISSIONS & SYSFS_READONLY,    configs_all_show,    NULL);

static DEVICE_ATTR(channels_en,  SYSFS_PERMISSIONS,                     channels_en_show,    channels_en_store);
static DEVICE_ATTR(channels_dis, SYSFS_PERMISSIONS,                     channels_dis_show,   channels_dis_store);
static DEVICE_ATTR(power_good,   SYSFS_PERMISSIONS & SYSFS_READONLY,    pgood_show,          NULL);
static DEVICE_ATTR(power_bad,    SYSFS_PERMISSIONS & SYSFS_READONLY,    pbad_show,           NULL);
static DEVICE_ATTR(enable_por,   SYSFS_PERMISSIONS,                     enable_por_show,     enable_por_store);
static DEVICE_ATTR(power_off,    SYSFS_PERMISSIONS                 ,    NULL,                gpio_shutdown);


static struct attribute *root_dev_attrs[] = {
		&dev_attr_simulate.attr,
		&dev_attr_output_state.attr,
		&dev_attr_configs.attr,
		&dev_attr_channels_en.attr,
		&dev_attr_channels_dis.attr,
		&dev_attr_power_good.attr,
		&dev_attr_power_bad.attr,
		&dev_attr_enable_por.attr,
		&dev_attr_power_off.attr,
	    NULL
};
static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static int make_group (struct device *dev, const char * name,
		ssize_t (*show)(struct device *dev, struct device_attribute *attr,
				char *buf),
		ssize_t (*store)(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count))
{
	int retval=-1;
	int index;
	struct attribute **pattrs; /* array of pointers to attibutes */
	struct device_attribute *dev_attrs;
	struct attribute_group *attr_group;
	pattrs = devm_kzalloc(dev,(ARRAY_SIZE(voltage_reg)+1)*sizeof(pattrs[0]), GFP_KERNEL);
	if (!pattrs) return -ENOMEM;
	dev_attrs = devm_kzalloc(dev, ARRAY_SIZE(voltage_reg)*sizeof(dev_attrs[0]), GFP_KERNEL);
	if (!dev_attrs) return -ENOMEM;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group) return -ENOMEM;
	memset(dev_attrs,  0, ARRAY_SIZE(voltage_reg)*sizeof(dev_attrs[0]));
	memset(attr_group, 0, sizeof(*attr_group));
	for (index=0;index<ARRAY_SIZE(voltage_reg);index++) {
		dev_attrs[index].attr.name=voltage_reg[index].name;
		dev_attrs[index].attr.mode=SYSFS_PERMISSIONS;
		if (!show)  dev_attrs[index].attr.mode &= SYSFS_WRITEONLY;
		if (!store) dev_attrs[index].attr.mode &= SYSFS_READONLY;
		dev_attrs[index].show= show;
		dev_attrs[index].store=store;
		pattrs[index]=&(dev_attrs[index].attr);
	}
	pattrs[index]=NULL;
	attr_group->name  = name;
	attr_group->attrs =pattrs;
	dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
	if (&dev->kobj) {
		retval = sysfs_create_group(&dev->kobj, attr_group);
	}
	return retval;
}


static ssize_t simulate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
    return sprintf(buf, "%d\n",clientdata->simulate);
}
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);
    sscanf(buf, "%du", &clientdata->simulate);
    ltc3589_set_simulate(ltc3589_client, clientdata->simulate);
    return count;
}

static ssize_t outputs_all_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, pg;
	char * cp = buf;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) {
		pg=(get_enable(dev, chn)>0)?get_pgood(dev, chn):-1;
		buf+=sprintf(buf,"%s: %s %d mV%s\n",
				voltage_reg[chn].name,
				get_enable(dev, chn)?"ON":"OFF",
				get_volt_mv(dev, chn),
				(pg==1)?", power good":((pg==0)?", power is NOT good":"")
		);
	}
	return buf-cp;
}
static ssize_t configs_all_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, pg;
	char * cp = buf;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) {
		pg=get_pgood(dev, chn);
		buf+=sprintf(buf,"%s: .r1=%d .r2=%d .awe_ref=0x%04x .awe_en=0x%04x .awe_pgood=0x%04x .mask_pgood=%d\n",
				voltage_reg[chn].name,voltage_reg[chn].r1,voltage_reg[chn].r2,voltage_reg[chn].awe_ref,
				voltage_reg[chn].awe_en,voltage_reg[chn].awe_pgood,voltage_reg[chn].mask_pgood);
	}
	return buf-cp;
}


#if 0
{
		.name="vp15",
		.r1=VP15_R1,
		.r2=VP15_R2,
		.awe_ref=LTC3589_AWE_B1DTV1_REF,
		.awe_en=0,
		.awe_pgood=LTC3589_AWE_PGSTAT_SD1,
		.mask_pgood=1
},

///if (strncmp(name,voltage_reg[i].name,strlen(voltage_reg[i].name))==0)
static ssize_t output_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn,pg;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	pg=get_pgood(dev, chn);
	return sprintf(buf,"%s: %s %d mV,  %s\n",
			voltage_reg[chn].name,
			get_enable(dev, chn)?"ON":"OFF",
			get_volt_mv(dev, chn),
			(pg=1)?"power good":((pg==0)?"power is NOT good":"")
	);
}
#endif
static ssize_t output_en_output_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	return sprintf(buf,"%d\n",	get_enable(dev, chn));
}

static ssize_t output_en_output_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, enable;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	sscanf(buf, "%du", &enable);
    return count;
}

static ssize_t outputs_pgood_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	return sprintf(buf,"%d\n",	get_pgood(dev, chn));
}

static ssize_t channels_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn_bits;
	char * cp=buf;
	chn_bits=get_enabled_mask(dev);
	if (chn_bits<0) return chn_bits;
	buf+=list_chn_bits(buf, chn_bits);
	buf+=sprintf(buf,"\n");
	return buf-cp;
}
/* also slews DAC(s) if applilcable. Call after changing voltage on enabled channels */
static ssize_t channels_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn_bits,rc,old_dis_por,pre_disabled;
	chn_bits=parse_chn_bits(buf);
	pre_disabled=get_and_disable_por(dev, chn_bits, &old_dis_por);
	if (pre_disabled<0) return pre_disabled;
	rc=slew_by_mask(dev, chn_bits); /* slew if needed - before enabling, waits for slew over */
	if (rc<0) return rc;
	rc=set_enabled_by_mask(dev, chn_bits, 1);
	if (rc<0) return rc;
	if (pre_disabled && (old_dis_por==0)){
		rc=reenable_por(dev); /* will wait pgood */
		if (rc<0) return rc;
	}
	return count;
}

static ssize_t channels_dis_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn_bits;
	char * cp=buf;
	chn_bits=get_enabled_mask(dev);
	if (chn_bits<0) return chn_bits;
	chn_bits=~chn_bits;
	buf+=list_chn_bits(buf, chn_bits);
	buf+=sprintf(buf,"\n");
	return buf-cp;
}

static ssize_t channels_dis_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn_bits,rc;
	chn_bits=parse_chn_bits(buf);
	rc=set_enabled_by_mask(dev, chn_bits, 0);
	if (rc<0) return rc;
	return count;
}

static ssize_t output_ref_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	return sprintf(buf,"%d\n",get_volt_mv(dev, chn)); 
}

static ssize_t output_ref_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn, v_mv;
	int rc,old_dis_por,pre_disabled;
	chn=get_voltage_channel(attr->attr.name);
	if (chn<0) return chn;
	/* if output was enabled, and pgood negation may cause POR, disable POR (later restore) */
	if (get_enable(dev,chn)) pre_disabled=get_and_disable_por(dev, 1<<chn, &old_dis_por);
	else                     pre_disabled=0;  
	if (pre_disabled<0) return pre_disabled;
    sscanf(buf, "%du", &v_mv);
	rc=set_volt_mv(dev, chn, v_mv);
	if (rc<0) return rc;
	if (pre_disabled && (old_dis_por==0)){
		rc=reenable_por(dev); /* will wait pgood */
		if (rc<0) return rc;
	}
	return count;
}

static ssize_t pgood_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, en_bits, pgood_bits=0;
	char * cp=buf;
	en_bits= get_enabled_mask(dev);
	if (en_bits<0) return en_bits;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if (en_bits & (1 << chn)){ /* only deal with enabled channels */
		if (get_pgood(dev, chn)>0) pgood_bits |= (1<<chn);
	}
	buf+=list_chn_bits(buf, pgood_bits);
	buf+=sprintf(buf,"\n");
	return buf-cp;
}

static ssize_t pbad_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn, en_bits, pbad_bits=0;
	char * cp=buf;
	en_bits= get_enabled_mask(dev);
	if (en_bits<0) return en_bits;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if (en_bits & (1 << chn)){ /* only deal with enabled channels */
		if (get_pgood(dev, chn)==0) pbad_bits |= (1<<chn);
	}
	buf+=list_chn_bits(buf, pbad_bits);
	buf+=sprintf(buf,"\n");
	return buf-cp;
}

static ssize_t enable_por_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	int gpio_disable_por_index=get_gpio_index_by_name("DIS_POR");
	if (gpio_disable_por_index<0) return gpio_disable_por_index;
	return sprintf(buf,"%d\n",(clientdata->pwr_gpio[gpio_disable_por_index].out_val)?0:1);
}

/* When enable_por is set to 1, it first waits for PGOOD and does not enable POR on error */
static ssize_t enable_por_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int en_por,rc;
    sscanf(buf, "%du", &en_por);
    if (en_por)	rc=reenable_por(dev); /* will wait pgood, then enable POR */
    else        rc=por_ctrl(dev, 1); /* disable POR */
	if (rc<0) return rc;
	return count;
}

int gpio_shutdown(struct device *dev)
{
	int gpio_shutdown_index=get_gpio_index_by_name("NSHUTDOWN");
	if (gpio_shutdown_index<0) return gpio_shutdown_index;
	return gpio_conf_by_index(dev, gpio_shutdown_index, 1,  0);
}

int por_ctrl(struct device *dev, int disable_por)
{
	int gpio_disable_por_index=get_gpio_index_by_name("DIS_POR");
	if (gpio_disable_por_index<0) return gpio_disable_por_index;
	return gpio_conf_by_index(dev, gpio_disable_por_index, 1,  disable_por);
}
/*
 *  disable POR (if needed) before changing value or enabling one of the voltages
 * chn_bits - 1 bit per channel 
 */

static int get_and_disable_por(struct device *dev, int chn_bits, int * old_dis_por)
{
	int rc,chn;
	int gpio_disable_por_index;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	gpio_disable_por_index=get_gpio_index_by_name("DIS_POR");
	if (gpio_disable_por_index<0) return gpio_disable_por_index;
	old_dis_por[0]=clientdata->pwr_gpio[gpio_disable_por_index].out_val;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if ((chn_bits & (1<<chn)) && voltage_reg[chn].mask_pgood) break;
	if (chn>=ARRAY_SIZE(voltage_reg)) return 0; /* POR was not required to be disabled */
	rc = gpio_conf_by_index(dev, gpio_disable_por_index, 1, 1); /* out turn on "disable_por" */
	if (rc<0) return rc;
	return 1; /* pgood-based POR was disabled (could already be disabled)*/ 
}

/* call if POR was diasabled before changing voltage (value or enabling), after waiting for pgood*/ 
static int reenable_por(struct device *dev)
{
	int gpio_disable_por_index, rc;
	gpio_disable_por_index=get_gpio_index_by_name("DIS_POR");
	if (gpio_disable_por_index<0) return gpio_disable_por_index;
	if (((rc=wait_all_pgood(dev)))<0) return  rc;
	return gpio_conf_by_index(dev, gpio_disable_por_index, 1, 0); /* out turn off "disable_por" */
}

static int wait_all_pgood(struct device *dev)
{
	int ntry,chn,all_good=0;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	
	for (ntry=0;ntry<clientdata->pgoot_timeout;ntry++){
		all_good=1;
		for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if (voltage_reg[chn].awe_pgood){
			if ((get_enable(dev,chn)>0) && (get_pgood(dev,chn)!=1)){ /* enabled or always enabled */
				all_good=0;
				break;
			}
		}
		if (all_good) break; /* all enabled channels that have pgood control are good */
	}
	if (!all_good) return -EAGAIN;
	return 0;
}

static int list_chn_bits(char * buf, int chn_bits)
{
	int chn;
	char * cp=buf;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++)
		if (chn_bits & (1<<chn)) buf+=sprintf(buf," %s",voltage_reg[chn].name);
	return buf-cp;
}
/* re-use in DT  */
static int parse_chn_bits(const char * buf)
{
	int chn,chn_bits=0;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if (strstr(buf,voltage_reg[chn].name)) chn_bits |= (1<<chn);
	return chn_bits;
}

static int get_enabled_mask(struct device *dev)
{
	int chn,en_mask=0;
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++){
		if (get_enable(dev, chn)>0) en_mask|= (1<<chn);
	}
	return en_mask;
}

static int set_enabled_by_mask(struct device *dev, int chn_bits, int enable)
{
	/* consolidate writes */
	/* assuming all enable bits in LTC3589 to be in a single register (LTC3589_AWE_OVEN) */
	int chn, awe=0, oven;
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	dev_dbg(dev,"set_enabled_by_mask(dev,0x%x,%d)\n",chn_bits,enable);
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if (chn_bits & (1<<chn)){
		dev_dbg(dev,"set_enabled_by_mask() chn=%d, awe=0x%x\n",chn,voltage_reg[chn].awe_en);
		if (voltage_reg[chn].awe_en<0) {
			set_enable(dev, chn, enable);
		} else if (voltage_reg[chn].awe_en>0){
			awe |= voltage_reg[chn].awe_en;
		}
	}
	awe &= 0xff; /* just WE mask */
	if (awe){
		dev_dbg(dev,"set_enabled_by_mask(), cumulative awe=0x%x\n",awe);
	    ltc3589_client = to_i2c_client(clientdata->ltc3489_dev);
		oven=ltc3589_read_field (ltc3589_client, LTC3589_AWE_OVEN);
		if (oven<0) return oven;
		if (enable) oven |= awe;
		else oven &= ~awe;
		return ltc3589_write_field (ltc3589_client, oven, LTC3589_AWE_OVEN);
	}
	return 0;
}

static int slew_by_mask(struct device *dev, int chn_bits)
{
	/* assuming all slew bits in LTC3589 to be in a single register (LTC3589_AWE_OVEN) */
	int chn, slew=0,rc,ntry;
	u32 adwe;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	struct i2c_client *ltc3589_client;
    ltc3589_client = to_i2c_client(clientdata->ltc3489_dev);
	dev_dbg(dev,"slew_by_mask(dev,0x%x)\n",chn_bits);
	for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++) if ((chn_bits & (1<<chn)) && voltage_reg[chn].awe_slew)  {
		slew |= voltage_reg[chn].awe_slew;
		dev_dbg(dev,"slew_by_mask() chn=%d, awe_slew=0x%x, slew=0x%x\n",chn,voltage_reg[chn].awe_slew,slew);
	}
	if (slew & 0xff){ /* do nothing if no slew channel is selected */
		adwe=((slew & 0xfff00)<<8) | (slew& 0xff) | 0xff00;
		rc=ltc3589_write_adwe(ltc3589_client, adwe);
		dev_dbg(dev,"slew_by_mask():ltc3589_write_adwe(ltc3589_client, 0x%x)->%d (slew = 0x%x)\n",adwe,rc,slew);
		if (rc<0) return rc;
		/* wait slew over */
		for (ntry=0;ntry<clientdata->pgoot_timeout;ntry++){
			rc=ltc3589_read_field(ltc3589_client, LTC3589_AWE_VCCR);
			dev_dbg(dev,"slew_by_mask():ltc3589_read_field(ltc3589_client, 0x%x)->0x%x(%d)\n",LTC3589_AWE_VCCR,rc,rc);
			if (rc<0) return rc;
			if ((rc & slew) ==0 ) break;
		}
		if (ntry>=clientdata->pgoot_timeout) return -EAGAIN;
	}
	return 0;
}

/* name should either completely match, or have "_*" suffix */
static int get_voltage_channel(const char * name)
{
	int i;
	for (i=0;i<ARRAY_SIZE(voltage_reg);i++) if (strncmp(name,voltage_reg[i].name,strlen(voltage_reg[i].name))==0){
		if ((strcmp(name,voltage_reg[i].name)==0) || (name[strlen(voltage_reg[i].name)]=='_')){
			return i;
		}
	}
	return -EINVAL;
}

static int get_gpio_index_by_name(const char * name)
{
	int i;
	for (i=0;i<ARRAY_SIZE(pwr_gpio);i++){
		if (strcmp(pwr_gpio[i].label,name)==0) return i;
	}
	return -EINVAL;
}

static int gpio_conf_by_index(struct device *dev,int gpio_index, int dir, int val)
{
	int rc=0;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	if ((gpio_index<0) || (gpio_index>=ARRAY_SIZE(clientdata->pwr_gpio))) return -EINVAL;
	if ((clientdata->pwr_gpio[gpio_index].dir==dir) && ((clientdata->pwr_gpio[gpio_index].out_val==val) || (dir==0))){
		dev_dbg(dev,"GPIO#%d(index=%d) did not change: old dir=%d, new dir=%d, old val = %d, new val=%d\n",
				clientdata->pwr_gpio[gpio_index].pin,
				gpio_index,
				clientdata->pwr_gpio[gpio_index].dir,
				dir,
				clientdata->pwr_gpio[gpio_index].out_val,
				val);
		return 0;
	}
	clientdata->pwr_gpio[gpio_index].dir=dir?1:0;
	clientdata->pwr_gpio[gpio_index].out_val=val?1:0;
	if (clientdata->pwr_gpio[gpio_index].dir){
		if (!clientdata->simulate) 	rc=gpio_direction_output(clientdata->pwr_gpio[gpio_index].pin, clientdata->pwr_gpio[gpio_index].out_val);
		dev_dbg(dev,"gpio_direction_output(%d,%d)->%d\n",clientdata->pwr_gpio[gpio_index].pin, clientdata->pwr_gpio[gpio_index].out_val,rc);
	} else {
		if (!clientdata->simulate) 	rc=gpio_direction_input(clientdata->pwr_gpio[gpio_index].pin);
		dev_dbg(dev,"gpio_direction_input(%d)->%d\n",clientdata->pwr_gpio[gpio_index].pin,rc);
	}
	return rc;
}
static int get_gpio_pwr_mgx_indices(int chn, int * indices) /* chn = 0 (VP10) or 1 (VP18) */
{
	indices[0]=get_gpio_index_by_name(chn?"PWR_MG1": "PWR_MG0");
	indices[1]=get_gpio_index_by_name(chn?"PWR_MGB1":"PWR_MGB0");
	return ((indices[0]>=0) && (indices[1]>=0))?0:-EINVAL;
}

/* calculate output voltage in mV */
static int get_volt_mv(struct device *dev, int chn)
{
	int v_mv,ref,rc;
	int pwr_mg_indices[2];
	s64 num;
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	if ((chn<0) || (chn>=ARRAY_SIZE(voltage_reg))) return -EINVAL;
	if (voltage_reg[chn].r1<=0) {
		if  (voltage_reg[chn].awe_ref<0) { /* vp10, vp18*/
			rc= get_gpio_pwr_mgx_indices(-1-voltage_reg[chn].awe_ref,pwr_mg_indices); /* chn = 0 (VP10) or 1 (VP18) */
			if (rc<0) return rc;
			if (clientdata->pwr_gpio[pwr_mg_indices[0]].dir==0)       ref=0;
			else if (clientdata->pwr_gpio[pwr_mg_indices[0]].out_val) ref=1;
			else                                                      ref=-1;
			if (ref) {
				if      (clientdata->pwr_gpio[pwr_mg_indices[1]].dir==0)  ref*=15;
				else if (clientdata->pwr_gpio[pwr_mg_indices[1]].out_val) ref*=10;
				else                                                      ref*= 5;
			}
			v_mv=(voltage_reg[chn].r2*(100+ref)*2+10)/2000; 
		} else { /* vp33sens01, vp33sens23, mmtavcc10, mmtavtt12 */
			v_mv=(voltage_reg[chn].r2+5)/10;
		} 

	} else if (voltage_reg[chn].awe_ref==0){ /* VP5, vldo18 */
#if 0		
		v_mv=(REF_FIXED_TENTH_MV*(voltage_reg[chn].r1+voltage_reg[chn].r2)+ 5*voltage_reg[chn].r2)/(10*voltage_reg[chn].r2);
#endif		
		num=((u64) REF_FIXED_TENTH_MV)* (voltage_reg[chn].r1+voltage_reg[chn].r2)+ 5*voltage_reg[chn].r2;
		v_mv=(int) div64_u64(num, 10*voltage_reg[chn].r2);
		dev_dbg(dev,"chn=%d REF_FIXED_TENTH_MV=%d .r1=%d .r2=%d v_mv=%d\n",chn, REF_FIXED_TENTH_MV,voltage_reg[chn].r1,voltage_reg[chn].r2,v_mv);
	} else { /* vp15, vcc_sens01,vcc_sens23 */
	    ltc3589_client = to_i2c_client(clientdata->ltc3489_dev);
		ref=ltc3589_read_field(ltc3589_client, voltage_reg[chn].awe_ref);
		if (ref<0) return ref;
		num=(REF_VAR_0_TENTH_MV+ REF_VAR_STEP_TENTH_MV* ref);
		num=num*(voltage_reg[chn].r1+voltage_reg[chn].r2)+ 5*voltage_reg[chn].r2;
		v_mv=div64_u64(num, 10*voltage_reg[chn].r2);
		dev_dbg(dev,"chn=%d ref=%d .r1=%d .r2=%d v_mv=%d\n",chn, ref,voltage_reg[chn].r1,voltage_reg[chn].r2,v_mv);
	}
	return v_mv;
}
/* 0 - OK, <0 - error */
/* does not iclude disabling/re-enabling PoR */
static int set_volt_mv(struct device *dev, int chn, int v_mv)
{
	int rc,index,d;
	s64 num;
	int pwr_mg_indices[2];
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	if ((chn<0) || (chn>=ARRAY_SIZE(voltage_reg))) return -EINVAL;
	dev_dbg(dev,"set_volt_mv(dev,%d,%d),.r1=%d\n",chn,v_mv,voltage_reg[chn].r1);
	if (voltage_reg[chn].r1<=0) {
		if  (voltage_reg[chn].awe_ref<0) { /* vp10, vp18*/
			index=(400*v_mv+voltage_reg[chn].r2)/(2*voltage_reg[chn].r2);
			dev_dbg(dev,"chn=%d v_mv=%d index=%d .r1=%d .r2=%d\n",chn, v_mv, index,voltage_reg[chn].r1,voltage_reg[chn].r2);
			if ((index<17) || (index>23)) {
				dev_err(dev,"specified voltage for %s is not in the range %dmV to %d mV\n", voltage_reg[chn].name,
						(17*voltage_reg[chn].r2)/200,(23*voltage_reg[chn].r2)/200);
				return -EINVAL;
			}
			/* disable -> chnage -> enable (if needed) */
			rc= get_gpio_pwr_mgx_indices(-1-voltage_reg[chn].awe_ref,pwr_mg_indices); /* chn = 0 (VP10) or 1 (VP18) */
			if (rc<0) return rc;
			rc = gpio_conf_by_index(dev,pwr_mg_indices[0], 0, 0); /* disable margining */
			if (rc < 0)return rc;
			if (index !=20){
				/* set margining absolute value */
				switch (index) {
				case 17:
				case 23:
					rc = gpio_conf_by_index(dev,pwr_mg_indices[1], 0, 0); /* float: +/- 15% */
					break;
				case 18:
				case 22:
					rc = gpio_conf_by_index(dev,pwr_mg_indices[1], 1, 1); /* out 1:  +/- 10% */
					break;
				case 19:
				case 21:
					rc = gpio_conf_by_index(dev,pwr_mg_indices[1], 1, 0); /* out 0:  +/- 5% */
					break;
				}
				if (rc < 0)return rc;
				/* set margining sign */
				if (index >20) rc = gpio_conf_by_index(dev,pwr_mg_indices[0], 1, 1); /* out 1:  positive margining */
				else           rc = gpio_conf_by_index(dev,pwr_mg_indices[0], 1, 0); /* out 0:  negative margining */
				if (rc < 0)return rc;
			}
		} else { /* vp33sens01, vp33sens23, mmtavcc10, mmtavtt12 */
			return -EINVAL; /* voltage not regulated */
		} 
	} else if (voltage_reg[chn].awe_ref==0){ /* VP5, vldo18 */
		return -EINVAL; /* voltage not regulated */
	} else { /* vp15, vcc_sens01,vcc_sens23 */
	    ltc3589_client = to_i2c_client(clientdata->ltc3489_dev);
#if 0
	    index=((10*v_mv*voltage_reg[chn].r2) -(REF_VAR_0_TENTH_MV-REF_VAR_STEP_TENTH_MV/2)*(voltage_reg[chn].r1+voltage_reg[chn].r2))/
	    		((voltage_reg[chn].r1+voltage_reg[chn].r2)*REF_VAR_STEP_TENTH_MV);
	    num=(10*v_mv*voltage_reg[chn].r2) -(REF_VAR_0_TENTH_MV-REF_VAR_STEP_TENTH_MV/2);
	    num*=(voltage_reg[chn].r1+voltage_reg[chn].r2);
	    index=div64_u64(num, (voltage_reg[chn].r1+voltage_reg[chn].r2)*REF_VAR_STEP_TENTH_MV);
#endif
	    num= (10LL*v_mv*voltage_reg[chn].r2) - ((s64) (voltage_reg[chn].r1+voltage_reg[chn].r2))*REF_VAR_0_TENTH_MV;
	    d=   REF_VAR_STEP_TENTH_MV*(voltage_reg[chn].r1+voltage_reg[chn].r2);
	    index=div64_u64(num +(d>>1), d);
		dev_dbg(dev,"chn=%d v_mv=%d index=%d .r1=%d .r2=%d\n",chn, v_mv, index,voltage_reg[chn].r1,voltage_reg[chn].r2);
		dev_dbg(dev,"index=%d\n",index);
	    if ((index<0) || (index>31)){
			dev_err(dev,"chn=%d v_mv=%d index=%d .r1=%d .r2=%d\n",chn, v_mv, index,voltage_reg[chn].r1,voltage_reg[chn].r2);
			dev_err(dev,"REF_VAR_0_TENTH_MV=%d REF_VAR_STEP_TENTH_MV=%d\n",REF_VAR_0_TENTH_MV,REF_VAR_STEP_TENTH_MV);
			dev_err(dev,"specified voltage for %s is not in the range %dmV to %d mV\n", voltage_reg[chn].name,
					(int) div64_u64((((u64)(REF_VAR_0_TENTH_MV+REF_VAR_STEP_TENTH_MV* 0))*(voltage_reg[chn].r1+voltage_reg[chn].r2)+5*voltage_reg[chn].r2),
							10*voltage_reg[chn].r2),
					(int) div64_u64((((u64)(REF_VAR_0_TENTH_MV+REF_VAR_STEP_TENTH_MV*31))*(voltage_reg[chn].r1+voltage_reg[chn].r2)+5*voltage_reg[chn].r2),
							10*voltage_reg[chn].r2));
			return -EINVAL;
	    }
		dev_dbg(dev,"ltc3589_client->name= %s\n", ltc3589_client->name);
	    rc=ltc3589_write_field(ltc3589_client, index,voltage_reg[chn].awe_ref);
		if (rc<0) return rc;
	}
	return 0;
}

/* get output enable state */
static int get_enable(struct device *dev, int chn)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);
	if ((chn<0) || (chn>=ARRAY_SIZE(voltage_reg))) return -EINVAL;
	if (voltage_reg[chn].awe_en==0) {
		return 2; /* always on */ 
	} else if (voltage_reg[chn].awe_en>0){
		if (clientdata->pinstrapped_oven & voltage_reg[chn].awe_en) return 1; /* pin-strapped on bit */
		return ltc3589_read_field(ltc3589_client, voltage_reg[chn].awe_en);
	} else {
		return (clientdata->pwr_gpio[-1-voltage_reg[chn].awe_en].dir && clientdata->pwr_gpio[-1-voltage_reg[chn].awe_en].out_val)?1:0;
	}
}

/* set output enable state */
static int set_enable(struct device *dev, int chn, int enable)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);
	if ((chn<0) || (chn>=ARRAY_SIZE(voltage_reg))) return -EINVAL;
	if (voltage_reg[chn].awe_en==0) {
		return -EINVAL; /* always on, not controlled */ 
	} else if (voltage_reg[chn].awe_en>0){
		return ltc3589_write_field(ltc3589_client, enable, voltage_reg[chn].awe_en);
	} else {
		return gpio_conf_by_index(dev,-1-voltage_reg[chn].awe_en, 1, enable);
	}
}

/* get power good state */
static int get_pgood(struct device *dev, int chn)
{
	int rc;
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);
	if ((chn<0) || (chn>=ARRAY_SIZE(voltage_reg))) return -EINVAL;
	if (voltage_reg[chn].awe_pgood==0) {
		if (((rc=get_enable(dev,chn)))<0) return rc; /* 0 - disabled */
		return 2; /* no status available */ 
	} else if (voltage_reg[chn].awe_pgood>0){
		return ltc3589_read_field(ltc3589_client, voltage_reg[chn].awe_pgood);
	} else {
/*		return gpio_get_value(clientdata->pwr_gpio[-1-voltage_reg[chn].awe_pgood].pin); */
		return gpio_get_value_cansleep(clientdata->pwr_gpio[-1-voltage_reg[chn].awe_pgood].pin);
	}
}

static int elphel393_pwr_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
		if (((retval = make_group (dev, "voltages_mv", output_ref_show, output_ref_store)))<0) return retval;
		if (((retval = make_group (dev, "outputs_en", output_en_output_show, output_en_output_store)))<0) return retval;
		if (((retval = make_group (dev, "outputs_pgood", outputs_pgood_show, NULL)))<0) return retval;
	}
	return retval;
}

static void elphel393_pwr_init_of_i2caddr(struct platform_device *pdev)
{
	const __be32 * config_data;
	int len,i;
	struct device_node *node = pdev->dev.of_node;
	struct elphel393_pwr_data_t *clientdata = platform_get_drvdata(pdev);
	if (node) {
		config_data = of_get_property(node, "elphel393_pwr,i2c_chips", &len);
		if (config_data){
			len /= sizeof(*config_data);
			dev_dbg(&pdev->dev,"Found %d items in 'elphel393_pwr,i2c_chips' in the Device Tree\n",len);
			if (len!= ARRAY_SIZE(clientdata->chip_i2c_addr)){
				dev_err(&pdev->dev,"Got %d items in 'elphel393_pwr,i2c_chips', expected %d\n",len,ARRAY_SIZE(clientdata->chip_i2c_addr));
				return;
			}
			for (i=0;i<len;i++)	clientdata->chip_i2c_addr[i]=be32_to_cpup(&config_data[i]);
		}
	}
}

static void elphel393_pwr_init_of(struct platform_device *pdev)
{
	const __be32 * config_data;
    const   char * config_string;
	char str[40];
	int len,chn,pre_disabled,old_dis_por,rc,chn_bits;
	struct device_node *node = pdev->dev.of_node;
	struct elphel393_pwr_data_t *clientdata = platform_get_drvdata(pdev);
	struct i2c_client *ltc3589_client= to_i2c_client(clientdata->ltc3489_dev);

	if (node) {
		/* find resistor values */
		for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++){
			sprintf(str,"elphel393_pwr,%s.r1",voltage_reg[chn].name);
			config_data = of_get_property(node, str, &len);
			if (config_data && (len>0)){
				dev_dbg(&pdev->dev,"Found %s=<%d>\n",str,be32_to_cpup(&config_data[0]));
				voltage_reg[chn].r1=be32_to_cpup(&config_data[0]);
			}			
			sprintf(str,"elphel393_pwr,%s.r2",voltage_reg[chn].name);
			config_data = of_get_property(node, str, &len);
			if (config_data && (len>0)){
				dev_dbg(&pdev->dev,"Found %s=<%d>\n",str,be32_to_cpup(&config_data[0]));
				voltage_reg[chn].r2=be32_to_cpup(&config_data[0]);
			}			
		}
		/* which channels are enabled by pin-strapping */
		config_data = of_get_property(node, "elphel393_pwr,pinstrapped_oven", &len);
		if (config_data && (len>0)){
			dev_dbg(&pdev->dev,"Found elphel393_pwr,pinstrapped_oven=<%d>\n",be32_to_cpup(&config_data[0]));
			clientdata->pinstrapped_oven=be32_to_cpup(&config_data[0]);
		}
		
		/* debug mode - simulate only, no actual power supply control */
		config_data = of_get_property(node, "elphel393_pwr,simulate", &len);
		if (config_data && (len>0)){
			dev_dbg(&pdev->dev,"Found elphel393_pwr,simulate=<%d>\n",be32_to_cpup(&config_data[0]));
			clientdata->simulate=config_data[0]?1:0;
		    ltc3589_set_simulate(ltc3589_client, clientdata->simulate);
		}

		/* disable output voltages (not likely to be needed - maybe for warm reboot) */
		config_string = of_get_property(node, "elphel393_pwr,channels_disable", &len);
		if (config_string){
			dev_dbg(&pdev->dev,"Found elphel393_pwr,channels_disable=\"%s\"\n",config_string);
			chn_bits=parse_chn_bits(config_string);
			rc=set_enabled_by_mask(&pdev->dev, chn_bits, 0);
			if (rc<0) return;
		}
		/* set output voltages (target voltages, in mV) */
		for (chn=0;chn<ARRAY_SIZE(voltage_reg);chn++){
			sprintf(str,"elphel393_pwr,%s_mv",voltage_reg[chn].name);
			config_data = of_get_property(node, str, &len);
			if (config_data && (len>0)){
				dev_dbg(&pdev->dev,"Found %s=<%d>\n",str,be32_to_cpup(&config_data[0]));
				if (get_enable(&pdev->dev,chn)) pre_disabled=get_and_disable_por(&pdev->dev, 1<<chn, &old_dis_por);
				else                            pre_disabled=0;  
				if (pre_disabled<0) return;
				dev_dbg(&pdev->dev,"pre_disabled=%d\n",pre_disabled);
				rc=set_volt_mv(&pdev->dev, chn,be32_to_cpup(&config_data[0]));
				dev_dbg(&pdev->dev,"set_volt_mv()->%d\n",rc);
				if (rc<0) return;
				if (pre_disabled && (old_dis_por==0)){
					rc=reenable_por(&pdev->dev); /* will wait pgood */
					if (rc<0){
						dev_err(&pdev->dev,"Timeout during wait for power good after chnging voltage for %s before re-enabling POR on power loss\n",\
								voltage_reg[chn].name);
						return;
					}
				}
			}			
		}
		/* enable output voltages */
		config_string = of_get_property(node, "elphel393_pwr,channels_enable", &len);
		if (config_string){
			dev_dbg(&pdev->dev,"Found elphel393_pwr,channels_enable=\"%s\"\n",config_string);
			chn_bits=parse_chn_bits(config_string);
			pre_disabled=get_and_disable_por(&pdev->dev, chn_bits, &old_dis_por);
			if (pre_disabled<0) return;
			rc=slew_by_mask(&pdev->dev, chn_bits); /* slew if needed - before enabling, waits for slew over */
			if (rc<0) {
				dev_err(&pdev->dev,"Timeout during wait for slew over\n");
				return;
			}
			rc=set_enabled_by_mask(&pdev->dev, chn_bits, 1);
			if (rc<0) return;
			if (pre_disabled && (old_dis_por==0)){
				rc=reenable_por(&pdev->dev); /* will wait pgood */
				if (rc<0) {
					dev_err(&pdev->dev,"Timeout during wait for power good before re-enabling POR on power loss\n");
					return;
				}
			}
		}
	}	
	dev_info(&pdev->dev,"elphel393_pwr configuration done\n");
}

static int device_by_i2c_addr_match(struct device *dev, void *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int *addr = (int *)data;
	dev_dbg(dev,"addr_given=0x%02x, addr found=0x%02x\n",addr[0],(int) client->addr);
	return i2c_verify_client(dev) && (client->addr==addr[0]);
}

static struct device * find_device_by_i2c_addr(int address)
{
	return bus_find_device(&i2c_bus_type, NULL, &address, device_by_i2c_addr_match);
}

static int i2c_addr_gpiochip_match(struct gpio_chip *chip, void *data)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	int *addr = (int *)data;
	dev_dbg(chip->dev,"addr_given=0x%02x, addr found=0x%02x\n",addr[0],(int) client->addr);
	return i2c_verify_client(chip->dev) && (client->addr==addr[0]);
}

static void shutdown(void){
	gpio_shutdown(shutdown_dev);
}

static int elphel393_pwr_probe(struct platform_device *pdev)
{
	struct gpio_chip *chip;
//	struct device * ltc3489_dev;
	int i,rc;
	int base[3];
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata = NULL;

	shutdown_dev = &pdev->dev;

	dev_info(&pdev->dev,"Probing elphel393-pwr\n");
	clientdata = devm_kzalloc(&pdev->dev, sizeof(*clientdata), GFP_KERNEL);
	clientdata->pgoot_timeout=DEAFULT_TIMEOUT;
	clientdata->pinstrapped_oven=PINSTRAPPED_OVEN;

	clientdata->chip_i2c_addr[0]=GPIO_CHIP1_ADDR;
	clientdata->chip_i2c_addr[1]=GPIO_CHIP2_ADDR;
	clientdata->chip_i2c_addr[2]=GPIO_10389_U4_ADDR;
	clientdata->chip_i2c_addr[3]=LTC3589_ADDR;
	platform_set_drvdata(pdev, clientdata);

	elphel393_pwr_sysfs_register(pdev);
//	elphel393_pwr_init_of(pdev);
	elphel393_pwr_init_of_i2caddr(pdev);
	mutex_init(&clientdata->lock);
	
	/* locate GPIO chips by i2c address */
    for (i=0;i<3;i++){
    	chip = gpiochip_find(&clientdata->chip_i2c_addr[i], i2c_addr_gpiochip_match);
    	base[i]=chip->base;
    	dev_dbg(&pdev->dev,"Found gpio_chip with i2c_addr=0x%02x, label=%s, base=0x%x\n",clientdata->chip_i2c_addr[i],chip->label,base[i]);
    }
    for (i=0;i<ARRAY_SIZE(pwr_gpio);i++) if (pwr_gpio[i].label){
    	clientdata->pwr_gpio[i].label=pwr_gpio[i].label;
    	clientdata->pwr_gpio[i].pin=base[i>>3]+(i & 7);
    	if (i<16) {
    		clientdata->pwr_gpio[i].dir=0; /* input */
    	}else{
    		clientdata->pwr_gpio[i].dir=1; /* output */
    	}
    	clientdata->pwr_gpio[i].out_val=0; 
    	rc=gpio_request(clientdata->pwr_gpio[i].pin, clientdata->pwr_gpio[i].label);
    	if (rc<0){
    		dev_err(&pdev->dev," Failed to get GPIO[%d] with label %s\n",clientdata->pwr_gpio[i].pin,clientdata->pwr_gpio[i].label);
    		return rc;
    	} else {
    		dev_dbg(&pdev->dev,"Confirmed request GPIO[%d] with label %s\n",clientdata->pwr_gpio[i].pin,clientdata->pwr_gpio[i].label);
    	}
    }

    /* find ltc3589 */
    clientdata->ltc3489_dev=find_device_by_i2c_addr(LTC3589_ADDR);
    if (!clientdata->ltc3489_dev){
		dev_err(&pdev->dev," Failed to find LTC3489 with i2c address 0x%02x\n",LTC3589_ADDR);
		return -EIO;
    }
    ltc3589_client = to_i2c_client(clientdata->ltc3489_dev);
   
	dev_dbg(&pdev->dev,"Located %s with i2c address 0x%02x\n",ltc3589_client->name,LTC3589_ADDR);
	dev_dbg(&pdev->dev,"LTC3589 status= 0x%02x\n",ltc3589_read_field(ltc3589_client, LTC3589_AWE_PGSTAT));
	elphel393_pwr_init_of(pdev);
	
	pm_power_off = shutdown;

	return 0;
}	

static int elphel393_pwr_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,"Removing elphel393-pwr");
	return 0;
}

static struct of_device_id elphel393_pwr_of_match[] = {
	{ .compatible = "elphel,elphel393-pwr-1.00", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, elphel393_pwr_of_match);

static struct platform_driver elphel393_pwr = {
	.probe   = elphel393_pwr_probe,
	.remove  = elphel393_pwr_remove,
	.driver  = {
		.name  = "elphel393-pwr",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_pwr_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_pwr);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("Elphel 10393 power supply control");
MODULE_LICENSE("GPL");
