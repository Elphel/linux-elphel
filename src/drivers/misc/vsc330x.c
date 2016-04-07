/*!***************************************************************************
 *! FILE NAME  : vsc330x.c
 *! DESCRIPTION: control of the VSC3304 4x4 crosspoint switch
 *! Copyright (C) 2013 Elphel, Inc.
 *! -----------------------------------------------------------------------------**
 *!
 *!  This program is free software: you can redistribute it and/or modify
 *!  it under the terms of the GNU General Public License as published by
 *!  the Free Software Foundation, either version 3 of the License, or
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

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of.h>

#define DRV_VERSION "1.0"

/* TODO: Descriptions from vsc3312 - check differences */
#define I2C_PAGE_CONNECTION        0x00	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding output (0..0xf) source
					   (input number) bit 4 (+0x10) - turn output off, bits 3:0 - source */
#define I2C_PAGE_INPUT_ISE         0x10	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding input (0..0xf)
					   ISE (equalization): Bits 5:4 ISE short: 0 - off, 1 - minimal, 2 - moderate, 3 - maximal;
					   bits 3:2 ISE medium, bits 1:0 ISE Long time constant */
#define I2C_PAGE_INPUT_STATE       0x11	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding input (0..0xf) enable,
					   polarity and termination (default 6)
					   Bit 2 (+4) Terminate to VDD ( 0 - connect,  1 - do not connect) - dedicated (0..7) inputs only
					   Bit 1 (+2) Input power (0 - on, 1 - off)
					   Bit 0 (+1) Invert signal at input  */
#define I2C_INPUT_STATE_DATA       0x04	/* terminated,enabled, not inverted */
#define I2C_PAGE_INPUT_LOS         0x12	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding input (0..0xf)
					   LOS (loss of signal) threshold
					   Bits 2:0 - level in mV for dedicated(bidirectional) inputs: 0,1,6,7 - unused, 2 - 150(170),
					   3 - 200(230), 4 - 250(280), 5 - 300(330) */
#define I2C_PAGE_OUTPUT_PRE_LONG   0x20	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding output (0..0xf)
					   long time constant pre-emphasis
					   Bits 6:3 Pre-Emphasis level (0x0 - off, 0x1 - min, 0xf - max - 0..6dB), bits 2:0 - Pre-emphasis
					   decay (0x0 - fastest, 0x7 - slowest) in 500..1500 ps range */
#define I2C_PAGE_OUTPUT_PRE_SHORT  0x21	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding output (0..0xf)
					   short time constant pre-emphasis
					   Bits 6:3 Pre-Emphasis level (0x0 - off, 0x1 - min, 0xf - max - 0..6dB),
					   bits 2:0 - Pre-emphasis decay (0x0 - fastest, 0x7 - slowest) in 30..500 ps range */
#define I2C_PAGE_OUTPUT_LEVEL      0x22	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding output (0..0xf)
					   short time constant pre-emphasis
					   Bits 3:0 - peak-to-peak 0,1,0xe,0xf - unused, 0x2-405mV,0x3-425V,0x4-455mV,0x5-485mV,0x6-520mV,
					   0x7-555mV,0x8-605mV,0x9-655mV,0xa-720mV,0xb-790mV,0xc-890mV,0xd-990mV (+3.3VDC required)
					   bit 4 (+0x10) - for 8-15 used as inputs only: terminate inputs 8..15 to VDDIO-0.7V */
#define I2C_PAGE_OUTPUT_STATE      0x23	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control corresponding output (0..0xf)
					   OOB signaling and output polarity
					   bits 4:1 - operation mode: 0xa  - inverted, 0x5 - normal, 0x0 - suppressed
					   bit 0 - OOB control:     1 - enable LOS forwarding, 0 - ignore LOS */
#define I2C_PAGE_CHANNEL_STATUS    0xf0	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf monitor corresponding input (0..0xf) LOS status
					   bit 0 - LOS status: 1 - LOS detected (loss of signal), 0 - signal present (input has to be enabled,
					   otherwise 0 is read)when reading from address 0x10 of this page:
					   bit 0 - value of STAT0
					   bit 1 - value of STAT1 */
#define I2C_PAGE_STATUS0_CONFIGURE 0x80	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control selected input LOS to be OR-ed
					   to STAT0 output pin (and bit)
					   bit 0 : 1 - OR this input channel LOS status to STAT0 */
#define I2C_PAGE_STATUS1_CONFIGURE 0x81	/* When written to I2C_CURRENT_PAGE, makes registers 0..0xf control selected input LOS to be OR-ed
					   to STAT1 output pin (and bit)
					   bit 0 : 1 - OR this input channel LOS status to STAT1 */
#define I2C_PAGE_STATUS_READ       0xf0	/* Read only from reg=0x10: bit 0 - status0, bit 1 - status 1 */

#define I2C_GLOBAL_CONNECTION      0x50	/* Bit 4 (+0x10) - disable all outputs, bits 3:0 - input number to connect to all outputs */
#define I2C_GLOBAL_INPUT_ISE       0x51	/* Bits 5:4 ISE short: 0 - off, 1 - minimal, 2 - moderate, 3 - maximal; bits 3:2 ISE medium,
					   bits 1:0 ISE Long time constant */
#define I2C_GLOBAL_INPUT_STATE     0x52	/* Bit 2 (+4) - terminate input to VDD (0..7 only) 0-connect, 1 Normal;
					   Bit 1 (+2) Input power off (0 - On, 1 - Off) bit0 (+1): Input polarity: 1 - inverted, 0 - normal  */
#define I2C_GLOBAL_INPUT_LOS       0x53	/* Bits 2:0 - level in mV for dedicated(bidirectional) inputs: 0,1,6,7 - unused, 2 - 150(170),
					   3 - 200(230), 4 - 250(280), 5 - 300(330) */
#define I2C_GLOBAL_OUTPUT_PRE_LONG 0x54	/* Bits 6:3 Pre-Emphasis level (0x0 - off, 0x1 - min, 0xf - max - 0..6dB),
					   bits 2:0 - Pre-emphasis decay (0x0 - fastest, 0x7 - slowest) in 500..1500 ps range */
#define I2C_GLOBAL_OUTPUT_PRE_SHORT 0x55 /* Bits 6:3 Pre-Emphasis level (0x0 - off, 0x1 - min, 0xf - max - 0..6dB),
					   bits 2:0 - Pre-emphasis decay (0x0 - fastest, 0x7 - slowest) in 30..500 ps range */
#define I2C_GLOBAL_OUTPUT_LEVEL    0x56	/* Bits 3:0 - peak-to-peak 0,1,0xe,0xf - unused,0x2-405mV,0x3-425V,0x4-455mV,0x5-485mV,
					   0x6-520mV,0x7-555mV,0x8-605mV,0x9-655mV,0xa-720mV,0xb-790mV,0xc-890mV,0xd-990mV (+3.3VDC required)
					   bit 4 (+0x10) terminate inputs 8..15 to VDDIO-0.7V */
#define I2C_GLOBAL_OUTPUT_STATE    0x57	/* +1 (bit 0) - LOS, +0x15 - inverted, 0xa0 - normal, +0 - "Common mode" ? */
#define I2C_GLOBAL_OUTPUT_STATE_DATA 0x0b /* No inversion, enable OOB forwarding on all channels */
#define I2C_GLOBAL_STATUS0         0x58	/* Bit 0 - selected for Status0 chanel LOS on from all channels */
#define I2C_GLOBAL_STATUS1         0x59	/* Bit 0 - selected for Status1 chanel LOS on from all channels */
#define I2C_CORE_CONFIGURATION     0x75
#define I2C_CORE_CONFIGURATION_DATA  0x18	/* default 0x18 - 0x10 - leftEn, 0x8 - rightEn, 0x4 - DNU, 0x2 - BufferForceOn, 0x1 - Config polarity */
#define I2C_CORE_CONFIGURATION_DATAF 0x19	/* default with inverted Config polarity (freeze update) */
#define I2C_SLAVE_ADDRESS          0x78	/* programmed only, not hardwired */
#define I2C_INTERFACE_MODE         0x79
#define I2C_INTERFACE_MODE_DATA    0x02	/* i2c (1 - 4-wire)  */
#define I2C_SOFTWARE_RESET         0x7a
//#define I2C_SOFTWARE_RESET_DATA    0x10 /* to reset, 0 - normal  */ not used - but number 4 is used instead
#define I2C_CURRENT_PAGE           0x7f
#define PORT_PEFIX                 "port_"
#define ALL_PORTS                  "all"
#define MAX_PORTS                   16
#define I2C_PAGE_GLOBAL             -1 /* does not use paging access */

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

static const char port_names[][8]={
		PORT_PEFIX "00",PORT_PEFIX "01",PORT_PEFIX "02",PORT_PEFIX "03",PORT_PEFIX "04",
		PORT_PEFIX "05",PORT_PEFIX "06",PORT_PEFIX "07",PORT_PEFIX "08",PORT_PEFIX "09",
		PORT_PEFIX "10",PORT_PEFIX "11",PORT_PEFIX "12",PORT_PEFIX "13",PORT_PEFIX "14",
		PORT_PEFIX "15",PORT_PEFIX "16",PORT_PEFIX "17",PORT_PEFIX "18",PORT_PEFIX "19",
		PORT_PEFIX "20",PORT_PEFIX "21",PORT_PEFIX "22",PORT_PEFIX "23",PORT_PEFIX "24",
		PORT_PEFIX "25",PORT_PEFIX "26",PORT_PEFIX "27",PORT_PEFIX "28",PORT_PEFIX "29",
		PORT_PEFIX "30",PORT_PEFIX "31"};

static const struct i2c_device_id vsc330x_id[] = {
	{ "vsc3304", 0 },
	{ "vsc3308", 1 },
	{ "vsc3312", 2 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, vsc3304_id);

struct vsc330x_data_t {
	int  address_mode_data; // vsc3304 needs 6 to be written, otherwise weird modification  of bit 3
	int  last_page;
	u32 in_ports;
	u32 out_ports;
};

static const struct vsc330x_data_t vsc330x_data[] = {
		{.address_mode_data=6,
				.in_ports=0xff00, .out_ports=0xff00}, /* 3304 - all ports I/O shared*/
		{.address_mode_data=-1, // No data, unknown if it is needed for 3308
				.in_ports=0x00ff, .out_ports=0x00ff}, /* 3308 - no ports I/O shared */
		{.address_mode_data=-1, // No data, unknown if it is needed for 3308
				.in_ports=0xffff, .out_ports=0xffff} /* 3312 - some ports I/O shared*/
};
static int init_device(struct i2c_client *client);
static int read_reg(struct i2c_client *client, u8 reg);
static int write_reg(struct i2c_client *client, u8 reg, u8 val);
static int read_field(struct i2c_client *client, u8 reg, int ls_bit_num, int width);
static int read_page_field(struct i2c_client *client, int page, u8 reg, int ls_bit_num, int width);
static int write_field(struct i2c_client *client, u8 reg, u8 val, int ls_bit_num, int width);
static int write_page_field(struct i2c_client *client, int page, u8 reg, u8 val, int ls_bit_num, int width);
static ssize_t field_show(struct device *dev, struct device_attribute *attr, char *buf,
		int page, int ls_bit_num, int width);
static ssize_t field_show_reg(struct device *dev, char *buf, int page, int reg, int ls_bit_num, int width);
static ssize_t field_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count,
		int page, int ls_bit_num, int width);
static ssize_t field_store_reg(struct device *dev, const char *buf, size_t count,
		int page, int reg, int ls_bit_num, int width);
static ssize_t connection_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t connection_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_ISE_short_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_ISE_short_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_ISE_medium_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_ISE_medium_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_ISE_long_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_ISE_long_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_state_off_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_state_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_state_invert_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_state_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_LOS_threshold_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_LOS_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_PRE_long_level_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_PRE_long_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_PRE_long_decay_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_PRE_long_decay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_PRE_short_level_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_PRE_short_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_PRE_short_decay_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_PRE_short_decay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
/* TODO - fix for vsc3312*/
static ssize_t input_terminate_low_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_terminate_low_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t input_terminate_high_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t input_terminate_high_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_level_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t output_mode_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t output_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t forward_OOB_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t forward_OOB_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t status_0_on_LOS_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t status_0_on_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t status_1_on_LOS_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t status_1_on_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t status_show (struct device *dev, struct device_attribute *attr, char *buf);
/* global */
static ssize_t global_connection_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_connection_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_ISE_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_ISE_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_input_state_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_input_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_input_LOS_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_input_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_output_PRE_long_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_output_PRE_long_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_output_PRE_short_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_output_PRE_short_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_output_level_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_output_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_output_state_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_output_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_status_0_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_status_0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t global_status_1_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t global_status_1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t core_config_word_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t core_config_word_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t core_left_bias_en_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t core_left_bias_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t core_right_bias_en_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t core_right_bias_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t core_buffer_on_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t core_buffer_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t core_config_pin_invert_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t core_config_pin_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t soft_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t address_range_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t address_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t status_combo_show (struct device *dev, struct device_attribute *attr, char *buf);

/* Global registers - writes applied to all port registers. No sense to read (so write only), but functions preserved
 * Placed in "globals" directory */
static DEVICE_ATTR(connection,        SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_connection_show, global_connection_store);
static DEVICE_ATTR(ISE,               SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_ISE_show, global_ISE_store);
static DEVICE_ATTR(input_state,       SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_input_state_show, global_input_state_store);
static DEVICE_ATTR(input_LOS,         SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_input_LOS_show, global_input_LOS_store);
static DEVICE_ATTR(output_PRE_long,   SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_output_PRE_long_show,  global_output_PRE_long_store);
static DEVICE_ATTR(output_PRE_short,  SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_output_PRE_short_show, global_output_PRE_short_store);
static DEVICE_ATTR(output_level,      SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_output_level_show, global_output_level_store);
static DEVICE_ATTR(output_state,      SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_output_state_show, global_output_state_store);
static DEVICE_ATTR(status_0_on_LOS,   SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_status_0_show, global_status_0_store);
static DEVICE_ATTR(status_1_on_LOS,   SYSFS_PERMISSIONS & SYSFS_WRITEONLY, global_status_1_show, global_status_1_store);

/* control/status registers , placed in "control" directory */
static DEVICE_ATTR(core_word,         SYSFS_PERMISSIONS,                   core_config_word_show, core_config_word_store);
static DEVICE_ATTR(core_left_bias_en, SYSFS_PERMISSIONS,                   core_left_bias_en_show, core_left_bias_en_store);
static DEVICE_ATTR(core_right_bias_en,SYSFS_PERMISSIONS,                   core_right_bias_en_show, core_right_bias_en_store);
static DEVICE_ATTR(core_buffer_on,    SYSFS_PERMISSIONS,                   core_buffer_on_show, core_buffer_on_store);
static DEVICE_ATTR(core_config_pin_invert,SYSFS_PERMISSIONS,               core_config_pin_invert_show, core_config_pin_invert_store);
static DEVICE_ATTR(soft_reset,        SYSFS_PERMISSIONS & SYSFS_WRITEONLY, NULL,                        soft_reset_store);
static DEVICE_ATTR(address_range,     SYSFS_PERMISSIONS,                   address_range_show,          address_range_store);
static DEVICE_ATTR(status,            SYSFS_PERMISSIONS & SYSFS_READONLY,  status_combo_show, NULL);

static struct attribute *globals_dev_attrs[] = {
	&dev_attr_connection.attr,
	&dev_attr_ISE.attr,
	&dev_attr_input_state.attr,
	&dev_attr_input_LOS.attr,
	&dev_attr_output_PRE_long.attr,
	&dev_attr_output_PRE_short.attr,
	&dev_attr_output_level.attr,
	&dev_attr_output_state.attr,
	&dev_attr_status_0_on_LOS.attr,
	&dev_attr_status_1_on_LOS.attr,
	NULL
};

static const struct attribute_group dev_attr_globals_group = {
	.attrs = globals_dev_attrs,
	.name  = "globals",
};

static struct attribute *control_dev_attrs[] = {
	&dev_attr_core_word.attr,
	&dev_attr_core_left_bias_en.attr,
	&dev_attr_core_right_bias_en.attr,
	&dev_attr_core_buffer_on.attr,
	&dev_attr_core_config_pin_invert.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_address_range.attr,
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group dev_attr_control_group = {
	.attrs = control_dev_attrs,
	.name  = "control",
};


/* seems we need to INITIALIZE all structures to zero - examples use static */
static int make_group (struct device *dev, const char * name, int port_mask, mode_t mode,
		ssize_t (*show)(struct device *dev, struct device_attribute *attr,
				char *buf),
		ssize_t (*store)(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count))
{
	int retval=-1;
	int port,index=0,num_regs;
	struct attribute **pattrs; /* array of pointers to attributes */
	struct device_attribute *dev_attrs;
	struct attribute_group *attr_group;
	for (port=0,num_regs=1;port<MAX_PORTS;port++) if (port_mask & (1<<port)) num_regs++; /* 1 extra - used for all ports */

	pattrs = devm_kzalloc(dev, (num_regs+1)*sizeof(pattrs[0]), GFP_KERNEL);
	if (!pattrs) return -ENOMEM;
	dev_attrs = devm_kzalloc(dev, num_regs*sizeof(dev_attrs[0]), GFP_KERNEL);
	if (!dev_attrs) return -ENOMEM;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group) return -ENOMEM;
	memset(dev_attrs,  0, num_regs*sizeof(dev_attrs[0]));
	memset(attr_group, 0, sizeof(*attr_group));
	for (port=0,index=0;port<MAX_PORTS;port++) if (port_mask & (1<<port)){
		dev_attrs[index].attr.name=port_names[port];
		dev_attrs[index].attr.mode=mode;
		dev_attrs[index].show= show;
		dev_attrs[index].store=store;
 		pattrs[index]=&(dev_attrs[index].attr);
 		index++;
	}
	/*  add all ports */
	dev_attrs[index].attr.name=ALL_PORTS;
	dev_attrs[index].attr.mode=mode; /* & 0222; */ /* write only */
	dev_attrs[index].show= show;
	dev_attrs[index].store=store;
	pattrs[index]=&(dev_attrs[index].attr);
	index++;
	pattrs[index]=NULL;

	attr_group->name  = name;
	attr_group->attrs =pattrs;
	dev_dbg(dev,"name=%s, &dev->kobj=0x%08x\n",attr_group->name, (int) (&dev->kobj));
	index=0;
	while ((*attr_group).attrs[index]){
		dev_dbg(dev,"attr=%s\n",attr_group->attrs[index]->name);
		index++;
	}
    if (&dev->kobj) {
    	retval = sysfs_create_group(&dev->kobj, attr_group);
    }
	return retval;
}

static int vsc330x_sysfs_register(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct vsc330x_data_t *clientdata = i2c_get_clientdata(client);
    int ports=clientdata->out_ports;
    int retval=0;
    if (&dev->kobj) {
    	if ((retval=make_group (dev, "connections",           ports, SYSFS_PERMISSIONS, connection_show, connection_store))) return retval;
    	if ((retval=make_group (dev, "input_ISE_short",       ports, SYSFS_PERMISSIONS, input_ISE_short_show, input_ISE_short_store))) return retval;
    	if ((retval=make_group (dev, "input_ISE_medium",      ports, SYSFS_PERMISSIONS, input_ISE_medium_show, input_ISE_medium_store))) return retval;
    	if ((retval=make_group (dev, "input_ISE_long",        ports, SYSFS_PERMISSIONS, input_ISE_long_show, input_ISE_long_store))) return retval;
    	if ((retval=make_group (dev, "input_state_off",       ports, SYSFS_PERMISSIONS, input_state_off_show, input_state_off_store))) return retval;
    	if ((retval=make_group (dev, "input_state_invert",    ports, SYSFS_PERMISSIONS, input_state_invert_show, input_state_invert_store))) return retval;
    	if ((retval=make_group (dev, "input_LOS_threshold",   ports, SYSFS_PERMISSIONS, input_LOS_threshold_show, input_LOS_threshold_store))) return retval;
    	if ((retval=make_group (dev, "output_PRE_long_level", ports, SYSFS_PERMISSIONS, output_PRE_long_level_show, output_PRE_long_level_store))) return retval;
    	if ((retval=make_group (dev, "output_PRE_long_decay", ports, SYSFS_PERMISSIONS, output_PRE_long_decay_show, output_PRE_long_decay_store))) return retval;
    	if ((retval=make_group (dev, "output_PRE_short_level",ports, SYSFS_PERMISSIONS, output_PRE_short_level_show, output_PRE_short_level_store))) return retval;
    	if ((retval=make_group (dev, "output_PRE_short_decay",ports, SYSFS_PERMISSIONS, output_PRE_short_decay_show, output_PRE_short_decay_store))) return retval;
    	if ((retval=make_group (dev, "input_terminate_low",   ports, SYSFS_PERMISSIONS, input_terminate_low_show, input_terminate_low_store))) return retval;
    	if ((retval=make_group (dev, "input_terminate_high",  ports, SYSFS_PERMISSIONS, input_terminate_high_show, input_terminate_high_store))) return retval;
    	if ((retval=make_group (dev, "output_level",          ports, SYSFS_PERMISSIONS, output_level_show, output_level_store))) return retval;
    	if ((retval=make_group (dev, "output_mode",           ports, SYSFS_PERMISSIONS, output_mode_show, output_mode_store))) return retval;
    	if ((retval=make_group (dev, "forward_OOB",           ports, SYSFS_PERMISSIONS, forward_OOB_show, forward_OOB_store))) return retval;
    	if ((retval=make_group (dev, "status_0_on_LOS",       ports, SYSFS_PERMISSIONS, status_0_on_LOS_show, status_0_on_LOS_store))) return retval;
    	if ((retval=make_group (dev, "status_1_on_LOS",       ports, SYSFS_PERMISSIONS, status_1_on_LOS_show, status_1_on_LOS_store))) return retval;
    	if ((retval=make_group (dev, "status",                ports, SYSFS_PERMISSIONS & SYSFS_READONLY, status_show, NULL))) return retval;

    	if ((retval = sysfs_create_group(&dev->kobj, &dev_attr_globals_group))) return retval;
    	if ((retval = sysfs_create_group(&dev->kobj, &dev_attr_control_group))) return retval;
    }
    return retval;
}


static ssize_t field_show(struct device *dev, struct device_attribute *attr, char *buf,
		int page, int ls_bit_num, int width)
{
	int reg, port_mask, rc=0, len=0, count=PAGE_SIZE;
	struct i2c_client *client = to_i2c_client(dev);
   /* do for all ports - not used. TODO: we can try to output all of them in a row, but not sure if count will permit*/
	if (strcmp(attr->attr.name,ALL_PORTS) == 0) {
		port_mask=((struct vsc330x_data_t *) i2c_get_clientdata(to_i2c_client(dev)))->out_ports;
		for (reg=0;reg<MAX_PORTS;reg++) if ((port_mask & (1<<reg)) && (count>5)) {
			dev_dbg(dev, "name='%s' reg=0x%x, page=0x%x, ls_bit_num=0x%x, width=0x%x\n",
					attr->attr.name, reg, page, ls_bit_num,  width);
			rc = read_page_field(client, page, reg, ls_bit_num, width);
//		    rc=field_show_reg(dev, buf, page, reg, ls_bit_num, width);
		    if (rc<0) return rc;
		    rc=sprintf(buf, "%d ", rc);
		    buf+=rc;
		    len+=rc;
		    count-=rc;

		}
		if (len>0) {
			len--;
			count++;
			buf--;
		}
	    rc=sprintf(buf, "\n");
	    buf+=rc;
	    len+=rc;
	    count-=rc;
	    return len;
	}
	/* process single port */
	sscanf(attr->attr.name+strlen(PORT_PEFIX), "%du", &reg);
	dev_dbg(dev, "name='%s' name+%d='%s' reg=0x%x, page=0x%x, ls_bit_num=0x%x, width=0x%x\n",
			attr->attr.name, strlen(PORT_PEFIX), ( attr->attr.name+strlen(PORT_PEFIX)),  reg, page, ls_bit_num,  width);
    return field_show_reg(dev, buf, page, reg, ls_bit_num, width);
}

static ssize_t field_show_reg(struct device *dev, char *buf, int page, int reg, int ls_bit_num, int width)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data=read_page_field(client, page, reg, ls_bit_num, width);
    return sprintf(buf, "%d\n", data);
}

static ssize_t field_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count,
		int page, int ls_bit_num, int width)
{
	int reg, port_mask, rc;
   /* do for all ports */
	if (strcmp(attr->attr.name,ALL_PORTS) == 0) {
		port_mask=((struct vsc330x_data_t *) i2c_get_clientdata(to_i2c_client(dev)))->out_ports;
		for (reg=0;reg<MAX_PORTS;reg++) if (port_mask & (1<<reg)) {
			dev_dbg(dev, "name='%s' reg=0x%x, page=0x%x, ls_bit_num=0x%x, width=0x%x\n",
					attr->attr.name, reg, page, ls_bit_num,  width);
			rc= field_store_reg(dev, buf, count, page, reg, ls_bit_num, width);
		    if (rc<0) return rc;
		}
	    return count;
	}
	/* process single port */
	sscanf(attr->attr.name+strlen(PORT_PEFIX), "%du", &reg);
	dev_dbg(dev, "name='%s' name+%d='%s' reg=0x%x, page=0x%x, ls_bit_num=0x%x, width=0x%x\n",
			attr->attr.name, strlen(PORT_PEFIX), ( attr->attr.name+strlen(PORT_PEFIX)),  reg, page, ls_bit_num,  width);

	return field_store_reg(dev, buf, count, page, reg, ls_bit_num, width);
}
static ssize_t field_store_reg(struct device *dev, const char *buf, size_t count,
		int page, int reg, int ls_bit_num, int width)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val,rc;
    sscanf(buf, "%du", &val);
    rc=write_page_field(client, page, reg, val, ls_bit_num, width);
    return count;
}


static ssize_t connection_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_CONNECTION, 0, 5);}
static ssize_t connection_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_CONNECTION, 0, 5);}

static ssize_t input_ISE_short_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_ISE, 4, 2);}
static ssize_t input_ISE_short_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_ISE, 4, 2);}

static ssize_t input_ISE_medium_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_ISE, 2, 2);}
static ssize_t input_ISE_medium_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_ISE, 2, 2);}

static ssize_t input_ISE_long_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_ISE, 0, 2);}
static ssize_t input_ISE_long_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_ISE, 0, 2);}

static ssize_t input_state_off_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_STATE, 1, 1);}
static ssize_t input_state_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_STATE, 1, 1);}

static ssize_t input_state_invert_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_STATE, 0, 1);}
static ssize_t input_state_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_STATE, 0, 1);}

static ssize_t input_LOS_threshold_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_LOS, 0, 3);}
static ssize_t input_LOS_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_LOS, 0, 3);}

static ssize_t output_PRE_long_level_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_PRE_LONG, 3, 4);}
static ssize_t output_PRE_long_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_PRE_LONG, 3, 4);}

static ssize_t output_PRE_long_decay_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_PRE_LONG, 0, 3);}
static ssize_t output_PRE_long_decay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_PRE_LONG, 0, 3);}

static ssize_t output_PRE_short_level_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_PRE_SHORT, 3, 4);}
static ssize_t output_PRE_short_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_PRE_SHORT, 3, 4);}

static ssize_t output_PRE_short_decay_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_PRE_SHORT, 0, 3);}
static ssize_t output_PRE_short_decay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_PRE_SHORT, 0, 3);}
/* TODO - fix for vsc3312*/
static ssize_t input_terminate_low_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_INPUT_STATE, 2, 1);}
static ssize_t input_terminate_low_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_INPUT_STATE, 2, 1);}
static ssize_t input_terminate_high_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_LEVEL, 4, 1);}
static ssize_t input_terminate_high_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_LEVEL, 4, 1);}

static ssize_t output_level_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_LEVEL, 0, 4);}
static ssize_t output_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_LEVEL, 0, 4);}

static ssize_t output_mode_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_STATE, 1, 4);}
static ssize_t output_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_STATE, 1, 4);}

static ssize_t forward_OOB_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_OUTPUT_STATE, 0, 1);}
static ssize_t forward_OOB_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_OUTPUT_STATE, 0, 1);}

static ssize_t status_0_on_LOS_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_STATUS0_CONFIGURE, 0, 1);}
static ssize_t status_0_on_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_STATUS0_CONFIGURE, 0, 1);}

static ssize_t status_1_on_LOS_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show(dev, attr, buf,         I2C_PAGE_STATUS1_CONFIGURE, 0, 1);}
static ssize_t status_1_on_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store(dev, attr, buf, count, I2C_PAGE_STATUS1_CONFIGURE, 0, 1);}
/* per-port LOS status */
static ssize_t status_show (struct device *dev, struct device_attribute *attr, char *buf)
    {return field_show(dev, attr, buf,         I2C_PAGE_STATUS_READ, 0, 1);}
/* combined LOS status */
static ssize_t status_combo_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg(dev, buf, I2C_PAGE_STATUS_READ, 0x10, 0, 2);}


/* Global registers use all 8 bits (and control multiple features at once, because it is not possible to read-modify-write them */
static ssize_t global_connection_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_CONNECTION, 0, 8);}
static ssize_t global_connection_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_CONNECTION, 0, 8);}


static ssize_t global_ISE_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_ISE, 0, 8);}
static ssize_t global_ISE_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_ISE, 0, 8);}

static ssize_t global_input_state_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_STATE, 0, 8);}
static ssize_t global_input_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_STATE, 0, 8);}

static ssize_t global_input_LOS_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_LOS, 0, 8);}
static ssize_t global_input_LOS_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_INPUT_LOS, 0, 8);}

static ssize_t global_output_PRE_long_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_PRE_LONG, 0, 8);}
static ssize_t global_output_PRE_long_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_PRE_LONG, 0, 8);}

static ssize_t global_output_PRE_short_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_PRE_SHORT, 0, 8);}
static ssize_t global_output_PRE_short_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_PRE_SHORT, 0, 8);}

static ssize_t global_output_level_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_LEVEL, 0, 8);}
static ssize_t global_output_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_LEVEL, 0, 8);}

static ssize_t global_output_state_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_STATE, 0, 8);}
static ssize_t global_output_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_OUTPUT_STATE, 0, 8);}

static ssize_t global_status_0_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_STATUS0, 0, 8);}
static ssize_t global_status_0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_STATUS0, 0, 8);}

static ssize_t global_status_1_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_GLOBAL_STATUS1, 0, 8);}
static ssize_t global_status_1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_GLOBAL_STATUS1, 0, 8);}

static ssize_t core_config_word_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 0, 8);}
static ssize_t core_config_word_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 0, 8);} /* 0x18/ 0x19 for 3312 */

static ssize_t core_left_bias_en_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 4, 1);}
static ssize_t core_left_bias_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 4, 1);}

static ssize_t core_right_bias_en_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 3, 1);}
static ssize_t core_right_bias_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 3, 1);}

static ssize_t core_buffer_on_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 1, 1);}
static ssize_t core_buffer_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 1, 1);}

static ssize_t core_config_pin_invert_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 0, 1);}
static ssize_t core_config_pin_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_CORE_CONFIGURATION, 0, 1);}

static ssize_t soft_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;
	if (((rc=field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_SOFTWARE_RESET, 4, 1)))<0) return rc;
	if (((rc=init_device(client)))<0) return rc;
	return count;
}

static ssize_t address_range_show (struct device *dev, struct device_attribute *attr, char *buf)
	{return field_show_reg (dev, buf,        I2C_PAGE_GLOBAL, I2C_SOFTWARE_RESET, 0, 4);}
static ssize_t address_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{return field_store_reg(dev, buf, count, I2C_PAGE_GLOBAL, I2C_SOFTWARE_RESET, 0, 4);}

/* Setup i2c, write address_mode. Needed after soft reset and power up */

static int _init_device(struct i2c_client *client, int address_mode_data)
{
	int rc;
    if (((rc=write_reg(client, I2C_INTERFACE_MODE, I2C_INTERFACE_MODE_DATA)))<0) return rc;
    if (address_mode_data >=0){ /* only write if needed, 3304 needs, 3312 - does not, 3308 - ? */
//        if (((rc=write_reg(client, I2C_SOFTWARE_RESET_DATA, address_mode_data)))<0) return rc;
// this bug caused me to assemble a new board, urgently find vsc3304 chip,spend several days trying to troubleshoot...
// Just five extra characters!
        if (((rc=write_reg(client, I2C_SOFTWARE_RESET, address_mode_data)))<0) return rc;
    }
    if (((rc=write_reg(client, I2C_CURRENT_PAGE, 0)))<0) return rc;
	return 0;
}


static int init_device(struct i2c_client *client)
{
	int rc;
	struct vsc330x_data_t *clientdata = i2c_get_clientdata(client);
	dev_info(&client->dev,"Re-initializing %s\n",client->name);
	if (((rc=_init_device(client, clientdata->address_mode_data)))<0) return rc;
	clientdata->last_page=0;
	return 0;
}

static int bitmask(int ls_bit_num, int width)
{
	return ((1 << width) -1) << ls_bit_num;
}
//address_mode_data
static int read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	val= i2c_smbus_read_byte_data(client, reg);
	dev_dbg(&client->dev,"reading i2c device : slave=0x%x, reg=0x%x -> 0x%x\n",(int) (client->addr),reg,val);
	return val;
}

static int write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	dev_dbg(&client->dev,"device write: slave=0x%x, reg=0x%x, val=0x%x\n", (int) (client->addr),reg,val);
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int read_field(struct i2c_client *client, u8 reg, int ls_bit_num, int width)
{
	int rc;
	if (((rc=read_reg(client, reg)))<0) return rc;
	rc= (rc >> ls_bit_num) & bitmask(0, width);
	dev_dbg(&client->dev,"reg=0x%x, ls_bit_num=%d, width=%d -> 0x%x\n", (int) reg, ls_bit_num,width,rc);
    return rc;
}
static int read_page_field(struct i2c_client *client, int page, u8 reg, int ls_bit_num, int width)
{
	int rc;
	struct vsc330x_data_t *clientdata = i2c_get_clientdata(client);
	dev_dbg(&client->dev,"page=0x%x (last was 0x%x) reg=0x%x, ls_bit_num=%d, width=%d:\n",
			page, (clientdata->last_page!=page), (int) reg, ls_bit_num, width);
	if ((page>=0) && (clientdata->last_page!=page)) {
		if (((rc=write_reg(client, I2C_CURRENT_PAGE, page)))<0) return rc;
		clientdata->last_page=page;
	}
	rc= read_field(client, reg, ls_bit_num, width);
	dev_dbg(&client->dev,"page=0x%x (last was 0x%x) reg=0x%x, ls_bit_num=%d, width=%d -> 0x%x\n",
			page, (clientdata->last_page!=page), (int) reg, ls_bit_num, width,rc);
	return rc;
}
static int write_field(struct i2c_client *client, u8 reg, u8 val, int ls_bit_num, int width)
{
	int rc;
	dev_dbg(&client->dev,"reg=0x%x, val=0x%x, ls_bit_num=%d, width=%d \n", (int) reg, (int) val,ls_bit_num,width);
	if (((rc=read_reg(client, reg)))<0) return rc;
	val<<=ls_bit_num;
	return  write_reg(client, reg, ((rc ^ val) &  bitmask(ls_bit_num, width))^ rc);
}

static int write_page_field(struct i2c_client *client, int page, u8 reg, u8 val, int ls_bit_num, int width)
{
	int rc;
	struct vsc330x_data_t *clientdata = i2c_get_clientdata(client);
	dev_dbg(&client->dev,"page=0x%x (last was 0x%x), reg=0x%x, val=0x%x, ls_bit_num=%d, width=%d \n",
			page,clientdata->last_page, (int) reg, (int) val,ls_bit_num,width);
	if ((page>=0) && (clientdata->last_page!=page)) {
		if (((rc=write_reg(client, I2C_CURRENT_PAGE, page)))<0) return rc;
		clientdata->last_page=page;
	}
	return write_field(client, reg, val, ls_bit_num, width);
}

static int write_with_mask(struct i2c_client *client, u8 reg, u8 val, u8 mask)
{
	int rc;
	dev_dbg(&client->dev,"reg=0x%x, val=0x%x, mask=0x%x\n", (int) reg, (int) val, (int) mask);
	if (mask !=0xff){
	    if (((rc=read_reg(client, reg)))<0) return rc;
	    val=((val ^ rc) & mask)^ rc;
	}
	return  write_reg(client, reg, val);
}

static int write_page_with_mask(struct i2c_client *client, int page, u8 reg, u8 val , u8 mask)
{
	int rc;
	struct vsc330x_data_t *clientdata = i2c_get_clientdata(client);
	dev_dbg(&client->dev,"page=0x%x (last was 0x%x), reg=0x%x, val=0x%x, mask=0x%x\n",
			page,clientdata->last_page, (int) reg, (int) val,(int) mask);
	if ((page>=0) && (clientdata->last_page!=page)) {
		if (((rc=write_reg(client, I2C_CURRENT_PAGE, page)))<0) return rc;
		clientdata->last_page=page;
	}
	return write_with_mask(client, reg, val, mask);
}



static void vsc330x_init_of(struct i2c_client *client)
{
//	struct device *dev=&client->dev;
	const __be32 * config_data;
	struct device_node *node = client->dev.of_node;
    int len,i,rc;
    struct vsc330x_setup_data  {
    	u8		page;
    	u8		reg;
    	u8		data;
    	u8		mask;
    };
    struct vsc330x_setup_data setup_data;
    __be32 * setup_data_be32= (__be32 *) &setup_data;


    const char * config_name;
	if (node) {

		config_name = of_get_property(client->dev.of_node, "vsc330x,configuration_name", &len);
		if (config_name){
			dev_info(&client->dev,"Initializing %s registers for \"%s\"\n",client->name,config_name);
		}
		config_data = of_get_property(client->dev.of_node, "vsc330x,configuration_data", &len);
		if (config_data){
			len /= sizeof(*config_data);
			dev_dbg(&client->dev,"Read %d values\n",len);
			for (i=0;i<len;i++){
			    dev_dbg(&client->dev,"0x%08x (0x%08x)\n", config_data[i],be32_to_cpup(config_data+i));
				*setup_data_be32=config_data[i];
			    dev_dbg(&client->dev,"page=0x%02x, reg=0x%02x, data=0x%02x, mask=0x%02x \n",
			    		(int)setup_data.page, (int)setup_data.reg,(int)setup_data.data,(int)setup_data.mask);
			    if (((rc=write_page_with_mask(client, (setup_data.page==0xff)?-1:setup_data.page, setup_data.reg,
			    		setup_data.data, setup_data.mask)))<0) return;
			}
		} else {
			dev_info(&client->dev,"'vsc330x,configuration_data' not found\n");
		}
	} else {
		dev_info(&client->dev,"Device tree data not found for %s\n",client->name);
	}
}
/*
 	dev_info(&client->dev,

 */




static int vsc330x_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int rc=0;
	struct vsc330x_data_t *clientdata = NULL;
	/* initialize i2c mode and (if needed) address range bit field */

	if (((rc=_init_device(client, vsc330x_data[id->driver_data].address_mode_data)))<0) goto wr_err;
	dev_info(&client->dev,
		 "Chip %s found, driver version %s\n", id->name, DRV_VERSION);
	clientdata = devm_kzalloc(&client->dev, sizeof(*clientdata), GFP_KERNEL);
	if (!clientdata) {
		rc = -ENOMEM;
		goto exit;
	}
	clientdata->last_page = 0;
	clientdata->address_mode_data = vsc330x_data[id->driver_data].address_mode_data;
	clientdata->in_ports = vsc330x_data[id->driver_data].in_ports;
	clientdata->out_ports = vsc330x_data[id->driver_data].out_ports;
	i2c_set_clientdata(client, clientdata);
	rc = vsc330x_sysfs_register(&client->dev);
	if (rc)
		goto exit;
	vsc330x_init_of(client);
	return 0;	/* found OK*/
wr_err:
	rc = -EIO;
	dev_err(&client->dev, "%s:%d error writing\n",__func__,__LINE__);
	goto exit;
#if 0
rd_err:
	rc = -EIO;
	dev_err(&client->dev, "%s:%d error reading\n",__func__,__LINE__);
	goto exit;
#endif
exit:
	return rc;

}

static int vsc330x_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver vsc330x_i2c_driver = {
	.driver = {
		.name	= "vsc330x",
		.owner	= THIS_MODULE,
	},
	.probe		= vsc330x_i2c_probe,
	.remove		= vsc330x_i2c_remove,
	.id_table	= vsc330x_id,
};

module_i2c_driver(vsc330x_i2c_driver);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("VSC330x I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:vsc330x");


