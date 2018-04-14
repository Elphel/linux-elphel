/***************************************************************************//**
 * @file      detect_sensors.c
 * @brief     Determine sensor boards attached to each of the ports. Use
 *            Device Tree, sysfs to set sensor types per port. Add autodetection
 *            (using pullup/pull downs) later
 * @copyright Copyright 2016 (C) Elphel, Inc.
 * @par <b>License</b>
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#define DEBUG
//#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>

//#include <asm/outercache.h>     // TODO: Implement cache operations for the membridge !!!!
//#include <asm/cacheflush.h>

#include <linux/errno.h>
#include <linux/fs.h>

#include "x393.h"
#include <uapi/elphel/x393_devices.h>
#include <uapi/elphel/c313a.h>
#include "latency.h"
#include "mt9x001.h"
#include "mt9f002.h"
#include "multi10359.h"
#include "detect_sensors.h"

#define DETECT_SENSORS_MODULE_DESCRIPTION "Detect sensor type(s) attached to each of the ports"
#define OF_PREFIX_NAME                    "elphel393-detect_sensors"

struct sensor_port_config_t *pSensorPortConfig;

// removed static to export
static struct sensor_port_config_t sensorPortConfig[] = {
        {.mux=SENSOR_NONE,.broadcast_addr=0,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.broadcast_addr=0,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.broadcast_addr=0,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.broadcast_addr=0,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}}
};

static const struct of_device_id elphel393_detect_sensors_of_match[];
static struct device *g_dev_ptr; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
struct sensor_name_t {
    const char * name;
    u32          code;
    int          type; ///< +1 - applicable to sensors, +2 - applicable to multiplexers
    sens_iface_t iface;
};
//typedef enum {NONE,PARALLEL12,HISPI} sens_iface_t; ///< Sensor port interface type

const struct sensor_name_t sensor_names[] ={
        {.name="detect",     .type=3, .iface=NONE,       .code = 0},                // to be automatically detected
        {.name="none",       .type=3, .iface=NONE,       .code = SENSOR_NONE},      // no device attached
        {.name="mux10359",   .type=2, .iface=PARALLEL12, .code = SENSOR_MUX_10359}, // no device attached
        {.name="zr32112",    .type=1, .iface=PARALLEL12, .code = SENSOR_ZR32112},   // Zoran ZR32112
        {.name="zr32212",    .type=1, .iface=PARALLEL12, .code = SENSOR_ZR32212},   // Zoran ZR32212
        {.name="kac1310",    .type=1, .iface=PARALLEL12, .code = SENSOR_KAC1310},   // Kodak KAC1310
        {.name="kac5000",    .type=1, .iface=PARALLEL12, .code = SENSOR_KAC5000},   // Kodak KAC5000
        {.name="mi1300",     .type=1, .iface=PARALLEL12, .code = SENSOR_MI1300},    // Micron MI1300
        {.name="mt9m001",    .type=1, .iface=PARALLEL12, .code = SENSOR_MT9M001},   // MT9M001
        {.name="mt9d001",    .type=1, .iface=PARALLEL12, .code = SENSOR_MT9D001},   // MT9D001
        {.name="mt9t001",    .type=1, .iface=PARALLEL12, .code = SENSOR_MT9T001},   // MT9T001
        {.name="mt9p006",    .type=1, .iface=PARALLEL12, .code = SENSOR_MT9P006},   // MT9P006
        {.name="mt9f002",    .type=1, .iface=HISPI4,     .code = SENSOR_MT9F002},   // MT9F002
        {.name="ibis51300",  .type=1, .iface=PARALLEL12, .code = SENSOR_IBIS51300}, // FillFactory IBIS51300
        {.name="kai11002",   .type=1, .iface=PARALLEL12, .code = SENSOR_KAI11000},  // Kodak KAI11002
        {.name=NULL,         .type=0, .iface=NONE,       .code = 0} // end of list
};
static sens_iface_t port_iface[SENSOR_PORTS];
//#define DETECT_SENSOR 1 ///< Include sensors, May be OR-ed when looking for sensor/multiplexer code/name
//#define DETECT_MUX 2    ///< Include multiplexers, May be OR-ed when looking for sensor/multiplexer code/name

/** Get sensor/multiplexer code (SENSOR_*) by name */
int get_code_by_name(const char * name, ///< sensor name
                     int    type)       ///< valid type [DETECT_SENSOR]|[DETECT_MUX]
                                        ///< @return sensor code or -EINVAL for invalid name
{
    int i;
    if (name) for (i = 0; sensor_names[i].name; i++){
        if ((sensor_names[i].type & type) && (!strncmp(sensor_names[i].name,name,80))){
            return sensor_names[i].code;
        }
    }
    return -EINVAL;

}

/** Get sensor/multiplexer name (SENSOR_*) by code */
const char * get_name_by_code(int code, ///< sensor code
                              int type) ///< valid type [DETECT_SENSOR]|[DETECT_MUX]
                                        ///< @return sensor name or NULL for invalid code
{
    int i;
    for (i = 0; sensor_names[i].name; i++){
        if ((sensor_names[i].type & type) && (sensor_names[i].code == code)){
            return sensor_names[i].name;
        }
    }
    return NULL;
}

/** Get sensor/multiplexer interface type by code */
sens_iface_t get_iface_by_code(int code, ///< sensor code
                               int type) ///< valid type [DETECT_SENSOR]|[DETECT_MUX]
                                         ///< @return sensor name or NULL for invalid code
{
    int i;
    for (i = 0; sensor_names[i].name; i++){
        if ((sensor_names[i].type & type) && (sensor_names[i].code == code)){
            return sensor_names[i].iface;
        }
    }
    return NONE;
}

/** Get sensor port multiplexer type */
int set_broadcast_address(int port,int value){
	sensorPortConfig[port & 3].broadcast_addr = value;
	return 0;
}

/** Get sensor port multiplexer type */
int get_detected_mux_code(int port) ///< Sensor port number (0..3)
                                    ///< @return port multiplexer code (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
{
    return sensorPortConfig[port & 3].mux;
}
/** Get sensor type */
int get_detected_sensor_code(int port,    ///< Sensor port number (0..3)
                             int sub_chn) ///< Sensor subchannel (0..3), -1 - use first defined sub channel
                                          ///< @return sensor code (SENSOR_DETECT, SENSOR_NONE, or SENSOR_*)
{
    int nchn,code;
    port &= 3;
    if (sub_chn >= 0)
        return sensorPortConfig[port].sensor[sub_chn & 3];
    // Negative sensor - find first defined
    nchn = (get_detected_mux_code(port) == SENSOR_NONE)? 1: MAX_SENSORS;
    for (sub_chn = 0; sub_chn < nchn; sub_chn++){
        code = sensorPortConfig[port].sensor[sub_chn];
        if ((code != SENSOR_DETECT) && (code != SENSOR_NONE))
            return code;
    }
    return SENSOR_NONE;
}

/** Get configured  sensor port subchannels */
int get_subchannels(int port) ///< Sensor port
                              ///< @return bitmask of available channels
{
    int sub_chn, chn_mask = 0;
    int nchn = (get_detected_mux_code(port) == SENSOR_NONE)? 1: MAX_SENSORS;
    for (sub_chn = 0; sub_chn < nchn; sub_chn++){
        if ((sensorPortConfig[port].sensor[sub_chn]!= SENSOR_DETECT) && (sensorPortConfig[port].sensor[sub_chn] != SENSOR_NONE)) {
            chn_mask |= 1 << sub_chn;
        }
    }
    return chn_mask;

}


/** Update per-port interface type after changing sensor/multiplexer */
void update_port_iface(int port)  ///< Sensor port number (0..3)
{
    sens_iface_t iface = get_iface_by_code(get_detected_mux_code(port), DETECT_MUX);
    if (iface != NONE) {
        port_iface[port] = iface;
        return;
    }
    port_iface[port] = get_iface_by_code(get_detected_sensor_code(port,-1), DETECT_MUX); // '-1' - any subchannel
}

/** Get per-port interface type */
sens_iface_t get_port_interface(int port)  ///< Sensor port number (0..3)
                                           ///< @ return interface type (none, parallel12, hispi4
{
    return port_iface[port];
}

/** init port and subchn with default ahead_tab from latency.h */
int init_port_ahead_table(int port,
						  int sub_chn)
{
	sensorPortConfig[port & 3].ahead_tab[sub_chn] = ahead_tab;
	return 0;
}

/** Set sensor port multiplexer type */
int set_detected_mux_code(int port,      ///< Sensor port number (0..3)
                          int mux_type)  ///< Sensor multiplexer type (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
                                         ///< @return 0 - OK, -EINVAL if mux_type is invalid
{
    if (mux_type < 0)
        return mux_type;
    if (!get_name_by_code(mux_type, DETECT_MUX)){
        pr_err("%s: Invalid port multiplexer code for port %d: 0x%x\n", __func__, port, mux_type);
        return -EINVAL;
    }
    sensorPortConfig[port & 3].mux = mux_type;
    update_port_iface(port);
    return 0;
}

/** Set sensor port multiplexer type */
int set_detected_sensor_code(int port,      ///< Sensor port number (0..3)
                             int sub_chn,   ///< Sensor subchannel (0..3)
                             int sens_type)  ///< Sensor code (SENSOR_DETECT, SENSOR_NONE, or SENSOR_*)
                                            ///< @return  0 - OK, -EINVAL if mux_type is invalid)
{
    if (sens_type < 0)
        return sens_type;
    if (!get_name_by_code(sens_type, DETECT_SENSOR)){
        pr_err("%s: Invalid sensor code for port %d, subchannel %d: 0x%x\n", __func__, port, sens_type);
        return -EINVAL;
    }
    sensorPortConfig[port & 3].sensor[sub_chn] = sens_type;
    update_port_iface(port);
    return 0;
}

// SysFS interface to read/modify video memory map
#define SYSFS_PERMISSIONS           0644 /* default permissions for sysfs files */
#define SYSFS_READONLY              0444
#define SYSFS_WRITEONLY             0222
/** Sysfs helper function - get channel number from the last character of the attribute name*/
static int get_channel_from_name(struct device_attribute *attr) ///< Linux kernel interface for exporting device attributes
///< @return channel number
{
    int reg = 0;
    sscanf(attr->attr.name + (strlen(attr->attr.name)-1), "%du", &reg);
    return reg;
}
/** Sysfs helper function - get channel and sub-channel numbers from the last 2 characters of the attribute name*/
static int get_channel_sub_from_name(struct device_attribute *attr) ///< Linux kernel interface for exporting device attributes
///< @return channel * 16 + sub_channel
{
    int reg = 0;
    sscanf(attr->attr.name + (strlen(attr->attr.name)-2), "%du", &reg);
    reg += (reg/10) * 6;
    return reg;
}

static ssize_t show_port_mux(struct device *dev, struct device_attribute *attr, char *buf)
{
    const char * name = get_name_by_code(get_detected_mux_code(get_channel_from_name(attr)), DETECT_MUX);
    if (name) return sprintf(buf,"%s\n", name);
    // Should never get here
    return sprintf(buf,"0x%x\n", sensorPortConfig[get_channel_from_name(attr)].mux);
}
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    int psch = get_channel_sub_from_name(attr);
    int port = (psch>>4) &3;
    int sub_chn = psch &3;
    const char * name = get_name_by_code(get_detected_sensor_code(port,sub_chn), DETECT_SENSOR);
    if (name) return sprintf(buf,"%s\n", name);
    // Should never get here
    return sprintf(buf,"0x%x\n", sensorPortConfig[(psch>>4) & 3].sensor[psch & 3]);
}
static ssize_t store_port_mux(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//    size_t len;
    char name[80];
    int port = get_channel_from_name(attr);
    int rslt;
    if (sscanf(buf, "%79s", name)){
        if ((rslt = set_detected_mux_code( port, get_code_by_name(name, DETECT_MUX)))<0)
            return rslt;

    }
    return count;
}
static ssize_t store_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//    size_t len;
    char name[80];
    int psch = get_channel_sub_from_name(attr);
    int port = (psch>>4) &3;
    int sub_chn = psch &3;
    int rslt;
    if (sscanf(buf, "%79s", name)){
        if ((rslt = set_detected_sensor_code(port, sub_chn, get_code_by_name(name, DETECT_SENSOR)))<0)
            return rslt;
    }
    return count;
}

static DEVICE_ATTR(port_mux0,                  SYSFS_PERMISSIONS,     show_port_mux,                   store_port_mux);
static DEVICE_ATTR(port_mux1,                  SYSFS_PERMISSIONS,     show_port_mux,                   store_port_mux);
static DEVICE_ATTR(port_mux2,                  SYSFS_PERMISSIONS,     show_port_mux,                   store_port_mux);
static DEVICE_ATTR(port_mux3,                  SYSFS_PERMISSIONS,     show_port_mux,                   store_port_mux);
static DEVICE_ATTR(sensor00,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor01,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor02,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor03,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor10,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor11,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor12,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor13,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor20,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor21,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor22,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor23,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor30,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor31,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor32,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);
static DEVICE_ATTR(sensor33,                   SYSFS_PERMISSIONS,     show_sensor,                     store_sensor);

static struct attribute *root_dev_attrs[] = {
        &dev_attr_port_mux0.attr,
        &dev_attr_port_mux1.attr,
        &dev_attr_port_mux2.attr,
        &dev_attr_port_mux3.attr,
        &dev_attr_sensor00.attr,
        &dev_attr_sensor01.attr,
        &dev_attr_sensor02.attr,
        &dev_attr_sensor03.attr,
        &dev_attr_sensor10.attr,
        &dev_attr_sensor11.attr,
        &dev_attr_sensor12.attr,
        &dev_attr_sensor13.attr,
        &dev_attr_sensor20.attr,
        &dev_attr_sensor21.attr,
        &dev_attr_sensor22.attr,
        &dev_attr_sensor23.attr,
        &dev_attr_sensor30.attr,
        &dev_attr_sensor31.attr,
        &dev_attr_sensor32.attr,
        &dev_attr_sensor33.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
        .attrs = root_dev_attrs,
        .name  = NULL,
};


static int elphel393_detect_sensors_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}
/** Initialize this driver from the Device Tree.
 * Read per-port multiplexers and sensors. */
 static void detect_sensors_init_of(struct platform_device *pdev) ///< Platform device structure for this driver
 ///< @return 0 on success, or negative error.
 {
     const   char * config_string;
     char names[4][80];
     struct device_node *node = pdev->dev.of_node;
     int num_ports, port, num_sub, sub_chn;
     if (node) {
         config_string = of_get_property(node,  OF_PREFIX_NAME",port-mux", NULL); // &len);
         pr_info ("Mux config_string = %s (was looking for '%s')\n",config_string, OF_PREFIX_NAME",port-mux");
         if (config_string) {
             num_ports = sscanf(config_string,"%79s %79s %79s %79s", names[0],  names[1],  names[2],  names[3]);
             if (num_ports > SENSOR_PORTS)
                 num_ports = SENSOR_PORTS;
             pr_info ("num_ports= %d\n",num_ports);
             for (port = 0; port < num_ports; port++){
                 pr_info ("Setting port %d mux  '%s' (0x%x)\n",port, names[port], get_code_by_name(names[port], DETECT_MUX));
                 set_detected_mux_code(port, get_code_by_name(names[port], DETECT_MUX));
             }
         }
         num_ports = of_property_count_strings(node,OF_PREFIX_NAME",sensors");
         if (num_ports > SENSOR_PORTS)
             num_ports = SENSOR_PORTS;
         pr_info ("num_ports = %d (was looking for '%s')\n",num_ports, OF_PREFIX_NAME",sensors");
         for (port = 0; port < num_ports; port++){
             if (of_property_read_string_index(node, OF_PREFIX_NAME",sensors", port, &config_string)) {
                 pr_err("%s: No data for selected port\n", __func__);
                 BUG();
             }
             pr_info ("Sensor config_string = %s\n",config_string);
             if (config_string) {
                 num_sub = sscanf(config_string,"%79s %79s %79s %79s", names[0],  names[1],  names[2],  names[3]);
                 pr_info ("port %d : %d subchannels\n",port, num_sub);
                 for (sub_chn = 0; sub_chn < num_sub; sub_chn++){
                     pr_info ("Setting sensor %d:%d '%s' (0x%x)\n",port, sub_chn, names[sub_chn], get_code_by_name(names[sub_chn], DETECT_SENSOR));
                     set_detected_sensor_code(port, sub_chn,  get_code_by_name(names[sub_chn], DETECT_SENSOR));
                     init_port_ahead_table(port,sub_chn);
                 }
             }
         }
     }
 }

/**
 * Fills the tables in sensorPortConfig with key-value pairs
 * @param par2addr - pointer to the look-up table
 * @param table - pointer to the global struct
 * @return 0
 */
static int par2addr_fill(const unsigned short *par2addr, u32 *table){

	int i=0;

	int key;
	unsigned short value;

	// reset
	for(i=0;i<MAX_SENSOR_REGS;i++){
		table[i] = 0xffffffff;
	}

	i=0;
	// fill with key-value pairs
	while(true){
		key   = par2addr[2*i];
		value = par2addr[2*i+1];
		if ((key==0xffff)||(i>255)){
			break;
		}
		table[key] = value;
		i++;
	}

	return 0;
}

/**
 * Based on sensorPortConfig[i].sensor[j], filled from DT,
 *   gets SENSOR_REGS to true register addresses table for
 *   the specified sensor
 * @return 0
 */
int detect_sensors_par2addr_init(int port,int sub_chn){

	const unsigned short *par2addr;
	const unsigned short *pages;
	const unsigned short *atab;

	/*
	struct sensor_port_config_t {
		u32  mux;                                    ///< Sensor multiplexer, currently 0 (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
	    u32  sensor[MAX_SENSORS];                    ///< Without mux only [0] is used, with 10359 - 0..2 are used (i2c addressing is shifted so 0 is broadcast)
	    u16  par2addr[MAX_SENSORS][MAX_SENSOR_REGS]; ///< Big LUT. SENSOR_REGSxxx par to sensor reg 'yyy' internal address: haddr+laddr for 16 bit
	    u16  haddr2rec[MAX_SENSORS][MAX_FPGA_RECS];  ///< Big LUT (but almost empty). Sensor's page address (haddr of reg addr) to fpga i2c record number (fpga line#)
	};
	*/

	switch (sensorPortConfig[port].sensor[sub_chn]) {
		case SENSOR_MT9P006:
			// get sensor table
			par2addr  = mt9x001_par2addr;
			pages     = mt9x001_pages;
			atab      = mt9x001_ahead_tab;
			break;
		case SENSOR_MT9F002:
			// get sensor table
			par2addr  = mt9f002_par2addr;
			pages     = mt9f002_pages;
			atab      = mt9f002_ahead_tab;
			break;
	}
	if (par2addr){
		// convert to key-value
		par2addr_fill(par2addr,sensorPortConfig[port].par2addr[sub_chn]);
		// save pointer to static LUT
		sensorPortConfig[port].pages_ptr[sub_chn] = pages;
	}
	if(atab){
		sensorPortConfig[port].ahead_tab[sub_chn] = atab;
	}

	/*
	// all .mux and .sensor are already filled out
	for (portx = 0; portx < SENSOR_PORTS; portx++){

		// that's from device tree, fpga is not programmed yet
		dev_dbg(g_dev_ptr,"port: %d mux: %d sensors: %d %d %d %d\n",
				portx,
				sensorPortConfig[portx].mux,
				sensorPortConfig[portx].sensor[0],
				sensorPortConfig[portx].sensor[1],
				sensorPortConfig[portx].sensor[2],
				sensorPortConfig[portx].sensor[3]
				);

		// sub_chn = 3 is never used
		for (sub_chn = 0; sub_chn < 4; sub_chn++){
			//sensorPortConfig[port].sensor[sub_chn];
			switch (sensorPortConfig[portx].sensor[sub_chn]) {
				case SENSOR_MT9P006:
					// get sensor table
					par2addr = mt9x001_par2addr;
					pages    = mt9x001_pages;
					break;
				case SENSOR_MT9F002:
					// get sensor table
					par2addr = mt9f002_par2addr;
					pages    = mt9f002_pages;
					break;
			}
			if (par2addr){
				// convert to key-value
				par2addr_fill(par2addr,sensorPortConfig[portx].par2addr[sub_chn]);
				// save pointer to static LUT
				sensorPortConfig[portx].pages_ptr[sub_chn] = pages;
			}
		}
	}
	*/

	return 0;
}

 static int detect_sensors_probe(struct platform_device *pdev)
 {
     //unsigned int irq;
     //int res;
     struct device *dev = &pdev->dev;
     const struct of_device_id *match;
     //const __be32 *bufsize_be;
     //struct device_node *node;

     g_dev_ptr = dev; // for debugfs
     pSensorPortConfig = sensorPortConfig;

     elphel393_detect_sensors_sysfs_register(pdev);
     pr_info ("Registered sysfs for detect_sensors");
     match = of_match_device(elphel393_detect_sensors_of_match, dev);
     if (!match) {
         pr_err("Detect sensors ERROR: No device tree for '%s' node found\n",elphel393_detect_sensors_of_match[0].compatible);
         return -EINVAL;
     }

     detect_sensors_init_of(pdev);

     // move this fucntion to a later stage, right before fpga pages get allocated (pgm_functions.c)
     //detect_sensors_par2addr_init();

     //    dev_dbg(dev, "Registering character device with name "DEV393_NAME(DEV393_DETECT_SENSORS));
     //    res = register_chrdev(DETECT_SENSORS_MAJOR, DEV393_NAME(DEV393_DETECT_SENSORS), &detect_sensors_fops);
     //    if(res < 0) {
     //        dev_err(dev, "\nlogger_init: couldn't get a major number  %d.\n ",DETECT_SENSORS_MAJOR);
     //        return res;
     //    }

     return 0;
 }


 /** IMU/GPS logger driver remove function */
 static int detect_sensors_remove(struct platform_device *pdev) ///< [in] pointer to @e platform_device structure
 ///< @return always 0
 {
     //    unregister_chrdev(DETECT_SENSORS_MAJOR, DEV393_NAME(DEV393_DETECT_SENSORS));

     return 0;
 }

 static const struct of_device_id elphel393_detect_sensors_of_match[] = {
         { .compatible = "elphel,elphel393-detect_sensors-1.00" },
         { /* end of list */ }
 };

 MODULE_DEVICE_TABLE(of, elphel393_detect_sensors_of_match);

 static struct platform_driver elphel393_detect_sensors = {
         .probe          = detect_sensors_probe,
         .remove         = detect_sensors_remove,
         .driver = {
                 .name =           DEV393_NAME(DEV393_DETECT_SENSORS),
                 .of_match_table = elphel393_detect_sensors_of_match,
         },
 };

 module_platform_driver(elphel393_detect_sensors);


 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
 MODULE_DESCRIPTION(DETECT_SENSORS_MODULE_DESCRIPTION);

