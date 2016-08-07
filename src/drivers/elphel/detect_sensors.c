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
//#include "x393_detect_sensors.h"
#include <elphel/driver_numbers.h>
#include <elphel/c313a.h>
#include "mt9x001.h"
#include "multi10359.h"
#include "detect_sensors.h"

#define DETECT_SENSORS_MODULE_DESCRIPTION "Detect sensor type(s) attached to each of the ports"
#define DETECT_SENSORS_DRIVER_NAME        "detect_sensors"
#define OF_PREFIX_NAME                    "elphel393-detect_sensors"
struct sensor_port_config_t
 {
     u32  mux;                 ///< sensor multiplexer, currently 0 (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
     u32  sensor[MAX_SENSORS]; ///< Without mux only [0] is used, with 10359 - 0..2 are used (i2c addressing is shifted so 0 is broadcast)
 };

static struct sensor_port_config_t sensorPortConfig[] = {
        {.mux=SENSOR_NONE,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}},
        {.mux=SENSOR_NONE,.sensor={SENSOR_NONE,SENSOR_NONE,SENSOR_NONE,SENSOR_NONE}}
};
//struct sensor_port_config_t *pSensorPortConfig;
static const struct of_device_id elphel393_detect_sensors_of_match[];
static struct device *g_dev_ptr; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
struct sensor_name_t {
    const char * name;
    u32          code;
    int          type; ///< +1 - applicable to sensors, +2 - applicable to multiplexers
};
const struct sensor_name_t sensor_names[] ={
        {.name="detect",     .type=3, .code = 0},                // to be automatically detected
        {.name="none",       .type=3, .code = SENSOR_NONE},      // no device attached
        {.name="mux10359",   .type=2, .code = SENSOR_MUX_10359}, // no device attached
        {.name="zr32112",    .type=1, .code = SENSOR_ZR32112},   // Zoran ZR32112
        {.name="zr32212",    .type=1, .code = SENSOR_ZR32212},   // Zoran ZR32212
        {.name="kac1310",    .type=1, .code = SENSOR_KAC1310},   // Kodak KAC1310
        {.name="kac5000",    .type=1, .code = SENSOR_KAC5000},   // Kodak KAC5000
        {.name="mi1300",     .type=1, .code = SENSOR_MI1300},    // Micron MI1300
        {.name="mt9m001",    .type=1, .code = SENSOR_MT9M001},   // MT9M001
        {.name="mt9d001",    .type=1, .code = SENSOR_MT9D001},   // MT9D001
        {.name="mt9t001",    .type=1, .code = SENSOR_MT9T001},   // MT9T001
        {.name="mt9p006",    .type=1, .code = SENSOR_MT9P006},   // MT9P006
        {.name="mt9f002",    .type=1, .code = SENSOR_MT9F002},   // MT9F002
        {.name="ibis51300",  .type=1, .code = SENSOR_IBIS51300}, // FillFactory IBIS51300
        {.name="kai11002",   .type=1, .code = SENSOR_KAI11000},  // Kodak KAI11002
        {.name=NULL,         .type=0, .code = 0} // end of list
};
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


/** Get sensor port multiplexer type */
int get_detected_mux_code(int port) ///< Sensor port number (0..3)
                                    ///< @return port multiplexer code (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
{
    return sensorPortConfig[port & 3].mux;
}
/** Get sensor type */
int get_detected_sensor_code(int port,    ///< Sensor port number (0..3)
                             int sub_chn) ///< Sensor subchannel (0..3)
                                          ///< @return sensor code (SENSOR_DETECT, SENSOR_NONE, or SENSOR_*)
{
    return sensorPortConfig[port&3].sensor[sub_chn & 3];
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
    int i;
    const char * name = get_name_by_code(sensorPortConfig[get_channel_from_name(attr)].mux, DETECT_MUX);
    if (name) return sprintf(buf,"%s\n", name);
    // Should never get here
    return sprintf(buf,"0x%x\n", sensorPortConfig[get_channel_from_name(attr)].mux);
}
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i;
    int psch = get_channel_sub_from_name(attr);
    int port = (psch>>4) &3;
    int sub_chn = psch &3;
    const char * name = get_name_by_code(sensorPortConfig[port].sensor[sub_chn], DETECT_SENSOR);
    if (name) return sprintf(buf,"%s\n", name);
    // Should never get here
    return sprintf(buf,"0x%x\n", sensorPortConfig[(psch>>4) & 3].sensor[psch & 3]);
}
static ssize_t store_port_mux(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    size_t len;
    char name[80];
    int port = get_channel_from_name(attr);
    int i, code, rslt;
    if (sscanf(buf, "%79s", name, &len)){
        if ((rslt = set_detected_mux_code( port, get_code_by_name(name, DETECT_MUX)))<0)
            return rslt;

    }
    return count;
}
static ssize_t store_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    size_t len;
    char name[80];
    int psch = get_channel_sub_from_name(attr);
    int port = (psch>>4) &3;
    int sub_chn = psch &3;
    int i, rslt;
    if (sscanf(buf, "%79s", name, &len)){
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
     //    struct x393_i2c_device_list * dl;
     char names[4][80];
//     int rslt;
     //    int num_devs, nd, ni, sa7, num_addr, num_data, khz;
     struct device_node *node = pdev->dev.of_node;
     //    struct device_attribute *new_attr;
//     struct device *dev =&pdev->dev;
     int num_ports, port, num_sub, sub_chn;
     if (node) {
         if (num_ports > SENSOR_PORTS)
             num_ports = SENSOR_PORTS;
         config_string = of_get_property(node,  OF_PREFIX_NAME",ports", NULL); // &len);
         num_ports = sscanf(config_string,"%79s %79s %79s %79s", names[0],  names[1],  names[2],  names[3]);
         for (port = 0; port < num_ports; port++){
             set_detected_mux_code(port, get_code_by_name(names[sub_chn], DETECT_MUX));
         }
         num_ports = of_property_count_strings(node,OF_PREFIX_NAME",sensors");
         for (port = 0; port < num_ports; port++){
             if (of_property_read_string_index(node, OF_PREFIX_NAME",sensors", port, &config_string)) {
                 pr_err("%s: No data for selected port\n", __func__);
                 BUG();
             }
             num_sub = sscanf(config_string,"%79s %79s %79s %79s", names[0],  names[1],  names[2],  names[3]);
             for (sub_chn = 0; sub_chn < num_sub; sub_chn++){
                 set_detected_sensor_code(port, sub_chn,  get_code_by_name(names[sub_chn], DETECT_SENSOR));
             }
         }
     }
 }



 static int detect_sensors_probe(struct platform_device *pdev)
 {
     unsigned int irq;
     int res;
     struct device *dev = &pdev->dev;
     const struct of_device_id *match;
     const __be32 *bufsize_be;
     struct device_node *node;
//     pSensorPortConfig = sensorPortConfig;

     elphel393_detect_sensors_sysfs_register(pdev);

     match = of_match_device(elphel393_detect_sensors_of_match, dev);
     if (!match)
         pr_err("Detect sensors ERROR: No device tree for '%s' node found\n",elphel393_detect_sensors_of_match[0].compatible);
     return -EINVAL;

     detect_sensors_init_of(pdev);


     //    dev_dbg(dev, "Registering character device with name "DETECT_SENSORS_DRIVER_NAME);
     //    res = register_chrdev(DETECT_SENSORS_MAJOR, DETECT_SENSORS_DRIVER_NAME, &detect_sensors_fops);
     //    if(res < 0) {
     //        dev_err(dev, "\nlogger_init: couldn't get a major number  %d.\n ",DETECT_SENSORS_MAJOR);
     //        return res;
     //    }
     g_dev_ptr = dev; // for debugfs
     return 0;
 }


 /** IMU/GPS logger driver remove function */
 static int detect_sensors_remove(struct platform_device *pdev) ///< [in] pointer to @e platform_device structure
 ///< @return always 0
 {
     //    unregister_chrdev(DETECT_SENSORS_MAJOR, DETECT_SENSORS_DRIVER_NAME);

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
                 .name =           DETECT_SENSORS_DRIVER_NAME,
                 .of_match_table = elphel393_detect_sensors_of_match,
         },
 };

 module_platform_driver(elphel393_detect_sensors);


 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
 MODULE_DESCRIPTION(DETECT_SENSORS_MODULE_DESCRIPTION);

