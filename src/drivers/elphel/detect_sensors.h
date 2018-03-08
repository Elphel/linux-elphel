/***************************************************************************//**
* @file      detect_sensors.h
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
#ifndef DETECT_SENSORS_H
#define DETECT_SENSORS_H

#define DETECT_SENSOR 1 ///< Include sensors, May be OR-ed when looking for sensor/multiplexer code/name
#define DETECT_MUX 2    ///< Include multiplexers, May be OR-ed when looking for sensor/multiplexer code/name

#define MAX_SENSOR_REGS 256
#define MAX_FPGA_RECS 256

#define MUX_BROADCAST_INDEX 3

struct sensor_port_config_t {
	u32  mux;                                    ///< Sensor multiplexer, currently 0 (SENSOR_DETECT, SENSOR_MUX_10359 or SENSOR_NONE)
	u32  broadcast_addr;                         ///< For MUX: broadcast address for connected sensors, for single sensor it's 0, for mux: 3
    u32  sensor[MAX_SENSORS];                    ///< Without mux only [0] is used, with 10359 - 0..2 are used (i2c addressing is shifted so 0 is broadcast)
    //u16  par2addr[MAX_SENSORS][MAX_SENSOR_REGS]; ///< Big LUT. SENSOR_REGSxxx par to sensor reg 'yyy' internal address: haddr+laddr for 16 bit
    // u32 for error handling
    u32  par2addr[MAX_SENSORS][MAX_SENSOR_REGS]; ///< Big LUT. SENSOR_REGSxxx par to sensor reg 'yyy' internal address: haddr+laddr for 16 bit
    //u8   haddr2rec[MAX_SENSORS][MAX_FPGA_RECS];  ///< Big LUT (but almost empty). Sensor's page address (haddr of reg addr) to fpga i2c record number (fpga line#)
    // u32 is for error handling
    u32   haddr2rec[MAX_SENSORS][MAX_FPGA_RECS];  ///< Big LUT (but almost empty). Sensor's page address (haddr of reg addr) to fpga i2c record number (fpga line#)
    unsigned short *pages_ptr[MAX_SENSORS];
};

extern struct sensor_port_config_t *pSensorPortConfig;

typedef enum {NONE,PARALLEL12,HISPI4} sens_iface_t; ///< Sensor port interface type

int          get_code_by_name(const char * name, int type);
const char * get_name_by_code(int code, int type);
sens_iface_t get_iface_by_code(int code, int type);
int detect_sensors_par2addr_init(int port, int sub_chn);

int get_detected_mux_code(int port);
int set_broadcast_address(int port,int value);
int get_detected_sensor_code(int port, int sub_chn);
int get_subchannels(int port);
int set_detected_mux_code(int port, int mux_type);
int set_detected_sensor_code(int port, int sub_chn,  int mux_type);
sens_iface_t get_port_interface(int port);

#endif
