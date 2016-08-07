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

#define DETECT_SENSOR 1 ///< Include sensors, May be OR-ed when looking for sensor/multiplexer code/name
#define DETECT_MUX 2    ///< Include multiplexers, May be OR-ed when looking for sensor/multiplexer code/name

int          get_code_by_name(const char * name, int type);
const char * get_name_by_code(int code, int type);
int get_detected_mux_code(int port);
int get_detected_sensor_code(int port, int sub_chn);
int set_detected_mux_code(int port, int mux_type);
int set_detected_sensor_code(int port, int sub_chn,  int mux_type);
