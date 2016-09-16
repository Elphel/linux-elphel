/***************************************************************************//**
* @file   clock10359.h
* @brief   Control of the CY22393 clock on the 10359 multiplexer connected
* to the sensor port
* Copyright 2002-2016 (C) Elphel, Inc.
* @par <b>License</b>
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
*******************************************************************************/
 
int x393_setClockFreq(int sensor_port, int nclock, int freq);
int x393_getClockFreq(int sensor_port, int nclock);
