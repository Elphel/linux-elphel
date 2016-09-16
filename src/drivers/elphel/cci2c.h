/** @file cci2c.h
 *
 * @brief Pre-393 I2c driver for FPGA communicating to sensors, software implementation
 * Porting to get GPS communication, sesnors in NC393 are handled by sensor_i2c.c driver
 *
 * @copyright Copyright (C) 2002-2016 Elphel, Inc
 *
 * @par <b>License</b>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

int i2c_delays (unsigned long delays);
int i2c_writeData(int n, unsigned char theSlave, unsigned char *theData, int size, int stop);
int i2c_readData(int n, unsigned char theSlave, unsigned char *theData, int size, int start);
int i2c_ioctl(struct inode *inode, struct file *file,  unsigned int cmd, unsigned long arg);

#ifdef NC353
    void i2c_reset_wait(void);
    void i2c_stop_wait(void);
    void i2c_run(void);
    int  i2s_running(void);
#endif
