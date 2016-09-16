/** @file imu_log393.h
 *
 * @brief reading logger data
 *
 * @copyright Copyright (C) 2011-2016 Elphel, Inc
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

void          logger_dma_start(void);
int           logger_dma_stop(void);
int           logger_is_dma_on(void);
unsigned long x313_dma1_init(void);
void          logger_irq_cmd(int cmd);
