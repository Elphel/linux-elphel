/**
 * @file klogger_393.c
 * @brief Header file for klogger_393.c
 * @copyright Copyright (C) 2016 Elphel, Inc.
 * @par <b>License</b>
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
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
#ifndef KLOGGER_393_H
#define KLOGGER_393_H
//int print_klog393(const int mode, const char *fmt, ...);
int print_klog393(const int mode, const char *file, const char *function, const int line, const char *fmt, ...);

int klog393_puts(const char * str);
int klog393_ts(const char * str);

#endif
