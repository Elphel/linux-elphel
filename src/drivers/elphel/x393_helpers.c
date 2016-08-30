/** @file x393_helpers.c
 *
 * @brief Helper functions for various routines form x393.h which require several actions to get
 * reliable result.
 *
 * @copyright Copyright (C) 2016 Elphel, Inc
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

#include <linux/module.h>
#include <stddef.h>
#include "x393_helpers.h"

/**
 * @brief Read RTC microsecond counter.
 * @return Current value of microsecond counter or 0 in case read sequence was
 * not successful.
 */
u32 get_rtc_usec(void)
{
	x393_rtc_status_t stat;
	x393_status_ctrl_t stat_ctrl;
	x393_rtc_usec_t   usec;
	unsigned int i;

	stat = x393_rtc_status();
	stat_ctrl.d32 = 0;
	stat_ctrl.mode = 1;
	stat_ctrl.seq_num = stat.seq_num + 1;
	set_x393_rtc_set_status(stat_ctrl);
	for (i = 0; i < REPEAT_READ; i++) {
		stat = x393_rtc_status();
		if (stat.seq_num == stat_ctrl.seq_num) {
			usec = x393_rtc_status_usec();
			return usec.usec;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(get_rtc_usec);

/**
 * @brief Read RTC second counter.
 * @return Current value of second counter or 0 in case read sequence was
 * not successful.
 */
u32 get_rtc_sec(void)
{
	x393_rtc_status_t stat;
	x393_status_ctrl_t stat_ctrl;
	x393_rtc_sec_t   sec;
	unsigned int i;

	stat = x393_rtc_status();
	stat_ctrl.d32 = 0;
	stat_ctrl.mode = 1;
	stat_ctrl.seq_num = stat.seq_num + 1;
	set_x393_rtc_set_status(stat_ctrl);
	for (i = 0; i < REPEAT_READ; i++) {
		stat = x393_rtc_status();
		if (stat.seq_num == stat_ctrl.seq_num) {
			sec = x393_rtc_status_sec();
			return sec.sec;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(get_rtc_sec);
