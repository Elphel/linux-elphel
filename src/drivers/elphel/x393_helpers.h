/** @file x393_helpers.h
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

#ifndef _X393_HELPERS_H
#define _X393_HELPERS_H

#include <asm/types.h>
#include "x393.h"

/** @brief The number of times to repeat register read sequence while waiting for
 * sequence number specified.
 */
#define REPEAT_READ               10

u32 get_rtc_usec(void);
u32 get_rtc_sec(void);

#endif /* _X393_HELPERS_H */
