//----------------------------------------------------------------------------------------//
//
//  Debug putstring, extension to dbg_putchar (by Dimitar Dimitrov) for software-serial
//    logging of formatted strings.
//
//    Copyright 2021 Arran Derbyshire
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//    arran@archaea.co.uk
//
//----------------------------------------------------------------------------------------//

#ifndef dbg_putstring_h
#define dbg_putstring_h

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define DEBUG_MAX_STR_LEN 48    // maximum formatted string length
                                // (including the terminating '\0')

#if DEBUG_LOG

extern void dbg_putstring1(const char string[], uint16_t a);

#else

#define dbg_putstring1(A,B)

#endif // DEBUG_LOG

#endif // dbg_putstring_h

