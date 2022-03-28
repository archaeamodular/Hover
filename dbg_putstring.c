//----------------------------------------------------------------------------------------//
//
//  Debug putstring, extension to dbg_putchar (by Dimitar Dimitrov) for software-serial
//    logging of strings.
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

#include <avr/pgmspace.h>

#include "dbg_putstring.h"
#include "dbg_putchar.h"

#if DEBUG_LOG

static char dbg_temp_buf[DEBUG_MAX_STR_LEN] = {0};

//----------------------------------------------------------------------------------------//
// dbg_putstring:
// Output a string using dbg_putchar to the software-serial port.
// This is a helper function, do not call this directly, use dbg_putstring1 instead
// otherwise you could quickly run out of SRAM on small devices!
//----------------------------------------------------------------------------------------//
static void dbg_putstring(const char string[])
{
    int i=0;
    while (string[i] != '\0')
    {
        dbg_putchar(string[i]);
        i++;
    }
}

//----------------------------------------------------------------------------------------//
// dbg_putstring1:
// Output a debug logging formatted string to the software-serial port with one argument
// (a 16-bit word) from flash program memory.
// Use with PSTR to define the string in flash program memory, e.g.:
//      dbg_putstring1(PSTR("Hello world!"),0);
//
// E.g. to log a string showing the value of one variable:
//      dbg_putstring1(PSTR("variable=%d"),variable);
//----------------------------------------------------------------------------------------//
void dbg_putstring1(const char string[], uint16_t a)
{
    // format the string from flash program memory to a buffer
    snprintf_P(dbg_temp_buf, DEBUG_MAX_STR_LEN, string, a);
    // output the formatted string to the serial port.
    dbg_putstring(dbg_temp_buf);
}

#endif // DEBUG_LOG

