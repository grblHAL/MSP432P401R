/*

  serial.h - low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#define BACKCHANNEL // comment out to use Launchpad Backchannel UART

#define XONOK (ASCII_XON|0x80)
#define XOFFOK (ASCII_XON|0x80)
#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300

void serialRegisterStreams (void);

/*EOF*/
