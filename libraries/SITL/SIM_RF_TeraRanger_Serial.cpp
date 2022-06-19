/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Base class for simulator for the TeraRanger Serial RangeFinders
*/

#include "SIM_RF_TeraRanger_Serial.h"

using namespace SITL;

uint32_t RF_TeraRanger_Serial::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    buffer[0] = 0x57;
    buffer[1] = (alt_cm & 0xff) * 10; //MSB mm
    buffer[2] = (alt_cm >> 8) * 10; //LSB mm
    buffer[3] = 0xC0; //full strength, no reading error, no overtemp
    
    // calculate CRC8:
    buffer[4] = crc_crc8((const uint8_t *)&buffer,4);;

    return 5;
}

