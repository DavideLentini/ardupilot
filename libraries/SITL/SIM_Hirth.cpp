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
  Simulator for the Hirth EFI system
*/

#include <AP_Math/AP_Math.h>

#include "SIM_Hirth.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

Hirth::Hirth() : SerialDevice::SerialDevice()
{
}

void Hirth::update(const struct sitl_input &input)
{
    // should use a dt here...
    throttle_out = 0.5*throttle_out + 0.5*throttle_in;

    update_receive(input);
    update_send();
}

void Hirth::handle_set_value()
{
    throttle_in = u.packed_setvalues.msg.throttle / 10.0;
    // gcs().send_text(MAV_SEVERITY_INFO, "throttle=%f", throttle_in);
}

void Hirth::update_receive(const struct sitl_input &input)
{
    const ssize_t n = read_from_autopilot((char*)&u.buffer[buflen], ARRAY_SIZE(u.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    if (buflen < 1) {
        return;
    }

    if (buflen < u.header.len) {
        return;
    }

    // there are no header bytes, so resync is going to be tough, and
    // given the weak checksum error-prone.  For now just discard.
    if (u.verify_checksum()) {
        switch (u.header.code) {
        case 0xC9:  // set value
            handle_set_value();
            break;
        case 0x04:  // request status 1
        case 0x0B:  // request status 2
        case 0x0D:  // request status 3
            active_request = u.header.code;
            break;
        default:
            abort();
        }
    }

    const uint8_t remaining = buflen - u.header.len;
    memmove(u.buffer, &u.buffer[buflen], remaining);
    buflen = remaining;
}

bool Hirth::send_status1()
{
    return true;
}
bool Hirth::send_status2()
{
    Status2 status2{};
    status2.fuel_consumption = 17;
    status2.throttle_position_pct = htole16(uint16_t(throttle_out * 10.0));

    PackedMessage<Status2> msg{0x0B, status2};
    write_to_autopilot((const char*)&msg, msg.len);
    return true;
}
bool Hirth::send_status3()
{
    return true;
}

void Hirth::update_send()
{
    switch (active_request) {
    case 0x04:  // request status 1
        if (!send_status1()) {
            return;
        }
        break;
    case 0x0B:  // request status 2
        if (!send_status2()) {
            return;
        }
        break;
    case 0x0D:  // request status 3
        if (!send_status3()) {
            return;
        }
        break;
    }
}
