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

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --uartF=sim:hirth --speedup=1 --console

param set SERIAL5_PROTOCOL 24
param set EFI_TYPE 7
param fetch
reboot

graph EFI_STATUS.throttle_position EFI_STATUS.throttle_out

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

namespace SITL {

class Hirth : public SerialDevice {
public:

    Hirth();

    void update(const struct sitl_input &input);

private:

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(uint8_t _code, T _msg) :
            code{_code},
            msg{_msg}
            {
                len = sizeof(_msg) + 3; // len=1 + code=1+ checksum=1
                update_checksum();
            }
        uint8_t len;
        uint8_t code;
        T msg;
        uint8_t checksum;

        uint8_t calculate_checksum() const WARN_IF_UNUSED {
            return checksum_sum_buffer(0, (const uint8_t*)this, len-1);
        }

        // bizarrely the checksum algorithm is different ATM between
        // sending/receiving messages - that's the 256 - here...
        void update_checksum() {
            checksum = 256 - calculate_checksum();
            // gcs().send_text(MAV_SEVERITY_INFO, "SITL: checksum: %u", (unsigned)checksum);
        }
    };

    // convenience class to be able to extract the len and code only;
    // not a real message.
    class PACKED HeaderOnly {
    public:
    };

    // a request is entirely defined by its code:
    class PACKED Request {
    public:
    };

    class PACKED Status2 {
    public:
        uint8_t unknown[52];
        uint16_t fuel_consumption;
        uint8_t unknown2[8];
        uint16_t throttle_position_pct;
        uint8_t unknown3[34];
    };
    static_assert(sizeof(Status2) == 98, "Status2 correct size");

    class PACKED SetValues {
    public:
        uint16_t throttle;
        uint8_t unknown[18];
    };
    static_assert(sizeof(SetValues) == 20, "SetValues correct size");


    union MessageUnion {
        uint8_t buffer[255];

        MessageUnion() { }
        PackedMessage<HeaderOnly> header;
        PackedMessage<Request> packed_request;
        PackedMessage<SetValues> packed_setvalues;

        uint8_t verify_checksum() const {
            return checksum_sum_buffer(0, buffer, buffer[0]) == buffer[buffer[0]];
        }
        uint8_t calculate_checksum() const {
            return checksum_sum_buffer(0, buffer, buffer[0]);
        }
    } u;
    uint8_t buflen;

    void update_send();
    void update_receive(const struct sitl_input &input);

    void handle_set_value();

    uint8_t active_request;

    bool send_status1();
    bool send_status2();
    bool send_status3();

    // values coming in from autopilot:
    float throttle_in;

    // output values to autopilot:
    float throttle_out;
};

}
