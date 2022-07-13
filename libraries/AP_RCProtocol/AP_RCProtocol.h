/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include "AP_RCProtocol_config.h"

#include <AP_HAL/AP_HAL.h>

class AP_RCProtocol_Backend;

#include "AP_RCProtocol_config.h"

enum class rcprotocol_t {
#if AP_RCPROTOCOL_PPMSUM_ENABLED || HAL_WITH_IO_MCU
    PPM        =  0,
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED || HAL_WITH_IO_MCU
    IBUS       =  1,
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED || HAL_WITH_IO_MCU
    SBUS       =  2,
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED || HAL_WITH_IO_MCU
    SBUS_NI    =  3,
#endif
#if AP_RCPROTOCOL_DSM_ENABLED || HAL_WITH_IO_MCU
    DSM        =  4,
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED || HAL_WITH_IO_MCU
    SUMD       =  5,
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED || HAL_WITH_IO_MCU
    SRXL       =  6,
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED || HAL_WITH_IO_MCU
    SRXL2      =  7,
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED || HAL_WITH_IO_MCU
    CRSF       =  8,
#endif
#if AP_RCPROTOCOL_ST24_ENABLED || HAL_WITH_IO_MCU
    ST24       =  9,
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED || HAL_WITH_IO_MCU
    FPORT      = 10,
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED || HAL_WITH_IO_MCU
    FPORT2     = 11,
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED || HAL_WITH_IO_MCU
    FASTSBUS   = 12,
#endif
    NONE    //last enum always is None
};

// return protocol name as a string
const char *rc_protocol_name_from_protocol(rcprotocol_t protocol);

#if AP_RCPROTOCOL_ENABLED

#include <AP_Common/AP_Common.h>

#define MAX_RCIN_CHANNELS 18
#define MIN_RCIN_CHANNELS  5

class AP_RCProtocol {
public:
    AP_RCProtocol() {}
    ~AP_RCProtocol();
    friend class AP_RCProtocol_Backend;

    void init();
    bool valid_serial_prot() const
    {
        return _valid_serial_prot;
    }
    bool should_search(uint32_t now_ms) const;
    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);
    bool process_byte(uint8_t byte, uint32_t baudrate);
    void process_handshake(uint32_t baudrate);
    void update(void);

    bool failsafe_active() const {
        return _failsafe_active;
    }
    void set_failsafe_active(bool active) {
        _failsafe_active = active;
    }

    void disable_for_pulses(enum rcprotocol_t protocol) {
        _disabled_for_pulses |= (1U<<(uint8_t)protocol);
    }

    // for protocols without strong CRCs we require 3 good frames to lock on
    bool requires_3_frames(enum rcprotocol_t p) {
        switch (p) {
#if AP_RCPROTOCOL_DSM_ENABLED
        case rcprotocol_t::DSM:
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
        case rcprotocol_t::FASTSBUS:
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
        case rcprotocol_t::SBUS:
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
        case rcprotocol_t::SBUS_NI:
#endif
#if AP_RCPROTOCOL_PPMSUM_ENABLED
        case rcprotocol_t::PPM:
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
        case rcprotocol_t::FPORT:
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
        case rcprotocol_t::FPORT2:
#endif
            return true;
#if AP_RCPROTOCOL_IBUS_ENABLED
        case rcprotocol_t::IBUS:
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED
        case rcprotocol_t::SUMD:
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
        case rcprotocol_t::SRXL:
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
        case rcprotocol_t::SRXL2:
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
        case rcprotocol_t::CRSF:
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
        case rcprotocol_t::ST24:
#endif
        case rcprotocol_t::NONE:
            return false;
        }
        return false;
    }

    uint8_t num_channels();
    uint16_t read(uint8_t chan);
    void read(uint16_t *pwm, uint8_t n);
    bool new_input();
    void start_bind(void);
    int16_t get_RSSI(void) const;
    int16_t get_rx_link_quality(void) const;

    // return protocol name as a string
    const char *protocol_name(void) const;

    // return detected protocol
    enum rcprotocol_t protocol_detected(void) const {
        return _detected_protocol;
    }

    // add a UART for RCIN
    void add_uart(AP_HAL::UARTDriver* uart);
    bool has_uart() const { return added.uart != nullptr; }

#ifdef IOMCU_FW
    // set allowed RC protocols
    void set_rc_protocols(uint32_t mask) {
        rc_protocols_mask = mask;
    }
#endif

    class SerialConfig {
    public:
        void apply_to_uart(AP_HAL::UARTDriver *uart) const;

        uint32_t baud;
        uint8_t parity;
        uint8_t stop_bits;
        bool invert_rx;
    };

    // return true if we are decoding a byte stream, instead of pulses
    bool using_uart(void) const {
        return _detected_with_bytes;
    }

private:
    void check_added_uart(void);

    // return true if a specific protocol is enabled
    bool protocol_enabled(enum rcprotocol_t protocol) const;

    enum rcprotocol_t _detected_protocol = rcprotocol_t::NONE;
    uint16_t _disabled_for_pulses;
    bool _detected_with_bytes;
    AP_RCProtocol_Backend *backend[uint8_t(rcprotocol_t::NONE)];
    bool _new_input;
    uint32_t _last_input_ms;
    bool _failsafe_active;
    bool _valid_serial_prot;

    // optional additional uart
    struct {
        AP_HAL::UARTDriver *uart;
        bool opened;
        uint32_t last_config_change_ms;
        uint8_t config_num;
    } added;

    // allowed RC protocols mask (first bit means "all")
    uint32_t rc_protocols_mask;
};

namespace AP {
    AP_RCProtocol &RC();
};

#include "AP_RCProtocol_Backend.h"

#endif  // AP_RCPROTOCOL_ENABLED
