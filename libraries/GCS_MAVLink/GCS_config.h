#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

// BATTERY2 is slated to be removed:
#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 1
#endif

// AUTOPILOT_VERSION_REQUEST is slated to be removed; an instance of
// AUTOPILOT_VERSION can be requested with MAV_CMD_REQUEST_MESSAGE,
// which gets you an ACK/NACK
#ifndef AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
#define AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED 1
#endif

// handling of MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES is slated to be
// removed; the message can be requested with MAV_CMD_REQUEST_MESSAGE
#ifndef AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
#define AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED 1
#endif
