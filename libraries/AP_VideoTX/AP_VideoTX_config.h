#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 1
#endif

#ifndef HAL_SMARTAUDIO_ENABLED
#define HAL_SMARTAUDIO_ENABLED (!HAL_MINIMIZE_FEATURES && AP_VIDEOTX_ENABLED)
#endif
