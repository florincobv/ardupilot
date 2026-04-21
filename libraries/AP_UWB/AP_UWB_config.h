#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>

#ifndef AP_UWB_ENABLED
#define AP_UWB_ENABLED 1
#endif

#ifndef AP_UWB_Backend_DEFAULT_ENABLED
#define AP_UWB_Backend_DEFAULT_ENABLED AP_UWB_ENABLED
#endif

#ifndef AP_UWB_FLNC_ENABLED
#define AP_UWB_FLNC_ENABLED AP_UWB_ENABLED
#endif

#ifndef AP_UWB_SIM_ENABLED
#define AP_UWB_SIM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_UWB_Backend_DEFAULT_ENABLED)
#endif

#ifndef AP_UWB_MAX_INSTANCES
#define AP_UWB_MAX_INSTANCES 1
#endif

#ifndef AP_UWB_PRIMARY_INSTANCE
#define AP_UWB_PRIMARY_INSTANCE 0
#endif

#ifndef AP_UWB_TIMEOUT_MS
#define AP_UWB_TIMEOUT_MS 3000
#endif
