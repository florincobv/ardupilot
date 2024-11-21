#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>

#define UWB_MAX_INSTANCES 1

#ifndef AP_UWB_ENABLED
#define AP_UWB_ENABLED 1
#endif

#ifndef AP_UWB_Backend_DEFAULT_ENABLED
#define AP_UWB_Backend_DEFAULT_ENABLED AP_UWB_ENABLED
#endif

#ifndef AP_UWB_SIM_ENABLED
#define AP_UWB_SIM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_UWB_Backend_DEFAULT_ENABLED)
#endif

#ifndef AP_UWB_FLNC_UWB_2_ENABLED
#define AP_UWB_FLNC_UWB_2_ENABLED AP_UWB_Backend_DEFAULT_ENABLED
#endif
