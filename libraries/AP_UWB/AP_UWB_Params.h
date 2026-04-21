#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_UWB_Params
{

public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_UWB_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_UWB_Params);

    // UWB sensor types
    enum class Type : uint8_t {
        NONE = 0,
        FLNC,
        // SITL = 100,
    };

    AP_Enum<Type> type;
};
