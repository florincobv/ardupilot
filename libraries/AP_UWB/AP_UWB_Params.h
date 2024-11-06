#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_UWB_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_UWB_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_UWB_Params);

    AP_Vector3f last_received_anchorPos;
    AP_Int32    last_anchor_id;
    AP_Float    last_distance;
    
    AP_Int8  type;
};
