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
#pragma once

#include "AP_UWB_config.h"

#if AP_UWB_ENABLED

#include "AP_UWB_Params.h"
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class AP_UWB_Backend;
class AP_UWB_FLNC;

class AP_UWB
{
    friend class AP_UWB_Backend;
    friend class AP_UWB_FLNC;

public:
    AP_UWB();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_UWB);

    static AP_UWB *get_singleton()
    {
        return _singleton;
    }

    // return the number of UWB sensors instances
    uint8_t num_instances(void) const
    {
        return _num_instances;
    }

    // detect and initialise any available UWBs
    void init(void);

    // update state of all UWBs
    void update(void);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy(const uint8_t instance = AP_UWB_PRIMARY_INSTANCE) const;

    // accessors to params
    AP_UWB_Params::Type get_driver_type(const uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *backend_var_info[AP_UWB_MAX_INSTANCES];

protected:
    AP_UWB_Params _params[AP_UWB_MAX_INSTANCES];

private:
    static AP_UWB *_singleton;

    // The UWB_State structure is filled in by the backend driver
    struct UWB_State {
        uint8_t instance; // the instance number of this UWB

        const struct AP_Param::GroupInfo *var_info;
    };

    UWB_State _state[AP_UWB_MAX_INSTANCES];
    AP_UWB_Backend *_drivers[AP_UWB_MAX_INSTANCES];

    uint8_t _num_instances;

    // Parameters
    AP_Int8 _log_flag; // log_flag: true if we should log all sensors data
};

namespace AP
{
AP_UWB *uwb();
};

#endif // AP_UWB_ENABLED
