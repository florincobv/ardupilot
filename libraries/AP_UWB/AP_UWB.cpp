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

#include "AP_UWB.h"

#if AP_UWB_ENABLED

#include "AP_UWB_Backend.h"
#include "AP_UWB_FLNC.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_UWB::var_info[] = {

    // SKIP INDEX 0

    // @Param: _LOG
    // @DisplayName: Logging
    // @Description: Enables UWB sensor logging
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_LOG", 1, AP_UWB, _log_flag, 0),

    // SKIP Index 2-9 to be for parameters that apply to every sensor

    // @Group: 1_
    // @Path: AP_UWB_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 10, AP_UWB, AP_UWB_Params),

    // @Group: 1_
    // @Path: AP_UWB_FLNC.cpp
    AP_SUBGROUPVARPTR(_drivers[0], "1_", 19, AP_UWB, backend_var_info[0]),

    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_UWB::backend_var_info[AP_UWB_MAX_INSTANCES];

AP_UWB::AP_UWB()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_UWB must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

// init - instantiate the UWBs
void AP_UWB::init()
{
    GCS_SEND_INFO("UWB init");
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // create each instance
    uint8_t serial_instance = 0;  // Track serial port allocation
    for (uint8_t instance = 0; instance < AP_UWB_MAX_INSTANCES; instance++) {
        _state[instance].instance = instance;

        switch (get_driver_type(instance)) {
#if AP_UWB_FLNC_ENABLED
        case AP_UWB_Params::Type::FLNC:
            _drivers[instance] = new AP_UWB_FLNC(*this, _state[instance], _params[instance]);
            break;
#endif
        case AP_UWB_Params::Type::NONE:
        default:
            break;
        }

        // call init function for each backend
        if (_drivers[instance] != nullptr) {
            if (_state[instance].var_info != nullptr) {
                // Load backend specific params
                backend_var_info[instance] = _state[instance].var_info;
                AP_Param::load_object_from_eeprom(_drivers[instance], backend_var_info[instance]);
            }
            if (_drivers[instance]->needs_serial()) {
                _drivers[instance]->init_serial(serial_instance);
                serial_instance++;
            } else {
                _drivers[instance]->init();
            }
            // _num_instances is actually the index for looping over instances
            // the user may have UWB_TYPE=0 and UWB2_TYPE=1, in which case
            // there will be a gap, but as we always check for _drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;
        }
    }

    if (_num_instances > 0) {
        // param count could have changed
        AP_Param::invalidate_count();
    }
}

void AP_UWB::update(void)
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (_drivers[i] != nullptr && get_driver_type(i) != AP_UWB_Params::Type::NONE) {
            _drivers[i]->update();
#if HAL_LOGGING_ENABLED
            const AP_Logger *logger = AP_Logger::get_singleton();
            if (logger != nullptr && _log_flag) {
                _drivers[i]->Log_Write_UWB();
            }
#endif
        }
    }
}

bool AP_UWB::healthy(const uint8_t instance) const
{
    return instance < _num_instances && _drivers[instance] != nullptr && _drivers[instance]->healthy();
}

AP_UWB_Params::Type AP_UWB::get_driver_type(const uint8_t instance) const
{
    if (instance >= AP_UWB_MAX_INSTANCES) {
        return AP_UWB_Params::Type::NONE;
    }
    return (AP_UWB_Params::Type)_params[instance].type.get();
}

AP_UWB* AP_UWB::_singleton;

namespace AP
{
AP_UWB *uwb()
{
    return AP_UWB::get_singleton();
}
};

#endif // AP_UWB_ENABLED
