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
#include "AP_UWB_Backend.h"
#include "AP_UWB_Backend_Serial.h"
#include "AP_UWB_FLNC_UWB_2.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_UWB::AP_UWB()
{
    // AP_Param::setup_object_defaults(this, params);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_UWB must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the UWB class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  UWBs.
 */
void AP_UWB::init(const AP_SerialManager& serial_manager)
{
    if (num_instances != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

    // search for serial ports with FLNC UWB
    uint8_t uart_idx = 0;
    for (uint8_t i=0, serial_instance = 0; i<UWB_MAX_INSTANCES; i++) {
        _port[i] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FLNC_UWB, uart_idx);
        state[i].type = Type::FLNC_UWB_2; // TODO State heeft geen type en hoort die ook niet te hebben geloof ik
        uart_idx++;

        state[i].instance = i;

        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN UWB may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
        // initialize signal_quality_pct for drivers that don't handle it.
        state[i].signal_quality_pct = SIGNAL_QUALITY_UNKNOWN;
    }
    // AP::logger().Write_Message("UWB Init Done");
    gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB INIT DONE(%i)", num_instances);
}

/*
  update UWB state for all instances. This should be called at
  around 140Hz by main loop
 */
void AP_UWB::update(void)
{    
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB Top level Update");
    for (uint8_t i=0; i<num_instances; i++) {
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB Top level Update(%i)", i);
        if (drivers[i] != nullptr) {
            if (state[i].type == Type::NONE) {
                // allow user to disable a UWB at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB Top level Update(Driver)");
            drivers[i]->update();
        }
    }
#if HAL_LOGGING_ENABLED
    // Log_RFND();
#endif
}

bool AP_UWB::_add_backend(AP_UWB_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= UWB_MAX_INSTANCES) {
        AP_HAL::panic("Too many UWB backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a UWB is connected. 
 */
void AP_UWB::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    // AP_UWB_Backend* (*serial_create_fn)(AP_UWB::UWB_State&, AP_UWB_Params&) = nullptr;

    const Type _type = (Type)state[instance].type;
    switch (_type) {
    case Type::FLNC_UWB_2:
        _add_backend(new AP_UWB_FLNC_UWB_2(state[instance], _port[instance], params[instance]), instance);
        break;

    case Type::SIM:
        break;
    case Type::NONE:
        break;
    }

    // if (serial_create_fn != nullptr) {
    //     if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_FLNC_UWB, serial_instance)) {
    //         auto *b = serial_create_fn(state[instance], params[instance]);
    //         if (b != nullptr) {
    //             _add_backend(b, instance, serial_instance++);
    //         }
    //     }
    // }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].params) {
        // AP_Param::load_object_from_eeprom(drivers[instance], state[instance].params->var_info);

        // param count could have changed
        AP_Param::invalidate_count();
    }

}

AP_UWB_Backend* AP_UWB::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};


float AP_UWB::distance_orient(enum Rotation orientation) const
{
    AP_UWB_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance();
}

uint32_t AP_UWB::last_reading_ms(enum Rotation orientation) const
{
    AP_UWB_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

// get temperature reading in C.  returns true on success and populates temp argument
bool AP_UWB::get_temp(enum Rotation orientation, float &temp) const
{
    AP_UWB_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_temp(temp);
}

AP_UWB_Backend* AP_UWB::find_instance(enum Rotation orientation) const
{
    return nullptr;
}

AP_UWB* AP_UWB::_singleton;

namespace AP {

AP_UWB &uwb()
{
    return *AP_UWB::get_singleton();
}

}

