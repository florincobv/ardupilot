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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_UWB.h"

class AP_UWB_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_UWB_Backend(AP_UWB::UWB_State &_state, AP_UWB_Params &_params);

    // we declare a virtual destructor so that UWB drivers can
    // override with a custom destructor if need be
    virtual ~AP_UWB_Backend(void) {}

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    float distance() const { return state.distance_m; }
    AP_UWB::Status status() const;
    AP_UWB::Type type() const { return (AP_UWB::Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t range_valid_count() const { return state.range_valid_count; }

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

    // get temperature reading in C.  returns true on success and populates temp argument
    virtual bool get_temp(float &temp) const { return false; }

    // return the actual type of the UWB, as opposed to the
    // parameter value which may be changed at runtime.
    AP_UWB::Type allocated_type() const { return _backend_type; }

protected:

    // Handle all serial data.
    // After this function the newest set will be available to be read by others
    void update_status();

    // set status and update valid_count
    void set_status(AP_UWB::Status status);

    AP_UWB::UWB_State &state;
    AP_UWB_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    AP_UWB::Type _backend_type;

};
