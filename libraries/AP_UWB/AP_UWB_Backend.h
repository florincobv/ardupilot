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

#include "AP_UWB.h"

#if AP_UWB_ENABLED

#include <AP_HAL/AP_HAL.h>

class AP_UWB_Backend
{
public:
    // constructor
    AP_UWB_Backend(AP_UWB &front, AP_UWB::UWB_State &state, AP_UWB_Params &params);

    // initialise
    virtual void init() {};
    virtual void init_serial(uint8_t serial_instance) {};

    // update the latest measurement
    virtual void update() = 0;

    // do we have a valid measurement reading?
    virtual bool healthy(void) const;

    // do we need a serial port? If so init_serial will be called instead of init.
    virtual bool needs_serial(void) const
    {
        return false;
    };

    // logging
    void Log_Write_UWB() const;

protected:

    AP_UWB &_front; // reference to front-end
    AP_UWB::UWB_State &_state; // reference to this instance's state (held in the front-end)
    AP_UWB_Params &_params; // reference to this instance's parameters (held in the front-end)

private:
    HAL_Semaphore _sem; // used to copy from backend to frontend
};

#endif // AP_UWB_ENABLED
