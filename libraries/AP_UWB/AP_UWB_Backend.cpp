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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_UWB.h"
#include "AP_UWB_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_UWB_Backend::AP_UWB_Backend(AP_UWB::UWB_State &_state, AP_UWB_Params &_params) :
        state(_state),
		params(_params)
{
    _backend_type = type();
}

AP_UWB::Status AP_UWB_Backend::status() const {
    if (type() == AP_UWB::Type::NONE) {
        // turned off at runtime?
        return AP_UWB::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_UWB_Backend::has_data() const {
    return ((state.status != AP_UWB::Status::NotConnected) &&
            (state.status != AP_UWB::Status::NoData));
}

// update status based on distance measurement
void AP_UWB_Backend::update_status()
{
    // TODO sanity check?
    set_status(AP_UWB::Status::Good);
}

// set status and update valid count
void AP_UWB_Backend::set_status(AP_UWB::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == AP_UWB::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

