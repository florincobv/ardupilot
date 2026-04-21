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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

AP_UWB_Backend::AP_UWB_Backend(AP_UWB &front,
                               AP_UWB::UWB_State &state,
                               AP_UWB_Params &params):
    _front(front),
    _state(state),
    _params(params)
{
}

// returns true if a UWB has been recently updated
bool AP_UWB_Backend::healthy(void) const
{
    return false;
}

void AP_UWB_Backend::Log_Write_UWB() const
{
}

#endif // AP_UWB_ENABLED
