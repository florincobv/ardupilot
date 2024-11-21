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

#include <AP_HAL/AP_HAL.h>
#include "AP_UWB_Backend_Serial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the UWB. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the UWB
*/
AP_UWB_Backend_Serial::AP_UWB_Backend_Serial(
    AP_UWB::UWB_State &_state,
    AP_HAL::UARTDriver *_port,
    AP_UWB_Params &_params) :
    AP_UWB_Backend(_state, _params)
{
    uart = _port;
}

void AP_UWB_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_FLNC_UWB, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AP_UWB_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_FLNC_UWB, serial_instance);
}

/*
   update the state of the sensor
*/
bool AP_UWB_Backend_Serial::update()
{
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB BES UPDATE");
    if (get_reading(state.last_twr_data.distance)) {
        state.signal_quality_pct = get_signal_quality_pct();
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        set_status(AP_UWB::Status::NoData);
    }
    return true;
}
