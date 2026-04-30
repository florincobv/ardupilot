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

#include "AP_Baro_FLNCUWB.h"

#if AP_BARO_FLNCUWB_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define TEMPERATURE_LIMIT_C 120

extern const AP_HAL::HAL& hal;

AP_Baro_FLNCUWB::AP_Baro_FLNCUWB(AP_Baro& baro)
    : AP_Baro_Backend(baro),
      _ground_pressure_kf(AP_BARO_FLNCUWB_NOISE, SSL_AIR_PRESSURE, 100000.0f),
      _count(0),
      _pressure_sum(0.0f),
      _temperature_sum(0.0f),
      _gnd_correction(0.0f)
{
    _instance = _frontend.register_sensor();
    GCS_SEND_INFO("Initialized AP_Baro_FLNCUWB instance %u", _instance);
}

void AP_Baro_FLNCUWB::handle_uwb_flnc(const AP_UWB_FLNC::pressure_data_message_t &pkt)
{
    if (!pressure_ok(pkt.pressure)) {
        return;
    }

    // TODO add temperature check

    WITH_SEMAPHORE(_sem);

    _pressure_sum += pkt.pressure;
    _temperature_sum += pkt.temperature;
    _count++;
}

void AP_Baro_FLNCUWB::handle_uwb_flnc(const AP_UWB_FLNC::ground_pressure_data_message_t &pkt)
{
    float variance = pkt.ground_pressure_var > 0 ? pkt.ground_pressure_var : AP_BARO_FLNCUWB_DEFAULT_VARIANCE;
    _ground_pressure_kf.update(pkt.ground_pressure, variance);
}

// transfer data to the frontend
void AP_Baro_FLNCUWB::update(void)
{
    if (_count == 0) {
        static uint32_t last_call_ms = 0;
        if (last_call_ms == 0 || AP_HAL::millis() - last_call_ms > 1000) {
            last_call_ms = AP_HAL::millis();
            GCS_SEND_WARNING("AP_Baro_FLNCUWB waiting for pressure data");
        }
        // Call UWB update (otherwise baro calibration will never pass).
        auto uwb = AP_UWB::get_singleton();
        if (uwb == nullptr || uwb->num_instances() == 0) {
            GCS_SEND_CRITICAL("AP_Baro_FLNCUWB doesn't have an UWB instance");
        }
        return;
    }

    WITH_SEMAPHORE(_sem);

    float pressure_diff = _pressure_sum/_count - _ground_pressure_kf.get_state();
    float pressure = SSL_AIR_PRESSURE + pressure_diff;
    _copy_to_frontend(_instance, pressure, _temperature_sum/_count);
    _pressure_sum = 0;
    _temperature_sum = 0;
    _count = 0;
}

#endif // AP_BARO_FLNCUWB_ENABLED
