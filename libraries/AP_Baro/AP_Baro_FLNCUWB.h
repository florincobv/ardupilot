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

#include "AP_Baro_Backend.h"

#if AP_BARO_FLNCUWB_ENABLED

#include <AP_UWB/AP_UWB_FLNC.h>

// TODO Make these parameters configurable
#ifndef AP_BARO_FLNCUWB_DEFAULT_VARIANCE
#define AP_BARO_FLNCUWB_DEFAULT_VARIANCE 90.f
#endif
#ifndef AP_BARO_FLNCUWB_NOISE
#define AP_BARO_FLNCUWB_NOISE 10.f
#endif
#ifndef AP_BARO_FLNCUWB_DRIFT_INTERVAL_MS
#define AP_BARO_FLNCUWB_DRIFT_INTERVAL_MS 0  // set to 0 to continuously update correction
#endif

class AP_Baro_FLNCUWB : public AP_Baro_Backend
{
public:
    AP_Baro_FLNCUWB(AP_Baro& baro);

    void update() override;

    void handle_uwb_flnc(const AP_UWB_FLNC::pressure_data_message_t &pkt) override;
    void handle_uwb_flnc(const AP_UWB_FLNC::ground_pressure_data_message_t &pkt) override;

private:
    class KalmanFilter1D
    {
    public:
        KalmanFilter1D(float process_noise, float initial_value, float initial_variance)
            : _Q(process_noise), _x(initial_value), _P(initial_variance) {}

        float update(float measurement, float variance)
        {
            _P += _Q;
            float K = _P / (_P + variance);
            _x += K * (measurement - _x);
            _P *= (1 - K);
            return _x;
        }

        float get_state() const
        {
            return _x;
        }
        float get_variance() const
        {
            return _P;
        }
    private:
        float _Q;
        float _x;
        float _P;
    };

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _instance;

    uint32_t _count;
    float _pressure_sum;
    float _temperature_sum;
    float _gnd_correction;

    KalmanFilter1D _ground_pressure_kf;
    bool _ground_pressure_updated = false;
    uint32_t _last_correction_update_ms = 0;
};

#endif // AP_BARO_FLNCUWB_ENABLED
