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
/*
  DPS280 barometer driver
 */

#include "AP_Baro_FlorincoUWB.h"

#if AP_BARO_FLNCUWB_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <utility>
#include <stdio.h>
#include <AP_Math/definitions.h>
#include <AP_BoardConfig/AP_BoardConfig.h> ///////////////////////////////////////////////////////////////////////////////////

extern const AP_HAL::HAL& hal;

#define TEMPERATURE_LIMIT_C 120

AP_Baro_FLNCUWB::AP_Baro_FLNCUWB(AP_Baro& baro)
    : AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

AP_Baro_Backend* AP_Baro_FLNCUWB::probe(AP_Baro& baro)
{
    return new AP_Baro_FLNCUWB(baro);
}

//  accumulate a new sensor reading
void AP_Baro_FLNCUWB::set_data(float pressure, float variance)
{
    // TODO add temperature
    float temperature = 20;

    //float cur_pressure, cur_temperature;
    last_temperature = temperature;
    
    if (!pressure_ok(pressure)) 
    {
        return;
    }

    if (fabsf(last_temperature) > TEMPERATURE_LIMIT_C) 
    {
        err_count++;
        if (err_count > 16)
        {
            // TODO reset something
        }
    }
    else
    {
        err_count = 0;
    }
    
    WITH_SEMAPHORE(_sem);

    pressure_sum += pressure;
    temperature_sum += temperature;
    count++;
}

// transfer data to the frontend
void AP_Baro_FLNCUWB::update(void)
{
    if (count == 0) {
        return;
    }
        
    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count); // Pressure in pascal
    pressure_sum = 0;
    temperature_sum = 0;
    count=0;
}

#endif  // AP_BARO_FLNCUWB_ENABLED
