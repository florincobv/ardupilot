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

#include "AP_UWB_config.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "AP_UWB_Params.h"

// Maximum number of range finder instances available on this platform
#ifndef UWB_MAX_INSTANCES 
  #define UWB_MAX_INSTANCES 1
#endif

class AP_UWB_Backend;

class AP_UWB
{
    friend class AP_UWB_Backend;
    friend class AP_UWB_Backend_Serial;
    //UAVCAN drivers are initialised in the Backend, hence list of drivers is needed there.
    friend class AP_UWB_FLNC_UWB_2;
public:
    AP_UWB();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_UWB);

    // UWB driver types
    enum class Type {
        NONE   = 0,
#ifdef AP_UWB_FLNC_UWB_2_ENABLED
        FLNC_UWB_2 = 1,
#endif
#ifdef AP_UWB_SIM_ENABLED
        SIM = 100,
#endif
    };

    enum class Status {
        NotConnected = 0,
        NoData,
//        OutOfRangeHigh,
//        OutOfRangeLow,
        Good
    };

    static constexpr int8_t SIGNAL_QUALITY_MIN = 0;
    static constexpr int8_t SIGNAL_QUALITY_MAX = 100;
    static constexpr int8_t SIGNAL_QUALITY_UNKNOWN = -1;

    // The UWB_State structure is filled in by the backend driver
    struct UWB_State {
        uint8_t instance; // the instance number of this UWB

        uint16_t range_valid_count;
        uint64_t last_reading_ms;
        int8_t   signal_quality_pct;
        
        const struct AP_Param::GroupInfo *var_info;
        
        enum AP_UWB::Status status; // sensor status
    };

    static const struct AP_Param::GroupInfo *backend_var_info[UWB_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

//     void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /*
      Return the number of range finder instances. Note that if users
      sets up rangefinders with a gap in the types then this is the
      index of the maximum sensor ID plus one, so this gives the value
      that should be used when iterating over all sensors
    */
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available UWBs
    void init(const class AP_SerialManager& serial_manager);

    // update state of all UWB. Should be called at around
    // 10Hz from main loop
    void update(void);

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    float distance_orient(enum Rotation orientation) const;
    uint32_t last_reading_ms(enum Rotation orientation) const;

    // get temperature reading in C.  returns true on success and populates temp argument
    bool get_temp(enum Rotation orientation, float &temp) const;

    
    static AP_UWB *get_singleton(void) { return _singleton; }

protected:
    AP_UWB_Params params[UWB_MAX_INSTANCES];

private:
    static AP_UWB *_singleton;

    UWB_State state[UWB_MAX_INSTANCES];
    AP_UWB_Backend *drivers[UWB_MAX_INSTANCES];
    AP_HAL::UARTDriver *_port[UWB_MAX_INSTANCES];
    uint8_t num_instances;
    HAL_Semaphore detect_sem;
    
    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AP_UWB_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    AP_UWB_Backend* get_backend(uint8_t id) const;
    AP_UWB_Backend* find_instance(enum Rotation orientation) const;



};

namespace AP {
    AP_UWB &uwb();
};
