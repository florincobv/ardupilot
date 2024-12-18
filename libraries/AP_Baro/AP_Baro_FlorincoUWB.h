#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_FLNCUWB_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

class AP_Baro_FLNCUWB : public AP_Baro_Backend {
public:
    static AP_Baro_Backend* probe(AP_Baro &baro);

    AP_Baro_FLNCUWB(AP_Baro& baro);
    
    /* AP_Baro public interface: */
    void update() override;

    void set_data(float pressure, float sigma) override;

protected:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t instance;

    uint32_t count;
    uint8_t err_count;
    float pressure_sum;
    float temperature_sum;
    float last_temperature;
    bool pending_reset;
private:
    uint8_t _instance;
};

#endif  // AP_BARO_FLNCUWB_ENABLED
