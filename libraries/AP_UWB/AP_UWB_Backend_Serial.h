#pragma once

#include "AP_UWB_Backend.h"

class AP_UWB_Backend_Serial : public AP_UWB_Backend
{
public:
    // constructor
    AP_UWB_Backend_Serial(AP_UWB::UWB_State &_state, AP_HAL::UARTDriver *_port,
                                  AP_UWB_Params &_params);

    void init_serial(uint8_t serial_instance) override;

    // update state; not all backends call this!
    virtual bool update() override;

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;


    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_reading(float &reading_m) = 0;

    // returns 0-100 or -1. This virtual method is for
    // serial drivers and is a companion to the previous method get_reading().
    // Like get_reading() this method is called in the base-class update() method.
    virtual int8_t get_signal_quality_pct() const WARN_IF_UNUSED
    { return AP_UWB::SIGNAL_QUALITY_UNKNOWN; }

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }
};