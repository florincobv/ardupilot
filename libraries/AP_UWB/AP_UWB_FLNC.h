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

#include "AP_UWB_Backend.h"

#if AP_UWB_FLNC_ENABLED

/**
 * @brief Backend driver for Florinco UWB module
 *
 * This UWB module also receives barometric pressure data from an
 * onboard barometer. This pressure data is directly fed into the
 * AP_Baro module via the handle_uwb_flnc() method. Further handling
 * of the (ground) pressure data is done in AP_Baro_FLNCUWB.
 */
class AP_UWB_FLNC : public AP_UWB_Backend
{

public:
    using AP_UWB_Backend::AP_UWB_Backend;

    void init_serial(uint8_t serial_instance) override;

    bool healthy() const override;

    void update() override;

    bool needs_serial(void) const override
    {
        return true;
    };

    static const struct AP_Param::GroupInfo var_info[];

    // TODO instance numbers should be added to the data messages,
    // such that we can also have multiple barometer drivers
    struct pressure_data_message_t {
        uint32_t timestamp_ms;
        float pressure;
        float temperature;
    };

    struct ground_pressure_data_message_t {
        uint32_t timestamp_ms;
        float ground_pressure;
        float ground_pressure_var;
    };

private:

    enum class PacketCommand : uint8_t {
        PRESSURE = 0x12,
        GROUND_PRESSURE = 0x34,
    };

    struct Packet {
        union {
            uint8_t buffer[257];
            struct {
                PacketCommand command;
                uint8_t datalength;
                uint8_t data[255];
            };
        };
    } __attribute__((packed));

    struct PressureData {
        float pressure_kPa;
    } __attribute__((packed));

    struct GroundPressureData {
        uint16_t id;
        float ground_pressure_kPa;
        float variance_pa;
    } __attribute__((packed));

    enum class PacketState : uint8_t {
        HEADER = 0,
        COMMAND,
        LENGTH,
        DATA,
        CRC8,  // CRC is already a global macro
    };

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t _baudrate = 0;

    PacketState _packet_state;
    Packet _current_packet;
    uint32_t _packet_arrival_time;
    uint8_t _data_index;
    uint32_t _last_update_ms = 0;
    bool _thread_started = false;

    void update_thread(void);
    bool read_serial(void);

    /**
     * @brief Compute the CRC8 checksum of a buffer
     *
     * @param buffer Pointer to the data buffer
     * @param length Length of the data buffer
     * @return Computed CRC8 value
     */
    uint8_t compute_crc(uint8_t* buffer, uint8_t length);

    void parse_packet();
};

#endif // AP_UWB_FLNC_ENABLED
