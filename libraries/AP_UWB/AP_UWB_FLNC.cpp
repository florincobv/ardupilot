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

#include "AP_UWB_FLNC.h"

#if AP_UWB_FLNC_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

void AP_UWB_FLNC::init_serial(uint8_t serial_instance)
{
    GCS_SEND_INFO("INITILIAZING UWB SERIAL");
    const AP_SerialManager &serialmanager = AP::serialmanager();
    uart = serialmanager.find_serial(AP_SerialManager::SerialProtocol_FLNC_UWB, serial_instance);
    if (uart == nullptr) {
        return;
    }
    GCS_SEND_INFO("UWB SERIAL INITIALIZED SUCCESSFULLY");
    uart->begin(serialmanager.find_baudrate(AP_SerialManager::SerialProtocol_FLNC_UWB, serial_instance));
}

// returns true if a UWB has been recently updated
bool AP_UWB_FLNC::healthy() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AP_UWB_TIMEOUT_MS);
}

// update the state of the sensor
void AP_UWB_FLNC::update(void)
{
    if (!healthy()) {
        GCS_SEND_WARNING_THROTTLE(1000, "UWB FLNC unhealthy");
    }
    if (uart == nullptr) {
        GCS_SEND_CRITICAL("UWB FLNC uart nullptr");
        return;
    }

    // read any available characters
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t byte;
        if (!uart->read(byte)) {
            break;
        }

        switch (_packet_state) {

        case PacketState::HEADER:
            if (byte == 0xAA) {
                _packet_arrival_time = AP_HAL::millis();
                _packet_state = PacketState::COMMAND;
            }
            break;

        case PacketState::COMMAND:
            _packet_state = PacketState::LENGTH;
            _current_packet.command = static_cast<PacketCommand>(byte);
            break;

        case PacketState::LENGTH:
            _current_packet.datalength = byte;
            _packet_state = (byte > 0) ? PacketState::DATA : PacketState::CRC8;
            _data_index = 0;
            break;

        case PacketState::DATA:
            _current_packet.data[_data_index++] = byte;
            if (_data_index >= _current_packet.datalength) {
                _packet_state = PacketState::CRC8;
            }
            break;

        case PacketState::CRC8:
            // CRC is computed over the command, length, and data
            if (compute_crc(_current_packet.buffer, _current_packet.datalength + 2) == byte) {
                parse_packet();
            } else {
                GCS_SEND_WARNING("UWB FLNC CRC error");
            }
            _packet_state = PacketState::HEADER;
            break;

        default:
            _packet_state = PacketState::HEADER;
            break;
        }
    }
}

// Compute the CRC8 checksum of a buffer
uint8_t AP_UWB_FLNC::compute_crc(uint8_t* buffer, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc += buffer[i];
    }
    return crc;
}

// parse buffer
void AP_UWB_FLNC::parse_packet()
{
    bool parsed = false;
    switch (_current_packet.command) {

    case PacketCommand::PRESSURE: {
        PressureData *data = reinterpret_cast<PressureData *>(_current_packet.data);
        pressure_data_message_t pkt;
        pkt.timestamp_ms = _packet_arrival_time;
        pkt.pressure = data->pressure_kPa * 1000.0f;
        pkt.temperature = 20.0f; // TODO no temperature in FLNC
        AP_Baro::get_singleton()->handle_uwb_flnc(pkt);
        parsed = true;
        break;
    }

    case PacketCommand::GROUND_PRESSURE: {
        GroundPressureData *data = reinterpret_cast<GroundPressureData *>(_current_packet.data);
        ground_pressure_data_message_t pkt;
        pkt.timestamp_ms = _packet_arrival_time;
        pkt.ground_pressure = data->ground_pressure_kPa * 1000.0f;
        pkt.ground_pressure_var = data->variance_pa;
        AP_Baro::get_singleton()->handle_uwb_flnc(pkt);
        parsed = true;
        break;
    }

    default:
        // unrecognised message id
        break;
    }

    // record success
    if (parsed) {
        _last_update_ms = AP_HAL::millis();
    }
}

#endif // AP_UWB_FLNC_ENABLED
