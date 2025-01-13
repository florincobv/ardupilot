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
#include "AP_UWB_FLNC_UWB_2.h"
#include "AP_UWB_Common.hpp"
#include <GCS_MAVLink/GCS.h>

#ifdef AP_UWB_FLNC_UWB_2_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <cstring>

extern const AP_HAL::HAL& hal;

#define FRAME_HEADER 0x54
#define FRAME_LENGTH 5
#define DIST_MAX_CM 3000
#define OUT_OF_RANGE_ADD_CM 1000
#define STATUS_MASK 0x1F
#define DISTANCE_ERROR 0x0001


AP_UWB_FLNC_UWB_2::AP_UWB_FLNC_UWB_2(AP_UWB::UWB_State &_state, AP_HAL::UARTDriver *_port, AP_UWB_Params &_params)
    : AP_UWB_Backend_Serial(_state, _port, _params)
{
    rxState     = UWB_SER_WAIT_START;
    linebuf_len = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "UWB Create Backend");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UWB_FLNC_UWB_2::update_thread, void), "UWB", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) 
    {
        AP_HAL::panic("Florinco UWB Failed to start UWB update thread");
    }

}

AP_UWB_FLNC_UWB_2::~AP_UWB_FLNC_UWB_2()
{

}

void AP_UWB_FLNC_UWB_2::update_thread()
{
    // // Open port in the thread
    // uart->begin(baudrate, 1024, 512);
    gcs().send_text(MAV_SEVERITY_INFO, "Start UWB Update thread");

    uint32_t lastMessageTime = AP_HAL::millis();
    while (true)
    {
        if (handle_serial())
        {
            lastMessageTime = AP_HAL::millis();
        }
        else
        {
            if ((AP_HAL::millis() - lastMessageTime) > 2000)
            {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "No UWB Update in 2 seconds");
                lastMessageTime = AP_HAL::millis();
            }
            hal.scheduler->delay(1);
        }
    }
}

bool AP_UWB_FLNC_UWB_2::update()
{
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB Update");
    return true;
}

// format of serial packets received from rangefinder
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0xAA
// byte 1               CMD             Packet command/type
// byte 2               LENGTH          Packet Length
// byte x               DATA            Packet Data
// byte x               CRC8            packet CRC

bool AP_UWB_FLNC_UWB_2::handle_serial()
{
    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB nptr");
        return false;
    }

    // ensure we own the uart
    uart->begin(0);

    // read any available lines from Serial interface
    for (auto i=0; i<512; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        // Temp loopback/echo
        // send_byte(c);

        linebuf[linebuf_len] = c;
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB (%u|%u)",linebuf[linebuf_len],linebuf_len);
        linebuf_len++;
        switch (rxState)
        {
            case UWB_SER_WAIT_START:
                if (linebuf[linebuf_len-1] == 0xAA)
                    rxState = UWB_SER_WAIT_CMD;
                else
                    linebuf_len--;
                break;

            case UWB_SER_WAIT_CMD:
                rxState = UWB_SER_WAIT_LENGTH;
                break;

            case UWB_SER_WAIT_LENGTH:
                if (linebuf[2] == 0)
                    rxState = UWB_SER_WAIT_CRC;
                else
                    rxState = UWB_SER_WAIT_DATA;
                break;

            case UWB_SER_WAIT_DATA:
                if (linebuf_len >= linebuf[2] + 3)// DataLength + 3 bytesbuffer(start + cmd + length)
                    rxState = UWB_SER_WAIT_CRC;

                break;

            case UWB_SER_WAIT_CRC:
                if (checkCRC())
                    rxState = UWB_SER_PACKET_DONE;
                else
                    rxState = UWB_SER_PACKET_CRC_ERROR;
                break;

            default:
                //ERROR
                break;
        }

        if (rxState == UWB_SER_PACKET_DONE)
        {
            handle_packet();
            rxState = UWB_SER_WAIT_START;
            linebuf_len = 0;

            return true;
        }
        if (rxState == UWB_SER_PACKET_CRC_ERROR)
        {
            //TODO: handle Error
            rxState = UWB_SER_WAIT_START;
            linebuf_len = 0;

            gcs().send_text(MAV_SEVERITY_WARNING, "UWB CRC error");
            return false;
        }
    }
    
    // no readings so return false
    return false;
}

void AP_UWB_FLNC_UWB_2::handle_packet()
{
    switch (linebuf[1])
    {
        case UWB_FLNC_GROUND_PRESSURE:
        {
            state.last_baro_data.ground_station.id        = bufferToHWord(linebuf+3);
            state.last_baro_data.ground_station.pressure  = bufferToFloat(linebuf+5);
            state.last_baro_data.ground_station.sigma     = bufferToFloat(linebuf+9);
            break;
        }
        case UWB_FLNC_RAW_BARO_MEASUREMENT:
        {
            state.last_baro_data.tag.pressure = bufferToFloat(linebuf+3);
            state.last_baro_data.tag.sigma    = bufferToFloat(linebuf+7);
            AP_Baro::get_singleton()->set_data(state.last_baro_data.tag.pressure, state.last_baro_data.tag.sigma);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB BARO: %f|%f", state.last_baro_data.tag.pressure, state.last_baro_data.tag.sigma);
            break;
        }
        default:
            break;
    }
    return;
}

bool AP_UWB_FLNC_UWB_2::checkCRC()
{
    // uint8_t crc = calcCRC(linebuf_len - 4);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "UWB CRC: 0x%02X|0x%02X", linebuf[linebuf_len-1], crc);
    
    return ( linebuf[linebuf_len-1] == calcCRC(linebuf_len - 4) );
}

/*
 * Calculate the crc for the packet stored in the class
 *
 * @return: (uint8_t)The calculated crc
 */
uint8_t AP_UWB_FLNC_UWB_2::calcCRC(uint8_t packetLength)
{
	uint8_t crc = 0;

	// _length is data length
	for (uint16_t index = 1; index <= packetLength +2U; index++)
	{
		crc += linebuf[index];
	}
	return crc;
}

bool AP_UWB_FLNC_UWB_2::get_reading(float &reading_m)
{
    return false;
}

bool AP_UWB_FLNC_UWB_2::send_message(uint8_t cmd, uint8_t length, const void *msg)
{
    if (uart->txspace() < (uint32_t)(length + 4)) // Length (Data) + 4(start + cmd + length + crc)
        return false;
    
    // Calc crc
    uint8_t crc = 0x00;

    uint8_t startByte = 0xAA;

    uart->write((const uint8_t *)&startByte, 1);
    uart->write((const uint8_t *)&cmd, 1);
    uart->write((const uint8_t *)&length, 1);
    uart->write((const uint8_t *)&msg, length);
    uart->write((const uint8_t *)&crc, 1);
    return true;
}

bool AP_UWB_FLNC_UWB_2::send_byte(uint8_t data)
{
    if (uart->txspace() < 1)
        return false;

    uart->write((const uint8_t *)&data, 1);
    return true;
}


#endif // AP_UWB_FLNC_UWB_2_ENABLED
