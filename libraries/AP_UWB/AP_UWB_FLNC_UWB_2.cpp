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

#ifdef AP_UWB_FLNC_UWB_2_ENABLED


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
    : AP_UWB_Backend_Serial(_state, _params)
{
    rxState     = UWB_SER_WAIT_START;
    linebuf_len = 0;
}

AP_UWB_FLNC_UWB_2::~AP_UWB_FLNC_UWB_2()
{

}

bool AP_UWB_FLNC_UWB_2::update()
{
    handle_packet();
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

// distance returned in reading_m, set to true if sensor reports a good reading
bool AP_UWB_FLNC_UWB_2::handle_serial()
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from Serial interface
    for (auto i=0; i<512; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        linebuf[linebuf_len] = c;
        switch (rxState)
        {
            case UWB_SER_WAIT_START:
                if (linebuf[linebuf_len] == 0xAA)
                {
                    linebuf_len++;
                    rxState = UWB_SER_WAIT_CMD;
                }
                break;

            case UWB_SER_WAIT_CMD:
                linebuf_len++;
                rxState = UWB_SER_WAIT_LENGTH;
                break;

            case UWB_SER_WAIT_LENGTH:
                linebuf_len++;
                rxState = UWB_SER_WAIT_DATA;
                break;

            case UWB_SER_WAIT_DATA:
                linebuf_len++;
                if (linebuf_len >= linebuf[2])// DataLength + 3 bytesbuffer(start + cmd + length)
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
            return true;
        }
        if (rxState == UWB_SER_PACKET_CRC_ERROR)
        {
            //TODO: handle Error
            rxState = UWB_SER_WAIT_START;
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
            // uint16_t anchorID   = bufferToHWord(linebuf+3);
            // float pressure      = bufferToFloat(linebuf+5);
            // float sigma         = bufferToFloat(linebuf+9);
            break;
        }
        case UWB_FLNC_RAW_BARO_MEASUREMENT:
        {
            // float pressure  = bufferToFloat(linebuf+3);
            // float sigma     = bufferToFloat(linebuf+7);
            break;
        }
        default:
            break;
    }
    return;
}

bool AP_UWB_FLNC_UWB_2::checkCRC()
{
    return true;
}

bool AP_UWB_FLNC_UWB_2::get_reading(float &reading_m)
{
    return false;
}


#endif // AP_UWB_FLNC_UWB_2_ENABLED
