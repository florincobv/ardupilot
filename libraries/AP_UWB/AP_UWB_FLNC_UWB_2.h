#pragma once

#include "AP_UWB_config.h"

#ifdef AP_UWB_FLNC_UWB_2_ENABLED

#include "AP_UWB.h"
#include "AP_UWB_Backend.h"
#include "AP_UWB_Backend_Serial.h"

#define AP_UWB_MAX_MSG_SIZE 50


class AP_UWB_FLNC_UWB_2 : public AP_UWB_Backend_Serial
{

enum {
    UWB_SER_WAIT_START = 1,
    UWB_SER_WAIT_CMD,
    UWB_SER_WAIT_LENGTH,
    UWB_SER_WAIT_DATA,
    UWB_SER_WAIT_CRC,
    UWB_SER_PACKET_DONE,
    UWB_SER_PACKET_CRC_ERROR,
    UWB_SER_PACKET_TO
};

enum {
	UWB_FLNC_GET_NODE_ID = 0x00,
	UWB_FLNC_SET_NODE_ID = 0x01,

	UWB_FLNC_RAW_TWR_DATA  = 0x10,
	UWB_FLNC_RAW_TDOA_DATA = 0x11,
	UWB_FLNC_PRESSURE      = 0x12,

	UWB_FLNC_HEIGHT               = 0x31,
	UWB_FLNC_TEMPERATURE          = 0x32,
	UWB_FLNC_RAW_BARO_MEASUREMENT = 0x33,
	UWB_FLNC_GROUND_PRESSURE      = 0x34,

	UWB_FLNC_SETTINGS       = 0xF0,
	UWB_FLNC_DRONE_POSITION = 0xF1,
};


public:
    AP_UWB_FLNC_UWB_2(AP_UWB::UWB_State &_state, AP_HAL::UARTDriver *_port, AP_UWB_Params &_params);
    ~AP_UWB_FLNC_UWB_2() override;
    bool update() override;
    bool get_reading(float &reading_m) override;

protected:

    using AP_UWB_Backend_Serial::AP_UWB_Backend_Serial;

private:
    
    bool handle_serial();
    void handle_packet();
    bool checkCRC();
    uint8_t calcCRC(uint8_t packetLength);
    bool send_message(uint8_t cmd, uint8_t length, const void *msg);
    bool send_byte(uint8_t data);


    uint8_t linebuf[AP_UWB_MAX_MSG_SIZE];
    uint8_t linebuf_len;
    uint8_t rxState;
};
#endif  // AP_UWB_FLNC_UWB_2_ENABLED
