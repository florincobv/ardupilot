#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_UWB \
    LOG_UWB_RNG_MSG

// @LoggerMessage: UWBR
// @Description: Ranging information from UWB sensors
// @Field: TimeUS: Time since system startup
// @Field: Instance: Instance ID of the UWB sensor
// @Field: Health: True if UWB sensor is healthy
// @Field: AId: Responder ID of last range measurement
// @Field: Ax: Responder position X coordinate
// @Field: Ay: Responder position Y coordinate
// @Field: Az: Responder position Z coordinate
// @Field: Rng: Range to responder in meters
// @Field: Var: Variance of the range measurement in meters squared
struct PACKED log_UWB_RNG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t health;
    uint16_t responder_id;
    float responder_pos_x;
    float responder_pos_y;
    float responder_pos_z;
    float range;
    float variance;
};

#define LOG_STRUCTURE_FROM_UWB \
    { LOG_UWB_RNG_MSG, sizeof(log_UWB_RNG), \
      "UWBR", \
      "Q"       "B"         "B"         "h"     "f"     "f"     "f"     "f"    "f",   \
      "TimeUS," "Instance," "Health,"   "AId,"  "Ax,"   "Ay,"   "Az,"   "Rng," "Var", \
      "s"       "#"         "-"         "-"     "m"     "m"     "m"     "m"    "?",   \
      "F"       "-"         "-"         "-"     "0"     "0"     "0"     "0"    "0",   \
      true \
    },
