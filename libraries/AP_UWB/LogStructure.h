#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_UWB                        \
    LOG_UWB_MSG

// @LoggerMessage: UWB
// @Description: Information received from UWB systems attached to the autopilot
// @Field: TimeUS: Time since system startup
// @Field: PGnd: Last received ground pressure
// @Field: PGndVar: Last received ground pressure variance
// @Field: PGndFlt: Filtered ground pressure
// @Field: PGndFltVar: Filtered ground pressure variance
// @Field: PTag: Tag pressure
// @Field: PTagVar: Tag pressure variance
struct PACKED log_UWB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float last_gnd_press;
    float last_gnd_press_var;
    float gnd_press_filtered;
    float gnd_press_var;
    float tag_press;
    float tag_press_var;    
};

#define LOG_STRUCTURE_FROM_UWB \
    { LOG_UWB_MSG, sizeof(log_UWB), \
      "UWB", \
      "Q"       "f"     "f"         "f"         "f"             "f"     "f",      \
      "TimeUS," "PGnd," "PGndVar,"  "PGndFlt,"  "PGndFltVar,"   "PTag," "PTagVar", \
      "s"       "P"     "P"         "P"         "P"             "P"     "P",      \
      "F"       "0"     "0"         "0"         "0"             "0"     "0",      \
      true \
    }
