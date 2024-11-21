#include "AP_UWB_Params.h"
#include "AP_UWB.h"


// table of user settable parameters
const AP_Param::GroupInfo AP_UWB_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: Type of connected rangefinder
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLite-I2C,5:PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:USD1_Serial,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X or VL53L1X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:Benewake-Serial,21:LidarLightV3HP,22:PWM,23:BlueRoboticsPing,24:DroneCAN,25:BenewakeTFminiPlus-I2C,26:LanbaoPSK-CM8JL65-CC5,27:BenewakeTF03,28:VL53L1X-ShortRange,29:LeddarVu8-Serial,30:HC-SR04,31:GYUS42v2,32:MSP,33:USD1_CAN,34:Benewake_CAN,35:TeraRangerSerial,36:Lua_Scripting,37:NoopLoop_TOFSense,38:NoopLoop_TOFSense_CAN,39:NRA24_CAN,40:NoopLoop_TOFSenseF_I2C,41:JRE_Serial,100:SITL
    // @User: Standard
    // AP_GROUPINFO_FLAGS("TYPE", 1, AP_RangeFinder_Params, type, 0, AP_PARAM_FLAG_ENABLE),


    AP_GROUPEND
};

AP_UWB_Params::AP_UWB_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
