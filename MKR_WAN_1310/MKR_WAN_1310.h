/*
* @file MKR_WAN_1310.h
*
* @brief TODO
* 
*/

#ifndef MKR_WAN_1310_H
#define MKR_WAN_1310_H

// ************* Device IDs **************
// Accelerometer Sensors: 1001, 1002, 1003
// LiDar Sesors         : 1004, 1005
#define DEVICE_ID 1001

// Measurements
#define BATTERY_MONITOR 0
#define ALT_IMU_10_V5_EN 1
#define TF_MINI_LIDAR_EN 0

// Debugging
#define DEBUG_MODE 1
#define SLEEP_ENABLED 1
#define LORA_EN 1

// Configuration
#define SAMPLE_RATE_DEFAULT_SEC 3600
#define SEP "  "

// LoRa
String LORA_APP_EUI = "7076FF00560808E0";
String LORA_APP_KEY = "46f60e71cb867b2a3de15f56168f226b";

#endif

/*** End of file ***/