/*
* @file MKR_WAN_1310.ino
*
* @brief TODO
* 
* Libraries:
* - Arduino SAMD Boards (32-bits ARM Cortex-M0+) - v1.8.13
* - Adafruit SleepyDog Library - v1.6.4
* - Arduino Low Power - v1.2.2
* - LoRa by Sandeep Mistry - v0.8.0 - NOT USED!
* - MKRWAN by Arduino - v1.1.0
* - LSM6 by Pololu - v2.0.1
* - LIS3MDL by Pololu - v1.0.0
* - LPS by Pololu - v3.0.1
*/

#include <Wire.h>
#include <SPI.h>
//#include <LoRa.h>
#include <MKRWAN.h>
#include <Adafruit_SleepyDog.h>
#include <ArduinoLowPower.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include "MKR_WAN_1310.h"

// Data types
struct AxisData 
{
    int x;
    int y;
    int z;
};

typedef struct
{
  // Device-related
  uint16_t dev_id;
  uint8_t sensor_type;
  uint16_t sample_rate_sec;
  uint16_t battery;
  uint16_t rssi;
  
  // Measurements
  uint8_t sensor_error;
  AxisData axis_data;
  uint16_t lidar_dist;

} Lora_sensor_t;

// Constants and macro definitions
#define SENSOR_TYPE_ACCELEROMETER 0
#define SENSOR_TYPE_LIDAR         1
#define LORA_FREQUENCY 868E6
#define MOSFET_PIN 7
#define RESOLUTION 10
#define MAG_RESOLUTION 10

// Global variables
const byte TFMINI_I2C_ADDR = 0x10; 

//LoRaModem modem;
LoRaModem modem(Serial1);
LSM6 imu;
LIS3MDL mag;
LPS bar;
Lora_sensor_t g_sensor = {0};

int iterator = 0;
int sumValue[3] = {0, 0, 0};
int readingValuesX[RESOLUTION];
int readingValuesY[RESOLUTION];
int readingValuesZ[RESOLUTION];
int mag_iterator = 0;
int mag_sumValue[3] = {0, 0, 0};
int mag_readingValuesX[MAG_RESOLUTION];
int mag_readingValuesY[MAG_RESOLUTION];
int mag_readingValuesZ[MAG_RESOLUTION];

// ***********************************************************************
// ****************************** Setup **********************************
// ***********************************************************************

void setup() 
{ 
  g_sensor.dev_id = DEVICE_ID;
  g_sensor.sample_rate_sec = SAMPLE_RATE_DEFAULT_SEC;

#if ALT_IMU_10_V5_EN
  g_sensor.sensor_type = SENSOR_TYPE_ACCELEROMETER;
#elif TF_MINI_LIDAR_EN
  g_sensor.sensor_type = SENSOR_TYPE_LIDAR;
#endif

  // GPIO Initializations
  pinMode(10, OUTPUT);    // DEBUG: For PPK2 D0
  pinMode(LED_BUILTIN, OUTPUT);    // Led init
  pinMode(MOSFET_PIN, OUTPUT);     // MOSFET init
  digitalWrite(MOSFET_PIN, HIGH);  // MOSFET turn off

  for (int i = 0; i < 15; i++) 
  {
    // TODO: Update for less battery consumption
    // pinMode(i, INPUT_PULLUP);
  }

#if BATTERY_MONITOR
  // Set the ADC reference and resolution
  analogReference(AR_INTERNAL1V65); // AR_DEFAULT, AR_INTERNAL1V65, AR_INTERNAL2V23
  analogReadResolution(12);
  // Perform a few measurements after changing the reference to stabilize ADC
  analogRead(A0);
  analogRead(A0);
  analogRead(A0);
  analogRead(A0);
  analogRead(A0);
#endif

  // Enable watchdog for library initializations
  Watchdog.enable(8000);

  // Library Initializations
  Wire.begin();
#if DEBUG_MODE
  Serial1.begin(9600);
#endif

#if LORA_EN
  LORA_setup();
  LORA_join();
#endif

#if DEBUG_MODE
  Serial1.println("Initializations completed!");
  Serial1.print("Device ID: ");
  Serial1.print(g_sensor.dev_id);
  Serial1.print(", Sensor Type: ");
  Serial1.print(g_sensor.sensor_type);
  Serial1.print(", Sample Rate: ");
  Serial1.println(g_sensor.sample_rate_sec);
#endif
}

// ***********************************************************************
// ****************************** Main ***********************************
// ***********************************************************************

void loop_debug() 
{  
  Watchdog.disable(); 

  digitalWrite(MOSFET_PIN, HIGH);
  Serial1.println("HIGH");
  delay(3000);

  digitalWrite(MOSFET_PIN, HIGH);
  Serial1.println("HIGH");
  delay(3000);
}

void loop() 
{

#if DEBUG_MODE
  // Serial1.println("Loop");
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  // Enable watchdog
  Watchdog.enable(8000);

  // *************** 1. Measurements **************  

#if BATTERY_MONITOR
  // Get the battery voltage
  Battery_monitor();
#endif

#if TF_MINI_LIDAR_EN
  // Get TF mini Lidar measurement
  digitalWrite(MOSFET_PIN, LOW);
  delay(100);
  TFMini_measurement();
  digitalWrite(MOSFET_PIN, HIGH);
#endif

#if ALT_IMU_10_V5_EN
  // Get Alt-IMU measurement
  ALTIMU10_init();
  ALTIMU10_measurement();
#endif

  // *************** 2. LoRa packet ***************

#if LORA_EN
  // Wake up LoRa module
  LORA_wake_up();
  // Transmit packet
  LORA_transmission();
#endif

  // *************** 3. Sleep *********************

#if LORA_EN
  // Set LoRa module to sleep mode
  LORA_sleep_mode();
#endif

#if ALT_IMU_10_V5_EN
  ALTIMU10_sleep();
#endif

#if DEBUG_MODE
  digitalWrite(LED_BUILTIN, LOW);
#endif

  // Reset/Disable watchdog
  Watchdog.disable(); 
  //Watchdog.reset();

#if SLEEP_ENABLED
  // Go to deep sleep mode
  LowPower.deepSleep(g_sensor.sample_rate_sec * 1000);
#else
  // Wait for some time
  delay(g_sensor.sample_rate_sec * 1000);
#endif

}

// ***********************************************************************
// ****************************** Sensors ********************************
// ***********************************************************************

void Battery_monitor()
{
  
  // TODO: Update the entire function

  float voltValue, battery_volt, battery_volt_norm, battery_percentage;
  float minimal_voltage = 1800;
  float battery_voltage_ref = 1.65;

  // Reading from the Battery Pin
  voltValue = analogRead(A0);
  
  // Convert ADC raw to mV
	// Vcc = ADC * (Vref / 1024) * (1 + R1/R2)
	battery_volt = (voltValue * battery_voltage_ref / 4095.0) * (1.0 + (680000.0/470000.0)) * 1000;

  // Calculate current voltage level
  battery_volt_norm = ((voltValue * battery_voltage_ref) / 4095) * 1000;

  // Battery level expressed in percentage
  battery_percentage = 100*abs((battery_volt - minimal_voltage) / ((battery_voltage_ref * 1000) - minimal_voltage));

  // TODO: Update with uint16 mV
  g_sensor.battery = voltValue;

#if DEBUG_MODE
  Serial1.print(F("Battery: "));
  Serial1.print(voltValue);
  Serial1.print(F(" ADC, "));
  Serial1.print(battery_volt);
  Serial1.print(F(" mV, "));
  Serial1.print(battery_volt_norm);
  Serial1.print(F(" mV, "));
  Serial1.print(battery_percentage);
  Serial1.println(F("%"));
#endif

}

// ***********************************************************************
// ****************************** TF-Mini Lidar **************************
// ***********************************************************************

#if TF_MINI_LIDAR_EN
/*!
* @brief Module initialization. 
*
* Default I2C address is 0x10. 
* Automatic shift gear mode and fixed ranging gear mode. TODO: Check if fixed is better for this use case
* Maximmum range limit is 12m. 
* Unit of measuremets: Millimeter (mm) and centimeter (cm). Default is cm.
* Measurement accuracy: ±4cm@ (0.3-6m) and ±1%@ (6m-12m)
* Range resolution: 1cm
*
*/
void TFMini_measurement()
{
  uint16_t distance = 0; 
  uint16_t strength = 0; 
  uint8_t range_type = 0;
  uint8_t retries = 3;
  bool res = false;

  do
  {
    res = TFMini_get_distance(TFMINI_I2C_ADDR, &distance, &strength, &range_type);  

  } while (!res && --retries);

  if (res)
  {
    g_sensor.sensor_error = 0;
    g_sensor.lidar_dist = distance;
  }
  else
  {
    g_sensor.sensor_error = 1;
    g_sensor.lidar_dist = 0;
  }
}

/*!
* @brief Module initialization. 
*
* @param[in] dev_address The I2C address of the device
* @param[in] distance Distance in [mm]
* @param[in] strength Signal strength. Rangee is 0 - 3000. 20 and 2000 are considered credible.
* @param[in] range_type Range Type: 0 - Short distance, 3 - Intermediate distance, 7 - Long distance
*
* @return True for succeess, ffalse otherwise
*/
boolean TFMini_get_distance(uint8_t dev_address, uint16_t *distance, uint16_t *strength, uint8_t *range_type)
{
  boolean valid_data = false;

  *distance = 0;
  *strength = 0;
  *range_type = 0;
  
  Serial1.println("A");

  Wire.beginTransmission(dev_address);
  Wire.write(0x01); // MSB
  Wire.write(0x02); // LSB
  Wire.write(7);    // Data length: 7 bytes for distance data


  Serial1.println("B");

  int res = Wire.endTransmission(true); // TODO: false

  if (res != 0) 
  {
#if DEBUG_MODE
          Serial1.print("Error: No sensor ");
          Serial1.println(res);
#endif
    // Sensor did not ACK
    return (false);
  }

  Serial1.println("C");

  // Request 7 bytes
  Wire.requestFrom(dev_address, (uint8_t)7); 

  if (Wire.available())
  {
    for (uint8_t x = 0 ; x < 7 ; x++)
    {
      uint8_t incoming = Wire.read();

      if (x == 0)
      {
        //Trigger done
        if (incoming == 0x00)
        {

          // NOTE: This is not invalid data. It's the previous frame

#if DEBUG_MODE
          Serial1.println("Data invalid");
#endif
          valid_data = false;
        }
        else if (incoming == 0x01)
        {
#if DEBUG_MODE
          Serial1.print("Data valid:     ");
#endif
          valid_data = true;
        }
      }
      else if (x == 2)
      {
        *distance = incoming; //LSB of the distance value "Dist_L"
      }
      else if (x == 3)
      {
        *distance |= incoming << 8; //MSB of the distance value "Dist_H"
      }
      else if (x == 4)
      {
        *strength = incoming; //LSB of signal strength value
      }
      else if (x == 5)
      {
        *strength |= incoming << 8; //MSB of signal strength value
      }
      else if (x == 6)
      {
        *range_type = incoming; //range scale
      }
    }

#if DEBUG_MODE
    if (valid_data == true) 
    {
      Serial1.print("\tDist[");
      Serial1.print(*distance);
      Serial1.print("]\tstrength[");
      Serial1.print(*strength);
      Serial1.print("]\tmode[");
      Serial1.print(*range_type);
      Serial1.print("]");
      Serial1.println();
    }
#endif

  }
  else
  {

#if DEBUG_MODE
    Serial1.println("No wire data avail");
#endif
    valid_data = false;
  }

  return valid_data;
}

#endif


// ***********************************************************************
// ****************************** Alt IMU 10 v5 **************************
// ***********************************************************************

#if ALT_IMU_10_V5_EN

/*!
* @brief Module initialization
*
*/
void ALTIMU10_init()
{
  g_sensor.sensor_error = 0;

  if(!imu.init()) 
  {
    g_sensor.sensor_error = 1;

#if DEBUG_MODE
    Serial1.println("Error: IMU failed to initialize");
#endif
  }
  else
  {
    imu.enableDefault();
    // Set the gyro full scale to 1000 dps because the default
    // value is too low, and leave the other settings the same.
    imu.writeReg(LSM6::CTRL2_G, 0b10001000);
    // Set the accelerometer full scale to 16 g because the default
    // value is too low, and leave the other settings the same.
    imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  }


  // NOTE: Magnetometer is disabled. N/A 

  if(!mag.init()) 
  {
    g_sensor.sensor_error = 1;

#if DEBUG_MODE
    Serial1.println("Error: Magnetometer failed to initialize");
#endif
  }
  else
  {
    mag.enableDefault();
  }



  if(!bar.init()) 
  {
    g_sensor.sensor_error = 1;

#if DEBUG_MODE
    Serial1.println("Error: Barometer failed to initialize");
#endif
  }
  else
  {
    bar.enableDefault();
  }


}

/*!
* @brief Accelerometer measurement
*
*/
void ALTIMU10_measurement()
{
  
  // TODO: Clear measurement in case of error !!!

  imu.read();
  //mag.read();
  
  AxisData accelData = calibreAccel(imu.a.x, imu.a.y, imu.a.z);
  AxisData axisDegree = calculateDegree(accelData);
  // AxisData magData = calibreMag(mag.m.x, mag.m.y, mag.m.z);

  g_sensor.axis_data.x = accelData.x;
  g_sensor.axis_data.y = accelData.y;
  g_sensor.axis_data.z = accelData.z;

#if DEBUG_MODE
    Serial1.print("Cal_X: ");
    Serial1.print(accelData.x);
    Serial1.print(SEP);
    Serial1.print("Cal_Y: ");
    Serial1.print(accelData.y);
    Serial1.print(SEP);
    Serial1.print("Cal_Z: ");
    Serial1.print(accelData.z);
    Serial1.print(SEP);

    Serial1.print("Deg_X: ");
    Serial1.print(axisDegree.x);
    Serial1.print(SEP);
    Serial1.print("Deg_Y: ");
    Serial1.print(axisDegree.y);
    Serial1.print(SEP);
    Serial1.print("Deg_Z: ");
    Serial1.print(axisDegree.z);
    Serial1.print(SEP);

    // Serial1.print("Mag_X: ");
    // Serial1.print(magData.x);
    // Serial1.print(SEP);
    // Serial1.print("Mag_Y: ");
    // Serial1.print(magData.y);
    // Serial1.print(SEP);
    // Serial1.print("Mag_Z :");
    // Serial1.print(magData.z);
    
    Serial1.println();
#endif

}

/*!
* @brief Accelerometer sleep mode
*
*/
void ALTIMU10_sleep()
{
  // uint8_t readReg(uint8_t reg);

  // Power down
  imu.writeReg(LSM6::CTRL1_XL, 0x00);
  imu.writeReg(LSM6::CTRL2_G, 0x00);
  mag.writeReg(LIS3MDL::CTRL_REG3, 0x03);
  bar.writeReg(LPS::CTRL_REG1, 0x30);
}

/*!
* @brief Calibrate accelerometer measurement
*
*/
AxisData calibreAccel(int aX, int aY, int aZ) 
{

    AxisData result{};

    // X axis
    sumValue[0] = sumValue[0] - readingValuesX[iterator];
    readingValuesX[iterator] = aX;
    sumValue[0] = sumValue[0] + aX;

    result.x = sumValue[0] / RESOLUTION;

    // Y axis
    sumValue[1] = sumValue[1] - readingValuesY[iterator];
    readingValuesY[iterator] = aY;
    sumValue[1] = sumValue[1] + aY;

    result.y = sumValue[1] / RESOLUTION;

    // Z axis
    sumValue[2] = sumValue[2] - readingValuesZ[iterator];
    readingValuesZ[iterator] = aZ;
    sumValue[2] = sumValue[2] + aZ;

    result.z = sumValue[2] / RESOLUTION;

    iterator = (iterator + 1) % RESOLUTION;

    return result;
}

/*!
* @brief Calibrate gyro measurement
*
*/
AxisData calculateDegree(AxisData data)
{
    int degreeResolution = 22;

    AxisData degree;
    degree.x = data.x / degreeResolution;
    degree.y = data.y / degreeResolution;
    degree.z = data.z / degreeResolution;
    return degree;
}

/*!
* @brief Calibrate magnetometer measurement
*/
AxisData calibreMag(int magX, int magY, int magZ) 
{

    AxisData result{};

    // X axis
    mag_sumValue[0] = mag_sumValue[0] - mag_readingValuesX[mag_iterator];
    mag_readingValuesX[mag_iterator] = magX;
    mag_sumValue[0] = mag_sumValue[0] + magX;

    result.x = mag_sumValue[0] / RESOLUTION;

    // Y axis
    mag_sumValue[1] = mag_sumValue[1] - mag_readingValuesY[mag_iterator];
    mag_readingValuesY[mag_iterator] = magY;
    mag_sumValue[1] = mag_sumValue[1] + magY;

    result.y = mag_sumValue[1] / RESOLUTION;

    // Z axis
    mag_sumValue[2] = mag_sumValue[2] - mag_readingValuesZ[mag_iterator];
    mag_readingValuesZ[mag_iterator] = magZ;
    mag_sumValue[2] = mag_sumValue[2] + magZ;

    result.z = mag_sumValue[2] / RESOLUTION;

    iterator = (iterator + 1) % RESOLUTION;

    return result;
}

#endif

// ***********************************************************************
// ****************************** LoRa module ****************************
// ***********************************************************************

/*!
* @brief Module initialization. 
*
* Setup LoRa module with European frequency. 
* NOTE: In case of failure, waits for watchdog reset.
*
*/
void LORA_setup()
{
// Initialize LoRa module. NOTE: This is required every time after sleep actions
  if (!modem.begin(EU868)) 
  {
#if DEBUG_MODE
    Serial1.println("Error: LoRa Lib init failed!");
#endif
    // NOTE: Wait for watchdog reset here in case of LoRa module failure
    while (1);
  }
  else
  {

  // Set data rate to 6
  if(!modem.dataRate(6))
  {
#if DEBUG_MODE
    Serial1.println("Error: LoRa data rate failed!");
#endif
  }
    
#if DEBUG_MODE
  Serial1.print("LoRa module version: ");
  Serial1.print(modem.version());
  Serial1.print(" Device EUI: ");
  Serial1.print(modem.deviceEUI());
  Serial1.print(" ADR: ");
  Serial1.print(modem.getADR());
  Serial1.print(" DR: ");
  Serial1.print(modem.getDataRate());
  Serial1.println();
  
#endif
  }
}

/*!
* @brief LoRa joins a network with OTAA 
*
*/
void LORA_join()
{
  int connected = modem.joinOTAA(LORA_APP_EUI, LORA_APP_KEY);

  if (!connected) 
  {
#if DEBUG_MODE
    Serial1.println("Error: LoRa Join failed!");
#endif
  }

  // Set poll interval to 60 secs.
  modem.minPollInterval(60); // TODO: To be checked or delete
  // Lib NOTE: independent of this setting, the modem will
  // not allow sending more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.
}

/*!
* @brief LoRa transmits a packet
*
*/
void LORA_transmission()
{
  
  static int counter = 0;
  int err = 0;
  
  const int BUF_SIZE = 16; 
  byte buffer_lora[BUF_SIZE] = {0};

  // Clear buffer
  for (int i = 0; i < BUF_SIZE; i++) 
  {
    buffer_lora[i] = 0;
  }

  // Copy globals to the LoRa buffer
  buffer_lora[0] = g_sensor.dev_id >> 8;
  buffer_lora[1] = g_sensor.dev_id;
  buffer_lora[2] = g_sensor.sensor_type;
  buffer_lora[3] = g_sensor.sample_rate_sec >> 8;
  buffer_lora[4] = g_sensor.sample_rate_sec;
  buffer_lora[5] = g_sensor.battery >> 8;
  buffer_lora[6] = g_sensor.battery;
  buffer_lora[7] = g_sensor.rssi >> 8;
  buffer_lora[8] = g_sensor.rssi;

#if ALT_IMU_10_V5_EN

  // Copy sensor measurements
  buffer_lora[9] = g_sensor.axis_data.x >> 8;
  buffer_lora[10] = g_sensor.axis_data.x;
  buffer_lora[11] = g_sensor.axis_data.y >> 8;
  buffer_lora[12] = g_sensor.axis_data.y;
  buffer_lora[13] = g_sensor.axis_data.z >> 8;
  buffer_lora[14] = g_sensor.axis_data.z;
  buffer_lora[15] = g_sensor.sensor_error;

#elif TF_MINI_LIDAR_EN

  // Copy sensor measurements
  buffer_lora[9] = g_sensor.lidar_dist >> 8;
  buffer_lora[10] = g_sensor.lidar_dist;
  buffer_lora[11] = g_sensor.sensor_error;

#endif

  // Transmit a packet
  modem.beginPacket();
  modem.write(buffer_lora, BUF_SIZE); 
  //modem.print(counter);
  err = modem.endPacket(true);

  if (err > 0) 
  {
#if DEBUG_MODE
    // TESTING
    for (int i = 0; i < sizeof(buffer_lora); i++) 
    {
      //Serial1.print(buffer_lora[i], DEC);
      //Serial1.print(" ");
    }
    Serial1.println("LoRa Tx: OK");
#endif
  } 
  else 
  {
#if DEBUG_MODE
    Serial1.print("LoRa Tx: Error ");
    Serial1.println(err);
#endif
  }
  
  // TODO: Implement downlink

  delay(100); // 1000

  // Check for downlink
  if (!modem.available()) 
  {
    return;
  }

  char rcv[64];
  int i = 0;
  
  // Receive downlink
  while (modem.available()) 
  {
    rcv[i++] = (char)modem.read();
  }

#if DEBUG_MODE
  Serial1.print("Received: ");

  for (unsigned int j = 0; j < i; j++) 
  {
    Serial1.print(rcv[j] >> 4, HEX);
    Serial1.print(rcv[j] & 0xF, HEX);
    Serial1.print(" ");
  }
  Serial1.println();
#endif

  counter++;
}

#if 0
void LORA_reception()
{

  while(1)
  {
    int packetSize = LoRa.parsePacket();

    if (packetSize) 
    {
      Serial1.print("Received packet '");

      // Read packet
      while (LoRa.available()) 
      {
        Serial1.print((char)LoRa.read());
      }

      // Print RSSI of packet
      Serial1.print("' with RSSI ");
      Serial1.println(LoRa.packetRssi());
    }

    Watchdog.reset();
  }
}
#endif

/*!
* @brief LoRa module sleep actions
*
*/
void LORA_sleep_mode()
{
  int res = modem.sleep(true);
  // Serial1.print("LoRa Sleep ");
  // Serial1.println(res);

  // Clear module reset pin
  // pinMode(LORA_RESET, OUTPUT);
  // digitalWrite(LORA_RESET, LOW);
}

/*!
* @brief LoRa module wake up actions
*
*/
void LORA_wake_up()
{
  int res = modem.sleep(false);
  // Serial1.print("LoRa Wake ");
  // Serial1.println(res);

  // pinMode(LORA_RESET, OUTPUT);
  // digitalWrite(LORA_RESET, HIGH);
  // delay(100);
}

/*** End of file ***/