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
  int dev_id;
  int sensor_type;
  int sample_rate_ms;
  int battery;
  int rssi;

  // Measurements
  AxisData axis_data;
  int lidar_dist;

} Lora_sensor_t;




#define LORA_FREQUENCY 868E6

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

void setup_debug()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);
}

void setup() 
{ 
  g_sensor.dev_id = DEVICE_ID;
  // g_sensor.sensor_type = TODO;
  g_sensor.sample_rate_ms = SAMPLE_RATE_DEFAULT_MS;

  // GPIO Initializations
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(10, OUTPUT); // DEBUG: For PPK2 D0
  
  // Test MOSFET
  pinMode(6, OUTPUT);
  

  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 15; i++) 
  {
    // TODO
    // pinMode(i, INPUT_PULLUP);
  }

#if BATTERY_MONITOR
  // Set the ADC to the default Analog Reference of 3.3V and 12-bit resolution
  analogReference(AR_INTERNAL1V65); // AR_DEFAULT
  analogReadResolution(12);
#endif

  // Enable watchdog for library initializations
  //Watchdog.enable(8000);

  // Library Initializations
  Wire.begin();
  Serial.begin(9600);

#if LORA_EN
  LORA_setup();
  LORA_join();
#endif

#if DEBUG_MODE
  Serial.println("Initializations completed!");
#endif
}

// ***********************************************************************
// ****************************** Main ***********************************
// ***********************************************************************

void loop_debug() 
{  
  digitalWrite(6, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LOW");
  delay(10000);
  digitalWrite(6, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("HIGH");
  delay(10000);
}

void loop() 
{

  // DEBUG: For PPK2 D0
  digitalWrite(10, LOW); 

#if DEBUG_MODE
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  // Enable watchdog
  //Watchdog.enable(8000);

  // *************** 1. Measurements **************  

#if BATTERY_MONITOR
  // Get the battery voltage
  Battery_monitor();
#endif

#if TF_MINI_LIDAR_EN
  // Get TF mini Lidar measurement
  TFMini_measurement();
#endif

#if ALT_IMU_10_V5_EN
  // Get Alt-IMU measurement
  ALTIMU10_init();
  ALTIMU10_measurement();
#endif

  // *************** 2. LoRa packet ***************

#if LORA_EN
  static int a = 0;
  if (a)
  {
    LORA_wake_up();
    LORA_setup();
    LORA_join();
  }
  a++;

  // Transmit packet
  LORA_transmission();
#endif

  // *************** 3. Sleep *********************
  
  // Disable watchdog
  //Watchdog.disable(); // Watchdog.reset();

  // Set LoRa module to sleep mode
  LORA_sleep_mode();

#if ALT_IMU_10_V5_EN
  ALTIMU10_sleep();
#endif

#if DEBUG_MODE
  digitalWrite(LED_BUILTIN, LOW);
#endif

// DEBUG: For PPK2 D0
digitalWrite(10, HIGH); 

#if SLEEP_ENABLED
  // Go to deep sleep mode
  LowPower.deepSleep(g_sensor.sample_rate_ms);
#else
  // Wait for some time
  delay(g_sensor.sample_rate_ms);
#endif

}

// ***********************************************************************
// ****************************** Sensors ********************************
// ***********************************************************************

void Battery_monitor()
{
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

#if DEBUG_MODE
  Serial.print(F("Battery: "));
  Serial.print(battery_volt);
  Serial.print(F(" mV, "));
  Serial.print(battery_volt_norm);
  Serial.print(F(" mV, "));
  Serial.print(battery_percentage);
  Serial.println(F("%"));
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
    // TODO: Add to packet
    g_sensor.lidar_dist = distance;
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
  
  Wire.beginTransmission(dev_address);
  Wire.write(0x01); // MSB
  Wire.write(0x02); // LSB
  Wire.write(7);    // Data length: 7 bytes for distance data

  if (Wire.endTransmission(false) != 0) 
  {
#if DEBUG_MODE
          Serial.println("No sensor");
#endif
    // Sensor did not ACK
    return (false);
  }

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
#if DEBUG_MODE
          Serial.println("Data invalid");
#endif
          valid_data = false;
        }
        else if (incoming == 0x01)
        {
#if DEBUG_MODE
          Serial.print("Data valid:     ");
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
      Serial.print("\tDist[");
      Serial.print(*distance);
      Serial.print("]\tstrength[");
      Serial.print(*strength);
      Serial.print("]\tmode[");
      Serial.print(*range_type);
      Serial.print("]");
      Serial.println();
    }
#endif

  }
  else
  {

#if DEBUG_MODE
    Serial.println("No wire data avail");
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

void ALTIMU10_init()
{
  if(!imu.init()) 
  {
#if DEBUG_MODE
    Serial.println("Error: IMU failed to initialize");
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

  if(!mag.init()) 
  {
#if DEBUG_MODE
    Serial.println("Error: Magnetometer failed to initialize");
#endif
  }
  else
  {
    mag.enableDefault();
  }

  if(!bar.init()) 
  {
#if DEBUG_MODE
    Serial.println("Error: Barometer failed to initialize");
#endif
  }
  else
  {
    bar.enableDefault();
  }

}


void ALTIMU10_measurement()
{
  imu.read();
  mag.read();

  printSensorData();
}


void ALTIMU10_sleep()
{
  // uint8_t readReg(uint8_t reg);

  // Power down
  imu.writeReg(LSM6::CTRL1_XL, 0x00);
  imu.writeReg(LSM6::CTRL2_G, 0x00);
  mag.writeReg(LIS3MDL::CTRL_REG3, 0x03);
  bar.writeReg(LPS::CTRL_REG1, 0x30);
}

void printSensorData() 
{
    //    char report[120];
//
//    snprintf_P(report, sizeof(report),
//               PSTR("A: %6d %6d %6d    M: %6d %6d %6d    G: %6d %6d %6d"),
//               imu.a.x, imu.a.y, imu.a.z,
//               mag.m.x, mag.m.y, mag.m.z,
//               imu.g.x, imu.g.y, imu.g.z);
//    Serial.println(report);


    AxisData accelData = calibreAccel(imu.a.x, imu.a.y, imu.a.z);
    AxisData axisDegree = calculateDegree(accelData);
    AxisData magData = calibreMag(mag.m.x, mag.m.y, mag.m.z);

    g_sensor.axis_data.x = accelData.x;
    g_sensor.axis_data.y = accelData.y;
    g_sensor.axis_data.z = accelData.z;

#if DEBUG_MODE
    Serial.print("Cal_X: ");
    Serial.print(accelData.x);
    Serial.print(SEP);
    Serial.print("Cal_Y: ");
    Serial.print(accelData.y);
    Serial.print(SEP);
    Serial.print("Cal_Z: ");
    Serial.print(accelData.z);
    Serial.print(SEP);

    Serial.print("Deg_X: ");
    Serial.print(axisDegree.x);
    Serial.print(SEP);
    Serial.print("Deg_Y: ");
    Serial.print(axisDegree.y);
    Serial.print(SEP);
    Serial.print("Deg_Z: ");
    Serial.print(axisDegree.z);
    Serial.print(SEP);

    Serial.print("Mag_X: ");
    Serial.print(magData.x);
    Serial.print(SEP);
    Serial.print("Mag_Y: ");
    Serial.print(magData.y);
    Serial.print(SEP);
    Serial.print("Mag_Z :");
    Serial.print(magData.z);
    
    Serial.println();
#endif
}


/// Eksenlere gore accelerometerı kalibrasyon yapıp
/// stabil sonuc dondurur.
/// Parametre olarak ham sensörden okundan veriler verilir
/// \param aX
/// \param aY
/// \param aZ
/// \return AxisData _.x, _,y, _,z formatında
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

///  Accelerometerdan okunan sensör verilerini
///  dereceye dönüştürür.
/// \param data : raw olarak verilen sensör bilgileri
/// \return _.x, _.y, _.z şeklinde derece döndürür
AxisData calculateDegree(AxisData data)
{
    int degreeResolution = 22;

    AxisData degree;
    degree.x = data.x / degreeResolution;
    degree.y = data.y / degreeResolution;
    degree.z = data.z / degreeResolution;
    return degree;
}

/// Eksenelere göre magnetometerı kalibre edip
/// stabil sonuc dondurur
/// \param magX
/// \param magY
/// \param magZ
/// \return
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
    Serial.println("Error: LoRa Lib init failed!");
#endif
    // NOTE: Wait for watchdog reset here in case of LoRa module failure
    while (1);
  }
  else
  {
#if DEBUG_MODE
  Serial.print("LoRa module version: ");
  Serial.print(modem.version());
  Serial.print(" Device EUI: ");
  Serial.println(modem.deviceEUI());
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
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
#endif
  }

  // Set poll interval to 60 secs.
  modem.minPollInterval(60); // TODO: To be checked or delete
  // NOTE: independent of this setting, the modem will
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
  uint16_t test = 40000;

#if ALT_IMU_10_V5_EN

  const int BUF_SIZE = 8;
  byte buffer_lora[BUF_SIZE];
  
  memcpy(&buffer_lora[0], &g_sensor.axis_data.x, sizeof(g_sensor.axis_data.x));
  memcpy(&buffer_lora[2], &g_sensor.axis_data.y, sizeof(g_sensor.axis_data.y));
  memcpy(&buffer_lora[4], &g_sensor.axis_data.z, sizeof(g_sensor.axis_data.z));

  // buffer_lora[0] = g_sensor.axis_data.x >> 8;
  // buffer_lora[1] = g_sensor.axis_data.x;
  // buffer_lora[2] = g_sensor.axis_data.y >> 8;
  // buffer_lora[3] = g_sensor.axis_data.y;
  // buffer_lora[4] = g_sensor.axis_data.z >> 8;
  // buffer_lora[5] = g_sensor.axis_data.z;
  buffer_lora[6] = 0;
  buffer_lora[7] = 0;

#elif TF_MINI_LIDAR_EN

  const int BUF_SIZE = 2;
  byte buffer_lora[BUF_SIZE];
  buffer_lora[0] = g_sensor.lidar_dist >> 8;
  buffer_lora[1] = g_sensor.lidar_dist;

#else

  const int BUF_SIZE = 2;
  byte buffer_lora[BUF_SIZE];

  buffer_lora[0] = test >> 8;
  buffer_lora[1] = test;

#endif

  // Transmit a packet
  modem.beginPacket();
  modem.write(buffer_lora, 2); // BUF_SIZE
  //modem.print(counter);
  err = modem.endPacket(true);

  if (err > 0) 
  {
#if DEBUG_MODE
    Serial.println("LoRa Tx: OK");
#endif
  } 
  else 
  {
#if DEBUG_MODE
    Serial.print("LoRa Tx: Error ");
    Serial.println(err);
#endif
  }

  // TODO: Why so big dely? To be checked.
  delay(1000);

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
  Serial.print("Received: ");

  for (unsigned int j = 0; j < i; j++) 
  {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.print(" ");
  }
  Serial.println();
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
      Serial.print("Received packet '");

      // Read packet
      while (LoRa.available()) 
      {
        Serial.print((char)LoRa.read());
      }

      // Print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
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

  // Clear module reset pin
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);

  // int res = modem.sleep();
  // Serial.print("LoRa Sleep ");
  // Serial.println(res);

  if (1)
  {
    //Serial.println("LoRa Sleep OK");
  }
  else
  {
    //Serial.println("LoRa Sleep NOT OK");
  }

}

// TO BE DELETED
void LORA_sleep_mode_bk()
{
  /*
  // digitalWrite(LORA_RESET, LOW);
  // digitalWrite(LORA_BOOT0, LOW);
  // pinMode(LORA_RESET, INPUT);
  // pinMode(LORA_DEFAULT_SS_PIN, INPUT);
  // pinMode(LORA_IRQ_DUMB, INPUT);
  // pinMode(LORA_BOOT0, INPUT);
  */

  //LoRa.sleep();
  digitalWrite(LORA_IRQ_DUMB, LOW);
  digitalWrite(LORA_RESET, LOW);
  digitalWrite(LORA_BOOT0, LOW);
  pinMode(LORA_IRQ_DUMB,INPUT);
  pinMode(LORA_RESET,INPUT);
  pinMode(LORA_BOOT0,INPUT);
}

/*!
* @brief LoRa module wake up actions
*
*/
void LORA_wake_up()
{
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(50);

  // pinMode(LORA_IRQ_DUMB, OUTPUT);
  // digitalWrite(LORA_IRQ_DUMB, LOW);

  // // Hardware reset
  // pinMode(LORA_BOOT0, OUTPUT);
  // digitalWrite(LORA_BOOT0, LOW);

  // pinMode(LORA_RESET, OUTPUT);
  // digitalWrite(LORA_RESET, HIGH);
  // delay(200);
  // digitalWrite(LORA_RESET, LOW);
  // delay(200);
  // digitalWrite(LORA_RESET, HIGH);
  // delay(50);

}

// ***********************************************************************
// ****************************** Debug **********************************
// ***********************************************************************

#if DEBUG_MODE
void debug_led()
{
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("Hello! ");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
#endif

