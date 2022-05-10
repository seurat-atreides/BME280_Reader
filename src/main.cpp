/**
 * @file main.cpp
 * @author Ernesto Lorenz (ernesto_lorenz@gmx.de)
 * @brief  BME280 atmospheric sensor reader/uploader to Blynk for an ESP8266 or ESP8285
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief BME280 reader with ubidots connectivity
 * This code will read a BME280 atmospheric sensor and delivers the read values
 * to a ubidots device that displays them in gauges.
 */

// User GPIO12 to power on the BME280
#define BME_PWR 12

//#define SERIAL_DEBUG 0 // This macro will be defined in platformio.ini under build_flags

#if SERIAL_DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
#endif    

//#define BLYNK_PRINT Serial

#define SEALEVELPRESSURE_HPA (1013.25)
#define MEASURE_INTERVAL (450e6) // measurement interval in usec. This value isn't precise! (~5 min)

#define UBIDOTS_TOKEN "BBFF-tjUD1TSEGKDIBas7Y3TbLDLsV3RvsF"

#include <Arduino.h> 
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ESP8266WiFi.h>

ADC_MODE(ADC_VCC); //vcc read-mode
#define ACD_CORR 1.146 // ACD correction coefficient

#include <Ubidots.h>
#include <credentials.h>

// Static WiFi IP address settings
IPAddress _ip (192,168, 25, 111);
IPAddress _gw (192,168, 25,  1);
IPAddress _net(255,255,255,  0);
IPAddress _dns(192,168, 25,  5);

// Instantiate an Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = SSID;
char pass[] = PASSWD;

/**
 * @brief Setup secction of the Arduino framework sketch.
 * 
 * 
 */
void setup() {

Adafruit_BME280 bme; // Instantiates a BME280 object

Ubidots ubidots(UBIDOTS_TOKEN); // Instantiates an Ubidots object

#if SERIAL_DEBUG  
  Serial.begin(74880);
  ubidots.setDebug(true);
#endif

  while (!Serial);
  DEBUG_PRINT();
  DEBUG_PRINT("Serial comms are up");

  WiFi.config(_ip, _gw, _net, _dns);
  WiFi.begin(ssid, pass);
  DEBUG_PRINT("WiFi is configured");

  // Power ON the BME280 board
  pinMode(BME_PWR, OUTPUT);
  digitalWrite(BME_PWR, HIGH);
  delay(5);

  bool BMEstarted = false;
  Wire.begin(4, 5); // Set the GPIO ports to use for the I2C bus
  do {
    if (!bme.begin(0x76, &Wire)) { // We pass bme.begin a Wire object: &Wire, defined during setup
      // Cycle power on BME280 board to reset it and try to initialize it again
      DEBUG_PRINT("Could not find the BME280 sensor, check your wiring!");
      digitalWrite(BME_PWR, LOW);
      delay(10);
      digitalWrite(BME_PWR, HIGH);
    }
    else
      BMEstarted = true;
  } while (!BMEstarted);  

  // Set up the BME280 in forced mode to use less power
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  DEBUG_PRINT("BEM is started");            

  
  DEBUG_PRINT("Ubidots client is configured"); 

  // Send the weather data to Blynk
  float h = bme.readHumidity();            // Read the humidity
  DEBUG_PRINT("H: ");
  DEBUG_PRINT(h);

  float t = bme.readTemperature();         // Read the temperature in °C
  DEBUG_PRINT("T: ");
  DEBUG_PRINT(t);

  float p = bme.readPressure() / 100.0F;   // Read the pressure and convert to millibar
  DEBUG_PRINT("P: ");  
  DEBUG_PRINT(p);

  float v = ((float)ESP.getVcc())/1024; // system_get_vdd33() unit is 1/1024 V; 1.131 is the correction coefficient
  DEBUG_PRINT("V: ");  
  DEBUG_PRINT(v); 

  // Send the atmospheric values to Ubidots
  ubidots.add("h", h);  
  ubidots.add("t", t);
  ubidots.add("p", p);
  ubidots.add("v", v);

  while(!ubidots.send("84f3eb1b1c8f", "BME280")) {
  }

  DEBUG_PRINT("Data sent to Ubidots");

  // Power off the BME280 to save energy
  digitalWrite(BME_PWR, LOW); // Power off the BME280 before going to sleep 

  // Go into deep sleep instantly for aprox. 5.5 min.
  ESP.deepSleep(MEASURE_INTERVAL , WAKE_NO_RFCAL);
  yield();
}

/**
 * @brief Loop secction of the Arduino framework sketch. 
 *        This secction will not be used since we are going into deep sleep.
 */
void loop() {
  /*
  We do notrhing in the loop secctiuon because
  all necesary steps are performed during setup
  and we are using deep sleep mode.
  */
}