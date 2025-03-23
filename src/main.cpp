/**
 * @file main.cpp
 * @author Ernesto Lorenz (ernesto_lorenz@gmx.de)
 * @brief  BME280 atmospheric sensor reader/uploader to Ubidots for an ESP8266 or ESP8285
 * @brief  BME280 atmospheric sensor reader/uploader to Ubidots for an ESP8266 or ESP8285
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

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <Ubidots.h>
#include "credentials.h"

// User GPIO12 to power on the BME280
#define BME_PWR 12

//#define SERIAL_DEBUG 0 // This macro will be defined in platformio.ini under build_flags

#if SERIAL_DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
  #define BLYNK_PRINT Serial
#else
  #define DEBUG_PRINT(x)
#endif

#define SEALEVELPRESSURE_HPA (1013.25)
#define MEASURE_INTERVAL (450e6) // measurement interval in usec. This value isn't precise! (~7 min)

// The following variables are not required because the device is only active for a very short term
// and then goes to sleep for ~ 7 min. No sense in assigning a static IP address.
/*
IPAddress _dns(208,67,222,222);
IPAddress _ip(192,168,25,33);
IPAddress _net(255,255,255,0);
IPAddress _gw(192,168,25,1);
*/

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
#if SERIAL_DEBUG
unsigned long ts1 = micros();
#endif

  Adafruit_BME280 bme; // Instantiates a BME280 object
  Ubidots ubidots(UBIDOTS_TOKEN); // Instantiates an Ubidots object

#if SERIAL_DEBUG
  Serial.begin(74880);
  ubidots.setDebug(true);
#endif

  DEBUG_PRINT("Serial comms are up");

  //WiFi.config(_ip, _gw, _net, _dns);
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

  bool bufferSent = false;
  ubidots.send("bme280");
  ubidots.send("bme280");
  DEBUG_PRINT("Data sent to Ubidots");

  // Power off the BME280 to save energy
  digitalWrite(BME_PWR, LOW); // Power off the BME280 before going to sleep
#if SERIAL_DEBUG
  Serial.print("Processing time = ");
  Serial.println((unsigned long)(micros() - ts1));
#endif
  // Go into deep sleep instantly for aprox. 5.5 min.
  ESP.deepSleep(MEASURE_INTERVAL , WAKE_NO_RFCAL);
  yield();
}

/**
 * @brief Loop secction of the Arduino framework sketch.
 *        This secction will not be used since we are going into deep sleep.
 */
void loop() {
  // Do nothing here. The BME280 read process is handled by the setup() routine.
}

