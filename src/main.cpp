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
 * @brief BME280 reader with Blynk connectivity
 * This code will read a BME280 atmospheric sensor and delivers the read values
 * to a Blynk device that displays them in charts.
 */

// User GPIO12 to power on the BME280
#define BME_PWR 12

#define DEBUG 0

#if DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
#endif    

//#define BLYNK_PRINT Serial

#define SEALEVELPRESSURE_HPA (1013.25)
#define MEASURE_INTERVAL (450e6) // measurement interval in usec. This value isn't precise! (~5 min)

#define BLYNK_TEMPLATE_ID "TMPLlf4frUl5"
#define BLYNK_AUTH_TOKEN "HgsHK7F6nggW7arD-RdhXVLPbm2NWFbs"

#include <Arduino.h> 
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266_SSL.h>
#include <credentials.h>

// Static WiFi IP address settings
IPAddress _ip (192,168, 25, 99);
IPAddress _gw (192,168, 25,  1);
IPAddress _net(255,255,255,  0);
IPAddress _dns(192,168, 25,  5);

char auth[] = "HgsHK7F6nggW7arD-RdhXVLPbm2NWFbs";

uint32_t ts1, ts2, startupTime;

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

Adafruit_BME280 bme; // Implements an I2C connectivity

#if DEBUG  
  Serial.begin(74880);
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

  // Connect to Blynk
  Blynk.config(auth); // for cloud server
  DEBUG_PRINT("Blynk is configured"); 
  while (Blynk.connect() == false) {
    DEBUG_PRINT(".");
  }
  DEBUG_PRINT("Blynk is connected");

  // Send the weather data to Blynk
  float h = bme.readHumidity();            // Read the humidity
  DEBUG_PRINT("H: ");
  DEBUG_PRINT(h);

  float t = bme.readTemperature();         // Read the temperature in °C
  DEBUG_PRINT("T: ");
  DEBUG_PRINT(t);

  float p = bme.readPressure() / 100.0F;   // Read the pressure
  DEBUG_PRINT("P: ");  
  DEBUG_PRINT(p);

  // Send the atmospheric values to Blynk
  Blynk.virtualWrite(V0, h);
  Blynk.virtualWrite(V1, t);
  Blynk.virtualWrite(V2, p);
  Blynk.run();
  DEBUG_PRINT("Data sent to Blynk");

  // Power off the BME280 to save energy
  digitalWrite(BME_PWR, LOW); // Power off the BME280 before going to sleep 

  // Go into deep sleep instantly for aprox. 5.5 min.
  ESP.deepSleep(MEASURE_INTERVAL , WAKE_NO_RFCAL);
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