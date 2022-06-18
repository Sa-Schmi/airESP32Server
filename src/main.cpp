/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// Import required libraries
//#include <WiFi.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SparkFunBME280.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"

#define NUMMEASUREMENTS 4


struct Measurements
{
  public:
	float co2;
	float temperature;
	float humidity;
	float pressure;
};

BME280 bmeSensor;
SCD30 airSensor;
BME280_SensorMeasurements measurementsBME280;
Measurements measurements;

//SSID of your network
char ssid[] = "Fritzi";
//password of your WPA Network
char pass[] = "45904177236097773256";

AsyncWebServer server(80);

String header;

String output13State = "off";

const int output13 = 13;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
// const long timeoutTime = 2000;

// float pressure = 0;
// int alltitude = 

float readSCD30co2() {
  float p = airSensor.getCO2();
  if (isnan(p)) {
    Serial.println("Failed to read from SCD30 sensor!");
    return 0;
  }
  else {
    //Serial.println(p);
    return p;
  }
}

float readSCD30temp() {
  float p = airSensor.getTemperature();
  if (isnan(p)) {
    Serial.println("Failed to read from SCD30 sensor!");
    return 0;
  }
  else {
    //Serial.println(p);
    return p;
  }
}

float readSCD30hum() {
  float p = airSensor.getHumidity();
  if (isnan(p)) {
    Serial.println("Failed to read from SCD30 sensor!");
    return 0;
  }
  else {
    //Serial.println(p);
    return p;
  }
}

float readpres() {
  if (bmeSensor.isMeasuring()) // Wait for sensor to finish measuring
  {
      return measurements.pressure;
  };
  bmeSensor.readAllMeasurements(&measurementsBME280);

  if (isnan(measurementsBME280.pressure))
  {
    Serial.println("Failed to read from BMP280 sensor!");
    return 0;
  }
  else {
    airSensor.setAltitudeCompensation(((float)-44330.77)*(pow(((float)measurementsBME280.pressure/(float)bmeSensor.getReferencePressure()), 0.190263) - (float)1)); //Set altitude of the sensor in m
    //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
    airSensor.setAmbientPressure(measurementsBME280.pressure/100); //Current ambient pressure in mBar: 700 to 1200

    return measurementsBME280.pressure/100;
  }

}

void readAll()
{
  measurements.co2 = readSCD30co2();
  measurements.temperature = readSCD30temp();
  measurements.humidity = readSCD30hum();
  measurements.pressure = readpres();
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!
  
  pinMode(output13, OUTPUT);  
  digitalWrite(output13, LOW);
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    // while (1);
  }
  else
  {
    airSensor.setMeasurementInterval(900); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

    //My desk is ~1600m above sealevel
    airSensor.setAltitudeCompensation(300); //Set altitude of the sensor in m

    //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
    airSensor.setAmbientPressure(1000); //Current ambient pressure in mBar: 700 to 1200

    float offset = airSensor.getTemperatureOffset();
    Serial.print("Current temp offset: ");
    Serial.print(offset, 2);
    Serial.println("C");
  }

  if(!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  bmeSensor.setI2CAddress(0x77);
  if(bmeSensor.beginI2C() == false) Serial.println("Sensor connect failed");

  bmeSensor.setI2CAddress(0x76); //Connect to a second sensor
  if(bmeSensor.beginI2C() == false) Serial.println("Sensor connect failed");

  bmeSensor.setReferencePressure(102300.0f); //Adjust the sea level pressure used for altitude calculations

  //Connect to Wi-Fi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi: " + String(ssid));
    Serial.println(".");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });

  server.on("/measurements", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    const int capacity = JSON_OBJECT_SIZE(NUMMEASUREMENTS);
    StaticJsonDocument<capacity> doc;
    readAll();
    doc["co2"] = measurements.co2;
    doc["temperature"] = measurements.temperature;
    doc["humidity"] = measurements.humidity;
    doc["pressure"] = measurements.pressure;
    serializeJson(doc, *response);
    request->send(response);
  });

  // Start server
  server.begin();
}
 
void loop() {
 if (airSensor.dataAvailable()){
  if (airSensor.getCO2() > 800){
    digitalWrite(output13, HIGH);
  }
  else{
    digitalWrite(output13, LOW);
  }
  }
  delay(1000);
}

