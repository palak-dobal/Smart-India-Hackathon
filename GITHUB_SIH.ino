int PulseSensorPurplePin = 5;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 32 VP
int LED13 = 2;   //  The on-board Arduion LED
int Time=0;
int Step_count = 0;
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024        // holds the incoming raw data. Signal value can range from 0-1024 

// Basic demo for accelerometer readings from Adafruit MPU6050
#include "BluetoothSerial.h"

// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4     
#define DHTTYPE    DHT11     // DHT 11

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
 
  SerialBT.begin(" Smart Knee Brace ");

dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  dht.humidity().getSensor(&sensor);

  delayMS = sensor.min_delay / 1000;

  

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:

    break;
  case MPU6050_RANGE_4_G:

    break;
  case MPU6050_RANGE_8_G:

    break;
  case MPU6050_RANGE_16_G:

    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:

    break;
  case MPU6050_RANGE_500_DEG:

    break;
  case MPU6050_RANGE_1000_DEG:
   
    break;
  case MPU6050_RANGE_2000_DEG:

    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:

    break;
  case MPU6050_BAND_184_HZ:

    break;
  case MPU6050_BAND_94_HZ:

    break;
  case MPU6050_BAND_44_HZ:

    break;
  case MPU6050_BAND_21_HZ:

    break;
  case MPU6050_BAND_10_HZ:

    break;
  case MPU6050_BAND_5_HZ:

    break;
  }

delay(100);
}

void loop() {
// Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  DynamicJsonBuffer jBuffer;
JsonObject& root = jBuffer.createObject();



  sensors_event_t event;
  dht.temperature().getEvent(&event);
    root["CurrentBodyTemperature"] = event.temperature ;
    
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);

     root["CurrentBodyHumidity"] = event.relative_humidity ;

  
  sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);

  

Signal = analogRead(PulseSensorPurplePin);
root["CurrentHeartbeat"] =Signal ;

if (a.acceleration.x >1){
  Step_count= Step_count+1;
 
}

Time = Time +1;
if( Time %10==0){
  char body[500];
root["StepsSinceLastTransmission"] = Step_count;
root.printTo(body);

SerialBT.println(body);
Step_count= 0;}
  delay(500);
}
