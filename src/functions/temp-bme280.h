#ifndef TASK_MEASURE_TEMP
#define TASK_MEASURE_TEMP

/*
- = ground
+ = VIN (+3.3V)
SDA = D21
SCL = D22
*/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// I2C device found at address 0x38

#include "../config/config.h"


Adafruit_BME280 bme;

extern float temp;
extern float humidity;
extern float pressure;


void setupTemp() {

    unsigned short int count = 0;
    unsigned short int retry = 10;
    while (!bme.begin(0x76)) {
      Serial.println(F("BME280 not connected"));
      delay(1000);
      if (count >= retry) {
        Serial.println("[TEMP] BME280 connection failed");
        return;
      }
      count++;
    }

    Serial.println("BME280 found!");
}

void measureTemp() {

    unsigned short maxRetry = 10;
    unsigned short countRetry = 0;
    while(countRetry <= maxRetry) {

      countRetry++;

      temp = bme.readTemperature();
      humidity = bme.readHumidity();
      pressure = bme.readPressure() / 100.0F;

      // Range checking
      if (humidity < 5 || humidity > 80) {
        Serial.print(F("[TEMP] humidity out of bounds: "));
        Serial.println(humidity);
        humidity = 0;
        delay(2000);
        continue;
      }
      if (temp < -6 || temp > 55) {
        Serial.print(F("[TEMP] temperature (C) out of bounds: "));
        Serial.println(temp);
        temp = 0;
        delay(2000);
        continue;
      }

      temp = ((temp * 9)/5) + 32;  // convert to fahrenheit
      temp = (float(int(temp * 10)) / 10);  // truncate to 1 decimal

      // Serial.print(F("[TEMP]: "));
      // Serial.print(temp);
      // Serial.print(F("ºF, "));
      // Serial.print(humidity);
      // Serial.print(F("%, "));
      // Serial.print(pressure);
      // Serial.print(F(" hPa"));
      break;

    }
}

#endif
