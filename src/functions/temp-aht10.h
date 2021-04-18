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
#include <Adafruit_AHTX0.h>    // Library for AHT10 sensor
Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;

#include "../config/config.h"

extern float temp;
extern float humidity;

void setupTemp() {

    unsigned short int count = 0;
    unsigned short int retry = 10;
    while (!aht.begin()) {
      Serial.println("[TEMP] AHT10 not connected");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      if (count >= retry) {
        Serial.println("[TEMP] AHT10 connection failed");
        return;
      }
      count++;
    }

    Serial.println("AHT10 found!");
    aht_temp = aht.getTemperatureSensor();
    aht_temp->printSensorDetails();
    aht_humidity = aht.getHumiditySensor();
    aht_humidity->printSensorDetails();
}

void measureTemp() {

    unsigned short maxRetry = 10;
    unsigned short countRetry = 0;
    while(countRetry <= maxRetry) {

      // humidity = 0;
      // temp = 0;
      countRetry++;

      //  /* Get a new normalized sensor event */
      sensors_event_t event_humidity, event_temp;
      aht_humidity->getEvent(&event_humidity);
      aht_temp->getEvent(&event_temp);

      Serial.println("[TEMP] Measuring...");

      temp = event_temp.temperature;
      humidity = event_humidity.relative_humidity;

      // Range checking
      if (humidity < 5 || humidity > 80) {
        Serial.print("[TEMP] humidity out of bounds: ");
        Serial.println(humidity);
        humidity = 0;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }
      if (temp < -6 || temp > 55) {
        Serial.print("[TEMP] temperature (C) out of bounds: ");
        Serial.println(temp);
        temp = 0;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }

      // temp = ((float )((int)(temp * 10))) / 10;  // truncate to 1 decimal

      temp = ((temp * 9)/5) + 32;  // convert to fahrenheit
      temp = (float(int(temp * 10)) / 10);  // truncate to 1 decimal

      // Serial.print(F("[TEMP]: "));
      // Serial.print(temp);
      // Serial.print(F("ºF, "));
      // Serial.print(humidity);
      // Serial.print(F("%, "));
      // Serial.print(pressure);
      // Serial.print(F(" hPa"));
      // Serial.println("[TEMP] finished measuring");
      break;
    }
}

#endif
