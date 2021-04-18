#ifndef TASK_LEAK_DETECT
#define TASK_LEAK_DETECT

#include <Arduino.h>
#include "../config/config.h"

extern int leakStatus;
extern bool leakChanged;

short int getLeakStatus() {
    // Get sensor reading and set currentWaterLevel
    float sensorReading = 0;  // Read the analog value for the touch sensor
    short int sampleSize = 8;      // Number of samples we want to take
    for (short int i=0; i<sampleSize; i++) {  // Average samples together to minimize false readings
        sensorReading += touchRead(LEAK_TOUCH_PIN); // We sample the touch pin here
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    sensorReading /= sampleSize;
    if (sensorReading < 5) {
        return(1);
    } else {
        return(0);
    }
}

void leakSetup() {
    leakStatus = getLeakStatus();
}

void checkLeakStatus(void * parameter) {

    short int newLeakStatus = getLeakStatus();

    if (newLeakStatus != leakStatus) {
        leakStatus = newLeakStatus;
        leakChanged = true;
        Serial.print("[LEAK] Leak sensor has changed to: ");
        if (leakStatus == 1) {
            Serial.println("WET");
        } else {
            Serial.println("DRY");
        }
    }
}

#endif
