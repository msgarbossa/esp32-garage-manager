// #define CLOSED LOW	
// #define OPEN HIGH

#include <Arduino.h>
#include "../config/config.h"

extern bool doorChanged;
extern int doorStatus;

short int getDoorStatus() {
    // digitalRead returns 1/HIGH/OPEN or 0/LOW/CLOSED
    // LOW = magnetic switch connected = door closed
    // HIGH = magentic switch broken = door open
    short int status = digitalRead(GARAGE_MAG_SWITCH_PIN);
    return status;
}

void monitorDoorSetup() {
    // pinMode(GARAGE_RELAY_PIN, INPUT_PULLUP);  // for the odd relay triggered by grounding the input pin
    pinMode(GARAGE_RELAY_PIN, OUTPUT);  // for stardard relay (triggered w/ 3.3V)
    pinMode(GARAGE_MAG_SWITCH_PIN, INPUT_PULLUP);
    doorStatus = getDoorStatus();
}

void toggleGarageRelay() {
    Serial.println("Garage relay toggle");
    // pinMode(GARAGE_RELAY_PIN, OUTPUT);  // for the odd relay triggered by grounding the input pin
    digitalWrite(GARAGE_RELAY_PIN, HIGH); // turn on the relay (standard relay triggered w/ 3.3V)
    vTaskDelay(300 / portTICK_PERIOD_MS);
    // pinMode(GARAGE_RELAY_PIN, INPUT_PULLUP);  // for the odd relay triggered by grounding the input pin
    digitalWrite(GARAGE_RELAY_PIN, LOW);  // turn off the relay (standard relay triggered w/ 3.3V)
}

void monitorDoorState() {
    short int newDoorStatus = getDoorStatus();
    if (newDoorStatus != doorStatus) {
        doorStatus = newDoorStatus;
        doorChanged = true;
        Serial.print("[GARAGE] Door has changed to: ");
        if (doorStatus == LOW) {
            Serial.println("CLOSED");
        } else {
            Serial.println("OPEN");
        }
    }
}
