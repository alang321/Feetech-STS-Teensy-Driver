#include <Arduino.h>
#include "STSServoDriver.h"
#include "host_serial_comms.h"
#include <servos.h>
#include "config.h"

#ifdef DEBUG
unsigned long last_time = millis();
#endif

void setup() {
  digitalWrite(LED_BUILTIN, HIGH);
#ifdef DEBUG
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  Serial.println("Starting Teensy");
#endif
  
  initSerialComms();

  initServos();
}


void loop()
{
  #ifdef DEBUG
  if(receiveValidPacket())
  {
    Serial.println("Received valid packet");

    unsigned long time_since_last_message = millis() - last_time;
    Serial.print("Time since last message: ");
    Serial.println(time_since_last_message);
    last_time = millis();
  }
  #else
  receiveValidPacket();
  #endif
}


