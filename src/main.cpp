#include <Arduino.h>
#include "STSServoDriver.h"

STSServoDriver servos;

enum cmd_identifier { enable_driver = 0,
                      set_speed = 1,
                      set_position = 2,
                      get_speed = 3,
                      moveTo_extra_revs = 4,
                      move = 5,
                      stop = 6,
                      wiggle = 7,
                      moveTo_min_steps = 8};

void setup() {
  Serial.begin(9600);

  if (!servos.init(1))
  {
    // Failed to get a ping reply, turn on the led.
    Serial.println("Cant find servos");
  }
}

void loop()
{
  // Move servo to 0.
  Serial.println("Move to 0");
  servos.setTargetPosition(9, 0);
  servos.setTargetPosition(6, 0);
  // Wait for servo to start moving, then wait for end of motion
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  // Wait a bit more to see it stop
  delay(500);

  // Move to 180deg.
  Serial.println("Move to 180");
  servos.setTargetPosition(9, 2048);
  servos.setTargetPosition(6, 2048);
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  delay(500);

  // Move to 360deg.
  
  Serial.println("Move to 360");
  servos.setTargetPosition(9, 4095);
  servos.setTargetPosition(6, 4095);
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  delay(500);
}
