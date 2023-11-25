#pragma once

#include "arduino.h"
#include <Servo.h>
#include <STSServoDriver.h>

STSServoDriver servos;

Servo ESC1;     // create servo object to control the ESC1
Servo ESC2;     // create servo object to control the ESC2

void initServos();