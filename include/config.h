#pragma once

#define SERIAL_COMMS Serial2
#define SERIAL_COMMS_BAUDRATE 576000
#define DEFAULT_SERIAL_PORT_SERVOS 4
#define SERIAL_SERVOS_BAUDRATE 1000000

#define MOTOR1_PWM_PIN 0
#define MOTOR2_PWM_PIN 1

#define MOTOR_INITIAL_PWM 1000

enum motor_selectors {
  motor1 = 1,
  motor2 = 2,
  all_motors = 0
};