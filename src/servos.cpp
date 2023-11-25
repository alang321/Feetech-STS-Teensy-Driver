#include <servos.h>
#include "config.h"

void initServos() {
    //initialize the pwm pins
    ESC1.attach(MOTOR1_PWM_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
    ESC2.attach(MOTOR2_PWM_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC1.writeMicroseconds(MOTOR_INITIAL_PWM);
    ESC2.writeMicroseconds(MOTOR_INITIAL_PWM);

    servos.init(DEFAULT_SERIAL_PORT_SERVOS, SERIAL_SERVOS_BAUDRATE, false);
}