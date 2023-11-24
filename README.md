# Teensy Driver for Feetech Servo Control for Teensy

This repository contains a driver for the Feetech STS Servos dpecifically designed to run on the Teensy 4.0. It uses its hardware features for the single wire serial communication so no additional Multiplexer or similar is required. The implementation for this is taken from [Teensy_actuators_PPZ](https://github.com/tudelft/Teensy_actuators_PPZ/tree/main). 

To drive the servos the Arduino STS library has been modified [STS_Servos](https://github.com/matthieuvigne/STS_servos). Some addititonaly functions such as mode control and torque enabling/disabling have been implemented.

The driver can be controlled over serial where a number of commands are implemented. The structure of these can be seen in cmd_structs.h and all the available commands can be seen in the enum at the top of main.cpp.

An example controller for this can be seen in the ROS node [squirrel_servo_control](https://github.com/alang321/squirrel_servo_control). The file teensy_python_interface.py implements easy to use functions to send and retreive data from the driver and thus the servos.

## Issues
Currently, the data read from the gfeedback functions is incorrect. Only when the getAll function is used correct feedback values can be obtained. This is due to implementation differing, this has not been implemented yet in the other feedback functions due to time pressure.

## Building and uploading

The code is compiled using the platformio extension running in VSCode. To upload simply connect the Teensy over USB and hit upload, after finishing press the program button on the Teensy. The code has been tested only on the Teensy 4.0.

## Wiring

To connect the Teensy 4.0 to the servos see the schematic in [Teensy_actuators_PPZ](https://github.com/tudelft/Teensy_actuators_PPZ/tree/main).

Briefly summarized: connect both the TX and RX pin of the chosen port together and then connect these combined pins through a 150 ohm resistor to the servo communication pin.

## Configuration

To configure which serial ports and PWM pins to use, se the defines at the top of main.cpp.