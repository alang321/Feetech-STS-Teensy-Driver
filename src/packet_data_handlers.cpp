#include "packet_data_handlers.h"
#include "STSServoDriver.h"
#include "packet_data_structs.h"
#include <servos.h>
#include "config.h"
#include "host_serial_comms.h"



void set_serial_id_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the buffer to the correct struct
  cmdstruct_set_serial_id cmd_set_serial_id;
  memcpy(&cmd_set_serial_id, buffer, sizeof(cmd_set_serial_id));

  #ifdef DEBUG
  Serial.print("cmd_set_serial_id.serialPort_id: ");
  Serial.println(cmd_set_serial_id.serialPort_id);
  #endif

  // initialize the servo
  servos.init(cmd_set_serial_id.serialPort_id, SERIAL_SERVOS_BAUDRATE, false);
}

void enable_servo_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the buffer to the correct struct
  cmdstruct_enable_servo cmd_enable_servo;
  memcpy(&cmd_enable_servo, buffer, sizeof(cmd_enable_servo));

  #ifdef DEBUG
  Serial.print("cmd_enable_servo.servo_id: ");
  Serial.println(cmd_enable_servo.servo_id);
  Serial.print("cmd_enable_servo.enable: ");
  Serial.println(cmd_enable_servo.enable);
  #endif

  servos.enableTorque(cmd_enable_servo.servo_id, cmd_enable_servo.enable);
}

void set_speed_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the buffer to the correct struct
  cmdstruct_set_speed cmd_set_speed;
  memcpy(&cmd_set_speed, buffer, sizeof(cmd_set_speed));

  #ifdef DEBUG
  Serial.print("cmd_set_speed.servo_id: ");
  Serial.println(cmd_set_speed.servo_id);
  Serial.print("cmd_set_speed.speed: ");
  Serial.println(cmd_set_speed.speed);
  #endif

  servos.setTargetVelocity(cmd_set_speed.servo_id, cmd_set_speed.speed);
}

void set_position_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the buffer to the correct struct
  cmdstruct_set_position cmd_set_position;
  memcpy(&cmd_set_position, buffer, sizeof(cmd_set_position));

  #ifdef DEBUG
  Serial.print("cmd_set_position.servo_id: ");
  Serial.println(cmd_set_position.servo_id);
  Serial.print("cmd_set_position.position: ");
  Serial.println(cmd_set_position.position);
  #endif
  
  servos.setTargetPosition(cmd_set_position.servo_id, cmd_set_position.position);
}

void get_speed_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the buffer to the correct struct
  cmdstruct_get_speed cmd_get_speed;
  memcpy(&cmd_get_speed, buffer, sizeof(cmd_get_speed));

  #ifdef DEBUG
  Serial.print("cmd_get_speed.servo_id: ");
  Serial.println(cmd_get_speed.servo_id);
  #endif

  //send reply
  replystruct_get_speed reply_get_speed;
  reply_get_speed.servo_id = cmd_get_speed.servo_id;
  reply_get_speed.speed = servos.getCurrentSpeed(cmd_get_speed.servo_id);

  sendData((uint8_t*) &reply_get_speed, sizeof(replystruct_get_speed));
}

void get_position_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_position cmd_get_position;
  memcpy(&cmd_get_position, buffer, sizeof(cmd_get_position));

  //send reply
  replystruct_get_position reply_get_position;
  reply_get_position.servo_id = cmd_get_position.servo_id;
  reply_get_position.position = servos.getCurrentPosition(cmd_get_position.servo_id);

  sendData((uint8_t*) &reply_get_position, sizeof(replystruct_get_position));
}

void get_load_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_load cmd_get_load;
  memcpy(&cmd_get_load, buffer, sizeof(cmd_get_load));

  //send reply
  replystruct_get_load reply_get_load;
  reply_get_load.servo_id = cmd_get_load.servo_id;
  reply_get_load.load = servos.getCurrentLoad(cmd_get_load.servo_id);

  sendData((uint8_t*) &reply_get_load, sizeof(reply_get_load));
}

void get_supply_volt_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_volt cmd_get_volt;
  memcpy(&cmd_get_volt, buffer, sizeof(cmd_get_volt));

  //send reply
  replystruct_get_supply_volt reply_get_volt;
  reply_get_volt.servo_id = cmd_get_volt.servo_id;
  reply_get_volt.volt = servos.getCurrentSupplyVoltage(cmd_get_volt.servo_id);

  sendData((uint8_t*) &reply_get_volt, sizeof(replystruct_get_supply_volt));
}

void get_temp_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_temp cmd_get_temp;
  memcpy(&cmd_get_temp, buffer, sizeof(cmd_get_temp));

  //send reply
  replystruct_get_temp reply_get_temp;
  reply_get_temp.servo_id = cmd_get_temp.servo_id;
  reply_get_temp.temp = servos.getCurrentTemperature(cmd_get_temp.servo_id);

  sendData((uint8_t*) &reply_get_temp, sizeof(replystruct_get_temp));
}

void get_isMoving_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_isMoving cmd_get_isMoving;
  memcpy(&cmd_get_isMoving, buffer, sizeof(cmd_get_isMoving));

  //send reply
  replystruct_get_isMoving reply_get_isMoving;
  reply_get_isMoving.servo_id = cmd_get_isMoving.servo_id;
  reply_get_isMoving.isMoving = servos.isMoving(cmd_get_isMoving.servo_id);

  sendData((uint8_t*) &reply_get_isMoving, sizeof(replystruct_get_isMoving));
}

void get_all_cmd_handler(uint8_t* buffer){
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_all cmd_get_all;
  memcpy(&cmd_get_all, buffer, sizeof(cmd_get_all));

  //retrieve data
  STSServoDriver::all_feedback feedback = servos.getAll(cmd_get_all.servo_id);

  #ifdef DEBUG
  Serial.print("cmd_get_all.servo_id: ");
  Serial.println(cmd_get_all.servo_id);
  Serial.print("position: ");
  Serial.println(feedback.position);
  Serial.print("speed: ");
  Serial.println(feedback.speed);
  Serial.print("load: ");
  Serial.println(feedback.load);
  Serial.print("temp: ");
  Serial.println(feedback.temperature);
  Serial.print("Voltage: ");
  Serial.println(feedback.voltage);
  #endif

  //send reply
  replystruct_get_all reply_get_all;
  reply_get_all.servo_id = cmd_get_all.servo_id;
  reply_get_all.position = feedback.position;
  reply_get_all.speed = feedback.speed;
  reply_get_all.load = feedback.load;
  reply_get_all.temp = feedback.temperature;
  reply_get_all.supply_volt = feedback.voltage;

  sendData((uint8_t*) &reply_get_all, sizeof(replystruct_get_all));
}

void set_position_async_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_position cmd_set_position_async;
  memcpy(&cmd_set_position_async, buffer, sizeof(cmd_set_position_async));

  #ifdef DEBUG
  Serial.print("cmd_set_position_async.servo_id: ");
  Serial.println(cmd_set_position_async.servo_id);
  Serial.print("cmd_set_position_async.position: ");
  Serial.println(cmd_set_position_async.position);
  #endif

  servos.setTargetPosition(cmd_set_position_async.servo_id, cmd_set_position_async.position, true);
}

void set_speed_async_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_speed cmd_set_speed_async;
  memcpy(&cmd_set_speed_async, buffer, sizeof(cmd_set_speed_async));

  #ifdef DEBUG
  Serial.print("cmd_set_speed_async.servo_id: ");
  Serial.println(cmd_set_speed_async.servo_id);
  Serial.print("cmd_set_speed_async.speed: ");
  Serial.println(cmd_set_speed_async.speed);
  #endif

  servos.setTargetVelocity(cmd_set_speed_async.servo_id, cmd_set_speed_async.speed, true);
}

void trigger_action_cmd_handler(uint8_t* buffer)
{
  servos.trigerAction();
}

void set_mode_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_mode cmd_set_mode;
  memcpy(&cmd_set_mode, buffer, sizeof(cmd_set_mode));

  #ifdef DEBUG
  Serial.print("cmd_set_mode.servo_id: ");
  Serial.println(cmd_set_mode.servo_id);
  Serial.print("cmd_set_mode.mode: ");
  Serial.println(cmd_set_mode.mode);
  #endif

  servos.setMode(cmd_set_mode.servo_id, cmd_set_mode.mode);
}

void set_motor_speed_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_motor_speed cmd;
  memcpy(&cmd, buffer, sizeof(cmd));
  
  if (cmd.motor_id == motor1 || cmd.motor_id == all_motors)
  {
    ESC1.writeMicroseconds(cmd.pwm);
  }
  if (cmd.motor_id == motor2 || cmd.motor_id == all_motors)
  {
    ESC2.writeMicroseconds(cmd.pwm);
  }
}

void set_zero_position_cmd_handler(uint8_t* buffer)
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_zero_position cmd_set_zero_position;
  memcpy(&cmd_set_zero_position, buffer, sizeof(cmd_set_zero_position));

  #ifdef DEBUG
  Serial.print("cmd_set_zero_position.servo_id: ");
  Serial.println(cmd_set_zero_position.servo_id);
  #endif

  servos.setZeroPosition(cmd_set_zero_position.servo_id);
}



