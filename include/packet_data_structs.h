#pragma once

#include "arduino.h"

enum cmd_identifier {
  set_serial_port = 0,
  enable_servo = 1,
  set_speed = 2,
  set_position = 3,
  get_speed = 4,
  get_position = 5,
  get_load = 6,
  get_supply_volt = 7,
  get_temp = 8,
  get_isMoving = 9,
  get_all = 10,
  set_mode = 11,
  set_position_async = 12,
  set_speed_async = 13,
  trigger_action = 14,
  set_motor_speed = 15,
  set_zero_position = 16
};

//this is the max id of the commands, used for checking if the id is valid, not pretty but whatever
int cmd_id_max = 16;

//array of the command lengths, used for getting the length to read
int cmd_lengths[] = {
  sizeof(cmdstruct_set_serial_id),
  sizeof(cmdstruct_enable_servo),
  sizeof(cmdstruct_set_speed),
  sizeof(cmdstruct_set_position),
  sizeof(cmdstruct_get_speed),
  sizeof(cmdstruct_get_position),
  sizeof(cmdstruct_get_load),
  sizeof(cmdstruct_get_volt),
  sizeof(cmdstruct_get_temp),
  sizeof(cmdstruct_get_isMoving),
  sizeof(cmdstruct_get_all),
  sizeof(cmdstruct_set_mode),
  sizeof(cmdstruct_set_position),
  sizeof(cmdstruct_set_speed),
  0, //for some reason an empty struct is 1 byte, but this has to be 0 since its the trigger action command, which has no data
  sizeof(cmdstruct_set_motor_speed),
  sizeof(cmdstruct_set_zero_position)
};

enum reply_identifier {
  reply_get_speed_id = 0,
  reply_get_position_id = 1,
  reply_get_load_id = 2,
  reply_get_supply_volt_id = 3,
  reply_get_temp_id = 4,
  reply_get_isMoving_id = 5,
  reply_get_all_id = 6
};

//pack the structs so that they are byte aligned, this is important for the serial communication
//however, this will be slower than the default alignment
#pragma pack(push, 1)

struct cmdstruct_set_serial_id {
  uint8_t serialPort_id; // 1 byte
};

struct cmdstruct_enable_servo {
  uint8_t servo_id; // 1 byte
  bool enable;     //1 byte
};

struct cmdstruct_set_speed {
  uint8_t servo_id; //1 byte
  int16_t speed;    //2 byte
};

struct cmdstruct_set_position {
  uint8_t servo_id; //1 byte
  int16_t position; // 2 byte
};

struct cmdstruct_set_mode {
  uint8_t servo_id; //1 byte
  uint8_t mode;     //1 byte
};

struct cmdstruct_set_zero_position {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_set_motor_speed{
  uint8_t motor_id; //1 byte
  uint16_t pwm;    //2 byte
};

struct cmdstruct_get_position {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_speed {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_volt {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_load {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_temp {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_isMoving {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_all {
  uint8_t servo_id; //1 byte
};

struct comdstruct_trigger_action {
};

struct replystruct_get_position {
  const uint8_t identifier = reply_get_position_id; //1 byte
  uint8_t servo_id; //1 byte
  int16_t position; // 2 byte
};

struct replystruct_get_speed {
  const uint8_t identifier = reply_get_speed_id; //1 byte
  uint8_t servo_id; //1 byte
  int16_t speed;    //2 byte
};

struct replystruct_get_load {
  const uint8_t identifier = reply_get_load_id; //1 byte
  uint8_t servo_id; //1 byte
  int16_t load;     //1 byte
};

struct replystruct_get_supply_volt {
  const uint8_t identifier = reply_get_supply_volt_id; //1 byte
  uint8_t servo_id; //1 byte
  int8_t volt;     //1 byte
};

struct replystruct_get_temp {
  const uint8_t identifier = reply_get_temp_id; //1 byte
  uint8_t servo_id; //1 byte
  int8_t temp;     //1 byte
};

struct replystruct_get_isMoving {
  const uint8_t identifier = reply_get_isMoving_id; //1 byte
  uint8_t servo_id; //1 byte
  bool isMoving;     //1 byte
};

struct replystruct_get_all {
  const uint8_t identifier = reply_get_all_id; //1 byte
  uint8_t servo_id; //1 byte
  int16_t position; //2 byte
  int16_t speed;    //2 byte
  int16_t load;     //2 byte
  int8_t supply_volt;     //1 byte
  int8_t temp;     //1 byte
};

#pragma pack(pop)

//union of all the structs
union cmd_union {
  cmdstruct_set_serial_id set_serial_id;
  cmdstruct_enable_servo enable_servo;
  cmdstruct_set_speed set_speed;
  cmdstruct_set_position set_position;
  cmdstruct_set_mode set_mode;
  cmdstruct_set_zero_position set_zero_position;
  cmdstruct_set_motor_speed set_motor_speed;
  cmdstruct_get_position get_position;
  cmdstruct_get_speed get_speed;
  cmdstruct_get_volt get_volt;
  cmdstruct_get_load get_load;
  cmdstruct_get_temp get_temp;
  cmdstruct_get_isMoving get_isMoving;
  cmdstruct_get_all get_all;
};