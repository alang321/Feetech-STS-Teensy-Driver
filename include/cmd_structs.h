#pragma once

struct cmdstruct_set_serial_id {
  uint8_t serialPort_id;
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

struct cmdstruct_get_temp {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_isMoving {
  uint8_t servo_id; //1 byte
};

struct cmdstruct_get_all {
  uint8_t servo_id; //1 byte
};

struct replystruct_get_position {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t position; // 2 byte
};

struct replystruct_get_speed {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t speed;    //2 byte
};

struct replystruct_get_load {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t load;     //1 byte
};

struct replystruct_get_supply_volt {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int8_t volt;     //1 byte
};

struct replystruct_get_temp {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int8_t temp;     //1 byte
};

struct replystruct_get_isMoving {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  bool isMoving;     //1 byte
};

struct replystruct_get_all {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t position; //2 byte
  int16_t speed;    //2 byte
  int16_t load;     //1 byte
  int8_t supply_volt;     //1 byte
  int8_t temp;     //1 byte
  bool isMoving;     //1 byte
};

