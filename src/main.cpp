#include <Arduino.h>
#include "STSServoDriver.h"

#define DEBUG
#define SERIAL_COMMS Serial4


STSServoDriver servos;

enum cmd_identifier {
  add_servo = 0,
  enable_servo = 1,
  set_speed = 1,
  set_position = 2,
  get_speed = 3,
  get_position = 4,
  get_volt = 5,
  get_temp = 6
};

enum reply_identifier {
  reply_get_speed_id = 0,
  reply_get_position_id = 1,
  reply_get_volt_id = 2,
  reply_get_temp_id = 3
};

struct cmdstruct_add_servo {
  uint8_t servo_id;
  uint8_t serialPort_id;
};

struct cmdstruct_enable_driver {
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

struct replystruct_get_position {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t position; // 1 byte
};

struct replystruct_get_speed {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int16_t speed;    //1 byte
};

struct replystruct_get_volt {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int8_t volt;     //1 byte
};

struct replystruct_get_temp {
  uint8_t identifier; //1 byte
  uint8_t servo_id; //1 byte
  int8_t temp;     //1 byte
};

//command handlers
void add_servo_cmd_hanlder();
void enable_servo_cmd_hanlder();
void set_speed_cmd_hanlder();
void set_position_cmd_hanlder();
void get_speed_cmd_hanlder();
void get_position_cmd_hanlder();
void get_position_cmd_hanlder();
void get_volt_cmd_hanlder();
void get_temp_cmd_hanlder();

typedef void (*cmd_handler) ();
cmd_handler serial_cmd_handlers[] = {add_servo_cmd_hanlder, enable_servo_cmd_hanlder, set_speed_cmd_hanlder, set_position_cmd_hanlder, get_speed_cmd_hanlder, get_position_cmd_hanlder, get_volt_cmd_hanlder, get_temp_cmd_hanlder};

// array of servo ids, dynamically sized at runtime
int* servo_ids = NULL;
int num_servos = 0;


void setup() {
#ifdef DEBUG
  delay(6000);
  digitalWriteFast(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  Serial.println("Starting Teensy message");
#endif
}


void loop()
{
  //read serial message if available
  if (SERIAL_COMMS.available() > 0)
  {
    #ifdef DEBUG
    Serial.println("received message");
    #endif

    // Read the command struct from the serial buffer
    uint8_t cmd_id = 0;
    SERIAL_COMMS.readBytes((char*) &cmd_id, 1);

    #ifdef DEBUG
    Serial.print("cmd_id: ");
    Serial.println(cmd_id);
    #endif

    //call the correct handler
    serial_cmd_handlers[cmd_id]();
  }
}

void add_servo_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_add_servo cmd_add_servo;
  SERIAL_COMMS.readBytes((char*) &cmd_add_servo, sizeof(cmdstruct_add_servo));

  // add the servo id to the array of servo ids
  servo_ids = (int*) realloc(servo_ids, (num_servos + 1) * sizeof(int));
  servo_ids[num_servos] = cmd_add_servo.servo_id;
  num_servos++;

  // initialize the servo
  servos.init(cmd_add_servo.serialPort_id, 1000000, true);
}

void enable_servo_cmd_hanlder()
{
  //not implemeneted yet
}

void set_speed_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_speed cmd_set_speed;
  SERIAL_COMMS.readBytes((char*) &cmd_set_speed, sizeof(cmdstruct_add_servo));

  //retrieve data from command struct
  int servo_id = cmd_set_speed.servo_id;
  int speed = cmd_set_speed.speed;
  servos.setTargetVelocity(servo_id, speed);
}

void set_position_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_position cmd_set_position;
  SERIAL_COMMS.readBytes((char*) &cmd_set_position, sizeof(cmdstruct_add_servo));

  //retrieve data from command struct
  int servo_id = cmd_set_position.servo_id;
  int position = cmd_set_position.position;
  servos.setTargetPosition(servo_id, position);
}

void get_speed_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_speed cmd_get_speed;
  SERIAL_COMMS.readBytes((char*) &cmd_get_speed, sizeof(cmdstruct_add_servo));

  //retrieve data from command struct
  int servo_id = cmd_get_speed.servo_id;
  int speed = servos.getCurrentSpeed(servo_id);

  //send reply
  replystruct_get_speed reply_get_speed;
  reply_get_speed.identifier = reply_get_speed_id;
  reply_get_speed.servo_id = servo_id;
  reply_get_speed.speed = speed;

  SERIAL_COMMS.write((uint8_t*) &reply_get_speed, sizeof(replystruct_get_speed));
}

void get_position_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_position cmd_get_position;
  SERIAL_COMMS.readBytes((char*) &cmd_get_position, sizeof(cmdstruct_add_servo));

  //retrieve data from command struct
  int servo_id = cmd_get_position.servo_id;
  int position = servos.getCurrentPosition(servo_id);

  //send reply
  replystruct_get_position reply_get_position;
  reply_get_position.identifier = reply_get_position_id;
  reply_get_position.servo_id = servo_id;
  reply_get_position.position = position;

  SERIAL_COMMS.write((uint8_t*) &reply_get_position, sizeof(replystruct_get_position));
}

void get_volt_cmd_hanlder()
{
  //not yet implemented
}

void get_temp_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_temp cmd_get_temp;
  SERIAL_COMMS.readBytes((char*) &cmd_get_temp, sizeof(cmdstruct_add_servo));

  //retrieve data from command struct
  int servo_id = cmd_get_temp.servo_id;
  int temp = servos.getCurrentTemperature(servo_id);

  //send reply
  replystruct_get_temp reply_get_temp;
  reply_get_temp.identifier = reply_get_temp_id;
  reply_get_temp.servo_id = servo_id;
  reply_get_temp.temp = temp;

  SERIAL_COMMS.write((uint8_t*) &reply_get_temp, sizeof(replystruct_get_temp));
}






