#include <Arduino.h>
#include "STSServoDriver.h"
#include "cmd_structs.h"
#include "cmd_handlers.h"
#include <Servo.h>

//config
#define DEBUG
#define SERIAL_COMMS Serial4
#define SERIAL_COMMS_BAUDRATE 230400
#define DEFAULT_SERIAL_PORT_SERVOS 1
#define SERIAL_SERVOS_BAUDRATE 1000000
#define INIT_SERVOS_ON_STARTUP true

#define MOTOR1_PWM_PIN 18
#define MOTOR2_PWM_PIN 19

#define MOTOR_INITIAL_PWM 1000

//weird servo stuff
#define SERVO_MAX_COMD 4096

STSServoDriver servos;

enum cmd_identifier {
  set_serial_port = 0,
  enable_servo = 1,
  set_speed = 2,
  set_position = 3,
  get_speed = 4,
  get_position = 5,
  get_volt = 6,
  get_temp = 7,
  get_isMoving = 8,
  get_all = 9,
  set_mode = 10,
  set_position_async = 11,
  set_speed_async = 12,
  trigger_action = 13,
  set_motor_speed = 14
};

enum reply_identifier {
  reply_get_speed_id = 0,
  reply_get_position_id = 1,
  reply_get_volt_id = 2,
  reply_get_temp_id = 3,
  reply_get_isMoving_id = 4,
  reply_get_all_id = 5
};

enum motor_selectors {
  motor1 = 1,
  motor2 = 2,
  all_motors = 0
};

Servo ESC1;     // create servo object to control the ESC1
Servo ESC2;     // create servo object to control the ESC2

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
  SERIAL_COMMS.begin(SERIAL_COMMS_BAUDRATE);

  //initialize the motor pins
  ESC1.attach(MOTOR1_PWM_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(MOTOR2_PWM_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC1.writeMicroseconds(MOTOR_INITIAL_PWM);
  ESC2.writeMicroseconds(MOTOR_INITIAL_PWM);

  if(INIT_SERVOS_ON_STARTUP)
      servos.init(DEFAULT_SERIAL_PORT_SERVOS, SERIAL_SERVOS_BAUDRATE, false);
}


void loop()
{
  //measure time between received messages


  //read serial message if available
  if (SERIAL_COMMS.available() > 0)
  {
    #ifdef DEBUG
    unsigned long current_time = millis();

    unsigned long time_since_last_message = current_time - last_time;
    Serial.print("time since last message: ");
    Serial.println(time_since_last_message);

    last_time = current_time;
    #endif

    //check if there is an end marker in the data
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

void set_serial_id_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_serial_id cmd_set_serial_id;
  SERIAL_COMMS.readBytes((char*) &cmd_set_serial_id, sizeof(cmd_set_serial_id));

  #ifdef DEBUG
  Serial.print("cmd_set_serial_id.serialPort_id: ");
  Serial.println(cmd_set_serial_id.serialPort_id);
  #endif

  // initialize the servo
  servos.init(cmd_set_serial_id.serialPort_id, SERIAL_SERVOS_BAUDRATE, false);
}

void enable_servo_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_enable_servo cmd_enable_servo;
  SERIAL_COMMS.readBytes((char*) &cmd_enable_servo, sizeof(cmd_enable_servo));
  // todo: implement
}

void set_speed_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_speed cmd_set_speed;

  //read the message into a buffer
  byte serial_buffer[sizeof(cmd_set_speed) - 1];
  SERIAL_COMMS.readBytes((char*) &serial_buffer, sizeof(cmd_set_speed) - 1);

  cmd_set_speed.servo_id = serial_buffer[0];
  cmd_set_speed.speed = (serial_buffer[2] << 8) | serial_buffer[1];

  //#ifdef DEBUG
  //Serial.print("serial_buffer in hex: ");
  //for (int i = 0; i < sizeof(cmd_set_speed) - 1; i++)
  //{
  //  Serial.print(serial_buffer[i], HEX);
  //  Serial.print(" ");
  //}
  //#endif

  #ifdef DEBUG
  Serial.print("cmd_set_speed.servo_id: ");
  Serial.println(cmd_set_speed.servo_id);
  Serial.print("cmd_set_speed.speed: ");
  Serial.println(cmd_set_speed.speed);
  #endif

  //retrieve data from command struct
  int servo_id = cmd_set_speed.servo_id;
  int speed = cmd_set_speed.speed;
  servos.setTargetVelocity(servo_id, speed);
}

void set_position_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_position cmd_set_position;
  byte serial_buffer[sizeof(cmd_set_position) - 1];
  SERIAL_COMMS.readBytes((char*) &serial_buffer, sizeof(cmd_set_position) - 1);

  cmd_set_position.servo_id = serial_buffer[0];
  cmd_set_position.position = (serial_buffer[2] << 8) | serial_buffer[1];

  #ifdef DEBUG
  Serial.print("cmd_set_position.servo_id: ");
  Serial.println(cmd_set_position.servo_id);
  Serial.print("cmd_set_position.position: ");
  Serial.println(cmd_set_position.position);
  #endif

  //retrieve data from command struct
  int servo_id = cmd_set_position.servo_id;
  int position = cmd_set_position.position;
  servos.setTargetPosition(servo_id, position);
}

void get_speed_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_speed cmd_get_speed;
  SERIAL_COMMS.readBytes((char*) &cmd_get_speed, sizeof(cmd_get_speed));

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
  SERIAL_COMMS.readBytes((char*) &cmd_get_position, sizeof(cmd_get_position));

  //retrieve data from command struct
  int servo_id = cmd_get_position.servo_id;
  int position = servos.getCurrentPosition(servo_id);

  //send reply
  replystruct_get_position reply_get_position;
  reply_get_position.identifier = reply_get_position_id;
  reply_get_position.servo_id = servo_id;
  reply_get_position.position = (int16_t)((position - SERVO_MAX_COMD/2) * 36000 / SERVO_MAX_COMD);;

  SERIAL_COMMS.write((uint8_t*) &reply_get_position, sizeof(replystruct_get_position));
}

void get_volt_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_volt cmd_get_volt;
  SERIAL_COMMS.readBytes((char*) &cmd_get_volt, sizeof(cmd_get_volt));

  //retrieve data from command struct
  int servo_id = cmd_get_volt.servo_id;
  int volt = servos.getCurrentDriveVoltage(servo_id);

  //send reply
  replystruct_get_volt reply_get_volt;
  reply_get_volt.identifier = reply_get_volt_id;
  reply_get_volt.servo_id = servo_id;
  reply_get_volt.volt = volt;

  SERIAL_COMMS.write((uint8_t*) &reply_get_volt, sizeof(replystruct_get_volt));
}

void get_temp_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_temp cmd_get_temp;
  SERIAL_COMMS.readBytes((char*) &cmd_get_temp, sizeof(cmd_get_temp));

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

void get_isMoving_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_isMoving cmd_get_isMoving;
  SERIAL_COMMS.readBytes((char*) &cmd_get_isMoving, sizeof(cmd_get_isMoving));

  //retrieve data from command struct
  int servo_id = cmd_get_isMoving.servo_id;
  bool isMoving = servos.isMoving(servo_id);

  //send reply
  replystruct_get_isMoving reply_get_isMoving;
  reply_get_isMoving.identifier = reply_get_isMoving_id;
  reply_get_isMoving.servo_id = servo_id;
  reply_get_isMoving.isMoving = isMoving;

  SERIAL_COMMS.write((uint8_t*) &reply_get_isMoving, sizeof(replystruct_get_isMoving));
}

void get_all_cmd_hanlder(){
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_get_all cmg_get_all;
  SERIAL_COMMS.readBytes((char*) &cmg_get_all, sizeof(cmg_get_all));

  //retrieve data from command struct
  int servo_id = cmg_get_all.servo_id;
  int position = servos.getCurrentPosition(servo_id);
  int speed = servos.getCurrentSpeed(servo_id);
  int volt = servos.getCurrentDriveVoltage(servo_id);
  int temp = servos.getCurrentTemperature(servo_id);
  bool isMoving = servos.isMoving(servo_id);

  //send reply
  replystruct_get_all reply_get_all;
  reply_get_all.identifier = reply_get_all_id;
  reply_get_all.servo_id = servo_id;
  reply_get_all.position = position;
  reply_get_all.speed = speed;
  reply_get_all.volt = volt;
  reply_get_all.temp = temp;
  reply_get_all.isMoving = isMoving;

  SERIAL_COMMS.write((uint8_t*) &reply_get_all, sizeof(replystruct_get_all));
}

void set_position_async_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_position cmd_set_position_async;
  byte serial_buffer[sizeof(cmd_set_position_async) - 1];
  SERIAL_COMMS.readBytes((char*) &serial_buffer, sizeof(cmd_set_position_async) - 1);

  cmd_set_position_async.servo_id = serial_buffer[0];
  cmd_set_position_async.position = (serial_buffer[2] << 8) | serial_buffer[1];

  #ifdef DEBUG
  Serial.print("cmd_set_position_async.servo_id: ");
  Serial.println(cmd_set_position_async.servo_id);
  Serial.print("cmd_set_position_async.position: ");
  Serial.println(cmd_set_position_async.position);
  #endif

  //retrieve data from command struct
  int servo_id = cmd_set_position_async.servo_id;
  int position = cmd_set_position_async.position;
  servos.setTargetPosition(servo_id, position, true);
}

void set_speed_async_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_speed cmd_set_speed_async;

  //read the message into a buffer
  byte serial_buffer[sizeof(cmd_set_speed_async) - 1];
  SERIAL_COMMS.readBytes((char*) &serial_buffer, sizeof(cmd_set_speed_async) - 1);

  cmd_set_speed_async.servo_id = serial_buffer[0];
  cmd_set_speed_async.speed = (serial_buffer[2] << 8) | serial_buffer[1];

  #ifdef DEBUG
  Serial.print("cmd_set_speed_async.servo_id: ");
  Serial.println(cmd_set_speed_async.servo_id);
  Serial.print("cmd_set_speed_async.speed: ");
  Serial.println(cmd_set_speed_async.speed);
  #endif

  //retrieve data from command struct
  int servo_id = cmd_set_speed_async.servo_id;
  int speed = cmd_set_speed_async.speed;
  servos.setTargetVelocity(servo_id, speed, true);
}

void trigger_action_cmd_hanlder()
{
  servos.trigerAction();
}

void set_mode_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_mode cmd_set_mode;
  SERIAL_COMMS.readBytes((char*) &cmd_set_mode, sizeof(cmd_set_mode));

  //todo implement
}

void set_motor_speed_cmd_hanlder()
{
  // Read the command struct from the serial buffer to the correct struct
  cmdstruct_set_motor_speed cmd_set_motor_speed;
  byte serial_buffer[sizeof(cmd_set_motor_speed) - 1];
  SERIAL_COMMS.readBytes((char*) &serial_buffer, sizeof(cmd_set_motor_speed) - 1);

  cmd_set_motor_speed.motor_id = serial_buffer[0];
  cmd_set_motor_speed.pwm = (serial_buffer[2] << 8) | serial_buffer[1];

  //retrieve data from command struct
  int motor_id = cmd_set_motor_speed.motor_id;
  int pwm = cmd_set_motor_speed.pwm;
  
  if (motor_id == motor1 || motor_id == all_motors)
  {
    ESC1.writeMicroseconds(pwm);
  }
  if (motor_id == motor2 || motor_id == all_motors)
  {
    ESC2.writeMicroseconds(pwm);
  }
}



