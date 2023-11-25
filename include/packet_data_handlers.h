#pragma once

//command handlers
#include <cstdint>

typedef void (*cmd_handler) (uint8_t* buffer);

void set_serial_id_cmd_handler(uint8_t* buffer);
void enable_servo_cmd_handler(uint8_t* buffer);
void set_speed_cmd_handler(uint8_t* buffer);
void set_position_cmd_handler(uint8_t* buffer);
void get_speed_cmd_handler(uint8_t* buffer);
void get_position_cmd_handler(uint8_t* buffer);
void get_load_cmd_handler(uint8_t* buffer);
void get_supply_volt_cmd_handler(uint8_t* buffer);
void get_temp_cmd_handler(uint8_t* buffer);
void get_isMoving_cmd_handler(uint8_t* buffer);
void get_all_cmd_handler(uint8_t* buffer);
void set_mode_cmd_handler(uint8_t* buffer);
void set_position_async_cmd_handler(uint8_t* buffer);
void set_speed_async_cmd_handler(uint8_t* buffer);
void trigger_action_cmd_handler(uint8_t* buffer);
void set_motor_speed_cmd_handler(uint8_t* buffer);
void set_zero_position_cmd_handler(uint8_t* buffer);

cmd_handler serial_cmd_handlers[] = { set_serial_id_cmd_handler, 
                                      enable_servo_cmd_handler, 
                                      set_speed_cmd_handler,
                                      set_position_cmd_handler, 
                                      get_speed_cmd_handler, 
                                      get_position_cmd_handler, 
                                      get_load_cmd_handler, 
                                      get_supply_volt_cmd_handler,
                                      get_temp_cmd_handler,   
                                      get_isMoving_cmd_handler,
                                      get_all_cmd_handler,
                                      set_mode_cmd_handler,
                                      set_position_async_cmd_handler,
                                      set_speed_async_cmd_handler,
                                      trigger_action_cmd_handler,
                                      set_motor_speed_cmd_handler,
                                      set_zero_position_cmd_handler};

