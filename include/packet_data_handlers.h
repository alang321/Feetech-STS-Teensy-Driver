#pragma once

//command handlers
void set_serial_id_cmd_handler();
void enable_servo_cmd_handler();
void set_speed_cmd_handler();
void set_position_cmd_handler();
void get_speed_cmd_handler();
void get_position_cmd_handler();
void get_load_cmd_handler();
void get_supply_volt_cmd_handler();
void get_temp_cmd_handler();
void get_isMoving_cmd_handler();
void get_all_cmd_handler();
void set_mode_cmd_handler();
void set_position_async_cmd_handler();
void set_speed_async_cmd_handler();
void trigger_action_cmd_handler();
void set_motor_speed_cmd_handler();
void set_zero_position_cmd_handler();

typedef void (*cmd_handler) ();
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

