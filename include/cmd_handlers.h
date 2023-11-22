#pragma once

//command handlers
void set_serial_id_cmd_hanlder();
void enable_servo_cmd_hanlder();
void set_speed_cmd_hanlder();
void set_position_cmd_hanlder();
void get_speed_cmd_hanlder();
void get_position_cmd_hanlder();
void get_load_cmd_hanlder();
void get_supply_volt_cmd_hanlder();
void get_temp_cmd_hanlder();
void get_isMoving_cmd_hanlder();
void get_all_cmd_hanlder();
void set_mode_cmd_hanlder();
void set_position_async_cmd_hanlder();
void set_speed_async_cmd_hanlder();
void trigger_action_cmd_hanlder();
void set_motor_speed_cmd_hanlder();

typedef void (*cmd_handler) ();
cmd_handler serial_cmd_handlers[] = { set_serial_id_cmd_hanlder, 
                                      enable_servo_cmd_hanlder, 
                                      set_speed_cmd_hanlder,
                                      set_position_cmd_hanlder, 
                                      get_speed_cmd_hanlder, 
                                      get_position_cmd_hanlder, 
                                      get_load_cmd_hanlder, 
                                      get_supply_volt_cmd_hanlder,
                                      get_temp_cmd_hanlder,   
                                      get_isMoving_cmd_hanlder,
                                      get_all_cmd_hanlder,
                                      set_mode_cmd_hanlder,
                                      set_position_async_cmd_hanlder,
                                      set_speed_async_cmd_hanlder,
                                      trigger_action_cmd_hanlder,
                                      set_motor_speed_cmd_hanlder};

