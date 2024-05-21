#ifndef CONVEYORSENSOR_H
#define CONVEYORSENSOR_H

extern const uint8_t convayor_motor_plus = 6;
extern const uint8_t convayor_motor_minus = 7;

void setup_sensor();
void setup_motor_driver();
uint16_t set_normal_range();
bool check_if_test_is_in_front(uint16_t);
void set_serial();
void run_convayor();
void drive_to_test();

#endif
