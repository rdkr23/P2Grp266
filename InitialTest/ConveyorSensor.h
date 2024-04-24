#ifndef CONVEYORSENSOR_H
#define CONVEYORSENSOR_H

void setup_sensor(uint16_t);
uint16_t set_normal_range();
bool check_if_test_is_in_front(uint16_t);
void set_serial();
void test_VL53L0X();

#endif
