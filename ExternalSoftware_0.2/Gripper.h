#ifndef GRIPPER_H
#define GRIPPER_H
#include <Stepper.h>

extern uint8_t switch_pin_presser = A2;
extern uint8_t switch_pin_gripper = A1;
extern uint8_t switch_pin_presser_syringe = A0;
	
extern const uint8_t steps_per_revolution = 200;
	
// the gripping mecanisem is conected to pin 8, 9, 10, 11
extern Stepper gripper(steps_per_revolution, 10, 11, 12, 13);
	
// the pressing mecanisem is conected to pin 2, 3, 4, 5
extern Stepper presser(steps_per_revolution, 2, 3, 4, 5);
	

void setup_gripper();
void home_gripper();
void home_presser();
void go_to_syringe();
void press_syringe(uint16_t);
void grip_syring();
void set_motor_speed();
void run_stepper_for_gripping();
void run_stepper_for_presser(uint16_t);
void gripper_function();

#endif
