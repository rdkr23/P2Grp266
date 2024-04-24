#ifndef GRIPPER_H
#define GRIPPER_H
#include <Stepper.h>

extern uint8_t switch_pin_presser = A0;
extern uint8_t switch_pin_gripper = A1;
extern uint8_t switch_pin_presser_syringe = A2;
	
extern const uint8_t steps_per_revolution = 200;
	
// the gripping mecanisem is conected to pin 8, 9, 10, 11
extern Stepper gripper(steps_per_revolution, 8, 9, 10, 11);
	
// the pressing mecanisem is conected to pin 2, 3, 4, 5
extern Stepper presser(steps_per_revolution, 2, 3, 4, 5);
	

void setup_gripper();
void set_motor_speed();
void run_stepper_for_gripping();
void run_stepper_for_presser(uint16_t);
void gripper_function();

#endif
