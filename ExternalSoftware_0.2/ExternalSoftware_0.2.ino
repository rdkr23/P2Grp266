#include <Wire.h>
#include "ConveyorSensor.h"
#include "Serial.h"
//#include "Gripper.h"

/**
@remark: Setup 
@return: void
*/
void setup() {
	Wire.begin();
	set_serial();
	setup_gripper();
	setup_motor_driver();
	setup_sensor();
}

/**
@remark: Loop 
@return: void
*/
void loop() {
	delay(100);
	//gripper_function();
	//run_convayor();
	handle_input_main();
	Serial.flush();
}
