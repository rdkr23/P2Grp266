#include <Wire.h>
#include "ConveyorSensor.h"
#include "Serial.h"
//#include "Gripper.h"

void setup() {
	Wire.begin();
	set_serial();
	//setup_gripper();
	setup_motor_driver();
	setup_sensor();
}

void loop() {
	delay(100);
	//gripper_function();
	handle_input_main();
	Serial.flush();
}
