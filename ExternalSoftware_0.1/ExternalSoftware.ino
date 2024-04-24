#include <Wire.h>
//#include "ConveyorSensor.h"
#include "Serial.h"
#include "Gripper.h"

void setup() {
	Wire.begin();
	set_serial();
	setup_gripper();
	//setup_sensor(20000);
}

void loop() {
	delay(100);
	gripper_function();
	Serial.flush();
}
