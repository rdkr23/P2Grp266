#include <VL53L0X.h>
#include "ConveyorSensor.h"
#include "Serial.h"


/*************************************Sensor********************************/ 

VL53L0X sensor;

/**
@remark: Show info about setting timeout of sensor 
@return: time
*/
uint16_t set_timeout_sensor(){
	Serial.println("Type The timeout in milliseconds (0 will disable timeout)");
	Serial.print("Enter number: ");
	return get_int_from_serial();
}

/**
@remark: Gets measurement time budget 
@return: measurement time budget
*/
uint16_t set_measurement_time_budget(){
	Serial.println("Type the measurement time budget in milliseconds (longer time allows for more accuret measurements)");
	Serial.print("Enter number: ");
	int measurement_time_budget = get_int_from_serial();
	if(measurement_time_budget < 20){
		Serial.println("Error measurement timing budget can't be lower than 20 ms");
		Serial.println("Try again");
		set_measurement_time_budget();
	}
	return measurement_time_budget; 
}

/**
@remark: Setting up the sensor 
@return: void
*/
void setup_sensor(){
	sensor.setTimeout(0);
	if(!sensor.init()){
		Serial.println("Failed to detect and initialize sensor");
			while(1){}
	}
	sensor.setMeasurementTimingBudget(20000);
}

/**
@remark: Measureing the curent range 
@return: millimeters
*/
uint16_t set_normal_range(){
	return sensor.readRangeSingleMillimeters();
}

/**
@remark: Checs if a test is in front 
@return: bool
*/
bool check_if_test_is_in_front(uint16_t normal_range){
	if(sensor.readRangeSingleMillimeters() < normal_range){
		return true;
	}
	return false;
}

/********************************Convayor motor**********************************************/ 
/**
@remark: Setting up the motor for conveyor 
@return: void
*/
void setup_motor_driver(){
	pinMode(convayor_motor_plus, OUTPUT);
	pinMode(convayor_motor_minus, OUTPUT);
}

/**
@remark: Driving the conveyor forward 
@return: void
*/
void drive_convayor_motor_forward(){
	digitalWrite(convayor_motor_plus, LOW);
	digitalWrite(convayor_motor_minus, HIGH);
}

/**
@remark: Driving the conveyor backward 
@return: void
*/
void drive_convayor_motor_backward(){
	digitalWrite(convayor_motor_plus, HIGH);
	digitalWrite(convayor_motor_minus, LOW);
}

/**
@remark: Stops the conveyor 
@return: void
*/
void stop_convayor_motor(){
	digitalWrite(convayor_motor_plus, HIGH);
	digitalWrite(convayor_motor_minus, HIGH);
	delay(100);
	digitalWrite(convayor_motor_plus, LOW);
	digitalWrite(convayor_motor_minus, LOW);
}

/**
@remark: Driving the conveyor to a new test 
@return: void
*/
void drive_to_test(){
	drive_convayor_motor_forward();
	while(sensor.readRangeSingleMillimeters() < 100)
		delay(10);
	while(sensor.readRangeSingleMillimeters() > 100)
		delay(10);
	stop_convayor_motor();
	Serial.print("Done");
}

/*-------------------------------------------------------------------------------------------*/
// Serial


/**
@remark: Printing info for the conveyor system 
@return: void
*/
void help_menu_convayor_sensor(){
	clear_serial_screen();
	Serial.println("The Convayor sensor measure when the patch is in front of it");
	Serial.println("------------------------------------------------------------");
	Serial.println("To get back press 'b'");
	Serial.println("To clear the screen press 'c'");
	Serial.println("To set up the sensor press 'setup'");
	Serial.println("To read the measured distance press 'rd'");
	Serial.println("To drive to next test press 'next'");
	Serial.println("To drive motor forward press 'f'");
	Serial.println("To drive motor backward press 'b'");
	Serial.println("To stop motor press 's'");
}


/**
@remark: Printing the sensors measurement 
@return: void
*/
void print_sensor_measurement(){
	uint16_t distance = sensor.readRangeSingleMillimeters();
	if(sensor.timeoutOccurred()){
		Serial.println("Error Timeout");
		return;
	}
	Serial.print("The distance is ");
	Serial.print(sensor.readRangeSingleMillimeters());
	Serial.print("mm\n");
}

/**
@remark: Handeling input for conveyor system 
@param: input
@return: void
*/
void handle_input(String input){
	if(input == "h")
		help_menu_convayor_sensor();
	else if(input == "c")
		clear_serial_screen();
	else if(input == "b")
		return;
	else if (input == "setup")
		setup_sensor();
	else if(input == "rd")
		print_sensor_measurement();
	else if(input == "next")
		drive_to_test();
	else if(input == "f")
		drive_convayor_motor_forward();
	else if(input == "b")
		drive_convayor_motor_backward();
	else if(input == "s")
		stop_convayor_motor();
}

/**
@remark: Runs the conveyor system 
@return: void
*/
void run_convayor(){
	handle_input(get_string_from_serial());
}

