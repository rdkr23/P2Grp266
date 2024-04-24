#include <VL53L0X.h>
#include "ConveyorSensor.h"
#include "Serial.h"

VL53L0X sensor;

uint16_t set_timeout_sensor(){
	Serial.println("Type The timeout in milliseconds (0 will disable timeout)");
	Serial.print("Enter number: ");
	return get_int_from_serial();
}

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

void setup_sensor(){
	bool default_settings = true;
	if(!default_settings)
		sensor.setTimeout(set_timeout_sensor());
	else
		sensor.setTimeout(0);
	
	Serial.println("Scaning....");
	if(!sensor.init()){
		Serial.println("Failed to detect and initialize sensor");
			while(1){}
	}
	Serial.println("Done Scaning");
	
	if(!default_settings)
		sensor.setMeasurementTimingBudget(set_measurement_time_budget());
	else
		sensor.setMeasurementTimingBudget(20000);
}

uint16_t set_normal_range(){
	return sensor.readRangeSingleMillimeters();
}

bool check_if_test_is_in_front(uint16_t normal_range){
	if(sensor.readRangeSingleMillimeters() < normal_range){
		return true;
	}
	return false;
}

/*-------------------------------------------------------------------------------------------*/
// Serial


void help_menu_convayor_sensor(){
	clear_serial_screen();
	Serial.println("The Convayor sensor measure when the patch is in front of it");
	Serial.println("------------------------------------------------------------");
	Serial.println("To get back press 'b'");
	Serial.println("To clear the screen press 'c'");
	Serial.println("To set up the sensor press 'setup'");
	Serial.println("To read the measured distance press 'rd'");
}


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

void handle_input(String input){
	if(input == "h")
		help_menu_convayor_sensor();
	else if(input == "c")
		clear_serial_screen();
	else if(input == "b")
		return;
	else if (input == "setup")
		setup_sensor(20000);
	else if(input == "rd")
		print_sensor_measurement();
}

void test_VL53L0X(){
	handle_input(get_string_from_serial());
}

