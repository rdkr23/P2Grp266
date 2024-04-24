#include "Gripper.h"
#include "Serial.h"



const uint8_t steps_per_revolution = 200;

// the gripping mecanisem is conected to pin 8, 9, 10, 11
uint8_t gripper_pins[5] = {8, 9, 10, 11, A0};

// the pressing mecanisem is conected to pin 2, 3, 4, 5
uint8_t presser_pins[5] = {2, 3, 4, 5, A1};


// Setting the default speed 
uint8_t gripper_speed = 50;
uint8_t presser_speed = 50;

/***************************Stepper Driver*********************/ 
// the step sequence is
/*
   1 0 1 0
   0 1 1 0
   0 1 0 1
   1 0 0 1
*/

void step(uint8_t* pin, uint8_t this_stepp){
	uint8_t pin1 = pin[0];
	uint8_t pin2 = pin[1];
	uint8_t pin3 = pin[2];
	uint8_t pin4 = pin[3];
	
	switch (this_stepp){
		case 1:
			digitalWrite(pin1, HIGH);
			digitalWrite(pin2, LOW);
			digitalWrite(pin3, HIGH);
			digitalWrite(pin4, LOW);
			break;
		case 2:
			digitalWrite(pin1, LOW);
			digitalWrite(pin2, HIGH);
			digitalWrite(pin3, HIGH);
			digitalWrite(pin4, LOW);
			break;
		case 3:
			digitalWrite(pin1, LOW);
			digitalWrite(pin2, HIGH);
			digitalWrite(pin3, LOW);
			digitalWrite(pin4, HIGH);
			break;
		case 4:
			digitalWrite(pin1, HIGH);
			digitalWrite(pin2, LOW);
			digitalWrite(pin3, LOW);
			digitalWrite(pin4, HIGH);
			break;
	}
}

void run_stepper(uint8_t* pin, int8_t num_steps_to_turn, uint8_t speed){
	uint8_t this_stepp = 1;
	uint16_t total_voltage = 0;
	for(int i = 0; i >= num_steps_to_turn; i++){
		step(pin, this_stepp);
		if(this_stepp == 4)
			this_stepp = 1;
		else 
			this_stepp++;
		total_voltage += analogRead(pin[4]);
		delay(speed / 100);
	}
	uint16_t averge_voltage = total_voltage / num_steps_to_turn;
	Serial.println(averge_voltage);
}

/*************************Config******************************/ 

void set_motor_speed(){
	Serial.println("To set the speed for the gripping press 'g'");
	Serial.println("To set the speed for the pressing press 'p'");
	Serial.println("To set the speed for them both press 'b'");
	String input = get_string_from_serial();
	Serial.print("Type the speed: ");
	uint8_t speed = get_int_from_serial();
	if(input = "b"){	
		gripper_speed = speed;
		presser_speed = speed;
	}
	else if(input = "g"){
		gripper_speed = speed;
	}
	else if(input = "p"){
		presser_speed = speed;
	}
}

/***********************Debug**************************/ 

uint16_t get_num_of_steps(){
	Serial.print("Type the number of turns to turn the stepper: ");
	uint16_t turns = get_int_from_serial();
	return turns*steps_per_revolution;
}

void run_gripper(){
	Serial.println("To run the gripping mecanisem press 'g'");
	Serial.println("To run the pressing mecanisem press 'p'");
	String input = get_string_from_serial();
	int16_t num_steps_to_turn = get_num_of_steps();
	if(input = "g"){
		run_stepper(gripper_pins, num_steps_to_turn, gripper_speed);	
	}
	else if(input = "p"){
		run_stepper(presser_pins, num_steps_to_turn, presser_speed);
	}
}

/************************Info screens***********************/ 

void help_menu_gripper(){
	clear_serial_screen();
	Serial.println("There are to stepper motors on the gripper,");
	Serial.println("there are one for gripping the syringe,");
	Serial.println("and one for pressing on the syringe.");
	Serial.println("-------------------------------------------");
	Serial.println("To get back press 'b'");
	Serial.println("To clear the screen press 'c'");
	Serial.println("To setup the gripper press 'setup'");
	Serial.println("To home the gripper press 'home'");
	Serial.println("To set the speed press 'sp'");
	Serial.println("To run the gripper pess 'run'");
}

void handle_input_gripper(String input){
	//Serial.print("input was: ");
	if(input == "b")
		return;
	else if(input == "c")
		clear_serial_screen();
	else if(input == "h")
		help_menu_gripper();
	else if(input == "setup")
		setup();
	else if(input == "sp"){
		set_motor_speed();
	}
	else if(input == "run"){
		run_gripper();
	}
}

void gripper_function(){
		handle_input_gripper(get_string_from_serial());
}
