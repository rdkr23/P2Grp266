#include "Gripper.h"
#include "Serial.h"

void setup_gripper(){
	pinMode(switch_pin_presser, INPUT);
	// Setting the default speed 
	gripper.setSpeed(80);
	presser.setSpeed(80);
}

/****************************Home******************************/

void home_gripper(){
	Serial.println("Getting gipper to home position");
	uint8_t contact = 0;
	digitalRead(switch_pin_gripper);
	while(contact != 1){
		contact = digitalRead(switch_pin_gripper);
		gripper.step(1);
	}
 	Serial.println("Done");
}

void home_presser(){
	Serial.println("Getting presser to home position");
	uint8_t contact = 0;
	while(contact != 1){
		contact = digitalRead(switch_pin_presser);
		presser.step(1);
	}	
	Serial.println("Done");
}

void go_to_home_pos(){
	Serial.println("To home the gripper press 'g'");
	Serial.println("To home the presser press 'p'");
	String input = get_string_from_serial();
	if(input == "g"){
		home_gripper();
	}
	else if(input == "p"){
		home_presser();
	}
	else{
		Serial.println("Input not valid");
	}
}

/*************************Pressing****************************/ 
void go_to_syringe(){
	Serial.println("Moving the presser to the top of the syringe");
	uint8_t contact = 0;
	while(contact != 1){
		contact = digitalRead(switch_pin_presser_syringe);
		presser.step(-1);
	}
	Serial.println("Done");
}

void press_syringe(){
	Serial.println("Pressing syringe");
	presser.step(-10);
	Serial.println("Done");
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
		gripper.setSpeed(speed);
		presser.setSpeed(speed);
	}
	else if(input = "g"){
		gripper.setSpeed(speed);
	}
	else if(input = "p"){
		presser.setSpeed(speed);
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
		gripper.step(num_steps_to_turn);	
	}
	else if(input = "p"){
		presser.step(num_steps_to_turn);
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
	Serial.println("To get presser to syringe press 'ps'");
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
		setup_gripper();
	else if(input == "home")
		go_to_home_pos();
	else if(input == "ps")
		go_to_syringe();
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
