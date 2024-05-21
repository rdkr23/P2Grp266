#include "Gripper.h"
#include "Serial.h"

// 2.28 steps per micro litter

/**
@remark: beskrivelse
@param: parameter
@param: parameter2
@return: hvad der retuneres
*/

/**
@remark: Setting up the gripper and sets the default speed for the stepper stepper 
@return: void
*/
void setup_gripper(){
	pinMode(switch_pin_presser, INPUT);
	pinMode(switch_pin_gripper, INPUT);
	pinMode(switch_pin_presser_syringe, INPUT);
	// Setting the default speed 
	gripper.setSpeed(100);
	presser.setSpeed(100);
}

/****************************Home******************************/

/**
@remark: Setting the gripper to the home position 
@return: void
*/
void home_gripper(){
	uint8_t contact = digitalRead(switch_pin_gripper);
	while(contact != 1){
		gripper.step(1);
		contact = digitalRead(switch_pin_gripper);
	}
 	Serial.print("Done");
}

/**
@remark: Setting the presser to the home position 
@return: void
*/
void home_presser(){
	presser.setSpeed(100);
	uint8_t contact = digitalRead(switch_pin_presser);
	while(contact != 0){	
		presser.step(1);
		contact = digitalRead(switch_pin_presser);
	}	
	Serial.print("Done");
}


/**
@remark: Getting g or p as a input and calling the correct function  
@return: void
*/
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
/**
@remark: Getting the presser down to the top of the syringe 
@return: void
*/
void go_to_syringe(){
	presser.setSpeed(100);
	uint8_t contact = digitalRead(switch_pin_presser_syringe);
	while(contact != 0){
		presser.step(-1);
		contact = digitalRead(switch_pin_presser_syringe);
	}
	Serial.print("Done");
}

/**
@remark: Pressing the number of micro litters out
@param: micro litter
@return: void
*/
void press_syringe(uint16_t micro_liter){
	presser.setSpeed(50);
	uint16_t step_to_take = micro_liter * 2.28;
	presser.step(-step_to_take);
	Serial.print("Done");
}


/**
@remark: Gripping the syringe 
@return: void
*/
void grip_syring(){
	gripper.step(-680);
	Serial.print("Done");
}

/*************************Config******************************/ 

/**
@remark: Getting an input and setting the speed on the correct stepper 
@return: void
*/
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

/**
@remark: Getting number of turns as input and returning number of steps 
@return: Number of steps
*/
uint16_t get_num_of_steps(){
	Serial.print("Type the number of turns to turn the stepper: ");
	uint16_t turns = get_int_from_serial();
	return turns*steps_per_revolution;
}

/**
@remark: Getting g or p as an input and turning the coresponding stepper  
@return: void
*/
void run_gripper(){
	Serial.println("To run the gripping mecanisem press 'g'");
	Serial.println("To run the pressing mecanisem press 'p'");
	String input = get_string_from_serial();
	int16_t num_steps_to_turn = get_num_of_steps();
	if(input == "g"){
		Serial.println("hi");
		gripper.step(-num_steps_to_turn);	
	}
	else if(input == "p"){
		presser.step(-num_steps_to_turn);
	}
}

/************************Info screens***********************/ 

/**
@remark: Printing info to Serial 
@return: void
*/
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
	Serial.println("To grip the syringe press 'grip'");
	Serial.println("To set the speed press 'sp'");
	Serial.println("To run the gripper pess 'run'");
}

/**
@remark: Handle input for gripper 
@param: input
@return: void
*/
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
	else if(input == "grip")
		grip_syring();
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
