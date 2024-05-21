#include "Serial.h"

/**
@remark: Setting up the Serial 
@return: void
*/
void set_serial(){
	Serial.begin(9600);
	while(!Serial) delay(10);
	Serial.setTimeout(1000);
}

/**
@remark: Resets the Serial 
@return: void
*/
void reset_serial(){
	Serial.end();
	Serial.begin(9600);
}

/**
@remark: Clearing the serial screen 
@return: void
*/
void clear_serial_screen(){
	// prints 100 empty lines to clear screen
	for(int i=0; i<100; i++)
		Serial.println("");
}

/**
@remark: Getting an int from serial and returns it 
@return: input
*/
uint16_t get_int_from_serial(){
	int indput = 0;
	int timeOut = 0;
	while(indput < 1 && timeOut < 30){
		indput = Serial.parseInt(SKIP_ALL);
		timeOut++;
		delay(1000);
	}
	reset_serial();
	return indput;
}

/**
@remark: Getting a string from serial and returns it 
@return: input
*/
//bool readyForCommand = false;
String get_string_from_serial(){
	//Serial.print("Enter Command: ");
	String input = "";
	while(input == ""){
		input = Serial.readString();
		input.trim();	
	}
	return input;
}

/**
@remark: Handle input for the main system 
@return: void
*/
void handle_input_main(){
	String input_raw = get_string_from_serial();
	String input = "";
	uint8_t i = 0;
	char curen_char = input_raw.charAt(0);
	while(int(curen_char) != 45){
		input += curen_char;
		i++;
		if(i == input_raw.length())
			break;
		curen_char = input_raw.charAt(i);	
	}
	i++;
	String input2 = "";
	for(;i<input_raw.length(); i++){
		input2 += input_raw.charAt(i);
	}
	if(input == "home "){
		if(input2 == "g")
			home_gripper();
		else
			home_presser();
	}
	else if(input == "close")
		grip_syring();
	else if(input == "prepare pressing")
		go_to_syringe();
	else if(input == "new test")
		drive_to_test();
	else if(input == "press ")
		press_syringe(input2.toInt());
}
