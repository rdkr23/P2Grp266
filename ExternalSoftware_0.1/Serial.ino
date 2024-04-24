#include "Serial.h"

void set_serial(){
	Serial.begin(9600);
	while(!Serial) delay(10);
	Serial.setTimeout(1000);
}

void reset_serial(){
	Serial.end();
	Serial.begin(9600);
}

void clear_serial_screen(){
	// prints 100 empty lines to clear screen
	for(int i=0; i<100; i++)
		Serial.println("");
}

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

//bool readyForCommand = false;
String get_string_from_serial(){
	Serial.print("Enter Command: ");
	String input = "";
	while(input == ""){
		input = Serial.readString();
		input.trim();	
	}
	return input;
}
