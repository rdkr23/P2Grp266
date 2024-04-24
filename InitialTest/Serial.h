#ifndef SERIAL_H
#define SERIAL_H

void set_serial();
void reset_serial();
void clear_serial_screen();
uint16_t get_int_from_serial();
String get_string_from_serial();

#endif SERIAL_H
