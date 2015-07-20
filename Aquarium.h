/*
    An Arduino PID PH Controller that uses Carbon Dioxide to modulate the PH of an Aquarium.
    Copyright (C) 2013  NigelB, eResearch, James Cook Univerity

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef Aquarium_H_
#define Aquarium_H_
#include "Arduino.h"
#include "LiquidCrystal.h"
#include "TimerThree.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "PID.h"
#include "HardwareSerial.h"
#include "AtlasPH.h"



#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

struct UI {
	void (*setpoint_up)();
	void (*setpoint_down)();
	void (*display)();
};

struct LOG_OUT {
	void (*output)();
};

template<typename T> struct list_node
{
	T node;
	list_node<T>* next;
	list_node():
		next(NULL){};
};

struct Command
{
	const char* command;
	int (*command_callback)(char** argv, int argc);


	Command():
		command(NULL),
		command_callback(NULL)	{};

};

void press();
void pid_isr();
uint8_t direction(double value);
void displayLCD();
void temp_setpoint_up();
void temp_setpoint_down();
void temp_display();
void ph_setpoint_up();
void ph_setpoint_down();
void ph_display();
volatile double _temperature;
volatile double _ph;
long _last;
int16_t getDelay(byte resolution);
void initilize_commands();

void basic_output(int code, double value, int value_digits, double setPoint, int set_digits);
void output_temp();
void output_ph();
void output_pid();
void millis_out();

bool process_input(String *value);

int temp_pid_callback(char** argv, int argc);
int ph_pid_callback(char** argv, int argc);
int pid_callback(char** argv, int argc, PID* pid);
int ph_time_callback(char** argv, int argc);

#endif /* Aquarium_H_ */
