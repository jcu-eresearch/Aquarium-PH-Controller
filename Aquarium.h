// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

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
//add your includes for the project Aquarium here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project Aquarium here

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

//Do not add code below this line
#endif /* Aquarium_H_ */
