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


#include "Aquarium.h"

#define HEATER_PIN 45
#define HEATER_DIR_PIN 39
#define CO2_PIN    44
#define CONTROL_PIN A1
#define _18B20_RESOLUTION 11

#define TEMP_INC 0.1;
#define PH_INC 0.1;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

int displayPos = 0;

OneWire bus(2);
DallasTemperature temp(&bus);

long CONVERSION_DELAY = getDelay(_18B20_RESOLUTION);
long PID_INTERVAL = (CONVERSION_DELAY * 1000);

boolean converting;
long conversionStart = 0;
long lastCO2DoseTime = 0;

PID tempPid(1, 0.3, 1, 255, -255);
PID phPid(-1, -1, -1, 1, 0);

volatile double tempPIDVal;
volatile double phPIDVal;
double ph_time;

UI elements[] = {
		{ &temp_setpoint_up, &temp_setpoint_down, &temp_display },
		{ &ph_setpoint_up, &ph_setpoint_down, &ph_display },
		{ NULL, NULL }
};

LOG_OUT out_types[] = {
		{ &output_temp },
		{ &output_ph },
		{ &output_pid },
		{ &millis_out },
		{ NULL }
};

int32_t element_count = 0;

HardwareSerial* ph = &Serial3;
AtlasPH* phProbe;

boolean newTemp, newPH;
boolean phWarn = true;
String* current_command;
list_node<Command>* commands;

char comma = ',';
char space = ' ';
char colan = ':';
char hash = '#';

#define TEMP_PID_EEPROM_POS 0
#define PH_PID_EEPROM_POS sizeof(PID)

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(115200);
	Serial3.begin(38400);
	phProbe = new AtlasPH(ph);
	newTemp = false;
	newPH = false;

	delay(1000);
	for (int32_t i = 0; elements[i].setpoint_down != NULL;
			i++, element_count++) {
	}
	Serial.println(PID_INTERVAL);
	Serial.println("Start");
	Serial.print("Element count: ");
	lcd.begin(16, 2);
	Serial.println(element_count);

	_temperature = NAN;
	_ph = NAN;

	temp.setResolution(_18B20_RESOLUTION);
	temp.setWaitForConversion(false);
	temp.begin();
	pinMode(HEATER_PIN, OUTPUT);
	pinMode(HEATER_DIR_PIN, OUTPUT);
	pinMode(CO2_PIN, OUTPUT);
	Timer3.initialize(PID_INTERVAL);
	Timer3.attachInterrupt(&pid_isr, PID_INTERVAL);
	Timer3.start();
	converting = false;
	tempPid.setPoint(20);
	phPid.setPoint(7);
	elements[displayPos].display();
	current_command = new String();
	initilize_commands();

	if(!tempPid.readFromEEPROM(TEMP_PID_EEPROM_POS))
	{
		tempPid.writeToEEPROM(TEMP_PID_EEPROM_POS);
		Serial.println("Wrote Temp PID");
	}
	if(!phPid.readFromEEPROM(PH_PID_EEPROM_POS))
	{
		phPid.writeToEEPROM(PH_PID_EEPROM_POS);
		Serial.println("Wrote PH PID");
	}
	lastCO2DoseTime = 0;

}

// The loop function is called in an endless loop
void loop() {
	if (!converting) {
		bus.reset_search();
		temp.requestTemperatures();
		converting = true;
		conversionStart = millis();
	} else if ((millis() - conversionStart) > 750) {
		uint8_t address[8];
		while (bus.search(address)) {
			if (OneWire::crc8(address, 7) == address[7]) {
				switch (address[0]) {
				case 0x28:
					_temperature = temp.getTempC(address);
					newTemp = true;
					break;
				default:
					break;
				}
			}
		}
		converting = false;
	}

	if(phProbe->isConverting())
	{
		double tmp = phProbe->getValue();
		if(!isnan(tmp))
		{
			_ph = tmp;
			newPH = true;
		}
	}else
	{

		if (!isnan(_temperature)) {
//			Serial.println("__Starting Conversion");
			phProbe->setDefaultTempNoBlock(_temperature);
		}else if(phWarn)
		{
			phWarn = false;
			Serial.println("No Temperature. Not calculating PH.");
		}
	}

	//press();
	if (true) {
		if (newTemp && newPH) {
			Serial.println("BEGIN AQUARIUM");
			for (int i = 0; out_types[i].output != NULL; i++) {
				out_types[i].output();
			}
			Serial.println("END AQUARIUM");
			newPH = false;
			newTemp = false;
		}
	}

	int tmp;
	while(Serial.available())
	{
		tmp = Serial.read();
		if (tmp == 13 || tmp == 10) {
			if (current_command->length() > 0) {
				process_input(current_command);
				delete current_command;
				current_command = new String();

			}
		} else
		{
			current_command->concat((char)tmp);
		}
	}

}

void pid_isr() {

	if (!isnan(_temperature)) {
		tempPIDVal = tempPid.caculate(_temperature);
		analogWrite(HEATER_PIN, abs(int(tempPIDVal)));
		digitalWrite(HEATER_DIR_PIN, direction(tempPIDVal));
	}
	if (!isnan(_ph)) {
		phPIDVal = phPid.caculate(_ph);
		if(phPIDVal == 1 && (millis() - lastCO2DoseTime) > 10000)
		{
			digitalWrite(CO2_PIN, HIGH);
			lastCO2DoseTime = millis();
		}else if (phPIDVal != 1)
		{
			digitalWrite(CO2_PIN, LOW);
		}
	}
	Timer3.initialize(PID_INTERVAL);
}

uint8_t direction(double value) {
	return value < 0 ? HIGH : LOW;
}

boolean _press() {
	int val = analogRead(0);
	delay(5);
	if (abs(analogRead(0) - val) <= 10) {
		if (val < 50) {
			//Right
			displayPos--;
			if (displayPos < 0) {
				displayPos = element_count + displayPos;
			}
			return true;
		};
		if (val < 200) {
			//Up
			elements[displayPos].setpoint_up();
			return true;
		}
		if (val < 400) {
			//Down
			elements[displayPos].setpoint_down();
			return true;
		};
		if (val < 700) {
			//Left
			displayPos++;
			displayPos = displayPos % element_count;
			return true;
		};
		if (val < 900) {
			//Select
			return true;
		};
	}
}

void press() {
	if (_press()) {
		displayLCD();
	}
}

void displayLCD() {
	elements[displayPos].display();
	delay(100);

}

void temp_setpoint_up() {
	double v = TEMP_INC;
	tempPid.setPoint(tempPid.getPoint() + v);
}
void temp_setpoint_down() {
	double v = TEMP_INC;
	tempPid.setPoint(tempPid.getPoint() - v);
}

void temp_display() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("_C: ");
	if (!isnan(_temperature)) {
		lcd.print(_temperature);
	}
	lcd.print(" P:");
	lcd.print(int(tempPIDVal));
	lcd.setCursor(0, 1);
	lcd.print("Target: ");
	lcd.print(tempPid.getPoint());

}
void ph_setpoint_up() {
	double v = PH_INC;
	phPid.setPoint(phPid.getPoint() + v);
}
void ph_setpoint_down() {
	double v = PH_INC;
	phPid.setPoint(phPid.getPoint() - v);

}
void ph_display() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("PH: ");
	if (!isnan(_ph)) {
		lcd.print(_ph);
	}
	lcd.setCursor(0, 1);
	lcd.print("Target: ");
	lcd.print(phPid.getPoint());

}

int16_t getDelay(byte resolution) {
	switch (resolution) {
	case 9:
		return 94;
		break;
	case 10:
		return 188;
		break;
	case 11:
		return 375;
		break;
	case 12:
	default:
		return 750;
		break;
	}
}

void basic_output(char* name, double value, int value_digits, double setPoint, int set_digits) {
	Serial.print(name);
	Serial.print(comma);
	Serial.print(value, value_digits);
	Serial.print(comma);
	Serial.println(setPoint, set_digits);

}
void output_temp() {
	if (!isnan(_temperature)) {
		basic_output("temp", _temperature, 2, tempPid.getPoint(), 2);
	}
}
void output_ph() {
	if (!isnan(_ph)) {
		basic_output("ph", _ph, 2, phPid.getPoint(), 2);
	}
}

void output_pid()
{
	Serial.print("temp_pid,");
	Serial.print(tempPid.getKp(), 10);
	Serial.print(comma);
	Serial.print(tempPid.getKi(), 10);
	Serial.print(comma);
	Serial.print(tempPid.getKd(), 10);
	Serial.print(comma);
	Serial.println(tempPIDVal, 10);

	Serial.print("ph_pid,");
	Serial.print(phPid.getKp(), 10);
	Serial.print(comma);
	Serial.print(phPid.getKi(), 10);
	Serial.print(comma);
	Serial.print(phPid.getKd(), 10);
	Serial.print(comma);
	Serial.println(phPIDVal, 10);

}

void millis_out()
{
	Serial.print("millis,");
	Serial.println(millis());
}

bool process_input(String *value)
{

	bool found = false;

	value->trim();
	unsigned int length = value->length();
	char command[length + 1];
	int argc = 1;
	for(unsigned int i = 0; i < length; i++)
	{
		command[i] = (*value)[i];
		if(command[i] == ' ')
		{
			argc++;
			command[i] = 0;
		}
	}

	command[length] = 0;
	char* argv[argc];
	int pos = 0;
	for (int i = 0; i < argc; i++)
	{
		argv[i] = command + pos;
		pos += strlen(argv[i]) + 1;
	}

	list_node<Command>* tmp = commands;

	while(tmp != NULL){
		if(strcmp(argv[0], tmp->node.command) == 0)
		{
			found = true;
			tmp->node.command_callback(argv, argc);
			return true;
		}
		tmp = tmp->next;
	}

	if(!found)
	{
		Serial.println("Error");
	}
	return true;
}

void initilize_commands()
{
	list_node<Command>* tmp;
	commands = new list_node<Command>();
	commands->node.command = "temp_pid";
	commands->node.command_callback = &temp_pid_callback;
	commands->next = new list_node<Command>();
	tmp = commands->next;
	tmp->node.command = "ph_pid";
	tmp->node.command_callback = &ph_pid_callback;

//	commands->next = new list_node<Command>();
//	tmp = commands->next;
//	tmp->node.command = "ph_time";
//	tmp->node.command_callback = &ph_time_callback;

	tmp->next = NULL;
}

int temp_pid_callback(char** argv, int argc)
{
	int toRet =  pid_callback(argv, argc, &tempPid);
	if(toRet == 0)
	{
		Serial.print(colan);
		Serial.println("OK");
	}else
	{
		Serial.print(colan);
		Serial.println("ERROR");
	}
	tempPid.writeToEEPROM(TEMP_PID_EEPROM_POS);
	return toRet;

}
int ph_pid_callback(char** argv, int argc)
{
	int toRet =  pid_callback(argv, argc, &phPid);
	if(toRet == 0)
	{
		Serial.print(colan);
		Serial.println("OK");
	}else
	{
		Serial.print(colan);
		Serial.println("ERROR");
	}
	phPid.writeToEEPROM(PH_PID_EEPROM_POS);
	return toRet;
}
int pid_callback(char** argv, int argc, PID* pid)
{
	if(argc > 1)
	{
		Serial.print(hash);
		Serial.print(argv[0]);

		if (strcmp(argv[1], "set") == 0 && argc > 2) {
			Serial.print(space);
			Serial.print(argv[1]);

			if(argc > 2){
				Serial.print(space);
				Serial.print(argv[2]);
			}
			if (strcmp(argv[2], "point") == 0 && argc == 4) {
				pid->setPoint(atof(argv[3]));
				return 0;
			}else if (strcmp(argv[2], "Kp") == 0 && argc == 4) {
				pid->setKp(atof(argv[3]));
				return 0;
			}else if (strcmp(argv[2], "Ki") == 0 && argc == 4) {
				pid->setKi(atof(argv[3]));
				return 0;
			}else if (strcmp(argv[2], "Kd") == 0 && argc == 4) {
				pid->setKd(atof(argv[3]));
				return 0;
			}else if (strcmp(argv[2], "max") == 0 && argc == 4) {
				pid->setMax(atoi(argv[3]));
				return 0;
			}else if (strcmp(argv[2], "min") == 0 && argc == 4) {
				pid->setMin(atoi(argv[3]));
				return 0;
			}
		} else if (strcmp(argv[1], "get") == 0) {

		}else if (strcmp(argv[1], "reset") == 0){
			EEPROM.write(TEMP_PID_EEPROM_POS, 0);
			EEPROM.write(PH_PID_EEPROM_POS, 0);
			EEPROM.write(PH_PID_EEPROM_POS + 1, 0);
			return 0;
		}
		else
		{

		}
	}
	return -1;
}

int ph_time_callback(char** argv, int argc)
{

}

