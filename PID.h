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

#ifndef PID_H_
#define PID_H_

#include "Arduino.h"
#include "EEPROM.h"

#define PID_EEPROM_MAGIC_NUMBER 0xDEADBEEF

class PID
{
	uint32_t magic_number;
	double _error;
	double last;
	double MAX, MIN;
	double Kp, Ki, Kd;
	double _setPoint;

public:
	PID(double Kp, double Ki, double Kd);
	PID(double Kp, double Ki, double Kd, double max, double min);
	void setPoint(double setPoint);
	double getPoint();
	double caculate(double newPosition);
	void setMax(double max);
	void setMin(double min);

	void setKp(double Kp);
	void setKi(double Ki);
	void setKd(double Kd);

	double getKp();
	double getKi();
	double getKd();

	boolean readFromEEPROM(uint16_t pos);
	boolean writeToEEPROM(uint16_t pos);
};


#endif /* PID_H_ */
