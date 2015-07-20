/*
 * PID.h
 *
 *  Created on: Jul 29, 2013
 *      Author: eng-nbb
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
