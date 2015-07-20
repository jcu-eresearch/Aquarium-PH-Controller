/*
 * PID.cpp
 *
 *  Created on: Jul 29, 2013
 *      Author: eng-nbb
 */

#include "PID.h"
#include "math.h"

PID::PID(double Kp, double Ki, double Kd)
{
	magic_number = PID_EEPROM_MAGIC_NUMBER;
	_error = 0;
	MAX = NAN;
	MIN = NAN;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->last = 0;

}
PID::PID(double Kp, double Ki, double Kd, double max, double min)
{
	magic_number = PID_EEPROM_MAGIC_NUMBER;
	_error = 0;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->last = 0;
	this->MAX = max;
	this->MIN = min;
}
void PID::setPoint(double setPoint)
{
	this->_setPoint = setPoint;
}

double PID::getPoint()
{
	return this->_setPoint;
}

double PID::caculate(double newPosition)
{
	double _p = _setPoint - newPosition ;
	_error += _p;
	double _d = _p - last;
	last = _p;

	double _pTerm = _p * Kp;
	double _iTerm = _error * Ki;
	double _dTerm = _d * Kd;
	double toRet =  _pTerm + _iTerm + _dTerm;

	if(!isnan(MAX) && toRet > MAX)
	{
		toRet = MAX;
		_error -= _p;
	}
	if(!isnan(MIN) && toRet < MIN)
	{

		toRet = MIN;
		_error -= _p;
	}
	return toRet;
}

void PID::setMax(double max)
{
	MAX = max;
}
void PID::setMin(double min)
{
	MIN = min;
}

void PID::setKp(double Kp)
{
	this->Kp = Kp;
}
void PID::setKi(double Ki)
{
	this->Ki = Ki;
}
void PID::setKd(double Kd)
{
	this->Kd = Kd;
}

double PID::getKp()
{
	return Kp;
}
double PID::getKi()
{
	return Ki;
}
double PID::getKd()
{
	return Kd;
}
boolean PID::readFromEEPROM(uint16_t pos)
{
	PID tmp(NAN, NAN, NAN, NAN, NAN);
	tmp.magic_number = 0;
	byte* ptr = (byte*)&tmp;
	for(uint16_t i = 0; i < sizeof(PID); i++)
	{
		ptr[i] = EEPROM.read(pos + i);
	}
	if(tmp.magic_number == PID_EEPROM_MAGIC_NUMBER && !isnan(tmp.Kp) && !isnan(tmp.Ki) && !isnan(tmp.Kd))
	{
		this->magic_number = tmp.magic_number;
		this->_error = 0;
		this->Kp = tmp.Kp;
		this->Ki = tmp.Ki;
		this->Kd = tmp.Kd;
		this->last = tmp.last;
		this->MAX = tmp.MAX;
		this->MIN = tmp.MIN;
		this->_setPoint = tmp._setPoint;
		return true;
	}else
	{
		Serial.println("Could not read from EEPROM.");
	}
	return false;

}
boolean PID::writeToEEPROM(uint16_t pos)
{
	double tmp_error = _error;
	_error = 0;
	byte* ptr = (byte*)this;
	for(uint16_t i = 0; i < sizeof(PID); i++)
	{
		if(EEPROM.read(pos + i) != ptr[i])
		{
			EEPROM.write(pos + i, ptr[i]);
		}
	}
	_error = tmp_error;

}


