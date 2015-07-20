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
 
#include "AtlasPH.h"

const char carrage_return = '\r';

AtlasPH::AtlasPH(Stream* stream)
{
	this->stream = stream;
	time = -1;

}

void AtlasPH::enableLED()
{
	stream->print("L1");
	stream->print(carrage_return);
}

void AtlasPH::disableLED()
{
	stream->print("L0");
	stream->print(carrage_return);
}

double AtlasPH::getDouble(String* value)
{
	char _value[value->length()+1];
	_value[value->length()] = 0;
	for(unsigned int i = 0; i < value->length(); i++)
	{
		_value[i] = value->operator [](i);
	}
	return atof(_value);
}

double AtlasPH::getPH()
{
	stream->print("R");
	stream->print(carrage_return);
	String value = stream->readStringUntil(carrage_return);
	return getDouble(&value);
}

double AtlasPH::setDefaultTemp(double temp)
{
	stream->print(temp, 2);
	stream->print(carrage_return);
	String value = stream->readStringUntil(carrage_return);
	Serial.println(value);
	return getDouble(&value);
}


void AtlasPH::continuiousPH(void (*ph_callback)(void (*end)(), double ph))
{

}

//	void endContinuious();

void AtlasPH::getDrverVersion(String &version)
{
	stream->print("I");
	stream->print(carrage_return);
	String value = stream->readStringUntil(carrage_return);
	for(unsigned int i = 0; i < value.length(); i++)
	{
		version.concat(value.operator [](i));
	}
}

void AtlasPH::setDefaultTempNoBlock(double temp)
{
	if(isnan(temp))
	{
		stream->print(25.0, 2);
	}else
	{
		stream->print(temp, 2);
	}
	stream->print(carrage_return);
	stream->print('R');
	stream->print(carrage_return);
	time = millis();
}

double AtlasPH::getValue()
{
	if(isConverting() && millis() - time >= 378)
	{
		String value = stream->readStringUntil(carrage_return);
		time = -1;
		return getDouble(&value);
	}
	return NAN;
}

boolean AtlasPH::isConverting()
{
	return time > 0;
}

boolean AtlasPH::calibrate(long cal_period, char cal_val)
{
	long start = millis();
	stream->print('C');
	stream->print(carrage_return);
	while((millis() - start) < cal_period)
	{
		while(stream->available())
		{
			stream->read();
		}
	}
	stream->print(cal_val);
	stream->print(carrage_return);
}

boolean AtlasPH::calibrate4()
{
	return calibrate(ATLAS_PH_CALIBRATION_DELAY, 'F');
}

boolean AtlasPH::calibrate7()
{
	return calibrate(ATLAS_PH_CALIBRATION_DELAY, 'S');
}

boolean AtlasPH::calibrate10()
{
	return calibrate(ATLAS_PH_CALIBRATION_DELAY, 'T');
}

