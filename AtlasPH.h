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

#ifndef ATLASPH_H_
#define ATLASPH_H_

#include "Arduino.h"

#define ATLAS_PH_CONVERSION_DELAY 378
#define ATLAS_PH_CALIBRATION_DELAY 120000

class AtlasPH
{
private:
	Stream* stream;
	double getDouble(String* value);
	long time;

	boolean calibrate(long cal_period, char cal_val);

public:

	AtlasPH(Stream* stream);
	void enableLED();
	void disableLED();
	/*
	 * The Atlas PH driver will calculate the PH using the current default temperature
	 */
	double getPH();

	/*
	 * Sets the new default temperature and retrieves a new PH calculation using this new temperature
	 */
	double setDefaultTemp(double temp);

	void setDefaultTempNoBlock(double temp);
	double getValue();

	boolean isConverting();

	/*
	 * Calculates the PH every 378milliseconds, until the end callback is called
	 */
	void continuiousPH(void (*ph_callback)(void (*end)(), double ph));

//	void endContinuious();

	void getDrverVersion(String &version);


	boolean calibrate4();
	boolean calibrate7();
	boolean calibrate10();
};

#endif /* ATLASPH_H_ */
