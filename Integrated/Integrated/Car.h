#ifndef _INCLUDED_CAR_H
#define _INCLUDED_CAR_H

#include "Point.h"
#include "Globals.h"

class Car {

public:

	Car();
	Car(Point pos, double dir, double spd, double length = DEFAULT_CAR_LENGTH_PIX, double width = DEFAULT_CAR_WIDTH_PIX);
	Car(const Car& p);	// Copy
	const Car &Car::operator = (const Car& c);
	void update(Point pos, double dir, double spd);
	void setPos(Point pos);

	// Getters
	Point getPos() const;
	double getDir() const;
	double getSpd() const;
	double getLength() const;
	double getWidth() const;
	long long getUpdateTime() const;

protected:

	// Note: metres:pixel ratio defined in Globals.h

	Point pos_;				// Position in pixels
	double dir_;			// Bearing clockwise from east, in radians
	double spd_;			// Speed in pixels/s
	double length_;			// Car length in pixels
	double width_;			// Car width in pixels
	long long update_time_;	// Time at last update according to time_now() in CommonFunctions.h

};


#endif