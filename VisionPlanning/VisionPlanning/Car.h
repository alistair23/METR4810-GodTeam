#ifndef _INCLUDED_CAR_H
#define _INCLUDED_CAR_H

#include "Point.h"
#include "Globals.h"

class Car {

public:

	Car();
	Car(Point pos, double dir, double vel, double length = DEFAULT_CAR_LENGTH_PIX, double width = DEFAULT_CAR_WIDTH_PIX);
	void update(Point pos, double dir, double vel);
	void setPos(Point pos);

	// Getters
	Point getPos();
	double getDir();
	double getVel();
	double getLength();
	double getWidth();
	long long getUpdateTime();

protected:

	Point pos_;				// Position in pixels
	double dir_;			// Bearing clockwise from east, in radians
	double vel_;			// Velocity in pixels/s
	double length_;			// Car length in pixels
	double width_;			// Car width in pixels
	long long update_time_;	// Time at last update

};


#endif