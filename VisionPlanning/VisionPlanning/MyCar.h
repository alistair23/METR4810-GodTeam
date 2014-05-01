#ifndef _INCLUDED_MYCAR_H
#define _INCLUDED_MYCAR_H

#include "Car.h"
#include "Point.h"
#include "Globals.h"

class MyCar: public Car {

public:

	MyCar();
	MyCar(Point pos, double dir, double vel, double length = DEFAULT_CAR_LENGTH_PIX,
		double width = DEFAULT_CAR_WIDTH_PIX, double axle_length_ = 0.075 / M_PER_PIX);
	
	void setRSpeed(double speed);
	void setLSpeed(double speed);
	void step(double seconds);

	double getAxleLength();
	double getRSpeed();
	double getLSpeed();

private:

	double axle_length_;	// Distance between wheels
	double r_speed_;		// Speed of right wheels in pixels/s
	double l_speed_;		// Speed of left wheels in pixels/s

};

#endif