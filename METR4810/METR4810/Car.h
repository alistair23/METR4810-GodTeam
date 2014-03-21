#ifndef _INCLUDED_CAR_H
#define _INCLUDED_CAR_H

#define _USE_MATH_DEFINES

#include <math.h>

#include "Point.h"
#include "CommonFunctions.h"

class Car {

public:

	Car(Point start_pos, double start_dir, double length = 0.15, double width = 0.075);

	double getX();
	double getY();
	void step(double seconds);

	Point pos_;	// Position of car centre
	double dir_;	// Heading in radians, counter-clockwise
	double length_;
	double width_;
	double axle_length_;	// Distance between wheels in m
	double l_wheel_speed_;
	double r_wheel_speed_;


};

#endif