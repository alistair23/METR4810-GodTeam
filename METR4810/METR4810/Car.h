#ifndef _INCLUDED_CAR_H
#define _INCLUDED_CAR_H

#include "Point.h"

class Car {

public:
	Car();
	Car(Point start_pos, double start_dir, double length = 0.15, double width = 0.075);
	Car(const Car& c);	// Copy
	const Car &Car::operator = (const Car& c);

	double getX();
	double getY();
	void step(double seconds);

	Point pos_;	// Position of car centre
	double dir_;	// Heading in radians, clockwise
	double length_;
	double width_;
	double axle_length_;	// Distance between wheels
	double l_wheel_speed_;
	double r_wheel_speed_;


};

#endif