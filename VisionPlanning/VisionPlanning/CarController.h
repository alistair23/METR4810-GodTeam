#ifndef _INCLUDED_CARCONTROLLER_H
#define _INCLUDED_CARCONTROLLER_H

#include <vector>

#include "Point.h"
#include "Car.h"

class CarController {

public:

	CarController();

	void setPos(); // TODO: e.g. from computer vision, update car positions
	void getPos();	// Get where controller thinks car is
	void setPath(std::vector<Point>);

	// Get the current speed commands. <left wheel, right wheel>
	std::vector<float> getSpeeds(); 

private:

	Car my_car;
	int path_index;

};

#endif