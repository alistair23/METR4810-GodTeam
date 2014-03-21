#ifndef _INCLUDED_RACETRACK_H
#define _INCLUDED_RACETRACK_H

#include <vector>

#include "Point.h"

class Racetrack {

public:

	Racetrack(std::vector<Point> midpoints = std::vector<Point>(), double width = 0.2);

	std::vector<Point> midpoints_;
	double width_;

};

#endif