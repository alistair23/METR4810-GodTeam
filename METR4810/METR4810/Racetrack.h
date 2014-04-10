#ifndef _INCLUDED_RACETRACK_H
#define _INCLUDED_RACETRACK_H

#include <vector>

#include "Point.h"

class Racetrack {

public:

	Racetrack(std::vector<Point> midpoints = std::vector<Point>(), std::vector<double> widths = std::vector<double>());

	std::vector<Point> midpoints_;
	std::vector<double> widths_;

};

#endif