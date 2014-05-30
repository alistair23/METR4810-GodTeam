#ifndef _INCLUDED_POINT_H
#define _INCLUDED_POINT_H

#include "Globals.h"

namespace RaceControl {

class Point
{

public:

	// *******************************
	// PUBLIC FUNCTIONS
	// *******************************

	Point();
	Point(const Point& p);	// Copy
	const Point &Point::operator = (const Point& p);

	// Constructor
	Point(double x, double y, double track_angle = 0);

	// Returns distance to other point
	double dist(const Point& p);

	// Returns distance squared, faster calculation
	double distSquared(const Point& n);

	// Returns bearing of other point relative to right horizontal
	// in radians, guaranteed < PI, > -PI
	double angle(const Point& p);	

	// Equal if coordinates are equal
	bool equals(const Point& p);

	// *******************************
	// PUBLIC VARIABLES
	// *******************************

	double x;
	double y;

	// Stuff for path planning
	bool locked;		// For trajectory deformation; do not move
	bool strict;		// Should be followed strictly (true for tunnel)

	double track_angle;	// Radians, parallel to track edge
	

	// TODO
	// double curvature;
	// double speed;

};
}
#endif