#ifndef _INCLUDED_POINT_H
#define _INCLUDED_POINT_H

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
	Point(double x, double y);

	// Returns distance to other point
	double distance(const Point& p);

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

};

#endif