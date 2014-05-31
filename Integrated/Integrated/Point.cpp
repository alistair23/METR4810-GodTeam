#define _USE_MATH_DEFINES

#include <math.h>

#include "Point.h"

using namespace std;
using namespace RaceControl;

// Default constructor
Point::Point():
	x(0),
	y(0),
	track_angle(0),
	path_num(0),
	locked(false),
	strict(false)
{}

// Copy
Point::Point(const Point& p) {

	// allocate variables
	Point();

	// copy values
	operator = (p);
}

// Overload equals operator
const Point &Point::operator = (const Point& p) {

	// Copy stuff
	x = p.x;
	y = p.y;
	track_angle = p.track_angle;
	path_num = p.path_num;
	locked = p.locked;
	strict = p.strict;

	return *this;
}

// Constructor
Point::Point(double arg_x, double arg_y, double arg_track_angle, int arg_path_num):
	x(arg_x),
	y(arg_y),
	track_angle(arg_track_angle),
	path_num(arg_path_num),
	locked(false),
	strict(false)
{}

// Returns distance to other point
double Point::dist(const Point& p) {
	double dx = p.x - x;
	double dy = p.y - y;
	return sqrt((dx * dx) + (dy * dy));
}

// Returns distance squared, faster calculation
double Point::distSquared(const Point& p) {
	double dx = p.x - x;
	double dy = p.y - y;
	return (dx * dx) + (dy * dy);
}

// Returns bearing of other point relative to right horizontal
// in radians, guaranteed < PI, > -PI
double Point::angle(const Point& p) {
	double dx = p.x - x;
	double dy = p.y - y;
	return atan2(dy, dx);
}

// Equal to another point if coordinates are equal
bool Point::equals(const Point& p) {
	return x == p.x && y == p.y;
}
