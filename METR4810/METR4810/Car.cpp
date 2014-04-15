#include "Car.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include "CommonFunctions.h"

Car::Car():
	pos_(Point(0,0)),
	dir_(0),
	length_(0),
	width_(0),
	axle_length_(0),
	l_wheel_speed_(0),
	r_wheel_speed_(0)
{
}

Car::Car(Point start_pos, double start_dir, double length, double width):
	pos_(start_pos),
	dir_(start_dir),
	length_(length),
	width_(width),
	axle_length_(width),
	l_wheel_speed_(0),
	r_wheel_speed_(0)
{
}

// Copy
Car::Car(const Car& c) {

	// Allocate variables
	Car();

	// Copy values
	operator = (c);
}

// Overload equals operator
const Car &Car::operator = (const Car& c) {

	// Copy stuff
	pos_ = c.pos_;
	dir_ = c.dir_;
	length_ = c.length_;
	width_ = c.width_;
	axle_length_ = c.axle_length_;
	l_wheel_speed_ = c.l_wheel_speed_;
	r_wheel_speed_ = c.r_wheel_speed_;

	return *this;
}


double Car::getX() {
	return pos_.x;
}

double Car::getY() {
	return pos_.y;
}

// Differential steering. Calculate new position 
// after moving a certain length of time.
// http://rossum.sourceforge.net/papers/DiffSteer/
void Car::step(double seconds) {
	double speed_diff = l_wheel_speed_ - r_wheel_speed_;
	
	// Special case for when left wheel speed == right wheel speed
	// Needed to avoid division by zero
	if (speed_diff == 0) {
		pos_.x += r_wheel_speed_ * seconds * cos(dir_);
		pos_.y += r_wheel_speed_ * seconds * sin(dir_);
	}
	else {
		double speed_sum = r_wheel_speed_ + l_wheel_speed_;
		double old_dir = dir_;
		dir_ += speed_diff * seconds / axle_length_;
		double a = axle_length_ * speed_sum * 0.5 / speed_diff;
		pos_.x += a * (sin(dir_) - sin(old_dir));
		pos_.y -= a * (cos(dir_) - cos(old_dir));

		// Ensure dir_ is between -pi and pi rad
		while (dir_ > M_PI)
			dir_ -= 2 * M_PI;
		while (dir_ < -M_PI)
			dir_ += 2 * M_PI;
	}
}