#include "Car.h"
#include "CommonFunctions.h"

Car::Car():
	pos_(Point(0,0)),
	dir_(0),
	spd_(0),
	length_(DEFAULT_CAR_LENGTH_PIX),
	width_(DEFAULT_CAR_WIDTH_PIX),
	update_time_(0)
{
}

Car::Car(Point pos, double dir, double spd, double length, double width):
	pos_(pos),
	dir_(dir),
	spd_(spd),
	length_(length),
	width_(width)
{
	update_time_ = time_now();
}

// Copy
Car::Car(const Car& c) {

	// allocate variables
	Car();

	// copy values
	operator = (c);
}

// Overload equals operator
const Car &Car::operator = (const Car& c) {

	// Copy stuff
	pos_ = c.getPos();
	dir_ = c.getDir();
	spd_ = c.getSpd();
	length_ = c.getLength();
	width_ = c.getWidth();
	update_time_ = c.getUpdateTime();
	return *this;
}

void Car::update(Point pos, double dir, double spd) {
	pos_ = pos;
	dir_ = dir;
	spd_ = spd;
	update_time_ = time_now();
}

void Car::setPos(Point pos) {
	pos_ = pos;
}

Point Car::getPos() const {
	return pos_;
}

double Car::getDir() const {
	return dir_;
}

double Car::getSpd() const {
	return spd_;
}

double Car::getLength() const {
	return length_;
}

double Car::getWidth() const {
	return width_;
}

long long Car::getUpdateTime() const {
	return update_time_;
}