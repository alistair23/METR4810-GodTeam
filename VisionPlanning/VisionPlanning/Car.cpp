#include "Car.h"
#include "CommonFunctions.h"

Car::Car():
	pos_(Point(0,0)),
	dir_(0),
	vel_(0),
	length_(DEFAULT_CAR_LENGTH_PIX),
	width_(DEFAULT_CAR_WIDTH_PIX),
	update_time_(0)
{
}

Car::Car(Point pos, double dir, double vel, double length, double width):
	pos_(pos),
	dir_(dir),
	vel_(vel),
	length_(length),
	width_(width)
{
	update_time_ = time_now();
}

void Car::update(Point pos, double dir, double vel) {
	pos_ = pos;
	dir_ = dir;
	vel_ = vel;
	update_time_ = time_now();
}

void Car::setPos(Point pos) {
	pos_ = pos;
}

Point Car::getPos() {
	return pos_;
}

double Car::getDir() {
	return dir_;
}

double Car::getVel() {
	return vel_;
}

double Car::getLength() {
	return length_;
}

double Car::getWidth() {
	return width_;
}

long long Car::getUpdateTime() {
	return update_time_;
}