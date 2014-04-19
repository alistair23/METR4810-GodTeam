#include "OtherCar.h"
#include "CommonFunctions.h"

OtherCar::OtherCar(Point pos, double dir, double vel):
	pos_(pos),
	dir_(dir),
	vel_(vel)
{
	update_time_ = time_now();
}

void OtherCar::update(Point pos, double dir, double vel) {
	pos_ = pos;
	dir_ = dir;
	vel_ = vel;
	update_time_ = time_now();
}