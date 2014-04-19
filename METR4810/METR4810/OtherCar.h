#ifndef _INCLUDED_OTHERCAR_H
#define _INCLUDED_OTHERCAR_H

#include "Point.h"

class OtherCar {

public:

	OtherCar(Point pos, double dir, double vel);
	void update(Point pos, double dir, double vel);

	Point pos_;
	double dir_;
	double vel_;
	double radius_;		// TODO get rid of this
	long long update_time_;	// Time at last update

};


#endif