#ifndef _INCLUDED_OTHERCAR_H
#define _INCLUDED_OTHERCAR_H

#include "Point.h"

class OtherCar {

public:

	bool pointInCollision(Point p, long long time);

	Point pos_;
	double angle_;
	double radius_;
	long long update_time_;

};


#endif