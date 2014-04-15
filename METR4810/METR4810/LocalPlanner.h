#ifndef _INCLUDED_LOCALPLANNER_H
#define _INCLUDED_LOCALPLANNER_H

#include <vector>

#include "Point.h"
#include "Car.h"
#include "OtherCar.h"

class LocalPlanner {

public:

	LocalPlanner(std::vector<Point> global_path);
	std::vector<Point> getSegment(int num_points);
	void update(Car my_car);

private:
	
	
	int getClosestGlobal(Point pos);
	bool isValid(Point pos, long long time);
	bool carInCollision(Point pos, double angle, long long time);

	std::vector<OtherCar> other_cars_;
	std::vector<Point> global_path_;

	Car my_car_;

};


#endif