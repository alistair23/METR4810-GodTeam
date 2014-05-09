#ifndef _INCLUDED_LOCALPLANNER_H
#define _INCLUDED_LOCALPLANNER_H

#include <cv.h>			// For RotatedRect
#include <vector>

#include "Point.h"
#include "Car.h"
#include "MyCar.h"

class LocalPlanner {

public:

	LocalPlanner(std::vector<Point> global_path);
	std::vector<Point> getSegment(int num_points = 20);
	void update(MyCar my_car, std::vector<Car> other_cars);

private:
	
	int getClosest(Point& pos, std::vector<Point>& path, double look_ahead);
	bool isValid(Point pos, long long time);

	// Returns id of car in collision given position of my car
	// Returns -1 if no collision detected
	int carInCollision(Point pos, double angle, long long time);
	bool lineIntersects(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2);

	std::vector<Point> global_path_;
	std::vector<Point> prev_segment_;
	int global_index_;

	MyCar my_car_;
	std::vector<Car> other_cars_;

};


#endif