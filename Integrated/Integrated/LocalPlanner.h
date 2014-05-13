#ifndef _INCLUDED_LOCALPLANNER_H
#define _INCLUDED_LOCALPLANNER_H

#include <cv.h>			// For RotatedRect
#include <vector>

#include "Point.h"
#include "Car.h"
#include "MyCar.h"

namespace RaceControl {

class LocalPlanner {

public:
	
	LocalPlanner(int num_cameras = 4);
	LocalPlanner(std::vector<std::vector<Point>> global_paths);
	void setGlobalPath(std::vector<Point> global_path, int camera);
	std::vector<Point> getSegment(int curr_camera, int num_points = 20);
	void updateMyCar(Point pos, double dir, double spd);
	void update(MyCar my_car, std::vector<Car> other_cars);

private:
	
	int getClosest(Point& pos, std::vector<Point>& path, double look_ahead);
	bool isValid(Point pos, long long time);

	// Returns id of car in collision given position of my car
	// Returns -1 if no collision detected
	int carInCollision(Point pos, double angle, long long time);
	bool lineIntersects(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2);

	std::vector<std::vector<Point>> global_paths_;
	std::vector<Point> prev_segment_;
	int prev_camera_;
	int global_index_;

	MyCar my_car_;
	std::vector<Car> other_cars_;

};

}
#endif