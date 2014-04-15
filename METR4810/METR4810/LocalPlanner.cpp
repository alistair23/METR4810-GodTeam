#include <cv.h>			// For RotatedRect

#include <math.h>

#include "LocalPlanner.h"
#include "Globals.h"

LocalPlanner::LocalPlanner(std::vector<Point> global_path):
	global_path_(global_path)
{

}

void LocalPlanner::update(Car my_car) {
	my_car_ = my_car;
}

std::vector<Point> LocalPlanner::getSegment(int num_points) {
	
	int closest_global = getClosestGlobal(my_car_.pos_);

	// If we are just behind the finish line, just start segment
	// after finish line (prevent index out of range errors)
	if (closest_global + num_points >= global_path_.size())
		closest_global = 0;

	// Copy over global points to a new segment
	std::vector<Point> segment;
	for (int i = closest_global; i < closest_global + num_points; i++) {
		segment.push_back(global_path_[i]);
	}

	// TODO Shift first point to match current?

	// Check points validity
	for (int i = 0; i < num_points; i++) {
		
	}
	
	return segment;
}

// Return the index of the point on the global path ahead 
// by look_ahead_dist.  If no points are within
// look_ahead_dist, returns index of closest point
int LocalPlanner::getClosestGlobal(Point pos) {
	double look_ahead_dist_sq = 2500;
	int i = global_path_.size() - 1;
	double dist_sq = pos.distSquared(global_path_[i]);
	double min_dist_sq = 999999;
	int min_index = 0;
	while (dist_sq > look_ahead_dist_sq && i > 0) {
		i -= 1;
		dist_sq = pos.distSquared(global_path_[i]);
		if (dist_sq < min_dist_sq) {
			min_dist_sq = dist_sq;
			min_index = i;
		}
	}
	if (i != 0)
		return i;
	else
		return min_index;
}

bool LocalPlanner::carInCollision(Point pos, double dir, long long time) {

	cv::Point2f cv_point(pos.x, pos.y);
	cv::RotatedRect rect(cv_point, cv::Size(my_car_.length_, my_car_.width_), dir);
	cv::Point2f vertices[4];
	rect.points(vertices);

	double check_dist = pow(DEFAULT_CAR_LENGTH + FRONT_CLEARANCE, 2);

	for (std::size_t i = 0; i < other_cars_.size(); i++) {
		
		// If other car is very far, no need to do further collision checks
		if (other_cars_[i].pos_.distSquared(pos) > check_dist)
			continue;

		for (std::size_t j = 0; j < 4; j++) {
			Point p(vertices[j].x, vertices[j].y);
			if (other_cars_[j].pointInCollision(p, time))
				return true;
		}
	}

	return false;

}