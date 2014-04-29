#define _CRT_SECURE_NO_WARNINGS		// Fix for opencv bug
#define _USE_MATH_DEFINES	
#include <math.h>

#include "LocalPlanner.h"
#include "Globals.h"
#include "Vector2D.h"

LocalPlanner::LocalPlanner(std::vector<Point> global_path):
	global_path_(global_path)
{

}

void LocalPlanner::update(Car my_car, std::vector<OtherCar> other_cars) {
	my_car_ = my_car;
	other_cars_ = other_cars;
}

std::vector<Point> LocalPlanner::getSegment(int num_points) {
	
	int closest_global = getClosestGlobal(my_car_.pos_);

	// If we are just behind the finish line, just start segment
	// after finish line (prevent index out of range errors)
	if (closest_global + num_points >= global_path_.size())
		closest_global = 0;

	// TODO keep stuff from previous segment (remove front, add to back)
	// Copy over global points to a new segment
	std::vector<Point> segment;
	for (int i = closest_global; i < closest_global + num_points; i++) {
		segment.push_back(global_path_[i]);
	}

	// TODO Shift first point to match current?

	// Check points validity, record indices of points in collision
	long long timetodo = 0;
	std::vector<int> invalidPoints;

	// First point needs to be checked specially, as 
	// there is no previous point to get angle from
	if (carInCollision(segment[0], my_car_.dir_, timetodo) != -1)
		invalidPoints.push_back(0);
	for (int i = 1; i < num_points; i++) {
 		if (carInCollision(segment[i], segment[i-1].angle(segment[i]), timetodo) != -1) {
			invalidPoints.push_back(i);
		}
	}
	
	// If there are no points in collision, we go with the existing plan
	if (invalidPoints.size() == 0) 
		return segment;
	

	// Otherwise there are three options. Check each and use the 
	// best one.
	// 1. Overtake on right
	// 2. Overtake on left
	// 3. Follow behind other car

	// Cost of path. -1 means invalid path
	double right_cost = 0;
	double left_cost = 0;
	double follow_cost = 0;

	// First check overtaking right option
	// Make a copy of the segment
	std::vector<Point> right_segment = segment;
	for (std::size_t i = 0; i < invalidPoints.size(); i++) {

		// Get references to invalid segment point and corresponding global path point
		Point& seg_point = right_segment[invalidPoints[i]];
		Point& global_point = global_path_[(invalidPoints[i] + closest_global) % global_path_.size()];

		// Get gradient from point towards right track edge
		double step_size = 1;	// In pixels
		double dy = step_size * sin(seg_point.track_angle + M_PI_2);
		double dx = step_size * cos(seg_point.track_angle + M_PI_2);

		// Go along gradient until valid point is found
		double deformation = seg_point.dist(global_point);
		bool pointValid = false;
		while (!pointValid && deformation < global_point.r_edge_dist) {
			deformation += step_size;
			seg_point.x += dx;
			seg_point.y += dy;
			pointValid = (carInCollision(seg_point, global_point.track_angle, timetodo) == -1);
		}

		// If a point can't be corrected, then cannot overtake right
		//if (!pointValid) {
		//	right_cost = -1;
		//	break;
		//}

		seg_point.locked = true;
	}
	
	// Check overtaking left option
	// Make a copy of the segment
	std::vector<Point> left_segment = segment;
	for (std::size_t i = 0; i < invalidPoints.size(); i++) {

		// Get references to invalid segment point and corresponding global path point
		Point& seg_point = left_segment[invalidPoints[i]];
		Point& global_point = global_path_[(invalidPoints[i] + closest_global) % global_path_.size()];

		// Get gradient from point towards left track edge
		double step_size = 1;	// In pixels
		double dy = step_size * sin(seg_point.track_angle - M_PI_2);
		double dx = step_size * cos(seg_point.track_angle - M_PI_2);

		// Go along gradient until valid point is found
		double deformation = seg_point.dist(global_point);
		bool pointValid = false;
		while (!pointValid && deformation < global_point.l_edge_dist) {
			deformation += step_size;
			seg_point.x += dx;
			seg_point.y += dy;
			pointValid = (carInCollision(seg_point, global_point.track_angle, timetodo) == -1);
		}

		// If a point can't be corrected, then cannot overtake left
		//if (!pointValid) {
		//	left_cost = -1;
		//	break;
		//}

		seg_point.locked = true;
	}
	
	// Check following other car option, i.e. match velocity

	// Smooth out path through iterative process
	int iterations = 25;
	double change_factor = 0.25;
	for (int i = 0; i < iterations; i++) {
		for (std::size_t j = 1; j < right_segment.size(); j++) {
			if (right_segment[j].locked)
				continue;

			double force = 0;	// Acts left/right, perpendicular to track edge

			// Get references to corresponding global path point
			Point& global_point = global_path_[(j + closest_global) % global_path_.size()];

			// Get unit vector perpendicular to track edge
			double angle = global_point.track_angle + (M_PI * 0.5);
			Vector2D v(cos(angle), sin(angle));

			// Get vector from this point to the one before
			double dx = right_segment[j - 1].x - right_segment[j].x;
			double dy = right_segment[j - 1].y - right_segment[j].y;
			Vector2D u(dx, dy);

			// Get length of projection of u on v (dot product)
			double d = v.dot(u);

			force += d;

			// Repeat for force due to point after this one
			if (j != right_segment.size() - 1) {
				u.y = right_segment[j + 1].y - right_segment[j].y;
				u.x = right_segment[j + 1].x - right_segment[j].x;
				force += v.dot(u);
			}

			right_segment[j].x += force * change_factor * cos(angle);
			right_segment[j].y += force * change_factor * sin(angle);

			// TODO attractive force due to global path point
		}
	}
	
	if (right_cost != -1)
		return right_segment;


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

// Returns id of car in collision given position of my car
// Returns -1 if no collision detected
int LocalPlanner::carInCollision(Point pos, double dir, long long time) {
	dir = 0;
	cv::RotatedRect my_rect(cv::Point2f(pos.x, pos.y),
		cv::Size(my_car_.length_, my_car_.width_), dir);
	cv::Point2f my_vertices[4];
	my_rect.points(my_vertices);

	double check_dist_sq = pow((DEFAULT_CAR_LENGTH + FRONT_CLEARANCE) / M_PER_PIX, 2);

	for (std::size_t i = 0; i < other_cars_.size(); i++) {
		
		// If other car is very far, no need to do further collision checks
		if (other_cars_[i].pos_.distSquared(pos) > check_dist_sq)
			continue;

		cv::Point2f other_cv_point(other_cars_[i].pos_.x, other_cars_[i].pos_.y);
		float other_length = (DEFAULT_CAR_LENGTH + 2 * FRONT_CLEARANCE) / M_PER_PIX;
		float other_width = (DEFAULT_CAR_WIDTH + 2 * SIDE_CLEARANCE) / M_PER_PIX;
		cv::RotatedRect other_rect(other_cv_point,
			cv::Size(other_length, other_width), other_cars_[i].dir_);
		cv::Point2f other_vertices[4];
		other_rect.points(other_vertices);

		// Check for collision between all the lines defining the rectangles
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				if (lineIntersects(my_vertices[j], my_vertices[(j+1)%4],
					other_vertices[k], other_vertices[(k+1)%4]))
					return i;
			}
		}

		// Check for case where one rectangle is inside the other
		// Define mid line for other car
		cv::Point2f mid1 = (other_vertices[0] + other_vertices[1]) * 0.5;
		cv::Point2f mid2 = (other_vertices[2] + other_vertices[3]) * 0.5;
		if (lineIntersects(my_vertices[0], my_vertices[1], mid1, mid2))
			return i;
	}

	// No collision detected
	return -1;
}

// Returns true if the two lines intersect
// Taken from http://devmag.org.za/2009/04/13/basic-collision-detection-in-2d-part-1/
bool LocalPlanner::lineIntersects(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2) {
	double denom = ((b2.y - b1.y) * (a2.x - a1.x)) - ((b2.x - b1.x) * (a2.y - a1.y));
	if (denom == 0)
		return false;
	double ua = (((b2.x - b1.x) * (a1.y - b1.y)) - ((b2.y - b1.y) * (a1.x - b1.x)))/denom;
	double ub = (((a2.x - a1.x) * (a1.y - b1.y)) - ((a2.y - a1.y) * (a1.x - b1.x)))/denom;
	return (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
}
