#define _CRT_SECURE_NO_WARNINGS		// Fix for opencv bug
#define _USE_MATH_DEFINES	
#include <math.h>

#include "LocalPlanner.h"
#include "Globals.h"
#include "Vector2D.h"

using namespace RaceControl;

LocalPlanner::LocalPlanner(int num_cameras):
	prev_camera_(-1)
{
	for (int i = 0; i < num_cameras; i++) {
		global_paths_.push_back(std::vector<Point>());
		obstacles.push_back(std::vector<cv::RotatedRect>());
	}

	pitstop_points_.push_back(Point(880, 30));
	pitstop_points_.push_back(Point(775, 185));
	pitstop_points_.push_back(Point(475, 185));
	pitstop_points_.push_back(Point(250, 185));
	pitstop_points_.push_back(Point(130, 30));
}

LocalPlanner::LocalPlanner(std::vector<std::vector<Point>> global_path):
	global_paths_(global_path)
{

}

void LocalPlanner::setGlobalPath(std::vector<Point> global_path, int camera) {
	global_paths_[camera] = global_path;
}

bool LocalPlanner::isGlobalPathSet(int camera) {
	return global_paths_[camera].size() > 0;
}

Point LocalPlanner::getGlobalPathEnd(int camera) {
	return global_paths_[camera][global_paths_[camera].size() - 1];
}

Point LocalPlanner::getGlobalPathStart(int camera) {
	return global_paths_[camera][0];
}

void LocalPlanner::updateMyCar(Point pos, double dir, double spd) {
	my_car_.update(pos, dir, spd);
}

void LocalPlanner::update(MyCar my_car, std::vector<Car> other_cars) {
	my_car_ = my_car;
	other_cars_ = other_cars;
}

std::vector<Point> LocalPlanner::getSegment(int camera, int num_points) {
	
	std::vector<Point> segment;

	// Reuse path from before (unless this is the first time)
	bool reuse_prev = false;
	/*
	if (prev_segment_.size() > 0 && prev_camera_ == camera) {

		// Get closest global point index
		int closest_global = getClosest(my_car_.getPos(), global_paths_[camera], 0);

		// Get point on previous segment a set distance lookahead of closest global
		int closest = getClosest(global_paths_[camera][closest_global], prev_segment_, 0);

		// Previous segment is valid if it is not too far from car
		// Must compare 2nd point because 1st point was shifted to car's position
		if (closest + 1 < prev_segment_.size() && 
			prev_segment_[closest + 1].dist(my_car_.getPos()) < DEFAULT_CAR_LENGTH_PIX * 0.5) {
			for (int i = closest; i < prev_segment_.size(); i++) {
				segment.push_back(prev_segment_[i]);
				segment.back().locked = false;
			}
			global_index_ += closest;
			reuse_prev = true;
		}
	}*/

	if (!reuse_prev) {
		global_index_ = getClosest(my_car_.getPos(), global_paths_[camera], 0);
		prev_camera_ = camera;
	}

	// Copy over global points to satisfy num_points
	std::size_t s = segment.size();
	for (int i = 0; i < num_points - s; i++) {
		int j = global_index_ + s + i;
		segment.push_back(global_paths_[camera][j % global_paths_[camera].size()]);
	}

	// Shift first point to match current
	segment[0].x = my_car_.getPos().x + DEFAULT_CAR_LENGTH_PIX * 0.25 * cos(my_car_.getDir());
	segment[0].y = my_car_.getPos().y + DEFAULT_CAR_LENGTH_PIX * 0.25 * sin(my_car_.getDir());
	segment[0].locked = true;

	// Check points validity, record indices of points in collision
	long long timetodo = 0;	// TODO
	std::vector<int> invalidPoints;

	// First point needs to be checked specially, as 
	// there is no previous point to get angle from
	if (carInCollision(segment[0], my_car_.getDir(), timetodo) != -1 ||
		obstacleCollision(segment[0], my_car_.getDir(), camera))
		invalidPoints.push_back(0);
	for (int i = 1; i < num_points; i++) {
 		if (carInCollision(segment[i], segment[i-1].angle(segment[i]), timetodo) != -1 ||
			obstacleCollision(segment[i], segment[i-1].angle(segment[i]), camera)) {
			invalidPoints.push_back(i);
		}
	}
	
	// If there are no points in collision, we go with the existing plan
	// TODO smoothen?
	//std::cout << invalidPoints.size() << std::endl;
	if (invalidPoints.size() == 0)  {
		prev_segment_ = segment;
		
		//return segment;
	}
	

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
	std::vector<Point> left_segment = segment;
	std::vector<Point> right_segment = segment;
	for (std::size_t i = 0; i < invalidPoints.size(); i++) {

		// Get references to invalid segment point and corresponding global path point
		Point& seg_point = right_segment[invalidPoints[i]];
		Point& global_point = global_paths_[camera][(invalidPoints[i] + global_index_) % global_paths_[camera].size()];

		// Get gradient from point towards right track edge
		double step_size = 1;	// In pixels
		double dy = step_size * sin(seg_point.track_angle + M_PI_2);
		double dx = step_size * cos(seg_point.track_angle + M_PI_2);

		// Go along gradient until valid point is found
		double deformation = seg_point.dist(global_point);
		bool pointValid = false;
		while (!pointValid && deformation < 0.5 * ROAD_WIDTH / M_PER_PIX) {
			deformation += step_size;
			seg_point.x += dx;
			seg_point.y += dy;
			pointValid = (carInCollision(seg_point, global_point.track_angle, timetodo) == -1);
		}

		// If a point can't be corrected, then cannot overtake right
		if (!pointValid) {
			right_cost = -1;
			break;
		}

		seg_point.locked = true;
	}
	
	// Check overtaking left option
	// Make a copy of the segment
	for (std::size_t i = 0; i < invalidPoints.size(); i++) {

		// Get references to invalid segment point and corresponding global path point
		Point& seg_point = left_segment[invalidPoints[i]];
		Point& global_point = global_paths_[camera][(invalidPoints[i] + global_index_) % global_paths_[camera].size()];

		// Get gradient from point towards left track edge
		double step_size = 1;	// In pixels
		double dy = step_size * sin(seg_point.track_angle - M_PI_2);
		double dx = step_size * cos(seg_point.track_angle - M_PI_2);

		// Go along gradient until valid point is found
		double deformation = seg_point.dist(global_point);
		bool pointValid = false;
		while (!pointValid && deformation < 0.5 * ROAD_WIDTH / M_PER_PIX) {
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
	
	// TODO Check following other car option, i.e. match velocity

	// TODO avoid all this copying?
	if (right_cost != -1) {
		segment = right_segment;
	}
	else if (left_cost != -1) {
		segment = left_segment;
	}

	// Smooth out path through iterative process
	int iterations = 25;
	double change_factor = 0.15;
	double global_path_attraction_factor = 0.01;
	for (int i = 0; i < iterations; i++) {
		for (std::size_t j = 1; j < segment.size(); j++) {
			if (segment[j].locked)
				continue;

			double force = 0;	// Acts left/right, perpendicular to track edge

			// Get references to corresponding global path point
			Point& global_point = global_paths_[camera][(j + global_index_) % global_paths_[camera].size()];

			// Get unit vector perpendicular to track edge
			double angle = global_point.track_angle + (M_PI * 0.5);
			Vector2D v(cos(angle), sin(angle));

			// Get vector from this point to the one before
			double dx = segment[j - 1].x - segment[j].x;
			double dy = segment[j - 1].y - segment[j].y;
			Vector2D u(dx, dy);

			// Get length of projection of u on v (dot product)
			double d = v.dot(u);

			force += d;

			// Repeat for force due to point after this one
			if (j != segment.size() - 1) {
				u.y = segment[j + 1].y - segment[j].y;
				u.x = segment[j + 1].x - segment[j].x;
				force += v.dot(u);
			}

			// Attractive force to global path point
			u.y = global_point.y - right_segment[j].y;
			u.x = global_point.x - right_segment[j].x;
			force += global_path_attraction_factor * v.dot(u);

			// Move point, but do not go beyond road
			float new_x = segment[j].x + force * change_factor * cos(angle);
			float new_y = segment[j].y + force * change_factor * sin(angle);
			//if (global_point.distSquared(Point(new_x, new_y)) < 0.5 * ROAD_WIDTH / M_PER_PIX) {
				segment[j].x = new_x;
				segment[j].y = new_y;
			//}
		}
	}

	prev_segment_ = segment;

	return segment;
}

// Return the index of a point on the path, ahead 
// by look_ahead_dist.  If look_ahead = 0, returns index
// of closest point
int LocalPlanner::getClosest(Point& pos, std::vector<Point>& path, double look_ahead) {

	// Find closest point
	double min_dist_sq = 999999;
	int min_index;
	double dist_sq;
	for (std::size_t i = 0; i < path.size(); i++) {
		dist_sq = pos.distSquared(path[i]);
		if (dist_sq < min_dist_sq) {
			min_dist_sq = dist_sq;
			min_index = i;
		}
	}
	if (look_ahead == 0)
		return min_index;
	
	// Look for point forward from closest point that
	// is at look ahead
	double look_ahead_sq = look_ahead * look_ahead;
	for (std::size_t i = min_index; i + 1 < path.size(); i++) {
		double temp = pos.distSquared(path[i + 1]);
		if (temp > look_ahead_sq)
			return i;
	}
	
	return min_index;
}

// Returns id of car in collision given position of my car
// Returns -1 if no collision detected
int LocalPlanner::carInCollision(Point pos, double dir, long long time) {
	cv::RotatedRect my_rect(cv::Point2f(pos.x, pos.y),
		cv::Size(my_car_.getLength(), my_car_.getWidth()), dir);
	cv::Point2f my_vertices[4];
	my_rect.points(my_vertices);

	double check_dist_sq = pow(DEFAULT_CAR_LENGTH_PIX + FRONT_CLEARANCE_PIX, 2);

	for (std::size_t i = 0; i < other_cars_.size(); i++) {
		
		// If other car is very far, no need to do further collision checks
		if (other_cars_[i].getPos().distSquared(pos) > check_dist_sq)
			continue;

		cv::Point2f other_cv_point(other_cars_[i].getPos().x, other_cars_[i].getPos().y);
		float other_length = other_cars_[i].getLength() + 2 * FRONT_CLEARANCE_PIX;
		float other_width = other_cars_[i].getWidth() + 2 * SIDE_CLEARANCE_PIX;
		cv::RotatedRect other_rect(other_cv_point,
			cv::Size(other_length, other_width), other_cars_[i].getDir());
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

bool LocalPlanner::obstacleCollision(Point pos, double dir, int camera) {
	cv::RotatedRect my_rect(cv::Point2f(pos.x, pos.y),
		cv::Size(my_car_.getLength(), my_car_.getWidth()), dir);
	cv::Point2f my_vertices[4];
	my_rect.points(my_vertices);

	double check_dist_sq = pow(DEFAULT_CAR_LENGTH_PIX + FRONT_CLEARANCE_PIX, 2);

	for (std::size_t i = 0; i < obstacles[camera].size(); i++) {
		
		// If obstacle is very far, no need to do further collision checks
		Point center(obstacles[camera][i].center.x, obstacles[camera][i].center.y);
		float longer_length = std::max(obstacles[camera][i].size.height,
			obstacles[camera][i].size.width);
		if (pos.distSquared(center) > my_car_.getLength() + longer_length);
			continue;

		cv::Point2f other_vertices[4];
		obstacles[camera][i].points(other_vertices);

		// Check for collision between all the lines defining the rectangles
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				if (lineIntersects(my_vertices[j], my_vertices[(j+1)%4],
					other_vertices[k], other_vertices[(k+1)%4]))
					return true;
			}
		}

		// Check for case where one rectangle is inside the other
		// Define mid line
		cv::Point2f mid1 = (other_vertices[0] + other_vertices[1]) * 0.5;
		cv::Point2f mid2 = (other_vertices[2] + other_vertices[3]) * 0.5;
		if (lineIntersects(my_vertices[0], my_vertices[1], mid1, mid2))
			return true;
	}

	// No collision detected
	return false;
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
