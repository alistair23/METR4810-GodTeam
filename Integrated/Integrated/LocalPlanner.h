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
	bool isGlobalPathSet(int camera);
	Point getGlobalPathEnd(int camera);
	Point getGlobalPathStart(int camera);
	Point getGlobalPoint(int camera, int index);
	std::size_t getGlobalPathLength(int camera);
	std::vector<Point> getSegment(int curr_camera, int num_points = 15);
	void updateMyCar(Point pos, double dir, double spd);
	void update(MyCar my_car, std::vector<Car> other_cars);
	int getClosest(Point& pos, std::vector<Point>& path,
		double look_ahead, int max_points_ahead = -1);
	void setPitstopPoints(int camera, Point finish_line_pos);

	std::vector<std::vector<cv::RotatedRect>> obstacles;

	// Pitstop path relative to midpoint between finish line circle markers
	std::vector<Point> source_enter_pitstop_points;
	std::vector<Point> source_exit_pitstop_points;
	std::vector<Point> enter_pitstop_points;
	std::vector<Point> exit_pitstop_points;

private:
	
	bool isValid(Point pos, long long time);

	// Returns id of car in collision given position of my car
	// Returns -1 if no collision detected
	int carInCollision(Point pos, double angle, long long time);

	// Returns true if there is collision for my car with obstacle
	bool obstacleCollision(Point pos, double angle, int camera);

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