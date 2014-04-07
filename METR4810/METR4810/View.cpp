#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>

#include "View.h"

View::View(Racetrack& racetrack, Car& my_car):
	racetrack_(racetrack),
	my_car_(my_car),
	scale_(250),	// pixels / metres
	frame_time_(20),	// milliseconds / frame
	map_width_(4.),	// metres
	map_height_(2.),	// metres
	background_(map_height_ * scale_, map_width_ * scale_, CV_8UC3, cv::Scalar(255,255,255))
{

	// Create display window
	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);

	// Draw racetrack
	for (std::size_t i = 0; i < racetrack.midpoints_.size(); i++) {
		Point& p1(racetrack.midpoints_[i]);
		Point& p2(racetrack.midpoints_[(i + 1) % racetrack.midpoints_.size()]);
		cv::line(background_,
			cv::Point(scale_ * p1.x, scale_ * (map_height_ - p1.y)),
			cv::Point(scale_ * p2.x, scale_ * (map_height_ - p2.y)),
			cv::Scalar(20, 20, 20), scale_ * racetrack.width_);
	}

	// Draw dots at midpoints
	for (std::size_t i = 0; i < racetrack.midpoints_.size(); i++) {
		Point& p1(racetrack.midpoints_[i]);
		cv::circle(background_, cv::Point(scale_ * p1.x,
			scale_ * (map_height_ - p1.y)), 2, cv::Scalar(100, 50, 50), -1);
	}

	redraw();
}

void View::spin() {
	redraw();
	cv::waitKey(frame_time_);
	spin();
}

void View::redraw() {

	cv::Mat image = background_.clone();

	// Draw my_car_
	cv::RotatedRect rRect(
		cv::Point2f(scale_ * my_car_.getX(), scale_ * (map_height_ - my_car_.getY())),
		cv::Size2f(scale_ * my_car_.length_, scale_ * my_car_.width_), -my_car_.dir_ * 180 / M_PI);
	cv::Point2f vertices[4];
	rRect.points(vertices);
	for (int i = 0; i < 4; i++)
		cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));

	cv::imshow("Display", image);
}


