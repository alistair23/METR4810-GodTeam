#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>

#include "View.h"
#include "Globals.h"

using namespace RaceControl;

// For some reason putting this in header causes an error...
void onMouse( int event, int x, int y, int, void* );	
Point mouse_pos(0, 0);

View::View():
	frame_time_(20)		// milliseconds / frame
{

	// Create display window
	//cv::namedWindow("View", cv::WINDOW_AUTOSIZE);
	//cv::setMouseCallback("View", onMouse, 0 );
	//background_ = background.clone();
	//background_no_dots_ = background.clone();

	// Draw racetrack
	/*
	for (std::size_t i = 0; i < racetrack.midpoints_.size(); i++) {
		Point& p1(racetrack.midpoints_[i]);
		Point& p2(racetrack.midpoints_[(i + 1) % racetrack.midpoints_.size()]);
		cv::line(background_,
			cv::Point(scale_ * p1.x, scale_ * (map_height_ - p1.y)),
			cv::Point(scale_ * p2.x, scale_ * (map_height_ - p2.y)),
			cv::Scalar(20, 20, 20), scale_ * 0.2);
	}

	// Draw dots at midpoints
	for (std::size_t i = 0; i < racetrack.midpoints_.size(); i++) {
		Point& p1(racetrack.midpoints_[i]);
		cv::circle(background_, cv::Point(scale_ * p1.x,
			scale_ * (map_height_ - p1.y)), 2, cv::Scalar(100, 50, 50), -1);
	}
	*/

	//redraw();
}

void View::createWindow() {
	//cv::namedWindow("View", cv::WINDOW_AUTOSIZE);
	//cv::setMouseCallback("View", onMouse, 0 );
}

void View::setBackground(cv::Mat& background) {
	background_ = background.clone();
	background_no_dots_ = background.clone();
}

void View::updateMyCar(MyCar my_car) {
	my_car_ = my_car;

	// Draw my_car_
	cv::RotatedRect rect(
		cv::Point2f(my_car_.getPos().x, my_car_.getPos().y),
		cv::Size2f(my_car_.getLength(), my_car_.getWidth()), my_car_.getDir() * 180 / M_PI);
	cv::Point2f vertices[4];
	rect.points(vertices);
	for (int i = 0; i < 4; i++)
		cv::line(background_no_dots_, vertices[i], vertices[(i+1)%4], cv::Scalar(100,255,0));
	cv::Point2f dir_p(my_car_.getPos().x + 50 * cos(my_car_.getDir()), my_car_.getPos().y + 50 * sin(my_car_.getDir()));
	cv::line(background_no_dots_, dir_p, cv::Point2f(my_car_.getPos().x, my_car_.getPos().y), cv::Scalar(0,0,255));

}

void View::redraw() {
	
	//img_display = background_.clone();
	
	

	// DEBUGGING
	// Draw other car at mouse click point
	/*
	float other_length = DEFAULT_CAR_LENGTH_PIX;
	float other_width = DEFAULT_CAR_WIDTH_PIX;
	rect.size = cv::Size(other_length, other_width);
	rect.center = cv::Point2f(mouse_pos.x, mouse_pos.y);
	rect.angle = 0;
	rect.points(vertices);
	for (int i = 0; i < 4; i++)
		cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,255));
		*/
		
	cv::imshow("View", background_);
	cv::waitKey(frame_time_);
}

cv::Mat* View::getDisplayImage() {
	return &img_display;
}

void View::drawNewDots(std::vector<Point>& segment) {
	background_ = background_no_dots_.clone();
	for (std::size_t i = 0; i < segment.size(); i++) {
		cv::circle(background_, cv::Point2f(segment[i].x, segment[i].y), 2, cv::Scalar(255,100,0));
	}
}

void onMouse(int event, int x, int y, int, void*) {
	if( event != CV_EVENT_LBUTTONDOWN )
		return;
	
	mouse_pos.x = x;
	mouse_pos.y = y;
}

Point View::getMousePos() {
	return mouse_pos;
}