#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <Windows.h>
#include <cv.h>

#include "View.h"
#include "Car.h"
#include "Racetrack.h"
#include "Point.h"
#include "CommonFunctions.h"
#include "Vision.h"
#include "Globals.h"
#include "LocalPlanner.h"
#include "RR_API.h"
#include "CamCalib.h"
#include "Intrinsics.h"

std::mutex rc_mutex;

Point mouse_click_pos;

// Code to test simulation. 
// Sets motor speeds, then updates car.
void carLoop(std::vector<Point>& segment, Car& c, int& current) {
	double dist_sq_thresh = 100;
	double max_speed = 1 / M_PER_PIX;
	double angle_thresh = 60 * M_PI / 180;
	long long update_time = time_now();
	
	while (true) {
		rc_mutex.lock();

		if (current < segment.size()) {
			Point& goal(segment[current]);
			double dist_sq = c.pos_.distSquared(goal);
			
			// Check if we have reached point
			if (dist_sq < dist_sq_thresh)
				current++;
			else {
				
				double angle = c.pos_.angle(goal) - c.dir_;

				// A simple control algorithm: just head 
				// straight to waypoints. Will be
				// replaced later.
				while (angle > M_PI)
					angle -= 2 * M_PI;
				while (angle < -M_PI)
					angle += 2 * M_PI;

				if (angle >= 0 && angle <= angle_thresh) {
					c.r_wheel_speed_ = (1 - angle/angle_thresh) * max_speed;
					c.l_wheel_speed_ = max_speed;
				}
				else if (angle <= 0 && angle >= -angle_thresh) {
					c.r_wheel_speed_ = max_speed;
					c.l_wheel_speed_ = (1 + angle/angle_thresh) * max_speed;
				}
				else if (angle > angle_thresh) { // && angle <= 180
					c.r_wheel_speed_ = -max_speed;
					c.l_wheel_speed_ = max_speed;
				}
				else {
					c.r_wheel_speed_ = max_speed;
					c.l_wheel_speed_ = -max_speed;
				}
			}
		}
		

		//std::cout << c.r_wheel_speed_ << " " << c.l_wheel_speed_ << std::endl;
		
		// Figure out time since last update
		long long old_time = update_time;
		update_time = time_now();
		double seconds = (update_time - old_time) / 1000.0;

		// Move car
		c.step(seconds);

		rc_mutex.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

// Thread for running View (for visualisation)
void viewLoop(cv::Mat img, Car& c, std::vector<Point>& segment, bool& view_up_to_date) {
	Car car_copy(c);
	View v(img, car_copy);
	bool was_zero = false;
	while (true) {
		rc_mutex.lock();

		// Update background image if segment has been changed
		if (!view_up_to_date) {
			v.drawNewDots(segment);
			view_up_to_date = true;
		}

		// Update last clicked mouse position
		mouse_click_pos = v.getMousePos();
		
		car_copy = c;
		rc_mutex.unlock();
		v.redraw();
		cv::waitKey(20);
	}
	
}

// For testing motor stuff using keyboard arrow keys.
// To use comment out thread2 and enable thread3 below.
void keyboardLoop(Car& c) {
	while (true) {
		rc_mutex.lock();

		// Shift and left slows down right motor
		// Shift and right slows down left motor
		if (GetAsyncKeyState(VK_SHIFT) & 0x8000) {
			if (GetAsyncKeyState(VK_LEFT) & 0x8000) 
				c.r_wheel_speed_ -= 0.05;
			if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
				c.l_wheel_speed_ -= 0.05;
		}

		// If shift isn't held down, speed up 
		else {
			if (GetAsyncKeyState(VK_LEFT) & 0x8000) 
				c.r_wheel_speed_ += 0.05;
			if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
				c.l_wheel_speed_ += 0.05;
		}
		rc_mutex.unlock();
		std::cout << c.l_wheel_speed_ << " " << c.r_wheel_speed_ << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

}

void localPlannerLoop(std::vector<Point>& global_path, std::vector<Point>& segment, int& current, Car& car, bool& view_up_to_date) {
	LocalPlanner planner(global_path);
	std::vector<Point> temp;

	// DEBUGGING
	// Make other car at mouse click
	OtherCar oc(mouse_click_pos, 0, 0);
	std::vector<OtherCar> other_cars;
	other_cars.push_back(oc);
	while (true) {
		other_cars[0].pos_ = mouse_click_pos;
		rc_mutex.lock();
		planner.update(car, other_cars);
		rc_mutex.unlock();
		temp = planner.getSegment(15);
		rc_mutex.lock();
		segment = temp;
		current = 0;
		view_up_to_date = false;
		rc_mutex.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}

int main(int argc, char *argv[]) {

	// Camera calibration
	//Intrinsics i = camCalib("CalibSettings.xml", true);

	/*
	RR_API roborealm;
	bool result = roborealm.connect("127.0.0.1", 6060);
	if (result == 1)
		std::cout << "Successfully connected to RoboRealm server" << std::endl;
	else {
		std::cout << "Failed to connect to RoboRealm server" << std::endl;
		return 0;
	}

	int width, height;
	roborealm.getDimension(&width, &height);
	std::cout << width << " " << height << std::endl;
	uchar *data = new uchar[width * height * 3];
	roborealm.getImage(data, &width, &height, width * height * 3);
	cv::Mat img(height, width, CV_8UC3, data);
	cv::cvtColor(img, img, CV_RGB2BGR);
	cv::namedWindow("BOB", cv::WINDOW_AUTOSIZE);
	cv::imshow("BOB", img);
	cv::waitKey();

	delete [] data;
	*/

	
	// Load image
	cv::Mat img_bgr = cv::imread("Resources/circle_marker.jpg");
	cv::imshow("Display", img_bgr);
	cv::waitKey();

	// Car detection test
	Vision vision;
	cv::Mat transform;
	//vision.getTransform(img_bgr, transform);
	cv::Point2f my_car_p1;
	cv::Point2f my_car_p2;
	std::vector<cv::Point2f> other_cars;
	vision.getCarMarkers(img_bgr, my_car_p1, my_car_p2, other_cars);
	
	cv::circle(img_bgr, my_car_p1, 2, cv::Scalar(255,0,0), 2);
	cv::circle(img_bgr, my_car_p2, 2, cv::Scalar(0,0,255), 2);
	cv::imshow("Display", img_bgr);
	cv::waitKey();

	/*
	Vision v;

	cv::Mat img_bgr = cv::imread("Resources/racetrack1.jpg");
	cv::Mat img_thresh;
	cv::Point2f centre;
	v.transformTrackImage(img_bgr, img_thresh, centre);
	float finish_tile_length = 1 / M_PER_PIX;
	std::vector<Point> track = v.extractRacetrack(img_thresh, cv::Point2f(centre.x - 10, centre.y),
		cv::Point2f(centre.x - 10, centre.y), M_PI, 
		cv::Point2f(centre.x + finish_tile_length, centre.y));

	// Define a car
	Car c(Point(200, 400), 0, 0.15/M_PER_PIX, 0.075/M_PER_PIX);

	std::vector<Point> segment;
	int current = 0;
	bool view_up_to_date = false;

	// Start threads
	std::thread thread1 (viewLoop, img_bgr, std::ref(c), std::ref(segment), std::ref(view_up_to_date));
	std::thread thread2 (carLoop, std::ref(segment), std::ref(c), std::ref(current));
	//std::thread thread3 (keyboardLoop, std::ref(c));
	std::thread thread4 (localPlannerLoop, std::ref(track), std::ref(segment), std::ref(current), std::ref(c), std::ref(view_up_to_date));

	// Wait for threads to finish
	thread1.join();
	thread2.join();
	//thread3.join();
	thread4.join();
	
	*/
	return 0;
}