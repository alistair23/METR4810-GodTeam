#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <Windows.h>

#include "View.h"
#include "Car.h"
#include "Racetrack.h"
#include "Point.h"
#include "CommonFunctions.h"
#include "Vision.h"
#include "Globals.h"

std::mutex rc_mutex;


// Code to test simulation. 
// Sets motor speeds, then updates car.
void carLoop(Racetrack& r, Car& c) {
	int current_mp = 0;
	double dist_sq_thresh = 0.0001;
	double max_speed = 1;
	double angle_thresh = 60 * M_PI / 180;
	long long update_time = time_now();
	
	while (true) {
		rc_mutex.lock();

		if (current_mp < r.midpoints_.size()) {
			Point& goal(r.midpoints_[current_mp]);
			double dist_sq = c.pos_.distSquared(goal);

			// Check if we have reached point
			if (dist_sq < dist_sq_thresh)
				current_mp++;
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
					c.r_wheel_speed_ = max_speed;
					c.l_wheel_speed_ = (1 - angle/angle_thresh) * max_speed;
				}
				else if (angle <= 0 && angle >= -angle_thresh) {
					c.r_wheel_speed_ = (1 + angle/angle_thresh) * max_speed;
					c.l_wheel_speed_ = max_speed;
				}
				else if (angle > angle_thresh) { // && angle <= 180
					c.r_wheel_speed_ = max_speed;
					c.l_wheel_speed_ = -max_speed;
				}
				else {
					c.r_wheel_speed_ = -max_speed;
					c.l_wheel_speed_ = max_speed;
				}
			}
		}
		else {
			current_mp = 0;
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
void viewLoop(Racetrack& r, Car& c) {
	View v(r, c);
	while (true) {
		rc_mutex.lock();
		v.redraw();
		rc_mutex.unlock();
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

int main(int argc, char *argv[]) {
	
#if 0
	// Define a racetrack
	std::vector<Point> path;
	
	path.push_back(Point(0.5, 0.5));
	path.push_back(Point(0.5, 1));
	path.push_back(Point(0.75, 1.4));
	path.push_back(Point(1, 1.5));
	path.push_back(Point(3.5, 1.5));
	path.push_back(Point(3.5, 0.5));
	
	Racetrack r(path);

	// Define a car
	Car c(Point(1, 1), 0, 0.15, 0.075);
	
	// Start threads
	std::thread thread1 (viewLoop, std::ref(r), std::ref(c));
	std::thread thread2 (carLoop, std::ref(r), std::ref(c));
	//std::thread thread3 (keyboardLoop, std::ref(c));

	// Wait for threads to finish
	thread1.join();
	thread2.join();
	//thread3.join();
	
#endif
	
#if 1
	Vision v;

	cv::Mat img_bgr = cv::imread("Resources/racetrack1.jpg");
	cv::Mat img_thresh;
	cv::Point2f centre;
	v.transformTrackImage(img_bgr, img_thresh, centre);
	float finish_tile_length = 1 / M_PER_PIX;
	v.extractRacetrack(img_thresh, cv::Point2f(centre.x - 10, centre.y),
		cv::Point2f(centre.x - 10, centre.y), M_PI, 
		cv::Point2f(centre.x + finish_tile_length, centre.y));
#endif
	return 0;
}