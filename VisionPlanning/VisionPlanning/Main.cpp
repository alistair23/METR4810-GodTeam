#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>
//#include <thread>
#include <chrono>
//#include <mutex>
#include <vector>
#include <Windows.h>
#include <cv.h>

#include "View.h"
#include "Car.h"
#include "MyCar.h"
#include "Racetrack.h"
#include "Point.h"
#include "CommonFunctions.h"
#include "Vision.h"
#include "Globals.h"
#include "LocalPlanner.h"
#include "RR_API.h"
#include "CamCalib.h"
#include "Intrinsics.h"

//std::mutex rc_mutex;

Point mouse_click_pos;

// Code to test simulation. 
// Sets motor speeds, then updates car.
/*
void carLoop(std::vector<Point>& segment, MyCar& c, int& current) {
	double dist_sq_thresh = 100;
	double max_speed = 0.5 / M_PER_PIX;
	double angle_thresh = 60 * M_PI / 180;
	long long update_time = time_now();
	
	while (true) {
		rc_mutex.lock();

		if (current < segment.size()) {
			Point& goal(segment[current]);
			double dist_sq = c.getPos().distSquared(goal);
			
			// Check if we have reached point
			if (dist_sq < dist_sq_thresh)
				current++;
			else {
				
				double angle = c.getPos().angle(goal) - c.getDir();

				// A simple control algorithm: just head 
				// straight to waypoints. Will be
				// replaced later.
				while (angle > M_PI)
					angle -= 2 * M_PI;
				while (angle < -M_PI)
					angle += 2 * M_PI;

				if (angle >= 0 && angle <= angle_thresh) {
					c.setRSpeed((1 - angle/angle_thresh) * max_speed);
					c.setLSpeed(max_speed);
				}
				else if (angle <= 0 && angle >= -angle_thresh) {
					c.setRSpeed(max_speed);
					c.setLSpeed((1 + angle/angle_thresh) * max_speed);
				}
				else if (angle > angle_thresh) { // && angle <= 180
					c.setRSpeed(-max_speed);
					c.setLSpeed(max_speed);
				}
				else {
					c.setRSpeed(max_speed);
					c.setLSpeed(-max_speed);
				}
			}
		}
		
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
void viewLoop(cv::Mat img, MyCar& c, std::vector<Point>& segment, bool& view_up_to_date) {
	MyCar car_copy(c);
	View v(img, car_copy);
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
void keyboardLoop(MyCar& c) {
	while (true) {
		rc_mutex.lock();

		// Shift and left slows down right motor
		// Shift and right slows down left motor
		if (GetAsyncKeyState(VK_SHIFT) & 0x8000) {
			if (GetAsyncKeyState(VK_LEFT) & 0x8000) 
				c.setRSpeed(c.getRSpeed() - 0.05);
			if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
				c.setLSpeed(c.getLSpeed() - 0.05);
		}

		// If shift isn't held down, speed up 
		else {
			if (GetAsyncKeyState(VK_LEFT) & 0x8000) 
				c.setRSpeed(c.getRSpeed() + 0.05);
			if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
				c.setLSpeed(c.getLSpeed() + 0.05);
		}
		rc_mutex.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void localPlannerLoop(std::vector<Point>& global_path, std::vector<Point>& segment, int& current, MyCar& car, bool& view_up_to_date) {
	LocalPlanner planner(global_path);
	std::vector<Point> temp;

	// DEBUGGING
	// Make other car at mouse click
	Car oc(mouse_click_pos, 0, 0);
	std::vector<Car> other_cars;
	other_cars.push_back(oc);
	while (true) {
		other_cars[0].setPos(mouse_click_pos);
		rc_mutex.lock();
		planner.update(car, other_cars);
		rc_mutex.unlock();
		temp = planner.getSegment(20);
		rc_mutex.lock();
		segment = temp;
		current = 0;
		view_up_to_date = false;
		rc_mutex.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}*/

void setWheelSpeeds(MyCar& c, Point& goal) {
	double dist_sq_thresh = 100;
	double max_speed = 0.5;	// 40%
	double angle_thresh = 60 * M_PI / 180;
	long long update_time = time_now();
	
	double dist_sq = c.getPos().distSquared(goal);
			
	// Check if we have reached point
	if (dist_sq < dist_sq_thresh) {
		
		// TODO Stop car
	
		return;
	}
				
	double angle = c.getPos().angle(goal) - c.getDir();

	// Ensure angle between -pi and pi
	while (angle > M_PI)
		angle -= 2 * M_PI;
	while (angle < -M_PI)
		angle += 2 * M_PI;

	if (angle >= 0 && angle <= angle_thresh) {
		c.setRSpeed((1 - angle/angle_thresh) * max_speed);
		c.setLSpeed(max_speed);
	}
	else if (angle <= 0 && angle >= -angle_thresh) {
		c.setRSpeed(max_speed);
		c.setLSpeed((1 + angle/angle_thresh) * max_speed);
	}
	else if (angle > angle_thresh) { // && angle <= 180
		c.setRSpeed(-max_speed);
		c.setLSpeed(max_speed);
	}
	else {
		c.setRSpeed(max_speed);
		c.setLSpeed(-max_speed);
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

	/*
	// Load image
	cv::Mat img_bgr = cv::imread("Resources/circle_marker.jpg");
	cv::imshow("Display", img_bgr);
	cv::waitKey();

	// MyCar detection test
	Vision vision;
	cv::Mat transform;
	//vision.getTransform(img_bgr, transform);
	cv::Point2f my_car_p1;
	cv::Point2f my_car_p2;
	std::vector<cv::Point2f> other_cars;
	vision.getMyCarMarkers(img_bgr, my_car_p1, my_car_p2, other_cars);
	
	cv::circle(img_bgr, my_car_p1, 2, cv::Scalar(255,0,0), 2);
	cv::circle(img_bgr, my_car_p2, 2, cv::Scalar(0,0,255), 2);
	cv::imshow("Display", img_bgr);
	cv::waitKey();
	*/

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
	MyCar c(Point(200, 400), 0, 0);

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



	/*
	Vision vision;
	while (!vision.connectRoboRealm());
	vision.setupCamTransform(0);

	MyCar my_car;
	cv::Mat background;
	vision.getCamImg(0, background);
	vision.applyTrans(background, vision.transform_mats_[0]);
	View view(background, my_car);
	view.redraw();

	long long time_before = time_now();
	int count = 0;

	while (true) {
		//vision.getCamImg(0, background);
		//vision.applyTrans(background, vision.transform_mats_[0]);
		//view.background_ = background;
		vision.update();
		Car temp = vision.getMyCarInfo();
		my_car.update(temp.getPos(), temp.getDir(), temp.getSpd());
		view.redraw();

		//Point mouse_pos = view.getMousePos();
		//setWheelSpeeds(my_car, mouse_pos);

		// TODO send speeds through bluetooth

		if (count < 10) {
			count ++;
		}
		else {
			float freq =  count* 1000.0 / (time_now() - time_before); 
			std::cout << "Update rate: " << freq << " Hz" << std::endl;
			time_before = time_now();
			count = 0;
		}
	}
	*/

	
	cv::Mat bob = cv::imread("Resources/racetrack1.jpg");
	Vision v;

	/*
	v.colorThresh(bob);
	cv::Mat erode_element = cv::getStructuringElement(
		cv::MORPH_ELLIPSE, cv::Size(2, 2));
	cv::Mat dilate_element = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,cv::Size(3, 3));

	// Apply the dilation operation
	//cv::erode(bob, bob, erode_element);
	cv::dilate( bob, bob, dilate_element );
	cv::Mat result, grad_x, grad_y, blurred;
	cv::distanceTransform(bob, result, CV_DIST_L2, 3); 
	cv::blur(result, blurred, cv::Size(10, 10));
	cv::Sobel(blurred, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
	cv::Sobel(blurred, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
	*/

	cv::Mat white(bob.rows, bob.cols, CV_8UC1, 255);

	View view(bob, MyCar());
	while (true) {
		
		Point mouse_pos = view.getMousePos();
		std::vector<Point> midpoints = v.getMidpoints(bob, white, cv::Point2f(mouse_pos.x, mouse_pos.y), 0);
		view.drawNewDots(midpoints);
		view.redraw();
		//std::cout << result.at<float>((int) (mouse_pos.y), (int) (mouse_pos.x)) << std::endl;

		/*
		short dx = grad_x.at<short>((int)mouse_pos.y, (int)mouse_pos.x);
		short dy = grad_y.at<short>((int)mouse_pos.y, (int)mouse_pos.x);
		float gradient = atan2(dy, dx);
		cv::Mat curr = bob.clone();
		cv::Point2f p1(mouse_pos.x, mouse_pos.y);
		float mag = pow(dx, 2) + pow(dy, 2);
		cv::Point2f p2(p1.x +dx, p1.y + dy);
		
		//cv::Point2f p2(p1.x + dx, p1.y + dy);
		cv::line(curr, p1, p2, cv::Scalar(100,100,100));
		view.background_ = curr;
		*/
		cv::waitKey(50);
	}
	

	return 0;
}