#ifndef _INCLUDED_VISION_H
#define _INCLUDED_VISION_H

#define NOMINMAX	// Fixes bug with including windows.h and using std::min

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <string>

#include "Point.h"
#include "Tile.h"
#include "Car.h"
#include "RR_API.h"

class Vision {

public:

	Vision(int num_cam = 1, std::string ip_address = "127.0.0.1", int port_num = 6060);

	// Connect to RoboRealm server. Returns true if success
	bool connectRoboRealm();

	void setupCamTransform(int cam_index);

	// Find perspective transform which when applied to image will
	// give unwarped top-down view. Input image must contain our 
	// special marker used during setup time. Returns true if success
	bool getTransform(cv::Mat& img_in, cv::Mat& transform_output);

	// Grab camera frame and update car and obstacle info
	void update();

	// Grab a camera image from RoboRealm server
	// cam is the index of the camera to grab image from
	void getCamImg(int cam, cv::Mat& img_out);

	Car getMyCarInfo();

	// Given an input image (BGR), looks for the car markers 
	// (concentric circles). My car's centre is output in my_car_p1,
	// and off-centre circle location in my_car_p2. Other car markers
	// output in other_cars. Returns true if my car is found
	bool getCarMarkers(cv::Mat& img_in, cv::Point2f& my_car_p1,
		cv::Point2f& my_car_p2, std::vector<cv::Point2f>& other_cars);

	// WORK IN PROGRESS
	std::vector<Point> extractRacetrack(cv::Mat& img_thresh, cv::Point2f origin,
		cv::Point2f start_position, float start_orientation, cv::Point2f end_position); 

	// Color threshold on img to get binary image with
	// road black and everything else white
	void colorThresh(cv::Mat& img);

	// Applies perspective transform to an image
	void applyTrans(cv::Mat& img, cv::Mat& transform);

	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);	// PROBABLY WILL DELETE

	// Stores perspective transforms for each camera
	std::vector<cv::Mat> transform_mats_;

private:

	// Returns euclidean distance between two opencv points
	float dist(cv::Point2f& p1, cv::Point2f& p2);

	// Returns euclidean distance squared between two opencv points
	float dist_sq(cv::Point2f& p1, cv::Point2f& p2);
	
	// Returns atan2(p2.y - p1.y, p2.x - p1.x)
	float angle(cv::Point2f& p1, cv::Point2f& p2);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	// Get average colour around point in image
	cv::Scalar getColour(cv::Mat& img, cv::Point2f p, int pix_length = 5);

	cv::Point2f getBoundary(cv::Mat& img, cv::Point2f start, float dir);

	int getTileType(cv::Mat& img_roi, int rot);

	Tile finish_tile_1;
	Tile finish_tile_2;

	std::vector<Tile> tiles;

	int curr_cam_;	// Current camera being monitored
	int num_cam_;	// Total number of cameras
	RR_API roborealm_;
	std::string ip_address_;
	int port_num_;
	
	// Approximate metres per pixel for camera before any transform
	std::vector<float> approx_cam_m_per_pix_;

	Point last_my_car_pos_; // Last known position in pixels before perspective warp
	Car my_car_;
	std::vector<Car> other_cars_;

};


#endif
