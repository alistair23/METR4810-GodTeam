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

namespace RaceControl {

class Vision {

public:

	Vision();
	void initCameras(std::vector<int> port_nums, std::string ip_address);

	// Connect to RoboRealm server. Returns true if success
	bool connectRoboRealm(int camera);

	void setupCamTransform(int camera);

	// Grab camera frame and update car and obstacle info
	// If no argument is supplied for my car position guess, 
	// jump straight to full image search.
	// Returns true if my car is found, false otherwise
	bool update(int camera, Point car_pos_guess = Point(-1, -1));

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

	void Vision::getObstacles(cv::Mat& img_in, std::vector<cv::RotatedRect>& obstacles);
	// WORK IN PROGRESS
	std::vector<Point> getMidpoints(
		cv::Mat& img_in, cv::Mat& img_white_warped,
		cv::Point2f start_pos, float start_dir);

	// Color threshold on img to get binary image with
	// road white and everything else black
	void colorThresh(cv::Mat& img);

	// Applies perspective transform to an image
	void applyTrans(cv::Mat& img, cv::Mat& transform);

	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);	// PROBABLY WILL DELETE

	// Stores perspective transforms for each camera
	std::vector<cv::Mat> transform_mats_;
	std::vector<cv::Mat> inv_transform_mats_;

	bool inImg(cv::Mat& img, int x, int y);

	void testColorThresh(int camera);

	cv::Mat* getDisplayImage();

private:

	cv::Mat img_display_;

	// Find perspective transform which when applied to image will
	// give unwarped top-down view. Input image must contain our 
	// special marker used during setup time. Returns true if success
	bool getTransform(cv::Mat& img_in, cv::Mat& transform_output, int camera);

	// Returns euclidean distance between two opencv points
	float dist(cv::Point2f& p1, cv::Point2f& p2);

	// Returns euclidean distance squared between two opencv points
	float distSq(cv::Point2f& p1, cv::Point2f& p2);
	
	// Returns atan2(p2.y - p1.y, p2.x - p1.x)
	float angle(cv::Point2f& p1, cv::Point2f& p2);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	// Get average colour around point in image
	cv::Scalar getColour(cv::Mat& img, cv::Point2f p, int pix_length = 5);

	cv::Point2f getBoundary(cv::Mat& img, cv::Point2f start, float dir,
		bool& at_img_edge, uchar road_colour = 0);

	int getTileType(cv::Mat& img_roi, int rot);

	Tile finish_tile_1;
	Tile finish_tile_2;

	std::vector<Tile> tiles;

	std::vector<RR_API> roborealms_;
	std::string ip_address_;
	std::vector<int> port_nums_;
	
	// Approximate metres per pixel for camera before any transform
	std::vector<float> approx_cam_m_per_pix_;

	Point last_my_car_pos_; // Last known position in pixels before perspective warp
	int last_car_size_; //Last known size of the car marker
	Car my_car_;
	std::vector<Car> other_cars_;

};

} // namespace RaceControl
#endif
