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

	void setupCamTransform(int camera, bool manual_mode);

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
		cv::Point2f& my_car_p2, std::vector<cv::Point2f>& other_cars, int roi_mode);

	void Vision::getObstacles(cv::Mat& img_in, std::vector<cv::RotatedRect>& obstacles);
	bool findFinishTile(cv::Mat& img, Point& pos_out);

	// Returns vector of 3 points, which are locations of 
	// go signals. Returns empty vector if failure
	std::vector<Point> findGoSignal(Point finish_line_pos, int camera);

	// Returns true if the 3 points are collinear (within some threshold)
	bool isCollinear(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);

	// Waits until go signal given, then returns true
	bool waitForGo(int camera, std::vector<Point> signal_pos);

	// WORK IN PROGRESS
	std::vector<Point> getMidpoints(
		cv::Mat& img_in, cv::Mat& img_white_warped,
		cv::Point2f start_pos, float start_dir, int camera);

	std::vector<Point> Vision::getMidpointsManual(cv::Mat& img_in,
		cv::Mat& img_white_warped, int camera);

	// Color threshold on img to get binary image with
	// road white and everything else black
	void colorThresh(cv::Mat& img, int camera);

	// Applies perspective transform to an image
	void applyTrans(cv::Mat& img, int camera);
	void applyInvTrans(cv::Mat& img, int camera);

	bool transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre);	// PROBABLY WILL DELETE

	// Stores perspective transforms for each camera
	std::vector<cv::Mat> transform_mats_;
	std::vector<cv::Mat> inv_transform_mats_;
	std::vector<cv::Size> transform_sizes_;

	bool inImg(cv::Mat& img, int x, int y);

	void testColorThresh(int camera);
	void setColorThresh(int camera, cv::Scalar lower, cv::Scalar upper);
	void previewImg(int camera);
	
	// Displays image in window, then records a number of 
	// mouse clicks and then returns the mouse click positions.
	std::vector<cv::Point2f> getMouseClicks(cv::Mat& img, int num_clicks);

	cv::Mat* getDisplayImage();

private:

	cv::Mat img_display_;

	// Find perspective transform which when applied to image will
	// give unwarped top-down view. Input image must contain our 
	// special marker used during setup time. Returns true if success
	bool getTransform(cv::Mat& img_in, cv::Mat& transform_output, int camera);

	// Find perspective transform, but user manually selects marker circles
	void getTransformManual(cv::Mat& img_in, cv::Mat& transform_output, int camera);

	// Add new points in between if distance is too large
	// Remove points if too small
	void addRemovePoints(std::vector<Point>& path);

	// Smooth path by bringing points towards together
	void smoothPath(std::vector<Point>& path, int selected);

	void generateTransform(cv::Point2f input_quad[4], int camera,
		cv::Mat& img_in, cv::Mat& transform_out);

	// Returns euclidean distance between two opencv points
	float dist(cv::Point2f& p1, cv::Point2f& p2);

	// Returns euclidean distance squared between two opencv points
	float distSq(cv::Point2f& p1, cv::Point2f& p2);
	
	// Returns atan2(p2.y - p1.y, p2.x - p1.x)
	float angle(cv::Point2f& p1, cv::Point2f& p2);

	// Returns index of point in path closest to target point
	std::size_t getClosest(std::vector<Point> path, Point target);

	// Return vertices of a rotated rectangle in clockwise order,
	// starting from top left
	std::vector<cv::Point2f> sortVertices(cv::RotatedRect& rect);

	// Get average color around point in image
	cv::Scalar getColor(cv::Mat& img, cv::Point2f p, int pix_length = 5);

	// Draws a black border around a perspective transformed image
	// Used in getMidpoints
	void applyBlackBorder(cv::Mat& img, int camera);

	cv::Point2f getBoundary(cv::Mat& img, cv::Point2f start, float dir,
		bool& at_img_edge, uchar road_color = 0);

	int getTileType(cv::Mat& img_roi, int rot);

	Tile finish_tile_1;
	Tile finish_tile_2;

	std::vector<Tile> tiles;

	RR_API roborealm_;
	int current_camera_;
	std::string ip_address_;
	std::vector<int> port_nums_;
	std::vector<cv::Scalar> lower_color_thresh_;
	std::vector<cv::Scalar> upper_color_thresh_;
	
	// Approximate metres per pixel for camera before any transform
	std::vector<float> approx_cam_m_per_pix_;

	Point last_my_car_pos_; // Last known position in pixels before perspective warp
	int last_car_size_; //Last known size of the car marker
	Car my_car_;
	std::vector<Car> other_cars_;

};

} // namespace RaceControl
#endif
