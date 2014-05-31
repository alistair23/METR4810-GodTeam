#define _CRT_SECURE_NO_WARNINGS		// Fix for opencv bug
#define _USE_MATH_DEFINES		

#include <math.h>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include "Vision.h"
#include "Globals.h"
#include "CommonFunctions.h"
#include "Vector2D.h"

using namespace RaceControl;

bool mouse_click_changed = false;
bool mouse_l_down = false;
cv::Point2f mouse_pos(0, 0);
cv::Point2f mouse_l_click_pos(0, 0);

// Callback for mouse events on OpenCV windows
// For some reason putting this in header causes an error...
void onMouse(int event, int x, int y, int, void*) {
	mouse_pos.x = x;
	mouse_pos.y = y;
	if(event == CV_EVENT_LBUTTONUP) {
		mouse_click_changed = true;
		mouse_l_click_pos.x = x;
		mouse_l_click_pos.y = y;
		mouse_l_down = false;
	} else if (event == CV_EVENT_LBUTTONDOWN) {
		mouse_l_down = true;
	}
}

struct sort_pred {
    bool operator()(const std::pair<int,int> &left, const std::pair<int,int> &right) {
        return left.second > right.second;
    }
};

struct sort_v_pred {
    bool operator()(const std::pair<std::vector<int>,int> &left, const std::pair<std::vector<int>,int> &right) {
        return left.second > right.second;
    }
};

Vision::Vision():
	last_car_size_(50),
	// Load tile images
	finish_tile_1(cv::imread("Resources/finish_1.jpg"), 0),
	finish_tile_2(cv::imread("Resources/finish_2.jpg"), 0)
{
	tiles.push_back(Tile(cv::imread("Resources/straight.jpg"), 0));
	tiles.push_back(Tile(cv::imread("Resources/right_turn.jpg"), -1));
	tiles.push_back(Tile(cv::imread("Resources/left_turn.jpg"), 1));
	tiles.push_back(Tile(cv::imread("Resources/chicane.jpg"), 0));
	tiles.push_back(Tile(cv::imread("Resources/right_hairpin.jpg"), -1));
	tiles.push_back(Tile(cv::imread("Resources/left_hairpin.jpg"), 1));

	get_img_time = 0;
	update_time = 0;
	num_cycles = 0;

}

void Vision::initCameras(std::vector<int> port_nums, std::string ip_address) {
	port_nums_ = port_nums;
	ip_address_ = ip_address;
	current_camera_ = -1;
	transform_mats_.clear();
	inv_transform_mats_.clear();
	transform_sizes_.clear();
	approx_cam_m_per_pix_.clear();
	lower_color_thresh_.clear();
	upper_color_thresh_.clear();
	for (std::size_t i = 0; i < port_nums.size(); i++) {
		transform_mats_.push_back(cv::Mat());
		inv_transform_mats_.push_back(cv::Mat());
		transform_sizes_.push_back(cv::Size());
		approx_cam_m_per_pix_.push_back(0);
		lower_color_thresh_.push_back(cv::Scalar(30, 0, 0));
		upper_color_thresh_.push_back(cv::Scalar(150, 255, 255));
		//connectRoboRealm(i);
	}
}

// Connect to RoboRealm server. Returns true if success
bool Vision::connectRoboRealm(int camera) {

	// For ip address, need to convert from std::string to char *
	char * writable = new char[ip_address_.size() + 1];
	std::strcpy(writable, ip_address_.c_str());
	writable[ip_address_.size()] = '\0'; 

	roborealm_.disconnect();
	bool result = roborealm_.connect(writable, port_nums_[camera]);
	delete[] writable;

	if (result == 1) {
		std::cout << "Successfully connected to RoboRealm server " << port_nums_[camera] << std::endl;
		current_camera_ = camera;
		return true;
	}
	else {
		std::cout << "Failed to connect to RoboRealm server " << port_nums_[camera] << std::endl;
		current_camera_ = -1;
		return false;
	}
}

cv::Mat* Vision::getDisplayImage() {
	return &img_display_;
}

// Save/load transform matrices to file
void Vision::saveToFile(int camera) {
	std::string filename = "camera" + std::to_string(camera) + ".xml";
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << "transform_mat" << transform_mats_[camera];
	fs << "inv_transform_mat" << inv_transform_mats_[camera];
	fs << "height" << transform_sizes_[camera].height;
	fs << "width" << transform_sizes_[camera].width;
	std::cout << "Saved transform matrices for camera " << camera << std::endl;
	fs.release();
}

void Vision::loadFromFile(int camera) {
	std::string filename = "camera" + std::to_string(camera) + ".xml";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["transform_mat"] >> transform_mats_[camera];
	fs["inv_transform_mat"] >> inv_transform_mats_[camera];
	int height = (int) fs["height"];
	int width = (int) fs["width"];
	transform_sizes_[camera] = cv::Size(width, height);
	std::cout << "Loaded transform matrices for camera " << camera << std::endl;
}

// Gets the perspective transform for a camera
void Vision::setupCamTransform(int camera, bool manual_mode) {
	bool gotTrans = false;
	cv::Mat img;
	getCamImg(camera, img);

	if (manual_mode) {
		getTransformManual(img, transform_mats_[camera], camera);
	} else {
		gotTrans = getTransform(img, transform_mats_[camera], camera);
		if (!gotTrans) {
			std::cout << "Failed to get transform for camera " << camera << std::endl;
			return;
		}
	}
	
	inv_transform_mats_[camera] = transform_mats_[camera].inv();
	std::cout << "Successfully got transform for camera " << camera << std::endl;
}

// Find perspective transform which when applied to image will
// give unwarped top-down view. Input image must contain our 
// special marker used during setup time. Returns true if success
bool Vision::getTransform(cv::Mat& img_in, cv::Mat& transform_out, int camera) {
	
	std::cout << "Getting transform..." << std::endl;
	//cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);

	// Make hls copy
	cv::Mat img_hls;
	cv::cvtColor(img_in, img_hls, CV_BGR2HLS_FULL);

	// Make grayscale copy, Reduce noise with a kernel 5x5
	cv::Mat img_temp;
	cv::cvtColor(img_in, img_temp, CV_BGR2GRAY);
	cv::blur(img_temp, img_temp, cv::Size(5,5) );

	// Canny edge detection
	int threshold = 35;
	cv::Canny(img_temp, img_temp, threshold, threshold * 3, 3);

	// Make color format for showing stuff
	cv::Mat cdst;
	cv::cvtColor(img_temp, cdst, CV_GRAY2BGR);
	
	// Find contours from Canny image
	// Warning, findContours affects input image (img_temp)
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	std::cout << "Looking through ellipses" << std::endl;

	// Fit ellipses to contours and keep circular ones
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {
		if (contours[i].size() < 5)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);	
		
		// Check circularity and that it isn't too small
		if (r.size.height/r.size.width < 3.5 && r.size.area() > 20) {
			ellipses.push_back(r);
			cv::ellipse(cdst, r, cv::Scalar(100,200,100), 1);
		}
	}

	// Look for concentric circles: this indicates our marker
	// Max distance between centres to consider concentric (in pixels):
	float concentric_thresh = 5;	
	float scale;
	bool error = true;
	std::vector<std::pair<std::vector<int>, int>> possible_configs;

	// Go through each ellipse pair
	for (std::size_t i = 0; i < ellipses.size(); i++) {
		for (std::size_t j = i + 1; j < ellipses.size(); j++) {

			// Check size isn't too similar
			if (abs(ellipses[i].size.height - ellipses[j].size.height) < 10)
				continue;

			// Check if concentric. Distance between centres is used as 
			// error rating
			float dist_in_pix = dist(ellipses[i].center, ellipses[j].center);
			if (dist_in_pix < concentric_thresh) {

				// Get an approximate scale, using larger circle's diameter
				// This is for finding the four other circles
				float large_circle_pix, small_circle_pix;
				int large_circle_index, small_circle_index;
				if (ellipses[i].size.height > ellipses[j].size.height) {
					large_circle_pix = ellipses[i].size.height;
					large_circle_index = i;
					small_circle_pix = ellipses[j].size.height;
					small_circle_index = j;
				} else {
					large_circle_pix = ellipses[j].size.height;
					large_circle_index = j; 
					small_circle_pix = ellipses[i].size.height;
					small_circle_index = i;
				}
				approx_cam_m_per_pix_[camera] = OUR_CENTRE_DIAMETER_BIG / large_circle_pix;
				float thresh_distSq = pow(((OUR_SQUARE_SIDE / approx_cam_m_per_pix_[camera]) * 1.2), 2); 

				// Identify red, green, blue, black circles
				int max_blueness = 0, max_greenness = 0, max_redness = 0, max_blackness = 0;
				std::vector<std::pair<int, int>> blacks, blues, greens, reds;
				for (std::size_t k = 0; k < ellipses.size(); k++) {
					if (distSq(ellipses[k].center, ellipses[i].center) < thresh_distSq) {
						cv::Scalar values = getColor(img_hls, ellipses[k].center, ellipses[k].size.height * 0.25);

						// Ignore circles which are too large/small
						if (ellipses[k].size.height > large_circle_pix ||
							ellipses[k].size.height < 0.6 * small_circle_pix)
							continue;
						
						// Ignore white (high luminosity)
						if (values[1] >= 250)
							continue;

						// Ignore the centre circles
						if (k == i || k == j)
							continue;

						//cv::ellipse(cdst, ellipses[k], cv::Scalar(123,45,67), 2);
						int blueness = 999 - abs(160 - values[0]);
						int redness = 999 - std::min(values[0], 255 - values[0]);
						int greenness = 999 - abs(110 - values[0]);
						if (values[1] < 20) {
							blueness = 0;
							redness = 0;
							greenness = 0;
						}
						int blackness = 999 - values[1] - values[2];
						blues.push_back(std::pair<int, int>(k, blueness));
						reds.push_back(std::pair<int, int>(k, redness));
						greens.push_back(std::pair<int, int>(k, greenness));
						blacks.push_back(std::pair<int, int>(k, blackness));
					}
				}
				
				std::sort(blacks.begin(), blacks.end(), sort_pred());
				std::sort(blues.begin(), blues.end(), sort_pred());
				std::sort(greens.begin(), greens.end(), sort_pred());
				std::sort(reds.begin(), reds.end(), sort_pred());

				// Limit number to speed up computation
				if (blacks.size() > 30) blacks.resize(30);
				if (blues.size() > 30) blues.resize(30);
				if (greens.size() > 30) greens.resize(30);
				if (reds.size() > 30) reds.resize(30);

				// Error checks
				
				float max_dist = OUR_SQUARE_SIDE / approx_cam_m_per_pix_[camera] * 2.0;
				float min_dist = (OUR_SQUARE_SIDE / approx_cam_m_per_pix_[camera]) * 0.4;

				for (std::size_t iblack = 0; iblack < blacks.size(); iblack++) {
					for (std::size_t iblue = 0; iblue < blues.size(); iblue++) {
						for (std::size_t igreen = 0; igreen < greens.size(); igreen++) {
							for (std::size_t ired = 0; ired < reds.size(); ired++) {
								std::vector<int> config;
								config.push_back(blues[iblue].first);
								config.push_back(reds[ired].first);
								config.push_back(greens[igreen].first);
								config.push_back(blacks[iblack].first);
								error = false;
								for (int l = 0; l < 3; l++) {
									if (config[l] == -1) {
										error = true;
										break;
									}
									for (int m = l + 1; m < 4; m++) {
										float this_dist = dist(ellipses[config[l]].center, ellipses[config[m]].center);
										bool is_on_center = dist(ellipses[config[l]].center, ellipses[small_circle_index].center) <= concentric_thresh;
										is_on_center = is_on_center || dist(ellipses[config[m]].center, ellipses[small_circle_index].center) <= concentric_thresh;
										
										if (config[l] == config[m] || this_dist > max_dist || this_dist < min_dist || is_on_center) {
											error = true;
											break;
										}
									}
								}

								if (!error) {
									int color_correctness = blues[iblue].second 
										+ reds[ired].second + greens[igreen].second + blacks[iblack].second;
									possible_configs.push_back(std::pair<std::vector<int>, int>(config, color_correctness));
								}
							}
						}
					}
				}
			}
		}
	}

	if (possible_configs.size() == 0) {
		std::cout << "Failed to find marker!" << std::endl;
		return false;
	}

	std::sort(possible_configs.begin(), possible_configs.end(), sort_v_pred());
	cv::Point2f blue_circle = ellipses[possible_configs[0].first[0]].center;
	cv::Point2f red_circle = ellipses[possible_configs[0].first[1]].center;
	cv::Point2f green_circle = ellipses[possible_configs[0].first[2]].center;
	cv::Point2f black_circle = ellipses[possible_configs[0].first[3]].center;

	cv::putText(cdst, "Blue", blue_circle,
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
	cv::putText(cdst, "Red", red_circle,
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
	cv::putText(cdst, "Green", green_circle,
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
	cv::putText(cdst, "Black", black_circle,
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(2555,255,255), 1, CV_AA);

	cv::imshow("Display1", cdst);
	//cv::waitKey();

	cv::Point2f input_quad[4] = {blue_circle, red_circle, green_circle, black_circle};
	generateTransform(input_quad, camera, img_in, transform_out);

	return true;
}

// Find perspective transform, but user manually selects marker circles
void Vision::getTransformManual(cv::Mat& img_in, cv::Mat& transform_out, int camera) {

	//cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);

	// Make hls copy
	cv::Mat img_hls;
	cv::cvtColor(img_in, img_hls, CV_BGR2HLS_FULL);

	// Make grayscale copy, Reduce noise with a kernel 5x5
	cv::Mat img_temp;
	cv::cvtColor(img_in, img_temp, CV_BGR2GRAY);
	cv::blur(img_temp, img_temp, cv::Size(5,5) );

	// Canny edge detection
	int threshold = 35;
	cv::Canny(img_temp, img_temp, threshold, threshold * 3, 3);

	// Make color format for showing stuff
	cv::Mat cdst;
	cv::cvtColor(img_temp, cdst, CV_GRAY2BGR);
	
	// Find contours from Canny image
	// Warning, findContours affects input image (img_temp)
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	// Fit ellipses to contours and keep circular ones
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {
		if (contours[i].size() < 5)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);	
		
		// Check circularity and that it isn't too small
		if (r.size.height/r.size.width < 4 && r.size.area() > 15) {
			ellipses.push_back(r);
			cv::Scalar color = getColor(img_in, r.center);
			cv::ellipse(cdst, r, color, 3);
		}
	}

	// Get mouse clicks
	int num_clicks = 0;
	cv::namedWindow("Mouse clicks");
	cv::imshow("Mouse clicks", cdst);
	cv::setMouseCallback("Mouse clicks", onMouse);
	mouse_click_changed = false;
	std::vector<cv::Point2f> click_pos;
	std::cout << "Click blue, red, green, black circles in order." << std::endl;
	while (num_clicks < 4) {
		if (mouse_click_changed) {
			click_pos.push_back(cv::Point2f(mouse_l_click_pos.x, mouse_l_click_pos.y));
			num_clicks++;
			mouse_click_changed = false;
		}
		cv::waitKey(100);
	}
	cv::destroyWindow("Mouse clicks");	
	
	// Find the closest ellipses to the mouse clicks
	int closest_ellipses[4];
	float distances[4] = {99999, 99999, 99999, 99999};
	for (std::size_t i = 0; i < ellipses.size(); i++) {
		for (int j = 0; j < 4; j++) {
			float this_distance = dist(click_pos[j], ellipses[i].center);
			if (this_distance < distances[j]) {
				distances[j] = this_distance;
				closest_ellipses[j] = i;
			}
		}
	}

	// Generate transform
	cv::Point2f blue_circle = ellipses[closest_ellipses[0]].center;
	cv::Point2f red_circle = ellipses[closest_ellipses[1]].center;
	cv::Point2f green_circle = ellipses[closest_ellipses[2]].center;
	cv::Point2f black_circle = ellipses[closest_ellipses[3]].center;
	cv::Point2f input_quad[4] = {blue_circle, red_circle, green_circle, black_circle};
	generateTransform(input_quad, camera, img_in, transform_out);
}

void Vision::generateTransform(cv::Point2f input_quad[4],
							   int camera, cv::Mat& img_in, 
							   cv::Mat& transform_out) {

	// Output Quadilateral or World plane coordinates
    cv::Point2f output_quad[4];
	float side_in_pix = OUR_SQUARE_SIDE / M_PER_PIX;

	// Order is blue, red, green, black (clockwise)
	output_quad[0] = cv::Point2f(0, 0); //ellipses[blue_circle].center;
	output_quad[1] = cv::Point2f(output_quad[0].x + side_in_pix, output_quad[0].y);
	output_quad[2] = cv::Point2f(output_quad[0].x + side_in_pix, output_quad[0].y + side_in_pix);
	output_quad[3] = cv::Point2f(output_quad[0].x, output_quad[0].y + side_in_pix);
      
    // Get the Perspective Transform Matrix
    transform_out = getPerspectiveTransform( input_quad, output_quad );

	// Find bounding box for transformed image corners
	std::vector<cv::Point2f> orig_corners, trans_corners;
	orig_corners.push_back(cv::Point2f(0, 0));
	orig_corners.push_back(cv::Point2f(0, img_in.rows));
	orig_corners.push_back(cv::Point2f(img_in.cols, img_in.rows));
	orig_corners.push_back(cv::Point2f(img_in.cols, 0));
	cv::perspectiveTransform(orig_corners, trans_corners, transform_out);
	int top = trans_corners[0].y;
	int bottom = trans_corners[0].y;
	int left = trans_corners[0].x;
	int right = trans_corners[0].x;
	for (int i = 1; i < 4; i++) {
		if (trans_corners[i].y < top)
			top = trans_corners[i].y;
		if (trans_corners[i].y > bottom)
			bottom = trans_corners[i].y;
		if (trans_corners[i].x < left)
			left = trans_corners[i].x;
		if (trans_corners[i].x > right)
			right = trans_corners[i].x;
	}

	// Translate transform
	output_quad[0] = cv::Point2f(-left, -top);
	output_quad[1] = cv::Point2f(output_quad[0].x + side_in_pix, output_quad[0].y);
	output_quad[2] = cv::Point2f(output_quad[0].x + side_in_pix, output_quad[0].y + side_in_pix);
	output_quad[3] = cv::Point2f(output_quad[0].x, output_quad[0].y + side_in_pix);

	// Get the Perspective Transform Matrix
    transform_out = getPerspectiveTransform( input_quad, output_quad );

	// Make output image the minimum required size
	transform_sizes_[camera] = cv::Size(right - left, bottom - top);

	// Show the effect of applying perspective transform to input image
	//cv::Mat img_output;
    warpPerspective(img_in, img_display_, transform_out, transform_sizes_[camera]);
	cv::imshow("Display2", img_display_);
	//cv::waitKey();
}


// Grab camera frame and update car info
// If no argument is supplied for my car position guess, 
// jump straight to full image search.
// Returns true if my car is found, false otherwise
bool Vision::update(int camera, Point car_pos_guess) {

	long long time1 = time_now();
	cv::Mat img_in;
	getCamImg(camera, img_in);
	get_img_time += time_now() - time1;

	cv::Point2f my_car_p1;
	cv::Point2f my_car_p2;
	std::vector<cv::Point2f> other_cars_points;

	bool found_my_car = false;

	// Define region of interest around last known position
	// This cuts down on computation
	if (car_pos_guess.x != -1) {

		// Apply inverse of perspective transform to get into 
		// camera coordinates
		std::vector<cv::Point2f> in_point;
		std::vector<cv::Point2f> camera_point;
		in_point.push_back(cv::Point2f(car_pos_guess.x, car_pos_guess.y));
		cv::perspectiveTransform(in_point, camera_point, inv_transform_mats_[camera]);
		cv::Point2f guess_pos = camera_point[0];

		// Size of roi is proportional to the size of the car marker
		int search_size = 120;//1.5 * std::max(last_car_size_, 50);

		int roi_x = guess_pos.x - search_size;
		int roi_y = guess_pos.y - search_size;

		// Ensure roi is within image bounds
		while (roi_x + 2 * search_size > img_in.cols -1)
			roi_x -= 1;
		if (roi_x < 0)
			roi_x = 0;
		while (roi_y + 2 * search_size > img_in.rows -1)
			roi_y -= 1;
		if (roi_y < 0)
			roi_y = 0;
		
		cv::Rect roi(roi_x, roi_y, 2 * search_size, 2 * search_size);
		cv::Mat img_roi = img_in(roi);
		found_my_car = getCarMarkers(img_roi, my_car_p1, my_car_p2, other_cars_points, 1);// 1 means roi_mode 

		if (found_my_car)  {

			// Need to adjust coordinates to account for roi origin
			my_car_p1.x += roi.x;
			my_car_p1.y += roi.y;

			if (my_car_p2.x != 0 && my_car_p2.y != 0) {
				my_car_p2.x += roi.x;
				my_car_p2.y += roi.y;
			}
		}
	}

	if (!found_my_car) {
		std::cout << "Failed to find my car, trying full image" << std::endl;
		found_my_car = getCarMarkers(img_in, my_car_p1, my_car_p2, other_cars_points, 0);//0 means no roi_mode
		if (!found_my_car) {
			std::cout << "Failed to find my car" << std::endl;
			update_time += time_now() - time1;
			return false;
		}
	}

	std::cout << "Found my car" << std::endl;

	// Apply perspective transform to points found
	std::vector<cv::Point2f> orig_points, trans_points;
	orig_points.push_back(my_car_p1);
	orig_points.push_back(my_car_p2);
	for (std::size_t i = 0; i < other_cars_points.size(); i++) {
		orig_points.push_back(other_cars_points[i]);
	}
	cv::perspectiveTransform(orig_points, trans_points, transform_mats_[camera]);

	double new_dir;

	// If we failed to find smallest off-centre circle, direction is not valid
	if (my_car_p2.x == 0 && my_car_p2.y == 0)
		new_dir = my_car_.getDir();
	else 
		new_dir = angle(trans_points[0], trans_points[1]);
	Point new_pos(trans_points[0].x, trans_points[0].y);
	double seconds = (time_now() - my_car_.getUpdateTime()) * 0.001;
	double new_spd = my_car_.getPos().dist(new_pos) / seconds;
	my_car_.update(new_pos, new_dir, new_spd);

	// TODO other cars
	update_time += time_now() - time1;
	return true;

}

void Vision::getCamImg(int camera, cv::Mat& img_out) {

	// Check if connected to roborealm server
	if (!roborealm_.connected || current_camera_ != camera)
		if (!connectRoboRealm(camera))	// Attempt connection
			return;					// Failed to connect

	// Wait for latest image
	roborealm_.waitImage();
	int width, height;
	roborealm_.getDimension(&width, &height);
	uchar *data = new uchar[width * height * 3];
	roborealm_.getImage(data, &width, &height, width * height * 3);
	cv::Mat img(height, width, CV_8UC3, data);
	cv::cvtColor(img, img_out, CV_RGB2BGR);
	delete [] data;
}

Car Vision::getMyCarInfo() {
	return my_car_;
}

// Given an input image (BGR), looks for the car markers 
// (concentric circles). My car's centre is output in my_car_p1,
// and off-centre circle location in my_car_p2. Other car markers
// output in other_cars. Returns true if my car is found
bool Vision::getCarMarkers(
	cv::Mat& img_in, cv::Point2f& my_car_p1, cv::Point2f& my_car_p2,
	std::vector<cv::Point2f>& other_cars, int roi_mode) {

	// Make grayscale copy, Reduce noise with a kernel 3x3
	cv::Mat img_gray;
	cv::cvtColor(img_in, img_gray, CV_BGR2GRAY);
	cv::blur(img_gray, img_gray, cv::Size(3,3) );

	// Canny edge detection
	cv::Mat img_canny;
	int threshold = 35;
	cv::Canny(img_gray, img_canny, threshold, threshold * 3, 3);

	// Make color format for showing stuff
	cv::Mat cdst;
	cv::cvtColor(img_canny, cdst, CV_GRAY2BGR);
	
	// Find contours from Canny image
	// Warning, findContours affects input image (img_canny)
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( img_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	// Fit ellipses to contours and keep circular ones
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {

		if (contours[i].size() < 5)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);	

		// Check circularity and that it isn't too small
		if (r.size.height/r.size.width < 4.5){// && r.size.area() > 5) {
			ellipses.push_back(r);
			//cv::ellipse(cdst, r, cv::Scalar(rand()%255,rand()%255,rand()%255), 1);
		}
	}

	// Show circles found
	//cv::imshow("Display", cdst);
	//cv::waitKey(100);
	
	other_cars.clear();

	// Max pixels between detected circle centres to consider circles concentric
	float dist_thresh[2] = {7, 9};	
	bool my_car_found = false;

	// Look for concentric circles: this indicates car markers
	for (std::size_t i = 0; i < ellipses.size(); i++) {
		for (std::size_t j = i + 1; j < ellipses.size(); j++) {
			float ratio_i = ellipses[i].size.height / ellipses[i].size.width;
			float ratio_j = ellipses[j].size.height / ellipses[j].size.width;
			if (abs(ratio_i - ratio_j) > 0.15) {
				continue;
			}
			float centre_dist = dist(ellipses[i].center, ellipses[j].center);
			if (centre_dist < dist_thresh[roi_mode] && abs(ellipses[i].size.height - ellipses[j].size.height) > 7)  {

				bool this_is_my_car = false;
				if (my_car_found) {
					continue;
				}
				
				// Found two concentric circles. Now check if my car
				// (three concentric circles)
				for (std::size_t k = j + 1; k < ellipses.size(); k++) {
					float ratio_k = ellipses[k].size.height / ellipses[k].size.width;
					if (abs(ratio_k - ratio_i) > 0.15 || abs(ratio_k - ratio_j) > 0.15)
						continue;
					centre_dist = dist(ellipses[i].center, ellipses[k].center);
					if (centre_dist < dist_thresh[roi_mode] && 
						abs(ellipses[i].size.height - ellipses[k].size.height) > 7 &&
						abs(ellipses[j].size.height - ellipses[k].size.height) > 7) {
						
						int biggest, smallest;
						if (ellipses[i].size.height >= ellipses[j].size.height)	{
							biggest = i;
							smallest = j;
						} else {
							biggest = j;
							smallest = i;
						} if (ellipses[biggest].size.height <= ellipses[k].size.height) {
							biggest = k;
						}
						if (ellipses[smallest].size.height >= ellipses[k].size.height) {
							smallest = k;
						}

						/*float up_thresh = MY_CAR_MARKER_BIGGEST/MY_CAR_MARKER_SMALLEST+ 0.35;
						float bottom_thresh =  MY_CAR_MARKER_BIGGEST/MY_CAR_MARKER_SMALLEST- 0.35;
						float aspect_ratio = ellipses[biggest].size.height/ellipses[smallest].size.height;
						if ( aspect_ratio> up_thresh || aspect_ratio < bottom_thresh )
							continue;*/
						

						// This is my car
						float cam_angle = acos(ellipses[biggest].size.width/ellipses[biggest].size.height);
						float adjust_length = 0.05 * ellipses[biggest].size.height/0.07;
						float marker_angle = ellipses[biggest].angle * M_PI/180;
						cv::Point2f marker_pos;
						marker_pos.x = (ellipses[k].center.x + ellipses[i].center.x + ellipses[j].center.x)/3;
						marker_pos.y = (ellipses[k].center.y + ellipses[i].center.y + ellipses[j].center.y)/3;
						my_car_p1.x = marker_pos.x  + adjust_length * sin(cam_angle) * cos(marker_angle); //+ my_car_.getHeight() * sin(cam_angle);// * cos(-M_PI/2 + marker_angle);
						my_car_p1.y = marker_pos.y + adjust_length * sin(cam_angle) * sin(marker_angle);// * sin(-M_PI/2 + marker_angle);
						last_car_size_ = ellipses[biggest].size.height;
						this_is_my_car = true;
						my_car_found = true;
						std::cout<< "marker angle: " << marker_angle << std::endl;
						//std::cout << "cam angle: " << cam_angle << "car height : " << my_car_.getHeight() << std::endl;
						//std::cout << "ratio : " << ellipses[biggest].size.width/ellipses[biggest].size.height << std::endl;
						cv::ellipse(cdst, ellipses[i], cv::Scalar(255,0,0), 1);
						cv::ellipse(cdst, ellipses[j], cv::Scalar(0,0,255), 1);
						cv::ellipse(cdst, ellipses[k], cv::Scalar(0,100,200), 1.5);
						
						// Now look for off-centre circle (used for finding orientation)
						// The diameter of the off-centre circle should be less than the
						// smallest of the concentric circles
						float max_diameter = ellipses[biggest].size.height * 0.3;
						float min_diameter = ellipses[biggest].size.height * 0.1;
						for (std::size_t a = 0; a < ellipses.size(); a++) {
							/*if (a == i || a == j || a == k)
								continue;*/
							float ratio_a = ellipses[a].size.height / ellipses[a].size.width;
							if (ellipses[a].size.height > max_diameter || 
								ellipses[a].size.height < min_diameter) {
								continue;
							}
							if(abs(ratio_a - ratio_i) > 0.15 || abs(ratio_a - ratio_j) > 0.15) {
								continue;
							}
							
							float this_dist = dist(ellipses[a].center, marker_pos);
							if (this_dist <= ellipses[biggest].size.height * 0.2) {
								my_car_p2.x = ellipses[a].center.x + adjust_length * sin(cam_angle) * cos(marker_angle);//+  my_car_.getHeight();// * sin(cam_angle) * cos(-M_PI/2 + marker_angle);
								my_car_p2.y = ellipses[a].center.y +  adjust_length * sin(cam_angle) * sin(marker_angle);// * sin(-M_PI/2 + marker_angle);
								cv::ellipse(cdst, ellipses[a], cv::Scalar(0,255,0), 2);
								
								break;
							}
						}
						cv::imshow("BOB", cdst);
						break;							
					}
				}
				
				if (!this_is_my_car) {
					other_cars.push_back(ellipses[j].center);
				}
			}
		}
	}
	return my_car_found;
}

void Vision::getObstacles(cv::Mat& img_in, std::vector<cv::RotatedRect>& obstacles) {

	// Make grayscale copy, Reduce noise with a kernel 5x5
	cv::Mat img_temp;
	cv::cvtColor(img_in, img_temp, CV_BGR2GRAY);
	cv::blur(img_temp, img_temp, cv::Size(5, 5));

	// Canny edge detection
	int threshold = 55;
	cv::Canny(img_temp, img_temp, threshold, threshold * 3, 3);

	// Find contours from Canny image
	// Warning, findContours affects input image (img_temp)
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	float max_side = 0.1 / M_PER_PIX;	// Max side length for obstacle
	float min_side = 0.008 / M_PER_PIX;	// Min side length to accept obstacle
	float clearance = 0.03 / M_PER_PIX;	// Added to side length
	float car_check_thresh = DEFAULT_CAR_LENGTH_PIX;
	for (int i = 0; i < contours.size(); i++)
	{
		cv::RotatedRect rect = cv::minAreaRect( cv::Mat(contours[i]));

		// Check size is reasonable
		if (rect.size.height > max_side || rect.size.width > max_side || 
			rect.size.height < min_side || rect.size.width < min_side)
			continue;

		// TODO getPos(timeNow())
		// Make sure we didn't detect a car as obstacle
		if (my_car_.getPos().dist(Point(rect.center.x, rect.center.y)) <
			car_check_thresh)
			continue;

		bool is_other_car = false;
		for (int j = 0; j < other_cars_.size(); j++) {
			if (other_cars_[j].getPos().dist(Point(rect.center.x, rect.center.y)) < car_check_thresh) {
				is_other_car = true;
				break;
			}
		}
		if (is_other_car)
			continue;

		// Checks have passed
		rect.size.height += clearance;
		rect.size.width += clearance;
		rect.center.x -= clearance * 0.5;
		rect.center.y -= clearance * 0.5;
		obstacles.push_back(rect);
	}

	/// Draw contours + rotated rects + ellipses
	//cv::Mat drawing;
	cv::Mat cdst = img_in.clone();
	for( int i = 0; i< obstacles.size(); i++ )
	{
		cv::Point2f rect_points[4]; obstacles[i].points( rect_points );
		for( int j = 0; j < 4; j++ )
			line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 2);
	}
	cv::imshow("Display", cdst);
	//cv::waitKey();
}

bool Vision::findFinishTile(cv::Mat& img_in, Point& pos_out) {

	// Make grayscale copy, Reduce noise with a kernel 5x5
	cv::Mat img_temp;
	cv::cvtColor(img_in, img_temp, CV_BGR2GRAY);
	cv::blur(img_temp, img_temp, cv::Size(5, 5) );

	// Canny edge detection
	int threshold = 35;
	cv::Canny(img_temp, img_temp, threshold, threshold * 3, 3);

	// Make color format for showing stuff
	//cv::Mat cdst;
	//cv::cvtColor(img_temp, cdst, CV_GRAY2BGR);
	
	// Find contours from Canny image
	// Warning, findContours affects input image (img_canny)
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( img_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	// Fit ellipses to contours 
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {

		if (contours[i].size() < 5)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);

		// Check circularity and size
		float max_diameter = CIRCLE_MARK_DIAMETER * 1.2 / M_PER_PIX;
		float min_diameter = CIRCLE_MARK_DIAMETER * 0.8 / M_PER_PIX;
		if (r.size.height/r.size.width < 1.2 && 
			r.size.height < max_diameter &&
			r.size.height > min_diameter) {
			//cv::ellipse(cdst, r, cv::Scalar(255, 0, 0));
			ellipses.push_back(r);
		}
	}

	//cv::imshow("Display", cdst);
	//cv::waitKey();

	// Go through ellipse and find circle pairs indicating starting line
	std::vector<cv::Point2f> pair_centres;
	float dist_thresh = 40;			// Pixels
	float max_angle = M_PI_2 * 1.2;	// Radians
	float min_angle = M_PI_2 * 0.8;
	float marker_dist_pix = CIRCLE_MARK_DIST / M_PER_PIX;
	for (std::size_t i = 0; i < ellipses.size() - 1; i++) {
		for (std::size_t j = i + 1; j < ellipses.size(); j++) {
			if (abs(dist(ellipses[i].center, ellipses[j].center) - marker_dist_pix) > dist_thresh)
				continue;

			// TODO image rotation, pixel comparison to source tile
			// Currently assumes finish line is orientated with direction
			// of car travel to the left
			float dir = angle(ellipses[i].center, ellipses[j].center);
			if (abs(dir) < max_angle && abs(dir) > min_angle) {
				float mid_x = 0.5 * (ellipses[i].center.x + ellipses[j].center.x);
				float mid_y = 0.5 * (ellipses[i].center.y + ellipses[j].center.y);
				pair_centres.push_back(cv::Point2f(mid_x, mid_y));
				cv::circle(img_in, ellipses[i].center, ellipses[i].size.height, cv::Scalar(0, 0, 255), 2);
				cv::circle(img_in, ellipses[j].center, ellipses[j].size.height, cv::Scalar(0, 0, 255), 2);
				cv::circle(img_in, pair_centres.back(), 3, cv::Scalar(255, 0, 0), 2);
			}
		}
	}

	if (pair_centres.size() == 0) {
		std::cout << "Could not find finish line" << std::endl;
		return false;
	}

	
	cv::imshow("Display", img_in);

	pos_out.x = pair_centres[0].x;
	pos_out.y = pair_centres[0].y;
	pos_out.track_angle = M_PI;	// TODO
}


// Returns vector of 3 points, which are locations of 
// go signals. Returns empty vector if failure
std::vector<Point> Vision::findGoSignal(Point finish_line_pos_in, int camera) {

	cv::Point2f finish_line_pos(finish_line_pos_in.x, finish_line_pos_in.y);
	cv::Mat img;
	getCamImg(camera, img);
	applyTrans(img, camera);

	// Make grayscale copy, Reduce noise with a kernel 5x5
	cv::Mat temp;
	cv::cvtColor(img, temp, CV_BGR2GRAY);
	cv::blur(temp, temp, cv::Size(5, 5) );

	// Canny edge detection
	int threshold = 35;
	cv::Canny(temp, temp, threshold, threshold * 3, 3);

	cv::imshow("Display", temp);
	cv::waitKey();
		
	// Find contours from Canny image
	// Warning, findContours affects input image
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	
	// Fit ellipses to contours
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {

		if (contours[i].size() < 25)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);	

		// Check circularity and size
		float max_diameter = GO_SIGNAL_DIAMETER_PIX * 1.2;
		float min_diameter = GO_SIGNAL_DIAMETER_PIX * 0.8;
		if (r.size.height/r.size.width > 1.2 || 
			r.size.height > max_diameter ||
			r.size.height < min_diameter)
			continue;

		// Check distance to finish line is below threshold
		if (dist(finish_line_pos, r.center) < GO_SIGNAL_MAX_DIST_TO_FINISH_LINE_PIX)
		{
			ellipses.push_back(r);
			cv::ellipse(img, r, cv::Scalar(100,200,100), 3);
		}
	}
	
	for (std::size_t i = 0; i < ellipses.size() - 2; i++) {
		for (std::size_t j = i + 1; j < ellipses.size() - 1; j++) {
			for (std::size_t k = j + 1; k < ellipses.size(); k++) {

				// First check they are collinear
				if (isCollinear(ellipses[i].center, ellipses[j].center, ellipses[k].center)) {
					
					float dist_thresh = 20;	// Pixels
					float dist1 = dist(ellipses[i].center, ellipses[j].center);
					float dist2 = dist(ellipses[j].center, ellipses[k].center);
					float dist3 = dist(ellipses[i].center, ellipses[k].center);

					// Check that they are somewhat equidistant
					if (dist1 < dist_thresh || dist2 < dist_thresh || dist3 < dist_thresh)
						continue;
					if (!(abs(dist1 - dist2) < dist_thresh ||
						abs(dist1 - dist3) < dist_thresh ||
						abs(dist2 - dist3) < dist_thresh))
						continue;
					
					int b = rand() % 255;
					int g = rand() % 255;
					int r = rand() % 255;
					cv::line(img, ellipses[i].center, ellipses[j].center, cv::Scalar(b, g, r), 2);
					cv::line(img, ellipses[j].center, ellipses[k].center, cv::Scalar(b, g, r), 2);
					cv::imshow("Display", img);

					std::vector<Point> result;
					result.push_back(Point(ellipses[i].center.x, ellipses[i].center.y));
					result.push_back(Point(ellipses[j].center.x, ellipses[j].center.y));
					result.push_back(Point(ellipses[k].center.x, ellipses[k].center.y));
					return result;
				}
			}
		}
	}

	return std::vector<Point>();
}

// Returns true if the 3 points are collinear (within some threshold)
bool Vision::isCollinear(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
	float thresh = 20 * M_PI/180;	// Radians
	float angle1 = angle(p1, p2);
	float angle2 = angle(p2, p3);
	return abs(angle1 - angle2) < thresh ||
		abs(abs(angle1 - angle2) - M_PI) < thresh;
}

// Waits until go signal given, then returns true
bool Vision::waitForGo(int camera, std::vector<Point> signal_pos) {

	// To distinguish if signal is on/off, use luminosity + saturation
	// Take off value from average over 4 pictures
	int lum_sat_off[3] = {0, 0, 0};
	int num_off_samples = 4;
	float cone_diameter = 180;// 0.1 / M_PER_PIX;
	for (int i = 0; i < num_off_samples; i++) {
		cv::Mat img_temp;
		getCamImg(camera, img_temp);
		applyTrans(img_temp, camera);
		cv::cvtColor(img_temp, img_temp, CV_BGR2HLS);
		for (int j = 0; j < 3; j++) {
			cv::Scalar vals = getColor(img_temp, cv::Point2f(signal_pos[j].x, signal_pos[j].x), cone_diameter);
			lum_sat_off[j] += vals[1] + vals[2];
		}
	}
	for (int i = 0; i < 3; i++) {
		lum_sat_off[i] = lum_sat_off[i] / num_off_samples;
	}
	std::cout << "Off lum sat sums: " << lum_sat_off[0] << 
		" " << lum_sat_off[1] << " " << lum_sat_off[2] << std::endl;
	std::cout << "Waiting ..." << std::endl;
	
	// Start monitoring
	long long last_update_time = time_now();
	int on_time = 0;
	int go_time = 5000;	// 5 seconds
	int lum_sat_thresh = 6;
	while (true) {
		cv::Mat img_temp;
		getCamImg(camera, img_temp);
		applyTrans(img_temp, camera);
		cv::cvtColor(img_temp, img_temp, CV_BGR2HLS);
		bool on_detected = false;
		for (int j = 0; j < 3; j++) {
			cv::Scalar vals = getColor(img_temp,  cv::Point2f(signal_pos[j].x, signal_pos[j].x), cone_diameter);			
			if (lum_sat_off[j]  + lum_sat_thresh < vals[1] + vals[2]) {
				on_detected = true;
				break;
			}
		}
		if (on_detected) {
			on_time += time_now() - last_update_time;
			std::cout << "Time before go: " << go_time - on_time << std::endl;
		}
		if (on_time > go_time)
			return true;
		last_update_time = time_now();
	}
}

cv::Scalar Vision::getColor(cv::Mat& img, cv::Point2f p, int pix_length) {

	// Define region of interest
	cv::Rect roi(p.x - pix_length, p.y - pix_length, 2 * pix_length, 2 * pix_length);

	// Make sure roi is within image bounds
	while (roi.x + roi.width >= img.cols)
		roi.x -= 1;
	while (roi.x < 0)
		roi.x += 1;
	while (roi.y + roi.height >= img.rows)
		roi.y -= 1;
	while (roi.y < 0)
		roi.y += 1;

	// Get image roi
	cv::Mat img_roi = img(roi);

	// Take mean over roi
	return mean(img_roi);
}

// Draws a black border around a perspective transformed image
// Used in getMidpoints
void Vision::applyBlackBorder(cv::Mat& img, int camera) {

	// Get input image dimensions
	int width = 1280;
	int height = 720;
	if (roborealm_.connected) {
		roborealm_.getDimension(&width, &height);
	}

	// Find bounding box for transformed image corners
	std::vector<cv::Point2f> orig_corners, trans_corners;
	orig_corners.push_back(cv::Point2f(0, 0));
	orig_corners.push_back(cv::Point2f(width, 0));
	orig_corners.push_back(cv::Point2f(width, height));
	orig_corners.push_back(cv::Point2f(0, height));
	cv::perspectiveTransform(orig_corners, trans_corners, transform_mats_[camera]);

	// Draw lines
	for (int i = 0; i < 4; i++) {
		if (img.channels() == 3) {
			cv::line(img, trans_corners[i], trans_corners[(i+1)%4], cv::Scalar(0,0,0), 3);
		} else if (img.channels() == 1) {
			cv::line(img, trans_corners[i], trans_corners[(i+1)%4], cv::Scalar(0), 3);
		}
	}
}

cv::Point getCentre(std::vector<cv::Point> corners) 
{
	cv::Point sum_point(0,0);
	double n = 0;

	for (std::size_t i = 0; i < corners.size(); i++)
	{
		sum_point += corners[i];
		n++;
	}

	return sum_point*(1/n);

}

std::vector<Point> Vision::getMidpoints(cv::Mat& img_in,
cv::Mat& img_white_warped, cv::Point2f start_pos, float start_dir, int camera) {
		
	cv::Mat img_thresh, img_dist, img_blur, img_grad_x, img_grad_y;

	// Apply color threshold to get road as white (255), everything 
	// else black (0)
	img_thresh = img_in.clone();
	colorThresh(img_thresh, camera);

	// Dilate to remove noise
	cv::Mat dilate_element = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,cv::Size(4, 4));
	cv::dilate(img_thresh, img_thresh, dilate_element);

	// Apply black border
	applyBlackBorder(img_thresh, camera);

	// Distance transform: makes each pixel value the distance to 
	// nearest zero (black) pixel
	cv::distanceTransform(img_thresh, img_dist, CV_DIST_L2, 3);

	// Get image gradients of img_dist. These will be used for centering 
	// the midpoints to the middle of the track
	cv::blur(img_dist, img_blur, cv::Size(10, 10));
	cv::Sobel(img_blur, img_grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::Sobel(img_blur, img_grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

	std::vector<Point> midpoints;
	midpoints.push_back(Point(start_pos.x, start_pos.y, start_dir));

	float step_size = 10;	// Distance between midpoints in pixels
	float adjust_factor = 0.6;

	for (int i = 0; i < 1000; i++) {

		// Extrapolate forward using last point
		float temp_x = midpoints.back().x + step_size * cos(midpoints.back().track_angle);
		float temp_y = midpoints.back().y + step_size * sin(midpoints.back().track_angle);

		// Stop if outside image
		cv::Point2f temp(temp_x, temp_y);
		if (!inImg(img_white_warped, temp_x, temp_y) || 
			img_white_warped.at<uchar>((int)temp_y, (int)temp_x) != 255)
			return midpoints;

		// Stop if we have somehow looped back to the start
		if (i > 2 + (ROAD_WIDTH / M_PER_PIX)/step_size && 
			midpoints[0].dist(Point(temp_x, temp_y)) < 0.5 * ROAD_WIDTH / M_PER_PIX)
			return midpoints;
		
		// Adjust slightly towards centre of track
		temp_x += adjust_factor * img_grad_x.at<short>((int)temp_y, (int)temp_x);
		temp_y += adjust_factor * img_grad_y.at<short>((int)temp_y, (int)temp_x);

		float angle = midpoints.back().angle(Point(temp_x, temp_y));
		float new_x = midpoints.back().x + step_size * cos(angle);
		float new_y = midpoints.back().y + step_size * sin(angle);
		midpoints.push_back(Point(new_x, new_y, angle));
	}
	checkStrictness(img_dist, midpoints);
	return midpoints;
}

std::vector<Point> Vision::getMidpointsManual(cv::Mat& img_in,
cv::Mat& img_white_warped, int camera) {

	// Apply color threshold to get road as white (255), everything 
	// else black (0)
	cv::Mat img_thresh = img_in.clone();
	cv::Mat img_dist;
	colorThresh(img_thresh, camera);

	// Dilate to remove noise
	cv::Mat dilate_element = cv::getStructuringElement(
		cv::MORPH_ELLIPSE,cv::Size(4, 4));
	cv::dilate(img_thresh, img_thresh, dilate_element);

	// Apply black border
	applyBlackBorder(img_thresh, camera);

	// Distance transform: makes each pixel value the distance to 
	// nearest zero (black) pixel
	cv::distanceTransform(img_thresh, img_dist, CV_DIST_L2, 3); 

	// Get starting position from first click
	cv::Mat img_temp = img_in.clone();
	cv::namedWindow("Mouse clicks");
	cv::imshow("Mouse clicks", img_temp);
	cv::setMouseCallback("Mouse clicks", onMouse);
	mouse_click_changed = false;
	std::cout << "Click to select start position" << std::endl;
	while (!mouse_click_changed) {
		cv::imshow("Mouse clicks", img_temp);
		cv::waitKey(50);
	}
	cv::Point2f start_pos(mouse_l_click_pos.x, mouse_l_click_pos.y);
	mouse_click_changed = false;

	// Get starting direction from another click
	std::cout << "Click again to set start direction" << std::endl;
	while (!mouse_click_changed) {
		img_temp = img_in.clone();
		cv::line(img_temp, start_pos, mouse_pos, cv::Scalar(0, 0, 255));
		cv::imshow("Mouse clicks", img_temp);
		cv::waitKey(50);
	}
	float start_dir = angle(start_pos, mouse_l_click_pos);

	// Get midpoints
	std::vector<Point> midpoints = getMidpoints(img_in, img_white_warped, start_pos, 
		start_dir, camera);
	checkStrictness(img_dist, midpoints);

	std::cout << "Click and drag to edit path. Spacebar when done." << std::endl;
	std::cout << "Press c to add new path segment" << std::endl;
	char keypress = 0;
	int num_paths = 1;
	while (keypress != ' ') {
		img_temp = img_in.clone();

		// Press c for creating new path segment
		if (keypress == 'c') {

			// Get starting position from first click
			mouse_click_changed = false;
			std::cout << "Click to select start position" << std::endl;
			while (!mouse_click_changed) {
				cv::imshow("Mouse clicks", img_temp);
				cv::waitKey(50);
			}
			cv::Point2f new_start_pos(mouse_l_click_pos.x, mouse_l_click_pos.y);
			mouse_click_changed = false;

			// Get starting direction from another click
			std::cout << "Click again to set start direction" << std::endl;
			while (!mouse_click_changed) {
				img_temp = img_in.clone();
				cv::line(img_temp, new_start_pos, mouse_pos, cv::Scalar(0, 0, 255));
				cv::imshow("Mouse clicks", img_temp);
				cv::waitKey(50);
			}
			float new_start_dir = angle(new_start_pos, mouse_l_click_pos);

			// Get midpoints
			std::vector<Point> new_midpoints = getMidpoints(img_in, img_white_warped, new_start_pos, 
				new_start_dir, camera);

			// Join and set to new path number
			for (std::size_t i = 0; i < new_midpoints.size(); i++) {
				new_midpoints[i].path_num = num_paths;
				midpoints.push_back(new_midpoints[i]);
			}
			num_paths++;
		}

		// Draw dots and lines
		for (std::size_t i = 0; i < midpoints.size(); i++) {
			cv::Scalar color;
			cv::Point2f cv_point(midpoints[i].x, midpoints[i].y);
			if (midpoints[i].locked) {
				color = cv::Scalar(0, 0, 255);	// Red
			} else {
				color = cv::Scalar(255, 100, 0);	// Blueish
			}
			cv::circle(img_temp, cv_point, 3, color);
			if (i != 0) {
				if (midpoints[i].path_num == midpoints[i-1].path_num) {
					cv::Point2f prev_cv_point(midpoints[i-1].x, midpoints[i-1].y);
					cv::line(img_temp, cv_point, prev_cv_point, cv::Scalar(0, 255, 0));
				}
			}
			if (midpoints[i].strict) {
				cv::circle(img_temp, cv_point, 5, cv::Scalar(100, 150, 200), 2);	// Yellow outline
			}
		}

		// Mouse drag for moving points
		int selected = -1;
		if (mouse_l_down) {
			selected = getClosest(midpoints, Point(mouse_pos.x, mouse_pos.y));
			midpoints[selected].x = mouse_pos.x;
			midpoints[selected].y = mouse_pos.y;
			midpoints[selected].locked = true;
			smoothPath(midpoints, selected);
			midpoints[selected].locked = false;
			addRemovePoints(midpoints);
			checkStrictness(img_dist, midpoints);
		}
		cv::imshow("Mouse clicks", img_temp);
		keypress = cv::waitKey(50);
	}

	cv::destroyWindow("Mouse clicks");	

	return midpoints;
}

// Add new points in between if distance is too large
// Remove points if too small
void Vision::addRemovePoints(std::vector<Point>& path) {
	std::size_t index = 0;
	double lower_dist = 7;	// pixels
	double ideal_dist = 10;
	double upper_dist = 13;
	while(index < path.size() - 1) {
		if (path[index].path_num != path[index+1].path_num) {
			index++;
			continue;
		}
		double distance = path[index].dist(path[index+1]);
		if (distance < lower_dist) {
			path.erase(path.begin() + index + 1);
			if (index < path.size() - 1) {
				path[index].track_angle = path[index].angle(path[index+1]);
			}
		} else if (distance > upper_dist) {
			double new_x = path[index].x + ideal_dist * cos(path[index].track_angle);
			double new_y = path[index].y + ideal_dist * sin(path[index].track_angle);
			Point new_point(new_x, new_y);
			new_point.path_num = path[index].path_num;
			new_point.track_angle = new_point.angle(path[index+1]);
			path.insert(path.begin() + index + 1, new_point); 
			index++;
		} else {
			index++;
		}
	}
}

// Smooth path by bringing points towards together
void Vision::smoothPath(std::vector<Point>& path, int selected) {
	int iterations = 10;
	double smoothing_factor = 0.1;
	int lower_neighbour = selected - 3;
	int upper_neighbour = selected + 3;
	if (lower_neighbour < 0) {
		lower_neighbour = 0;
	}
	if (upper_neighbour > path.size() - 1) {
		upper_neighbour = path.size() - 1;
	}
	for (int i = 0; i < iterations; i++) {
		for (std::size_t j = lower_neighbour + 1; j < upper_neighbour; j++) {
			if (path[j].locked)
				continue;

			// Move towards previous point
			if (path[j-1].path_num == path[j].path_num) {
				double angle_to_prev = path[j-1].track_angle - M_PI;
				double dist_to_prev = path[j-1].dist(path[j]);
				path[j].x += dist_to_prev * cos(angle_to_prev) * smoothing_factor;
				path[j].y += dist_to_prev * sin(angle_to_prev) * smoothing_factor;
			}

			// Move towards next point
			if (j + 1 < path.size() && path[j].path_num == path[j+1].path_num) {
				double angle_to_next = path[j].track_angle;
				double dist_to_next = path[j].dist(path[j+1]);
				path[j].x += dist_to_next * cos(angle_to_next) * smoothing_factor;
				path[j].y += dist_to_next * sin(angle_to_next) * smoothing_factor;
			}

			// Update track angles
			path[j-1].track_angle = path[j-1].angle(path[j]);
			if (j + 1 < path.size() && path[j].path_num == path[j+1].path_num) {
				path[j].track_angle = path[j].angle(path[j+1]);
			} else {
				path[j].track_angle = path[j-1].track_angle;
			}
		}
	}		
}

// Set each point's strictness. Strict for points in a tunnel
void Vision::checkStrictness(cv::Mat& img_dist, std::vector<Point>& path) {

	// Assign strictness depends on road width,
	// as tunnel is detected as a wider road
	float min_dist = ROAD_WIDTH * 0.6 / M_PER_PIX;
	for (std::size_t i = 0; i < path.size(); i++) {
		float this_dist = img_dist.at<float>((int) path[i].y, path[i].x);
		if (this_dist > min_dist) {			
			path[i].strict = true;
		}
	}
	
	// Check that the condition for minimum number of points
	// in a straight line is met
	int min_adjacent = 4;
	float max_angle_diff = 10 * M_PI / 180;
	int strict_buffer = 3;
	for (std::size_t i = 0; i < path.size(); i++) {
		if (!path[i].strict) {
			continue;
		}
		
		int adjacent_count = 0;
		
		// Count forwards
		for (std::size_t j = i; j < path.size(); j++) {
			float angle_diff = abs(path[j].track_angle - path[i].track_angle);
			if (angle_diff > M_PI) {
				angle_diff = 2 * M_PI - abs(angle_diff);
			}
			if (!path[j].strict || angle_diff > max_angle_diff) {
				break;
			} 
			else {
				adjacent_count++;
			}
		}

		if (adjacent_count < min_adjacent) {
			path[i].strict = false;
		} else {

			// We can skip some adjacency checks
			i += adjacent_count - 1;

			// Make a few at the start and end strict
			int lower = i - strict_buffer;
			int higher = i + strict_buffer;
			if (lower < 0) {
				lower = 0;
			}
			if (higher >= path.size()) {
				higher = path.size() - 1;
			}
			for (int j = lower; j < higher; j++) {
				path[j].strict = true;
			}
		}
	}
}

// Color threshold on img to get binary image with
// road white and everything else black
void Vision::colorThresh(cv::Mat& img, int camera) {

	// Make hls copy
	cv::Mat img_hls;
	cv::cvtColor(img, img_hls, CV_BGR2HLS_FULL);

	// Use green threshold
	// Green is used due to high contrast between road and "grass"
	inRange(img_hls, lower_color_thresh_[camera],
		upper_color_thresh_[camera], img);
	img = 255 - img;	// Flip white and black
}

// Applies perspective transform to an image
void Vision::applyTrans(cv::Mat& img, int camera) {
	cv::warpPerspective(img, img, transform_mats_[camera],
		transform_sizes_[camera]);
}

void Vision::applyInvTrans(cv::Mat& img, int camera) {
	cv::warpPerspective(img, img, inv_transform_mats_[camera],
		img.size());
}

// Given a image of the racetrack, perspective transforms it to get
// top down view aligned with circle markers and scaled to 1mm:1pixel
// Centre of marker pair is provided in pair_centre.
// Returns true if success in finding marker pair
bool Vision::transformTrackImage(cv::Mat& img_in, cv::Mat& img_thresh, cv::Point2f& pair_centre) {
	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);

	//cv::imshow("Display", img_in);
	//cv::waitKey();

	// Convert to hsv
	cv::Mat img_hsv;
	cv::cvtColor(img_in, img_hsv, CV_BGR2HSV);

	// Use green threshold
	// Green is used due to high contrast between road and "grass"
	inRange(img_hsv, cv::Scalar(25, 0, 50),
		cv::Scalar(40, 255, 255), img_thresh);

	//cv::imshow("Display", img_thresh);
	//cv::waitKey();

	// Canny edge detection
	cv::Mat dst;
	int threshold = 50;
	cv::Canny(img_thresh, dst, threshold, threshold * 3, 3);

	//cv::imshow("Display", dst);
	//cv::waitKey();

	// Make color format for showing stuff
	cv::Mat cdst;
	cv::cvtColor(dst, cdst, CV_GRAY2BGR);
	
	// Find contours from Canny image
	// Warning, findContours affects input image
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	// Fit ellipses to contours and keep circular ones
	std::vector<cv::RotatedRect> ellipses;
	for (std::size_t i = 0; i < contours.size(); i++) {
		if (contours[i].size() < 5)
			continue;
		cv::RotatedRect r = cv::fitEllipse(contours[i]);	
		

		// Check circularity and that it isn't too small
		if (r.size.height/r.size.width < 1.5 && r.size.area() > 100)
			ellipses.push_back(r);

	}

	// Draw ellipses
	for (std::size_t i = 0; i < ellipses.size(); i++) {
		cv::ellipse(cdst, ellipses[i], cv::Scalar(100,200,100), 2);
	}

	//cv::imshow("Display", cdst);
	//cv::waitKey();

	// Now look for the circle pairs marking a checkpoint/finish line
	// This will be used for the perspective transform
	int index1 = -1;
	int index2 = -1;
	float pair_error = 99999;
	float scale;

	// Go through each ellipse...
	for (std::size_t i = 0; i < ellipses.size(); i++) {

		// Get an estimate of metres per pixel scale from size of ellipse
		float diameter = (ellipses[i].size.width + ellipses[i].size.height) / 2.0;
		float m_per_pixel = CIRCLE_MARK_DIAMETER / diameter;

		// Go through other ellipses...
		for (std::size_t j = i + 1; j < ellipses.size(); j++) {

			// Check size similiarity. Shouldn't be radically different
			if (ellipses[i].size.width / ellipses[j].size.width > 2 || 
				ellipses[i].size.width / ellipses[j].size.width < 0.5) 
				continue;

			// Compare distance between them to the known distance between
			// the markers. This will be used as a confidence rating
			float dist_in_pix = dist(ellipses[i].center, ellipses[j].center);
			float dist_in_m = m_per_pixel * dist_in_pix;
			float this_pair_error = abs(dist_in_m - CIRCLE_MARK_DIST);

			if (this_pair_error < pair_error) {
				index1 = i;
				index2 = j;
				pair_error = this_pair_error;
				scale = (CIRCLE_MARK_DIST/M_PER_PIX) / dist_in_pix;
			}
		}
	}
	
	if (index1 == -1)	// Failed to find any marker pair
		return false;

	cv::Point2f p1(ellipses[index1].center.x, ellipses[index1].center.y);
	float foox = ellipses[index1].size.height * cos(ellipses[index1].angle) / 2;
	float fooy = ellipses[index1].size.height * sin(ellipses[index1].angle) / 2;
	cv::Point2f p2 = p1 + cv::Point2f(foox, fooy);
	cv::line(cdst, p1, p2, cv::Scalar(0,0,255), 2);
	
	cv::Point2f bp1(ellipses[index2].center.x, ellipses[index2].center.y);
	float bfoox = ellipses[index2].size.height * cos(ellipses[index2].angle) / 2;
	float bfooy = ellipses[index2].size.height * sin(ellipses[index2].angle) / 2;
	cv::Point2f bp2 = bp1 + cv::Point2f(bfoox, bfooy);
	cv::line(cdst, bp1, bp2, cv::Scalar(255,0,0), 2);

	cv::imshow("Display",cdst);
	cv::waitKey();

	float mid_x = (ellipses[index1].center.x + ellipses[index2].center.x) / 2;
	float mid_y = (ellipses[index1].center.y + ellipses[index2].center.y) / 2;
	float angle = M_PI;	// TODO

	// Resize
	cv::resize(img_thresh, img_thresh, cv::Size(0,0), scale, scale); // x scale == y scale
	cv::resize(cdst, cdst, cv::Size(0,0), scale, scale);
	cv::imshow("Display", img_thresh);
	cv::waitKey();
	mid_x *= scale;
	mid_y *= scale;

	pair_centre.x = mid_x;
	pair_centre.y = mid_y;
	/*
	// TODO handle other possible rotations, checkpoint
	// Check if markers detected are from a finish line 

	// Define origin in pixels at top left of the tile found
	float side_length = TILE_M_LENGTH / M_PER_PIX;
	int centre_x = FINISH_SOURCE_MARKER_X * SOURCE_M_PER_PIX / M_PER_PIX;
	int centre_y = FINISH_SOURCE_MARKER_Y * SOURCE_M_PER_PIX / M_PER_PIX;
	int origin_x = mid_x - centre_x;
	int origin_y = mid_y - centre_y;

	// Define region of interest 
	cv::Rect roi(origin_x, origin_y, side_length, side_length);
	cv::Mat img_roi = img_thresh(roi);

	cv::line(cdst, cv::Point2f(roi.x, roi.y), cv::Point2f(roi.x + side_length, roi.y), cv::Scalar(50, 50, 150), 2);
	cv::line(cdst, cv::Point2f(roi.x + side_length, roi.y), cv::Point2f(roi.x + side_length, roi.y + side_length), cv::Scalar(50, 50, 150), 2);
	cv::line(cdst, cv::Point2f(roi.x + side_length, roi.y + side_length), cv::Point2f(roi.x, roi.y + side_length), cv::Scalar(50, 50, 150), 2);
	cv::line(cdst, cv::Point2f(roi.x, roi.y + side_length), cv::Point2f(roi.x, roi.y), cv::Scalar(50, 50, 150), 2);

	std::cout << finish_tile_1.compare(img_roi, 0) << std::endl;

	*/

	/*

	cv::imshow("Display", cdst);
	cv::waitKey();

 	std::vector<std::vector<int>> grid;
	for (int i = 0; i < 20; i++) {
		std::vector <int> v;
		grid.push_back(v);
		for (int j = 0; j < 20; j++) {
			grid.back().push_back(-1);
		}
	}
	int grid_row_origin = 10;
	int grid_col_origin = 10;
 	int row = 0;
	int col = -1;  // TODO change this and starting rot
	int rot = 0;
	while (row != 0 || col != 0) {

		// Define region of interest
		cv::Rect roi(origin_x + col * side_length, origin_y + row * side_length, side_length, side_length);
  		cv::Mat img_roi = img_thresh(roi);

		int tile_type = getTileType(img_roi, rot);
     	grid[grid_row_origin + row][grid_col_origin + col] = tile_type;
		rot = (rot + tiles[tile_type].exit_) % 4;

		// Ensure 0 <= rot <= 3
		rot = rot % 4;
		if (rot < 0) 
			rot += 4;

		if (rot == 0)
			col -= 1;
		else if (rot == 1)
			row += 1;
		else if (rot == 2)
			col += 1;
		else if (rot == 3)
			row -= 1;
	}

	for (int row = 0; row < 20; row++) {
		for (int col = 0; col < 20; col++) {
			std::cout << std::right << std::setw(3) << grid[row][col];
		}
		std::cout << std::endl;
	}
	*/


	/*
	// Calculate angle of line from ellipse 1 to ellipse 2
	// (anti-clockwise, starting from east)
	float dx = ellipses[index2].center.x - ellipses[index1].center.x;
	float dy = ellipses[index2].center.y - ellipses[index1].center.y;
	double angle = atan2(-dy, dx);

	// n = 0 for 0 radians, n = 1 for PI/2 radians, etc.
	int n = floor((angle/M_PI_2) + 0.5); 

	// Extract bounding vertices of each ellipse
	std::vector<cv::Point2f> corners1 = sortVertices(ellipses[index1]);
	std::vector<cv::Point2f> corners2 = sortVertices(ellipses[index2]);

	// Input Quadilateral or Image plane coordinates
    cv::Point2f input_quad[4]; 
	// Output Quadilateral or World plane coordinates
    cv::Point2f output_quad[4];
	double e1x = ellipses[index1].center.x;
	double e1y = ellipses[index1].center.y;
	double shift_diam = (CIRCLE_MARK_DIAMETER * 0.5) / M_PER_PIX;
	double shift_dist = CIRCLE_MARK_DIST / M_PER_PIX;

	// This is somewhat brute forced...
	std::vector<cv::Point2f> vertices;
	if (n == 0) {
		input_quad[0] = corners1[4];
		input_quad[1] = corners1[1];
		input_quad[2] = corners2[2];
		input_quad[3] = corners2[3];
		output_quad[0] = cv::Point2f(e1x - shift_diam, e1y + shift_diam);
		output_quad[1] = cv::Point2f(e1x - shift_diam, e1y - shift_diam);
		output_quad[2] = cv::Point2f(e1x + shift_diam + shift_dist, e1y - shift_diam);
		output_quad[3] = cv::Point2f(e1x + shift_diam + shift_dist, e1y + shift_diam);
	}
	else if (n == 1) {
		input_quad[0] = corners2[0];
		input_quad[1] = corners2[1];
		input_quad[2] = corners1[2];
		input_quad[3] = corners1[3];
		output_quad[0] = cv::Point2f(e1x - shift_diam, e1y - shift_dist - shift_diam);
		output_quad[1] = cv::Point2f(e1x + shift_diam, e1y - shift_dist - shift_diam);
		output_quad[2] = cv::Point2f(e1x + shift_diam, e1y + shift_diam);
		output_quad[3] = cv::Point2f(e1x - shift_diam, e1y + shift_diam);
	}
	else if (n == 2 || n == -2) {
		input_quad[0] = corners2[0];
		input_quad[1] = corners1[1];
		input_quad[2] = corners1[2];
		input_quad[3] = corners2[3];
		output_quad[0] = cv::Point2f(e1x - shift_dist - shift_diam, e1y - shift_diam);
		output_quad[1] = cv::Point2f(e1x + shift_diam, e1y - shift_diam);
		output_quad[2] = cv::Point2f(e1x + shift_diam, e1y + shift_diam);
		output_quad[3] = cv::Point2f(e1x - shift_dist - shift_diam, e1y + shift_diam);
	}
	else if (n == -1) {
		input_quad[0] = corners1[0];
		input_quad[1] = corners1[1];
		input_quad[2] = corners2[2];
		input_quad[3] = corners2[3];
		output_quad[0] = cv::Point2f(e1x - shift_diam, e1y - shift_diam);
		output_quad[1] = cv::Point2f(e1x + shift_diam, e1y - shift_diam);
		output_quad[2] = cv::Point2f(e1x + shift_diam, e1y + shift_dist + shift_diam);
		output_quad[3] = cv::Point2f(e1x - shift_diam, e1y + shift_dist + shift_diam);
	}

	// Draw lines showing quadrilaterals
	for (int i = 0; i < 4; i++) {
		cv::line(cdst, input_quad[i], input_quad[(i+1)%4], cv::Scalar(100,100,255), 2);
		cv::line(cdst, output_quad[i], output_quad[(i+1)%4], cv::Scalar(255,100,100), 2);
	}
	cv::imshow("Display", cdst);
	cv::waitKey();

    //Input and Output Image;
    cv::Mat transform, output;
      
    // Get the Perspective Transform Matrix i.e. lambda 
    transform = getPerspectiveTransform( input_quad, output_quad );
    // Apply the Perspective Transform just found to the src image
    warpPerspective(img_in, output, transform, output.size() );
	*/
	
	cv::imshow("Display", img_thresh);
	cv::waitKey();
	

}

// Returns euclidean distance between two opencv points
float Vision::dist(cv::Point2f& p1, cv::Point2f& p2) {
	return sqrt(pow(p1.x - p2.x, 2) + (pow(p1.y - p2.y, 2)));
}

// Returns euclidean distance squared between two opencv points
float Vision::distSq(cv::Point2f& p1, cv::Point2f& p2) {
	return pow(p1.x - p2.x, 2) + (pow(p1.y - p2.y, 2));
}

float Vision::angle(cv::Point2f& p1, cv::Point2f& p2) {
	return atan2(p2.y - p1.y, p2.x - p1.x);
}

// Returns index of point in path closest to target point
std::size_t Vision::getClosest(std::vector<Point> path, Point target) {
	double min_dist_sq = 999999;
	int min_index;
	double dist_sq;
	for (std::size_t i = 0; i < path.size(); i++) {
		dist_sq = target.distSquared(path[i]);
		if (dist_sq < min_dist_sq) {
			min_dist_sq = dist_sq;
			min_index = i;
		}
	}
	return min_index;
}

// Return vertices of a rotated rectangle in clockwise order,
// starting from top left
std::vector<cv::Point2f> Vision::sortVertices(cv::RotatedRect& rect) {
	cv::Point2f corners[4];
	rect.points(corners);

	std::vector<cv::Point2f> top, bot;

    for (int i = 0; i < 4; i++)
    {
        if (corners[i].y < rect.center.y)
            top.push_back(corners[i]);
        else
            bot.push_back(corners[i]);
    }

    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    std::vector<cv::Point2f> result;
    result.push_back(tl);
    result.push_back(tr);
    result.push_back(br);
    result.push_back(bl);

	return result;
}

// TODO make more efficient & precise. Bisection method?
// Point returned guaranteed to be within boundary
cv::Point2f Vision::getBoundary(cv::Mat& img_thresh, cv::Point2f start, float dir,
								bool& at_img_edge, uchar road_color){
	float step_size = 1;	
	float dy = step_size * sin(dir);
	float dx = step_size * cos(dir);
	float x = start.x;
	float y = start.y;

	if (!inImg(img_thresh, x, y)) {
		at_img_edge = true;
		return cv::Point2f(0, 0);
	}
	else if (!inImg(img_thresh, x + dx, y + dy)) {
		at_img_edge = true;
		return start;
	}

	uchar next_value = img_thresh.at<uchar>((int) (y + dy), (int) (x + dx));

	while (next_value == road_color) {
		x += dx;
		y += dy;
		if (!inImg(img_thresh, x + dx, y + dy)) {
			at_img_edge = true;
			return cv::Point2f(x, y);
		}
		next_value = img_thresh.at<uchar>((int) (y + dy), (int) (x + dx));
	}
	at_img_edge = false;
	return cv::Point2f(x, y);
}

bool Vision::inImg(cv::Mat& img, int x, int y) {
	if (x < 0 || y < 0 || x >= img.cols || y >= img.rows)
		return false;
	return true;
}

int Vision::getTileType(cv::Mat& img_roi, int rot) {
	
	float max_percent = 0;
	float best_tile;

	// Go over all the tile types
	for (std::size_t i = 0; i < tiles.size(); i++) {
		float percent = tiles[i].compare(img_roi, rot);
		if (percent > max_percent) {
			max_percent = percent;
			best_tile = i;
		}
	}

	return best_tile;
}

bool isCarValid(cv::Point2f pos, double angle, long long time);

void Vision::testColorThresh(int camera) {
	cv::Mat img;
	getCamImg(camera, img);
	colorThresh(img, camera);
	imshow("Display", img);
}

void Vision::setColorThresh(int camera, cv::Scalar lower, cv::Scalar upper) {
	lower_color_thresh_[camera] = lower;
	upper_color_thresh_[camera] = upper;
}

void Vision::previewImg(int camera) {
	cv::Mat img;
	getCamImg(camera, img);
	imshow("Display", img);
}
