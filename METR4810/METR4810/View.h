#ifndef _INCLUDED_VIEW_H
#define _INCLUDED_VIEW_H

#include <cv.h>
#include <highgui.h>
#include <vector>

#include "Point.h"
#include "Car.h"

class View 
{
public:

	View(cv::Mat background, Car& my_car);
	
	void spin();	// Starts loop to continuously redraw

	void redraw();
	void drawNewDots(std::vector<Point>& segment);

private:

	Car& my_car_;

	const int frame_time_;	// milliseconds / frame

	cv::Mat background_;
	cv::Mat background_no_dots_;

};


#endif