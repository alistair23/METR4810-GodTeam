#ifndef _INCLUDED_VIEW_H
#define _INCLUDED_VIEW_H

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <cv.h>
#include <highgui.h>
#include <math.h>

#include "Racetrack.h"
#include "Car.h"

class View 
{
public:

	View(Racetrack& racetrack, Car& my_car);
	
	void spin();	// Starts loop to continuously redraw

	void redraw();

private:

	Racetrack& racetrack_;
	Car& my_car_;

	const int scale_;	// pixels / metres
	const int frame_time_;	// milliseconds / frame
	const double map_width_;	// metres
	const double map_height_;	// metres

	cv::Mat background_;

};


#endif