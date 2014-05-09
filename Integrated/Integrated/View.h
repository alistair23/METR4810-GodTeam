#ifndef _INCLUDED_VIEW_H
#define _INCLUDED_VIEW_H

#include <cv.h>
#include <highgui.h>
#include <vector>

#include "Point.h"
#include "MyCar.h"

namespace RaceControl {

class View 
{
public:

	View();
	
	void createWindow();
	void setBackground(cv::Mat& background);
	void updateMyCar(MyCar my_car);
	void redraw();
	void drawNewDots(std::vector<Point>& segment);

	cv::Mat* getDisplayImage();
	Point getMousePos();

private:
	
	MyCar my_car_;
	const int frame_time_;	// milliseconds / frame
	
	cv::Mat background_;
	cv::Mat background_no_dots_;
	cv::Mat img_display;

};

}

#endif