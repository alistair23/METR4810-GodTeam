#ifndef _INCLUDED_CONTROLLER_H
#define _INCLUDED_CONTROLLER_H

#include "MyCar.h"
#include "MyForm.h"
#include "Vision.h"
#include "View.h"
#include "LocalPlanner.h"

using namespace System;
using namespace System::Windows::Forms;

namespace RaceControl{
public ref class Controller
{

public:
	Controller();
	~Controller();
	!Controller();

	void showImage(cv::Mat im);
	void getCameraTransform( int camera);
	void getMidPoints(int camera);

	cv::Mat* img;
	bool car_tracking_on;
	bool local_planning_on;

private:
	MyCar *my_car_;
	MyForm^ form_;
	Vision *vision_;
	View *view_;
	LocalPlanner *planner_;
	int current_camera_;
	std::vector<Point>* current_path_;
	int current_path_lock_;	// 0 if current_path_ is free, otherwise 1
	int my_car_lock_;
	void updateView(Object^ stateInfo);
	void detectCar();
	void runPlanner();
	

};
}

#endif