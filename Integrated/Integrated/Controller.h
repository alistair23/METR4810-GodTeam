#ifndef _INCLUDED_CONTROLLER_H
#define _INCLUDED_CONTROLLER_H

#include "MyCar.h"
#include "MyForm.h"
#include "Vision.h"
#include "View.h"

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

private:
	MyCar *my_car_;
	MyForm^ form_;
	Vision *vision_;
	View *view_;
	int current_camera_;
	void updateView(Object^ stateInfo);
	void Controller::detectCar();
	

};
}

#endif