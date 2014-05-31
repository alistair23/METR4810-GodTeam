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
	void getCameraTransform(int camera, bool manual_mode);
	void getMidPoints(int camera, bool manual_mode);
	void connectToRoborealm(int port_num_1, int port_num_2, int port_num_3, int port_num_4, System::String^ ip_address, int num_cameras);
	void testColorThresh(int camera);
	void setColorThresh(int camera, int lower_hue, int lower_lum, int lower_sat,
		int upper_hue, int upper_lum, int upper_sat);
	void getObstacles(int camera);
	void getFinishLine(int camera);
	void getGoSignal(int camera);
	void launchOnGo(int camera);
	void enterPitstop();
	void exitPitstop();
	void previewImg(int camera);
	void saveToFile(int camera);
	void loadFromFile(int camera);

	cv::Mat* img;
	bool car_tracking_on;
	bool local_planning_on;
	bool go_signal_found;

private:
	MyCar *my_car_;
	MyForm^ form_;
	Vision *vision_;
	View *view_;
	LocalPlanner *planner_;
	int current_camera_;
	int current_midpoint_index_;
	int num_cameras_;
	std::vector<Point>* current_path_;
	int current_path_lock_;	// 0 if current_path_ is free, otherwise 1
	int my_car_lock_;
	long long old_time_;
	int path_index_;
	bool was_beyond_thresh_;
	bool did_tight_turn_;

	Point* finish_line_pos_;
	float finish_line_dir_;
	std::vector<Point>* go_signals_;
	bool wants_to_enter_pitstop;
	bool wants_to_exit_pitstop;

	void updateView(Object^ stateInfo);
	void detectCar();
	void runPlanner();
	void sendCarCommand();
	float getCurveRadius(RaceControl::Point &p0, RaceControl::Point &p1, RaceControl::Point &p2);
	bool pointsOnStraightLine(RaceControl::Point &p0, RaceControl::Point &p1, RaceControl::Point &p2);
	float getPursuitRadius();

};
}

#endif