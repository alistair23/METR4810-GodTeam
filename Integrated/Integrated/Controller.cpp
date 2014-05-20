#define _USE_MATH_DEFINES
#include <msclr/lock.h>
#include <cv.h>
#include <string>
#include <msclr\marshal_cppstd.h>	// For converting System::String^ to std::string

#include "Controller.h"
#include "CommonFunctions.h"
#include "math.h"


using namespace System::Threading;
using namespace msclr;
using namespace RaceControl;

Controller::Controller() :
	my_car_(new MyCar()),
	vision_(new Vision()),
	view_(new View()),
	planner_(new LocalPlanner()),
	current_camera_(0),
	num_cameras_(0),
	current_path_(new std::vector<Point>()),
	local_planning_on(false),
	car_tracking_on(false),
	go_signal_found(true)

{
	form_ = gcnew MyForm();
	form_->setParent(this);


	//while(!vision_->connectRoboRealm());

	

	Thread^ carDetectionThread = gcnew Thread( gcnew ThreadStart(this, &Controller::detectCar) );
	carDetectionThread -> Start();

	Thread^ localPlannerThread = gcnew Thread( gcnew ThreadStart(this, &Controller::runPlanner) );
	localPlannerThread -> Start();

	Thread^ commandCarThread = gcnew Thread( gcnew ThreadStart(this, &Controller::sendCarCommand) );
	commandCarThread -> Start();
	
	img = new cv::Mat();
	
	//Console::WriteLine(L"Hello World");
	Application::EnableVisualStyles();
    // Create the main window and run i
	//form = gcnew RaceControl::MyForm();
	Application::Run(form_);

}

Controller::~Controller() {
	this->!Controller();
}

Controller::!Controller() {
	delete my_car_;
	delete vision_;
	delete view_;
	delete img;
	delete current_path_;
}

void Controller::showImage(cv::Mat im) {
	std::cout << "vision is started" << std::endl;
	//view_->createWindow();
	
	std::cout << " updateView is called..." << std::endl;
	std::cout << "view is updating..." << std::endl;
	
	*img = im; //cv::imread("Resources/circle_marker.jpg");
	std::cout <<"columns:" <<(*img).cols << "rows:" << (*img).rows << std::endl;
	form_->DrawCVImage(img);
	//AutoResetEvent^ autoResetEvent     = gcnew AutoResetEvent(false);
	//System::Threading::TimerCallback^ viewTimer = gcnew TimerCallback(this, &Controller::updateView);
	//System::Threading::Timer^ stateTimer = gcnew System::Threading::Timer(viewTimer, autoResetEvent, 10, 10);
	//Thread^ viewThread = gcnew Thread( gcnew ThreadStart(this, &Controller::updateView ) );
	//viewThread->IsBackground = true;
	//viewThread -> Start();
	
	
}

void Controller::updateView(Object^ stateInfo) {
	//view_->redraw();
	while(form_->camera_vision)
	{
		std::cout << " updateView is called..." << std::endl;
		std::cout << "view is updating..." << std::endl;
		cv::Mat img = cv::imread("Resources/circle_marker.jpg");
		std::cout <<"columns:" <<img.cols << "rows:" << img.rows << std::endl;
		form_->DrawCVImage(&img);
		Thread::Sleep(10);
	}
	
}

void Controller::getCameraTransform( int camera)
{
	vision_->setupCamTransform(camera);
	//showImage(*(vision_->getDisplayImage()));
}

void Controller::getMidPoints(int camera)
{
	vision_->update(camera);
	Car temp = vision_->getMyCarInfo();
	cv::Mat img_bgr;
	vision_->getCamImg(camera, img_bgr);
	cv::Mat img_white(img_bgr.rows, img_bgr.cols, CV_8UC1, 255);
	vision_->applyTrans(img_bgr, vision_->transform_mats_[camera]);
	vision_->applyTrans(img_white, vision_->transform_mats_[camera]);
	view_->setBackground(img_bgr);
	std::vector<Point> track = vision_->getMidpoints(img_bgr, img_white,
		cv::Point2f(temp.getPos().x, temp.getPos().y), temp.getDir());
	view_->drawNewDots(track);
	view_->updateMyCar(*my_car_);
	planner_->setGlobalPath(track, camera);
	view_->redraw();
	//showImage(*(view_->getDisplayImage()));
}

void Controller::detectCar()
{
	while (true)
	{
		if(!car_tracking_on)
		{
			Thread::Sleep(10);
			
		}
		else
		{
			bool found_my_car = false;

			// Use predicted car position as a guess
			while (0 != Interlocked::Exchange(my_car_lock_, 1));
			Point guess(my_car_->getPos());
			Interlocked::Exchange(my_car_lock_, 0);

			// If near end of current global path, preferentially 
			// use next camera
			if (num_cameras_ > 1) {
				Point next_camera_guess;
				bool prefer_next_camera = false;
				double thresh = 0.2 / M_PER_PIX;
				int next_camera = (current_camera_ + 1) % num_cameras_;
				while (0 != Interlocked::Exchange(current_path_lock_, 1));
				if (guess.dist(planner_->getGlobalPathEnd(current_camera_)) < thresh) {
					next_camera_guess = planner_->getGlobalPathStart(next_camera);
					prefer_next_camera = true;
				}
				Interlocked::Exchange(current_path_lock_, 0);
				if (prefer_next_camera)
					found_my_car = vision_->update(next_camera, next_camera_guess);
			}
			int temp_camera = current_camera_;
			while (!found_my_car) {				
				if (current_camera_ == temp_camera) {
					while (0 != Interlocked::Exchange(my_car_lock_, 1));
					guess = my_car_->getPos();
					Interlocked::Exchange(my_car_lock_, 0);
					found_my_car = vision_->update(current_camera_, guess);
				}
				else {
					found_my_car = vision_->update(current_camera_);
				}

				// If we didn't find car in current camera, step through 
				// other cameras until car is found.
				if (!found_my_car)
					current_camera_ = (current_camera_ + 1) % num_cameras_;
			}
			
			Car temp = vision_->getMyCarInfo();
			while (0 != Interlocked::Exchange(my_car_lock_, 1));
			my_car_->update(temp.getPos(), temp.getDir(), temp.getSpd());
			view_->updateMyCar(*my_car_);
			Interlocked::Exchange(my_car_lock_, 0);
			
			//cv::Mat img_bgr;
			//vision_->getCamImg(current_camera_, img_bgr);
			//vision_->applyTrans(img_bgr, vision_->transform_mats_[current_camera_]);
			//view_->setBackground(img_bgr);
			
			if (0 == Interlocked::Exchange(current_path_lock_, 1)) {
				std::cout << "Current path size: " << current_path_->size() << std::endl;
				view_->drawNewDots(*current_path_);
				Interlocked::Exchange(current_path_lock_, 0);
			}
			view_->redraw();
		}
	}
}

void Controller::runPlanner(){

	while (true) {
		if (!this->local_planning_on)
		{
			Thread::Sleep(50);
		}
		else
		{
			while (0 != Interlocked::Exchange(my_car_lock_, 1));
			planner_->updateMyCar(my_car_->getPos(), my_car_->getDir(), my_car_->getSpd());
			Interlocked::Exchange(my_car_lock_, 0);

			while (0 != Interlocked::Exchange(current_path_lock_, 1));
			*current_path_ = planner_->getSegment(current_camera_);
			path_index_ = 0;
			//std::cout << "Got path " << current_path_->size() << std::endl;
			
			// Release the lock
			Interlocked::Exchange(current_path_lock_, 0);
			Thread::Sleep(50);
		}
	}
}

void Controller::sendCarCommand() {

	while(true)
	{
		if (!go_signal_found || !local_planning_on || current_path_->size() == 0)
		{
			Sleep(10);
			old_time_ = time_now();
		}
		else {
			long long update_time = time_now();

			while (0 != Interlocked::Exchange(my_car_lock_, 1)); 
			my_car_->step(0.001 * (update_time - old_time_));
			old_time_ = update_time;
			double dist_sq_thresh = 100;
			double max_speed = 25;
			double angle_thresh = 60 * M_PI / 180;

			while (0 != Interlocked::Exchange(current_path_lock_, 1));

			Point* goal = &(*current_path_)[path_index_];
			double dist_sq = my_car_->getPos().distSquared(*goal);

			while (dist_sq < dist_sq_thresh && path_index_ < current_path_->size() - 1) {
				path_index_++;
				goal = &(*current_path_)[path_index_];
				dist_sq = my_car_->getPos().distSquared(*goal);
			}

			if (path_index_ < current_path_->size()) {

				double angle = my_car_->getPos().angle(*goal) - my_car_->getDir();

				// A simple control algorithm: just head 
				// straight to waypoints. Will be
				// replaced later.
				while (angle > M_PI)
					angle -= 2 * M_PI;
				while (angle <= -M_PI)
					angle += 2 * M_PI;

				if (angle >= 0 && angle <= angle_thresh) {
					my_car_->setRSpeed((1 - angle/angle_thresh) * max_speed);
					my_car_->setLSpeed(max_speed);
				}
				else if (angle <= 0 && angle >= -angle_thresh) {
					my_car_->setRSpeed(max_speed);
					my_car_->setLSpeed((1 + angle/angle_thresh) * max_speed);
				}
				else if (angle > angle_thresh) { // && angle <= 180
					my_car_->setRSpeed(-max_speed);
					my_car_->setLSpeed(max_speed);
				}
				else {
					my_car_->setRSpeed(max_speed);
					my_car_->setLSpeed(-max_speed);
				}
			}
			else {
				my_car_->setRSpeed(0);
				my_car_->setLSpeed(0);
			}
			Interlocked::Exchange(my_car_lock_, 0);

			// Send commands over bluetooth
			std::cout <<"Right speed: " << my_car_->getRSpeed()<<std::endl;
			std::cout << "Left speed: " << my_car_->getLSpeed() << std::endl; 
			form_ ->setMotorSpeeds((int)(my_car_->getLSpeed()), (int)(my_car_->getRSpeed()));		

			// Release the lock
			Interlocked::Exchange(current_path_lock_, 0);

		}
		

		Thread::Sleep(100);
	}
}

void Controller::connectToRoborealm(int port_num_1, int port_num_2, int port_num_3, int port_num_4, System::String^ ip_address, int num_cameras){
	num_cameras_ = num_cameras;
	int temp[4] = {port_num_1, port_num_2, port_num_3, port_num_4};
	std::vector<int> port_nums;
	for (int i = 0; i < num_cameras; i++) {
		port_nums.push_back(temp[i]);
	}
	vision_->initCameras(port_nums, msclr::interop::marshal_as<std::string>(ip_address));
}
