#include <msclr/lock.h>
#include <cv.h>

#include "Controller.h"



using namespace System::Threading;
using namespace msclr;
using namespace RaceControl;

Controller::Controller() :
	my_car_(new MyCar()),
	vision_(new Vision()),
	view_(new View()),
	planner_(new LocalPlanner()),
	current_path_(new std::vector<Point>()),
	local_planning_on(false),
	car_tracking_on(false)

{
	form_ = gcnew MyForm();
	form_->setParent(this);

	while(!vision_->connectRoboRealm());

	
	Thread^ carDetectionThread = gcnew Thread( gcnew ThreadStart(this, &Controller::detectCar) );
	carDetectionThread -> Start();

	Thread^ localPlannerThread = gcnew Thread( gcnew ThreadStart(this, &Controller::runPlanner) );
	localPlannerThread -> Start();
	
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
	vision_->update();
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
	view_->redraw();
	planner_->setGlobalPath(track, camera);
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
			vision_->update();
			Car temp = vision_->getMyCarInfo();
			while (0 != Interlocked::Exchange(my_car_lock_, 1));
			my_car_->update(temp.getPos(), temp.getDir(), temp.getSpd());
			Interlocked::Exchange(my_car_lock_, 0);
			
			//cv::Mat img_bgr;
			//vision_->getCamImg(current_camera_, img_bgr);
			//vision_->applyTrans(img_bgr, vision_->transform_mats_[current_camera_]);
			//view_->setBackground(img_bgr);
			view_->updateMyCar(*my_car_);
			if (0 == Interlocked::Exchange(current_path_lock_, 1)) {
				std::cout << "Current path size: " << current_path_->size() << std::endl;
				view_->drawNewDots(*current_path_);
				Interlocked::Exchange(current_path_lock_, 0);
			}
			view_->redraw();
			//showImage(*(view_->getDisplayImage()));
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
			if (0 == Interlocked::Exchange(current_path_lock_, 1)) {
				if (0 == Interlocked::Exchange(my_car_lock_, 1)) {
					planner_->updateMyCar(my_car_->getPos(), my_car_->getDir(), my_car_->getSpd());
					Interlocked::Exchange(my_car_lock_, 0);
					std::cout << "Updated planner" << std::endl;
				}

				*current_path_ = planner_->getSegment(current_camera_);
				std::cout << "Got path " << current_path_->size() << std::endl;
				// Release the lock
				Interlocked::Exchange(current_path_lock_, 0);

				Thread::Sleep(50);
			}
		}
	}
}