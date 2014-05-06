#include <msclr/lock.h>
#include <cv.h>

#include "Controller.h"



using namespace System::Threading;
using namespace msclr;
using namespace RaceControl;

Controller::Controller() :
	my_car_(new MyCar()),
	vision_(new Vision()),
	view_(new CarView::View())
{
	form_ = gcnew MyForm();
	form_->setParent(this);

	cv::Mat bob = cv::imread("Resources/circle_marker.jpg");
	view_->setBackground(bob);
	startVision();

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
}

void Controller::startVision() {
	view_->createWindow();
	Thread^ viewThread = gcnew Thread( gcnew ThreadStart(this, &Controller::updateView ) );
}

void Controller::updateView() {
	view_->redraw();
}