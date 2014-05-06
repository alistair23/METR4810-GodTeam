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

	void startVision();

private:
	MyCar *my_car_;
	MyForm^ form_;
	Vision *vision_;
	CarView::View *view_;

	void updateView();

};
}

#endif