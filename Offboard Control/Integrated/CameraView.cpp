#include "stdafx.h"
#include "CameraView.h"
#include "MyForm.h"
//ref class RaceControl::MyForm;

void RaceControl::CameraView::setParent(ref class MyForm^ f){

	this->pForm_ = f;
}

RaceControl::CameraView:: ~CameraView()
		{
			pForm_->camera_vision = false;
			if (components)
			{
				delete components;
			}
		}
