#include "stdafx.h"
#include "MyForm.h"
#include "PIDTuningForm.h"

using namespace System;
using namespace System::Windows::Forms;
System::Void RaceControl::MyForm::button4_Click(System::Object^  sender, System::EventArgs^  e) {
	PIDTuningForm^ form2 = gcnew PIDTuningForm();
	form2->setParent(this);
	form2->Show();
			 
}
