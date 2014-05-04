#include "stdafx.h"
#include "PIDTuningForm.h"
#include "MyForm.h"


System::Void RaceControl::PIDTuningForm::button1_Click(System::Object^  sender, System::EventArgs^  e) {
		this->pForm->kp_l = System::Convert::ToInt16(this->textBox1->Text);
		this->pForm->ki_l = System::Convert::ToInt16(this->textBox2->Text);
		this->pForm->kd_l = System::Convert::ToInt16(this->textBox3->Text);
		this->pForm->kp_r = System::Convert::ToInt16(this->textBox4->Text);
		this->pForm->ki_r = System::Convert::ToInt16(this->textBox5->Text);
		this->pForm->kd_r = System::Convert::ToInt16(this->textBox6->Text);
		this->pForm->setMotorGains();
		/*this->pForm->SetText(System::Convert::ToString(this->pForm->kp_l));
		this->pForm->SetText(System::Convert::ToString(this->pForm->ki_l));
		this->pForm->SetText(System::Convert::ToString(this->pForm->kd_l));
		this->pForm->SetText(System::Convert::ToString(this->pForm->kp_r));
		this->pForm->SetText(System::Convert::ToString(this->pForm->ki_r));
		this->pForm->SetText(System::Convert::ToString(this->pForm->kd_r));*/
		this->Close();
	}
void RaceControl::PIDTuningForm::setParent(ref class MyForm^ f)
               
{
                       
	this->pForm = f;
	this->textBox1->Text = System::Convert::ToString(this->pForm->kp_l);
	this->textBox2->Text = System::Convert::ToString(this->pForm->ki_l);
	this->textBox3->Text = System::Convert::ToString(this->pForm->kd_l);
	this->textBox4->Text = System::Convert::ToString(this->pForm->kp_r);
	this->textBox5->Text = System::Convert::ToString(this->pForm->ki_r);
	this->textBox6->Text = System::Convert::ToString(this->pForm->kd_r);
               
}

