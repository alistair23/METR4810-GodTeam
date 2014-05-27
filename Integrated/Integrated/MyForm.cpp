#include "stdafx.h"
#include "MyForm.h"
#include "PIDTuningForm.h"

using namespace System;
using namespace System::Windows::Forms;
/*System::Void RaceControl::MyForm::button4_Click(System::Object^  sender, System::EventArgs^  e) {
	PIDTuningForm^ form2 = gcnew PIDTuningForm();
	form2->setParent(this);
	form2->Show();		 
}*/

void RaceControl::MyForm::setParent(ref class Controller^ c) {
	this->controller_ = c;
}

bool RaceControl::MyForm::is_decimal(System::String^ s)
{
	System::Decimal n;
	bool m = System::Decimal::TryParse(s, n);
	return m;
}

void RaceControl::MyForm::intializeTrackbars()
{
	this->trackBar1->Minimum = -100;
	this->trackBar2->Minimum = -100;
	this->trackBar1->Value = 0;
	this->trackBar2->Value = 0;
	this->trackBar1->ValueChanged+=gcnew System::EventHandler(this, &MyForm::LeftTrackBarChanged);
	this->trackBar2->ValueChanged+=gcnew System::EventHandler(this, &MyForm::RightTrackBarChanged);
}
array<System::Byte>^ RaceControl::MyForm::interpretHexadecimalCommand()
{
	System::String^ s  = this->textBox1->Text;
	array<System::String^>^ hexvalues = s->Split(' ');
	array<unsigned char>^ byteArray = gcnew array<unsigned char>(hexvalues->Length);
	//int value = System::Convert::ToInt32(hexvalues[0], 16);
	//System::String^ charValue = System::Convert::ToString(value);
	//this->textBox2->AppendText(charValue);

	for (int i = 0; i<hexvalues->Length; i++)
	{
		// Convert the number expressed in base-16 to an integer. 
		int value = System::Convert::ToInt32(hexvalues[i], 16); 
		System::String^ charValue = System::Convert::ToString(value);
		//this->textBox2->AppendText(charValue);
		//this->textBox2->AppendText(" ");
		byteArray[i] = System::Convert::ToByte(value);//(value);

	}
	//this->textBox2->AppendText("\n");
	return byteArray;
}

void RaceControl::MyForm::sendSerialData(array<System::Byte>^ byteArray)
{
	try{
		//if port is open send data
		if (serialPort1->IsOpen)
		{
			serialPort1->Write(byteArray, 0, byteArray->Length);
			//this->SetText(System::Convert::ToString(byteArray->Length));
		}

		//if it's not display message for the user
		else
			this->SetText("\n Connect to COM port first\n");
	}
	catch(TimeoutException ^)
	{
	}
}
void RaceControl::MyForm::setMotorSpeeds()
{
	int left_motor_value = System::Convert::ToInt32(this->textBox3->Text);
	int right_motor_value = System::Convert::ToInt32(this->textBox4->Text);

	array<System::Byte>^ packet = gcnew array<System::Byte>(4);
	//set packet code
	packet[0] = (Byte)PCKTCODE::PCKTCODE_CONTROL_IN;
	//set data length
	packet[1] = 2;
	//add speed
	packet[2] = System::Convert::ToSByte(left_motor_value);
	packet[3] = System::Convert::ToSByte(right_motor_value);
	//packet[2] = System::Convert::ToByte(2);
	this->sendSerialData(packet);
}
void RaceControl::MyForm::setMotorSpeeds(int speed_L, int speed_R)
{
	array<System::Byte>^ packet = gcnew array<System::Byte>(4);
	//set packet code
	if (speed_L > 100)
		speed_L = 100;
	if (speed_L <-100)
		speed_L = -100;
	if (speed_R > 100)
		speed_R = 100;
	if (speed_R < -100)
		speed_R = -100;
	packet[0] = (Byte)PCKTCODE::PCKTCODE_CONTROL_IN;
	//set data length
	packet[1] = 2;
	//add speed
	packet[2] = System::Convert::ToSByte(speed_L);
	packet[3] = System::Convert::ToSByte(speed_R);
	//packet[2] = System::Convert::ToByte(2);
	this->sendSerialData(packet);
}
void RaceControl::MyForm::setMotorGains()
{
	array<System::Byte>^ packet = gcnew array<System::Byte>(5);
	//set packet code
	packet[0] = (Byte)PCKTCODE::PCKTCODE_SETGAIN_IN;
	//set packet length
	packet[1] = 3;
	//insert gains
	packet[2] = System::Convert::ToByte(kp_l);
	packet[3] = System::Convert::ToByte(kd_l);
	packet[4] = System::Convert::ToByte(ki_l);
	//packet[5] = System::Convert::ToSByte(kp_r);
	//packet[6] = System::Convert::ToSByte(ki_r);
	//packet[7] = System::Convert::ToSByte(kd_r);
	this->sendSerialData(packet);

}


void RaceControl::MyForm::SetText(String^ text)
{
	// InvokeRequired required compares the thread ID of the
	// calling thread to the thread ID of the creating thread.
	// If these threads are different, it returns true.
	if (this->textBox2->InvokeRequired)
	{	
		SetTextDelegate^ d = gcnew SetTextDelegate(this, &MyForm::SetText);
		this->Invoke(d, gcnew array<Object^> { text });
	}
	else
	{
		this->textBox2->AppendText(" ");
		this->textBox2->AppendText (text);
	}
}


void RaceControl::MyForm::DrawCVImage(cv::Mat* colorImage)
{
	
	this->image = colorImage;
	this->camera_vision = true;
	this->drawTimer->Enabled = true;
	if (this->vision_form->IsDisposed)
	{
		this->vision_form = gcnew CameraView();
		this->vision_form->setParent(this);
	}
	this->vision_form->Size = System::Drawing::Size(this->image->cols, this->image->rows);
	this->vision_form->Show();

	/*

	if (camera_vision)
	{
		
		if (this->vision_form->IsDisposed)
		{
			std::cout <<"I come here for some reason" <<std::endl;
			this->vision_form = gcnew CameraView();
			this->vision_form->setParent(this);
			
		}
		this->vision_form->Show();
		System::Drawing::Graphics^ graphics = this->vision_form->CreateGraphics();
		System::IntPtr ptr(colorImage.ptr());
		System::Drawing::Bitmap^ b  =gcnew System::Drawing::Bitmap(colorImage.cols,colorImage.rows,colorImage.step,System::Drawing::Imaging::PixelFormat::Format24bppRgb,ptr);
		System::Drawing::RectangleF rect(0,0,this->vision_form->Width,this->vision_form->Height);
		graphics->DrawImage(b,rect);

	}*/
}

System::Void RaceControl::MyForm::button6_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	this->controller_->getCameraTransform(camera);
	this->button7->Enabled = true;
	this->button11->Enabled = true;
	this->button12->Enabled = true;
}

System::Void RaceControl::MyForm::button7_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->getMidPoints(camera);
}

 System::Void RaceControl::MyForm::button8_Click_1(System::Object^  sender, System::EventArgs^  e) {

	 controller_->car_tracking_on = true;
	 controller_->local_planning_on = true;
	 controller_->go_signal_found = true;

}
System::Void RaceControl::MyForm::button3_Click(System::Object^  sender, System::EventArgs^  e) {
					  
	controller_->go_signal_found = false;
	this->setMotorSpeeds(0 , 0);
	//if(this->aTimer->Enabled)
		//this->aTimer->Enabled = false;
}


System::Void RaceControl::MyForm::button9_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 port_num_1, port_num_2, port_num_3, port_num_4, num_cameras;
	System::String^ ip_address;
	port_num_1 = System::Convert::ToInt16(this->textBox7->Text);
	port_num_2 = System::Convert::ToInt16(this->textBox8->Text);
	port_num_3 = System::Convert::ToInt16(this->textBox9->Text);
	port_num_4 = System::Convert::ToInt16(this->textBox10->Text);
	ip_address = this->textBox11->Text;
	num_cameras = System::Convert::ToInt16(this->comboBox3->Text);
	controller_->connectToRoborealm(port_num_1, port_num_2, port_num_3, port_num_4, ip_address, num_cameras);
	this->button6->Enabled = true;
	this->button10->Enabled = true;
	this->button15->Enabled = true;
}
 
System::Void RaceControl::MyForm::button10_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->testColorThresh(camera);
}

System::Void RaceControl::MyForm::button11_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->getObstacles(camera);
}

System::Void RaceControl::MyForm::button12_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->getFinishLine(camera);
	this->button13->Enabled = true;
}

System::Void RaceControl::MyForm::button13_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->getGoSignal(camera);
}

System::Void RaceControl::MyForm::button14_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	controller_->launchOnGo(camera);
}

System::Void RaceControl::MyForm::button15_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Int16 camera = System::Convert::ToInt16(this->comboBox4->Text);
	System::Int16 lower_hue = System::Convert::ToInt16(this->textBox12->Text);	
	System::Int16 lower_lum = System::Convert::ToInt16(this->textBox13->Text);	
	System::Int16 lower_sat = System::Convert::ToInt16(this->textBox14->Text);
	System::Int16 upper_hue = System::Convert::ToInt16(this->textBox15->Text);
	System::Int16 upper_lum = System::Convert::ToInt16(this->textBox16->Text);
	System::Int16 upper_sat = System::Convert::ToInt16(this->textBox17->Text);
	controller_->setColorThresh(camera, lower_hue, lower_lum, lower_sat,
		upper_hue, upper_lum, upper_sat);
}

System::Void RaceControl::MyForm::button16_Click(System::Object^  sender, System::EventArgs^  e) {
	controller_->enterPitstop();
}

System::Void RaceControl::MyForm::button5_Click(System::Object^  sender, System::EventArgs^  e) {
	controller_->exitPitstop();
}
