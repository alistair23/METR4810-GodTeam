#pragma once
#include "PIDTuningForm.h"
namespace RaceControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	//ref class PIDTuningForm;
	
	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//intialize textbox values 
			this->textBox1->Text = "00";
			this->textBox3->Text = "0";
			this->textBox4->Text = "0";
			this->textBox5->Text = "0";
			this->textBox6->Text = "0";
			motor_control = false;
			manual_nav = true;
			continuous_control = false;
			this->textBox2->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox2_KeyDown);
			this->textBox2->KeyUp += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox2_KeyUp);
			this->textBox3->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox3_KeyDown);
			this->textBox4->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox4_KeyDown);
			this->comboBox1->Items->AddRange(this->serialPort1->GetPortNames());
		    array<Object^>^ baudArray = {"2400", "4800", "9600", "14400", "28800"};
			this->comboBox2->Items->AddRange(baudArray);
			this->intializeTrackbars();

			//setup timer for continuous communication
			this->aTimer = gcnew System::Timers::Timer( 100 ); //every 10ms

			// Hook up the Elapsed event for the timer.
			aTimer->Elapsed += gcnew System::Timers::ElapsedEventHandler(this, &MyForm::OnTimedEvent);
			aTimer->Enabled = false;
			this->button2->Enabled = false;

			//initialize gains to zero
			 this->kp_l = 0;
			 this->kp_r = 0;
			 this->ki_l = 0;
			 this->ki_r = 0;
			 this->kd_l = 0;
			 this->kd_r = 0;
			

			//add serial port receiv event
			this->serialPort1->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::dataReceivedHandler);
			//
			//TODO: Add the constructor code here
			//
			/*Bitmap^ bitmap = gcnew Bitmap(pictureBox1->Width, pictureBox1->Height);
			Graphics^ g = Graphics::FromImage(bitmap);
			SolidBrush^ mybrush = gcnew SolidBrush(System::Drawing::Color::Black);
			System::Drawing::Pen^ pen = gcnew System::Drawing::Pen(mybrush);
			g->DrawRectangle(pen,(pictureBox1->Width)/4, (pictureBox1->Height)/4, (pictureBox1->Width)/2, (pictureBox1->Height)/2);
			g->FillEllipse(mybrush,(pictureBox1->Width)*0.45, (pictureBox1->Height)*0.45, (pictureBox1->Width)/10, (pictureBox1->Height)/10);
			pictureBox1->Image = bitmap;*/
			
		}
		MyForm(PIDTuningForm^ form1)
		{
			MyForm();
			this->form = form1;

		}
	private: System::Windows::Forms::RadioButton^  radioButton3;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::CheckBox^  checkBox3;
	private: System::Windows::Forms::CheckBox^  checkBox2;




	public: 

	public: PIDTuningForm^ form;
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: enum class PCKTCODE
			 {
				 PCKTCODE_CONTROL_IN = 1,
					PCKTCODE_SETGAIN_IN,
					PCKTCODE_DEBUG
			 };
	private: System::Windows::Forms::Button^  button1;
	protected: 
	private: System::Windows::Forms::Button^  button2;
	private: System::IO::Ports::SerialPort^  serialPort1;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::ComboBox^  comboBox1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::ComboBox^  comboBox2;
	private: System::Windows::Forms::Label^  label4;

	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::TrackBar^  trackBar2;
	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::TextBox^  textBox6;
	private: System::Windows::Forms::TextBox^  textBox5;
	private: System::Windows::Forms::CheckBox^  checkBox1;

	private: System::ComponentModel::IContainer^  components;

	delegate void SetTextDelegate(String^ text);
	private: bool continuous_control;
			 bool motor_control;
			 bool manual_nav;
			 bool moving_right;
			 bool moving_left;
			 bool moving_forward;
			 bool moving_reverse;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::RadioButton^  radioButton2;
	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::Button^  button4;
	private: System::Timers::Timer^ aTimer;
	public:  UInt16 kp_l;
			 UInt16 kp_r;
			 UInt16 ki_l;
			 UInt16 ki_r;
			 UInt16 kd_l;
			 UInt16 kd_r;


			 
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->comboBox2 = (gcnew System::Windows::Forms::ComboBox());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->radioButton3 = (gcnew System::Windows::Forms::RadioButton());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->radioButton2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->textBox6 = (gcnew System::Windows::Forms::TextBox());
			this->textBox5 = (gcnew System::Windows::Forms::TextBox());
			this->textBox4 = (gcnew System::Windows::Forms::TextBox());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->trackBar2 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->checkBox2 = (gcnew System::Windows::Forms::CheckBox());
			this->checkBox3 = (gcnew System::Windows::Forms::CheckBox());
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->BeginInit();
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(171, 390);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(98, 23);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Connect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(275, 390);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(92, 23);
			this->button2->TabIndex = 1;
			this->button2->Text = L"Send";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(168, 145);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(348, 20);
			this->textBox1->TabIndex = 2;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(168, 129);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(118, 13);
			this->label1->TabIndex = 3;
			this->label1->Text = L"Hexadecimal Command";
			// 
			// comboBox1
			// 
			this->comboBox1->AllowDrop = true;
			this->comboBox1->FormattingEnabled = true;
			this->comboBox1->Location = System::Drawing::Point(395, 31);
			this->comboBox1->Name = L"comboBox1";
			this->comboBox1->Size = System::Drawing::Size(121, 21);
			this->comboBox1->TabIndex = 4;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(392, 15);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(86, 13);
			this->label2->TabIndex = 5;
			this->label2->Text = L"Select COM Port";
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(168, 184);
			this->textBox2->Multiline = true;
			this->textBox2->Name = L"textBox2";
			this->textBox2->ScrollBars = System::Windows::Forms::ScrollBars::Vertical;
			this->textBox2->Size = System::Drawing::Size(348, 149);
			this->textBox2->TabIndex = 6;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(165, 168);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(47, 13);
			this->label3->TabIndex = 7;
			this->label3->Text = L"Terminal";
			// 
			// comboBox2
			// 
			this->comboBox2->AllowDrop = true;
			this->comboBox2->FormattingEnabled = true;
			this->comboBox2->Location = System::Drawing::Point(395, 83);
			this->comboBox2->Name = L"comboBox2";
			this->comboBox2->Size = System::Drawing::Size(121, 21);
			this->comboBox2->TabIndex = 9;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(392, 67);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(58, 13);
			this->label4->TabIndex = 10;
			this->label4->Text = L"Buad Rate";
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->checkBox3);
			this->groupBox1->Controls->Add(this->checkBox2);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->radioButton3);
			this->groupBox1->Controls->Add(this->button4);
			this->groupBox1->Controls->Add(this->checkBox1);
			this->groupBox1->Controls->Add(this->radioButton2);
			this->groupBox1->Controls->Add(this->radioButton1);
			this->groupBox1->Controls->Add(this->button3);
			this->groupBox1->Controls->Add(this->label8);
			this->groupBox1->Controls->Add(this->label7);
			this->groupBox1->Controls->Add(this->textBox6);
			this->groupBox1->Controls->Add(this->textBox5);
			this->groupBox1->Controls->Add(this->textBox4);
			this->groupBox1->Controls->Add(this->textBox3);
			this->groupBox1->Controls->Add(this->label6);
			this->groupBox1->Controls->Add(this->button2);
			this->groupBox1->Controls->Add(this->label5);
			this->groupBox1->Controls->Add(this->button1);
			this->groupBox1->Controls->Add(this->trackBar2);
			this->groupBox1->Controls->Add(this->trackBar1);
			this->groupBox1->Controls->Add(this->comboBox2);
			this->groupBox1->Controls->Add(this->label2);
			this->groupBox1->Controls->Add(this->label4);
			this->groupBox1->Controls->Add(this->label3);
			this->groupBox1->Controls->Add(this->textBox2);
			this->groupBox1->Controls->Add(this->comboBox1);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->textBox1);
			this->groupBox1->Location = System::Drawing::Point(12, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(560, 438);
			this->groupBox1->TabIndex = 12;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Serial Command";
			this->groupBox1->Enter += gcnew System::EventHandler(this, &MyForm::groupBox1_Enter);
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(171, 343);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(80, 13);
			this->label9->TabIndex = 30;
			this->label9->Text = L"Receive Mode:";
			// 
			// radioButton3
			// 
			this->radioButton3->AutoSize = true;
			this->radioButton3->Location = System::Drawing::Point(275, 342);
			this->radioButton3->Name = L"radioButton3";
			this->radioButton3->Size = System::Drawing::Size(86, 17);
			this->radioButton3->TabIndex = 29;
			this->radioButton3->TabStop = true;
			this->radioButton3->Text = L"Motor Speed";
			this->radioButton3->UseVisualStyleBackColor = true;
			this->radioButton3->CheckedChanged += gcnew System::EventHandler(this, &MyForm::radioButton3_CheckedChanged);
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(16, 343);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(114, 25);
			this->button4->TabIndex = 28;
			this->button4->Text = L"Tune PID";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &MyForm::button4_Click);
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Location = System::Drawing::Point(20, 374);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(125, 17);
			this->checkBox1->TabIndex = 23;
			this->checkBox1->Text = L"Enable Motor Control";
			this->checkBox1->UseVisualStyleBackColor = true;
			this->checkBox1->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkBox1_CheckedChanged);
			// 
			// radioButton2
			// 
			this->radioButton2->AutoSize = true;
			this->radioButton2->Location = System::Drawing::Point(375, 342);
			this->radioButton2->Name = L"radioButton2";
			this->radioButton2->Size = System::Drawing::Size(52, 17);
			this->radioButton2->TabIndex = 27;
			this->radioButton2->TabStop = true;
			this->radioButton2->Text = L"String";
			this->radioButton2->UseVisualStyleBackColor = true;
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Checked = true;
			this->radioButton1->Location = System::Drawing::Point(446, 342);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(70, 17);
			this->radioButton1->TabIndex = 26;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"Hex View";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(375, 390);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(103, 23);
			this->button3->TabIndex = 25;
			this->button3->Text = L"Stop Command";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &MyForm::button3_Click);
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(166, 64);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(88, 13);
			this->label8->TabIndex = 20;
			this->label8->Text = L"right motor speed";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(168, 14);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(82, 13);
			this->label7->TabIndex = 19;
			this->label7->Text = L"left motor speed";
			// 
			// textBox6
			// 
			this->textBox6->Location = System::Drawing::Point(168, 83);
			this->textBox6->Name = L"textBox6";
			this->textBox6->Size = System::Drawing::Size(100, 20);
			this->textBox6->TabIndex = 18;
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(168, 31);
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(100, 20);
			this->textBox5->TabIndex = 17;
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(92, 313);
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(67, 20);
			this->textBox4->TabIndex = 16;
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(6, 313);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(67, 20);
			this->textBox3->TabIndex = 15;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(103, 286);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(56, 13);
			this->label6->TabIndex = 14;
			this->label6->Text = L"right motor";
			this->label6->Click += gcnew System::EventHandler(this, &MyForm::label6_Click);
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(15, 286);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(50, 13);
			this->label5->TabIndex = 13;
			this->label5->Text = L"left motor";
			// 
			// trackBar2
			// 
			this->trackBar2->Location = System::Drawing::Point(114, 18);
			this->trackBar2->Maximum = 100;
			this->trackBar2->Name = L"trackBar2";
			this->trackBar2->Orientation = System::Windows::Forms::Orientation::Vertical;
			this->trackBar2->Size = System::Drawing::Size(45, 265);
			this->trackBar2->TabIndex = 12;
			// 
			// trackBar1
			// 
			this->trackBar1->Location = System::Drawing::Point(20, 20);
			this->trackBar1->Maximum = 100;
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Orientation = System::Windows::Forms::Orientation::Vertical;
			this->trackBar1->Size = System::Drawing::Size(45, 268);
			this->trackBar1->TabIndex = 11;
			// 
			// checkBox2
			// 
			this->checkBox2->AutoSize = true;
			this->checkBox2->Location = System::Drawing::Point(20, 392);
			this->checkBox2->Name = L"checkBox2";
			this->checkBox2->Size = System::Drawing::Size(115, 17);
			this->checkBox2->TabIndex = 31;
			this->checkBox2->Text = L"Manual Navigation";
			this->checkBox2->UseVisualStyleBackColor = true;
			// 
			// checkBox3
			// 
			this->checkBox3->AutoSize = true;
			this->checkBox3->Location = System::Drawing::Point(20, 415);
			this->checkBox3->Name = L"checkBox3";
			this->checkBox3->Size = System::Drawing::Size(115, 17);
			this->checkBox3->TabIndex = 32;
			this->checkBox3->Text = L"Continuous Control";
			this->checkBox3->UseVisualStyleBackColor = true;
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(584, 462);
			this->Controls->Add(this->groupBox1);
			this->Name = L"MyForm";
			this->Text = L"Serial Command";
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion

private: bool is_decimal(System::String^ s)
{
	System::Decimal n;
	bool m = System::Decimal::TryParse(s, n);
    return m;
}

private : void intializeTrackbars()
		  {
			  this->trackBar1->Minimum = -100;
			  this->trackBar2->Minimum = -100;
			  this->trackBar1->Value = 0;
			  this->trackBar2->Value = 0;
			  this->trackBar1->ValueChanged+=gcnew System::EventHandler(this, &MyForm::LeftTrackBarChanged);
			  this->trackBar2->ValueChanged+=gcnew System::EventHandler(this, &MyForm::RightTrackBarChanged);
		  }
private: array<System::Byte>^ interpretHexadecimalCommand()
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

private: void sendSerialData(array<System::Byte>^ byteArray)
		 {
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
private: void setMotorSpeeds()
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
private: void setMotorSpeeds(int speed_L, int speed_R)
		 {
			array<System::Byte>^ packet = gcnew array<System::Byte>(4);
			 //set packet code
			 packet[0] = (Byte)PCKTCODE::PCKTCODE_CONTROL_IN;
			 //set data length
			 packet[1] = 2;
			 //add speed
			 packet[2] = System::Convert::ToSByte(speed_L);
			 packet[3] = System::Convert::ToSByte(speed_R);
			 //packet[2] = System::Convert::ToByte(2);
			 this->sendSerialData(packet);
		 }
public: void setMotorGains()
		{
			array<System::Byte>^ packet = gcnew array<System::Byte>(5);
			//set packet code
			packet[0] = (Byte)PCKTCODE::PCKTCODE_SETGAIN_IN;
			//set packet length
			packet[1] = 6;
			//insert gains
			packet[2] = System::Convert::ToSByte(kp_l);
			packet[3] = System::Convert::ToSByte(ki_l);
			packet[4] = System::Convert::ToSByte(kd_l);
			//packet[5] = System::Convert::ToSByte(kp_r);
			//packet[6] = System::Convert::ToSByte(ki_r);
			//packet[7] = System::Convert::ToSByte(kd_r);
			this->sendSerialData(packet);

		}


public: void SetText(String^ text)
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

//**************************Event Handlers*****************************************
private : void LeftTrackBarChanged(System::Object^  sender, System::EventArgs^  e)
		  {
			  this->textBox3->Text = System::Convert::ToString(this->trackBar1->Value);
		  }
private : void RightTrackBarChanged(System::Object^ sender, System::EventArgs^ e)
		  {
			  this->textBox4->Text = System::Convert::ToString(this->trackBar2->Value);
		  }
private: System::Void groupBox1_Enter(System::Object^  sender, System::EventArgs^  e) {
			 }


private: System::Void textBox2_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^ e){


			 if (e->KeyCode == Keys::Up)
				 moving_forward = true;
			 if (e->KeyCode == Keys::Down)
				 moving_reverse = true;
			 if (e->KeyCode == Keys::Left)
				 moving_left = true;
			 if (e->KeyCode == Keys::Right)
				 moving_right = true;

			 if (this->manual_nav && this->motor_control)
			 {
				 if (moving_forward && !moving_left && !moving_right && !moving_reverse)
				 {
					 setMotorSpeeds(50, 50);
					 //this->SetText("Forward\n");
				 }

				 if(moving_reverse && !moving_left && !moving_right && !moving_forward)
				 {
					 setMotorSpeeds(-50, -50);
					 //this->SetText("Reverse\n");
				 }

				 if(moving_right && !moving_left && !moving_reverse  && !moving_forward)
				 {
					 setMotorSpeeds(50, -50);
					 //this->SetText("Right\n");
				 }
				  if(moving_left && !moving_right && !moving_reverse  && !moving_forward)
				 {
					 setMotorSpeeds(-50, 50);
					 //this->SetText("Left\n");
				 }

				  if (moving_forward && moving_right && !moving_reverse  && !moving_left)
				  {
					  setMotorSpeeds(50, 25);
					  //this->SetText("Align forward Right\n");
				  }

				  if (moving_forward && moving_left && !moving_reverse  && !moving_right)
				  {
					  setMotorSpeeds(25, 50);
					  //this->SetText("Align forward Left\n");
				  }
				    if (moving_reverse && moving_right && !moving_forward  && !moving_left)
				  {
					  setMotorSpeeds(-50, -25);
					  //this->SetText("Align reverse right\n");
				  }

					  if (moving_reverse && moving_left && !moving_forward  && !moving_right)
				  {
					  setMotorSpeeds(-25, -50);
					  //this->SetText("Align reverse left\n");
				  }

	

			 }
			 else
				 return;
		 }

private: System::Void textBox2_KeyUp(System::Object^  sender, System::Windows::Forms::KeyEventArgs^ e){
			 
			 if(this->manual_nav && this->motor_control)
			 {
				 if (e->KeyCode == Keys::Up)
					 moving_forward = false;
				 if (e->KeyCode == Keys::Down)
					 moving_reverse = false;
				 if (e->KeyCode == Keys::Left)
					 moving_left = false;
				 if (e->KeyCode == Keys::Right)
					 moving_right = false;
				 if(!moving_reverse && !moving_left && !moving_forward  && !moving_right)
				{
					setMotorSpeeds(0, 0);
					//this->SetText("Stopped\n");
				}
			 }
		 }

private: System::Void textBox3_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^ e) {
			 
			 if(e->KeyCode == Keys::Return)
			 {
				 if(this->textBox3->Text == "")
					 return;
				 System::Int16 n = System::Convert::ToInt16(this->textBox3->Text);
				 if(is_decimal(this->textBox3->Text) && this->trackBar1->Maximum >= n)
					this->trackBar1->Value = n;
				 else
					 this->textBox2->AppendText("entry must be a number in valid range!\n");
				e->SuppressKeyPress = true;
			 }
			 e->Handled = true;
			 
			 
		 }
private: System::Void textBox4_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^ e) {
			 
			 if(e->KeyCode == Keys::Return)
			 {
				 if(this->textBox4->Text == "")
					 return;
				 System::Int16 n = System::Convert::ToInt16(this->textBox4->Text);
				 if(is_decimal(this->textBox4->Text) && this->trackBar2->Maximum >= n)
					this->trackBar2->Value = n;
				 else
					 this->textBox2->AppendText("entry must be a number in valid range!\n");
				e->SuppressKeyPress = true; 
			 }
			  e->Handled = true;
			  
		 }
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {


			 //if port is open close it
			 if (this->serialPort1->IsOpen)
				 {
					 this->aTimer->Enabled = false;
					 this->serialPort1->Close();
					 this->button1->Text = "Connect";
					 this->button2->Enabled = false;
					 
				 }
			 //otherwise open the port that is specified
			 else
			 {
				 if(this->comboBox1->SelectedItem == nullptr ||\
					 this->comboBox2->SelectedItem == nullptr)
				 {
					 if(this->comboBox1->SelectedItem == nullptr)
						this->textBox2->AppendText("Select a COM port!\n");
					 if(this->comboBox2->SelectedItem == nullptr)
						 this->textBox2->AppendText("Select baud rate!\n");
				 
					 return;
				 }
				 else
				 {
					 this->serialPort1->PortName = System::Convert::ToString(comboBox1->SelectedItem);
					 this->serialPort1->BaudRate = System::Convert::ToInt16(comboBox2->SelectedItem);
			 
					 //set the number of milliseconds before time out occurs if read/write op 
					 //doesn't finish
					 this->serialPort1->ReadTimeout = 500;
					 this->serialPort1->WriteTimeout = 500;

					 //connect to the port

				     this->serialPort1->Open();
					 this->button1->Text = "Disconnect";
					 this->button2->Enabled = true;
					 
				 }
			 }
		 }

private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
			 
			 if(this->checkBox3->Checked)
			{	 
				continuous_control = true;
				this->aTimer->Enabled = true;
			 }
			 else
			 {
				 continuous_control = false;
				 this->aTimer->Enabled = false;
			 }

			 if (this->checkBox2->Checked)
			 {
				 this->manual_nav = true;
			 }
			 else
			 {
				 this->manual_nav = false;
			 }

			 if(this->checkBox1->Checked)
				 motor_control = true;
			 else
			 { //if not motor control mode, send the hex command
				 motor_control = false;
				 this->sendSerialData(this->interpretHexadecimalCommand());
			 }
			 if(motor_control == true && continuous_control == false)
				 this->setMotorSpeeds();
		 }

private: System::Void checkBox1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 
		 }
private: System::Void checkBox2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 
		 }
private: System::Void OnTimedEvent( System::Object^ source, System::Timers::ElapsedEventArgs^ e ){
			 
			 if(motor_control)
				 this->setMotorSpeeds();
		 }
private: System::Void dataReceivedHandler(System::Object^ sender,\
			 System::IO::Ports::SerialDataReceivedEventArgs^ e)
		 {
			 if (this->radioButton1->Checked)
			 {
				int data = this->serialPort1->ReadByte();
				this->SetText(System::Convert::ToString(data));
			 }
			 else if (this->radioButton2->Checked)

			 {
				 //this->SetText("in string mode\n");
				 System::String^ data_char = this->serialPort1->ReadLine();
				 this->SetText(System::Convert::ToString(data_char));
				 this->SetText("\n");
				 
			 }
			 else if (this->radioButton3->Checked)
			 {
				 /*
				 int motor_speed_left = ;
				 int motor_speed_right = ;
				 this->textBox5->Text = System::Convert::ToString(motor_speed_left);
				 this->textBox6->Text = System::Convert::ToString(motor_speed_right);
				 */
			 }
		 }
		 private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
					  if(this->aTimer->Enabled)
						  this->aTimer->Enabled = false;
				  }
private: System::Void label6_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e);// {
			 //PIDTuningForm^ form2 = gcnew PIDTuningForm(this);
			// form2->Show();
			 
		// }
private: System::Void radioButton3_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
};
}
