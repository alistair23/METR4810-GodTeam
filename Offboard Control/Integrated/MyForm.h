#pragma once
#include "PIDTuningForm.h"
#include "Controller.h"
#include "CameraView.h"
#include "Vision.h"
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
			this->comboBox2->SelectedIndex = 2;
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

			this->vision_form = gcnew CameraView();
			this->vision_form->setParent(this);
			
			
			this->drawTimer = gcnew System::Timers::Timer( 1000 );
			drawTimer->Elapsed += gcnew System::Timers::ElapsedEventHandler(this, &MyForm::updateImage);
			drawTimer->Enabled = false;

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
	private: System::Windows::Forms::TabControl^  tabControl1;
	private: System::Windows::Forms::TabPage^  tabPage1;

	public: 

	private: System::Windows::Forms::Button^  button8;



	private: System::Windows::Forms::CheckBox^  checkBox3;
	private: System::Windows::Forms::CheckBox^  checkBox2;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::RadioButton^  radioButton3;
	private: System::Windows::Forms::Button^  button4;
	private: System::Windows::Forms::CheckBox^  checkBox1;
	private: System::Windows::Forms::RadioButton^  radioButton2;
	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::TextBox^  textBox6;
	private: System::Windows::Forms::TextBox^  textBox5;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::TrackBar^  trackBar2;
	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::ComboBox^  comboBox2;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::ComboBox^  comboBox1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::TextBox^  textBox11;
private: System::Windows::Forms::Button^  btn_connect;



	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::TextBox^  textBox10;
	private: System::Windows::Forms::TextBox^  textBox9;
	private: System::Windows::Forms::TextBox^  textBox8;
	private: System::Windows::Forms::TextBox^  textBox7;
private: System::Windows::Forms::Label^  label15;
private: System::Windows::Forms::ComboBox^  comboBox3;
private: System::Windows::Forms::Label^  label16;
private: System::Windows::Forms::ComboBox^  comboBox4;
private: System::Windows::Forms::Label^  label17;
private: System::Windows::Forms::Button^  btn_get_midpoints;

private: System::Windows::Forms::Button^  btn_get_transform;
private: System::Windows::Forms::Button^  btn_test_color_thresh;


		 /*MyForm(PIDTuningForm^ form1)
		{
			MyForm();
			this->form = form1;

		} */             

private: System::Windows::Forms::Button^  btn_get_go_signal;

private: System::Windows::Forms::Button^  btn_get_finish_line;

private: System::Windows::Forms::Button^  btn_get_obstacles;

private: System::Windows::Forms::Button^  button14;
private: System::Windows::Forms::TextBox^  textBox17;
private: System::Windows::Forms::TextBox^  textBox16;
private: System::Windows::Forms::TextBox^  textBox15;
private: System::Windows::Forms::TextBox^  textBox14;
private: System::Windows::Forms::TextBox^  textBox13;
private: System::Windows::Forms::Label^  label22;
private: System::Windows::Forms::Label^  label21;
private: System::Windows::Forms::Label^  label20;
private: System::Windows::Forms::TextBox^  textBox12;
private: System::Windows::Forms::Label^  label19;
private: System::Windows::Forms::Label^  label18;
private: System::Windows::Forms::Button^  btn_set_color_thresh;

private: System::Windows::Forms::Button^  button16;
private: System::Windows::Forms::Button^  button5;
private: System::Windows::Forms::ImageList^  imageList1;
private: System::Windows::Forms::Button^  btn_preview_image;

private: System::Windows::Forms::CheckBox^  chkbox_manual_mode;


private: System::Windows::Forms::Button^  btn_load;

private: System::Windows::Forms::Button^  btn_save;
private: System::Windows::Forms::Button^  btn_pitstop_entry;
private: System::Windows::Forms::CheckBox^  chkbox_pace;





















	public: 

	public: 
			 bool camera_vision;
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

	protected: 

	private: System::IO::Ports::SerialPort^  serialPort1;


	private: System::ComponentModel::IContainer^  components;

	delegate void SetTextDelegate(String^ text);
	private: bool continuous_control;
			 bool motor_control;
			 bool manual_nav;
			 bool moving_right;
			 bool moving_left;
			 bool moving_forward;
			 bool moving_reverse;
			




	private: System::Timers::Timer^ aTimer;
			 System::Timers::Timer^ drawTimer;
	public:  UInt16 kp_l;
			 UInt16 kp_r;
			 UInt16 ki_l;
			 UInt16 ki_r;
			 UInt16 kd_l;
			 UInt16 kd_r;

	private: ref class CameraView^ vision_form;
			 ref class PIDTuningForm^ PID_form;

			 
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
			this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
			this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->button16 = (gcnew System::Windows::Forms::Button());
			this->button14 = (gcnew System::Windows::Forms::Button());
			this->button8 = (gcnew System::Windows::Forms::Button());
			this->checkBox3 = (gcnew System::Windows::Forms::CheckBox());
			this->checkBox2 = (gcnew System::Windows::Forms::CheckBox());
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
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->trackBar2 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->comboBox2 = (gcnew System::Windows::Forms::ComboBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
			this->btn_pitstop_entry = (gcnew System::Windows::Forms::Button());
			this->btn_load = (gcnew System::Windows::Forms::Button());
			this->btn_save = (gcnew System::Windows::Forms::Button());
			this->chkbox_manual_mode = (gcnew System::Windows::Forms::CheckBox());
			this->btn_preview_image = (gcnew System::Windows::Forms::Button());
			this->textBox17 = (gcnew System::Windows::Forms::TextBox());
			this->textBox16 = (gcnew System::Windows::Forms::TextBox());
			this->textBox15 = (gcnew System::Windows::Forms::TextBox());
			this->textBox14 = (gcnew System::Windows::Forms::TextBox());
			this->textBox13 = (gcnew System::Windows::Forms::TextBox());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->textBox12 = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->btn_set_color_thresh = (gcnew System::Windows::Forms::Button());
			this->btn_get_go_signal = (gcnew System::Windows::Forms::Button());
			this->btn_get_finish_line = (gcnew System::Windows::Forms::Button());
			this->btn_get_obstacles = (gcnew System::Windows::Forms::Button());
			this->btn_test_color_thresh = (gcnew System::Windows::Forms::Button());
			this->comboBox4 = (gcnew System::Windows::Forms::ComboBox());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->btn_get_midpoints = (gcnew System::Windows::Forms::Button());
			this->btn_get_transform = (gcnew System::Windows::Forms::Button());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->comboBox3 = (gcnew System::Windows::Forms::ComboBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->textBox11 = (gcnew System::Windows::Forms::TextBox());
			this->btn_connect = (gcnew System::Windows::Forms::Button());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->textBox10 = (gcnew System::Windows::Forms::TextBox());
			this->textBox9 = (gcnew System::Windows::Forms::TextBox());
			this->textBox8 = (gcnew System::Windows::Forms::TextBox());
			this->textBox7 = (gcnew System::Windows::Forms::TextBox());
			this->imageList1 = (gcnew System::Windows::Forms::ImageList(this->components));
			this->chkbox_pace = (gcnew System::Windows::Forms::CheckBox());
			this->tabControl1->SuspendLayout();
			this->tabPage1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->BeginInit();
			this->tabPage2->SuspendLayout();
			this->SuspendLayout();
			// 
			// tabControl1
			// 
			this->tabControl1->Controls->Add(this->tabPage1);
			this->tabControl1->Controls->Add(this->tabPage2);
			this->tabControl1->Location = System::Drawing::Point(12, 6);
			this->tabControl1->Name = L"tabControl1";
			this->tabControl1->SelectedIndex = 0;
			this->tabControl1->Size = System::Drawing::Size(558, 529);
			this->tabControl1->TabIndex = 13;
			// 
			// tabPage1
			// 
			this->tabPage1->Controls->Add(this->chkbox_pace);
			this->tabPage1->Controls->Add(this->button5);
			this->tabPage1->Controls->Add(this->button16);
			this->tabPage1->Controls->Add(this->button14);
			this->tabPage1->Controls->Add(this->button8);
			this->tabPage1->Controls->Add(this->checkBox3);
			this->tabPage1->Controls->Add(this->checkBox2);
			this->tabPage1->Controls->Add(this->label9);
			this->tabPage1->Controls->Add(this->radioButton3);
			this->tabPage1->Controls->Add(this->button4);
			this->tabPage1->Controls->Add(this->checkBox1);
			this->tabPage1->Controls->Add(this->radioButton2);
			this->tabPage1->Controls->Add(this->radioButton1);
			this->tabPage1->Controls->Add(this->button3);
			this->tabPage1->Controls->Add(this->label8);
			this->tabPage1->Controls->Add(this->label7);
			this->tabPage1->Controls->Add(this->textBox6);
			this->tabPage1->Controls->Add(this->textBox5);
			this->tabPage1->Controls->Add(this->textBox4);
			this->tabPage1->Controls->Add(this->textBox3);
			this->tabPage1->Controls->Add(this->label6);
			this->tabPage1->Controls->Add(this->button2);
			this->tabPage1->Controls->Add(this->label5);
			this->tabPage1->Controls->Add(this->button1);
			this->tabPage1->Controls->Add(this->trackBar2);
			this->tabPage1->Controls->Add(this->trackBar1);
			this->tabPage1->Controls->Add(this->comboBox2);
			this->tabPage1->Controls->Add(this->label2);
			this->tabPage1->Controls->Add(this->label4);
			this->tabPage1->Controls->Add(this->label3);
			this->tabPage1->Controls->Add(this->textBox2);
			this->tabPage1->Controls->Add(this->comboBox1);
			this->tabPage1->Controls->Add(this->label1);
			this->tabPage1->Controls->Add(this->textBox1);
			this->tabPage1->Location = System::Drawing::Point(4, 22);
			this->tabPage1->Name = L"tabPage1";
			this->tabPage1->Padding = System::Windows::Forms::Padding(3);
			this->tabPage1->Size = System::Drawing::Size(550, 503);
			this->tabPage1->TabIndex = 0;
			this->tabPage1->Text = L"Main";
			this->tabPage1->UseVisualStyleBackColor = true;
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(289, 439);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(92, 23);
			this->button5->TabIndex = 72;
			this->button5->Text = L"Exit pitstop";
			this->button5->UseVisualStyleBackColor = true;
			this->button5->Click += gcnew System::EventHandler(this, &MyForm::button5_Click);
			// 
			// button16
			// 
			this->button16->Location = System::Drawing::Point(188, 439);
			this->button16->Name = L"button16";
			this->button16->Size = System::Drawing::Size(97, 23);
			this->button16->TabIndex = 71;
			this->button16->Text = L"Enter pitstop";
			this->button16->UseVisualStyleBackColor = true;
			this->button16->Click += gcnew System::EventHandler(this, &MyForm::button16_Click);
			// 
			// button14
			// 
			this->button14->Location = System::Drawing::Point(289, 401);
			this->button14->Name = L"button14";
			this->button14->Size = System::Drawing::Size(134, 23);
			this->button14->TabIndex = 70;
			this->button14->Text = L"Launch on signal";
			this->button14->UseVisualStyleBackColor = true;
			this->button14->Click += gcnew System::EventHandler(this, &MyForm::button14_Click);
			// 
			// button8
			// 
			this->button8->Location = System::Drawing::Point(188, 401);
			this->button8->Name = L"button8";
			this->button8->Size = System::Drawing::Size(97, 23);
			this->button8->TabIndex = 69;
			this->button8->Text = L"Launch ";
			this->button8->UseVisualStyleBackColor = true;
			this->button8->Click += gcnew System::EventHandler(this, &MyForm::button8_Click_1);
			// 
			// checkBox3
			// 
			this->checkBox3->AutoSize = true;
			this->checkBox3->Location = System::Drawing::Point(34, 401);
			this->checkBox3->Name = L"checkBox3";
			this->checkBox3->Size = System::Drawing::Size(115, 17);
			this->checkBox3->TabIndex = 65;
			this->checkBox3->Text = L"Continuous Control";
			this->checkBox3->UseVisualStyleBackColor = true;
			// 
			// checkBox2
			// 
			this->checkBox2->AutoSize = true;
			this->checkBox2->Location = System::Drawing::Point(34, 378);
			this->checkBox2->Name = L"checkBox2";
			this->checkBox2->Size = System::Drawing::Size(115, 17);
			this->checkBox2->TabIndex = 64;
			this->checkBox2->Text = L"Manual Navigation";
			this->checkBox2->UseVisualStyleBackColor = true;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(185, 329);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(80, 13);
			this->label9->TabIndex = 63;
			this->label9->Text = L"Receive Mode:";
			// 
			// radioButton3
			// 
			this->radioButton3->AutoSize = true;
			this->radioButton3->Location = System::Drawing::Point(289, 328);
			this->radioButton3->Name = L"radioButton3";
			this->radioButton3->Size = System::Drawing::Size(86, 17);
			this->radioButton3->TabIndex = 62;
			this->radioButton3->TabStop = true;
			this->radioButton3->Text = L"Motor Speed";
			this->radioButton3->UseVisualStyleBackColor = true;
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(30, 329);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(114, 25);
			this->button4->TabIndex = 61;
			this->button4->Text = L"Tune PID";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &MyForm::button4_Click);
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Location = System::Drawing::Point(34, 360);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(125, 17);
			this->checkBox1->TabIndex = 57;
			this->checkBox1->Text = L"Enable Motor Control";
			this->checkBox1->UseVisualStyleBackColor = true;
			// 
			// radioButton2
			// 
			this->radioButton2->AutoSize = true;
			this->radioButton2->Location = System::Drawing::Point(389, 328);
			this->radioButton2->Name = L"radioButton2";
			this->radioButton2->Size = System::Drawing::Size(52, 17);
			this->radioButton2->TabIndex = 60;
			this->radioButton2->TabStop = true;
			this->radioButton2->Text = L"String";
			this->radioButton2->UseVisualStyleBackColor = true;
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Checked = true;
			this->radioButton1->Location = System::Drawing::Point(460, 328);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(70, 17);
			this->radioButton1->TabIndex = 59;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"Hex View";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(387, 360);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(103, 23);
			this->button3->TabIndex = 58;
			this->button3->Text = L"Stop command";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &MyForm::button3_Click);
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(180, 50);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(93, 13);
			this->label8->TabIndex = 56;
			this->label8->Text = L"Right motor speed";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(179, 0);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(86, 13);
			this->label7->TabIndex = 55;
			this->label7->Text = L"Left motor speed";
			// 
			// textBox6
			// 
			this->textBox6->Location = System::Drawing::Point(182, 69);
			this->textBox6->Name = L"textBox6";
			this->textBox6->Size = System::Drawing::Size(100, 20);
			this->textBox6->TabIndex = 54;
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(182, 17);
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(100, 20);
			this->textBox5->TabIndex = 53;
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(106, 299);
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(67, 20);
			this->textBox4->TabIndex = 52;
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(20, 299);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(67, 20);
			this->textBox3->TabIndex = 51;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(117, 272);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(56, 13);
			this->label6->TabIndex = 50;
			this->label6->Text = L"right motor";
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(289, 360);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(92, 23);
			this->button2->TabIndex = 38;
			this->button2->Text = L"Send";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click);
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(29, 272);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(50, 13);
			this->label5->TabIndex = 49;
			this->label5->Text = L"left motor";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(188, 360);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(98, 23);
			this->button1->TabIndex = 37;
			this->button1->Text = L"Connect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// trackBar2
			// 
			this->trackBar2->Location = System::Drawing::Point(128, 4);
			this->trackBar2->Maximum = 100;
			this->trackBar2->Name = L"trackBar2";
			this->trackBar2->Orientation = System::Windows::Forms::Orientation::Vertical;
			this->trackBar2->Size = System::Drawing::Size(45, 265);
			this->trackBar2->TabIndex = 48;
			// 
			// trackBar1
			// 
			this->trackBar1->Location = System::Drawing::Point(34, 6);
			this->trackBar1->Maximum = 100;
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Orientation = System::Windows::Forms::Orientation::Vertical;
			this->trackBar1->Size = System::Drawing::Size(45, 268);
			this->trackBar1->TabIndex = 47;
			// 
			// comboBox2
			// 
			this->comboBox2->AllowDrop = true;
			this->comboBox2->FormattingEnabled = true;
			this->comboBox2->Location = System::Drawing::Point(409, 69);
			this->comboBox2->Name = L"comboBox2";
			this->comboBox2->Size = System::Drawing::Size(121, 21);
			this->comboBox2->TabIndex = 45;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(406, 0);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(85, 13);
			this->label2->TabIndex = 42;
			this->label2->Text = L"Select COM port";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(406, 53);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(53, 13);
			this->label4->TabIndex = 46;
			this->label4->Text = L"Baud rate";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(179, 154);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(47, 13);
			this->label3->TabIndex = 44;
			this->label3->Text = L"Terminal";
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(182, 170);
			this->textBox2->Multiline = true;
			this->textBox2->Name = L"textBox2";
			this->textBox2->ScrollBars = System::Windows::Forms::ScrollBars::Vertical;
			this->textBox2->Size = System::Drawing::Size(348, 149);
			this->textBox2->TabIndex = 43;
			// 
			// comboBox1
			// 
			this->comboBox1->AllowDrop = true;
			this->comboBox1->FormattingEnabled = true;
			this->comboBox1->Location = System::Drawing::Point(409, 17);
			this->comboBox1->Name = L"comboBox1";
			this->comboBox1->Size = System::Drawing::Size(121, 21);
			this->comboBox1->TabIndex = 41;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(182, 106);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(117, 13);
			this->label1->TabIndex = 40;
			this->label1->Text = L"Hexadecimal command";
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(182, 131);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(348, 20);
			this->textBox1->TabIndex = 39;
			// 
			// tabPage2
			// 
			this->tabPage2->Controls->Add(this->btn_pitstop_entry);
			this->tabPage2->Controls->Add(this->btn_load);
			this->tabPage2->Controls->Add(this->btn_save);
			this->tabPage2->Controls->Add(this->chkbox_manual_mode);
			this->tabPage2->Controls->Add(this->btn_preview_image);
			this->tabPage2->Controls->Add(this->textBox17);
			this->tabPage2->Controls->Add(this->textBox16);
			this->tabPage2->Controls->Add(this->textBox15);
			this->tabPage2->Controls->Add(this->textBox14);
			this->tabPage2->Controls->Add(this->textBox13);
			this->tabPage2->Controls->Add(this->label22);
			this->tabPage2->Controls->Add(this->label21);
			this->tabPage2->Controls->Add(this->label20);
			this->tabPage2->Controls->Add(this->textBox12);
			this->tabPage2->Controls->Add(this->label19);
			this->tabPage2->Controls->Add(this->label18);
			this->tabPage2->Controls->Add(this->btn_set_color_thresh);
			this->tabPage2->Controls->Add(this->btn_get_go_signal);
			this->tabPage2->Controls->Add(this->btn_get_finish_line);
			this->tabPage2->Controls->Add(this->btn_get_obstacles);
			this->tabPage2->Controls->Add(this->btn_test_color_thresh);
			this->tabPage2->Controls->Add(this->comboBox4);
			this->tabPage2->Controls->Add(this->label17);
			this->tabPage2->Controls->Add(this->btn_get_midpoints);
			this->tabPage2->Controls->Add(this->btn_get_transform);
			this->tabPage2->Controls->Add(this->label16);
			this->tabPage2->Controls->Add(this->label15);
			this->tabPage2->Controls->Add(this->comboBox3);
			this->tabPage2->Controls->Add(this->label14);
			this->tabPage2->Controls->Add(this->textBox11);
			this->tabPage2->Controls->Add(this->btn_connect);
			this->tabPage2->Controls->Add(this->label13);
			this->tabPage2->Controls->Add(this->label12);
			this->tabPage2->Controls->Add(this->label11);
			this->tabPage2->Controls->Add(this->label10);
			this->tabPage2->Controls->Add(this->textBox10);
			this->tabPage2->Controls->Add(this->textBox9);
			this->tabPage2->Controls->Add(this->textBox8);
			this->tabPage2->Controls->Add(this->textBox7);
			this->tabPage2->Location = System::Drawing::Point(4, 22);
			this->tabPage2->Name = L"tabPage2";
			this->tabPage2->Padding = System::Windows::Forms::Padding(3);
			this->tabPage2->Size = System::Drawing::Size(550, 503);
			this->tabPage2->TabIndex = 1;
			this->tabPage2->Text = L"Camera Setup";
			this->tabPage2->UseVisualStyleBackColor = true;
			// 
			// btn_pitstop_entry
			// 
			this->btn_pitstop_entry->Enabled = false;
			this->btn_pitstop_entry->Location = System::Drawing::Point(429, 386);
			this->btn_pitstop_entry->Name = L"btn_pitstop_entry";
			this->btn_pitstop_entry->Size = System::Drawing::Size(121, 23);
			this->btn_pitstop_entry->TabIndex = 94;
			this->btn_pitstop_entry->Text = L"Pitstop Entry Point";
			this->btn_pitstop_entry->UseVisualStyleBackColor = true;
			this->btn_pitstop_entry->Click += gcnew System::EventHandler(this, &MyForm::btn_pitstop_entry_Click);
			// 
			// btn_load
			// 
			this->btn_load->Enabled = false;
			this->btn_load->Location = System::Drawing::Point(285, 387);
			this->btn_load->Name = L"btn_load";
			this->btn_load->Size = System::Drawing::Size(138, 23);
			this->btn_load->TabIndex = 93;
			this->btn_load->Text = L"Load Transform";
			this->btn_load->UseVisualStyleBackColor = true;
			this->btn_load->Click += gcnew System::EventHandler(this, &MyForm::btn_load_Click);
			// 
			// btn_save
			// 
			this->btn_save->Enabled = false;
			this->btn_save->Location = System::Drawing::Point(151, 387);
			this->btn_save->Name = L"btn_save";
			this->btn_save->Size = System::Drawing::Size(128, 23);
			this->btn_save->TabIndex = 92;
			this->btn_save->Text = L"Save Transform";
			this->btn_save->UseVisualStyleBackColor = true;
			this->btn_save->Click += gcnew System::EventHandler(this, &MyForm::btn_save_Click);
			// 
			// chkbox_manual_mode
			// 
			this->chkbox_manual_mode->AutoSize = true;
			this->chkbox_manual_mode->Checked = true;
			this->chkbox_manual_mode->CheckState = System::Windows::Forms::CheckState::Checked;
			this->chkbox_manual_mode->Location = System::Drawing::Point(438, 322);
			this->chkbox_manual_mode->Name = L"chkbox_manual_mode";
			this->chkbox_manual_mode->Size = System::Drawing::Size(90, 17);
			this->chkbox_manual_mode->TabIndex = 91;
			this->chkbox_manual_mode->Text = L"Manual mode";
			this->chkbox_manual_mode->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->chkbox_manual_mode->UseVisualStyleBackColor = true;
			// 
			// btn_preview_image
			// 
			this->btn_preview_image->Enabled = false;
			this->btn_preview_image->Location = System::Drawing::Point(17, 416);
			this->btn_preview_image->Name = L"btn_preview_image";
			this->btn_preview_image->Size = System::Drawing::Size(132, 23);
			this->btn_preview_image->TabIndex = 89;
			this->btn_preview_image->Text = L"Preview Image";
			this->btn_preview_image->UseVisualStyleBackColor = true;
			this->btn_preview_image->Click += gcnew System::EventHandler(this, &MyForm::btn_preview_image_Click);
			// 
			// textBox17
			// 
			this->textBox17->Location = System::Drawing::Point(382, 480);
			this->textBox17->Name = L"textBox17";
			this->textBox17->Size = System::Drawing::Size(55, 20);
			this->textBox17->TabIndex = 88;
			// 
			// textBox16
			// 
			this->textBox16->Location = System::Drawing::Point(305, 481);
			this->textBox16->Name = L"textBox16";
			this->textBox16->Size = System::Drawing::Size(55, 20);
			this->textBox16->TabIndex = 87;
			// 
			// textBox15
			// 
			this->textBox15->Location = System::Drawing::Point(229, 481);
			this->textBox15->Name = L"textBox15";
			this->textBox15->Size = System::Drawing::Size(55, 20);
			this->textBox15->TabIndex = 86;
			// 
			// textBox14
			// 
			this->textBox14->Location = System::Drawing::Point(382, 457);
			this->textBox14->Name = L"textBox14";
			this->textBox14->Size = System::Drawing::Size(55, 20);
			this->textBox14->TabIndex = 85;
			// 
			// textBox13
			// 
			this->textBox13->Location = System::Drawing::Point(305, 457);
			this->textBox13->Name = L"textBox13";
			this->textBox13->Size = System::Drawing::Size(55, 20);
			this->textBox13->TabIndex = 84;
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(379, 434);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(55, 13);
			this->label22->TabIndex = 83;
			this->label22->Text = L"Saturation";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(304, 434);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(56, 13);
			this->label21->TabIndex = 82;
			this->label21->Text = L"Luminosity";
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Location = System::Drawing::Point(226, 434);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(27, 13);
			this->label20->TabIndex = 81;
			this->label20->Text = L"Hue";
			// 
			// textBox12
			// 
			this->textBox12->Location = System::Drawing::Point(229, 457);
			this->textBox12->Name = L"textBox12";
			this->textBox12->Size = System::Drawing::Size(55, 20);
			this->textBox12->TabIndex = 80;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Location = System::Drawing::Point(175, 484);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(39, 13);
			this->label19->TabIndex = 79;
			this->label19->Text = L"Upper:";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(175, 464);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(39, 13);
			this->label18->TabIndex = 78;
			this->label18->Text = L"Lower:";
			// 
			// btn_set_color_thresh
			// 
			this->btn_set_color_thresh->Enabled = false;
			this->btn_set_color_thresh->Location = System::Drawing::Point(17, 474);
			this->btn_set_color_thresh->Name = L"btn_set_color_thresh";
			this->btn_set_color_thresh->Size = System::Drawing::Size(132, 23);
			this->btn_set_color_thresh->TabIndex = 77;
			this->btn_set_color_thresh->Text = L"Set Color Thresh";
			this->btn_set_color_thresh->UseVisualStyleBackColor = true;
			this->btn_set_color_thresh->Click += gcnew System::EventHandler(this, &MyForm::btn_set_color_thresh_Click);
			// 
			// btn_get_go_signal
			// 
			this->btn_get_go_signal->Enabled = false;
			this->btn_get_go_signal->Location = System::Drawing::Point(429, 352);
			this->btn_get_go_signal->Name = L"btn_get_go_signal";
			this->btn_get_go_signal->Size = System::Drawing::Size(121, 23);
			this->btn_get_go_signal->TabIndex = 76;
			this->btn_get_go_signal->Text = L"Get Go Signal";
			this->btn_get_go_signal->UseVisualStyleBackColor = true;
			this->btn_get_go_signal->Click += gcnew System::EventHandler(this, &MyForm::btn_get_go_signal_Click);
			// 
			// btn_get_finish_line
			// 
			this->btn_get_finish_line->Enabled = false;
			this->btn_get_finish_line->Location = System::Drawing::Point(285, 352);
			this->btn_get_finish_line->Name = L"btn_get_finish_line";
			this->btn_get_finish_line->Size = System::Drawing::Size(138, 23);
			this->btn_get_finish_line->TabIndex = 75;
			this->btn_get_finish_line->Text = L"Get Finish Line";
			this->btn_get_finish_line->UseVisualStyleBackColor = true;
			this->btn_get_finish_line->Click += gcnew System::EventHandler(this, &MyForm::btn_get_finish_line_Click);
			// 
			// btn_get_obstacles
			// 
			this->btn_get_obstacles->Enabled = false;
			this->btn_get_obstacles->Location = System::Drawing::Point(151, 352);
			this->btn_get_obstacles->Name = L"btn_get_obstacles";
			this->btn_get_obstacles->Size = System::Drawing::Size(128, 23);
			this->btn_get_obstacles->TabIndex = 74;
			this->btn_get_obstacles->Text = L"Get Obstacles";
			this->btn_get_obstacles->UseVisualStyleBackColor = true;
			this->btn_get_obstacles->Click += gcnew System::EventHandler(this, &MyForm::btn_get_obstacles_Click);
			// 
			// btn_test_color_thresh
			// 
			this->btn_test_color_thresh->Enabled = false;
			this->btn_test_color_thresh->Location = System::Drawing::Point(17, 445);
			this->btn_test_color_thresh->Name = L"btn_test_color_thresh";
			this->btn_test_color_thresh->Size = System::Drawing::Size(132, 23);
			this->btn_test_color_thresh->TabIndex = 73;
			this->btn_test_color_thresh->Text = L"Test Color Thresh";
			this->btn_test_color_thresh->UseVisualStyleBackColor = true;
			this->btn_test_color_thresh->Click += gcnew System::EventHandler(this, &MyForm::btn_test_color_thresh_Click);
			// 
			// comboBox4
			// 
			this->comboBox4->FormattingEnabled = true;
			this->comboBox4->Items->AddRange(gcnew cli::array< System::Object^  >(4) {L"0", L"1", L"2", L"3"});
			this->comboBox4->Location = System::Drawing::Point(69, 325);
			this->comboBox4->Name = L"comboBox4";
			this->comboBox4->Size = System::Drawing::Size(61, 21);
			this->comboBox4->TabIndex = 72;
			this->comboBox4->Text = L"0";
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(20, 328);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(43, 13);
			this->label17->TabIndex = 71;
			this->label17->Text = L"Camera";
			// 
			// btn_get_midpoints
			// 
			this->btn_get_midpoints->Enabled = false;
			this->btn_get_midpoints->Location = System::Drawing::Point(285, 318);
			this->btn_get_midpoints->Name = L"btn_get_midpoints";
			this->btn_get_midpoints->Size = System::Drawing::Size(138, 22);
			this->btn_get_midpoints->TabIndex = 70;
			this->btn_get_midpoints->Text = L"Get Midpoints";
			this->btn_get_midpoints->UseVisualStyleBackColor = true;
			this->btn_get_midpoints->Click += gcnew System::EventHandler(this, &MyForm::btn_get_midpoints_Click);
			// 
			// btn_get_transform
			// 
			this->btn_get_transform->Enabled = false;
			this->btn_get_transform->Location = System::Drawing::Point(151, 318);
			this->btn_get_transform->Name = L"btn_get_transform";
			this->btn_get_transform->Size = System::Drawing::Size(128, 23);
			this->btn_get_transform->TabIndex = 69;
			this->btn_get_transform->Text = L"Get Transform";
			this->btn_get_transform->UseVisualStyleBackColor = true;
			this->btn_get_transform->Click += gcnew System::EventHandler(this, &MyForm::btn_get_transform_Click);
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(241, 49);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(102, 13);
			this->label16->TabIndex = 13;
			this->label16->Text = L"Number of cameras:";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(85, 16);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(64, 13);
			this->label15->TabIndex = 12;
			this->label15->Text = L"Port number";
			// 
			// comboBox3
			// 
			this->comboBox3->FormattingEnabled = true;
			this->comboBox3->Items->AddRange(gcnew cli::array< System::Object^  >(4) {L"1", L"2", L"3", L"4"});
			this->comboBox3->Location = System::Drawing::Point(349, 43);
			this->comboBox3->Name = L"comboBox3";
			this->comboBox3->Size = System::Drawing::Size(121, 21);
			this->comboBox3->TabIndex = 11;
			this->comboBox3->Text = L"1";
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(15, 231);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(58, 13);
			this->label14->TabIndex = 10;
			this->label14->Text = L"IP Address";
			// 
			// textBox11
			// 
			this->textBox11->Location = System::Drawing::Point(88, 231);
			this->textBox11->Name = L"textBox11";
			this->textBox11->Size = System::Drawing::Size(148, 20);
			this->textBox11->TabIndex = 9;
			this->textBox11->Text = L"169.254.88.211";
			// 
			// btn_connect
			// 
			this->btn_connect->Location = System::Drawing::Point(18, 271);
			this->btn_connect->Name = L"btn_connect";
			this->btn_connect->Size = System::Drawing::Size(182, 30);
			this->btn_connect->TabIndex = 8;
			this->btn_connect->Text = L"Connect to Roborealm";
			this->btn_connect->UseVisualStyleBackColor = true;
			this->btn_connect->Click += gcnew System::EventHandler(this, &MyForm::btn_connect_Click);
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(15, 181);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(52, 13);
			this->label13->TabIndex = 7;
			this->label13->Text = L"Camera 4";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(15, 134);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(52, 13);
			this->label12->TabIndex = 6;
			this->label12->Text = L"Camera 3";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(15, 92);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(52, 13);
			this->label11->TabIndex = 5;
			this->label11->Text = L"Camera 2";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(15, 43);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(52, 13);
			this->label10->TabIndex = 4;
			this->label10->Text = L"Camera 1";
			// 
			// textBox10
			// 
			this->textBox10->Location = System::Drawing::Point(88, 178);
			this->textBox10->Name = L"textBox10";
			this->textBox10->Size = System::Drawing::Size(82, 20);
			this->textBox10->TabIndex = 3;
			this->textBox10->Text = L"6063";
			// 
			// textBox9
			// 
			this->textBox9->Location = System::Drawing::Point(88, 134);
			this->textBox9->Name = L"textBox9";
			this->textBox9->Size = System::Drawing::Size(82, 20);
			this->textBox9->TabIndex = 2;
			this->textBox9->Text = L"6062";
			// 
			// textBox8
			// 
			this->textBox8->Location = System::Drawing::Point(88, 89);
			this->textBox8->Name = L"textBox8";
			this->textBox8->Size = System::Drawing::Size(82, 20);
			this->textBox8->TabIndex = 1;
			this->textBox8->Text = L"6061";
			// 
			// textBox7
			// 
			this->textBox7->Location = System::Drawing::Point(88, 42);
			this->textBox7->Name = L"textBox7";
			this->textBox7->Size = System::Drawing::Size(82, 20);
			this->textBox7->TabIndex = 0;
			this->textBox7->Text = L"6060";
			// 
			// imageList1
			// 
			this->imageList1->ColorDepth = System::Windows::Forms::ColorDepth::Depth8Bit;
			this->imageList1->ImageSize = System::Drawing::Size(16, 16);
			this->imageList1->TransparentColor = System::Drawing::Color::Transparent;
			// 
			// chkbox_pace
			// 
			this->chkbox_pace->AutoSize = true;
			this->chkbox_pace->Location = System::Drawing::Point(34, 425);
			this->chkbox_pace->Name = L"chkbox_pace";
			this->chkbox_pace->Size = System::Drawing::Size(98, 17);
			this->chkbox_pace->TabIndex = 73;
			this->chkbox_pace->Text = L"Pace car mode";
			this->chkbox_pace->UseVisualStyleBackColor = true;
			this->chkbox_pace->CheckedChanged += gcnew System::EventHandler(this, &MyForm::chkbox_pace_CheckedChanged);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(587, 547);
			this->Controls->Add(this->tabControl1);
			this->Name = L"MyForm";
			this->Text = L"Serial Command";
			this->tabControl1->ResumeLayout(false);
			this->tabPage1->ResumeLayout(false);
			this->tabPage1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->EndInit();
			this->tabPage2->ResumeLayout(false);
			this->tabPage2->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion


private: ref class Controller^ controller_;
public: void setParent(ref class Controller^ c); 

private: bool is_decimal(System::String^ s);
private: void intializeTrackbars();
private: array<System::Byte>^ interpretHexadecimalCommand();
private: void sendSerialData(array<System::Byte>^ byteArray);
public: void setMotorSpeeds();
public: void setMotorSpeeds(int speed_L, int speed_R);
public: void setMotorGains();


public: void SetText(String^ text);
		void MyForm:: DrawCVImage(cv::Mat* colorImage);

public : cv::Mat *image;

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

private: System::Void updateImage( System::Object^ source, System::Timers::ElapsedEventArgs^ e ){
			 
			 std::cout << "timer running..." << std::endl;
			 if (camera_vision)
			{
		
				
				System::Drawing::Graphics^ graphics = this->vision_form->CreateGraphics();
				System::IntPtr ptr(this->image->ptr());
				std::cout <<"columns:" <<this->image->cols << "rows:" << this->image->rows << std::endl;
				System::Drawing::Bitmap^ b  =gcnew System::Drawing::Bitmap(this->image->cols,this->image->rows,this->image->step,System::Drawing::Imaging::PixelFormat::Format24bppRgb,ptr);
				System::Drawing::RectangleF rect(0,0,this->vision_form->Width,this->vision_form->Height);
				std::cout << "image drawn..."<<std::endl;
				graphics->DrawImage(b,rect);
				std::cout << "graphic draws..."<<std::endl;

			}
			 else
				 drawTimer->Enabled = false;
		 }
private: System::Void dataReceivedHandler(System::Object^ sender,\
			 System::IO::Ports::SerialDataReceivedEventArgs^ e)
		 {
			 try
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
			 catch(TimeoutException ^)
			 {
				 std::cout<< "serial port timed out..." << std::endl;
			 }
		 }
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) ;
private: System::Void label6_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->PID_form = gcnew PIDTuningForm();
			 this->PID_form->setParent(this);
			 this->PID_form->Show();
		 }
private: System::Void radioButton3_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		 }

private: System::Void button5_Click(System::Object^  sender, System::EventArgs^  e);

private: System::Void btn_get_transform_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_get_midpoints_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void button8_Click_1(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_connect_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_test_color_thresh_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_get_obstacles_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_get_finish_line_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_get_go_signal_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void button14_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_set_color_thresh_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void button16_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_preview_image_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_save_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_load_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void btn_pitstop_entry_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void chkbox_pace_CheckedChanged(System::Object^  sender, System::EventArgs^  e);
};
}
