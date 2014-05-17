#pragma once

namespace RaceControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	

	/// <summary>
	/// Summary for PIDTuningForm
	/// </summary>
	public ref class PIDTuningForm : public System::Windows::Forms::Form
	{
	public:
		 PIDTuningForm(void)
		{
			this->InitializeComponent();
			this->textBox1->Text = "0";
			this->textBox2->Text = "0";
			this->textBox3->Text = "0";


			//
			//TODO: Add the constructor code here
			//
		}
		
	    //set parent and initialize texboxes
		void setParent(ref class MyForm^ f);
   
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~PIDTuningForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::GroupBox^  groupBox1;
	protected: 
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label1;
	public: System::Windows::Forms::TextBox^  textBox3;
	private: 
	public: System::Windows::Forms::TextBox^  textBox2;
	public: System::Windows::Forms::TextBox^  textBox1;








	private: 





	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  button2;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;
		ref class MyForm^ pForm;
	
	

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label3);
			this->groupBox1->Controls->Add(this->label2);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->textBox3);
			this->groupBox1->Controls->Add(this->textBox2);
			this->groupBox1->Controls->Add(this->textBox1);
			this->groupBox1->Location = System::Drawing::Point(9, 13);
			this->groupBox1->Name = L"groupBox1";

			this->groupBox1->Size = System::Drawing::Size(255, 202);

			this->groupBox1->Size = System::Drawing::Size(240, 202);

			this->groupBox1->TabIndex = 0;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"PID parameters";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(6, 155);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(26, 13);
			this->label3->TabIndex = 5;
			this->label3->Text = L"K_d";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(6, 109);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(22, 13);
			this->label2->TabIndex = 4;
			this->label2->Text = L"K_i";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(6, 61);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(26, 13);
			this->label1->TabIndex = 3;
			this->label1->Text = L"K_p";
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(46, 155);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(58, 20);
			this->textBox3->TabIndex = 2;
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(46, 109);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(58, 20);
			this->textBox2->TabIndex = 1;
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(46, 61);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(58, 20);
			this->textBox1->TabIndex = 0;
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(43, 234);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(88, 22);
			this->button1->TabIndex = 2;
			this->button1->Text = L"Set Gains";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &PIDTuningForm::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(172, 235);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(92, 21);
			this->button2->TabIndex = 3;
			this->button2->Text = L"Cancel";
			this->button2->UseVisualStyleBackColor = true;
			// 
			// PIDTuningForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(284, 262);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->groupBox1);
			this->Name = L"PIDTuningForm";
			this->Text = L"PIDTuningForm";
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e); //{
				// this->form->kp_l = System::Convert::ToInt16(this->textBox1->Text);
				 //this->form->ki_l = System::Convert::ToInt16(this->textBox2->Text);
				 //this->form->kd_l = System::Convert::ToInt16(this->textBox3->Text);
				 //this->form->kp_r = System::Convert::ToInt16(this->textBox4->Text);
				 //this->form->ki_r = System::Convert::ToInt16(this->textBox5->Text);
				 //this->form->kd_r = System::Convert::ToInt16(this->textBox6->Text);
				// this->form->SetText(System::Convert::ToString(this->form->kp_l));
			// }
};
}
