
#pragma once

namespace RaceControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for CameraView
	/// </summary>
	public ref class CameraView : public System::Windows::Forms::Form
	{
	public:
		CameraView(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}
		CameraView(ref class MyForm^ f)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

		void setParent(ref class MyForm^ f);
	private: ref class MyForm^ pForm_;
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~CameraView();
		/*{
			pForm_->camera_vision = false;
			if (components)
			{
				delete components;
			}
		}*/

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;
		

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = gcnew System::ComponentModel::Container();
			this->Size = System::Drawing::Size(300,300);
			this->Text = L"CameraView";
			this->Padding = System::Windows::Forms::Padding(0);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
		}
#pragma endregion
	};
}
