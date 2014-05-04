// RaceControl.cpp : main project file.

#include "stdafx.h"

#include "MyForm.h"

using namespace System;
using namespace System::Windows::Forms;


using namespace System;
[STAThread]
int main(array<System::String ^> ^args)
{
    //Console::WriteLine(L"Hello World");
	Application::EnableVisualStyles();
    Application::SetCompatibleTextRenderingDefault(false); 
    // Create the main window and run it
	RaceControl::MyForm form;
	//form = gcnew RaceControl::MyForm();
	Application::Run(%form);
    return 0;
}
