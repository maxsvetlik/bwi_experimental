#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <vector>
#include <ios>
//Note that under windows, you may/will have to perform other #include

using namespace std;

/**
 * @example Example of the function GetCodeVersion().
 */
int main()
{
	int result;
	std::vector<int> data;

	cout << "GetCodeVersion function example" << endl;

	//Handle for the library's command layer.
	void * commandLayer_handle;

	//Function pointers to the functions we need
	int (*MyInitAPI)();
	int (*MyCloseAPI)();
	int (*MyGetCodeVersion)(std::vector<int> &);

	//We load the library (Under Windows, use the function LoadLibrary)
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library (Under Windows, use GetProcAddress)
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyGetCodeVersion = (int (*)(std::vector<int> &)) dlsym(commandLayer_handle,"GetCodeVersion");

	//If the was loaded correctly
	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetCodeVersion == NULL))
	{
		cout << "Unable to initialize the command layer." << endl;
	}
	else
	{
		cout << "The command has been initialized correctly." << endl;

		cout << "Calling the method InitAPI()" << endl;
		result = (*MyInitAPI)();
		cout << "result of InitAPI() = " << result << endl;

		result = (*MyGetCodeVersion)(data);

		cout << "DSP's firmware version is : " << std::hex << data[0] << "." << std::hex <<  data[1] << "." << std::hex <<  data[2] << "." << std::hex <<  data[30] << endl;

		cout << "Actuator 1's firmware version is : " << std::hex <<  data[3] << "." << std::hex <<  data[4] << "." << std::hex <<  data[5]  << endl;
		cout << "Actuator 2's firmware version is : " << std::hex <<  data[6] << "." << std::hex <<  data[7] << "." << std::hex <<  data[8]  << endl;
		cout << "Actuator 3's firmware version is : " << std::hex <<  data[9] << "." << std::hex <<  data[10] << "." << std::hex <<  data[11] << endl;
		cout << "Actuator 4's firmware version is : " << std::hex <<  data[12] << "." << std::hex <<  data[13] << "." << std::hex <<  data[14] << endl;
		cout << "Actuator 5's firmware version is : " << std::hex <<  data[15] << "." << std::hex <<  data[16] << "." << std::hex <<  data[17] << endl;
		cout << "Actuator 6's firmware version is : " << std::hex <<  data[18] << "." << std::hex <<  data[19] << "." << std::hex <<  data[20] << endl;

		cout << "Finger 1's firmware version is : " << std::hex <<  data[21] << "." << std::hex <<  data[22] << "." << std::hex <<  data[23] << endl;
		cout << "Finger 2's firmware version is : " << std::hex <<  data[24] << "." << std::hex <<  data[25] << "." << std::hex <<  data[26] << endl;
		cout << "Finger 3's firmware version is : " << std::hex <<  data[27] << "." << std::hex <<  data[28] << "." << std::hex <<  data[29] << endl;

		cout << "CAN interface 1's firmware version is : " << std::hex <<  data[31] << "." << std::hex <<  data[32] << "." << std::hex <<  data[33] << endl;
		cout << "CAN interface 2's firmware version is : " << std::hex <<  data[34] << "." << std::hex <<  data[35] << "." << std::hex <<  data[36] << endl;

		cout << "Calling the method CloseAPI()" << endl;
		result = (*MyCloseAPI)();
		cout << "result of CloseAPI() = " << result << endl;
	}

	return 0;
}
