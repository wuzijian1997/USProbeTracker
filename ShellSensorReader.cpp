#include "ShellSensorReader.h"


ShellSensorReader::ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout)
	: _portName(portName), _baudRate(baudRate),_timeout(timeout)
{}

ShellSensorReader::~ShellSensorReader()
{
	if (_isSerialPortOpen)
	{
		//If the serial port is open we close it
		CloseHandle(_serialHandle);
	}
}


//Configures the serial handle and serial interface settings
bool ShellSensorReader::initialize()
{
	_serialHandle = CreateFile(_portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
	
	//Checks that the serial port is opened
	if (_serialHandle == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error Opening Serial Port" << std::endl;
		return false;
	}

	_isSerialPortOpen = configurePort(); //Configures serial port settings and checks if it is open
	return _isSerialPortOpen; 

}

bool ShellSensorReader::configurePort()
{
	DCB serialParams = { 0 };
	serialParams.DCBlength = sizeof(serialParams);

	if (!GetCommState(_serialHandle, &serialParams))
	{
		std::cout << "Error Getting Serial Port State" << std::endl;
		return false;
	}

	//Sets serial interface parameters
	serialParams.BaudRate = _baudRate;
	serialParams.ByteSize = 8;
	serialParams.StopBits = ONESTOPBIT;
	serialParams.Parity = NOPARITY;
	if (!SetCommState(_serialHandle, &serialParams))
	{
		std::cout << "Error Setting Serial Port State" << std::endl;
		return false;
	}

	//Set the timeouts
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = SHELLSENSOR_TIMEOUT;
	timeouts.ReadTotalTimeoutConstant = SHELLSENSOR_TIMEOUT;
	timeouts.ReadTotalTimeoutMultiplier = SHELLSENSOR_TIMEOUT;
	if (!SetCommTimeouts(_serialHandle, &timeouts))
	{
		std::cout << "Error Setting Serial Port Timeouts" << std::endl;
		return false;
	}
	return true;

}


void ShellSensorReader::readLines()
{
	if (!_isSerialPortOpen)
	{
		std::cout << "Serial Port to Shell Not Open" << std::endl;
		return;
	}
	char tempChar;
	DWORD bytesRead;

	//Vars for timeout checking
	std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point curr_time = std::chrono::steady_clock::now();
	

	while (true)
	{
		//Read it into bytesRead, and then check if it is greater than 0
		if (ReadFile(_serialHandle, &tempChar, 1, &bytesRead, nullptr) && bytesRead > 0)
		{
			last_time = std::chrono::steady_clock::now(); //Updates last successful read time
			if (tempChar == '\n') //New line character is met, and we read out the line
			{
				std::string line = _buffer;
				_buffer.clear();
				std::cout << line << std::endl; //Print the line
			}
			else if (tempChar != '\r') //ignore carriage return
			{
				_buffer += tempChar; // appends char to the buffer
			}
		}
		else
		{
			curr_time = std::chrono::steady_clock::now(); //updates curr time of unsuccessful read
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - last_time).count();
			if ((int)elapsed_ms > SHELLSENSOR_TIMEOUT)
			{
				std::cout << "Serial Timout Triggered" << std::endl;
				return;

			}

		}
	}
}