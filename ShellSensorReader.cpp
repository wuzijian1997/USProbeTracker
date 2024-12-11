#include "ShellSensorReader.h"


ShellSensorReader::ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout)
	: _portName(portName), _baudRate(baudRate),_timeout(timeout)
{}

ShellSensorReader::~ShellSensorReader()
{
	//Closes force sensor threading things
	{
		std::lock_guard<std::mutex> lock(_forceSenseMutex);
		_runForceSense_Thread = false;
	}

	if (_forceSense_Thread && _forceSense_Thread->joinable())
	{
		_forceSense_Thread->join();
	}

	while (!_forceSenseQueue.empty()) {
		_forceSenseQueue.pop();
	}

	//Closes IMU sensor threading things
	{
		std::lock_guard<std::mutex> lock(_tempImuMutex);
		_runTempImu_Thread = false;
	}

	if (_tempImu_Thread && _tempImu_Thread->joinable())
	{
		_tempImu_Thread->join();
	}

	while (!_tempImuQueue.empty()) {
		_tempImuQueue.pop();
	}

	//Closes the serial port
	if (_isSerialPortOpen)
	{
		//If the serial port is open we close it
		CloseHandle(_serialHandle);
	}
}


//Configures the serial handle, serial interface settings, and starts the threads
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
	
	if (_isSerialPortOpen)
	{
		//Starts the reading threads

		//Starts the force sensor thread
		_runForceSense_Thread = true;
		_forceSense_Thread= std::make_shared<std::thread>([this]() {readLines(_forceSenseQueue, _forceSenseTag, _forceSenseMutex, _ForceSenseReadingArrived, _runForceSense_Thread); });
		
		//Starts the temperature/IMU reading thread
		//Starts the force sensor thread
		_runTempImu_Thread = true;
		_tempImu_Thread = std::make_shared<std::thread>([this]() {readLines(_tempImuQueue, _tempImuTag, _tempImuMutex, _tempImuReadingArrived, _runTempImu_Thread); });


	}
	
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


void ShellSensorReader::readLines(std::queue<std::string>& sensor_queue,std::string& lineTag, std::mutex& mutex_ptr,std::condition_variable& reading_arrived,std::atomic<bool>& run_thread)
{
	if (!_isSerialPortOpen)
	{
		std::cout << "Serial Port to Shell Not Open" << std::endl;
		//Ends thread process if serial port is not connected
		return;
	}

	char tempChar;
	DWORD bytesRead;
	std::string _buffer; //Buffer of characters read in on serial port

	//Vars for timeout checking
	std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point curr_time = std::chrono::steady_clock::now();	

	while (run_thread)
	{
		//Read it into bytesRead, and then check if it is greater than 0
		if (ReadFile(_serialHandle, &tempChar, 1, &bytesRead, nullptr) && bytesRead > 0)
		{
			last_time = std::chrono::steady_clock::now(); //Updates last successful read time
			if (tempChar == '\n') //New line character is met, we clear the line buffer, check if it is a force line, and append to the queue if it is
			{
				std::string line = _buffer;
				_buffer.clear();

				//Check if it is line with name "lineTag" if it is, we push to the sensor queue
				if (checkLineTag(lineTag, line))
				{
					//Strips the tag from the string
					line.erase(0, lineTag.length());
					{
						std::lock_guard<std::mutex> l{ mutex_ptr };
						if (sensor_queue.size() > 10) {
							sensor_queue.pop(); // Discard the oldest readings if the queue is too large
						}
						sensor_queue.push(line);
					}
					//Notifies the recipient
					reading_arrived.notify_one(); 

				}

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
				//Does nothing for now

			}

		}
	}
}

bool ShellSensorReader::checkLineTag(std::string& lineTag,std::string& serialLine)
{
	//Returns true if the tag is in the serial line (force sensor tag is "FSN:" or the temp/IMU is "TGA:")
	return serialLine.find(lineTag) != std::string::npos;

}


//Methods to get the force and IMU lines from the two threading functions

void ShellSensorReader::getForceString(std::string& force_string)
{
	std::unique_lock<std::mutex> lock{ _forceSenseMutex };
	//Returns empty frame if waiting longer than 20 ms (50 Hz)
	if (!_ForceSenseReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return !_forceSenseQueue.empty(); }))
	{
		//We had a timeout event, return with force_string set to 12 NaN's
		force_string = TWELVE_NaNs;
		lock.unlock();
		return;
	}
	if (!_forceSenseQueue.empty())
	{
		//Grab frame if not empty queue
		force_string = _forceSenseQueue.front();
		_forceSenseQueue.pop();
	}
	else {
		force_string = TWELVE_NaNs;
	}
	lock.unlock();
	return;
}

void ShellSensorReader::getTempIMUString(std::string& temp_imu_string)
{
	std::unique_lock<std::mutex> lock{ _tempImuMutex };
	//Returns empty frame if waiting longer than 20 ms (50 Hz)
	if (!_tempImuReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return !_tempImuQueue.empty(); }))
	{
		//We had a timeout event, return with force_string set to 12 NaN's
		temp_imu_string = SIX_NaNs;
		lock.unlock();
		return;
	}
	if (!_tempImuQueue.empty())
	{
		//Grab frame if not empty queue
		temp_imu_string = _tempImuQueue.front();
		_tempImuQueue.pop();
	}
	else {
		temp_imu_string = SIX_NaNs;
	}
	lock.unlock();
	return;

}
