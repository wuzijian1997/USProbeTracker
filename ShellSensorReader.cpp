#include "ShellSensorReader.h"


ShellSensorReader::ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout, Eigen::MatrixXd force_calibration_mat, Eigen::Vector3d force_zeroing_offset, Eigen::MatrixXd force_compensation_mat)
	: _portName(portName), _baudRate(baudRate),_timeout(timeout), _force_calibration_mat(force_calibration_mat),_force_zeroing_offset(force_zeroing_offset),_force_compensation_mat(force_compensation_mat)
{
	enableForceSensor = ConfigInstance::GetInstance()->core().enableForceSensor;
}

ShellSensorReader::~ShellSensorReader()
{
	//Closes force sensor threading things
	{
		std::lock_guard<std::mutex> lock(_sensorReadingMutex);
		_runThread = false;
	}

	if (_senseReadingThread && _senseReadingThread->joinable())
	{
		_senseReadingThread->join();
	}

	while (!_forceSenseQueue.empty()) {
		_forceSenseQueue.pop();
	}

	while (!_tempImuQueue.empty()) {
		_tempImuQueue.pop();
	}

	while (!_forceXYZQueue.empty()) {
		_forceXYZQueue.pop();
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
	if (!enableForceSensor) {
		std::cout << "Force sensor reading is disabled in the config file." << std::endl;
		return true;
	}

	_serialHandle = CreateFile(reinterpret_cast<LPCSTR>(_portName.c_str()), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
	
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

		//Starts Sensor Reading Thread
		_runThread = true;
		_senseReadingThread = std::make_shared<std::thread>([this]() {readLines();});		

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


void ShellSensorReader::readLines()
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
	std::string force_string_xyz; //Holds the converted raw force values to xyz force

	//Vars for timeout checking
	std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point curr_time = std::chrono::steady_clock::now();	

	while (_runThread)
	{
		//Read it into bytesRead, and then check if it is greater than 0
		if (ReadFile(_serialHandle, &tempChar, 1, &bytesRead, nullptr) && bytesRead > 0)
		{
			last_time = std::chrono::steady_clock::now(); //Updates last successful read time
			if (tempChar == '\n') //New line character is met, we clear the line buffer, check if it is a force line, and append to the queue if it is
			{
				std::string line = _buffer;
				_buffer.clear(); //Clears the buffer

				//Check if it is line with name _forceSenseTag, if it is push to the force sensor queue
				//Checking for force line
				if (checkLineTag(_forceSenseTag, line))
				{
					//Strips the tag from the string
					line.erase(0, _forceSenseTag.length());
					//Writes force reading to the queue
					{
						std::lock_guard<std::mutex> l{_sensorReadingMutex};
						//Clears the queue before pushing the newest reading (we only have one reading in the queue at a time)
						while (!_forceSenseQueue.empty()) {
							_forceSenseQueue.pop();
						}
						_forceSenseQueue.push(line);

						//Calculates the xyz from raw force
						force_string_xyz = calculateForceVals(line, _force_calibration_mat, _force_zeroing_offset,_force_compensation_mat);
						while (!_forceXYZQueue.empty()) {
							_forceXYZQueue.pop();
						}
						_forceXYZQueue.push(force_string_xyz);

					}
					//Notifies the recipient
					_ForceSenseReadingArrived.notify_one();

				}

				else if(checkLineTag(_tempImuTag, line))
				{
					//Strips the tag from the string
					line.erase(0, _tempImuTag.length());
					//Writes force reading to the queue
					{
						std::lock_guard<std::mutex> l{ _sensorReadingMutex };
						//Clears the queue before pushing the newest reading (we only have one reading in the queue at a time)
						while (!_tempImuQueue.empty()) {
							_tempImuQueue.pop();
						}
						_tempImuQueue.push(line);;
					}
					//Notifies the recipient
					_tempImuReadingArrived.notify_one();
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
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
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


//Methods to get the force and Temp/IMU lines from the two threading functions

void ShellSensorReader::getForceString(std::string& force_string,std::string& force_xyz_string)
{
	std::unique_lock<std::mutex> lock{ _sensorReadingMutex };
	//Returns NaNs if waiting longer than _timeout value
	if (!_ForceSenseReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return (!_forceSenseQueue.empty())&&(!_forceXYZQueue.empty()); }))
	{
		//We had a timeout event, return with force_string set to 12 NaN's
		force_string = TWELVE_NaNs;
		lock.unlock();
		return;
	}

	if ((!_forceSenseQueue.empty()) && (!_forceXYZQueue.empty()))
	{
		//Grab frame if not empty queue
		force_string = _forceSenseQueue.front();
		_forceSenseQueue.pop();
		force_xyz_string = _forceXYZQueue.front();
		_forceXYZQueue.pop();
	}
	else {
		force_string = TWELVE_NaNs;
		force_xyz_string = "NaN,NaN,NaN";
	}
	lock.unlock();
}

void ShellSensorReader::getTempIMUString(std::string& temp_imu_string)
{
	std::unique_lock<std::mutex> lock{ _sensorReadingMutex };
	//Returns NaNs if waiting longer than _timeout value
	if (!_tempImuReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return !_tempImuQueue.empty(); }))
	{
		//We had a timeout event, return with string set to 6 NaN's
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
}


