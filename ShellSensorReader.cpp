#include "ShellSensorReader.h"


ShellSensorReader::ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout)
	: _portName(portName), _baudRate(baudRate),_timeout(timeout)
{}

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
						if (_forceSenseQueue.size() > 10) {
							_forceSenseQueue.pop(); // Discard the oldest readings if the queue is too large
						}
						_forceSenseQueue.push(line);
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
						if (_tempImuQueue.size() > 10) {
							_tempImuQueue.pop(); // Discard the oldest readings if the queue is too large
						}
						_tempImuQueue.push(line);
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

void ShellSensorReader::getForceString(std::string& force_string)
{
	std::unique_lock<std::mutex> lock{ _sensorReadingMutex };
	//Returns NaNs if waiting longer than 20 ms (50 Hz)
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
	std::unique_lock<std::mutex> lock{ _sensorReadingMutex };
	//Returns NaNs if waiting longer than 20 ms (50 Hz)
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
	return;

}


//Helper method to convert raw force sensor readings to actual forces

std::string ShellSensorReader::calculateForceVals(std::string &raw_force_string, Eigen::MatrixXd &calib_mat, Eigen::Vector3d &zeroing_offset)
{
	
	//raw_force_string is 1x12 raw sensor string
	//raw_force_vector is 1x13 eigen vector: 1x12 raw sensor values/4096 with a 1 appended
	Eigen::VectorXd raw_force_vector = forcestringToForceVector(raw_force_string);

	//Computes the x,y,z force from initial ATI reading
	Eigen::Vector3d force_xyz = calib_mat * raw_force_vector.transpose();

	//Adds the zeroing offset if present
	force_xyz = force_xyz + zeroing_offset;


	//Converts the Eigen vector back to a string
	std::string string_force_xyz = eigenForceToStringForce(force_xyz);
	return string_force_xyz;

}

Eigen::VectorXd ShellSensorReader::forcestringToForceVector(std::string& raw_force_string)
{
	Eigen::VectorXd raw_force_vector(13); //Inits the eigen force vector
	std::stringstream ss(raw_force_string);

	std::string substr;
	int i = 0;

	while (std::getline(ss, substr, ',') && i < 12) {
		raw_force_vector[i] = std::stod(substr) / 4096.0f; // Normalize value
		i++;
	}

	raw_force_vector[12] = 1.0f;
	return raw_force_vector;

}

std::string ShellSensorReader::eigenForceToStringForce(Eigen::Vector3d& force_xyz)
{
	std::ostringstream oss;
	for (int i = 0; i < force_xyz.size(); ++i) {
		oss << force_xyz[i];
		if (i != force_xyz.size() - 1) { // Add a comma between elements
			oss << ",";
		}
	}
	std::string string_force = oss.str();
	return string_force;
}

Eigen::MatrixXd ShellSensorReader::readCSVToEigenMatrix(const std::string& file_path, int rows, int cols)
{
	//Creates ifstream object to read csv row by row
	std::ifstream file(file_path);
	Eigen::MatrixXd matrix(rows, cols);
	std::string line;
	int row = 0;

	while (std::getline(file, line) && row < rows) {
		std::stringstream line_stream(line);
		std::string cell;
		int col = 0;

		while (std::getline(line_stream, cell, ',') && col < cols) {
			matrix(row, col) = std::stod(cell);
			++col;
		}

		++row;
	}

	file.close();
	return matrix;

}