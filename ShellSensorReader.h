#pragma once
#include "IRSegmentation.h" //Has a bunch of includes
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <condition_variable>
#include <queue>
#include "Utils.h"

const DWORD SHELLSENSOR_BAUDRATE = CBR_115200;
const std::wstring SHELLSENSOR_PORTNAME = L"COM9";
const int SHELLSENSOR_TIMEOUT = 100; //Timeout that we will try to read for in ms
const std::string FORCESENSOR_TAG = "FSN:";
const std::string TEMPIMU_TAG = "TGA:";
const std::string TWELVE_NaNs = "NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN";
const std::string SIX_NaNs = "NaN,NaN,NaN,NaN,NaN,NaN";
class ShellSensorReader
{
public:
	//Inits port name and baud rate vars
	int _timeout = 20; //Timeout is how long we wait for a thread
	ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout, Eigen::MatrixXd force_calibration_mat, Eigen::Vector3d force_zeroing_offset, Eigen::MatrixXd force_compensation_mat);
	~ShellSensorReader();

	bool initialize(); //Inits the port connection and serial interface setup
	
	void getForceString(std::string& force_string, std::string& force_xyz_string);
	void getTempIMUString(std::string& temp_imu_string);

	Eigen::Vector3d _force_zeroing_offset; //We manually change this during zeroing
	

private:
	std::wstring _portName;
	HANDLE _serialHandle;
	DWORD _baudRate;
	bool _isSerialPortOpen = false;; //Tracks whether the serial port "_portName" is open

	bool configurePort(); //Sets up port parameters (e.g. baud rate, byte size etc.)
	bool checkLineTag(std::string& lineTag, std::string& serialLine); //Checks whether the line starts with "FSN:" or "TGA:"
	void readLines(); //Reads a lines of force sensors up until "\n", runs on one thread

	//Values for converting raw force values to xyz force vals
	Eigen::MatrixXd _force_calibration_mat;	
	Eigen::MatrixXd _force_compensation_mat;
																	  //Threading setup
	std::queue<std::string> _forceSenseQueue, _tempImuQueue,_forceXYZQueue; //Two queues that the thread pushes shell sensor readings to, depends on line read
	std::mutex _sensorReadingMutex;
	std::condition_variable _ForceSenseReadingArrived, _tempImuReadingArrived;
	std::shared_ptr<std::thread> _senseReadingThread;
	std::atomic<bool> _runThread = true;
	std::string _forceSenseTag = FORCESENSOR_TAG;
	std::string _tempImuTag = TEMPIMU_TAG;

};

