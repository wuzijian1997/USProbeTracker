#pragma once
#include "SetupAndSegment.h" //Has a bunch of includes
#include <Windows.h>
#include <iostream>
#include <condition_variable>
#include <queue>

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
	ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout);
	~ShellSensorReader();

	bool initialize(); //Inits the port connection and serial interface setup
	
	void getForceString(std::string& force_string);
	void getTempIMUString(std::string& temp_imu_string);


private:
	std::wstring _portName;
	HANDLE _serialHandle;
	DWORD _baudRate;
	bool _isSerialPortOpen = false;; //Tracks whether the serial port "_portName" is open

	bool configurePort(); //Sets up port parameters (e.g. baud rate, byte size etc.)
	bool checkLineTag(std::string& lineTag, std::string& serialLine); //Checks whether the line starts with "FSN:" or "TGA:"
	void readLines(std::queue<std::string>& sensor_queue, std::string& lineTag, std::mutex& mutex_ptr, std::condition_variable& reading_arrived, std::atomic<bool>& run_thread); //Reads a lines of force sensors up until "\n", runs on one thread

																	  //Threading setup
	std::queue<std::string> _forceSenseQueue, _tempImuQueue; //Two queues that the thread pushes shell sensor readings to
	std::mutex _forceSenseMutex,_tempImuMutex;
	std::condition_variable _ForceSenseReadingArrived, _tempImuReadingArrived;
	std::shared_ptr<std::thread> _forceSense_Thread, _tempImu_Thread;
	std::atomic<bool> _runForceSense_Thread = true;
	std::atomic<bool> _runTempImu_Thread = true;
	std::string _forceSenseTag = FORCESENSOR_TAG;
	std::string _tempImuTag = TEMPIMU_TAG;

};

