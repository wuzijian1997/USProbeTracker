#pragma once
#include "SetupAndSegment.h" //Has a bunch of includes
#include <Windows.h>
#include <iostream>
#include <condition_variable>
#include <queue>

const DWORD SHELLSENSOR_BAUDRATE = CBR_115200;
const std::wstring SHELLSENSOR_PORTNAME = L"COM9";
const int SHELLSENSOR_TIMEOUT = 100; //Timeout that we will try to read for in ms

class ShellSensorReader
{
public:
	//Inits port name and baud rate vars
	int _timeout = 20;
	ShellSensorReader(const std::wstring& portName, DWORD baudRate,int timeout);
	~ShellSensorReader();

	bool initialize(); //Inits the port connection and serial interface setup
	void readLines(); //Reads a line up until "\n" 
	
private:
	std::wstring _portName;
	HANDLE _serialHandle;
	DWORD _baudRate;
	bool _isSerialPortOpen = false;; //Tracks whether the serial port "_portName" is open
	std::string _buffer; //Buffer of entries in serial 

	bool configurePort(); //Sets up port parameters (e.g. baud rate, byte size etc.)

	//Threading setup
	std::queue<std::string> _forceSenseQueue, _tempImuQueue; //Two queues that the thread pushes shell sensor readings to
	std::mutex _shellSenseMutex;
	std::condition_variable _readingArrived;
	std::shared_ptr<std::thread> _shellSense_Thread;
	std::atomic<bool> _runSense_Thread = true;

};

