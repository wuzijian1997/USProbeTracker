#pragma once
#include "IRSegmentation.h" //Has a bunch of includes
#include "configs.hpp"
#include <Windows.h>
#include <condition_variable>
#include <queue>

const DWORD SHELLSENSOR_BAUDRATE = CBR_115200;
const int SHELLSENSOR_TIMEOUT = 100; // Timeout that we will try to read for in ms
const std::string FORCESENSOR_TAG = "FSN:";
const std::string TEMPIMU_TAG = "TGA:";
const std::string TWELVE_NaNs = "NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN";
const std::string SIX_NaNs = "NaN,NaN,NaN,NaN,NaN,NaN";

class ShellSensorReader
{
public:
    // Inits port name and baud rate vars
    int _timeout = 20; // Timeout is how long we wait for a thread
    ShellSensorReader(const std::string &portName,
                      DWORD baudRate,
                      int timeout,
                      const Eigen::MatrixXd &force_calibration_mat,
                      const Eigen::Vector3d &force_zeroing_offset,
                      const Eigen::MatrixXd &force_compensation_mat);
    ~ShellSensorReader();

    bool initialize(); // Inits the port connection and serial interface setup

    void getForceString(std::string &force_string, std::string &force_xyz_string);
    void getTempIMUString(std::string &temp_imu_string);

    Eigen::Vector3d _force_zeroing_offset; // We manually change this during zeroing

    void appendForceString(std::string &str)
    {
        std::lock_guard<std::mutex> l{_sensorReadingMutex};
        // Clears the queue before pushing the newest reading (we only have one reading in the queue
        // at a time)
        while (!_forceSenseQueue.empty()) {
            _forceSenseQueue.pop();
        }
        _forceSenseQueue.push(str);

        // Calculates the xyz from raw force
        // Holds the converted raw force values to xyz force
        const std::string force_string_xyz = calculateForceVals(str,
                                                                _force_calibration_mat,
                                                                _force_zeroing_offset,
                                                                _force_compensation_mat);
        while (!_forceXYZQueue.empty()) {
            _forceXYZQueue.pop();
        }
        _forceXYZQueue.push(force_string_xyz);

        // Notifies the recipient
        _ForceSenseReadingArrived.notify_one();
    }

    void appendTempIMUString(const std::string &str)
    {
        // Writes force reading to the queue
        {
            std::lock_guard<std::mutex> l{_sensorReadingMutex};
            // Clears the queue before pushing the newest reading (we only have one reading in the
            // queue at a time)
            while (!_tempImuQueue.empty()) {
                _tempImuQueue.pop();
            }
            _tempImuQueue.push(str);
            // Notifies the recipient
        }
        _tempImuReadingArrived.notify_one();
    }

private:
    std::string _portName;
    HANDLE _serialHandle{};
    DWORD _baudRate;
    bool _isSerialPortOpen = false; // Tracks whether the serial port "_portName" is open

    bool configurePort(); // Sets up port parameters (e.g. baud rate, byte size etc.)
    bool checkLineTag(std::string &lineTag, std::string &serialLine);
    // Checks whether the line starts with "FSN:" or "TGA:"
    void readLines(); // Reads a lines of force sensors up until "\n", runs on one thread

    // Values for converting raw force values to xyz force vals
    Eigen::MatrixXd _force_calibration_mat;
    Eigen::MatrixXd _force_compensation_mat;
    // Threading setup
    std::queue<std::string> _forceSenseQueue, _tempImuQueue, _forceXYZQueue;
    // Two queues that the thread pushes shell sensor readings to, depends on line read
    std::mutex _sensorReadingMutex;
    std::condition_variable _ForceSenseReadingArrived, _tempImuReadingArrived;
    std::shared_ptr<std::thread> _senseReadingThread;
    std::atomic<bool> _runThread = true;
    std::string _forceSenseTag = FORCESENSOR_TAG;
    std::string _tempImuTag = TEMPIMU_TAG;

    bool enableForceSensor{false}; // Config entry to enable force sensor reading
};