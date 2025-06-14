#include "ShellSensorReader.h"

ShellSensorReader::ShellSensorReader(const std::string &portName,
                                     DWORD baudRate,
                                     int timeout,
                                     const Eigen::MatrixXd &force_calibration_mat,
                                     const Eigen::Vector3d &force_zeroing_offset,
                                     const Eigen::MatrixXd &force_compensation_mat)
    : _timeout(timeout)
    , _force_zeroing_offset(force_zeroing_offset)
    , _portName(portName)
    , _baudRate(baudRate)
    , _force_calibration_mat(force_calibration_mat)
    , _force_compensation_mat(force_compensation_mat)
{
    enableForceSensor = ConfigInstance::GetInstance()->core().enableForceSensor;
}

ShellSensorReader::~ShellSensorReader()
{
    // Closes force sensor threading things
    {
        std::lock_guard<std::mutex> lock(_sensorReadingMutex);
        _runThread = false;
    }

    if (_senseReadingThread && _senseReadingThread->joinable()) {
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

    // Closes the serial port
    if (_isSerialPortOpen) {
        // If the serial port is open we close it
        CloseHandle(_serialHandle);
    }
}

// Configures the serial handle, serial interface settings, and starts the threads
bool ShellSensorReader::initialize()
{
    if (!enableForceSensor) {
        spdlog::info("Force sensor reading is disabled in the config file.");
        return true;
    }

    _serialHandle = CreateFile(_portName.c_str(),
                               GENERIC_READ | GENERIC_WRITE,
                               0,
                               nullptr,
                               OPEN_EXISTING,
                               0,
                               nullptr);

    // Checks that the serial port is opened
    if (_serialHandle == INVALID_HANDLE_VALUE) {
        spdlog::error("Error Opening Serial Port");
        return false;
    }

    _isSerialPortOpen = configurePort(); // Configures serial port settings and checks if it is open

    if (_isSerialPortOpen) {
        // Starts the reading threads

        // Starts Sensor Reading Thread
        _runThread = true;
        _senseReadingThread = std::make_shared<std::thread>([this]() { readLines(); });
    }

    return _isSerialPortOpen;
}

bool ShellSensorReader::configurePort()
{
    DCB serialParams = {0};
    serialParams.DCBlength = sizeof(serialParams);

    if (!GetCommState(_serialHandle, &serialParams)) {
        spdlog::error("Error Getting Serial Port State");
        return false;
    }

    // Sets serial interface parameters
    serialParams.BaudRate = _baudRate;
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = NOPARITY;
    if (!SetCommState(_serialHandle, &serialParams)) {
        spdlog::error("Error Setting Serial Port State");
        return false;
    }

    // Set the timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = SHELLSENSOR_TIMEOUT;
    timeouts.ReadTotalTimeoutConstant = SHELLSENSOR_TIMEOUT;
    timeouts.ReadTotalTimeoutMultiplier = SHELLSENSOR_TIMEOUT;
    if (!SetCommTimeouts(_serialHandle, &timeouts)) {
        spdlog::error("Error Setting Serial Port Timeouts");
        return false;
    }
    return true;
}

void ShellSensorReader::readLines()
{
    if (!_isSerialPortOpen) {
        spdlog::error("Serial Port to Shell Not Open");
        // Ends thread process if serial port is not connected
        return;
    }

    char tempChar;
    DWORD bytesRead;
    std::string _buffer; // Buffer of characters read in on serial port

    // Vars for timeout checking
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point curr_time = std::chrono::steady_clock::now();

    while (_runThread) {
        // Read it into bytesRead, and then check if it is greater than 0
        if (ReadFile(_serialHandle, &tempChar, 1, &bytesRead, nullptr) && bytesRead > 0) {
            last_time = std::chrono::steady_clock::now(); // Updates last successful read time
            // New line character is met, we clear the line buffer, check if it is a force line, and
            // append to the queue if it is
            if (tempChar == '#') {
                std::string line = _buffer;
                _buffer.clear(); // Clears the buffer

                // Check if it is line with name _forceSenseTag, if it is push to the force sensor
                // queue Checking for force line
                if (checkLineTag(_forceSenseTag, line)) {
                    // Strips the tag from the string
                    line.erase(0, _forceSenseTag.length());
                    appendForceString(line);
                } else if (checkLineTag(_tempImuTag, line)) {
                    // Strips the tag from the string
                    line.erase(0, _tempImuTag.length());
                    appendTempIMUString(line);
                } else {
                    // Generic case 12 force + 7 IMU values concat together
                    auto &[f_str, imu_str] = splitForceImuReadings(line);
                    appendForceString(f_str);
                    appendTempIMUString(imu_str);
                }
            } else if (tempChar != '\r') {
                // ignore carriage return
                _buffer += tempChar; // appends char to the buffer
            }
        } else {
            curr_time = std::chrono::steady_clock::now(); // updates curr time of unsuccessful read
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        curr_time - last_time)
                                        .count();
            if (static_cast<int>(elapsed_ms) > SHELLSENSOR_TIMEOUT) {
                spdlog::warn("Serial Timout Triggered");
            }
        }
    }
}

bool ShellSensorReader::checkLineTag(std::string &lineTag, std::string &serialLine)
{
    // Returns true if the tag is in the serial line (force sensor tag is "FSN:" or the temp/IMU is "TGA:")
    return serialLine.find(lineTag) != std::string::npos;
}

// Methods to get the force and Temp/IMU lines from the two threading functions

void ShellSensorReader::getForceString(std::string &force_string, std::string &force_xyz_string)
{
    std::unique_lock<std::mutex> lock{_sensorReadingMutex};
    // Returns NaNs if waiting longer than _timeout value
    if (!_ForceSenseReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() {
            return (!_forceSenseQueue.empty()) && (!_forceXYZQueue.empty());
        })) {
        // We had a timeout event, return with force_string set to 12 NaN's
        force_string = TWELVE_NaNs;
        lock.unlock();
        return;
    }

    if ((!_forceSenseQueue.empty()) && (!_forceXYZQueue.empty())) {
        // Grab frame if not empty queue
        force_string = _forceSenseQueue.front();
        _forceSenseQueue.pop();
        force_xyz_string = _forceXYZQueue.front();
        _forceXYZQueue.pop();
    } else {
        force_string = TWELVE_NaNs;
        force_xyz_string = "NaN,NaN,NaN";
    }
    lock.unlock();
}

void ShellSensorReader::getTempIMUString(std::string &temp_imu_string)
{
    std::unique_lock<std::mutex> lock{_sensorReadingMutex};
    // Returns NaNs if waiting longer than _timeout value
    if (!_tempImuReadingArrived.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() {
            return !_tempImuQueue.empty();
        })) {
        // We had a timeout event, return with string set to 6 NaN's
        temp_imu_string = SIX_NaNs;
        lock.unlock();
        return;
    }

    if (!_tempImuQueue.empty()) {
        // Grab frame if not empty queue
        temp_imu_string = _tempImuQueue.front();
        _tempImuQueue.pop();
    } else {
        temp_imu_string = SIX_NaNs;
    }
    lock.unlock();
}