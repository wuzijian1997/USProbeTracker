# magft
Magnetic field-based force/torque sensor firmware

### Project Description
The project contains the firmware for the ESP32-WROOM-32E mounted on the Sensor Power Control Module which also contains an IMU, and interfaces with 6 Sensor Modules (the Power Control and Sensor Modules are custom PCBs designed by David Black). This is the firmware design for a 6DOF force sensor system mounted on an ultrasound probe which will be used as a part of a novel teleultrasound system.\
We continuously:
- Take data from 6 Sensor Modules (MEASURE_TASK)
- Take gyroscope, accelerometer, temperature data from IMU (IMU_TASK)
- Send data via UART or UDP (COMM_TASK)


### Installation
Installation instructions for ESP-IDF are on the Espressif website: 
```
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
```
Note: I had issues with installing through VS Code so I did manual installation (for Mac).

Clone MAGFT repository into ~/esp

### Running the Project
```
. $HOME/esp/esp-idf/export.sh
```
To build/flash the project, use the UDP or UART servers, or update configurations you need to be inside the MAGFT folder:
```
cd ~/esp/magft
```
Everytime you open a new terminal that needs ESP-IDF you need to set-up the environment variables:

#### Configure ESP32
sdkconfig.defaults contains all the changes to configuration from the default values.

Any changes to config are done in menuconfig:
```
idf.py menuconfig
```
Any changes in menuconfig should update the automatically generated sdkconfig file as well as your sdkconfig.defaults file.

#### Running project on ESP-IDF
Build the project:
```
idf.py build
```
To run the project (with monitor):
```
idf.py -p /dev/cu.usbserial-0001 flash monitor
```
If you get CMake errors when you try to flash the project, hold down EN and BOOT buttons and release before flashing the project.

#### Starting UDP Server
Run udpServer.py in new terminal after ESP32 flash is complete:
```
python udpServer.py
```

#### Update WIFI Network
In uartServer.py:
- Change wifiSSID and wifiPass to network of choice
- Set update = TRUE
Run uartServer.py in new terminal:
```
python uartServer.py
```

### Structure
app_main.c contains the main function. This simply initializes the communication interfaces, starts 3 FreeRTOS tasks and contains the task functions for the 3 tasks:

Task 0: (COMM_TASK) Runs on CPU core 0 and constantly tries to pop elements from the queues to form packets containing recent measures from all the sensors and IMU. The resulting packet is either sent over WiFi using UDP or over UART, depending on the SEND_MODE flag. This task is implemented in magft_wifi.h and makes use of the UDP code in wifiudp.h.

Task 1: (MEASURE_TASK) Runs on CPU Core 0 and constantly polls the Hall effect sensors over SPI, storing the readings in a FIFO queue with at most 10 elements. When the queue is full, old elements are discarded and replace with new ones. This task is implemented in app_main.c and makes use of code in magft_spi.h.

Task 2: (IMU_TASK) Runs on CPU Core 1 and polls the IMU, also adding the temperature, gyroscope, and accelerometer readings to the same kind of queue. This task is implemented in app_main.c and makes use of IMU code in magft_imu.h.

### Files
app_main.c: Described above. Contains the main program  
magft_imu.h: I2C communication with the IMU  
magft_spi.h: SPI communication with the sensors  
magft_wifi.h: Connecting to wifi, running the COMM_TASK, and sending over UDP  
wifiudp.h: UDP wireless communication protocol to send data to host PC / HoloLens / something  
magft_sync.h: The queues and synchronization of data from all sensors into one packet  
magft_pindef.h: Definitions for a lot of the constants, including pin numbers on the ESP32

### Synchronization of Tasks
COMM_TASK runs at a rate of ~600-700 times/second\
MEASURE_TASK runs at a rate of ~2000 times/second\
IMU_TASK runs at a rate of ~250 times/second

We run MEASURE_TASK and COMM_TASK on the same core using vTaskSuspend and vTaskResume such that COMM_TASK is only ever called after MEASURE_TASK has obtained data for COMM_TASK to send. This ensures that evry time COMM_TASK fetches data (magft_sync.h), there is data available to send. Running MEASURE_TASK immediately after COMM_TASK acts as a "pause" between sending packets (COMM_TASK runs slower without a "pause" between packets). This is preferable to using vTaskDelay because the minimum vTaskDelay is 1ms which would add unnessecary time between sent packets. 

IMU_TASK is by far the slowest (uses I2C which is significantly slower than SPI) and runs pinned to its own core. 


### Speeding up Tasks
Any configuration mentioned below are already saved to sdkconfig.defaults.

#### MEASURE_TASK
Things that affect speed of reading from SPI:

**CPU frequency = 240MHz (maximum)**\
The greater the CPU frequency the greater the speed.\
Changed in: menuconfig > Component Config > ESP Systems Settings > CPU Frequency

**SPI clock frequency = 5000000Hz**\
The greater the SPI clock frequency the greater the speed.\
DO NOT USE frequencies > 5000000 because the data is may not be good.\
Changed in: "magft_spi.h" file in spi_device_interface_config_t.clock_speed_hz

**Using polling transactions** \
Polling transactions ~10X faster than interrupt transactions.

**Other things that were tested**\
The following things were tested but did not noticeably affect SPI reading speed:
- Flash Frequency
- Flash Mode

### IMU_TASK
Things that affect speed of reading from I2C:

**FreeRTOS Tick Rate = 1000Hz (maximum)**\
The greater the tick rate, the greater the speed of reading from I2C.\
Changing tick rate from 100 to 1000 resulted in 10X speed increase.\
Changed in: menuconfig > Component Config > FreeRTOS > Kernel > configTICK_RATE_HZ

**Using IMU FIFO**\
Improve speed by decreasing the number of times we write/read to I2C device.\
Using FIFO instead of individual registers reduces the number of read/write function calls from 15 to 3.

### COMM_TASK
**Enable LWIP IRAM optimization = y**\
5X to 10X faster when we enable.

**Other things that were tested**\
The following things were tested but did not noticeably affect UDP transfer speed:
- RX buffer size
- TX buffer size

### magft_sync.h
We control the flow of data using queues and functions that allow us to save and fetch samples from these queues. Each queue transaction takes a set amount of time. We optimize for speed by limiting the number of queue transactions we need to save and fetch samples. 
After changes, magft_sync.h functions are very fast and will not be a bottleneck for any tasks that use them.

Note: Updating FreeRTOS Tick Rate from 100 to 1000 caused 10X speed increase (like for IMU_TASK).

### Potential Improvements / Updates
- Transfer some code to IRAM and see if it increases overall speed
- Change IMU connection to ESP32 from I2C connection to SPI (SPI is significantly faster) on the Power Control Module. Although VSPI and HSPI are already in use, SPI1 can be configured to act as a third SPI driver 