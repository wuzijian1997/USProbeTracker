#pragma once

//**********Includes**********
//C++ Includes
#include <iostream>
#include <string>
#include <stdio.h>

//RealSense Includes
#include <librealsense2/rs.hpp>

//Threading Includes
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <memory>



//**********RealSense Constants***************
const int REALSENSE_WIDTH = 848;
const int REALSENSE_HEIGHT = 480;
const int REALSENSE_FPS = 60;
const float ENABLE_REALSENSE_LASER = 1.0f; //1 if we want the laser on
const float REALSENSE_LASER_POWER = 50.0f;
const float REALSENSE_GAIN = 16.0f; //Gain of IR image, key for removing noise
const int REALSENSE_WAITFORFRAMES_DELAY = 1000; //The thread waits for 1000 ms for frames to arrive

//Depth Frame Filtering Constants
const float TEMPORAL_DEPTH_FILTER_ALPHA = 0.2f;
const float TEMPORAL_DEPTH_FILTER_DELTA = 17;

//Structure of data streamed from the realsense camera
struct RealSenseData
{
	rs2::frame irLeftFrame;
	rs2::frame irRightFrame;
	rs2::frame depth_frame;
	std::unique_ptr <std::vector<uint16_t>> depthFilteredPtr;
	std::unique_ptr <std::vector<uint16_t>> irLeftPtr;
};



class RealSense
{
public:

	//************Public Class Functions************

	//Initializes the class
	// @param timeout the amount of time we wait for realsense thread to update the data
	RealSense(int timeout);
	~RealSense();

	//Inits the realsense camera
	// @param width the width of desired realsense frame
	// @param height the height of desired realsense frame
	// @param fps the fps of desired realsense streaming
	// @param enable_laser toggles the laser on (1.0f for on)
	// @param laser_power sets the emitter power (50.0f is standard)
	// @param gain sets the gain of the left/right IR images
	// @param filter_alpha sets alpha of temporal depth filter used on depth vector passed to segmentation
	// @param filter_delta sets delta of temporal depth filter used on depth vector passed to segmentation
	bool RealSenseInit(int width, int height,int fps, float enable_laser, float laser_power,float gain,float filter_alpha,float filter_delta);

	//Starts threading and reading camera data
	void start();

	//Gets the realsense frame data
	bool getRealSenseData(RealSenseData& frame_data);

	//*************Public Class vars****************	
	rs2_intrinsics _realSense_intrinsics_leftIR; //Factory set intrinsics
	double _depth_scale; //depth scale (scales realsense depth to meters)

	bool _isPipelineInit=false;

private:

	//**********Private Class Functions***********
	//Helper method to align IR with depth
	void initializeAlign(rs2_stream stream_Type);
	//The threading function
	void frameProducer();

	//************Private Class Vars**************
	int _timeout; //Amount of time we wait for the thread to update a new frame before returning empty frames

	//Pipeline class vars
	rs2::config _realSense_config;
	rs2::pipeline _realSense_pipeline;
	rs2::context _realSense_context;
	std::unique_ptr<rs2::align> _align_to_left_ir;

	//Temporal Filter Applied to Depth before sent to findkeypoints worldframe
	rs2::temporal_filter _temp_filter;

	//Threading class vars
	
	std::queue<RealSenseData> _realSenseDataQueue;
	std::mutex _realSenseMutex;
	std::condition_variable _realSenseFrameArrivedVar;
	std::shared_ptr<std::thread> _realSenseThread;
	std::atomic<bool> _isRealSenseRunThread = false;

};

