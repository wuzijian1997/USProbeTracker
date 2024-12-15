#include "RealSense.h"

RealSense::RealSense(int timeout)
	: _timeout(timeout), _isPipelineInit(false), _isRealSenseRunThread(false)
{}

RealSense::~RealSense()
{
    stop();
}


//Inits the RealSense Camera
bool RealSense::RealSenseInit(int width, int height, int fps, float enable_laser, float laser_power, float gain,
    float filter_alpha,float filter_delta)
{
    std::cout << "************************************" << std::endl;
    std::cout << "Setting Up RealSense Device(s)" << std::endl;
    if (_realSense_context.query_devices().size() == 0)
    {
        std::cout << "No RealSense Devices Were Found" << std::endl;
        std::cout << "************************************" << std::endl;
        return false;
    }

    //Enabling the streams:
    try {
        _realSense_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
        _realSense_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
        _realSense_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    }
    catch (const rs2::error& e) {
        std::cout << "Error enabling RealSense streams: " << e.what() << std::endl;
        return false;
    }


    //Starting Device
    rs2::pipeline_profile pipeline_profile;
    try {
        pipeline_profile = _realSense_pipeline.start(_realSense_config);
        std::cout << "RealSense initialized successfully" << std::endl;
        std::cout << "************************************" << std::endl;
    }
    catch (const rs2::error& e) {
        std::cout << "Failed to initialize RealSense camera: " << e.what() << std::endl;
        std::cout << "************************************" << std::endl;
        return false;
    }

    //Gets the depth scale
    try {
        auto depth_units = _realSense_context.query_devices().front().query_sensors().front().get_option(RS2_OPTION_DEPTH_UNITS);
        _depth_scale = (double)depth_units; //Sets the depth scale

    }
    catch (const rs2::error& e) {
        std::cout << "Error retrieving depth scale: " << e.what() << std::endl;
        return false;
    }

    

    //Gets the factory set intrinsics of the left IR camera
    try {
        _realSense_intrinsics_leftIR = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>().get_intrinsics();
        _realSense_intrinsics_rightIR = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>().get_intrinsics();
    }
    catch (const rs2::error& e) {
        std::cout << "Error finding intrinsics: " << e.what() << std::endl;
        return false;
    }


    //Setting Laser Power
    try {
        rs2::device selected_device = pipeline_profile.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, enable_laser); // Enable or disable emitter
        }
        if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
        {
            // Query min and max values:
            //auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power); // Set  power
        }

        //Sets the gain of the IR images (sensors)
        auto sensors = selected_device.query_sensors();

        //Loops and checks if the sensor is the stereo module and if it supports gain
        for (auto& sensor : sensors)
        {
            if (sensor.supports(RS2_OPTION_GAIN) && sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("Stereo Module"))
            {
                sensor.set_option(RS2_OPTION_GAIN, static_cast<float>(gain));
                std::cout << "Set Camera Gain" << std::endl;
            }
        }


    }
    catch (const rs2::error& e) {
        std::cout << "Error setting laser power: " << e.what() << std::endl;
        return false;
    }



    //Aligns to left infrared stream
    try {
        initializeAlign(RS2_STREAM_INFRARED);
    }
    catch (const rs2::error& e) {
        std::cout << "Error initializing IR Frame alignment: " << e.what() << std::endl;
        return false;
    }

    //Configures the temporal depth filter for the findkeypoints function
    _temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, filter_alpha);
    _temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, filter_delta);

    _isPipelineInit = true;
    //Returns true if all initialization worked
    return true;
}

//Helper to initialze the allignment object to the left ir
void RealSense::initializeAlign(rs2_stream stream_Type) {
    _align_to_left_ir = std::make_unique<rs2::align>(stream_Type);
}

//Starts getting frames from the realsense camera via a thread
void RealSense::start()
{
    if (!_isPipelineInit) {
        std::cout << "Pipeline not initialized. Call RealSenseInit() first." << std::endl;
        return;
    }
    _isRealSenseRunThread = true;
    _realSenseThread = std::thread(&RealSense::frameProducer, this);
    //_realSenseThread= std::make_shared<std::thread>([this]() {frameProducer(); });
}

void RealSense::stop()
{
    //Closes realsense threading things
    {
        std::lock_guard<std::mutex> lock(_realSenseMutex);
        _isRealSenseRunThread = false;

    }

    if (_realSenseThread.joinable())
    {
        _realSenseThread.join();
    }
    {
        std::lock_guard<std::mutex> lock(_realSenseMutex);
        while (!_realSenseDataQueue.empty()) {
            _realSenseDataQueue.pop();
        }
    }


    if (_isPipelineInit)
    {
        _realSense_pipeline.stop();
    }
    _align_to_left_ir.reset();

}


void RealSense::frameProducer()
{
    int retry_count = 0;
    while (_isRealSenseRunThread)
    {
        try {
            rs2::frameset frameset;
            //frameset = _realSense_pipeline.wait_for_frames();
            /*if (!frameset) {
                std::cout << "Error: Failed to get frames from RealSense pipeline." << std::endl;
                continue;
            }*/

            //THis seems like the most robust method to handle timeouts
            if (!_realSense_pipeline.poll_for_frames(&frameset))
            {
                std::cout << "Error: Failed to get frames from RealSense pipeline." << std::endl;
                retry_count++;
                if (retry_count > REALSENSE_RETRY)
                {
                    ResetRealSensePipeline();
                    retry_count = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(REALSENSE_THREAD_DELAY));
                continue;
            }
            retry_count = 0;

            //Gets the aligned IR frames
            /*if (!_align_to_left_ir) {
                std::cout << "Error: Alignment object not initialized." << std::endl;
                continue;
            }*/
            rs2::frameset aligned_frameset = _align_to_left_ir->process(frameset);

            //Gets the realsense data object
            RealSenseData realsense_data;
            realsense_data.irLeftFrame = aligned_frameset.get_infrared_frame(1);
            realsense_data.irRightFrame = aligned_frameset.get_infrared_frame(2);

            if (!realsense_data.irLeftFrame) {
                std::cout << "Error: Failed to get left IR frame." << std::endl;
                continue;
            }

            realsense_data.depthFrame = aligned_frameset.get_depth_frame();

            if (!realsense_data.depthFrame) {
                std::cout << "Error: Failed to get depth frame." << std::endl;
                continue;
            }

            realsense_data.depthFrameFiltered = _temp_filter.process(realsense_data.depthFrame);

            //Updates the data queue
            //locks the mutex before writing to the queue and notifying


            {
                std::lock_guard<std::mutex> l{ _realSenseMutex };

                if (_realSenseDataQueue.size() > 10) {
                    _realSenseDataQueue.pop(); // Discard the oldest readings if the queue is too large
                }
                _realSenseDataQueue.push(realsense_data);

            }
            _realSenseFrameArrivedVar.notify_one();
            std::this_thread::sleep_for(std::chrono::milliseconds(REALSENSE_THREAD_DELAY));

        }
        catch (const rs2::error& e) {
            std::cout << "RealSense error: " << e.what() << std::endl;
            ResetRealSensePipeline();
        }
    }
}

bool RealSense::getRealSenseData(RealSenseData& realsense_data)
{
    //bool return_bool = false;
    std::unique_lock<std::mutex> lock{_realSenseMutex};
    //Returns empty frame if waiting longer than 20 ms (50 Hz)
    if (!_realSenseFrameArrivedVar.wait_for(lock, std::chrono::milliseconds(_timeout), [this]() { return !_realSenseDataQueue.empty(); }))
    {
        //We had a timeout event, return false
        //lock.unlock();
        std::cout << "Timeout Occured" << std::endl;
        return false;
    }

    realsense_data = _realSenseDataQueue.front();
    _realSenseDataQueue.pop();
    //lock.unlock();
    
    return true;
}

void RealSense::ResetRealSensePipeline() {
    try {       
        _realSense_pipeline.stop();
        _realSense_pipeline.start(_realSense_config); // Restart the pipeline
        std::cout << "RealSense pipeline reset successfully." << std::endl;
    }
    catch (const rs2::error& e) {
        std::cerr << "Failed to reset RealSense pipeline: " << e.what() << std::endl;
    }
}

