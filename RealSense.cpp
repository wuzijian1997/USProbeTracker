#include "RealSense.h"

RealSense::RealSense(int timeout)
	: _timeout(timeout), _isPipelineInit(false), _isRealSenseRunThread(false)
{}


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


    //Gets the depth scale
    try {
        auto depth_units = _realSense_context.query_devices().front().query_sensors().front().get_option(RS2_OPTION_DEPTH_UNITS);
        _depth_scale = (double)depth_units; //Sets the depth scale

    }
    catch (const rs2::error& e) {
        std::cout << "Error retrieving depth scale: " << e.what() << std::endl;
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

    //Gets the intrinsics of the left IR camera
    auto stream_profiles = pipeline_profile.get_streams();
    bool found = false;
    for (auto& sp : stream_profiles) {
        if (sp.stream_type() == RS2_STREAM_INFRARED && sp.stream_index() == 1) {
            auto video_stream_profile = sp.as<rs2::video_stream_profile>();
            _realSense_intrinsics_leftIR = video_stream_profile.get_intrinsics();
            found = true;
            break;
        }
    }

    if (!found) {
        std::cout << "Could not find the left infrared camera stream profile." << std::endl;
        return EXIT_FAILURE;
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

    //Returns true if all initialization worked
    return true;
}