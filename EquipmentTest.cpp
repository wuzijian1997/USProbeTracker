// ObstetricUS_Project.cpp : This file contains a test script to ensure the GPU and Intel RealSense Software is working
//
/*

//C++ Includes
#include <iostream>
#include <string>

//OpenCV Includes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

//RealSense Includes
#include <librealsense2/rs.hpp>

int main()
{
    //Checking OpenCV Version
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Major version: " << CV_MAJOR_VERSION << std::endl;
    std::cout << "Minor Version: " << CV_MINOR_VERSION << std::endl;
    std::cout << "************************************" << std::endl;

    //*************************Testing OpenCV*************************
    //Read in image of goat
    std::string imDir = "Resources\\GoatPicture.jpg";
    cv::Mat inputImage = cv::imread(imDir, cv::IMREAD_UNCHANGED);

    //Calculate new width and height based on scale factor
    double scalingFactor = 0.25;
    int newWidth = static_cast<int>(inputImage.cols * scalingFactor);
    int newHeight = static_cast<int>(inputImage.rows * scalingFactor);

    //Resize the image
    cv::resize(inputImage, inputImage, cv::Size(newWidth, newHeight));

    // Show the image
    cv::imshow("Goat Image", inputImage);
    cv::waitKey();

    //Create grayImage and show
    cv::Mat grayImage;
    cv::cvtColor(inputImage, grayImage, cv::COLOR_BGR2GRAY);
    cv::imshow("Gray Goat", grayImage);
    cv::waitKey();

    std::cout << "Showed CPU-Processed Images" << std::endl;
    std::cout << "************************************" << std::endl;

    //**********Testing GPU Availability and Functions*********************
    int cudaEnabDevCount = cv::cuda::getCudaEnabledDeviceCount();
    bool isCuda = true;
    if (cudaEnabDevCount){
        std::cout << "Number of available CUDA device(s): " << cudaEnabDevCount << std::endl;
    }
    else {
        std::cout << "You don't have any available CUDA device(s), please set this up" << std::endl;
        isCuda = false;
    }

    //More CUDA details
    if (isCuda)
    {
        cv::cuda::DeviceInfo cudaDeviceInfo;
        bool devCompat = false;
        std::cout << "List of all compatable CUDA device(s):" << std::endl;
        for (int devId = 0; devId < cudaEnabDevCount; ++devId)
        {
            cudaDeviceInfo = cv::cuda::DeviceInfo::DeviceInfo(devId);
            devCompat = cudaDeviceInfo.isCompatible();
            if (devCompat) {
                std::cout << "Compatable CUDA Device "<<std::endl;
                cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
            }
        }
        std::cout << "************************************" << std::endl;

       
        if (devCompat)
        {
            //Found a compatable device
            cv::cuda::GpuMat inputImageGpu, imageGrayGpu;
            inputImageGpu.upload(inputImage); //Adds colour image to gpu object for GPU processing
            cv::cuda::cvtColor(inputImageGpu, imageGrayGpu, cv::COLOR_BGR2GRAY);
            cv::Mat imageColourCpu, imageGrayCpu;

            //Move images from GPU to CPU
            inputImageGpu.download(imageColourCpu);
            imageGrayGpu.download(imageGrayCpu);

            // Show the images
            cv::imshow("Colour Goat Image GPU", imageColourCpu);
            cv::waitKey();

            //Create grayImage and show
            cv::imshow("Gray Goat Image GPU", imageGrayCpu);
            cv::waitKey();

            cv::destroyAllWindows();

            std::cout << "Showed GPU-Processed Images" << std::endl;
            std::cout << "************************************" << std::endl;

        }
    }


    //*************************Check the RealSense Camera*****************************
    std::cout << "Testing RealSense Camera" << std::endl;
    int width = 848;
    int height = 480;
    int fps = 60;
    rs2::frameset framset;
    rs2::frame ir_frame_left, ir_frame_right;
    cv::Mat ir_mat_left, ir_mat_right;

    //Create Configuration For Non-Default Camera Setup
    rs2::config config;
    config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    config.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);

    //Starting Device
    rs2::pipeline pipeline; //Setup a pipeline which abstracts the device
    rs2::pipeline_profile pipeline_profile = pipeline.start(config);

    //Setting Laser Power
    rs2::device selected_device = pipeline_profile.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Enable or disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        //auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER,150.f); // Set  power
    }
 
    while (1) //Runs until we quit the window
    {
        framset = pipeline.wait_for_frames();

        //Get left and right infrared frames
        ir_frame_left = framset.get_infrared_frame(1);
        ir_frame_right = framset.get_infrared_frame(2);

        //Gets the depth frame
        rs2::depth_frame depth_frame = framset.get_depth_frame();

        //Converts IR to OpenCV Mat:
        ir_mat_left = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
        ir_mat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

        //Converts Depth to OpenCV Mat:

        //cv::Mat depth_mat(cv::Size(width, height), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        float width = depth_frame.get_width();
        float height = depth_frame.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth_frame.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";

        //Shows the OpenCV image
        cv::imshow("Realsense IR Left", ir_mat_left);
        cv::imshow("Realsense IR Right", ir_mat_right);
        //cv::imshow("Realsense Depth Image", depth_mat);
        char c = cv::waitKey(1);
        if (c == 'q')
            break;

        
    }

    std::cout << "Showed RealSense Frames" << std::endl;
    std::cout << "************************************" << std::endl;

    cv::destroyAllWindows();


}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

*/
