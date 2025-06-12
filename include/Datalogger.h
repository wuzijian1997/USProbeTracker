#pragma once
#include "AsyncVideoWriter.hpp"
#include "IRSegmentation.h" //Has a bunch of includes
#include "RealSense.h"
#include <fstream>

// Constants for cropping the realsense frames to only the ROI (not doing this now)
const bool IS_ROI_CROP = false;
const int CROP_WIDTH = REALSENSE_WIDTH; // Set to no cropping (same as realsense width)
const int CROP_HEIGHT = REALSENSE_HEIGHT;
const int CROP_X = 0;
const int CROP_Y = 0;

// For file management
#include <filesystem>

namespace fs = std::filesystem;

// Header put into the csv file
const std::string SCANNING_CSV_HEADER
    = "System Time,Run Time (seconds),Depth Frame #,US Frame #,Landmark"
      "CAM_T_US:,T00,T01,T02,R00,R01,R02,R10,R11,R12,R20,R21,R22,FORCE_RAW:,fr0,fr1,fr2,fr3,fr4,"
      "fr5,"
      "fr6,fr7,fr8,fr9,fr10,fr11,FORCE_XYZ:,fx,fy,fz,IMU:,temp,g0,g1,g2,a0,a1,a2\n";

class Datalogger
{
public:
    // Inits the path, csv filename (scanning_datetime.csv), and csv header
    Datalogger(std::string root_path,
               std::string participant_directory,
               Eigen::MatrixXd &calib_mat,
               Eigen::Vector3d &zeroing_offset,
               Eigen::MatrixXd force_compensation_mat,
               std::string &left_camera_intrinsics,
               std::string &right_camera_intrinsics,
               std::string &R_string,
               std::string &T_string,
               std::string &E_string,
               std::string &F_string,
               double &depth_scale,
               int depth_width,
               int depth_height,
               double depth_fps,
               int us_width,
               int us_height,
               double us_fps,
               std::shared_ptr<IRSegmentation> ir_segmentation_object);
    ~Datalogger();

    void close();

    // Writes the data to the scan_datetime.csv
    void writeCSVRow(float &run_seconds,
                     int &depth_frame_num,
                     int &us_frame_num,
                     Eigen::Matrix4d &cam_T_us,
                     std::string &raw_force_string,
                     std::string &force_string_xyz,
                     std::string &temp_imu_string,
                     std::string &landmark_name);

    void writeDepthFrame(cv::Mat &frame);
    void writeIRFrames(cv::Mat &irLeft, cv::Mat &irRight);
    void writeColourFrame(cv::Mat &colourFrame);
    void writeUSFrame(const cv::Mat &us_frame);

private:
    // Class Vars
    std::ofstream _csv_file;
    std::string _data_path, _csv_filename;
    std::string _depth_file, _us_file, _irLeft_file, _irRight_file, _rgb_file;
    FILE *_depth_pipeout;
    FILE *_us_pipeout;

    // OpenCV Video Writing Vars
    cv::Rect _leftROI, _rightROI;
    cv::Mat _us_converted_frame;

    // Async video writer
    UBCRCL::AsyncVideoWriter usVidWriter_, depthVidWriter_, irLeftVidWriter_, irRightVidWriter_,
        camVidWriter_;
};