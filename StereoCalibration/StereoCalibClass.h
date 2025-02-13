#pragma once
//Includes
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>

#include "RealSense.h"

const int MINIMUM_NUM_CHECKERBOARDS = 10;

class StereoCalibClass
{
public:
	// Constructor: Set board dimensions and square size
	StereoCalibClass(int board_width, int board_height, float square_size, std::string chessboard_filepath);
	
	//Destructor
	~StereoCalibClass();

	//Adds a pair of calibration images
	bool addCalibrationImages(const cv::Mat& left_image, const cv::Mat& right_image);

	// Perform the stereo calibration
	bool calibrate();

	//Save the calibration
	void saveCalibration(const std::string& filename);


private:
	int _board_width, _board_height;
	float _square_size;
	std::string _chessboard_filepath;
	int _chessboard_num;

	//Vector of vectors of corresponding left/right images points
	std::vector<std::vector<cv::Point2f>> _left_image_points, _right_image_points;

	//Vector of vectors of the object points (3D chessboard points)
	std::vector<std::vector<cv::Point3f>> _object_points;

	//Camera Calibration coefficients
	cv::Mat _left_camera_mat, _left_dist, _right_camera_mat, _right_dist;
	cv::Mat _R, _T, _E, _F;
	double _calibration_error;


	void prepareObjectPoints(); //Depending on the size of the left_image_points, we create vector of vector of checkerboard points
};

