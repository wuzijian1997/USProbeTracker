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

class StereoCalibClass
{
public:
	// Constructor: Set board dimensions and square size
	StereoCalibClass(int board_width, int board_height, float square_size,std::string chessboard_filepath);
	
	//Destructor
	~StereoCalibClass();

	//Adds a pair of calibration images
	bool addCalibrationImages(const cv::Mat& left_image, const cv::Mat& right_image);


private:
	int _board_width, _board_height;
	float _square_size;
	std::string _chessboard_filepath;
	int _chessboard_num;

	//Vector of vectors of corresponding left/right images points
	std::vector<std::vector<cv::Point2f>> _left_image_points, _right_image_points;
};

