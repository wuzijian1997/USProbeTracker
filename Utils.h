#pragma once
#include "IRSegmentation.h" //Has a bunch of includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


//************Force Calculation Helper Methods*************
//Helper methods
std::string calculateForceVals(std::string& force_string, Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset);
Eigen::VectorXd forcestringToForceVector(std::string& raw_force_string);
std::string eigenForceToStringForce(Eigen::Vector3d& force_xyz);
Eigen::MatrixXd readCSVToEigenMatrix(const std::string& file_path, int rows, int cols);
std::string EigenMatrixToString(Eigen::MatrixXd& matrix);
std::string getDatetimeWithMilliseconds();
//Transforms a point from the left camera to the right camera
cv::Rect transformROIToRight(const cv::Rect& left_ROI, const cv::Mat& left_mat,
	const cv::Mat& right_mat, const cv::Mat& R, const cv::Mat& T);

cv::Point2f transformPointToRight(cv::Point2f leftPoint, float assumedDepth,
	const cv::Mat& cameraMatrixL, const cv::Mat& cameraMatrixR,
	const cv::Mat& R, const cv::Mat& T);