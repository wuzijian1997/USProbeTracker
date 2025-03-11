#pragma once
#include "IRSegmentation.h" //Has a bunch of includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


//************Force Calculation Helper Methods*************
//Helper methods
std::string calculateForceVals(std::string& force_string, Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset, Eigen::MatrixXd& force_compensation_mat);
Eigen::VectorXd forcestringToForceVector(std::string& raw_force_string);
std::string eigenForceToStringForce(Eigen::Vector3d& force_xyz);
Eigen::MatrixXd readCSVToEigenMatrix(const std::string& file_path, int rows, int cols);
std::string EigenMatrixToString(Eigen::MatrixXd& matrix);
std::string getDatetimeWithMilliseconds();
Eigen::Matrix3d convertCVMatToEigen_3b3(cv::Mat R_mat);
Eigen::Vector3d convertCVMatToEigen_1b3(cv::Mat T_mat);
std::string openCVMatToCSVString(const cv::Mat& mat);
cv::Mat cropMat(const cv::Mat& inputMat, cv::Rect cropRegion);
Eigen::Vector3d XYZforcestringToForceXYZVector(const std::string& xyz_force_string);
void triangulatePointsIterativeLinear(const cv::Mat& P1, const cv::Mat& P2, const std::vector<cv::Point2f>& left_matched, const std::vector<cv::Point2f>& right_matched, cv::Mat& points4D);
void triangulatePointsIterativeEigen(const cv::Mat& P1, const cv::Mat& P2, const std::vector<cv::Point2f>& left_matched, const std::vector<cv::Point2f>& right_matched, cv::Mat& points4D);