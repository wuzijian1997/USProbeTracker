#pragma once
#include "IRSegmentation.h" //Has a bunch of includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <windows.h>
#include <string>
#include <sstream>
#include <iomanip>

//************Force Calculation Helper Methods*************
//Helper methods
std::string calculateForceVals(std::string &force_string,
                               Eigen::MatrixXd &calib_mat,
                               Eigen::Vector3d &zeroing_offset,
                               Eigen::MatrixXd &force_compensation_mat);
Eigen::VectorXd forcestringToForceVector(std::string &raw_force_string);
std::string eigenForceToStringForce(Eigen::Vector3d &force_xyz);
Eigen::MatrixXd readCSVToEigenMatrix(const std::string &file_path, int rows, int cols);
std::string EigenMatrixToString(Eigen::MatrixXd &matrix);
std::string getDatetimeWithMilliseconds();
Eigen::Matrix3d convertCVMatToEigen_3b3(cv::Mat R_mat);
Eigen::Vector3d convertCVMatToEigen_1b3(cv::Mat T_mat);
std::string openCVMatToCSVString(const cv::Mat &mat);
cv::Mat cropMat(const cv::Mat &inputMat, cv::Rect cropRegion);
Eigen::Vector3d XYZforcestringToForceXYZVector(const std::string &xyz_force_string);
void triangulatePointsIterativeLinear(const cv::Mat &P1,
                                      const cv::Mat &P2,
                                      const std::vector<cv::Point2f> &left_matched,
                                      const std::vector<cv::Point2f> &right_matched,
                                      cv::Mat &points4D);
void triangulatePointsIterativeEigen(const cv::Mat &P1,
                                     const cv::Mat &P2,
                                     const std::vector<cv::Point2f> &left_matched,
                                     const std::vector<cv::Point2f> &right_matched,
                                     cv::Mat &points4D);


inline std::string getCurrentTimestampISO8601()
{
    SYSTEMTIME local, utc;
    GetLocalTime(&local);
    GetSystemTime(&utc);

    // Calculate UTC offset in minutes
    TIME_ZONE_INFORMATION tzi;
    GetTimeZoneInformation(&tzi);
    int offset = -(tzi.Bias);

    // Format hours and minutes for UTC offset
    char offsetStr[7];
    if (offset == 0) {
        strcpy(offsetStr, "Z");
    } else {
        sprintf(offsetStr, "%+03d:%02d", offset / 60, abs(offset % 60));
    }

    std::ostringstream oss;
    oss << local.wYear << "-"
        << (local.wMonth < 10 ? "0" : "") << local.wMonth << "-"
        << (local.wDay < 10 ? "0" : "") << local.wDay << "T"
        << (local.wHour < 10 ? "0" : "") << local.wHour << ":"
        << (local.wMinute < 10 ? "0" : "") << local.wMinute << ":"
        << (local.wSecond < 10 ? "0" : "") << local.wSecond << "."
        << std::setfill('0') << std::setw(3) << local.wMilliseconds << "Z" << offsetStr;
    return oss.str();
}