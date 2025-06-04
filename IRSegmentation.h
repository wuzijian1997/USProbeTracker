#pragma once
#define NOMINMAX
//C++ Includes
#include <iostream>
#include <string>
#include <stdio.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include <opencv2/world.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
//RealSense Includes
#include <librealsense2/rs.hpp>

//Pose Tracking Includes
#include <eigen3/Eigen/Dense>
#include "log.h"
#include "framework.h"
#include <thread>
#include <mutex>

#include "RealSense.h"
#include "Utils.h"

//*********2D Image Crop Constants
const int IMAGE_X_MAXCROP = 848;
const int IMAGE_Y_MAXCROP = 480;
const int IMAGE_X_MINCROP = 0;
const int IMAGE_Y_MINCROP = 0;

//*********Contour Segmentation Constants**************
const int CONTOUR_BIN_THRESHOLD = 100; //222
const int COUNTOUR_MIN_AREA = 10;
const int COUNTOUR_MAX_AREA = 150;
const float CONTOUR_CONVEXITY = 0.68f;
const float COUNTOUR_CIRCULARITY = 0.54f;

//**********Blob Segmentation Constants****************
const int BLOB_MIN_THRESHOLD = 100; //215
const int BLOB_MAX_THRESHOLD = 255;
const int BLOB_MIN_AREA = 6;
const int BLOB_MAX_AREA = 60;
const float BLOB_MIN_CONVEXITY = 0.68f;
const int BLOB_MIN_DSTANCE_BETWEEN = 2;

//*******Find Keypoints Constants***************
const float KEYPOINTS_MAX_INTENSITY = 1000.0f;
const double NEAR_CLIP = 0.25f;
const double FAR_CLIP = 1.3f; //Average Arm Span is 65 cm, we want the tracker to be about the same distance from eyes to hand
const double EPIPOLAR_MATCH_Y_THRESHOLD = 4.0f; //Difference in y-direction for two points to be in same epipolar line for stereo triangulation
const double EPIPOLAR_MATCH_X_THRESHOLD = 75.0f; //Difference in x-direction for two points to be the same in matching epipolar points
const int UNDISTORTION_MAX_COUNT = 40; //Number of times undistortPoints() iterates
const double UNDISTORTION_EPS = 1e-12; //Threshold of undistortion accuracy (either this or the max iterations are met)

class IRSegmentation
{
public:
	//Markers in IR image detection structure
	struct IrDetection {
		std::vector<Eigen::Vector3d> points;
		std::vector<Eigen::Vector2i> imCoords;
		std::vector<float> markerDiameters;
	};

	enum class DetectionMode {
		Contour,
		Blob
	};
	enum class LogLevel {
		Silent,
		Verbose,
		VeryVerbose
	};		

	/// Create an IrTracker object.
	/// @param width the width of the input image in pixels
	/// @param height the height of the input image in pixels
	IRSegmentation(int width, int height, double depth_scale, LogLevel logLevel, bool show_clip_area,
		cv::Mat left_camera_mat, cv::Mat left_dist, cv::Mat right_camera_mat, cv::Mat right_dist,
		cv::Mat R, cv::Mat T, cv::Mat E, cv::Mat F);
	//~IRSegmentation();

	/// Set the rectangular region of const interest in the image in which to search for the markers.
	/// @param xMax the column of the left edge of the ROI
	/// @param xMax the column of the right edge of the ROI
	/// @param yMin the row of the top edge of the ROI
	/// @param yMax the row of the bottom edge of the ROI
	Eigen::Vector4i setROI(int xMin, int xMax, int yMin, int yMax);
	
	/// Set the settings of the camera, including the region of interest and near/far clipping planes
	/// @param roiUpperRow the row index of the upper limit of the camera's region of interest
	/// @param roiLowerRow the row index of the lower limit of the camera's region of interest
	/// @param roiLeftCol the column index of the left limit of the camera's region of interest
	/// @param roiRightCol the column index of the right limit of the camera's region of interest
	/// @param nearClipPlane the depth value of the near clipping plane
	/// @param farClipPlane the depth value of the far clipping plane
	void setCameraBoundaries(int roiUpperRow, int roiLowerRow, int roiLeftCol, int roiRightCol, double nearClipPlane, double farClipPlane);

	/// Register a function which is used to compute the camera intrinsics. It should take a point in the image coordinates and map it to the camera unit plane
	/// @param uv the (column, row) coordinates of a point in the image, in pixels
	/// @param xy This array is filled in when the function is called. The point is at position depth * (xy[0], xy[1], 1) in the camera frame
	void setCameraIntrinsics(std::function<void(const std::array<double, 2>&, std::array<double, 2>&)> intrinsics);
	
	/// Use contour or blob detection. Blob detection is much slower but has fewer outliers
	void setDetectionMode(DetectionMode mode);

	LogLevel getLogLevel();

	double m_depth_scale; //depth scale that we use
	
	//Finds marker points in the world frame
	std::shared_ptr<IrDetection> findKeypointsWorldFrame(std::unique_ptr<std::vector<uint8_t>> irImLeft, std::unique_ptr<std::vector<uint8_t>> irImRight);
	
	//*************Camera Parameters***************
	//Camera Matrices and distortion coefficients
	cv::Mat _left_camera_mat, _left_dist, _right_camera_mat, _right_dist;
	//Individual camera params
	double _fxL, _fyL, _cxL, _cyL;
	double _fxR, _fyR, _cxR, _cyR;

	//Rotation (R) and translation (T) between left=> right cameras
	//Essential matrix (E) and fundamental matrix (F) of stereo calibration
	cv::Mat _R, _T,_E,_F;
	cv::Mat _R_left,_R_left_inv, _R_right; //Transforms points from unrectified camera coordinate system to rectified coordinate system for each camera
	cv::Mat _P_left, _P_right; //Transforms points in rectified camera coordinate system to camera's recitified image

	// R and T in eigen
	Eigen::Matrix3d _R_eigen;
	Eigen::Vector3d _T_eigen;
	//ROI Conversions
	void transformROIToRight(cv::Rect& left_ROI, cv::Rect& right_ROI);
	cv::Point2f transformPointToRight(int left_point_x, int left_point_y, float assumed_depth);

private:
	std::function<void(const std::array<double, 2>&, std::array<double, 2>&)> m_imagePointToCameraUnitPlane;
	// Marker Segmentation Methods
	bool blobDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints);
	bool contourDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints);	

	// Debugging
	LogLevel m_logLevel;
	//Segmentation Mode
	DetectionMode m_mode = DetectionMode::Contour; //Default is contour detection

	//Camera Settings:
	int m_depthCamRoiLowerRow = REALSENSE_HEIGHT - 1;
	int m_depthCamRoiUpperRow = 0;
	int m_depthCamRoiLeftCol = 0;
	int m_depthCamRoiRightCol = REALSENSE_WIDTH - 1;
	double m_depthNearClip = NEAR_CLIP;
	double m_depthFarClip = FAR_CLIP;
	bool m_show_clip_area;

	// Region of Interest for search
	int m_xMaxCrop = IMAGE_X_MAXCROP - 1;
	int m_yMaxCrop = IMAGE_Y_MAXCROP - 1;
	int m_xMinCrop = IMAGE_X_MINCROP;
	int m_yMinCrop = IMAGE_Y_MINCROP;
	cv::Rect _leftROI, _rightROI;

	// Input image settings
	int m_imWidth = REALSENSE_WIDTH;
	int m_imHeight = REALSENSE_HEIGHT;

	

	//Params for contour detection
	double _area, _cvxArea, _cvxity, _circy;

	//std::mutex m_paramMutex;
	cv::SimpleBlobDetector::Params _blob_params;
	cv::Ptr<cv::SimpleBlobDetector> _detector;


};

