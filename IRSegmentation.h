#pragma once

//C++ Includes
#include <iostream>
#include <string>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/world.hpp>

//RealSense Includes
#include <librealsense2/rs.hpp>

//Pose Tracking Includes
#include <eigen3/Eigen/Dense>
#include "log.h"
#include "framework.h"
#include <thread>
#include <mutex>

#include "RealSense.h"

//*********2D Image Crop Constants
const int IMAGE_X_MAXCROP = 848;
const int IMAGE_Y_MAXCROP = 480;
const int IMAGE_X_MINCROP = 0;
const int IMAGE_Y_MINCROP = 0;

//*********Contour Segmentation Constants**************
const int CONTOUR_BIN_THRESHOLD = 222;
const int COUNTOUR_MIN_AREA = 2;
const int COUNTOUR_MAX_AREA = 450;
const float CONTOUR_CONVEXITY = 0.66f;
const float COUNTOUR_CIRCULARITY = 0.52f;

//**********Blob Segmentation Constants****************
const int BLOB_MIN_THRESHOLD = 215;
const int BLOB_MAX_THRESHOLD = 255;
const int BLOB_MIN_AREA = 2;
const int BLOB_MAX_AREA = 450;
const float BLOB_MIN_CONVEXITY = 0.62f;
const int BLOB_MIN_DSTANCE_BETWEEN = 2;

//*******Find Keypoints Constants***************
const float KEYPOINTS_MAX_INTENSITY = 1000.0f;
const double NEAR_CLIP = 0.25f;
const double FAR_CLIP = 1.7f; //Average Arm Span is 65 cm, we want the tracker to be about the same distance from eyes to hand


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
	IRSegmentation(const int width, const int height, double depth_scale,   LogLevel logLevel = LogLevel::Silent);
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
	
	//Finds marker points in the world frame
	std::shared_ptr<IrDetection> findKeypointsWorldFrame(std::unique_ptr<std::vector<uint16_t>> irIm, std::unique_ptr<std::vector<uint16_t>> depthMap);

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

	// Region of Interest for search
	int m_xMaxCrop = IMAGE_X_MAXCROP - 1;
	int m_yMaxCrop = IMAGE_Y_MAXCROP - 1;
	int m_xMinCrop = IMAGE_X_MINCROP;
	int m_yMinCrop = IMAGE_Y_MINCROP;

	// Input image settings
	int m_imWidth = REALSENSE_WIDTH;
	int m_imHeight = REALSENSE_HEIGHT;

	double m_depth_scale; //depth scale that we use

	//Params for contour detection
	double _area, _cvxArea, _cvxity, _circy;

	cv::SimpleBlobDetector::Params _blob_params;
	cv::Ptr<cv::SimpleBlobDetector> _detector;


};

