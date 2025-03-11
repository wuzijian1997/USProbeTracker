#include "IRSegmentation.h"

//Inits the IR tracker:

IRSegmentation::IRSegmentation(int width, int height, double depth_scale, LogLevel logLevel,bool show_clip_area,
    cv::Mat left_camera_mat,cv::Mat left_dist,cv::Mat right_camera_mat,cv::Mat right_dist,
    cv::Mat R,cv::Mat T, cv::Mat E, cv::Mat F)
    : m_imWidth(width)
    , m_imHeight(height)
    , m_logLevel(logLevel),
    m_depth_scale(depth_scale),
    m_show_clip_area(show_clip_area),
    _left_camera_mat(left_camera_mat),
    _left_dist(left_dist),
    _right_camera_mat(right_camera_mat),
    _right_dist(right_dist),
    _R(R),
    _T(T),
    _E(E),
    _F(F)
{
    m_xMaxCrop = width - 1;
    m_yMaxCrop = height - 1;


    //Gets the stereo rectification map
    cv::Mat Q;
    cv::stereoRectify(_left_camera_mat, _left_dist, _right_camera_mat, _right_dist, cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), R, T,
        _R_left, _R_right, _P_left, _P_right, Q,cv::CALIB_ZERO_DISPARITY,-1,cv::Size(),nullptr,nullptr);

    _R_left_inv = _R_left.t(); //Takes the inverse of the rotation matrix moving the camera coordinate to the rectified coordinate system


    //Individual camera parameters
    _fxL = _left_camera_mat.at<double>(0, 0);
    _fyL = _left_camera_mat.at<double>(1, 1);
    _cxL = _left_camera_mat.at<double>(0, 2);
    _cyL = _left_camera_mat.at<double>(1, 2);

    _fxR = _right_camera_mat.at<double>(0, 0);
    _fyR = _right_camera_mat.at<double>(1, 1);
    _cxR = _right_camera_mat.at<double>(0, 2);
    _cyR = _right_camera_mat.at<double>(1, 2);

    //Convert R and T to eigen for faster computation
    _R_eigen = convertCVMatToEigen_3b3(R);
    _T_eigen = convertCVMatToEigen_1b3(T);



    // Default to normalized coordinates (not used)
    m_imagePointToCameraUnitPlane = [width, height](const std::array<double, 2>& uv, std::array<double, 2>& xy) {
        xy[0] = uv[0] / width;
        xy[1] = uv[1] / height;
    };
    //****Vars for Blob Detection*****
    _blob_params.filterByColor = true; // Filter by brightness. Markers should be bright
    _blob_params.blobColor = 255;
    _blob_params.minThreshold = BLOB_MIN_THRESHOLD;
    _blob_params.maxThreshold = BLOB_MAX_THRESHOLD;
    _blob_params.filterByArea = true;
    _blob_params.minArea = BLOB_MIN_AREA; // size in pixels
    _blob_params.maxArea = BLOB_MAX_AREA;
    _blob_params.filterByConvexity = true;
    _blob_params.minConvexity = BLOB_MIN_CONVEXITY;

    _blob_params.minDistBetweenBlobs = BLOB_MIN_DSTANCE_BETWEEN;
    _blob_params.filterByInertia = false;
    _blob_params.filterByCircularity = false;

    // Create detector
    _detector = cv::SimpleBlobDetector::create(_blob_params);

}

//*****************Setup Methods*******************


//Sets up the ROI:
Eigen::Vector4i IRSegmentation::setROI(int xMin, int xMax, int yMin, int yMax) {
    // Keep within the bounds of the camera
    if (xMin < m_depthCamRoiLeftCol) xMin = m_depthCamRoiLeftCol;
    if (yMin < m_depthCamRoiUpperRow) yMin = m_depthCamRoiUpperRow;
    if (xMax > m_depthCamRoiRightCol) xMax = m_depthCamRoiRightCol;
    if (yMax > m_depthCamRoiLowerRow) yMax = m_depthCamRoiLowerRow;

    if (xMax - xMin <= 10 || yMax - yMin <= 10) {
        std::cerr << "ROI width or height too small" << std::endl;
        return Eigen::Vector4i{ m_xMinCrop, m_xMaxCrop, m_yMinCrop, m_yMaxCrop };
    }

    if (m_logLevel == LogLevel::VeryVerbose)
        LOG << "Set ROI to " << xMin << ", " << xMax << ", " << yMin << ", " << yMax;

    //std::scoped_lock<std::mutex> l(m_paramMutex);
    m_xMinCrop = xMin;
    m_yMinCrop = yMin;
    m_xMaxCrop = xMax;
    m_yMaxCrop = yMax;

    return Eigen::Vector4i{ xMin, xMax, yMin, yMax };
}

//Sets up the camera boundaries
void IRSegmentation::setCameraBoundaries(int roiUpperRow, int roiLowerRow, int roiLeftCol, int roiRightCol, double nearClipPlane, double farClipPlane) {
    m_depthCamRoiUpperRow = roiUpperRow;
    m_depthCamRoiLowerRow = roiLowerRow;
    m_depthCamRoiLeftCol = roiLeftCol;
    m_depthCamRoiRightCol = roiRightCol;
    m_depthNearClip = nearClipPlane;
    m_depthFarClip = farClipPlane;

    // Update the ROI so it cuts off at the correct boundaries if it is currently searching the whole image
    if (m_xMaxCrop == m_imWidth - 1 && m_yMaxCrop == m_imHeight - 1 && m_xMinCrop == 0 && m_yMinCrop == 0) {
        setROI(m_xMinCrop, m_xMaxCrop, m_yMinCrop, m_yMaxCrop);
    }


    if (m_logLevel == LogLevel::VeryVerbose)
        LOG << "Top row " << m_depthCamRoiUpperRow << ", bottom row " << m_depthCamRoiLowerRow << ", left col " << m_depthCamRoiLeftCol << ", right col " << m_depthCamRoiRightCol << ", near " << m_depthNearClip << ", far " << m_depthFarClip;
}

void IRSegmentation::setCameraIntrinsics(std::function<void(const std::array<double, 2>&, std::array<double, 2>&)> intrinsics) {
    m_imagePointToCameraUnitPlane = std::move(intrinsics);
}

void IRSegmentation::setDetectionMode(DetectionMode mode) {
    //std::scoped_lock<std::mutex> l(m_paramMutex);
    m_mode = mode;
}

IRSegmentation::LogLevel IRSegmentation::getLogLevel() {
    return m_logLevel;
}


//*****************Segmentation Methods******************

std::shared_ptr<IRSegmentation::IrDetection> IRSegmentation::findKeypointsWorldFrame(std::unique_ptr<std::vector<uint8_t>> irImLeft, std::unique_ptr<std::vector<uint8_t>> irImRight)
{
    auto detection = std::make_shared<IrDetection>();
    int xMinCrop, yMinCrop, xMaxCrop, yMaxCrop, width, height;
    //{
        //std::scoped_lock<std::mutex> l(m_paramMutex);  //Maybe change this back
    xMinCrop = m_xMinCrop;
    yMinCrop = m_yMinCrop;
    xMaxCrop = m_xMaxCrop;
    yMaxCrop = m_yMaxCrop;
    width = m_imWidth;
    height = m_imHeight;
    //}
    const std::vector<int> _imagesz = { height,width };

    //Creates cv Mat images from left/right data
    cv::Mat imLeft = cv::Mat(_imagesz, CV_8UC1, irImLeft->data(), 0);
    cv::Mat imRight = cv::Mat(_imagesz, CV_8UC1, irImRight->data(), 0);

    if (imLeft.empty() || imRight.empty()) { //Checks that image is valid
        return detection;
    }    

    //Crops the images 
    _leftROI=cv::Rect(xMinCrop, yMinCrop, xMaxCrop - xMinCrop + 1, yMaxCrop - yMinCrop + 1);

    //Computes ROI in the right image based on left image roi (assumes depth of 0.9m)
    transformROIToRight(_leftROI,_rightROI);

    //Ensure the ROI is within valid bounds
    _rightROI.x = std::max(0, std::min(_rightROI.x, width - 1));
    _rightROI.y = std::max(0, std::min(_rightROI.y, height - 1));

    _rightROI.width = std::min(width - _rightROI.x, _rightROI.width);
    _rightROI.height = std::min(height - _rightROI.y, _rightROI.height);

    //Crops images to the ROIs
    imLeft = imLeft(_leftROI);
    imRight = imRight(_rightROI);
    
    //Detect keypoints in the unrectified left image
    bool success_left = false;
    std::vector<cv::KeyPoint> keypoints_left; //Creates keypoints object
    if (m_mode == DetectionMode::Blob)
        success_left = blobDetect(imLeft, keypoints_left);
    else //Use contour detection
        success_left = contourDetect(imLeft, keypoints_left); //Finds the object contours

    //Detect keypoints in the unrectified right image
    bool success_right = false;
    std::vector<cv::KeyPoint> keypoints_right; //Creates keypoints object
    if (m_mode == DetectionMode::Blob)
        success_right = blobDetect(imRight, keypoints_right);
    else //Use contour detection
        success_right = contourDetect(imRight, keypoints_right); //Finds the object contours

    //Map to 3D using stereo triangulation
    if (success_left && success_right)
    {
        //For display
        if (m_show_clip_area)
        {
            cv::Mat outputImage_left, outputImage_right;
            //Keypoints drawn on left image
            cv::drawKeypoints(imLeft, keypoints_left, outputImage_left, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(imRight, keypoints_right, outputImage_right, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::imshow("Contours on Left Cropped", outputImage_left);
            cv::imshow("Contours on Right Cropped", outputImage_right);
            cv::waitKey(1);
        }


        //*********Step 1: Get 2D Keypoint Location in Original Left/Right (uncropped) image planes*************
        std::vector<cv::Point2f> left_points, right_points;
        std::vector<int> valid_keypoints_left_indices; //Tracks indices of valid keypoints (ones that aren't ouside the left camera sensor)

        //Left 2D keypoints
        for (size_t i = 0; i < keypoints_left.size(); i++)
        {
            double x = keypoints_left[i].pt.x + xMinCrop;
            double y = keypoints_left[i].pt.y + yMinCrop;
            
            if (y < m_depthCamRoiUpperRow || y > m_depthCamRoiLowerRow || x < m_depthCamRoiLeftCol || x > m_depthCamRoiRightCol) {
                if (m_logLevel == LogLevel::Verbose)
                    LOG << "Actually out of bounds";
                continue;
            }

            valid_keypoints_left_indices.push_back(i);
            left_points.emplace_back(x, y);
        }

        //Right 2D keypoints
        for (size_t i = 0; i < keypoints_right.size(); i++)
        {
            double x = keypoints_right[i].pt.x + _rightROI.x;
            double y = keypoints_right[i].pt.y + _rightROI.y;

            if (y < m_depthCamRoiUpperRow || y > m_depthCamRoiLowerRow || x < m_depthCamRoiLeftCol || x > m_depthCamRoiRightCol) {
                if (m_logLevel == LogLevel::Verbose)
                    LOG << "Actually out of bounds";
                continue;
            }
            right_points.emplace_back(x,y);
        }

        //******************Step 2: Undistort the keypoints in the left/right images***********************       
        //It projects them onto rectified image planes
        std::vector<cv::Point2f> left_undistorted, right_undistorted;

        cv::undistortPoints(left_points, left_undistorted, _left_camera_mat, _left_dist,_R_left,_P_left,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, UNDISTORTION_MAX_COUNT, UNDISTORTION_EPS));
        cv::undistortPoints(right_points, right_undistorted, _right_camera_mat, _right_dist,_R_right,_P_right, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, UNDISTORTION_MAX_COUNT, UNDISTORTION_EPS));



        //**********Step 3: Match points between left/right images using epipolar constraints**************

        std::vector<cv::Point2f> left_matched, right_matched;

        // Create a boolean vector to track used points in the right image
        std::vector<bool> right_used(right_undistorted.size(), false);

        //Tracks which left points end up being used in left_matched
        std::vector<int> valid_left_point_indices;

        for (size_t i = 0; i < left_undistorted.size(); i++) //Loops for all points in left
        {
            float min_dist = 1e6; //Inits the minimum distance in the x-direction
            int best_match_idx = -1; //The best match index in the right_undistorted image
            cv::Point2f left_point = left_undistorted[i];

            // Iterate over right points
            for (size_t j = 0; j < right_undistorted.size();j++) {
                cv::Point2f right_point = right_undistorted[j];

                if (!right_used[j] && std::abs(left_point.y - right_point.y) < EPIPOLAR_MATCH_Y_THRESHOLD) { // Check epipolar constraint
                    float distance = std::abs(left_point.x - right_point.x);

                    if (distance < min_dist) {
                        min_dist = distance;
                        best_match_idx = j;
                    }
                }
            }

            if (best_match_idx != -1 && min_dist < EPIPOLAR_MATCH_X_THRESHOLD)
            {
                left_matched.push_back(left_point);
                right_matched.push_back(right_undistorted[best_match_idx]);
                right_used[best_match_idx] = true; // Mark the right point as used

                valid_left_point_indices.push_back(i);
            }

        }


        //************Step 3.5 Refine Correspondances (maybe not needed)*****************
    /*    cv::Mat refined_left, refined_right;
        cv::correctMatches(_F, left_matched, right_matched, refined_left, refined_right);
        left_matched = refined_left;
        right_matched = refined_right;*/


        //**********************Step 4: Triangulate the 3D points************************
        cv::Mat left_cam_points4d;
        // Check if matched points are empty
        if (left_matched.empty() || right_matched.empty()) {
            std::cout << "Error: No matched points found. Exiting triangulation"<<std::endl;
            return detection;  // Exit early
        }
        cv::triangulatePoints(_P_left, _P_right, left_matched, right_matched, left_cam_points4d); //For a rectified image

        if (left_cam_points4d.empty()) {
            std::cout << "No Points Could Be Triangulated" << std::endl;
            return detection;  // Exit early
        }


        cv::Mat left_cam_points3D_rect;
        cv::convertPointsFromHomogeneous(left_cam_points4d.t(), left_cam_points3D_rect);

        //****Step5: transform to left camera frame and find blob diameter******
        for (int i = 0; i < left_cam_points3D_rect.rows; i++)
        {
            //Get current 3D point
            //Extract row and ensure it's a 3x1 column vector
            cv::Mat ptMat_rect = left_cam_points3D_rect.row(i).t(); // Transpose ensures it's 3x1
            ptMat_rect = ptMat_rect.reshape(1, 3);  // Ensure it's a single-channel 3x1

             //Convert to CV_64F to match _R_left_inv
            cv::Mat ptMat_rect_64;
            ptMat_rect.convertTo(ptMat_rect_64, CV_64F);

             //Perform matrix multiplication
            cv::Mat ptMat_original = _R_left_inv * ptMat_rect_64;

            //Extract the transformed coordinates (in left camera frame) => if rectified
            double X = ptMat_original.at<double>(0, 0);
            double Y = ptMat_original.at<double>(1, 0);
            double Z = ptMat_original.at<double>(2, 0);

            //Check that the point is within the clip area
            if (Z<m_depthNearClip || Z>m_depthFarClip) {
                //        //std::cout << "Entered" << std::endl;
                continue;

            }

            //****************Find Blob Diameter in Left Camera Coord System********************

            //Find the diameter of the blob in the left image plane
            double keypoint_diameter_px = keypoints_left[valid_keypoints_left_indices[valid_left_point_indices[i]]].size; //Get pixel diameter in pixels

            //Gets the 2D positions of left/right edges in the left image plane
            double x_left_edge_px = left_points[valid_left_point_indices[i]].x - keypoint_diameter_px / 2;
            double x_right_edge_px = left_points[valid_left_point_indices[i]].x + keypoint_diameter_px / 2;


            std::vector<cv::Point2f> edge_pixels = {
                cv::Point2f(x_left_edge_px, left_points[valid_left_point_indices[i]].y),
                 cv::Point2f(x_right_edge_px, left_points[valid_left_point_indices[i]].y)
            };

            //Get the edges in the left camera coordinate system
            std::vector<cv::Point2f> edge_leftcamera;
            cv::undistortPoints(edge_pixels, edge_leftcamera, _left_camera_mat, _left_dist);

            //Convert the two edges to 3D using estimated Z of center of keypoint
            Eigen::Vector3d left_edge_3D(edge_leftcamera[0].x * Z, edge_leftcamera[0].y * Z, Z);
            Eigen::Vector3d right_edge_3D(edge_leftcamera[1].x* Z, edge_leftcamera[1].y* Z, Z);

            double diam = (left_edge_3D - right_edge_3D).norm();



            //*******************Project the X,Y,Z points onto the left camera plane******************
            //Don't actually need to project points because I already have that info in the left camera (left_points)
            //std::vector<cv::Point3f> objectPoints = { cv::Point3f(X, Y, Z) }; // 3D point in left camera coordinate system
            //std::vector<cv::Point2f> projectedPoints; // 2D point in left image plane

            //cv::projectPoints(objectPoints,
            //    cv::Mat::zeros(3, 1, CV_64F), // No rotation (identity)
            //    cv::Mat::zeros(3, 1, CV_64F), // No translation
            //    _left_camera_mat,             // Left camera intrinsic matrix
            //    _left_dist,                    // Left camera distortion coefficients
            //    projectedPoints);

            //// The reprojected (u,v) coordinates
            //cv::Point2f uv = projectedPoints[0];
            int xInt = static_cast<int>(std::round(left_points[valid_left_point_indices[i]].x));
            int yInt = static_cast<int>(std::round(left_points[valid_left_point_indices[i]].y));


            detection->points.push_back(Eigen::Vector3d(X, Y, Z));
            detection->imCoords.emplace_back(xInt, yInt);
            detection->markerDiameters.push_back(diam);

        }

    }

    return detection;
}


bool IRSegmentation::contourDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {
    cv::Mat imbW;
    cv::threshold(im, imbW, CONTOUR_BIN_THRESHOLD, 255, cv::THRESH_BINARY);
    
    // Use findContours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imbW, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {

        // Compute area
        _area = cv::contourArea(contours[i]);

        // Compute convexity
        std::vector<cv::Point> cvxHull;

        cv::convexHull(contours[i], cvxHull);
        _cvxArea = cv::contourArea(cvxHull);
        _cvxity = _area / _cvxArea;

        // Compute circularity
        cv::Point2f centre; float radius;
        cv::minEnclosingCircle(contours[i], centre, radius);
        _circy = _area / (3.14159265 * radius * radius);

        // Filter by area, convexity, circularity
        if (_area < COUNTOUR_MIN_AREA || _area > COUNTOUR_MAX_AREA || _cvxity < CONTOUR_CONVEXITY || _circy < COUNTOUR_CIRCULARITY)
            continue;

        // Find contour centroids and save as keypoints
        float x = 0; float y = 0;
        for (size_t j = 0; j < contours[i].size(); j++) {
            x += contours[i][j].x;
            y += contours[i][j].y;
        }
        x /= contours[i].size();
        y /= contours[i].size();

        cv::KeyPoint p(x, y, 0);
        p.size = radius * 2;
        keypoints.push_back(p);
    }

    return !keypoints.empty();
}

//Simple Blob Detection
bool IRSegmentation::blobDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {

    // Detect keypoints
    _detector->detect(im, keypoints);
    return !keypoints.empty();
}


void IRSegmentation::transformROIToRight(cv::Rect& left_ROI, cv::Rect& right_ROI)
{
    float assumed_depth = 0.9f; //Assume a fixed distance so we can find a point along the epipolar line

    // Transform each corner to right image
    cv::Point2f topLeftR = transformPointToRight(left_ROI.x, left_ROI.y, assumed_depth);
    cv::Point2f bottomRightR = transformPointToRight(left_ROI.x + left_ROI.width, left_ROI.y + left_ROI.height, assumed_depth);

    // Create the corresponding ROI in the right image
    right_ROI = cv::Rect(topLeftR.x, topLeftR.y, bottomRightR.x - topLeftR.x, bottomRightR.y - topLeftR.y);

}

cv::Point2f IRSegmentation::transformPointToRight(int left_point_x,int left_point_y, float assumed_depth) {
    // Step 1: Convert 2D point to normalized camera coordinates in the left camera
    Eigen::Vector3d T_lc(((left_point_x - _cxL) * assumed_depth) / _fxL,
        (left_point_y - _cyL) * assumed_depth / _fyL,
        assumed_depth);

    // Step 2: Transform the point to the right camera coordinate system
    Eigen::Vector3d T_rc = _R_eigen * T_lc + _T_eigen; // Apply stereo transformation

    // Step 3: Project the transformed point to the right image using right camera intrinsics
    double u_right = _fxR * (T_rc.x() / T_rc.z()) + _cxR;
    double v_right = _fyR * (T_rc.y() / T_rc.z()) + _cyR;

    return cv::Point2f(u_right, v_right);
}
