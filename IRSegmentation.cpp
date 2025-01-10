#include "IRSegmentation.h"

//Inits the IR tracker:

IRSegmentation::IRSegmentation(int width, int height, double depth_scale, LogLevel logLevel,bool show_clip_area)
    : m_imWidth(width)
    , m_imHeight(height)
    , m_logLevel(logLevel),
    m_depth_scale(depth_scale),
    m_show_clip_area(show_clip_area)
{
    m_xMaxCrop = width - 1;
    m_yMaxCrop = height - 1;

    // Default to normalized coordinates
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

std::shared_ptr<IRSegmentation::IrDetection> IRSegmentation::findKeypointsWorldFrame(std::unique_ptr<std::vector<uint8_t>> irIm, std::unique_ptr<std::vector<uint16_t>> depthMap)
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

    //Creates cv Mat image from data
    cv::Mat im = cv::Mat(_imagesz, CV_8UC1, irIm->data(), 0);
    if (im.empty()) {
        return detection;
    }

    //Crops the image to the ROI

    cv::Rect roi(xMinCrop, yMinCrop, xMaxCrop - xMinCrop + 1, yMaxCrop - yMinCrop + 1);
    cv::Mat im8b = im(roi).clone(); //Crops the image to the ROI

    bool success = false;
    std::vector<cv::KeyPoint> keypoints; //Creates keypoints object
    if (m_mode == DetectionMode::Blob)
        success = blobDetect(im8b, keypoints);
    else //Use contour detection
        success = contourDetect(im8b, keypoints); //Finds the object contours

    //Map to 3D
    if (success)
    {
        //For display
        if (m_show_clip_area)
        {
            cv::Mat outputImage;
            cv::drawKeypoints(im8b, keypoints, outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::imshow("Contours on Cropped", outputImage);
            cv::waitKey(1);
        }

        //Save the 3D Points
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            // Point in image array
            double x = keypoints[i].pt.x + xMinCrop;
            double y = keypoints[i].pt.y + yMinCrop;
            int xInt = static_cast<int>(std::round(x));
            int yInt = static_cast<int>(std::round(y));


            if (y < m_depthCamRoiUpperRow || y > m_depthCamRoiLowerRow || x < m_depthCamRoiLeftCol || x > m_depthCamRoiRightCol) {
                if (m_logLevel == LogLevel::Verbose)
                    LOG << "Actually out of bounds";
                continue;
            }

            std::array<double, 2> xy = { 0, 0 };
            std::array<double, 2> uv = { x , y };
            m_imagePointToCameraUnitPlane(uv, xy);
            auto pointOnUnitPlane = Eigen::Vector3d(xy[0], xy[1], 1);


            //Find Depth at point
            auto depth = (*depthMap)[(int)(yInt * width + xInt)];
            double depth_m = (double)depth * m_depth_scale; //Converts depth (16 bit representation) to meters

            if (depth_m<m_depthNearClip || depth_m>m_depthFarClip) {
                //std::cout << "Entered" << std::endl;
                continue;

            }
            Eigen::Vector3d pt = depth_m * pointOnUnitPlane.normalized();

            //Find the diameter of the blob in the world coordinates
            auto left = (x - keypoints[i].size / 2 > 0) ? x - keypoints[i].size / 2 : 0;
            auto right = (x + keypoints[i].size / 2 < width) ? x + keypoints[i].size / 2 : width;
            std::array<double, 2> xyL = { 0, 0 };
            std::array<double, 2> uvL = { left , y };
            m_imagePointToCameraUnitPlane(uvL, xyL);
            std::array<double, 2> xyR = { 0, 0 };
            std::array<double, 2> uvR = { right , y };
            m_imagePointToCameraUnitPlane(uvR, xyR);

            auto leftPoint = Eigen::Vector3d(xyL[0], xyL[1], 1);
            auto rightPoint = Eigen::Vector3d(xyR[0], xyR[1], 1);

            leftPoint = depth_m * leftPoint.normalized();
            rightPoint = depth_m * rightPoint.normalized();
            double diam = (rightPoint - leftPoint).norm();

            // Save the points
            detection->points.push_back(pt);
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