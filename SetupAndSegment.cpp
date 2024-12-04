#include "SetupAndSegment.h"


//*****************************Equipment Setup Methods*********************************
//GPU Check and Setup
bool SetupAndSegment::GPUSetup()
{
	bool GPU_Found = false; //Set to true if we have GPU device
    
    //Quick Check of Number of CUDA Devices    
    int cudaEnabDevCount = cv::cuda::getCudaEnabledDeviceCount();

    if (cudaEnabDevCount) {
        std::cout << "************************************" << std::endl;
        std::cout << "Setting Up CUDA Device(s)" << std::endl;
        std::cout << "Number of available CUDA device(s): " << cudaEnabDevCount << std::endl;

        //Selecting Which CUDA Device(s) We Want to Use (if more than 1)
        if (cudaEnabDevCount>1) {
            cv::cuda::DeviceInfo cudaDeviceInfo;
            bool devCompat = false;
            std::cout << "List of all CUDA device(s):" << std::endl;
            std::cout << "" << std::endl;
            for (int devId = 0; devId < cudaEnabDevCount; ++devId)
            {
                cudaDeviceInfo = cv::cuda::DeviceInfo::DeviceInfo(devId);
                devCompat = cudaDeviceInfo.isCompatible();
                if (devCompat) {
                    std::cout << "This CUDA Device Is Compatable with OpenCV: " << std::endl;
                    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
                    GPU_Found = true;
                }
                else {
                    std::cout << "This CUDA Device Is Not Compatable with OpenCV: " << std::endl;
                    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

                }
            }
            std::cout << "" << std::endl;
            std::cout << "Pleases Enter the ID Number Of One Of The Above Compatable Devices: " << std::endl;
            int id_selected;
            std::cin >> id_selected;
            cv::cuda::setDevice(id_selected); //Sets the CUDA device

        }
        else
        {
            cv::cuda::DeviceInfo cudaDeviceInfo;
            bool devCompat = false;
            cudaDeviceInfo = cv::cuda::DeviceInfo::DeviceInfo(0);
            devCompat = cudaDeviceInfo.isCompatible();
            std::cout << "Your CUDA Device:" << std::endl;
            std::cout << "" << std::endl;
            if (devCompat) {
                cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
                std::cout << "This CUDA Device Is Compatable with OpenCV: " << std::endl;
               
                GPU_Found = true;
            }
            else {
                cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
                std::cout << "This CUDA Device Is Not Compatable with OpenCV: " << std::endl;               

            }
        }
    }
    else {
        std::cout << "You don't have any available CUDA device(s), please set this up" << std::endl;
    }

    if (!GPU_Found) {
        std::cout << "You don't have any compatable CUDA device(s), please set this up" << std::endl;
    }

    std::cout << "************************************" << std::endl;
    return(GPU_Found);
}

//RealSense Check and Setup

bool SetupAndSegment::RealSenseSetup()
{
    std::cout << "************************************" << std::endl;
    std::cout << "Setting Up RealSense Device(s)" << std::endl;
    if (_realSense_context.query_devices().size() == 0)
    {
        std::cout << "No RealSense Devices Were Found" << std::endl;
        std::cout << "************************************" << std::endl;
        return(false);
    }

    //Enabling the streams:
    _realSense_config.enable_stream(RS2_STREAM_INFRARED, 1, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Y8, REALSENSE_FPS);
    _realSense_config.enable_stream(RS2_STREAM_INFRARED, 2, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Y8, REALSENSE_FPS);
    _realSense_config.enable_stream(RS2_STREAM_DEPTH, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Z16, REALSENSE_FPS);

    //Gets the depth scale
    auto depth_units = _realSense_context.query_devices().front().query_sensors().front().get_option(RS2_OPTION_DEPTH_UNITS);
    m_depth_scale = (double)depth_units; //Sets the depth scale
    //Starting Device
    rs2::pipeline_profile pipeline_profile;
    try {
        pipeline_profile = _realSense_pipeline.start(_realSense_config);
        std::cout << "RealSense initialized successfully" << std::endl;
        std::cout << "************************************" << std::endl;
    }
    catch (const rs2::error& e) {
        std::cout << "Failed to initialize RealSense camera: " << e.what() << std::endl;
        std::cout << "************************************" << std::endl;
        return(false);
    }

    //Setting Laser Power
    rs2::device selected_device = pipeline_profile.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, ENABLE_REALSENSE_LASER); // Enable or disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        //auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, REALSESENSE_LASER_POWER); // Set  power
    }

    

    return(true); //Properly configured

}


//******************************Marker Segmentation****************************

//Inits the IR tracker:

SetupAndSegment::SetupAndSegment(int width, int height, LogLevel logLevel)
    : m_imWidth(width)
    , m_imHeight(height)
    , m_logLevel(logLevel)
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

void SetupAndSegment::setCameraIntrinsics(std::function<void(const std::array<double, 2>&, std::array<double, 2>&)> intrinsics) {
    m_imagePointToCameraUnitPlane = std::move(intrinsics);
}

SetupAndSegment::LogLevel SetupAndSegment::getLogLevel() {
    return m_logLevel;
}

void SetupAndSegment::setDetectionMode(DetectionMode mode) {
    std::scoped_lock<std::mutex> l(m_paramMutex);
    m_mode = mode;
}

//Sets up the ROI:
Eigen::Vector4i SetupAndSegment::setROI(int xMin, int xMax, int yMin, int yMax) {
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

    std::scoped_lock<std::mutex> l(m_paramMutex);
    m_xMinCrop = xMin;
    m_yMinCrop = yMin;
    m_xMaxCrop = xMax;
    m_yMaxCrop = yMax;

    return Eigen::Vector4i{ xMin, xMax, yMin, yMax };
}

//Sets up the camera boundaries
void SetupAndSegment::setCameraBoundaries(int roiUpperRow, int roiLowerRow, int roiLeftCol, int roiRightCol, int nearClipPlane, int farClipPlane) {
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


//Contour Detection
bool SetupAndSegment::contourDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {
    
    //Upload image to GPU
    _d_im.upload(im);

    //Threshold
    cv::cuda::threshold(_d_im, _d_imBW, CONTOUR_BIN_THRESHOLD, 255, cv::THRESH_BINARY);

    //Download Result back to cpu for contour detection
    _d_imBW.download(_imBW);

    // Use findContours
    cv::findContours(_imBW, _contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < _contours.size(); i++) {

        // Compute area
        _area = cv::contourArea(_contours[i]);

        // Compute convexity
        std::vector<cv::Point> cvxHull;
     
        cv::convexHull(_contours[i], cvxHull);
        _cvxArea = cv::contourArea(cvxHull);
        _cvxity = _area / _cvxArea;

        // Compute circularity
        cv::Point2f centre; float radius;
        cv::minEnclosingCircle(_contours[i], centre, radius);
        _circy = _area / (3.14159265 * radius * radius);

        // Filter by area, convexity, circularity
        if (_area < COUNTOUR_MIN_AREA || _area > COUNTOUR_MAX_AREA || _cvxity < CONTOUR_CONVEXITY || _circy < COUNTOUR_CIRCULARITY)
            continue;

        // Find contour centroids and save as keypoints
        float x = 0; float y = 0;
        for (size_t j = 0; j < _contours[i].size(); j++) {
            x += _contours[i][j].x;
            y += _contours[i][j].y;
        }
        x /= _contours[i].size();
        y /= _contours[i].size();

        cv::KeyPoint p(x, y, 0);
        p.size = radius * 2;
        keypoints.push_back(p);
    }

    return !keypoints.empty();
}

//Simple Blob Detection
bool SetupAndSegment::blobDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {
    
    // Detect keypoints
    _detector->detect(im, keypoints);
    return !keypoints.empty();
}


std::shared_ptr<SetupAndSegment::IrDetection> SetupAndSegment::findKeypointsWorldFrame(std::unique_ptr<std::vector<uint16_t>> irIm, std::unique_ptr<std::vector<uint16_t>> depthMap)
{
    auto detection = std::make_shared<IrDetection>();

    int xMinCrop, yMinCrop, xMaxCrop, yMaxCrop, width, height;
    {
        std::scoped_lock<std::mutex> l(m_paramMutex);
        xMinCrop = m_xMinCrop;
        yMinCrop = m_yMinCrop;
        xMaxCrop = m_xMaxCrop;
        yMaxCrop = m_yMaxCrop;
        width = m_imWidth;
        height = m_imHeight;
    }

    //Creates cv Mat image from data
    cv::Mat im = cv::Mat(_imagesz, CV_8UC1, irIm->data(), 0);
    if (im.empty()) {
        return detection;
    }

    //Crops the image to the ROI
    //const std::vector<int> croppedSz = { m_yMaxCrop - m_yMaxCrop + 1, m_xMaxCrop - m_xMinCrop + 1 };
    //cv::Mat im8b = cv::Mat(croppedSz, CV_16UC1); //The image that we are going to crop to


    cv::Rect roi(xMinCrop, yMinCrop, xMaxCrop - xMinCrop + 1, yMaxCrop - yMinCrop + 1);
    cv::Mat im8b=im(roi).clone(); //Crops the image to the ROI
    //const void* depthData = depthFrame.get_data(); // Retrieve the data once
    //if (!depthFrame || depthData == nullptr) {
    //    return detection; // Return empty depth detection
    //}
    //const uint16_t* depthMap = static_cast<const uint16_t*>(depthData);

    //Scales Contrast so it uses whole range
    //No Cropping for now
    //Uses GPU
    //d_im.upload(im);
    //d_im8b.create(im.size(), CV_8UC1);
    //cv::cuda::threshold(d_im, d_im8b, KEYPOINTS_MAX_INTENSITY, KEYPOINTS_MAX_INTENSITY, cv::THRESH_TRUNC);
    //cv::cuda::multiply(d_im8b, cv::Scalar(255.0f / KEYPOINTS_MAX_INTENSITY), d_im8b);
    //d_im8b.download(im);
    //cv::imshow("Thresholded", im);
    //char c = cv::waitKey(1);	//Grabs Key Press, if q we close

    /*
    *Uncomment if using CPU
    cv::Mat im8b = cv::Mat(_imagesz, CV_8UC1); //Scaled image

    for (int x = 0; x < im.cols; x++) 
    {
        for (int y = 0; y < im.rows; y++) 
        {
            uint16_t el = im.at<uint16_t>(y, x);
            if (el > 1000)
                im8b.at<uchar>(y, x) = 255; // Maximum intensity for values > 1000
            else
                im8b.at<uchar>(y, x) = (uchar)(el * 255 / 1000); // Scale contrast
        }
    }
    */



    bool success = false;
    std::vector<cv::KeyPoint> keypoints; //Creates keypoints object
    if (m_mode == DetectionMode::Blob)
        success = blobDetect(im8b, keypoints);
    else //Use contour detection
        success = contourDetect(im8b, keypoints); //Finds the object contours

    //Map to 3D
    if (success)
    {
        cv::Mat outputImage;
        cv::drawKeypoints(im,keypoints,outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("Keypoints", outputImage);
        char c = cv::waitKey(1);	//Grabs Key Press, if q we close

        //Save the 3D Points
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            // Point in image array
            double x = keypoints[i].pt.x+xMinCrop;
            double y = keypoints[i].pt.y+yMinCrop;
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
            if (depth_m<m_depthNearClip || depth_m>m_depthFarClip) continue;
            Eigen::Vector3d pt = depth_m * pointOnUnitPlane.normalized(); //Might need to change the divide by 1000

            //Find the diameter of the blob in the world coordinates
            auto left = (x - keypoints[i].size / 2 > 0) ? x - keypoints[i].size / 2 : 0;
            auto right = (x + keypoints[i].size / 2 < REALSENSE_WIDTH) ? x + keypoints[i].size / 2 : REALSENSE_WIDTH;
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