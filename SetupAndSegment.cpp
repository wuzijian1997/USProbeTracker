#include "SetupAndSegment.h"

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


SetupAndSegment::~SetupAndSegment()
{
    //Closes the threading
    {
        std::lock_guard<std::mutex> lock(_realSenseFrameMutex);
        _realsenseRunFrameThread = false;
    }

    if (_realsenseFrameThread && _realsenseFrameThread->joinable())
    {
        _realsenseFrameThread->join();
    }

    //CLears the queues
    while (!_irLeftFrameQueue.empty()) {
        _irLeftFrameQueue.pop();
    }
    while (!_irRightFrameQueue.empty()) {
        _irRightFrameQueue.pop();
    }
    while (!_depthFrameQueue.empty()) {
        _depthFrameQueue.pop();
    }
    while (!_depthFilteredPtrQueue.empty()) {
        _depthFilteredPtrQueue.pop();
    }
    while (!_irLeftPtrQueue.empty()) {
        _irLeftPtrQueue.pop();
    }

    //Closes the realsense objects
    _realSense_config.disable_all_streams();
    _realSense_pipeline.stop();
    

}

//*****************************Equipment Setup Methods*********************************
//GPU Check and Setup (if needed)
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



//RealSense Check and Setup, also starts the RealSense Thread
bool SetupAndSegment::RealSenseSetup(int timeout)
{
    _timeout = timeout;
    std::cout << "************************************" << std::endl;
    std::cout << "Setting Up RealSense Device(s)" << std::endl;
    if (_realSense_context.query_devices().size() == 0)
    {
        std::cout << "No RealSense Devices Were Found" << std::endl;
        std::cout << "************************************" << std::endl;
        return false;
    }

    //Enabling the streams:
    try {
        _realSense_config.enable_stream(RS2_STREAM_INFRARED, 1, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Y8, REALSENSE_FPS);
        _realSense_config.enable_stream(RS2_STREAM_INFRARED, 2, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Y8, REALSENSE_FPS);
        _realSense_config.enable_stream(RS2_STREAM_DEPTH, REALSENSE_WIDTH, REALSENSE_HEIGHT, RS2_FORMAT_Z16, REALSENSE_FPS);
    } catch (const rs2::error& e) {
        std::cout << "Error enabling RealSense streams: " << e.what() << std::endl;
        return false;
    }


    //Gets the depth scale
    try {
        auto depth_units = _realSense_context.query_devices().front().query_sensors().front().get_option(RS2_OPTION_DEPTH_UNITS);
        m_depth_scale = (double)depth_units; //Sets the depth scale

    } catch (const rs2::error& e) {
        std::cout << "Error retrieving depth scale: " << e.what() << std::endl;
        return false;
    }

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
        return false;
    }

    //Gets the intrinsics of the left IR camera
    auto stream_profiles = pipeline_profile.get_streams();
    bool found = false;
    for (auto& sp : stream_profiles) {
        if (sp.stream_type() == RS2_STREAM_INFRARED && sp.stream_index() == 1) {
            auto video_stream_profile = sp.as<rs2::video_stream_profile>();
            _realSense_intrinsics_leftIR = video_stream_profile.get_intrinsics();
            found = true;
            break;
        }
    }

    if (!found) {
        std::cout << "Could not find the left infrared camera stream profile." << std::endl;
        return EXIT_FAILURE;
    }

    
      

    //Setting Laser Power

    try{
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
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, REALSENSE_LASER_POWER); // Set  power
        }

        //Sets the gain of the IR images (sensors)
        auto sensors = selected_device.query_sensors();

        //Loops and checks if the sensor is the stereo module and if it supports gain
        for (auto& sensor : sensors)
        {
            if (sensor.supports(RS2_OPTION_GAIN) && sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("Stereo Module"))
            {
                sensor.set_option(RS2_OPTION_GAIN, static_cast<float>(REALSENSE_GAIN)); 
                std::cout << "Set Camera Gain" << std::endl;
            }
        }


    }
    catch (const rs2::error& e) {
        std::cout << "Error setting laser power: " << e.what() << std::endl;
        return false;
    }

    

    //Resets the pipeline:
    //ResetRealSensePipeline();

    //Aligns to left infrared stream
    try{
        initializeAlign(RS2_STREAM_INFRARED);
    }
    catch (const rs2::error& e) {
        std::cout << "Error initializing IR Frame alignment: " << e.what() << std::endl;
        return false;
    }
    
    //Configures the temporal depth filter for the findkeypoints function
    _temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, TEMPORAL_DEPTH_FILTER_ALPHA);
    _temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, TEMPORAL_DEPTH_FILTER_DELTA);


    //Starts the thread function for the realsense (starts grabbing frames)
    _realsenseRunFrameThread = true;
    _realsenseFrameThread=std::make_shared<std::thread>([this]() {ReadFrames(); });


    return true; //Properly configured

}

//Helper functions for equipment
void SetupAndSegment::ResetRealSensePipeline() {
    try {
        _realSense_pipeline.stop();
        _realSense_pipeline.start(_realSense_config); // Restart the pipeline
        std::cout << "RealSense pipeline reset successfully." << std::endl;
    }
    catch (const rs2::error& e) {
        std::cerr << "Failed to reset RealSense pipeline: " << e.what() << std::endl;
    }
}


void SetupAndSegment::initializeAlign(rs2_stream stream_Type) {
    _align_to_left_ir = std::make_unique<rs2::align>(stream_Type);
}

void SetupAndSegment::setCameraIntrinsics(std::function<void(const std::array<double, 2>&, std::array<double, 2>&)> intrinsics) {
    m_imagePointToCameraUnitPlane = std::move(intrinsics);
}

//Sets up the camera boundaries
void SetupAndSegment::setCameraBoundaries(int roiUpperRow, int roiLowerRow, int roiLeftCol, int roiRightCol, double nearClipPlane, double farClipPlane) {
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


//******************************Marker Segmentation****************************



SetupAndSegment::LogLevel SetupAndSegment::getLogLevel() {
    return m_logLevel;
}
void SetupAndSegment::setDetectionMode(DetectionMode mode) {
    //std::scoped_lock<std::mutex> l(m_paramMutex);
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

    //std::scoped_lock<std::mutex> l(m_paramMutex);
    m_xMinCrop = xMin;
    m_yMinCrop = yMin;
    m_xMaxCrop = xMax;
    m_yMaxCrop = yMax;

    return Eigen::Vector4i{ xMin, xMax, yMin, yMax };
}


//Contour Detection
bool SetupAndSegment::contourDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {
    cv::Mat imbW;
    cv::threshold(im, imbW, CONTOUR_BIN_THRESHOLD, 255, cv::THRESH_BINARY);
    //Upload image to GPU
    //_d_im.upload(im);

    //Threshold
    //cv::cuda::threshold(_d_im, _d_imBW, CONTOUR_BIN_THRESHOLD, 255, cv::THRESH_BINARY);

    //Download Result back to cpu for contour detection
    //_d_imBW.download(_imBW);

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
bool SetupAndSegment::blobDetect(cv::Mat& im, std::vector<cv::KeyPoint>& keypoints) {
    
    // Detect keypoints
    _detector->detect(im, keypoints);
    return !keypoints.empty();
}


std::shared_ptr<SetupAndSegment::IrDetection> SetupAndSegment::findKeypointsWorldFrame(std::unique_ptr<std::vector<uint16_t>> irIm, std::unique_ptr<std::vector<uint16_t>> depthMap)
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
    //const std::vector<int> croppedSz = { m_yMaxCrop - m_yMaxCrop + 1, m_xMaxCrop - m_xMinCrop + 1 };
    //cv::Mat im8b = cv::Mat(croppedSz, CV_16UC1); //The image that we are going to crop to


    cv::Rect roi(xMinCrop, yMinCrop, xMaxCrop - xMinCrop + 1, yMaxCrop - yMinCrop + 1);
    cv::Mat im8b=im(roi).clone(); //Crops the image to the ROI


    bool success = false;
    std::vector<cv::KeyPoint> keypoints; //Creates keypoints object
    if (m_mode == DetectionMode::Blob)
        success = blobDetect(im8b, keypoints);
    else //Use contour detection
        success = contourDetect(im8b, keypoints); //Finds the object contours

    //Map to 3D
    if (success)
    {
        /*cv::Mat outputImage;
        cv::drawKeypoints(im8b,keypoints,outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("Contours on Cropped", outputImage);
        cv::waitKey(1);*/

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



//******************Frame Grabbing Threading Methods*********************

void SetupAndSegment::ReadFrames()
{
    while (_realsenseRunFrameThread)
    {
        {
            std::lock_guard<std::mutex> l{ _realSenseFrameMutex };
            if (_realSense_pipeline.try_wait_for_frames(&_frameset, 1000))
            {
                //frameset = realSenseObj->_realSense_pipeline.wait_for_frames();
            ////Error Checking 
            //if (!frameset) {
                std::cout << "Error: Failed to get frames from RealSense pipeline." << std::endl;
                continue;
            }

            //Gets the IR frames
            if (!_align_to_left_ir) {
                std::cout << "Error: Alignment object not initialized." << std::endl;
                continue;
            }
            _aligned_frameset = _align_to_left_ir->process(_frameset);
            _ir_frame_left = _aligned_frameset.get_infrared_frame(1);
            _ir_frame_right = _aligned_frameset.get_infrared_frame(2);
            if (!_ir_frame_left) {
                std::cout << "Error: Failed to get left IR frame." << std::endl;
                continue;
            }

            //Gets the depth frame
            rs2::depth_frame depth_frame = _aligned_frameset.get_depth_frame();

            if (!depth_frame) {
                std::cout << "Error: Failed to get depth frame." << std::endl;
                continue;
            }
            rs2::frame depth_filtered = depth_frame;
            depth_filtered = _temp_filter.process(depth_filtered);

            //Converts left IR to vector representation //ToDO: Change this so I am not initializing an std:;vector<uint16_t> on every iteration
            auto ir_data = reinterpret_cast<const uint16_t*>(_ir_frame_left.get_data());
            std::vector<uint16_t> ir_vector(ir_data, ir_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
            auto ir_ptr = std::make_unique<std::vector<uint16_t>>(std::move(ir_vector));
        
        
            //Converts depth frame to vector representation //ToDO: Change this so I am not initializing an std:;vector<uint16_t> on every iteration
            auto depth_data = reinterpret_cast<const uint16_t*>(depth_filtered.get_data());
            std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
            auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));

            //Updates the frame queues and locks the mutex before writing to the queue and notifying
        
            std::cout << "pushing to queue" << std::endl;
            //Updates the left IR frame queue
            if (_irLeftFrameQueue.size() > 5) {
                _irLeftFrameQueue.pop(); // Discard the oldest frame if the queue is too large
            }
            _irLeftFrameQueue.push(_ir_frame_left);

            //Updates the right IR frame queue
            if (_irRightFrameQueue.size() > 5) {
                _irRightFrameQueue.pop(); // Discard the oldest frame if the queue is too large
            }
            _irRightFrameQueue.push(_ir_frame_right);

            //Updates the depth frame queue
            if (_depthFrameQueue.size() > 5) {
                _depthFrameQueue.pop(); // Discard the oldest frame if the queue is too large
            }
            _depthFrameQueue.push(depth_frame);

            //Updates the std::unique_ptr <std::vector<uint16_t>> representation of depth and IR left
            if (_depthFilteredPtrQueue.size() > 5) {
                _depthFilteredPtrQueue.pop(); // Discard the oldest frame if the queue is too large
            }
            _depthFilteredPtrQueue.push(std::move(depth_ptr));

            if (_irLeftPtrQueue.size() > 5) {
                _irLeftPtrQueue.pop(); // Discard the oldest frame if the queue is too large
            }
            _irLeftPtrQueue.push(std::move(ir_ptr));
            std::cout << "done pushing to queue" << std::endl;

        }
        _realsenseFrameArrivedVar.notify_one();
    }
}


//Method to call from main to get the realsense data

bool SetupAndSegment::getRealSenseData(
    rs2::frame& ir_frame_left, rs2::frame& ir_frame_right, 
    rs2::depth_frame& depth_frame, 
    std::unique_ptr <std::vector<uint16_t>>& depth_ptr, std::unique_ptr <std::vector<uint16_t>>& ir_ptr)
{
    std::cout << "RealSense Data Grabber Called" << std::endl;
    std::unique_lock<std::mutex> lock{ _realSenseFrameMutex };

    //Waits for a packet from the realsense thread
    if (!_realsenseFrameArrivedVar.wait_for(lock, std::chrono::milliseconds(_timeout), [this]()
        { return (!_irLeftFrameQueue.empty()) && (!_irRightFrameQueue.empty()) && (!_depthFrameQueue.empty())
        && (!_depthFilteredPtrQueue.empty()) && (!_irLeftPtrQueue.empty()); }))
    {
        //We had a timeout event
        return false;
    }
    std::cout << "RealSense Got Packet" << std::endl;

    //Gets the left IR frame
    if (!_irLeftFrameQueue.empty())
    {
        //Grab frame if not empty queue
        ir_frame_left = _irLeftFrameQueue.front();
        _irLeftFrameQueue.pop();
    }
    else {
        return false;
    }

    //Gets the right IR frame
    if (!_irRightFrameQueue.empty())
    {
        //Grab frame if not empty queue
        ir_frame_right = _irRightFrameQueue.front();
        _irRightFrameQueue.pop();
    }
    else {
        return false;
    }

    //Gets the depth frame
    if (!_depthFrameQueue.empty())
    {
        //Grab frame if not empty queue
        depth_frame = _depthFrameQueue.front();
        _depthFrameQueue.pop();
    }
    else {
        return false;
    }

    //Gets the depth filtered vector
    if (!_depthFilteredPtrQueue.empty())
    {
        //Grab frame if not empty queue
        depth_ptr = std::move(_depthFilteredPtrQueue.front());
        _depthFilteredPtrQueue.pop();
        if (!depth_ptr) {  // Validate unique_ptr is not nullptr
            std::cout << "Error: depth_ptr is nullptr after move operation." << std::endl;
            return false;
        }
    }
    else {
        return false;
    }

    //Gets the ir filtered vector
    if (!_irLeftPtrQueue.empty())
    {
        //Grab frame if not empty queue
        ir_ptr = std::move(_irLeftPtrQueue.front());
        _irLeftPtrQueue.pop();

        if (!ir_ptr) {  // Validate unique_ptr is not nullptr
            std::cout << "Error: depth_ptr is nullptr after move operation." << std::endl;
            return false;
        }
    }
    else {
        return false;
    }

    lock.unlock();
    return true;

}



