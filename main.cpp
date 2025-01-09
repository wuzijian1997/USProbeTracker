//Depth is aligned to left infrared frames

//***************Includes*************
#include "PoseTracker.h" //Does US probe pose tracking, runs on separate thread
#include "USVideoStreaming.h" //Streams US video from scrcpy window, runs on separate thread
#include "ShellSensorReader.h" //Reads the force sensor and IMU, runs on separate thread
#include "RealSense.h" //Hardware interface with RealSense D435i camera, runs on separate thread
#include "IRSegmentation.h" //Segments the IR markers in the RealSense image
#include "Utils.h" //Utility functions (i.e. file conversion methods
#include "Datalogger.h" //Logs scanning data to .csv



//**************Init Vars*************

//Defines the marker geometry to track
Eigen::Vector3d marker1(0.06925,0.01133,0);
Eigen::Vector3d marker2(0.03395,-0.02407,0);
Eigen::Vector3d marker3(-0.05605,-0.02329,0);
Eigen::Vector3d marker4(-0.03879,0.04455,0);
auto geom= std::vector<Eigen::Vector3d>{ marker1, marker2, marker3, marker4 };

//Adjustable parameters
std::string data_root_path = "data";
std::string data_participant_directory = "P0"; //Participant number

//Semi-Permanent Setup Parameters
int realsense_timeout = 35; //Realsense Frame Grabber Returns False if waiting more than 35 ms
float pose_markerDiameter = 0.011; //IR Marker diameter in meters
bool pose_filterJumps = true; //Filter jumps in pose tracking 
float pose_jumpThresholdMetres = 0.3;
int pose_numFramesUntilSet = 4;
float pose_smoothing = 0.0f; //We are not smoothing the pose
int forcesensor_timeout = 2; //We wait for 2 ms, force grabber returns NaN's if waiting more than this
int posetracker_timeout = 10; //We wait for 10ms for the pose tracker to update pose
int i;

int main()
{
	//*******************Init RealSense Camera Parameters********************
	//Inits realsense camera object
	RealSense realsense_camera(realsense_timeout); //timeout is the amount of time we wait for realsense thread to update the data
	 
	//Configures the camera
	//These constants are in RealSense.h
	bool realsense_check= realsense_camera.RealSenseInit(REALSENSE_WIDTH,
		REALSENSE_HEIGHT,REALSENSE_FPS, ENABLE_REALSENSE_LASER,REALSENSE_LASER_POWER,
		REALSENSE_GAIN,TEMPORAL_DEPTH_FILTER_ALPHA,TEMPORAL_DEPTH_FILTER_DELTA);


	//OpenCV Matrices of right/left IR
	cv::Mat ir_mat_left, ir_mat_right;
	//Boolean for if realsense data is returned
	bool is_realsense_data_returned = false;
	//object to hold realsense data
	RealSense::RealSenseData realsense_data; 
	
	//Enters if camera is configured correctly
	if (realsense_check)
	{
		std::cout << "RealSense Camera Properly Setup" << std::endl;
		//***************Init IR Segmentation Parameters***********************
		//Creates IR segmentation object, constants are in RealSense.h
		auto ir_segmenter = std::make_shared<IRSegmentation>(REALSENSE_WIDTH, REALSENSE_HEIGHT, realsense_camera._depth_scale,IRSegmentation::LogLevel::Silent);
		ir_segmenter->setDetectionMode(IRSegmentation::DetectionMode::Contour); //Sets the segmentation method
		ir_segmenter->setCameraBoundaries(0,REALSENSE_HEIGHT-1,0,REALSENSE_WIDTH-1,NEAR_CLIP,FAR_CLIP); //Sets the camera boundaries
		
		//Gets RealSense intrinsics for left IR (IR aligned to depth camera)
		double fx = realsense_camera._realSense_intrinsics_leftIR.fx;
		double fy = realsense_camera._realSense_intrinsics_leftIR.fy;
		double cx = realsense_camera._realSense_intrinsics_leftIR.ppx;
		double cy = realsense_camera._realSense_intrinsics_leftIR.ppy;

		//Sets the camera intrinsics in the segmentation object
		ir_segmenter->setCameraIntrinsics([&](const std::array<double, 2>& uv, std::array<double, 2>& xy) {
			xy[0] = (uv[0] - cx) / fx; //Normalized x-coordinate
			xy[1] = (uv[1] - cy) / fy; //Normalized y-coordinate

			});

		//*********************Start the pose calculator************************
		PoseTracker poseTracker(ir_segmenter, geom, pose_markerDiameter); //Starts the pose tracker thread
		poseTracker.setJumpSettings(pose_filterJumps, pose_jumpThresholdMetres, pose_numFramesUntilSet);
		poseTracker.setSmoothing(pose_smoothing);

		//************************Init the Force Sensor************************
		ShellSensorReader shellReader(SHELLSENSOR_PORTNAME, SHELLSENSOR_BAUDRATE, forcesensor_timeout); //Sets the baud rate, timeout is wait time thread before returning NaN's
		//Initializes the force sensor, and checks if the port is connected
		//Starts the force sensor thread
		if (!shellReader.initialize())
		{
			std::cout << "Failed to Initialize Force Sensor Serial Stream" << std::endl;
			return 0;
		}
		
		Eigen::MatrixXd force_calibration_mat = readCSVToEigenMatrix("Resources/calmat.csv", 3, 13); //Read in force calibration matrix		
		Eigen::Vector3d force_zeroing_offset(0.0, 0.0, 0.0); //Zeroing is set to zeros for now		
		std::string raw_force_string, temp_imu_string, force_string_xyz; //String that we read force readings into


		//**********************Init Datalogger***********************
		//Initializes the datalogger object, first string is root directory second string is subdirectory
		//defaults it to data/PXX where XX is the most recent participant number
		Datalogger datalogger(data_root_path, data_participant_directory, force_calibration_mat, force_zeroing_offset);


		//********************Init variables to display pose***********
		cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
			0, fy, cy,
			0, 0, 1);

		cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
			realsense_camera._realSense_intrinsics_leftIR.coeffs[0], // k1
			realsense_camera._realSense_intrinsics_leftIR.coeffs[1], // k2
			realsense_camera._realSense_intrinsics_leftIR.coeffs[2], // p1
			realsense_camera._realSense_intrinsics_leftIR.coeffs[3], // p2
			realsense_camera._realSense_intrinsics_leftIR.coeffs[4]  // k3
			);

		cv::Mat rotation;
		cv::Mat translation;
		cv::Mat rvec;		


		//********************Init Frame Counter Vars**********************
		int realsense_frame_count = 0;
		int us_frame_count = 0;

		//Start the realsense thread
		realsense_camera.start();
		while (true)
		{			
			is_realsense_data_returned =realsense_camera.getRealSenseData(realsense_data);
			
			//!!!Data Collection Syncrhonized to RealSense!!!
			if (is_realsense_data_returned) //Enters if depth/ir frames arrive
			{
				//***********************RealSense Data Conversions******************
				//Converts left IR to vector representation (for pose tracker)
				auto ir_data = reinterpret_cast<const uint8_t*>(realsense_data.irLeftFrame.get_data());
				std::vector<uint8_t> ir_vector(ir_data, ir_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto ir_ptr = std::make_unique<std::vector<uint8_t>>(std::move(ir_vector));

				//Converts depth frame to vector representation 
				auto depth_data = reinterpret_cast<const uint16_t*>(realsense_data.depthFrameFiltered.get_data());
				std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));
				//std::cout << "Converted Frames" << std::endl;

				//Converts IR to OpenCV representation
				//ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irLeftFrame.get_data());
				//ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irRightFrame.get_data());


				//***********************Compute the Pose***********************
				//Update the pose tracker with new realsense frames
				poseTracker.update(std::move(ir_ptr), std::move(depth_ptr));

				for (i = 0; i < 10; i++) {
					if (poseTracker.hasNewPose()) break; //Breaks if new pose is calculated
					using namespace std::chrono_literals;
					std::this_thread::sleep_for(); //Sleeps main to wait for new pose
				}

				if (i == 10) {
					//Failed to compute pose, make the matrix all "-1's" to indicate it is false
					std::cout << "Failed to compute pose" << std::endl;
				}
				else {
					Eigen::Matrix4d T = poseTracker.getPose();
				}






				//	cv::Mat cvT(4, 4, CV_64F);
				//	for (int i = 0; i < 4; ++i) {
				//		for (int j = 0; j < 4; ++j) {
				//			cvT.at<double>(i, j) = T(i, j);
				//		}
				//	}

				//	//Displaying image
				//	//rotation = cvT(cv::Range(0, 3), cv::Range(0, 3));
				//	//translation = cvT(cv::Range(0, 3), cv::Range(3, 4));
				//	//cv::Mat outputImage;
				//	//cv::cvtColor(ir_mat_left, outputImage, cv::COLOR_GRAY2BGR);
				//	//cv::Rodrigues(rotation, rvec);
				//	//cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, translation, 0.1, 3);
				//	//cv::imshow("Pose Visualization", outputImage);
				//	//
				//	//for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
				//	//		// draw the point on the image (circle with radius 3, red color)
				//	//		cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
				//	//}



				//	
				//}


				//***************Force Readings***************
				shellReader.getForceString(raw_force_string); //Gets most recent force string
				shellReader.getTempIMUString(temp_imu_string); //Gets most recent Temperature + IMU String


				//Converts raw force values (binary) to estimated force values, if raw forces are NaN's then NaN's are returned
				force_string_xyz=calculateForceVals(raw_force_string, force_calibration_mat, force_zeroing_offset);
				//std::cout << "Raw Force Reading: " << raw_force_string << ", XYZ Force Reading: " << force_string_xyz << std::endl;


				//***************Logging Data******************
				Eigen::Matrix4d dummyPose;
				dummyPose << 1, 0, 0, 5,
							0, 1, 0, 10,
							0, 0, 1, 15,
							0, 0, 0, 1;
				double dummy_seconds = 1.0f;
				int dummy_frame_num = 1;


				datalogger.writeCSVRow(dummy_seconds, realsense_frame_count, dummy_frame_num, dummyPose, raw_force_string, force_string_xyz, temp_imu_string);
				
				//Writes the depth frame to the depth video
				cv::Mat depth_mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_16UC1, (void*)realsense_data.depthFrame.get_data(), cv::Mat::AUTO_STEP);

				datalogger.writeDepthFrame(depth_mat);
				realsense_frame_count++; //Increments the depth frame counter



				//For Display
				//cv::imshow("ir mat left", ir_mat_left);
				//char c = cv::waitKey(1);	//grabs key press, if q we close
				//if (c == 'q')
				//{
				//	break;

				//}

				


				//std::cout << "Converted mat" << std::endl;
				//auto detection = ir_segmenter->findKeypointsWorldFrame(std::move(ir_ptr), std::move(depth_ptr));
				//std::cout << "Got Keypoints" << std::endl;
				////std::cout << "Success" << std::endl;
				//for (const auto& coord : detection->imCoords) {
				//	 //draw the point on the image (circle with radius 3, red color)
				//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
				//}

				//cv::imshow("left ir", ir_mat_left);
				////cv::imshow("right ir", ir_mat_right);
				//char c = cv::waitKey(1);	//grabs key press, if q we close
				//if (c == 'q')
				//{
				//	break;

				//}

			}

		}
		realsense_camera.stop();
	}
	



	return 0;
}




//*********************************Commented Code*********************************
//*****************Init Vars****************
//rs2::frameset frameset, aligned_framset; //Holds RealSense Frame Data
//rs2::frame ir_frame_left, ir_frame_right;
//rs2::depth_frame depth_frame=rs2::frame();
//std::unique_ptr <std::vector<uint16_t>> depth_ptr;
//std::unique_ptr <std::vector<uint16_t>> ir_ptr;
//
//
//cv::Mat ir_mat_left, ir_mat_right; //OpenCV Matrices of right/left
//
//cv::Mat US_frame;
//
//bool continueUS = true;

//std::chrono::steady_clock::time_point last_time;
//std::chrono::steady_clock::time_point curr_time;
//last_time = std::chrono::steady_clock::now();
//curr_time = std::chrono::steady_clock::now();
//auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
//std::cout << "dt:" << elapsed_ms << std::endl;


	//Constructs the ShellSensor Serial Streamer Object
	// 
	//ShellSensorReader shellReader(SHELLSENSOR_PORTNAME, SHELLSENSOR_BAUDRATE,10); //Sets the baud rate, timeout is wait time thread before returning NaN's
	////Initializes parameters, and checks if the port is connected
	//if (!shellReader.initialize())
	//{
	//	std::cout << "Failed to Initialize Serial Port" << std::endl;
	//	return 0;
	//}

	////Init strings that we read into
	//std::string force_string,temp_imu_string;
	//while (true)
	//{
	//	//Gets the strings of data from the force sensor
	//	
	//	shellReader.getForceString(force_string);
	//	shellReader.getTempIMUString(temp_imu_string);

	//	std::cout << "Force String: " << force_string << std::endl;
	//	std::cout << "Temp/IMU String: " << temp_imu_string << std::endl;

	//}

	


	//Inits the US Video Streamer Object and stream thread
	//USVideoStreaming USStreamer(true,10); //Shows the US stream when true, 20 ms timeout for reading from thread
	//
	//while (true)
	//{
	//	cv::Mat usFrame = USStreamer.getFrame(); //Gets most recent frame

	//	if (usFrame.empty())
	//	{
	//		//do something here later
	//		std::cout << "Empty US Frame" << std::endl;
	//		continue;
	//	}
	//	continueUS =USStreamer.showFrame();

	//	if(!continueUS) //The US Stream Window was closed via 'Esc'
	//	{
	//		break;
	//	}

	//}


	////Sets up the marker geometry
	//Eigen::Vector3d marker1(0.0157,0.0154,0);
	//Eigen::Vector3d marker2(0.0358,-0.023,0);
	//Eigen::Vector3d marker3(-0.0366,-0.0237,0);
	//Eigen::Vector3d marker4(-0.0247,0.0121,0);
	//auto geom= std::vector<Eigen::Vector3d>{ marker1, marker2, marker3, marker4 };


	//Inits the setup/segmentation object
	//auto realSenseObj=std::make_shared<SetupAndSegment>(REALSENSE_WIDTH,REALSENSE_HEIGHT,SetupAndSegment::LogLevel::Silent);
	////bool GPU_check = realSenseObj->GPUSetup();
	//bool GPU_check = true;

//	if (GPU_check)
//	{
//		//Config RealSense
//		bool RealSense_check=realSenseObj->RealSenseSetup(20); //20 is the amount of time the we wait for thread to update before returning false
//		std::cout << "Setup" << std::endl;
//		if (RealSense_check)
//		{
//			//Set up tracking parameters
//			realSenseObj->setDetectionMode(SetupAndSegment::DetectionMode::Blob); //Sets the segmentation method
//			realSenseObj->setCameraBoundaries(0,REALSENSE_HEIGHT-1,0,REALSENSE_WIDTH-1,NEAR_CLIP,FAR_CLIP); //Sets the camera boundaries
//			//double fx = realSenseObj->_realSense_intrinsics_leftIR.fx;
//			//double fy = realSenseObj->_realSense_intrinsics_leftIR.fy;
//			//double cx = realSenseObj->_realSense_intrinsics_leftIR.ppx;
//			//double cy = realSenseObj->_realSense_intrinsics_leftIR.ppy;
//
//			////Sets the camera intinsics function
//			//realSenseObj->setCameraIntrinsics([&](const std::array<double, 2>& uv, std::array<double, 2>& xy) {
//			//	xy[0] = (uv[0] - cx) / fx; //Normalized x-coordinate
//			//	xy[1] = (uv[1] - cy) / fy; //Normalized y-coordinate
//
//			//	});
//			std::cout << "Object Init Successful" << std::endl;
//
//			//Set up the pose calculator
//			//PoseTracker poseTracker(realSenseObj, geom,0.011);
//			//poseTracker.setJumpSettings(false,0.1,4);
//			//poseTracker.setSmoothing(0);
//
//			//Init some variables for display:
//			//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
//			//	0, fy, cy,
//			//	0, 0, 1);
//
//			//cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
//			//	realSenseObj->_realSense_intrinsics_leftIR.coeffs[0], // k1
//			//	realSenseObj->_realSense_intrinsics_leftIR.coeffs[1], // k2
//			//	realSenseObj->_realSense_intrinsics_leftIR.coeffs[2], // p1
//			//	realSenseObj->_realSense_intrinsics_leftIR.coeffs[3], // p2
//			//	realSenseObj->_realSense_intrinsics_leftIR.coeffs[4]  // k3
//			//	);
//			//cv::Mat rotation;
//			//cv::Mat translation;
//			//cv::Mat rvec;
//			
//			//Execution Loop
//			bool data_returned = false;
//			while (1)
//			{
//				data_returned=realSenseObj->getRealSenseData(ir_frame_left, ir_frame_right, depth_frame, depth_ptr, ir_ptr);
//				//************Grabbing RealSense Frames***************
//
//				//if(!realSenseObj->_realSense_pipeline.try_wait_for_frames(&frameset,1000))
//				//{
//				//	//frameset = realSenseObj->_realSense_pipeline.wait_for_frames();
//				//////Error Checking 
//				////if (!frameset) {
//				//	std::cout << "Error: Failed to get frames from RealSense pipeline." << std::endl;
//				//	continue;
//				//}
//
//				//aligned_framset = realSenseObj->_align_to_left_ir->process(frameset);
//
//				////Get left and right infrared frames, depth info is aligned to left frame
//				////ir_frame_left = aligned_framset.get_infrared_frame(1);
//				////if (!ir_frame_left) {
//				////	std::cout << "Error: Failed to get left IR frame." << std::endl;
//				////	continue;
//				////}
//
//				////ir_frame_right = aligned_framset.get_infrared_frame(2);
//				//ir_frame_left = aligned_framset.get_infrared_frame(1);
//				//ir_frame_right = aligned_framset.get_infrared_frame(2);
//				//if (!ir_frame_left) {
//				//	std::cout << "Error: Failed to get left IR frame." << std::endl;
//				//	continue;
//				//}
//
//
//				////Converts Right Infrared data to vector representation
//
//				//auto ir_data = reinterpret_cast<const uint16_t*>(ir_frame_left.get_data());
//				//std::vector<uint16_t> ir_vector(ir_data, ir_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
//				//auto ir_ptr = std::make_unique<std::vector<uint16_t>>(std::move(ir_vector));
//
//				////Gets the depth frame
//				//rs2::depth_frame depth_frame = aligned_framset.get_depth_frame();
//
//				//if (!depth_frame) {
//				//	std::cout << "Error: Failed to get depth frame." << std::endl;
//				//	continue;
//				//}
//				//rs2::frame depth_filtered = depth_frame; 
//				//depth_filtered = realSenseObj->_temp_filter.process(depth_filtered);
//
//				////Converts depth frame to vector representation
//				//auto depth_data = reinterpret_cast<const uint16_t*>(depth_filtered.get_data());
//				//std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
//				//auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));
//
//				if (data_returned)
//				{
//					ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_left.get_data());
//					cv::imshow("left ir", ir_mat_left);
//					char c = cv::waitKey(1);	//grabs key press, if q we close
//					if (c == 'q')
//					{
//						break;
//
//					}
//
//				}
//				////Converts IR to OpenCV Mat:
//				//ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_left.get_data());
//				////ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_right.get_data());
//
//				//auto detection = realSenseObj->findKeypointsWorldFrame(std::move(ir_ptr), std::move(depth_ptr));
//
//				//////std::cout << "Success" << std::endl;
//				//for (const auto& coord : detection->imCoords) {
//				//	// draw the point on the image (circle with radius 3, red color)
//				//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
//				//}
//
//				//cv::imshow("left ir", ir_mat_left);
//				//char c = cv::waitKey(1);	//grabs key press, if q we close
//				//if (c == 'q')
//				//{
//				//	break;
//
//				//}
//
//
//				//Finding Pose
//				//poseTracker.update(std::move(ir_ptr), std::move(depth_ptr));
//				//if (poseTracker.hasNewPose())
//				//{
//				//	Eigen::Matrix4d T = poseTracker.getPose();
//				//	//PoseTracker::IRPose mes = poseTracker.getLastMeasurement();
//				//	//std::cout << "Computed pose" << std::endl;
//				//	//std::cout << T << std::endl;
//
//				//	for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
//				//	// draw the point on the image (circle with radius 3, red color)
//				//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
//				//	}
//
//				//	cv::imshow("keypoints", ir_mat_left);
//				//	char c = cv::waitKey(1);	//grabs key press, if q we close
//				//	if (c == 'q')
//				//	{
//				//		break;
//
//				//	}
//
//				//}
//				
//				//int i;
//				//for (i = 0; i < 10; i++) {
//				//	if (poseTracker.hasNewPose()) break;
//				//	using namespace std::chrono_literals;
//				//	std::this_thread::sleep_for(10ms);
//				//}
//
//				//if (i == 10) {
//				//	std::cout << "Failed to compute pose" << std::endl;
//				//}
//				//else {
//				//	Eigen::Matrix4d T = poseTracker.getPose();
//				//	//PoseTracker::IRPose mes = poseTracker.getLastMeasurement();
//				//	std::cout << "Computed pose" << std::endl;
//				//	//std::cout << T << std::endl;
//				//	//std::cout << mes.pose.matrix() << std::endl;
//
//				//	cv::Mat cvT(4, 4, CV_64F);
//				//	for (int i = 0; i < 4; ++i) {
//				//		for (int j = 0; j < 4; ++j) {
//				//			cvT.at<double>(i, j) = T(i, j);
//				//		}
//				//	}
//
//				//	rotation = cvT(cv::Range(0, 3), cv::Range(0, 3));
//				//	translation = cvT(cv::Range(0, 3), cv::Range(3, 4));
//				//	cv::Mat outputImage;
//				//	cv::cvtColor(ir_mat_left, outputImage, cv::COLOR_GRAY2BGR);
//				//	cv::Rodrigues(rotation, rvec);
//				//	cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, translation, 0.1, 3);
//				//	cv::imshow("Pose Visualization", outputImage);
//				//	cv::waitKey(1);
//
//
//
//
//				//	//for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
//				//	//		// draw the point on the image (circle with radius 3, red color)
//				//	//		cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
//				//	//}
//
//				//	//cv::imshow("keypoints", ir_mat_left);
//				//	//cv::waitKey(1);	//grabs key press, if q we close
//				//}
//
//				//Finding Keypoints
//
//				
//
//
//				//****************Segmenting Markers******************
//				//std::vector<cv::KeyPoint> keypoints;
//				//bool success = realSenseObj.contourDetect(ir_mat_right, keypoints);
//				//if (success)
//				//{
//				//	//Draw Keypoints
//				//	cv::Mat outputImage;
//				//	cv::drawKeypoints(ir_mat_right,keypoints,outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//				//	cv::imshow("Keypoints", outputImage);
//				//	char c = cv::waitKey(1);	//Grabs Key Press, if q we close
//				//	if (c == 'q')
//				//		break;
//
//				//}
//
//				
//				
//			}
//
//			//Release RealSense Objects
//			/*realSenseObj->_realSense_pipeline.stop();*/
//
//		}
//
//	}
//
//	return 0;
//}