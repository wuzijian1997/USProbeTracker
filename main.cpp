//Depth is aligned to left infrared frames

//***************Includes*************
#include "PoseTracker.h" //Does US probe pose tracking, runs on separate thread
#include "USVideoStreaming.h" //Streams US video from scrcpy window, runs on separate thread
#include "ShellSensorReader.h" //Reads the force sensor and IMU, runs on separate thread
#include "RealSense.h" //Hardware interface with RealSense D435i camera, runs on separate thread
#include "IRSegmentation.h" //Segments the IR markers in the RealSense image
#include "Utils.h" //Utility functions (i.e. file conversion methods
#include "Datalogger.h" //Logs scanning data to .csv
#include <conio.h> // For non-blocking keyboard input (_kbhit(), _getch())



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
std::string stereo_camera_calib_file = "StereoCalibration\\CalibFiles\\Calib\\calibration_params_2.yaml";

bool show_us_stream = true; //Show the us stream 
bool show_pose = true; //Show the pose on entire image
bool show_clip_area_andkeypoints = false; //Show the clipped area around the marker, also show keypoints
bool show_ir = true; //Shows the left ir frame
bool show_depth = false; //shows the depth map
std::vector<std::string> landmarkVector = { "Xyphoid","Naval","LeftMid","RightMid" };

//Semi-Permanent Setup Parameters
int realsense_timeout = 35; //Realsense Frame Grabber Returns False if waiting more than 35 ms

float pose_markerDiameter = 0.011f; //IR Marker diameter in meters
bool pose_filterJumps = true; //Filter jumps in pose tracking 
float pose_jumpThresholdMetres = 0.3;
int pose_numFramesUntilSet = 4;
float pose_smoothing = 0.0f; //We are not smoothing the pose
int posetracker_timeout = 5; //We wait for 5ms for the pose tracker to update pose

int forcesensor_timeout = 2; //We wait for 2 ms, force grabber returns NaN's if waiting more than this

int us_timeout = 2; //We wait for 2 ms for us frames

//****Variables for Tracking Time******
std::chrono::steady_clock::time_point last_loop_time;
std::chrono::steady_clock::time_point last_time;
std::chrono::steady_clock::time_point curr_loop_time;
std::chrono::steady_clock::time_point curr_time;

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
		REALSENSE_GAIN,TEMPORAL_DEPTH_FILTER_ALPHA,TEMPORAL_DEPTH_FILTER_DELTA,REALSENSE_AUTOEXPOSURE_ENABLE,REALSENSE_EXPOSURE_LEVEL);


	//OpenCV Matrices of right/left IR
	cv::Mat ir_mat_left, ir_mat_right;
	cv::Mat colour_frame;
	//Vars for displaying
	cv::Mat depth_normalized;
	//double minVal, maxVal;
	cv::Mat depth_colormap;
	//cv::Mat depth_bgr(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT),CV_8UC3); //We encode the depth data into 3 8-bit channels. 

	cv::Size realsense_framesize = cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT);

	//Boolean for if realsense data is returned
	bool is_realsense_data_returned = false;
	//object to hold realsense data
	RealSense::RealSenseData realsense_data; 
	Eigen::Matrix4d cam_T_us; //The pose that we are tracking
	
	//Enters if camera is configured correctly
	if (realsense_check)
	{
		std::cout << "RealSense Camera Properly Setup" << std::endl;
		//***************Init IR Segmentation Parameters and Camera Intrinsics***********************
		
		//*****Gets the camera intrinsics
		cv::FileStorage fs(stereo_camera_calib_file, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cout << "Unable to open calibration file" << std::endl;
			return 0;
		}

		//declare the cv::Mat objects with the calibration parameters
		cv::Mat left_camera_matrix, left_dist_coeffs;
		cv::Mat right_camera_matrix, right_dist_coeffs;
		cv::Mat R_cam, T_cam, E_cam, F_cam;

		// Read matrices from the file
		fs["left_mat"] >> left_camera_matrix;
		fs["left_dist"] >> left_dist_coeffs;
		fs["right_mat"] >> right_camera_matrix;
		fs["right_dist"] >> right_dist_coeffs;
		fs["R"] >> R_cam;
		fs["T"] >> T_cam;
		fs["E"] >> E_cam;
		fs["F"] >> F_cam;

		fs.release();

		//Converts cv::Mat camera parameters to strings to store the calibration params
		std::string left_mat_str = openCVMatToCSVString(left_camera_matrix);
		std::string left_dist_str = openCVMatToCSVString(left_dist_coeffs);
		std::string right_mat_str = openCVMatToCSVString(right_camera_matrix);
		std::string right_dist_str = openCVMatToCSVString(right_dist_coeffs);

		std::stringstream left_camera_stream;
		left_camera_stream << left_mat_str << "," << left_dist_str;
		std::string left_camera_intrinsics = left_camera_stream.str();

		std::stringstream right_camera_stream;
		right_camera_stream << right_mat_str << "," << right_dist_str;
		std::string right_camera_intrinsics = right_camera_stream.str();
		
		std::string R_mat_str = openCVMatToCSVString(R_cam);
		std::string T_mat_str = openCVMatToCSVString(T_cam);
		std::string E_mat_str = openCVMatToCSVString(E_cam);
		std::string F_mat_str = openCVMatToCSVString(F_cam);

		/*std::string left_camera_intrinsics = std::to_string(left_camera_matrix.at<double>(0, 0)) + "," +
			std::to_string(left_camera_matrix.at<double>(1, 1)) + "," + std::to_string(left_camera_matrix.at<double>(0, 2)) +
			"," + std::to_string(left_camera_matrix.at<double>(1, 2)) + "," +
			std::to_string(left_dist_coeffs.at<double>(0, 0)) + "," + std::to_string(left_dist_coeffs.at<double>(0, 1)) + "," +
			std::to_string(left_dist_coeffs.at<double>(0, 2)) + "," + std::to_string(left_dist_coeffs.at<double>(0, 3)) + "," +
			std::to_string(left_dist_coeffs.at<double>(0, 4));

		std::string right_camera_intrinsics = std::to_string(right_camera_matrix.at<double>(0, 0)) + "," +
			std::to_string(right_camera_matrix.at<double>(1, 1)) + "," + std::to_string(right_camera_matrix.at<double>(0, 2)) +
			"," + std::to_string(right_camera_matrix.at<double>(1, 2)) + "," +
			std::to_string(right_dist_coeffs.at<double>(0, 0)) + "," + std::to_string(right_dist_coeffs.at<double>(0, 1)) + "," +
			std::to_string(right_dist_coeffs.at<double>(0, 2)) + "," + std::to_string(right_dist_coeffs.at<double>(0, 3)) + "," +
			std::to_string(right_dist_coeffs.at<double>(0, 4));*/

		//To do: add the R, T, E, and F matrices to the .csv as well


		//Creates IR segmentation object, constants are in RealSense.h
		auto ir_segmenter = std::make_shared<IRSegmentation>(REALSENSE_WIDTH, REALSENSE_HEIGHT, realsense_camera._depth_scale,IRSegmentation::LogLevel::Silent,show_clip_area_andkeypoints,
			left_camera_matrix,left_dist_coeffs,right_camera_matrix,right_dist_coeffs,R_cam,T_cam,E_cam,F_cam);
		
		ir_segmenter->setDetectionMode(IRSegmentation::DetectionMode::Contour); //Sets the segmentation method
		ir_segmenter->setCameraBoundaries(0,REALSENSE_HEIGHT-1,0,REALSENSE_WIDTH-1,NEAR_CLIP,FAR_CLIP); //Sets the camera boundaries	

		
		
		//*********************Start the pose calculator************************
		PoseTracker poseTracker(ir_segmenter, geom, pose_markerDiameter); //Starts the pose tracker thread
		poseTracker.setJumpSettings(pose_filterJumps, pose_jumpThresholdMetres, pose_numFramesUntilSet);
		poseTracker.setSmoothing(pose_smoothing);


		//************************Init the Force Sensor************************
		Eigen::MatrixXd force_calibration_mat = readCSVToEigenMatrix("Resources/calmat.csv", 3, 13); //Read in force calibration matrix		
		Eigen::Vector3d force_zeroing_offset(0.0, 0.0, 0.0); //Zeroing is set to zeros for now until we do zeroing below
		Eigen::MatrixXd force_compensation_mat = readCSVToEigenMatrix("Resources/compensationmat_1.csv", 3, 3);
		std::string raw_force_string, temp_imu_string, force_string_xyz; //String that we read force readings into
		ShellSensorReader shellReader(SHELLSENSOR_PORTNAME, SHELLSENSOR_BAUDRATE, forcesensor_timeout,force_calibration_mat,force_zeroing_offset,force_compensation_mat); //Sets the baud rate, timeout is wait time thread before returning NaN's
		//Initializes the force sensor, and checks if the port is connected
		//Starts the force sensor thread
		if (!shellReader.initialize())
		{
			std::cout << "Failed to Initialize Force Sensor Serial Stream" << std::endl;
			return 0;
		}		
		


		//***********************Init the US Frame Grabber*********************
		USVideoStreaming USStreamer(show_us_stream, us_timeout); //Shows US stream when true, timeout for frabbing from the us thread

		//********************Init Vars**********************
		int realsense_frame_count = -1; //-1 Denotes frames haven't started
		int us_frame_count = -1; //-1 Denotes frames haven't started

		//Init variables to display pose
		cv::Mat rotation;
		cv::Mat translation;
		cv::Mat rvec;

		//Init the clock recording time
		auto current_time = std::chrono::high_resolution_clock::now();
		auto start_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> elapsed_time;
		float elapsed_seconds;

		int MAX_LANDMARKS = landmarkVector.size(); //How many landmarks we set at the start
		int landmark_counter = 0;
		std::string landmark_string = "-1";
		
		

		//***********User Input for Zeroing the Force Vector**********
		std::cout << "~~~~~~~~Move ultrasound probe to zero forces and press Enter to log~~~~~~~~\n";
		std::cin.get(); //Wait for user to press enter
		shellReader.getForceString(raw_force_string, force_string_xyz);
		//Updates the zeroing offset to the recorded force value
		shellReader._force_zeroing_offset = XYZforcestringToForceXYZVector(force_string_xyz);
		force_zeroing_offset = shellReader._force_zeroing_offset;
		shellReader.getForceString(raw_force_string, force_string_xyz); //Extra call to get rid of spurious measurement due to threading

		//Start the realsense thread
		realsense_camera.start();

		//**********************Init Datalogger***********************
		//Initializes the datalogger object, first string is root directory second string is subdirectory
		//defaults it to data/PXX where XX is the most recent participant number
		Datalogger datalogger(data_root_path, data_participant_directory, force_calibration_mat, force_zeroing_offset, force_compensation_mat,
			left_camera_intrinsics, right_camera_intrinsics, R_mat_str, T_mat_str, E_mat_str, F_mat_str,
			realsense_camera._depth_scale, REALSENSE_WIDTH, REALSENSE_HEIGHT, REALSENSE_FPS, USStreamer._windowWidth_original, USStreamer._windowHeight_original, REALSENSE_FPS, ir_segmenter);

		//******************Anatomy Landmarks Prompts*****************
		std::cout << "~~~~~~~~Move ultrasound probe to collect landmarks~~~~~~~~\n";
		std::cout << "Press Enter to record pose of: " << landmarkVector[landmark_counter] <<"\n";
		

		while (true)
		{		
			last_loop_time = std::chrono::steady_clock::now();
			//*****************Get RealSense Data******************			
			is_realsense_data_returned =realsense_camera.getRealSenseData(realsense_data);
			
			
			//!!!Data Collection Synchronized to RealSense!!!
			
			if (is_realsense_data_returned) //Enters if depth/ir frames arrive
			{
				//***********************RealSense Data Conversions******************
				//last_time = std::chrono::steady_clock::now();
				//Converts left IR to vector representation (for pose tracker)
				auto ir_data_left = reinterpret_cast<const uint8_t*>(realsense_data.irLeftFrame.get_data());
				std::vector<uint8_t> ir_vector_left(ir_data_left, ir_data_left + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto ir_ptr_left = std::make_unique<std::vector<uint8_t>>(std::move(ir_vector_left));

				//Converts right IR to vector respresentation (for pose tracker)
				auto ir_data_right = reinterpret_cast<const uint8_t*>(realsense_data.irRightFrame.get_data());
				std::vector<uint8_t> ir_vector_right(ir_data_right, ir_data_right + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto ir_ptr_right = std::make_unique<std::vector<uint8_t>>(std::move(ir_vector_right));
				/*curr_time = std::chrono::steady_clock::now();
				auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "RealSense Data Conversions dt: " << elapsed_ms << std::endl;*/

				//**********************Update Pose Tracker********************
				//Update the pose tracker with new realsense frames
				//Updates the pose tracker
				//last_time = std::chrono::steady_clock::now();
				poseTracker.update(std::move(ir_ptr_left), std::move(ir_ptr_right));
				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();*/
				//std::cout << "Update Pose dt: " << elapsed_ms << std::endl;
				
				

				//************************Get Force/IMU************************
				//last_time = std::chrono::steady_clock::now();
				shellReader.getForceString(raw_force_string,force_string_xyz); //Gets most recent force string
				shellReader.getTempIMUString(temp_imu_string); //Gets most recent Temperature + IMU String

				//std::cout << "Raw Force Reading: " << raw_force_string << ", XYZ Force Reading: " << force_string_xyz << std::endl;

				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "Get Force dt: " << elapsed_ms << std::endl;*/

				//************************Get US Frame*************************
				//last_time = std::chrono::steady_clock::now();
				cv::Mat usFrame = USStreamer.getFrame(); //Gets most recent us frame
				if (!usFrame.empty()) //If the ultraasound frame is not empty, we write the US frame and increment us counter
				{
					us_frame_count++;
					datalogger.writeUSFrame(usFrame);
					if (show_us_stream)
					{
						USStreamer.showFrame();
					}
				}

				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "US dt: " << elapsed_ms << std::endl;*/


				//****************RealSense OpenCV Conversions*****************
				//last_time = std::chrono::steady_clock::now();
				//Converts depth frame to vector representation 
				/*auto depth_data = reinterpret_cast<const uint16_t*>(realsense_data.depthFrameFiltered.get_data());
				std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));*/
				cv::Mat depth_mat(realsense_framesize, CV_16UC1, (void*)realsense_data.depthFrame.get_data(), cv::Mat::AUTO_STEP);
				//cv::Mat depth_new(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT),CV_8UC3, (void*)realsense_data.depthFrame.get_data(), cv::Mat::AUTO_STEP);

				//Split depth into blue and green channels (8 bits to blue, 4 bits to red)
				//for (int y = 0; y < depth_mat.rows; y++) {
				//	for (int x = 0; x < depth_mat.cols; x++) {
				//		uint16_t depth_val = depth_mat.at<uint16_t>(y, x);
				//		depth_bgr.at<cv::Vec3b>(y, x)[0] = (depth_val >> 4) & 0xFF; // Blue channel (MSB)
				//		depth_bgr.at<cv::Vec3b>(y, x)[1] = (depth_val & 0xF) << 4;  // Green channel (LSB)
				//		depth_bgr.at<cv::Vec3b>(y, x)[2] = 0; // Blue channel (optional)
				//	}
				//}

				//IR Frames to OpenCV
				ir_mat_left = cv::Mat(realsense_framesize, CV_8UC1, (void*)realsense_data.irLeftFrame.get_data(), cv::Mat::AUTO_STEP);
				ir_mat_right = cv::Mat(realsense_framesize, CV_8UC1, (void*)realsense_data.irRightFrame.get_data(), cv::Mat::AUTO_STEP);

				//RGB Frame to OpenCV
				colour_frame = cv::Mat(realsense_framesize, CV_8UC3, (void*)realsense_data.colourFrame.get_data(), cv::Mat::AUTO_STEP);
				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "Data Conversions dt: " << elapsed_ms << std::endl;*/

				//**********************Logging RealSense Frames*************************
				//last_time = std::chrono::steady_clock::now();
				datalogger.writeDepthFrame(depth_mat); //Writes depth frame
				datalogger.writeIRFrames(ir_mat_left, ir_mat_right); //Writes left/right ir
				datalogger.writeColourFrame(colour_frame);
				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "Frame Writing dt: " << elapsed_ms << std::endl;*/

				//************************Get Pose*****************************				
				//Waits for pose calculator to grab pose
				//last_time = std::chrono::steady_clock::now();
				std::unique_lock<std::mutex> lock{ poseTracker.m_poseMutex };
				bool poseReceived = poseTracker.m_pose_here_CV.wait_for(lock, std::chrono::milliseconds(posetracker_timeout), [&] {return poseTracker.hasNewPose(); });
				lock.unlock();				

				//Check if pose is computed
				if (!poseReceived) {
					//Failed to compute pose, make the matrix all "-1's" to indicate it is false			
					//std::cout << "Failed Pose Computation" << std::endl;
					cam_T_us << -1, -1, -1, -1,
						-1, -1, -1, -1,
						-1, -1, -1, -1,
						-1, -1, -1, -1;
				}
				else {
					//Computes Pose
					cam_T_us = poseTracker.getPose();
				}
				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "Pose dt: " << elapsed_ms << std::endl;*/

				//********************Checking for Enter Press for Landmarks*****************
				if (_kbhit() && (_getch() == '\r')&& (landmark_counter<MAX_LANDMARKS)) //if enter is pressed, record landmark in landmark_string
				{
					landmark_string = landmarkVector[landmark_counter];
					landmark_counter++;
					if (landmark_counter < MAX_LANDMARKS)
					{
						std::cout << "Captured Landmark" << std::endl;
						std::cout << "Press Enter to record pose of: " << landmarkVector[landmark_counter] << "\n";
					}
					else {
						std::cout << "Captured All Landmarks" << "\n";
						std::cout << "~~~~~~~~~~~~~~~~~~~~~~" << "\n";
					}

				}
				else
				{
					landmark_string = "-1";
				}


				//************************Logging Pose/Force/IMU Data*************************
				//last_time = std::chrono::steady_clock::now();
				// 
				////get the time since running
				auto current_time = std::chrono::high_resolution_clock::now();
				elapsed_time = current_time - start_time;
				elapsed_seconds = elapsed_time.count();
				realsense_frame_count++; //Increments the depth frame counter
				////Writes pose/force to scandata_datetime.csv
				datalogger.writeCSVRow(elapsed_seconds, realsense_frame_count, us_frame_count, cam_T_us, raw_force_string, force_string_xyz, temp_imu_string, landmark_string);
				
				/*curr_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				std::cout << "CSV Logging dt: " << elapsed_ms<<std::endl;*/
				
				

				/*curr_loop_time = std::chrono::steady_clock::now();
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_loop_time - last_loop_time).count();
				std::cout << "Loop Time dt: " << elapsed_ms << std::endl;*/

				//************************Displaying Frames********************
				if (show_pose) //Shows the pose
				{
						cv::Mat cvT(4, 4, CV_64F);
						for (int i = 0; i < 4; ++i) {
							for (int j = 0; j < 4; ++j) {
								cvT.at<double>(i, j) = cam_T_us(i, j);
							}
						}

						//Displaying image
						rotation = cvT(cv::Range(0, 3), cv::Range(0, 3));
						translation = cvT(cv::Range(0, 3), cv::Range(3, 4));
						cv::Mat outputImage;
						cv::cvtColor(ir_mat_left, outputImage, cv::COLOR_GRAY2BGR);
						cv::Rodrigues(rotation, rvec);
						cv::drawFrameAxes(outputImage, left_camera_matrix, left_dist_coeffs, rvec, translation, 0.1, 3);
						cv::imshow("Pose Visualization", outputImage);
						if (!(show_ir || show_clip_area_andkeypoints))
						{
							char c = cv::waitKey(1);	//grabs key press, if q we close
							if (c == 'q')
							{
								break;

							}

						}
				}

				if (show_clip_area_andkeypoints) //Shows the keypoints
				{
					for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
							// draw the point on the image (circle with radius 3, red color)
							cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
					}
				}
				if (show_ir || show_clip_area_andkeypoints) //Shows the ir image
				{

					cv::imshow("IR Left", ir_mat_left);
					char c = cv::waitKey(1);	//grabs key press, if q we close
					if (c == 'q')
					{
						break;

					}
				}

				if (show_depth) //Shows the depth frame from the realsense
				{					
					//cv::minMaxIdx(depth_mat, &minVal, &maxVal); //Finds the min and max values 
					depth_mat.setTo(4095, depth_mat > 4095); //Clamps the depth data from 0->4 (4095 mm) meters

					depth_mat.convertTo(depth_normalized, CV_8U, 255.0 / 4095); //Normalizes from 0-255					
					cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_HOT); //Applies the colour map

					// Display the heatmap
					cv::imshow("Depth Heatmap", depth_colormap);

					if (!(show_ir || show_clip_area_andkeypoints||show_pose))
					{
						char c = cv::waitKey(1);	//grabs key press, if q we close
						if (c == 'q')
						{
							break;

						}

					}


				}

				//****************Testing findkeypointsworldframe***************
				//ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irLeftFrame.get_data());
				////ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irRightFrame.get_data());
				//last_time = std::chrono::steady_clock::now();
				//auto detection = ir_segmenter->findKeypointsWorldFrame(std::move(ir_ptr_left), std::move(ir_ptr_right));
				//curr_time = std::chrono::steady_clock::now();
				//auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time).count();
				//std::cout << "Segmentation dt:" << elapsed_ms << std::endl;
				//
				//for (const auto& coord : detection->imCoords) {
				//	// draw the point on the image (circle with radius 3, red color)
				//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
				//}
				//
				//cv::imshow("left ir", ir_mat_left);
				//char c = cv::waitKey(1);	//grabs key press, if q we close
				//if (c == 'q')
				//{
				//	break;
				//
				//}

			}
		}
		realsense_camera.stop();	
		
	}

	return 0;
}




//*********************************Rough Commented Code*********************************
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