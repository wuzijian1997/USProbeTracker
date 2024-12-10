//Depth is aligned to left infrared frames

//***************Class Includes*************

#include "SetupAndSegment.h"
#include "PoseTracker.h"
#include "USVideoStreaming.h"


//*****************Init Vars****************
rs2::frameset frameset, aligned_framset; //Holds RealSense Frame Data
rs2::frame ir_frame_left, ir_frame_right;

cv::Mat ir_mat_left, ir_mat_right; //OpenCV Matrices of right/left

cv::Mat US_frame;

bool continueUS = true;;

int main()
{
	//Inits the US Video Streamer Object and stream thread
	USVideoStreaming USStreamer(true); //Shows the US stream when true
	
	while (true)
	{
		cv::Mat usFrame = USStreamer.getFrame(); //Gets most recent frame

		if (usFrame.empty())
		{
			//do something here later
			std::cout << "Empty US Frame" << std::endl;
			continue;
		}
		continueUS =USStreamer.showFrame();

		if(!continueUS) //The US Stream Window was closed via 'Esc'
		{
			break;
		}

	}


	////Sets up the marker geometry
	//Eigen::Vector3d marker1(0.0157,0.0154,0);
	//Eigen::Vector3d marker2(0.0358,-0.023,0);
	//Eigen::Vector3d marker3(-0.0366,-0.0237,0);
	//Eigen::Vector3d marker4(-0.0247,0.0121,0);
	//auto geom= std::vector<Eigen::Vector3d>{ marker1, marker2, marker3, marker4 };


	////Inits the setup/segmentation object
	//auto realSenseObj=std::make_shared<SetupAndSegment>(REALSENSE_WIDTH,REALSENSE_HEIGHT,SetupAndSegment::LogLevel::Silent);
	//bool GPU_check = realSenseObj->GPUSetup();
	//

	//if (GPU_check)
	//{
	//	//Config RealSense
	//	bool RealSense_check=realSenseObj->RealSenseSetup();
	//	if (RealSense_check)
	//	{
	//		//Set up tracking parameters
	//		realSenseObj->setDetectionMode(SetupAndSegment::DetectionMode::Contour); //Sets the segmentation method
	//		realSenseObj->setCameraBoundaries(0,REALSENSE_HEIGHT-1,0,REALSENSE_WIDTH-1,NEAR_CLIP,FAR_CLIP); //Sets the camera boundaries
	//		
	//		double fx = realSenseObj->_realSense_intrinsics_leftIR.fx;
	//		double fy = realSenseObj->_realSense_intrinsics_leftIR.fy;
	//		double cx = realSenseObj->_realSense_intrinsics_leftIR.ppx;
	//		double cy = realSenseObj->_realSense_intrinsics_leftIR.ppy;

	//		//Sets the camera intinsics function
	//		realSenseObj->setCameraIntrinsics([&](const std::array<double, 2>& uv, std::array<double, 2>& xy) {
	//			xy[0] = (uv[0] - cx) / fx; //Normalized x-coordinate
	//			xy[1] = (uv[1] - cy) / fy; //Normalized y-coordinate

	//			});

	//		//Set up the pose calculator
	//		PoseTracker poseTracker(realSenseObj, geom,0.011);
	//		poseTracker.setJumpSettings(false,0.1,4);
	//		poseTracker.setSmoothing(0);

	//		//Init some variables for display:
	//		cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
	//			0, fy, cy,
	//			0, 0, 1);

	//		cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
	//			realSenseObj->_realSense_intrinsics_leftIR.coeffs[0], // k1
	//			realSenseObj->_realSense_intrinsics_leftIR.coeffs[1], // k2
	//			realSenseObj->_realSense_intrinsics_leftIR.coeffs[2], // p1
	//			realSenseObj->_realSense_intrinsics_leftIR.coeffs[3], // p2
	//			realSenseObj->_realSense_intrinsics_leftIR.coeffs[4]  // k3
	//			);
	//		cv::Mat rotation;
	//		cv::Mat translation;
	//		cv::Mat rvec;
	//		
	//		//Execution Loop
	//		while (1)
	//		{
	//			//************Grabbing RealSense Frames***************
	//			frameset = realSenseObj->_realSense_pipeline.wait_for_frames();
	//			aligned_framset = realSenseObj->_align_to_left_ir->process(frameset);
	//			//Get left and right infrared frames, depth info is aligned to left frame
	//			ir_frame_left = aligned_framset.get_infrared_frame(1);
	//			//ir_frame_right = aligned_framset.get_infrared_frame(2);

	//			//Converts Right Infrared data to vector representation
	//			auto ir_data = reinterpret_cast<const uint16_t*>(ir_frame_left.get_data());
	//			std::vector<uint16_t> ir_vector(ir_data, ir_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
	//			auto ir_ptr = std::make_unique<std::vector<uint16_t>>(std::move(ir_vector));

	//			//Gets the depth frame
	//			rs2::depth_frame depth_frame = aligned_framset.get_depth_frame();

	//			//Converts depth frame to vector representation
	//			auto depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	//			std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
	//			auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));

	//			//Converts IR to OpenCV Mat:
	//			ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_left.get_data());
	//			//ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_right.get_data());


	//			//Finding Pose
	//			poseTracker.update(std::move(ir_ptr), std::move(depth_ptr));
	//			//if (poseTracker.hasNewPose())
	//			//{
	//			//	Eigen::Matrix4d T = poseTracker.getPose();
	//			//	//PoseTracker::IRPose mes = poseTracker.getLastMeasurement();
	//			//	//std::cout << "Computed pose" << std::endl;
	//			//	//std::cout << T << std::endl;

	//			//	for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
	//			//	// draw the point on the image (circle with radius 3, red color)
	//			//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
	//			//	}

	//			//	cv::imshow("keypoints", ir_mat_left);
	//			//	char c = cv::waitKey(1);	//grabs key press, if q we close
	//			//	if (c == 'q')
	//			//	{
	//			//		break;

	//			//	}

	//			//}
	//			
	//			int i;
	//			for (i = 0; i < 10; i++) {
	//				if (poseTracker.hasNewPose()) break;
	//				using namespace std::chrono_literals;
	//				std::this_thread::sleep_for(10ms);
	//			}

	//			if (i == 10) {
	//				std::cout << "Failed to compute pose" << std::endl;
	//			}
	//			else {
	//				Eigen::Matrix4d T = poseTracker.getPose();
	//				//PoseTracker::IRPose mes = poseTracker.getLastMeasurement();
	//				std::cout << "Computed pose" << std::endl;
	//				//std::cout << T << std::endl;
	//				//std::cout << mes.pose.matrix() << std::endl;

	//				cv::Mat cvT(4, 4, CV_64F);
	//				for (int i = 0; i < 4; ++i) {
	//					for (int j = 0; j < 4; ++j) {
	//						cvT.at<double>(i, j) = T(i, j);
	//					}
	//				}

	//				rotation = cvT(cv::Range(0, 3), cv::Range(0, 3));
	//				translation = cvT(cv::Range(0, 3), cv::Range(3, 4));
	//				cv::Mat outputImage;
	//				cv::cvtColor(ir_mat_left, outputImage, cv::COLOR_GRAY2BGR);
	//				cv::Rodrigues(rotation, rvec);
	//				cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, translation, 0.1, 3);
	//				cv::imshow("Pose Visualization", outputImage);
	//				cv::waitKey(1);




	//				//for (const auto& coord : poseTracker.m_objectPose.imageCoords) {
	//				//		// draw the point on the image (circle with radius 3, red color)
	//				//		cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
	//				//}

	//				//cv::imshow("keypoints", ir_mat_left);
	//				//cv::waitKey(1);	//grabs key press, if q we close
	//			}

	//			//Finding Keypoints

	//			//auto detection = realSenseObj->findKeypointsWorldFrame(std::move(ir_ptr), std::move(depth_ptr));


	//			////std::cout << "Success" << std::endl;
	//			//for (const auto& coord : detection->imCoords) {
	//			//	// draw the point on the image (circle with radius 3, red color)
	//			//	cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
	//			//}

	//			//cv::imshow("keypoints", ir_mat_left);
	//			//char c = cv::waitKey(1);	//grabs key press, if q we close
	//			//if (c == 'q')
	//			//{
	//			//	break;

	//			//}


	//			//****************Segmenting Markers******************
	//			//std::vector<cv::KeyPoint> keypoints;
	//			//bool success = realSenseObj.contourDetect(ir_mat_right, keypoints);
	//			//if (success)
	//			//{
	//			//	//Draw Keypoints
	//			//	cv::Mat outputImage;
	//			//	cv::drawKeypoints(ir_mat_right,keypoints,outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//			//	cv::imshow("Keypoints", outputImage);
	//			//	char c = cv::waitKey(1);	//Grabs Key Press, if q we close
	//			//	if (c == 'q')
	//			//		break;

	//			//}

	//			
	//			
	//		}

	//	}

	//}

return 0;
}