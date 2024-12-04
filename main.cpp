//Depth is aligned to left infrared frames

//***************Class Includes*************

#include "SetupAndSegment.h"


//*****************Init Vars****************
rs2::frameset frameset, aligned_framset; //Holds RealSense Frame Data
rs2::frame ir_frame_left, ir_frame_right;

cv::Mat ir_mat_left, ir_mat_right; //OpenCV Matrices of right/left



int main()
{
	//Inits the setup/segmentation object
	auto realSenseObj=std::make_shared<SetupAndSegment>(REALSENSE_WIDTH,REALSENSE_HEIGHT,SetupAndSegment::LogLevel::VeryVerbose);
	bool GPU_check = realSenseObj->GPUSetup();


	if (GPU_check)
	{
		//Config RealSense
		bool RealSense_check=realSenseObj->RealSenseSetup();
		if (RealSense_check)
		{
			//Set up tracking parameters
			realSenseObj->setDetectionMode(SetupAndSegment::DetectionMode::Contour); //Sets the segmentation method
			realSenseObj->setCameraBoundaries(0,REALSENSE_HEIGHT-1,0,REALSENSE_WIDTH-1,NEAR_CLIP,FAR_CLIP); //Sets the camera boundaries
			//Execution Loop
			while (1)
			{
				//************Grabbing RealSense Frames***************
				frameset = realSenseObj->_realSense_pipeline.wait_for_frames();
				aligned_framset = realSenseObj->_align_to_left_ir->process(frameset);
				//Get left and right infrared frames, depth info is aligned to left frame
				ir_frame_left = aligned_framset.get_infrared_frame(1);
				//ir_frame_right = aligned_framset.get_infrared_frame(2);

				//Converts Right Infrared data to vector representation
				auto ir_data = reinterpret_cast<const uint16_t*>(ir_frame_left.get_data());
				std::vector<uint16_t> ir_vector(ir_data, ir_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto ir_ptr = std::make_unique<std::vector<uint16_t>>(std::move(ir_vector));

				//Gets the depth frame
				rs2::depth_frame depth_frame = aligned_framset.get_depth_frame();

				//Converts depth frame to vector representation
				auto depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
				std::vector<uint16_t> depth_vector(depth_data, depth_data + (REALSENSE_HEIGHT * REALSENSE_WIDTH));
				auto depth_ptr = std::make_unique<std::vector<uint16_t>>(std::move(depth_vector));

				//Converts IR to OpenCV Mat:
				ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_left.get_data());
				//ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)ir_frame_right.get_data());

				//Finding Keypoints

				auto detection = realSenseObj->findKeypointsWorldFrame(std::move(ir_ptr), std::move(depth_ptr));


				//std::cout << "Success" << std::endl;
				for (const auto& coord : detection->imCoords) {
					// draw the point on the image (circle with radius 3, red color)
					cv::circle(ir_mat_left, cv::Point(coord[0], coord[1]), 3, cv::Scalar(0, 0, 255), -1);
				}

				cv::imshow("keypoints", ir_mat_left);
				char c = cv::waitKey(1);	//grabs key press, if q we close
				if (c == 'q')
				{
					break;

				}


				//****************Segmenting Markers******************
				//std::vector<cv::KeyPoint> keypoints;
				//bool success = realSenseObj.contourDetect(ir_mat_right, keypoints);
				//if (success)
				//{
				//	//Draw Keypoints
				//	cv::Mat outputImage;
				//	cv::drawKeypoints(ir_mat_right,keypoints,outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				//	cv::imshow("Keypoints", outputImage);
				//	char c = cv::waitKey(1);	//Grabs Key Press, if q we close
				//	if (c == 'q')
				//		break;

				//}

				
				
			}

		}

	}

}