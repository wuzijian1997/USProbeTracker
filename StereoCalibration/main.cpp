#include "RealSense.h"
#include "StereoCalibClass.h"

std::string checkerboard_rootpath = "C:\\Users\\alexa\\OneDrive\\Documents\\UBC_Thesis\\Code\\ObstetricUS_Project\\StereoCalibration\\CalibFiles\\CheckerboardImages";
std::string calib_filepath="C:\\Users\\alexa\\OneDrive\\Documents\\UBC_Thesis\\Code\\ObstetricUS_Project\\StereoCalibration\\CalibFiles\\Calib\\calibration_params_2.yaml";

int realsense_timeout = 35; //Realsense Frame Grabber Returns False if waiting more than 35 ms

int main()
{
	//************Initialize Stereo Calibration Object*****************
	StereoCalibClass stereoCalibrator(10, 7, 0.025f,checkerboard_rootpath);


	//****************Init the RealSense Object************************
	//Inits realsense camera object
	RealSense realsense_camera(realsense_timeout); //timeout is the amount of time we wait for realsense thread to update the data

	//Configures the camera
	//These constants are in RealSense.h
	bool realsense_check = realsense_camera.RealSenseInit(REALSENSE_WIDTH,
		REALSENSE_HEIGHT, REALSENSE_FPS, ENABLE_REALSENSE_LASER, REALSENSE_LASER_POWER,
		REALSENSE_GAIN, TEMPORAL_DEPTH_FILTER_ALPHA, TEMPORAL_DEPTH_FILTER_DELTA, REALSENSE_AUTOEXPOSURE_ENABLE, REALSENSE_EXPOSURE_LEVEL);


	//OpenCV Matrices of right/left IR
	cv::Mat ir_mat_left, ir_mat_right;

	//Boolean for if realsense data is returned
	bool is_realsense_data_returned = false;
	//object to hold realsense data
	RealSense::RealSenseData realsense_data;

	if (realsense_check)
	{

		//Start the realsense thread
		realsense_camera.start();
		std::cout << "Press 'Enter' to capture frames, 'c' to calibrate, 'q' to quit.\n";

		while (true)
		{
			is_realsense_data_returned = realsense_camera.getRealSenseData(realsense_data);
			if (is_realsense_data_returned) //Enters if depth/ir frames arrive
			{

				//***********************RealSense Data Conversions******************
				ir_mat_left = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irLeftFrame.get_data());
				ir_mat_right = cv::Mat(cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT), CV_8UC1, (void*)realsense_data.irRightFrame.get_data());			

				//**********************Displaying Frames****************************
				cv::imshow("IR Left", ir_mat_left);
				cv::imshow("IR Right", ir_mat_right);
				char key = cv::waitKey(1);	//grabs key press
				if (key == '\r' || key == '\n') //Enter key is pressed
				{
					stereoCalibrator.addCalibrationImages(ir_mat_left, ir_mat_right);
					std::cout << "Captured Stereo Image" << std::endl;

				}
				else if (key == 'c') //start calibration
				{
					std::cout << "Starting Calibration..." << std::endl;
					if (stereoCalibrator.calibrate())
					{
						stereoCalibrator.saveCalibration(calib_filepath);
						std::cout << "Stereo Calibration Complete and Saved" << std::endl;
					}
					else
					{
						std::cout << "Calibration Failed" << std::endl;
					}
					
				}

				if (key == 'q')
				{
					std::cout << "Exiting..." << std::endl;
					break;

				}

			}

		}
		realsense_camera.stop();

	}



	return 0;
}