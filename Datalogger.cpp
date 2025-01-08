#include "Datalogger.h"

Datalogger::Datalogger(std::string root_path, std::string participant_directory,
	Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset)
{
	//************Handle Root Path***********

	//Check if root_path is NaN, if it is we resort to default "data" path
	if (root_path.find("NaN") != std::string::npos)
	{
		root_path = "data"; //Set to default root (relative directory)
	}
	//Check if root_path exists, if not create
	if (!fs::is_directory(root_path))
	{
		fs::create_directory(root_path);
	}

	//if participant_directory is NaN we create PXX where XX is the new participant number
	if (participant_directory.find("NaN") != std::string::npos)
	{
		int participant_number = 0;
		_data_path = root_path + "/P" + std::to_string(participant_number);
		while (fs::is_directory(_data_path))
		{
			participant_number++;
			_data_path = root_path + "/P" + std::to_string(participant_number);
		}


		
	}
	else
	{
		_data_path = root_path + "/" + participant_directory;
	}

	//Creates the datapath
	if (!fs::is_directory(_data_path))
	{
		fs::create_directory(_data_path);
	}

	//Gets the current datetime to name the file
	auto t = std::time(nullptr);
	std::tm bt;
	localtime_s(&bt, &t);
	std::ostringstream oss;
	oss << std::put_time(&bt, "%d-%m-%Y_%H-%M-%S");
	auto datetime_str = oss.str();

	_csv_filename = _data_path + "/scandata_" + datetime_str+ ".csv";

	//Creates the .csv file
	_csv_file.open(_csv_filename);

	//************Writes the .csv header*************
	_csv_file << SCANNING_CSV_HEADER;

	//************Writes initial values to .csv**************
	//gets the zeroing vector as a string
	std::string zeroing_offset_string = eigenForceToStringForce(zeroing_offset);
	zeroing_offset_string = zeroing_offset_string + '\n';

	//get the calibration matrix as a string separated, each row of .csv is a row of the the matrix
	std::string calib_mat_string = EigenMatrixToString(calib_mat);

	//Writes these strings to the csv
	_csv_file << "Force Zeroing XYZ:\n";
	_csv_file << zeroing_offset_string;
	_csv_file << "\n";
	_csv_file << "Force Calibration Matrix (final column bias):\n";
	_csv_file << calib_mat_string;
	_csv_file << "\n";

	//Filename for the depth and ultrasound frames
	_depth_file = _data_path + "/depthvideo_"+ datetime_str+".mp4";
	_us_file = _data_path + "/usvideo_" + datetime_str+".mp4";

	//Opens ffmpef applications as a subprocess
	//FFMPEG input: RAW 16-bit grayscale images
	//FFMPEG ouput: 16-bit H.265 video

	std::string depth_ffmpeg_cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo "
		"-pixel_format gray16le -video_size " + std::to_string(REALSENSE_WIDTH) + "x" + std::to_string(REALSENSE_HEIGHT) +
		" -r " + std::to_string(REALSENSE_FPS) + " -i pipe: -vcodec ffv1 -pix_fmt gray16le " + _depth_file;
	std::cout << "Wrote Command" << std::endl;
	_depth_pipeout = _popen(depth_ffmpeg_cmd.c_str(), "wb");
	if (!_depth_pipeout)
	{
		throw std::runtime_error("Failed to open FFMPEG subprocess for depth frames.");
	}

}

Datalogger::~Datalogger()
{
	_csv_file.close();
	fflush(_depth_pipeout);
	_pclose(_depth_pipeout);

}

void Datalogger::close()
{
	_csv_file.close();
	fflush(_depth_pipeout);
	_pclose(_depth_pipeout);
}



//Writes data row to .csv
void Datalogger::writeCSVRow(double& run_seconds, int& depth_frame_num, int& us_frame_num,
	Eigen::Matrix4d& cam_T_us, std::string& raw_force_string,
	std::string& force_string_xyz, std::string& temp_imu_string)
{
	std::string row_to_write;

	//calculates the system time
	std::string system_time = getDatetimeWithMilliseconds();

	//converts cam_T_us to be a string
	std::string cam_T_us_string = std::to_string(cam_T_us(0, 3)) + "," + std::to_string(cam_T_us(1, 3)) + "," + std::to_string(cam_T_us(2, 3)) + "," +
		std::to_string(cam_T_us(0, 0)) + "," + std::to_string(cam_T_us(0, 1)) + "," + std::to_string(cam_T_us(0, 2)) + "," +
		std::to_string(cam_T_us(1, 0)) + "," + std::to_string(cam_T_us(1, 1)) + "," + std::to_string(cam_T_us(1, 2)) + "," +
		std::to_string(cam_T_us(2, 0)) + "," + std::to_string(cam_T_us(2, 1)) + "," + std::to_string(cam_T_us(2, 2));

	row_to_write = system_time + "," + std::to_string(run_seconds) + "," + std::to_string(depth_frame_num) + "," + std::to_string(us_frame_num) + "," +
		" ," + cam_T_us_string + "," + " ," + raw_force_string + "," + " ," + force_string_xyz + "," + " ," + temp_imu_string + "\n";

	//writes the row
	_csv_file << row_to_write;

}

void Datalogger::writeDepthFrame(const cv::Mat& frame)
{
	if (frame.type() != CV_16UC1) {
		throw std::invalid_argument("Depth frames must be of type CV_16UC1.");
	}
	if (!_depth_pipeout) {
		throw std::runtime_error("FFMPEG subprocess for depth is not open.");
	}
	fwrite(frame.data, 1, REALSENSE_WIDTH * REALSENSE_HEIGHT * 2, _depth_pipeout);
	//std::cout << "Wrote Frame" << std::endl;
}