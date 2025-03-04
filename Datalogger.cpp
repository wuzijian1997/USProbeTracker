#include "Datalogger.h"

Datalogger::Datalogger(std::string root_path, std::string participant_directory,
	Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset, Eigen::MatrixXd force_compensation_mat,
	std::string& left_camera_intrinsics, std::string& right_camera_intrinsics,std::string& R_string,std::string& T_string,std::string& E_string,std::string& F_string,
	double& depth_scale,int depth_width,int depth_height, double depth_fps, int us_width,int us_height,double us_fps,
	std::shared_ptr<IRSegmentation> ir_segmentation_object)
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
	std::string compensation_mat_string = EigenMatrixToString(force_compensation_mat);

	//Writes these strings to the csv initially
	_csv_file << "Force Zeroing XYZ:\n";
	_csv_file << zeroing_offset_string;

	_csv_file << "Force Calibration Matrix (final column bias):\n";
	_csv_file << calib_mat_string;

	_csv_file << "Force Compensation Matrix:\n";
	_csv_file << compensation_mat_string;

	_csv_file << "Left Camera Intrinsics (fx-fy-cx-cy-k1-k2-p1-p2-k3):\n";
	_csv_file << left_camera_intrinsics + "\n";

	_csv_file << "Right Camera Intrinsics (fx-fy-cx-cy-k1-k2-p1-p2-k3):\n";
	_csv_file << right_camera_intrinsics + "\n";

	_csv_file << "Stereo R:\n";
	_csv_file << R_string + "\n";

	_csv_file << "Stereo T:\n";
	_csv_file << T_string + "\n";

	_csv_file << "Stereo E:\n";
	_csv_file << E_string + "\n";

	_csv_file << "Stereo F:\n";
	_csv_file << F_string + "\n";

	_csv_file << "RealSense Depth Scale (converts depth pixels to meters):\n";
	_csv_file << std::to_string(depth_scale)+"\n";
	_csv_file << "\n";

	//Filename for the depth and ultrasound frames
	_depth_file = _data_path + "/depthvideo_"+ datetime_str+".avi";
	_irLeft_file = _data_path + "/irLeftvideo_" + datetime_str + ".avi";
	_irRight_file = _data_path + "/irRightvideo_" + datetime_str + ".avi";
	_rgb_file = _data_path + "/rgbvideo_" + datetime_str + ".avi";

	_us_file = _data_path + "/usvideo_" + datetime_str+".avi";

	//Inits the OpenCV Video Writers
	cv::Size frameSize = cv::Size(depth_width, depth_height);

	//Depth video writer
	_depth_videowriter.open(_depth_file, cv::CAP_FFMPEG,
		cv::VideoWriter::fourcc('F', 'F', 'V', '1'), depth_fps,
		frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_DEPTH, CV_16UC1,
		cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, false });

	/*_depth_videowriter.open(_depth_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), depth_fps,
		frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, true });*/

	//ir Left video writer
	_irLeft_videowriter.open(_irLeft_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), depth_fps,
		frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_DEPTH, CV_8UC1,
		cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, false });

	//ir Right video writer
	_irRight_videowriter.open(_irRight_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), depth_fps,
		frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_DEPTH, CV_8UC1,
		cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, false });

	//rgb video writer
	_bgr_videowriter.open(_rgb_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), depth_fps,
		frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, true });

	cv::Size us_frameSize = cv::Size(us_width, us_height);
	std::cout << "US Frame Size: " << us_frameSize << std::endl;
	//US video writer
	_us_videowriter.open(_us_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), us_fps,
		us_frameSize,
		{ cv::VideoWriterProperties::VIDEOWRITER_PROP_IS_COLOR, true });

	//Sets the left ROI in case we do cropping before storing the realsense frames
	_leftROI = cv::Rect(CROP_X, CROP_Y, CROP_WIDTH, CROP_HEIGHT);
	ir_segmentation_object->transformROIToRight(_leftROI, _rightROI);


	//***********Opens ffmpef applications as a subprocess************
	//FFMPEG input: RAW 16-bit grayscale images
	//FFMPEG ouput: 16-bit FFV1 video

	//!!!!!!!!!!Inits the depth ffmpeg pipe!!!!!!!!!!!!!!!!
	//initFfmpegipe(_depth_pipeout, _depth_file,"gray16le"); //gray16le format in ffmpeg
	//Inits the US ffmpeg pipe 
	//initFfmpegipe(_us_pipeout, _us_file,"bgr24"); //bgr24 format in ffmpeg


	//Opens the Zstandard compressed file for writting
	//if (fopen_s(&_depth_zstd_file, _depth_file.c_str(), "wb") != 0) {
	//	throw std::runtime_error("Failed to open compressed file.");
	//}

	//_depth_zstd_ctx = ZSTD_createCCtx();
	//if (!_depth_zstd_ctx) {
	//	throw std::runtime_error("Failed to create Zstandard context.");
	//}

	//// Allocate compression buffer
	//_depth_compressed_buffer.resize(ZSTD_compressBound(65536)); // Adjust size as needed

}


void Datalogger::initFfmpegipe(FILE*& pipeout,std::string& file_name,std::string px_fmt) {
	std::cout << "Filename: " << file_name << std::endl;
	std::string ffmpeg_cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo "
		"-pixel_format gray16le -video_size " + std::to_string(REALSENSE_WIDTH) + "x" + std::to_string(REALSENSE_HEIGHT) +
		" -r " + std::to_string(REALSENSE_FPS) + " -i pipe: -vcodec ffv1 -level 3 -coder 2 -slices 4 -pix_fmt "+ px_fmt +" "+ file_name; //" + _depth_file;

	pipeout = _popen(ffmpeg_cmd.c_str(), "wb");
	if (!pipeout)
	{
		throw std::runtime_error("Failed to open FFMPEG subprocess.");
	}

}

Datalogger::~Datalogger()
{
	_csv_file.close();
	_depth_videowriter.release();
	_irLeft_videowriter.release();
	_irRight_videowriter.release();
	_bgr_videowriter.release();
	_us_videowriter.release();

	//fflush(_depth_pipeout);
	//_pclose(_depth_pipeout);

	//fflush(_us_pipeout);
	//_pclose(_us_pipeout);

	// Close Zstandard file
	//if (_depth_zstd_file) {
	//	fclose(_depth_zstd_file);
	//}

	//// Free Zstandard context
	//if (_depth_zstd_ctx) {
	//	ZSTD_freeCCtx(_depth_zstd_ctx);
	//}

}

void Datalogger::close()
{
	_csv_file.close();
	_depth_videowriter.release();
	_irLeft_videowriter.release();
	_irRight_videowriter.release();
	_bgr_videowriter.release();
	_us_videowriter.release();

	//// Close Zstandard file
	//if (_depth_zstd_file) {
	//	fclose(_depth_zstd_file);
	//}

	//// Free Zstandard context
	//if (_depth_zstd_ctx) {
	//	ZSTD_freeCCtx(_depth_zstd_ctx);
	//}
}



//Writes data row to .csv
void Datalogger::writeCSVRow(float& run_seconds, int& depth_frame_num, int& us_frame_num,
	Eigen::Matrix4d& cam_T_us, std::string& raw_force_string,
	std::string& force_string_xyz, std::string& temp_imu_string,std::string& landmark_name)
{
	std::string row_to_write;

	//calculates the system time
	std::string system_time = getDatetimeWithMilliseconds();

	//converts cam_T_us to be a string
	std::string cam_T_us_string;
	if ((cam_T_us.array() == -1).all())
	{
		//All values in the pose array are -1, indicating to write NaN's
		cam_T_us_string ="NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN";

	}
	else {
		//We received a pose
		cam_T_us_string = std::to_string(cam_T_us(0, 3)) + "," + std::to_string(cam_T_us(1, 3)) + "," + std::to_string(cam_T_us(2, 3)) + "," +
			std::to_string(cam_T_us(0, 0)) + "," + std::to_string(cam_T_us(0, 1)) + "," + std::to_string(cam_T_us(0, 2)) + "," +
			std::to_string(cam_T_us(1, 0)) + "," + std::to_string(cam_T_us(1, 1)) + "," + std::to_string(cam_T_us(1, 2)) + "," +
			std::to_string(cam_T_us(2, 0)) + "," + std::to_string(cam_T_us(2, 1)) + "," + std::to_string(cam_T_us(2, 2));
	}

	row_to_write = system_time + "," + std::to_string(run_seconds) + "," + std::to_string(depth_frame_num) + "," + std::to_string(us_frame_num) + "," + landmark_name + ","
		" ," + cam_T_us_string + "," + " ," + raw_force_string + "," + " ," + force_string_xyz + "," + " ," + temp_imu_string + "\n";

	//writes the row
	_csv_file << row_to_write;

}



void Datalogger::writeDepthFrame(cv::Mat& frame) {

	if (frame.empty()) return;
	//Applies cropping before storing if we are doing this
	if (IS_ROI_CROP)
	{
		frame = cropMat(frame, _leftROI);
	}

	_depth_videowriter.write(frame);
	
	//if (frame.type() != CV_16UC1) {
	//	throw std::invalid_argument("Depth frames must be of type CV_16UC1.");
	//}
	//if (!_depth_pipeout) {
	//	throw std::runtime_error("FFMPEG subprocess for depth is not open.");
	//}

	//// Write raw frame to FFmpeg
	//fwrite(frame.data, 1, REALSENSE_WIDTH * REALSENSE_HEIGHT * 2, _depth_pipeout);
	//fflush(_depth_pipeout);
	
}

void Datalogger::writeIRFrames(cv::Mat& irLeft, cv::Mat& irRight) {
	if (irLeft.empty() || irRight.empty()) return;
	if (IS_ROI_CROP)
	{
		irLeft = cropMat(irLeft, _leftROI);
		irRight = cropMat(irRight, _rightROI);
	}
	_irLeft_videowriter.write(irLeft);
	_irRight_videowriter.write(irRight);

}

void Datalogger::writeColourFrame(cv::Mat& colourFrame) {
	if (colourFrame.empty()) return;
	if (IS_ROI_CROP)
	{
		colourFrame = cropMat(colourFrame, _leftROI);
	}
	_bgr_videowriter.write(colourFrame);

}



void Datalogger::writeUSFrame(const cv::Mat& us_frame) {
	if (us_frame.empty())
	{
		std::cout << "!!!!!!!Ultrasound Frame Empty!!!!!!" << std::endl;
		return;
	}
	

	if (us_frame.type() == CV_8UC4) {
		cv::cvtColor(us_frame, _us_converted_frame, cv::COLOR_BGRA2BGR); // Remove Alpha channel
	}
	else {
		_us_converted_frame = us_frame;
	}

	_us_videowriter.write(_us_converted_frame);

	//if (frame.type() != CV_8UC3) {
	//	throw std::invalid_argument("US frames must be of type CV_8UC3.");
	//}
	//if (!_us_pipeout) {
	//	throw std::runtime_error("FFMPEG subprocess for us is not open.");
	//}

	//// Write raw frame to FFmpeg
	//fwrite(frame.data, 1, REALSENSE_WIDTH * REALSENSE_HEIGHT * 2, _us_pipeout);
	//fflush(_us_pipeout);

}

//void Datalogger::writeDepthFrame(const cv::Mat& frame) {
//	if (frame.type() != CV_16UC1) {
//		throw std::invalid_argument("Depth frames must be of type CV_16UC1.");
//	}
//	if (!_depth_pipeout) {
//		throw std::runtime_error("FFMPEG subprocess for depth is not open.");
//	}
//
//	// Write raw frame to FFmpeg
//	fwrite(frame.data, 1, REALSENSE_WIDTH * REALSENSE_HEIGHT * 2, _depth_pipeout);
//	//fflush(_depth_pipeout); // Ensure data is flushed to FFmpeg
//
//	// Read FFmpeg output
//	char buffer[65536]; // Buffer for FFmpeg output
//	size_t bytesRead = fread(buffer, 1, sizeof(buffer), _depth_pipein);
//	if (bytesRead > 0) {
//		std::cout << "bytesRead: " << bytesRead << std::endl;
//		// Compress the data with Zstandard
//		size_t compressedSize = ZSTD_compressCCtx(
//			_depth_zstd_ctx, _depth_compressed_buffer.data(), _depth_compressed_buffer.size(),
//			buffer, bytesRead, 1); // Compression level 1
//
//		if (ZSTD_isError(compressedSize)) {
//			throw std::runtime_error("Zstandard compression failed.");
//		}
//
//		// Write compressed data to the Zstandard file
//		fwrite(_depth_compressed_buffer.data(), 1, compressedSize, _depth_zstd_file);
//		//fflush(_depth_zstd_file); // Ensure data is written to disk
//	}
//	else {
//		std::cerr << "No Data Read from FFmpeg!" << std::endl;
//	}
//}