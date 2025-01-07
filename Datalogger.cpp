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
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
	auto datetime_str = oss.str();

	_csv_filename = _data_path + "/scan_" + datetime_str+ ".csv";

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
	_csv_file << "Force Calibration Matrix (final column bias):\n";
	_csv_file << calib_mat_string;
	

}

Datalogger::~Datalogger()
{
	_csv_file.close();

}

void Datalogger::close()
{
	_csv_file.close();
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
		std::to_string(cam_T_us(0, 0)) + "," + std::to_string(cam_T_us(0, 1)) + "," + std::to_string(cam_T_us(0, 2)) +
		std::to_string(cam_T_us(1, 0)) + "," + std::to_string(cam_T_us(1, 1)) + "," + std::to_string(cam_T_us(1, 2)) +
		std::to_string(cam_T_us(2, 0)) + "," + std::to_string(cam_T_us(2, 1)) + "," + std::to_string(cam_T_us(2, 2));

	row_to_write = system_time + "," + std::to_string(run_seconds) + "," + std::to_string(depth_frame_num) + "," + std::to_string(us_frame_num) + "," +
		" ," + cam_T_us_string + "," + " ," + raw_force_string + "," + " ," + force_string_xyz + "," + " ," + temp_imu_string + "\n";

	//writes the row
	_csv_file << row_to_write;

}