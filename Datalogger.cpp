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
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
	auto datetime_str = oss.str();

	_csv_filename = _data_path + "/scan_" + datetime_str+ ".csv";

	//Creates the .csv file
	_csv_file.open(_csv_filename);

	//************Writes the .csv header*************
	_csv_file << SCANNING_CSV_HEADER;

	//************Writes initial values**************
	//gets the zeroing vector as a string
	std::string zeroing_offset_string = eigenForceToStringForce(zeroing_offset);
	zeroing_offset_string = zeroing_offset_string + '\n';

	//get the calibration matrix as a string
	std::string calib_mat_string = EigenMatrixToString(calib_mat);

	//Writes these strings to the csv
	



	

}

Datalogger::~Datalogger()
{
	_csv_file.close();

}