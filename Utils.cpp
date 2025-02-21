//Helper method to convert raw force sensor readings to actual forces
#include "Utils.h"


std::string calculateForceVals(std::string& raw_force_string, Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset)
{
	std::string string_force_xyz;

	if (raw_force_string.find("NaN") != std::string::npos)
	{
		string_force_xyz = "NaN,NaN,NaN";
	}
	else {

		//raw_force_string is 1x12 raw sensor string
		//raw_force_vector is 1x13 eigen vector: 1x12 raw sensor values/4096 with a 1 appended

		Eigen::VectorXd raw_force_vector = forcestringToForceVector(raw_force_string);

		//Computes the x,y,z force from initial ATI reading
		Eigen::Vector3d force_xyz = calib_mat * raw_force_vector; // .transpose(); Transpose here is wrong

		//Adds the zeroing offset if present
		force_xyz = force_xyz + zeroing_offset;


		//Converts the Eigen vector back to a string
		string_force_xyz = eigenForceToStringForce(force_xyz);
	}
	return string_force_xyz;

}

Eigen::VectorXd forcestringToForceVector(std::string& raw_force_string)
{
	Eigen::VectorXd raw_force_vector(13); //Inits the eigen force vector
	std::stringstream ss(raw_force_string);

	std::string substr;
	int i = 0;

	while (std::getline(ss, substr, ',') && i < 12) {
		raw_force_vector[i] = std::stod(substr) / 4096.0f; // Normalize value
		i++;
	}

	raw_force_vector[12] = 1.0f;
	return raw_force_vector;

}

std::string eigenForceToStringForce(Eigen::Vector3d& force_xyz)
{
	std::ostringstream oss;
	for (int i = 0; i < force_xyz.size(); ++i) {
		oss << force_xyz[i];
		if (i != force_xyz.size() - 1) { // Add a comma between elements
			oss << ",";
		}
	}
	std::string string_force = oss.str();
	return string_force;
}

Eigen::MatrixXd readCSVToEigenMatrix(const std::string& file_path, int rows, int cols)
{
	//Creates ifstream object to read csv row by row
	std::ifstream file(file_path);
	Eigen::MatrixXd matrix(rows, cols);
	std::string line;
	int row = 0;

	while (std::getline(file, line) && row < rows) {
		std::stringstream line_stream(line);
		std::string cell;
		int col = 0;

		while (std::getline(line_stream, cell, ',') && col < cols) {
			matrix(row, col) = std::stod(cell);
			++col;
		}

		++row;
	}

	file.close();
	return matrix;

}


//Converts an eigen matrix, to a string to be written to a .csv
std::string EigenMatrixToString(Eigen::MatrixXd& matrix)
{
	std::ostringstream oss;

	//Loop for all the rows
	for (int i = 0; i < matrix.rows(); ++i)

	{
		//loop for all the columns 
		for (int j = 0; j < matrix.cols(); ++j)
		{
			oss << matrix(i, j); //Index row,col of matrix
			//add a comma if it's not the last column
			if (j < matrix.cols() - 1)
			{
				oss << ",";
			}
		}
		//Add a newline after each row

		oss << "\n";
	}

	//Returns matrix as string
	return oss.str();

}

std::string getDatetimeWithMilliseconds() {
	auto now = std::chrono::system_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
	
	auto timer = std::chrono::system_clock::to_time_t(now);

	// convert to broken time
	std::tm bt;
	localtime_s(&bt, &timer);
	

	std::ostringstream oss;

	oss << std::put_time(&bt, "%d-%m-%Y_%H-%M-%S"); // HH:MM:SS
	oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

	return oss.str();
}


Eigen::Matrix3d convertCVMatToEigen_3b3(cv::Mat R_mat)
{
	Eigen::Matrix3d _R_eigen;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			_R_eigen(i, j) = R_mat.at<double>(i, j);
	return _R_eigen;
}

Eigen::Vector3d convertCVMatToEigen_1b3(cv::Mat T_mat)
{
	Eigen::Vector3d _T_eigen;
	for (int i = 0; i < 3; ++i)
		_T_eigen(i) = T_mat.at<double>(i, 0);
	return _T_eigen;

}