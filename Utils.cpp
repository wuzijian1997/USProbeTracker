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


cv::Rect transformROIToRight(const cv::Rect& left_ROI, const cv::Mat& left_mat,
	const cv::Mat& right_mat,const cv::Mat& R, const cv::Mat& T)
{
	float assumed_depth = 1.25f; //Assume a fixed distance so we can find a point along the epipolar line

	//Corners of the ROI in the left image
	cv::Point2f topLeft(left_ROI.x, left_ROI.y);
	cv::Point2f bottomRight(left_ROI.x + left_ROI.width, left_ROI.y + left_ROI.height);

	// Transform each corner to right image
	cv::Point2f topLeftR = transformPointToRight(topLeft, assumed_depth, left_mat, right_mat, R, T);
	cv::Point2f bottomRightR = transformPointToRight(bottomRight, assumed_depth, left_mat, right_mat, R, T);

	// Create the corresponding ROI in the right image
	cv::Rect right_ROI(topLeftR.x, topLeftR.y,
		bottomRightR.x - topLeftR.x, bottomRightR.y - topLeftR.y);

	return right_ROI;

}

cv::Point2f transformPointToRight(cv::Point2f left_point, float assumed_depth,
	const cv::Mat& left_mat, const cv::Mat& right_mat,
	const cv::Mat& R, const cv::Mat& T) {
	// Extract focal lengths and principal points
	double fxL = left_mat.at<double>(0, 0);
	double fyL = left_mat.at<double>(1, 1);
	double cxL = left_mat.at<double>(0, 2);
	double cyL = left_mat.at<double>(1, 2);

	double fxR = right_mat.at<double>(0, 0);
	double fyR = right_mat.at<double>(1, 1);
	double cxR = right_mat.at<double>(0, 2);
	double cyR = right_mat.at<double>(1, 2);

	// Step 1: Convert 2D point to normalized camera coordinates in the left camera
	double X_lc = ((left_point.x - cxL) * assumed_depth)/ fxL;
	double Y_lc = (left_point.y - cyL) * assumed_depth / fyL;
	double Z_lc = assumed_depth;

	// Create 3D homogeneous point in left camera coordinates
	cv::Mat T_lc = (cv::Mat_<double>(3, 1) << X_lc, Y_lc, Z_lc);

	// Step 2: Transform the point to the right camera coordinate system
	cv::Mat T_rc = R * T_lc + T; // Apply stereo transformation

	double X_rc = T_rc.at<double>(0, 0);
	double Y_rc = T_rc.at<double>(1, 0);
	double Z_rc = T_rc.at<double>(2, 0);

	// Step 3: Project the transformed point to the right image using right camera intrinsics
	double u_right = fxR * (X_rc / Z_rc) + cxR;
	double v_right = fyR * (Y_rc / Z_rc) + cyR;

	return cv::Point2f(u_right, v_right);
}

