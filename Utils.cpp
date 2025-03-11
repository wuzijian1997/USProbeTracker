//Helper method to convert raw force sensor readings to actual forces
#include "Utils.h"


std::string calculateForceVals(std::string& raw_force_string, Eigen::MatrixXd& calib_mat, Eigen::Vector3d& zeroing_offset,Eigen::MatrixXd& force_compensation_mat)
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
		force_xyz = force_compensation_mat * force_xyz;
		//Adds the zeroing offset if present
		force_xyz = force_xyz - zeroing_offset;


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

Eigen::Vector3d XYZforcestringToForceXYZVector(const std::string& xyz_force_string)
{
	Eigen::Vector3d eigen_xyz_force_vector; //Inits the eigen force vector
	std::stringstream ss(xyz_force_string);

	std::string substr;
	int i = 0;

	while (std::getline(ss, substr, ',') && i < 3) {
		eigen_xyz_force_vector[i] = std::stod(substr);
		i++;
	}

	return eigen_xyz_force_vector;
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

std::string openCVMatToCSVString(const cv::Mat& mat) {
	std::ostringstream oss;

	for (int i = 0; i < mat.rows; ++i) {
		for (int j = 0; j < mat.cols; ++j) {
			oss << mat.at<double>(i, j);
			if (!(i == mat.rows - 1 && j == mat.cols - 1)) {
				oss << ",";  // Add comma separator
			}
		}
	}

	return oss.str();
}

//For cropping the ROI of the realsense frames (if wanted)
cv::Mat cropMat(const cv::Mat& inputMat, cv::Rect cropRegion) {
	// Perform the crop
	return inputMat(cropRegion).clone(); // Clone to create an independent Mat
}


//Iterative midpoint method for stereo triangulation
#include <opencv2/opencv.hpp>
#include <iostream>

// Iterative Midpoint Triangulation Function
void triangulatePointsIterativeLinear(
	const cv::Mat& P0, const cv::Mat& P1,
	const std::vector<cv::Point2f>& left_matched,
	const std::vector<cv::Point2f>& right_matched,
	cv::Mat& points4D)
{


	// Check if input vectors are valid
	if (left_matched.size() != right_matched.size() || left_matched.empty()) {
		std::cerr << "Error: Mismatched or empty input points." << std::endl;
		return;
	}

	int N = left_matched.size();  // Number of points
	points4D = cv::Mat(4, N, CV_64F);  // Output matrix (4xN)
	double EPS = 1e-10;	
	int MAX_ITERS = 10;

	for (int i = 0; i < N; i++) {
		cv::Point2d p0 = left_matched[i];
		cv::Point2d p1 = right_matched[i];
		cv::Mat x;
		double w0 = 1, w1 = 1;
		for (int j = 0; j < MAX_ITERS; j++) { // 10 iterations should be enough
			cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
			A.row(0) = (p0.x * P0.row(2) - P0.row(0)) / w0;
			A.row(1) = (p0.y * P0.row(2) - P0.row(1)) / w0;
			A.row(2) = (p1.x * P1.row(2) - P1.row(0)) / w1;
			A.row(3) = (p1.y * P1.row(2) - P1.row(1)) / w1;
			cv::SVD::solveZ(A, x);

			// Compute new weight estimates
			double new_w0 = cv::Mat(P0.row(2) * x).at<double>(0);
			double new_w1 = cv::Mat(P1.row(2) * x).at<double>(0);

			if (std::abs(w0 - new_w0) <= EPS && std::abs(w1 - new_w1) <= EPS)
			{
				break;
			}

			w0 = new_w0;
			w1 = new_w1;
		}
		
		

		// Convert to homogeneous coordinates
		points4D.at<double>(0, i) = x.at<double>(0)/ x.at<double>(3);
		points4D.at<double>(1, i) = x.at<double>(1)/ x.at<double>(3);
		points4D.at<double>(2, i) = x.at<double>(2)/ x.at<double>(3);
		points4D.at<double>(3, i) = 1;  // Homogeneous coordinate
		
	}
	
}

void triangulatePointsIterativeEigen(
	const cv::Mat& P0, const cv::Mat& P1,
	const std::vector<cv::Point2f>& left_matched,
	const std::vector<cv::Point2f>& right_matched,
	cv::Mat& points4D)
{


	// Check if input vectors are valid
	if (left_matched.size() != right_matched.size() || left_matched.empty()) {
		std::cerr << "Error: Mismatched or empty input points." << std::endl;
		return;
	}

	int N = left_matched.size();  // Number of points
	points4D = cv::Mat(4, N, CV_64F);  // Output matrix (4xN)
	double EPS = 1e-8;
	int MAX_ITERS = 20;

	for (int i = 0; i < N; i++) {
		cv::Point2d p0 = left_matched[i];
		cv::Point2d p1 = right_matched[i];
		cv::Mat x;
		double w0 = 1, w1 = 1;
		cv::Mat eigen_vectors;
		std::vector<double> eigen_values;
		for (int i = 0; i < MAX_ITERS; i++) { // 10 iterations should be enough
			cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
			A.row(0) = (p0.x * P0.row(2) - P0.row(0)) / w0;
			A.row(1) = (p0.y * P0.row(2) - P0.row(1)) / w0;
			A.row(2) = (p1.x * P1.row(2) - P1.row(0)) / w1;
			A.row(3) = (p1.y * P1.row(2) - P1.row(1)) / w1;
			cv::eigen(A.t() * A, eigen_values, eigen_vectors);
			x = eigen_vectors.row(eigen_vectors.rows - 1).t();

			double new_w0 = cv::Mat(P0.row(2) * x).at<double>(0);
			double new_w1 = cv::Mat(P1.row(2) * x).at<double>(0);

			if (std::abs(w0 - new_w0) <= EPS && std::abs(w1 - new_w1) <= EPS)
			{
				break;
			}

			w0 = new_w0;
			w1 = new_w1;
		}


		// Convert to homogeneous coordinates
		points4D.at<double>(0, i) = x.at<double>(0) / x.at<double>(3);
		points4D.at<double>(1, i) = x.at<double>(1) / x.at<double>(3);
		points4D.at<double>(2, i) = x.at<double>(2) / x.at<double>(3);
		points4D.at<double>(3, i) = 1;  // Homogeneous coordinate
	}
}

