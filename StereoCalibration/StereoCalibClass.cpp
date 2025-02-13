#include "StereoCalibClass.h"


//Constructor to init the checkerboard parameters
StereoCalibClass::StereoCalibClass(int board_width, int board_height, float square_size,std::string chessboard_filepath) :
    _board_width(board_width), _board_height(board_height), _square_size(square_size), _chessboard_filepath(chessboard_filepath)
{
    _chessboard_num = 0; //Counter that is used to track chessboard photos used for calibration
}

StereoCalibClass::~StereoCalibClass()
{
    cv::destroyAllWindows();
}

bool StereoCalibClass::addCalibrationImages(const cv::Mat& left_image, const cv::Mat& right_image)
{
    //Checks if the images are empty
    if (left_image.empty() || right_image.empty()) {
        std::cout << "Error: Empty calibration images." << std::endl;
        return false;
    }

    //Now we search the checkerboards for corners, and add it to our image points lists
    std::vector<cv::Point2f> corners_left, corners_right;
    cv::Size board_size(_board_width, _board_height);

    //Finding the chessboard corners
    bool found_left = cv::findChessboardCorners(left_image, board_size, corners_left , cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    bool found_right = cv::findChessboardCorners(right_image, board_size, corners_right , cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (found_left && found_right)
    {
        //Subpix the corners
        cv::cornerSubPix(left_image, corners_left, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 500, 0.001));
        
        cv::cornerSubPix(right_image, corners_right, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 500, 0.001));


        //Push the corresponding corner points to the image points objects
        _left_image_points.push_back(corners_left);
        _right_image_points.push_back(corners_right);


        //Draws corners onto images
        cv::Mat left_new = left_image.clone();
        cv::Mat right_new = right_image.clone();
       
        cv::cvtColor(left_new, left_new, cv::COLOR_GRAY2BGR);
        cv::cvtColor(right_new, right_new, cv::COLOR_GRAY2BGR);

        cv::drawChessboardCorners(left_new, board_size, corners_left, found_left);
        cv::drawChessboardCorners(right_new, board_size, corners_right, found_right);

        //Saves the images
        std::string right_image_filepath = _chessboard_filepath + "\\withoutcorners\\right\\rightIR_" + std::to_string(_chessboard_num) + ".png";
        std::string left_image_filepath = _chessboard_filepath + "\\withoutcorners\\left\\leftIR_" + std::to_string(_chessboard_num) + ".png";

        std::string right_image_withcorners_filepath = _chessboard_filepath + "\\withcorners\\right\\rightIR_" + std::to_string(_chessboard_num) + ".png";
        std::string left_image_withcorners_filepath = _chessboard_filepath + "\\withcorners\\left\\leftIR_" + std::to_string(_chessboard_num) + ".png";

        cv::imwrite(right_image_filepath, right_image);
        cv::imwrite(left_image_filepath, left_image);

        cv::imwrite(right_image_withcorners_filepath, right_new);
        cv::imwrite(left_image_withcorners_filepath, left_new);

        //Shows the left/right chessboard corners
        cv::imshow("Left Checkerboard", left_new);
        cv::imshow("Right Checkerboard", right_new);
        cv::waitKey(1);


        //Prints and Increments the chessboard number
        std::cout << "Chessboard Number: " + std::to_string(_chessboard_num) << std::endl;
        _chessboard_num++;


        return true;
    }
    else
    {
        std::cout << "Didn't get chessboard images in both left/right cameras." << std::endl;
        return false;    
    }


}


void StereoCalibClass::prepareObjectPoints() {
    _object_points.clear();
    std::vector<cv::Point3f> objP;

    for (int i = 0; i < _board_height; i++) {
        for (int j = 0; j < _board_width; j++) {
            objP.emplace_back((float)j * _square_size, (float)i * _square_size, 0);
        }
    }

    for (size_t i = 0; i < _left_image_points.size(); i++) {
        _object_points.push_back(objP);
    }
}

bool StereoCalibClass::calibrate()
{
    if (_left_image_points.size() < MINIMUM_NUM_CHECKERBOARDS || _right_image_points.size() < MINIMUM_NUM_CHECKERBOARDS) {
        std::cout << "Error: Not enough valid calibration images." << std::endl;
        return false;
    }

    //Creates the corresponding point object
    prepareObjectPoints();

    //Does mono calibration for each IR camera to seed stereo calibration
    cv::calibrateCamera(_object_points, _left_image_points, cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT),
        _left_camera_mat,_left_dist,cv::noArray(),cv::noArray());

    cv::calibrateCamera(_object_points, _right_image_points, cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT),
        _right_camera_mat, _right_dist, cv::noArray(), cv::noArray());

    
    //Does the stereo calibration
    _calibration_error=cv::stereoCalibrate(_object_points, _left_image_points, _right_image_points,
        _left_camera_mat, _left_dist, _right_camera_mat, _right_dist, cv::Size(REALSENSE_WIDTH, REALSENSE_HEIGHT),
        _R, _T, _E, _F, cv::CALIB_FIX_INTRINSIC);

    std::cout << "Stereo Calibration Error: " << _calibration_error << std::endl;


    return true;

}

void StereoCalibClass::saveCalibration(const std::string& filename)
{
    cv::FileStorage fs1(filename, cv::FileStorage::WRITE);
    fs1 << "left_mat" << _left_camera_mat;
    fs1 << "left_dist" << _left_dist;
    fs1 << "right_mat" << _right_camera_mat;
    fs1 << "right_dist" << _right_dist;
    fs1 << "R" << _R;
    fs1 << "T" << _T;
    fs1 << "E" << _E;
    fs1 << "F" << _F;
    fs1 << "CalibError" << _calibration_error;
    fs1.release();
    std::cout << "Calibration Saved To: " << filename << std::endl;
}