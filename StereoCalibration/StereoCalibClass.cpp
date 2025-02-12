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
    bool found_left = cv::findChessboardCorners(left_image, board_size, corners_left);
    bool found_right = cv::findChessboardCorners(right_image, board_size, corners_right);
    if (found_left && found_right)
    {
        //Subpix the corners
        cv::cornerSubPix(left_image, corners_left, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 500, 0.001));
        
        cv::cornerSubPix(right_image, corners_right, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 500, 0.001));


        //Push the corresponding corner points to the image points objects
        _left_image_points.push_back(corners_left);
        _right_image_points.push_back(corners_right);


        //Draws corners onto images
        cv::Mat left_new = left_image.clone();
        cv::Mat right_new = right_image.clone();
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