#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "Utils.h"
#include "loggers.hpp"
#include <catch2/catch_all.hpp>

TEST_CASE("LoggerTest")
{
    setupMultiSinkSpdlog("MainLog");
    setupDataSinkSpdlog("DataLog");
    std::shared_ptr<spdlog::logger> logger_ = spdlog::get("DataLog");
    logger_ = spdlog::get("DataLog");

    logger_->info("This is a test log message");

    std::array<double, 3> folForce = {0.1, 0.2, 0.3};
    std::array<double, 3> expForce = {0.1, 0.2, 0.3};
    std::array<double, 16> expPose = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    logger_->info("\"FollowerForce\": {}, \"ExpectForce\": {}, \"ExpectPose\": {}, \"zero\": {}",
                  folForce,
                  expForce,
                  expPose,
                  0);
}

TEST_CASE("SensorReadingTest")
{
    std::string line_force = "3964, 1432, 1946, 2119, 1620, 1654, 2178, 4050, 0, 0, 1552, 1972";
    std::string line_imu = "0, 29, 65483, 65531, 460, 82, 57589";

    Eigen::Vector3d force_zeroing_offset{};
    Eigen::MatrixXd force_calibration_mat = readCSVToEigenMatrix("../Resources/calmat.csv", 3, 13);
    Eigen::MatrixXd force_compensation_mat = readCSVToEigenMatrix("Resources/compensationmat_1.csv",
                                                                  3,
                                                                  3);

    std::cout << "Size of force_calibration_mat: " << force_calibration_mat.size() << std::endl;
    std::cout << force_calibration_mat << std::endl;

    std::cout << "Size of force_compensation_mat: " << force_compensation_mat.size() << std::endl;
    std::cout << force_compensation_mat << std::endl;

    // Read in force calibration matrix
    auto force_string_xyz = calculateForceVals(line_force,
                                               force_calibration_mat,
                                               force_zeroing_offset,
                                               force_compensation_mat);

    std::cout << "force_string_xyz" << std::endl;
    std::cout << force_string_xyz << std::endl;
}

TEST_CASE("reading parsing")
{
    const std::string line_force = "3964,1432,1946,2119,1620,1654,2178,4050,0,0,1552,1972";
    const std::string line_imu = "0,29,65483,65531,460,82,57589";
    const std::string line = line_force + "," + line_imu;
    std::cout << "Raw Readings: " << line << std::endl;

    auto [f_str, imu_str] = splitForceImuReadings(line);
    std::cout << "Force String: " << f_str << std::endl;
    std::cout << "IMU String: " << imu_str << std::endl;
}