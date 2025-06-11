#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include <catch2/catch_all.hpp>
#include "loggers.hpp"

TEST_CASE("LoggerTest")
{
    setupMultiSinkSpdlog("MainLog");
    setupDataSinkSpdlog("DataLog");
    std::shared_ptr<spdlog::logger> logger_ = spdlog::get("DataLog");
    logger_ = spdlog::get("DataLog");

    logger_->info("This is a test log message");

    std::array<double, 3> folForce = {0.1, 0.2, 0.3};
    std::array<double, 3> expForce = {0.1, 0.2, 0.3};
    std::array<double, 16> expPose = {1, 0, 0, 0,
                                      0, 1, 0, 0,
                                      0, 0, 1, 0,
                                      0, 0, 0, 1};

    logger_->info("\"FollowerForce\": {}, \"ExpectForce\": {}, \"ExpectPose\": {}, \"zero\": {}",
                  folForce,
                  expForce,
                  expPose,
                  0);
}