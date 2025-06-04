cmake_minimum_required(VERSION 3.16)
project(USProbeTracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vcpkg integration
if (UNIX AND NOT APPLE)
    set(USER_HOME_DIR $ENV{HOME})
    include(${USER_HOME_DIR}/.vcpkg-clion/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif ()
if (WIN32)
    set(USER_HOME_DIR $ENV{USERPROFILE})
    include(${USER_HOME_DIR}/.vcpkg-clion/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif ()
message(STATUS "USER_HOME_DIR: ${USER_HOME_DIR}")

# Find dependencies
find_package(Eigen3 3.4 REQUIRED)
find_package(Catch2 3.8 REQUIRED)
find_package(realsense2 2.54 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(zstd REQUIRED)

set(SOURCES
        Datalogger.cpp
        IRSegmentation.cpp
        main.cpp
        pch.cpp
        PoseTracker.cpp
        RealSense.cpp
        ShellSensorReader.cpp
        USVideoStreaming.cpp
        Utils.cpp
)

set(HEADERS
        atomicops.h
        Datalogger.h
        framework.h
        IRSegmentation.h
        log.h
        pch.h
        PoseTracker.h
        readerwriterqueue.h
        RealSense.h
        ShellSensorReader.h
        USVideoStreaming.h
        Utils.h
)

# Add your source files here
add_executable(USProbeTracker
        ${SOURCES} ${HEADERS}
)

target_include_directories(USProbeTracker PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${realsense2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${ZSTD_INCLUDE_DIRS}
)

target_link_libraries(USProbeTracker
        Eigen3::Eigen
        realsense2::realsense2
        ${OpenCV_LIBS}
        zstd::libzstd
)

add_subdirectory(StereoCalibration)