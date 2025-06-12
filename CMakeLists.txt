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
find_package(realsense2 2.56 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(zstd REQUIRED)
find_package(toml11 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)


set(SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(USPROBETRACKER_SOURCES
        ${SOURCE_DIR}/Datalogger.cpp
        ${SOURCE_DIR}/IRSegmentation.cpp
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/PoseTracker.cpp
        ${SOURCE_DIR}/RealSense.cpp
        ${SOURCE_DIR}/ShellSensorReader.cpp
        ${SOURCE_DIR}/USVideoStreaming.cpp
        ${SOURCE_DIR}/Utils.cpp
)

set(HEADER_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(USPROBETRACKER_HEADERS
        ${HEADER_DIR}/atomicops.h
        ${HEADER_DIR}/Datalogger.h
        ${HEADER_DIR}/framework.h
        ${HEADER_DIR}/IRSegmentation.h
        ${HEADER_DIR}/log.h
        ${HEADER_DIR}/PoseTracker.h
        ${HEADER_DIR}/readerwriterqueue.h
        ${HEADER_DIR}/RealSense.h
        ${HEADER_DIR}/ShellSensorReader.h
        ${HEADER_DIR}/USVideoStreaming.h
        ${HEADER_DIR}/Utils.h
        ${HEADER_DIR}/configs.hpp
        ${HEADER_DIR}/loggers.hpp
        ${HEADER_DIR}/AsyncVideoWriter.hpp
)

set(USPROBETRACKER_LIBS
        Eigen3::Eigen
        realsense2::realsense2
        ${OpenCV_LIBS}
        zstd::libzstd
        toml11::toml11
        spdlog::spdlog_header_only
)

# Add your source files here
add_executable(USProbeTracker
        ${USPROBETRACKER_SOURCES} ${USPROBETRACKER_HEADERS}
)

target_include_directories(USProbeTracker PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${realsense2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${ZSTD_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(USProbeTracker
        PRIVATE
        ${USPROBETRACKER_LIBS}
)

add_subdirectory(StereoCalibration)

# copy calibration files
add_custom_command(TARGET USProbeTracker POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/StereoCalibration/CalibFiles
        $<TARGET_FILE_DIR:USProbeTracker>/StereoCalibration/CalibFiles
)

# copy configs files
add_custom_command(TARGET USProbeTracker POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/configs
        $<TARGET_FILE_DIR:USProbeTracker>/configs
)

# copy resources files
add_custom_command(TARGET USProbeTracker POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources
        $<TARGET_FILE_DIR:USProbeTracker>/Resources
)

option(ENABLE_TEST "Enabling test case" ON)
if (ENABLE_TEST)
    find_package(Catch2 3.8 REQUIRED)
    enable_testing()
    add_subdirectory(test)
endif ()