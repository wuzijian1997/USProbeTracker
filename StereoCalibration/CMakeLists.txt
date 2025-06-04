set(SOURCES
        main.cpp
        RealSense.cpp
        StereoCalibClass.cpp
)

set(HEADERS
        RealSense.h
        StereoCalibClass.h
)

add_executable(StereoCalibration ${SOURCES} ${HEADERS})

target_include_directories(StereoCalibration PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIR}
        ${ZSTD_INCLUDE_DIRS}
)

target_link_libraries(StereoCalibration
        Eigen3::Eigen
        ${OpenCV_LIBS}
        realsense2::realsense2
        zstd::libzstd
)