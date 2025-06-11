# Catch2 Unit test
add_executable(USProbeTrackerTest
    USProbeTrackerTest.cpp
    ${SOURCE_DIR}/Utils.cpp
)

target_link_libraries(USProbeTrackerTest
        PRIVATE Catch2::Catch2 Catch2::Catch2WithMain ${USPROBETRACKER_LIBS}
)

target_include_directories(USProbeTrackerTest PRIVATE ${CMAKE_SOURCE_DIR}/include)