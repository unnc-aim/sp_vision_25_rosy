cmake_minimum_required(VERSION 3.16)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

add_library(tools STATIC 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    profile_log.cpp
    pid.cpp
    crc.cpp
)

target_link_libraries(tools ${OpenCV_LIBS} fmt::fmt)
