cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# Keep runtime resources available from build directory.
if(EXISTS ${CMAKE_BINARY_DIR}/configs AND NOT IS_SYMLINK ${CMAKE_BINARY_DIR}/configs)
    file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/configs)
endif()
if(EXISTS ${CMAKE_BINARY_DIR}/assets AND NOT IS_SYMLINK ${CMAKE_BINARY_DIR}/assets)
    file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/assets)
endif()
if(NOT EXISTS ${CMAKE_BINARY_DIR}/configs)
    file(CREATE_LINK ${PROJECT_SOURCE_DIR}/configs ${CMAKE_BINARY_DIR}/configs SYMBOLIC)
endif()
if(NOT EXISTS ${CMAKE_BINARY_DIR}/assets)
    file(CREATE_LINK ${PROJECT_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/assets SYMBOLIC)
endif()


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/omniperception)

# camera test
add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND sensor_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    add_executable(sentry src/sentry.cpp)
    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

install(
    TARGETS camera_test
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(TARGET sentry)
    install(
        TARGETS sentry
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

install(DIRECTORY configs assets DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
