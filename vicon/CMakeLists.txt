cmake_minimum_required(VERSION 3.8)
project(vicon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_library(vicon_lib NAMES ViconDataStreamSDK_CPP PATHS ${CMAKE_SOURCE_DIR}/libs/Vicon_DataStreamSDK_Linux64 REQUIRED NO_DEFAULT_PATH)
link_directories(libs/Vicon_DataStreamSDK_Linux64)
add_executable(pose_publisher src/pose_publisher.cpp)
add_executable(multi_robot_pose_publisher src/multi_robot_pose_publisher.cpp)
add_executable(pose_stamped_publisher src/pose_stamped_publisher.cpp)

target_link_libraries(pose_publisher ViconDataStreamSDK_CPP)
target_link_libraries(multi_robot_pose_publisher ViconDataStreamSDK_CPP)
target_link_libraries(pose_stamped_publisher ViconDataStreamSDK_CPP)

ament_target_dependencies(pose_publisher rclcpp std_msgs geometry_msgs)
ament_target_dependencies(multi_robot_pose_publisher rclcpp std_msgs geometry_msgs)
ament_target_dependencies(pose_stamped_publisher rclcpp geometry_msgs std_msgs)

install(TARGETS
  pose_publisher
  multi_robot_pose_publisher
  pose_stamped_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY libs/Vicon_DataStreamSDK_Linux64/
  DESTINATION lib)

ament_package()
