cmake_minimum_required(VERSION 3.8)
project(simple_image_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(softsys_msgs REQUIRED)


add_executable(image_subscriber_node src/image_subscriber_node.cpp)
ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs cv_bridge OpenCV std_msgs softsys_msgs)

install(TARGETS image_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()


