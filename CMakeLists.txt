cmake_minimum_required(VERSION 3.5)
project(ros2_dds_camera_benchmark)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_subscriber_node src/image_subscriber_node.cpp)
ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs)

install(TARGETS
  image_subscriber_node
  DESTINATION lib/${PROJECT_NAME})


install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
