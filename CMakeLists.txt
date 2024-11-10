cmake_minimum_required(VERSION 3.8)
project(imu_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(imu_reader_node src/imu_reader_node.cpp)
ament_target_dependencies(imu_reader_node rclcpp sensor_msgs)
target_link_libraries(imu_reader_node Boost::system)


install(TARGETS imu_reader_node DESTINATION lib/${PROJECT_NAME})


ament_package()
