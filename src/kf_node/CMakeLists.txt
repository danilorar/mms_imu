cmake_minimum_required(VERSION 3.8)
project(kf_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(imu_kf_node src/kf_noise.cpp)
ament_target_dependencies(imu_kf_node rclcpp sensor_msgs)
target_link_libraries(imu_kf_node Eigen3::Eigen)

add_executable(kf_angles src/kf_angles.cpp)
ament_target_dependencies(kf_angles rclcpp sensor_msgs geometry_msgs)

install(TARGETS 
imu_kf_node
kf_angles
 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
