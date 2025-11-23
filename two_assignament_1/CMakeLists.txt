cmake_minimum_required(VERSION 3.8)
project(two_assignament_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

add_executable(apriltag_manager_node src/apriltag_manager.cpp)
ament_target_dependencies(apriltag_manager_node rclcpp apriltag_msgs sensor_msgs geometry_msgs tf2)

target_include_directories(apriltag_manager_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltag_manager_node ${OpenCV_LIBS})

install(TARGETS apriltag_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()