cmake_minimum_required(VERSION 3.8)
project(two_assignament_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

add_executable(apriltag_manager_node src/apriltag_manager.cpp)

ament_target_dependencies(apriltag_manager_node rclcpp apriltag_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

target_include_directories(apriltag_manager_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltag_manager_node ${OpenCV_LIBS})

install(TARGETS apriltag_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# build the driver node
add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp rclcpp_action geometry_msgs std_msgs nav2_msgs)
install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


add_executable(table_detector_node src/table_detector.cpp)
ament_target_dependencies(table_detector_node rclcpp sensor_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS
  table_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
