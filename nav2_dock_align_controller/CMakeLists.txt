cmake_minimum_required(VERSION 3.5)
project(nav2_dock_align_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(cv_bridge REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
# find_package(message_generation)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

set(dependencies 
  cv_bridge
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs 
  image_geometry 
  image_transport
  rclcpp 
  pluginlib
  sensor_msgs 
  std_msgs 
  tf2
  builtin_interfaces
)

set(library_name dock_align_controller_core)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

include_directories(
    include
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ${library_name} SHARED 
  src/common_functions.cpp 
  src/dock_align_controller.cpp 
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(${library_name}
  ${dependencies}
)
rosidl_target_interfaces(${library_name} ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${library_name}
  ${EIGEN3_LIBRARIES}
  ${OPENCV_LIBRARIES}
  apriltag::apriltag
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core dock_align_controller_plugin.xml)

ament_package()
