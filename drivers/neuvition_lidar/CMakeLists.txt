cmake_minimum_required(VERSION 3.8)
project(neuvition_lidar)

set(CMAKE_CXX_FLAGS "-g3 -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(ament_lint_auto REQUIRED)

find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(CURL REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(neuvition_sdk_conf "/opt/neuvition/lib/libneusdk.so")
if (NOT EXISTS ${neuvition_sdk_conf})
  execute_process(COMMAND bash -c "cd ${CMAKE_CURRENT_SOURCE_DIR} && chmod +x install_sdk.sh && ./install_sdk.sh")
endif ()
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindNeuvition.cmake")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(lidar_streamer SHARED
  src/lidar_streamer.cpp)
target_compile_definitions(lidar_streamer
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(lidar_streamer rclcpp rclcpp_components std_msgs cv_bridge sensor_msgs pcl_conversions pcl_ros OpenCV)
target_include_directories(lidar_streamer
  SYSTEM
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${neuvition_sdk_INCLUDES}
  )
target_compile_features(lidar_streamer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(lidar_streamer
        ${neuvition_sdk_LIBRARIES} # need to explicitly add here
        ${Boost_LIBRARIES} # need to explicitly add here
        ${CURL_LIBRARY} # need to explicitly add here
        )
rclcpp_components_register_nodes(lidar_streamer "LidarStreamer")
set(node_plugins "${node_plugins}LidarStreamer;$<TARGET_FILE:lidar_streamer>\n")

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.
ament_export_include_directories(${neuvition_sdk_INCLUDES})

install(TARGETS
  lidar_streamer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_streamer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES
  ${neuvition_sdk_LIBRARIES}
  DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
