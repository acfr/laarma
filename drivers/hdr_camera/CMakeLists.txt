cmake_minimum_required(VERSION 3.8)
project(hdr_camera)

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
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_lint_auto REQUIRED)

find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(arena_sdk_conf "/etc/ld.so.conf.d/Arena_SDK.conf")
if (NOT EXISTS ${arena_sdk_conf})
  execute_process(COMMAND bash -c "cd ${CMAKE_CURRENT_SOURCE_DIR} && chmod +x install_sdk.sh && ./install_sdk.sh")
endif ()
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindARENA.cmake")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(cam_streamer SHARED
  src/cam_streamer.cpp)
target_compile_definitions(cam_streamer
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(cam_streamer rclcpp rclcpp_components std_msgs sensor_msgs cv_bridge camera_info_manager image_transport)
target_include_directories(cam_streamer
  SYSTEM
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${arena_sdk_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
  )
target_compile_features(cam_streamer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(cam_streamer
        ${OpenCV_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        )
rclcpp_components_register_nodes(cam_streamer "CamStreamer")
set(node_plugins "${node_plugins}CamStreamer;$<TARGET_FILE:cam_streamer>\n")

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.
ament_export_include_directories(${arena_sdk_INCLUDES})

install(TARGETS
  cam_streamer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  cam_streamer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
