cmake_minimum_required(VERSION 3.8)

# the project is intended to be built on NVIDIA Jetson targets
project(yolov8 LANGUAGES CXX CUDA)

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 87)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_lint_auto REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")

# CUDA
find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(yolov8_detector SHARED
  src/yolov8_detector.cpp)
target_compile_definitions(yolov8_detector
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(yolov8_detector rclcpp rclcpp_components std_msgs sensor_msgs vision_msgs cv_bridge)
target_include_directories(yolov8_detector
  SYSTEM
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  )
target_compile_features(yolov8_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(yolov8_detector
        nvinfer nvinfer_plugin
        ${CUDA_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${TensorRT_LIBRARIES}
        )
rclcpp_components_register_nodes(yolov8_detector "Yolov8Detector")
set(node_plugins "${node_plugins}Yolov8Detector;$<TARGET_FILE:yolov8_detector>\n")

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.
#ament_export_include_directories()

install(TARGETS
  yolov8_detector
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  yolov8_detector
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
