cmake_minimum_required(VERSION 3.8)
project(image_collage_maker)

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
find_package(ament_lint_auto REQUIRED)

find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(collage_maker SHARED
  src/collage_maker.cpp)
target_compile_definitions(collage_maker
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(collage_maker rclcpp rclcpp_components std_msgs sensor_msgs cv_bridge)
target_include_directories(collage_maker
  SYSTEM
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  )
target_compile_features(collage_maker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(collage_maker
        ${OpenCV_LIBRARIES}
        )
rclcpp_components_register_nodes(collage_maker "CollageMaker")
set(node_plugins "${node_plugins}CollageMaker;$<TARGET_FILE:collage_maker>\n")

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.

install(TARGETS
  collage_maker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  collage_maker
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
  #config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
