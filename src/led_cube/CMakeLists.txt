cmake_minimum_required(VERSION 3.8)
project(led_cube)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(led_cube_interfaces REQUIRED)

add_executable(led_cube src/led_cube.cpp)
ament_target_dependencies(led_cube rclcpp std_msgs visualization_msgs led_cube_interfaces)
target_include_directories(led_cube PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(led_cube PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


add_executable(colors_pub src/colors_pub.cpp)
ament_target_dependencies(colors_pub rclcpp std_msgs visualization_msgs led_cube_interfaces)
target_include_directories(colors_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(colors_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS colors_pub led_cube
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
