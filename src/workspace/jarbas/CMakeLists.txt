cmake_minimum_required(VERSION 3.8)
project(revision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Include this if your code depends on rclcpp
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(jarbas include/revision/revision.cpp)
ament_target_dependencies(jarbas rclcpp std_msgs)  # Specify dependencies here

# Install the executable
install(TARGETS jarbas
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
