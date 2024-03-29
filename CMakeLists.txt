cmake_minimum_required(VERSION 3.12)
project(forque_sensor_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

## Building
include_directories(include)
add_executable(forque_sensor_hardware src/main.cpp src/wireless_ft.cpp)
ament_target_dependencies(forque_sensor_hardware rclcpp std_msgs geometry_msgs std_srvs)

install(TARGETS
  forque_sensor_hardware
  DESTINATION lib/${PROJECT_NAME})

## Testing
if(BUILD_TESTING)
  ### Linting
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
