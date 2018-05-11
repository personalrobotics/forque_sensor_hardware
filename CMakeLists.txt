cmake_minimum_required(VERSION 2.8.3)
project(forque_hardware_interface)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    controller_manager
)

catkin_package(
      INCLUDE_DIRS
        include
      CATKIN_DEPENDS
        controller_manager
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})

find_package(aikido 0.0.2 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner
    planner_ompl
    planner_parabolic
    planner_vectorfield
    robot
    rviz
    statespace
)
include_directories(${aikido_INCLUDE_DIRS})


set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SRC_FILES   ${SRC_DIR}/netft/netft_rdt_driver.cpp
                ${SRC_DIR}/netft/netft_rdt_bias.cpp
                ${SRC_DIR}/netft/ft_listener.cpp
                ${SRC_DIR}/ForceTorqueSensorHW.cpp
                ${SRC_DIR}/TestRobotHW.cpp

)

add_executable(forque_hardware_interface_test
  src/main.cpp ${SRC_FILES})

target_link_libraries(forque_hardware_interface_test
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
  )


include_directories(
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)


install(TARGETS forque_hardware_interface_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})