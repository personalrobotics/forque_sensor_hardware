cmake_minimum_required(VERSION 2.8.3)
project(forque_sensor_hardware)


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

find_package(netft_rdt_driver REQUIRED)
include_directories(${netft_rdt_driver_INCLUDE_DIRS})

find_package(pr_hardware_interfaces REQUIRED)
include_directories(${pr_hardware_interfaces_INCLUDE_DIRS})

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SRC_FILES   ${SRC_DIR}/main.cpp
                ${SRC_DIR}/ForqueSensorHW.cpp
)

add_executable(forque_sensor_hardware
  src/main.cpp ${SRC_FILES})

target_link_libraries(forque_sensor_hardware
  ${pr_hardware_interfaces_LIBRARIES}
  ${netft_rdt_driver_LIBRARIES}
  ${catkin_LIBRARIES}
  )


include_directories(
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)


install(TARGETS forque_sensor_hardware RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
