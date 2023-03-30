cmake_minimum_required(VERSION 2.8.3)
project(forque_sensor_hardware)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(catkin REQUIRED COMPONENTS
    roscpp std_msgs geometry_msgs actionlib
)

catkin_package(
      INCLUDE_DIRS
        include
      CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        actionlib

)
include_directories(${catkin_INCLUDE_DIRS})

find_package(pr_control_msgs REQUIRED)
include_directories(${pr_control_msgs_INCLUDE_DIRS})

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SRC_FILES   ${SRC_DIR}/main.cpp
                ${SRC_DIR}/wireless_ft.cpp
)
file(GLOB_RECURSE ALL_SOURCE_FILES
  LIST_DIRECTORIES false
  src/*.cpp
)

add_executable(forque_sensor_hardware
  src/main.cpp ${ALL_SOURCE_FILES})

target_link_libraries(forque_sensor_hardware
  ${pr_control_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
  )


include_directories(
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)


install(TARGETS forque_sensor_hardware RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



# ===============================================================================
# Formatting
#

include(ClangFormat)
clang_format_setup(VERSION 6.0)

if(CLANG_FORMAT_EXECUTABLE)

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.h
    include/*.hpp
    src/*.c
    src/*.cpp)

  clang_format_add_sources(${ALL_SOURCE_FILES})

  clang_format_add_targets()
endif()
