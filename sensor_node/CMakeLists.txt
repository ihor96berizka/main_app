cmake_minimum_required(VERSION 3.8)
project(SensorNode VERSION 1.0.0 LANGUAGES CXX)

message("------------Configuring Sensor Node lib-------------")
set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(GNUInstallDirs)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_node.cpp
)
set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensor_node.hpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "data_interface"
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensor_node.hpp)

message("----------------done-----------------")