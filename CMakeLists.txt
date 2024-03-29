cmake_minimum_required(VERSION 3.8)
project(main_app)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(data_interface REQUIRED)


add_subdirectory(solver)
add_subdirectory(sensor_node)
add_subdirectory(main_node)

add_executable(sensor
  src/sensor.cpp
)

add_executable(main_swc
  src/main_swc.cpp
)

#target_include_directories(main_swc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#ament_target_dependencies(main_swc
#  rclcpp
#  data_interface
#)

target_link_libraries(sensor SensorNode)
target_link_libraries(main_swc MainNode)

install(TARGETS sensor main_swc
 DESTINATION lib/${PROJECT_NAME})

ament_package()
