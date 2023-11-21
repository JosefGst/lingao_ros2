cmake_minimum_required(VERSION 3.8)
project(lingao_base_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lingao_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options regex )
include_directories( ${Boost_INCLUDE_DIRS} )


include_directories(include)

add_executable(lingao_base_node src/lingao_base_node.cpp 
  src/base_driver.cpp
  src/myObject.cpp
  src/Serial_Async.cpp
  src/data_stream.cpp
  # src/TCP_Async.cpp
# src/UDP_Async.cpp
)
ament_target_dependencies(lingao_base_node rclcpp tf2 tf2_ros nav_msgs lingao_msgs geometry_msgs sensor_msgs std_msgs)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(lingao_base_node ${Boost_LIBRARIES})


# add_library(data_stream src/data_stream.cpp
#   
#   # src/base_driver.cpp
#   # src/Serial_Async.cpp
#   # src/TCP_Async.cpp
#   # src/UDP_Async.cpp
# )
# ament_target_dependencies(data_stream rclcpp std_msgs)

# Install our headers
install(
  DIRECTORY include/lingao_base_ros2
  DESTINATION include
)

# Install our node
install(TARGETS
  lingao_base_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install library
# install(
#   TARGETS data_stream
#   EXPORT data_stream
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
