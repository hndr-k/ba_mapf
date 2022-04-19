cmake_minimum_required(VERSION 3.8)
project(ecbs_server)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mapf_actions REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(yaml-cpp)

include_directories(
        inc/
)

set( SOURCE_FILES
      src/ecbs_server.cpp
      src/main.cpp)

add_executable(ecbs_server ${SOURCE_FILES})
target_include_directories(ecbs_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ecbs_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ecbs_server
  "rclcpp"
  "rclcpp_lifecycle"
  "nav2_costmap_2d"
  "std_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "tf2"
  "nav_msgs"
  "nav2_msgs"
  "nav2_util"
  "geometry_msgs"
  "rclcpp_action"
  "mapf_actions"
)
target_link_libraries(ecbs_server
  ${Boost_LIBRARIES}
  yaml-cpp
)


install(TARGETS ecbs_server
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
# Install map files.
install(DIRECTORY
maps
DESTINATION share/${PROJECT_NAME}/
)
# Install param files.
install(DIRECTORY
params
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
