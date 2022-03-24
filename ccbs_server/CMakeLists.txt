cmake_minimum_required(VERSION 3.5)
project(mapf_server)

## Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()
#
## Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()
#
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()


set(Boost_USE_STATIC_LIBS OFF) 
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mapf_actions REQUIRED)
find_package(Boost COMPONENTS multi_index)
find_package(nav2_costmap_2d)
find_package(nav2_core)
find_package(nav2_util)

include_directories( ${Boost_INCLUDE_DIRS} )

include_directories(
        include
        include/mapf_server
        include/ccbs
        include/simplex
)
set( SOURCE_FILES 
     src/ccbs/main.cpp  
     src/ccbs/config.cpp
     src/ccbs/tinyxml2.cpp
     src/ccbs/xml_logger.cpp 
     src/ccbs/map.cpp 
     src/ccbs/heuristic.cpp 
     src/ccbs/sipp.cpp 
     src/ccbs/task.cpp
     src/ccbs/cbs.cpp 
     src/simplex/columnset.cpp
     src/simplex/constraint.cpp
     src/simplex/matrix.cpp
     src/simplex/objectivefunction.cpp
     src/simplex/simplex.cpp
     src/simplex/variable.cpp
     src/mapf_server.cpp)

set( dependencies
        rclcpp
        std_msgs
        nav_msgs
        nav2_msgs
        nav2_util
        tf2_ros
        rclcpp_action
        rclcpp_lifecycle
        nav2_costmap_2d
        nav2_core
        mapf_actions)

add_executable( mapf_server ${SOURCE_FILES})
ament_target_dependencies(mapf_server ${dependencies})

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

install(TARGETS
  mapf_server
  DESTINATION lib/${PROJECT_NAME}
)
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
ament_package()
