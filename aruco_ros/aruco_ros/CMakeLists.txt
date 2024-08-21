cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils SHARED src/aruco_ros_utils.cpp)
target_include_directories(aruco_ros_utils
  PUBLIC
  include)

target_include_directories(aruco_ros_utils
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_ros_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(aruco_ros_utils ${OpenCV_LIBRARIES})

add_executable(quadruple src/simple_quadruple.cpp
                      src/aruco_ros_utils.cpp)

target_include_directories(quadruple
  PUBLIC
  include)

target_include_directories(quadruple
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(quadruple ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(quadruple ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
target_include_directories(marker_publisher
  PUBLIC
  include)

target_include_directories(marker_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(marker_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS aruco_ros_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS marker_publisher quadruple
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/aruco_ros
)

ament_package()
