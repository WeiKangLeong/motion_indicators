cmake_minimum_required(VERSION 2.8.3)
project(motion_indicators)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
  roscpp
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  pcl_ros
  visualization_msgs
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf nav_msgs sensor_msgs 
  DEPENDS TinyXML
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS}
)


find_package(Boost REQUIRED COMPONENTS filesystem)

IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
ENDIF()

add_library (${PROJECT_NAME}
src/SvgPath.cpp
)


#common commands for building c++ executables and libraries

add_executable(special_region src/special_region.cpp)
target_link_libraries(special_region ${catkin_LIBRARIES} ${TINYXML_LIBRARIES} ${CURL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${PROJECT_NAME})
