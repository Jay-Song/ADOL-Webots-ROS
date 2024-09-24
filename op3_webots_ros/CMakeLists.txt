cmake_minimum_required(VERSION 3.0.2)
project(op3_webots_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  std_msgs
  sensor_msgs
  webots_ros
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

# /usr/local/webots/lib/controller/libCppController.so
# VARIABLE_NAME "libarary_name (file_name)" "folder_path (the loacation of the library)"
set(WEBOTS_HOME /usr/local/webots)
FIND_LIBRARY(WEBOTS_CPP_LIBRARY CppController ${WEBOTS_HOME}/lib/controller)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES op3_webots_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${WEBOTS_HOME}/include/controller/cpp
)

add_executable(${PROJECT_NAME}_node src/op3_webots_ros_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(op3_webots_extern 
  src/op3_webots_extern.cpp
  src/op3_extern_ros_controller.cpp
)
add_dependencies(op3_webots_extern ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(op3_webots_extern
  ${catkin_LIBRARIES}
  ${WEBOTS_CPP_LIBRARY}
  ${YAML_CPP_LIBRARIES}
)