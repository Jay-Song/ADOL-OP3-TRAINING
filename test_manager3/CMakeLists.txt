cmake_minimum_required(VERSION 2.8.3)
project(test_manager3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller_msgs
  cmake_modules
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  op3_kinematics_dynamics  
  adol_open_cr_module
  op3_base_module
  test_module3
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp robotis_controller_msgs cmake_modules dynamixel_sdk robotis_framework_common robotis_device robotis_controller robotis_math op3_kinematics_dynamics adol_open_cr_module op3_base_module test_module3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/test_manager3.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

