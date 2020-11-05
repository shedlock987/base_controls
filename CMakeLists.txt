cmake_minimum_required(VERSION 2.8.3)
project(pid_controller)

add_compile_options(-std=c++11)

find_package(catkin 
REQUIRED COMPONENTS
roscpp)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
)

#############
## PID Lib ##
#############

add_library(pid_controller_api
  src/pid_controller.cpp
)

target_link_libraries(pid_controller_api
  ${catkin_LIBRARIES}
)

add_dependencies(pid_controller_api
  ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############

catkin_add_gtest(pid_controller_test test/pid_controller_test.cpp 
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/test/data/")

target_include_directories(pid_controller_api
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pid_controller_api 
                      ${catkin_LIBRARIES})

add_dependencies(pid_controller_test
  ${catkin_EXPORTED_TARGETS})
