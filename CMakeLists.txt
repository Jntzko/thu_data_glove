cmake_minimum_required(VERSION 2.8.3)
project(glove)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Core Network Gui Widgets)

add_message_files(
  FILES
  GloveData.msg
  SensorData.msg
  SensorDataSingle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES glove
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
  DEPENDS system_lib
)

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/glovemanager.cpp
  src/protocol.cpp
  src/rosbridge.cpp
  src/calculator.cpp
)
set(${PROJECT_NAME}_HEADERS
  src/calculator.h
  src/datastruct.h
  src/glovemanager.h
  src/protocol.h
  src/rosbridge.h
)
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HEADERS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_MOCS})

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Network
  Qt5::Gui
  ${catkin_LIBRARIES}
)
