cmake_minimum_required(VERSION 2.8.3)
project(thu_data_glove)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  tf
  tf2_ros
  message_filters
)

find_package(Qt5 REQUIRED COMPONENTS Core Network Gui Widgets)

add_message_files(
  FILES
  GloveData.msg
  SensorData.msg
  SensorDataSingle.msg
  ImuArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
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
  include/${PROJECT_NAME}/calculator.h
  include/${PROJECT_NAME}/datastruct.h
  include/${PROJECT_NAME}/glovemanager.h
  include/${PROJECT_NAME}/protocol.h
  include/${PROJECT_NAME}/rosbridge.h
)
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HEADERS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_MOCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Network
  Qt5::Gui
  ${catkin_LIBRARIES}
)

add_executable(viz src/viz.cpp)
add_dependencies(viz ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(viz ${catkin_LIBRARIES})

add_executable(kinematic_filter src/kinematic_filter.cpp)
add_dependencies(kinematic_filter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(kinematic_filter ${catkin_LIBRARIES})

add_executable(state_publisher src/state_publisher.cpp)
add_dependencies(state_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(state_publisher ${catkin_LIBRARIES})

add_executable(imu_tf src/imu_tf.cpp)
add_dependencies(imu_tf ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(imu_tf ${catkin_LIBRARIES})
