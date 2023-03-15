cmake_minimum_required(VERSION 3.16)
project(moving_horizon_jnt_calib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(awesome_utils REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(matlogger2 REQUIRED)

find_package(catkin REQUIRED roscpp rospy urdf sensor_msgs std_msgs message_generation)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

add_service_files(
  FILES
  StartCalib.srv
  PerformCalibTraj.srv
  SetCalibParams.srv
)

add_message_files(
  FILES
  JntCalibStatus.msg
  CalibTrajStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

add_xbot2_plugin(calib_traj_replayer_rt
    src/calib_traj_replayer_rt.cpp)
target_include_directories(calib_traj_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS})
add_dependencies(calib_traj_replayer_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(calib_traj_replayer_rt PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support matlogger2::matlogger2)

add_xbot2_plugin(mhe_rt
    src/mhe_rt.cpp)
target_include_directories(mhe_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS})
add_dependencies(mhe_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(mhe_rt PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support matlogger2::matlogger2)

add_executable(calib_logger src/utils/calib_logger.cpp)
target_link_libraries(calib_logger ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} matlogger2::matlogger2)
target_include_directories(calib_logger PRIVATE ${catkin_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${xbot_msgs_INCLUDE_DIRS})
set_target_properties(calib_logger PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(start_calib_traj src/utils/start_calib_trajectory.cpp)
target_include_directories(start_calib_traj PRIVATE include src ${matlogger2_INCLUDE_DIRS})
add_dependencies(start_calib_traj ${PROJECT_NAME}_generate_messages)
target_link_libraries(start_calib_traj PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support matlogger2::matlogger2)

install(
    TARGETS
    calib_traj_replayer_rt
    mhe_rt
    DESTINATION lib
)

install(TARGETS calib_logger start_calib_traj DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS src/utils/launch_rosbag.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
