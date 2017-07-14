cmake_minimum_required(VERSION 3.5)
project(gmapping)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(openslam_gmapping REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_console REQUIRED)

find_package(Boost REQUIRED signals system thread program_options)

set(INCLUDE_DIRS
  ${ament_cmake_INCLUDE_DIRS}
  ${openslam_gmapping_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${ros2_console_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS
  ${ament_cmake_LIBRARY_DIRS}
  ${openslam_gmapping_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS}
  ${nav_msgs_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS}
  ${tf2_ros_LIBRARY_DIRS}
  ${ros2_console_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

link_directories(${LIBRARY_DIRS})

set(LIBS
  ${ament_cmake_LIBRARIES}
  ${openslam_gmapping_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${ros2_console_LIBRARIES}
  ${Boost_LIBRARIES} 
)

add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(slam_gmapping ${LIBS})

add_library(slam_gmapping_nodelet src/slam_gmapping.cpp src/nodelet.cpp)
target_link_libraries(slam_gmapping_nodelet ${LIBS})

add_executable(slam_gmapping_replay src/slam_gmapping.cpp src/replay.cpp)
target_link_libraries(slam_gmapping_replay ${LIBS})

install(TARGETS slam_gmapping slam_gmapping_replay
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  if(TARGET tests)
    add_executable(gmapping-rtest EXCLUDE_FROM_ALL test/rtest.cpp)
    target_link_libraries(gmapping-rtest ${LIBS} ${GTEST_LIBRARIES})
    add_dependencies(tests gmapping-rtest)
  endif()

  # Need to make the tests more robust; currently the output map can differ
  # substantially between runs.
  # TODO: need to be able to download test data
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/gmapping/basic_localization_stage_indexed.bag
    DESTINATION share/${PROJECT_NAME}/test MD5
    322a0014f47bcfbb0ad16a317738b0dc)
  catkin_download_test_data(${PROJECT_NAME}_hallway_slow_2011-03-04-21-41-33.bag
    http://download.ros.org/data/gmapping/hallway_slow_2011-03-04-21-41-33.bag
    DESTINATION share/${PROJECT_NAME}/test MD5
    e772b89713693adc610f4c5b96f5dc03)
  catkin_download_test_data(
    ${PROJECT_NAME}_basic_localization_stage_groundtruth.pgm
    http://download.ros.org/data/gmapping/basic_localization_stage_groundtruth.pgm
    DESTINATION share/${PROJECT_NAME}/test MD5
    abf208f721053915145215b18c98f9b3)
  catkin_download_test_data(${PROJECT_NAME}_test_replay_crash.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_replay_crash.bag
    DESTINATION share/${PROJECT_NAME}/test MD5
    bb0e086207eb4fccf0b13d3406f610a1)
  catkin_download_test_data(${PROJECT_NAME}_test_turtlebot.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_turtlebot.bag
    DESTINATION share/${PROJECT_NAME}/test MD5
    402e1e5f7c00445d2a446e58e3151830)
  catkin_download_test_data(${PROJECT_NAME}_test_upside_down.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_upside_down.bag
    DESTINATION share/${PROJECT_NAME}/test MD5
    3b026a2144ec14f3fdf218b5c7077d54)
  set(LOCAL_DEPENDENCIES gmapping-rtest
    ${PROJECT_NAME}_basic_localization_stage_indexed.bag
    ${PROJECT_NAME}_test_replay_crash.bag ${PROJECT_NAME}_test_turtlebot.bag
    ${PROJECT_NAME}_test_upside_down.bag
    ${PROJECT_NAME}_hallway_slow_2011-03-04-21-41-33.bag
    ${PROJECT_NAME}_basic_localization_stage_groundtruth.pgm slam_gmapping
    slam_gmapping_replay)
  add_rostest(test/basic_localization_stage.launch
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_stage_replay.launch
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_stage_replay2.launch
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_symmetry.launch
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_upside_down.launch
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_laser_different_beamcount.test
    DEPENDENCIES ${LOCAL_DEPENDENCIES})
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(openslam_gmapping)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(ros2_console)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(slam_gmapping_nodelet ${LIBS})

ament_package()
