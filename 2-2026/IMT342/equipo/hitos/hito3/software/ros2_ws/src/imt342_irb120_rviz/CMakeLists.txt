cmake_minimum_required(VERSION 3.8)
project(imt342_irb120_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  src/joint_panel.cpp
  src/reference_axes.cpp
  src/analysis_panel.cpp
  src/analysis_widget.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  rviz_common rviz_rendering rviz_ogre_vendor pluginlib rclcpp sensor_msgs std_msgs geometry_msgs trajectory_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

include(CTest)
if(BUILD_TESTING)
  add_executable(panel_navigation_test test/panel_navigation_test.cpp src/analysis_widget.cpp)
  target_include_directories(panel_navigation_test PRIVATE include)
  ament_target_dependencies(panel_navigation_test
    rviz_common pluginlib rclcpp sensor_msgs std_msgs geometry_msgs trajectory_msgs)
  target_link_libraries(panel_navigation_test Qt5::Widgets)
  add_test(NAME panel_navigation_test COMMAND panel_navigation_test)
  set_tests_properties(panel_navigation_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  add_executable(panel_values_test test/panel_values_test.cpp)
  target_include_directories(panel_values_test PRIVATE include)
  add_test(NAME panel_values_test COMMAND panel_values_test)
  add_executable(task_editor_test test/task_editor_test.cpp)
  target_include_directories(task_editor_test PRIVATE include)
  target_link_libraries(task_editor_test Qt5::Widgets)
  add_test(NAME task_editor_test COMMAND task_editor_test)
  set_tests_properties(task_editor_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
  add_executable(panel_pose_test test/panel_pose_test.cpp)
  target_link_libraries(panel_pose_test PRIVATE Qt5::Widgets)
  add_test(NAME panel_pose_test COMMAND panel_pose_test)
  add_executable(analysis_widget_test test/analysis_widget_test.cpp src/analysis_widget.cpp)
  target_link_libraries(analysis_widget_test PRIVATE Qt5::Widgets)
  add_test(NAME analysis_widget_test COMMAND analysis_widget_test)
  set_tests_properties(analysis_widget_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()

ament_package()
