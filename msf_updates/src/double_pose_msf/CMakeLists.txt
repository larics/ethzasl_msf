add_executable(double_pose_sensor main.cpp)
target_link_libraries(double_pose_sensor pose_distorter ${catkin_LIBRARIES})
add_dependencies(double_pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
