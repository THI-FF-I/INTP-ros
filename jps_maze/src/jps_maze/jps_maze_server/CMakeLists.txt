add_executable(jps_maze_server src/server.cpp)
target_include_directories(jps_maze_server PRIVATE
        include)
target_link_libraries(jps_maze_server
        jps_maze_game)
ament_target_dependencies(jps_maze_server rclcpp std_msgs jps_maze_msgs)