add_executable(jps_maze_client src/client.cpp)
target_include_directories(jps_maze_client PRIVATE
        include)
target_link_libraries(jps_maze_client
        jps_maze_game
        jps_maze_visualizer)
ament_target_dependencies(jps_maze_client rclcpp std_msgs jps_maze_msgs)