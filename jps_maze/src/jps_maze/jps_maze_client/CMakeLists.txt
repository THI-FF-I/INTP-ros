add_executable(jps_maze_client src/client.cpp)
target_include_directories(jps_maze_client PRIVATE
        include)
ament_target_dependencies(jps_maze_client rclcpp jps_maze_msgs)