add_executable(jps_maze_server src/server.cpp)
target_include_directories(jps_maze_server PRIVATE
        include)
ament_target_dependencies(jps_maze_server rclcpp jps_maze_msgs)