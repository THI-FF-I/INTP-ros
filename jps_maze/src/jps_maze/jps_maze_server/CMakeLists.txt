add_executable(jps_maze_server src/server.cpp src/board.cpp src/player.cpp src/game.cpp)
target_include_directories(jps_maze_server PRIVATE
        include)
ament_target_dependencies(jps_maze_server rclcpp jps_maze_msgs)