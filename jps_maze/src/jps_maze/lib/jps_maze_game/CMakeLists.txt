add_library(jps_maze_game STATIC src/game.cpp src/board.cpp src/player.cpp)
target_include_directories(jps_maze_game PUBLIC
        include)
ament_target_dependencies(jps_maze_game rclcpp jps_maze_msgs)