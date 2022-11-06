add_executable(jps_maze_server src/server.cpp ../lib/jps_maze_game/src/board.cpp ../lib/jps_maze_game/src/player.cpp ../lib/jps_maze_game/src/game.cpp)
target_include_directories(jps_maze_server PRIVATE
        include)
target_link_libraries(jps_maze_server
        jps_maze_game)
ament_target_dependencies(jps_maze_server rclcpp jps_maze_msgs)