add_library(jps_maze_visualizer STATIC
        src/visualizer.cpp)
target_include_directories(jps_maze_visualizer PUBLIC
        include)
ament_target_dependencies(jps_maze_visualizer rclcpp)
