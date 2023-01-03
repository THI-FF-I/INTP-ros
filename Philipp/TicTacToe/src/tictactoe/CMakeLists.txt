cmake_minimum_required(VERSION 3.5)
project(tictactoe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tictactoe_interfaces REQUIRED)


add_executable(server src/server.cpp)
target_include_directories(server PRIVATE
        include)
ament_target_dependencies(server rclcpp std_msgs tictactoe_interfaces)

install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})


add_executable(player src/player.cpp)
target_include_directories(player PRIVATE
        include)
ament_target_dependencies(player rclcpp std_msgs tictactoe_interfaces)

install(TARGETS
  server
  player
  DESTINATION lib/${PROJECT_NAME})

ament_package()