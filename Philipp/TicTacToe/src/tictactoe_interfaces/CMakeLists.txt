cmake_minimum_required(VERSION 3.8)
project(tictactoe_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Board.msg"
  "msg/Field.msg"
  "msg/State.msg"
  "srv/AddPlayer.srv"
  "srv/Turn.srv"
  )

ament_package()
