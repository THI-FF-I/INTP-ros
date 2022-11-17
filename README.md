# How to install (Ubuntu 22.04)

1. Clone Git
2. Install ROS2 humble & curses:<br>
ROS: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html<br>
`sudo apt-get install libncurses5-dev libncursesw5-dev`

# How to play
1. Source: `source jps_maze/install/setup.bash`
2. Start Server: `ros2 launch jps_maze jps_maze_launch.py`
3. Enter `-s -b [BOARD NUMBER] --ppt [PLAYER PER TEAM, standard = 2]`
4. Start Player (repeat for every player): `ros2 launch jps_maze jps_maze_launch.py`
5. Enter `-c -t [TEAM: 'A' or 'B'] -n [NAME OF PLAYER]`
6. Game starts automatically if all players are connected
