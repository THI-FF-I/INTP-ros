# How to install (Ubuntu 22.04)

1. Clone Git Repository: `git clone https://github.com/THI-FF-I/INTP-ros.git`
2. Install ROS2 humble & ncurses:<br>
ROS: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html<br>
ncurses: `sudo apt-get install libncurses5-dev libncursesw5-dev`
3. CD into Git Repository: `cd INTP-ros`
4. Build: `./jps_maze/abuild.sh`

# How to play
1. Source *(on every new window)*: `source jps_maze/install/setup.bash`
2. Start **Server**: `ros2 launch jps_maze jps_maze_launch.py`
3. Enter `-s -b [BOARD NUMBER] --ppt [PLAYER PER TEAM, standard = 2]`
4. Start **Player** *(repeat for every player)*: `ros2 launch jps_maze jps_maze_launch.py`
5. Enter `-c -t [TEAM: 'A' or 'B'] -n [NAME OF PLAYER]`
6. Game starts automatically if all players are connected

# Credits
- **Johan Bücker** - ROS & UDP communication, ncurses visualizer
- **Philipp Grüber** - game engine
- **Simon Demuth** - node.js visualizer *(not working)*
