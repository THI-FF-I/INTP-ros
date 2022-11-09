import argparse, re, sys, os, random
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnShutdown
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package
def parse_arguments():
    parser = argparse.ArgumentParser(prog='jps_maze')
    run_mode_group = parser.add_mutually_exclusive_group(required=True)
    run_mode_group.add_argument('-s', '--server', dest='start_server', action='store_true', help='Start the server node')
    run_mode_group.add_argument('-c', '--client', dest='start_server', action='store_false', help='Start the client node')
    parser.add_argument('-t', '--team', type=str, choices=['A', 'a', 'B', 'b'], dest='team', required=False, help='If client mode is chosen, select the team to play on')
    parser.add_argument('-n', '--name', type=str, dest='name', required=False, help='If client mode is chosen, select the player name')
    parser.add_argument('--ns', '--node_ns', type=str, dest='node_ns', required=False, default='jps_maze', help='Set the namespace to use')
    parser.add_argument('-b', '--board_no', type=int, choices=range(0,1), dest='board_no', required=False, default=0, help='Select the index for the board to use')
    parser.add_argument('-d', '--debug', dest='debug', action='store_true', required=False, default=False, help='Set the log-level to debug for more verbose logging')
    parser.add_argument('--ppt', '--player_per_team', type=int, dest='player_per_team', required=False, default=2, help='Specify the amount of players per team')
    parser.add_argument('--host_name', type=str, dest='host_name', required=False, default='localhost', help='Specify the hostname to connect to for visualisation')
    parser.add_argument('-p', '--port', type=int, dest='target_port', required=False, default=42069, help='Specify the port to connect to')
    parser.print_usage()
    args = parser.parse_args(input('Enter options:\n').split())
    if not 0 < args.target_port < 65535:
        parser.error('Invalid port')
    if not args.start_server:
        if args.team:
            if args.name:
                return args.node_ns, args.start_server, True if args.team.upper() == 'A' else  False, args.name, args.board_no, args.debug, args.player_per_team, args.host_name, args.target_port
            else:
                parser.error('Missing player name')
        else:
            parser.error('Missing team selection')
    else:
        return args.node_ns, args.start_server, True, 'server', args.board_no, args.debug, args.player_per_team, args.host_name, args.target_port

def generate_launch_description():
    node_ns, start_server, team_A, player_name, board_no, debug, player_per_team, host_name, target_port = parse_arguments()
    package = 'jps_maze'
    if start_server:
        node_name = 'server'
        executable = 'jps_maze_server'
        os.environ['status_topic'] = '/status'
    else:
        node_name = 'client_' + player_name + '_' + str(random.randrange(10000))
        executable = 'jps_maze_client'
        if team_A:
            team = 'team_A'
        else:
            team = 'team_B'
        os.environ['status_topic'] = '/' + node_ns +'/' +  team + '/status'
    os.environ['node_ns'] = node_ns
    os.environ['node_name'] = node_name
    os.environ['create_player_topic'] = '/' + node_ns + '/create_player'
    os.environ['move_player_topic'] = '/' + node_ns + '/move_player'
    os.environ['next_round_topic'] = '/' + node_ns + 'next_round'
    os.environ['team_A'] = str(team_A)
    os.environ['player_name'] = player_name
    os.environ['board_path'] = get_share_file_path_from_package(package_name=package, file_name='board' + str(board_no) + '.csv')
    os.environ['player_per_team'] = str(player_per_team)
    os.environ['host_name'] = host_name
    os.environ['target_port'] = str(target_port)
    if debug:
        ros_arguments=['--log-level', 'DEBUG']
    else:
        ros_arguments=[]
    return LaunchDescription(
        [
            Node(
                package=package,
                namespace=node_ns,
                executable=executable,
                name=node_name,
                parameters=[
                    launch_ros.parameter_descriptions.ParameterFile(
                        param_file=get_share_file_path_from_package(package_name=package, file_name='parameters.yaml'),
                        allow_substs=True),
                ],
                ros_arguments=ros_arguments,
            ),
            RegisterEventHandler(
                OnProcessStart(
                    on_start=[LogInfo(
                        msg=['Starting nodejs server: ']
                    )]
                )
            ),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[LogInfo(
                        msg=['Stopping nodejs server: ']
                    )]
                )
            )
        ]
    )
