import argparse, re
import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package
def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--resolution', type=str, dest='res', required=True, help='Resolution of the maze (<width>x<height>)')
    run_mode_group = parser.add_mutually_exclusive_group(required=True)
    run_mode_group.add_argument('-s', '--server', dest='start_server', action='store_true', help='Start the server node')
    run_mode_group.add_argument('-c', '--client', dest='start_server', action='store_false', help='Start the client node')
    parser.add_argument('-t', '--team', type=str, choices=['A', 'a', 'B', 'b'], dest='team', required=False, help='I client mode is chosen, select the team to play on')
    args = parser.parse_args()
    matches = re.search('^([0-9]+)x([0-9]+)$', args.res)
    if matches:
        width = int(matches.group(1))
        height = int(matches.group(2))
        if not args.start_server and args.team:
            return width, height, args.start_server, True if args.team.upper() == 'A' else  False
        else:
            parser.error('Missing team selection')
    else:
        parser.error('Invalid formatting of resolution')

def generate_launch_description():
    width, height, start_server, team_A = parse_arguments()
    package = 'jps_maze'
    node_ns = 'jps_maze'
    if start_server:
        node_name = 'server_node'
        executable = 'jps_maze_server'
    else:
        node_name = 'client_node' + str(random.randrange(1000))
        executable = 'jps_maze_client'
        if team_A:
            team = 'team_A'
        else:
            team = 'team_B'
    os.environ['node_ns'] = node_ns
    os.environ['node_name'] = node_name
    os.environ['create_player_topic'] = '/' + node_ns + 'create_player'
    os.environ['get_status_topic'] = '/' + node_ns +'/' +  team + '/status'
    os.environ['team_A'] = str(team_A)
    return LaunchDescription(
        [
            Node(
                package=package,
                namespace=node_ns,
                executable=executable,
                name=node_name,
                parameters=[
                    launch_ros.parameter_descriptions.ParameterFile(
                        param_file=get_share_file_path_from_package(package_name= package, file_name='parameters.yaml'),
                        allow_substs=True),
                ]
            ),
        ]
    )

generate_launch_description()