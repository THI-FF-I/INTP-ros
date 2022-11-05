import argparse, re, sys, os, random
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package
def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--resolution', type=str, dest='res', metavar='<width>x<height>', required=True, help='Resolution of the maze (<width>x<height>)')
    run_mode_group = parser.add_mutually_exclusive_group(required=True)
    run_mode_group.add_argument('-s', '--server', dest='start_server', action='store_true', help='Start the server node')
    run_mode_group.add_argument('-c', '--client', dest='start_server', action='store_false', help='Start the client node')
    parser.add_argument('-t', '--team', type=str, choices=['A', 'a', 'B', 'b'], dest='team', required=False, help='If client mode is chosen, select the team to play on')
    parser.add_argument('-n', '--name', type=str, dest='name', required=False, help='I client mode is chosen, select the player name')
    parser.add_argument('--ns', '--node_ns', type=str, dest='node_ns', required=False, default='jps_maze', help='Set the namespace to use')
    parser.print_usage()
    args = parser.parse_args(input('Enter options:\n').split())
    matches = re.search('^([0-9]+)x([0-9]+)$', args.res)
    if matches:
        width = matches.group(1)
        height = matches.group(2)
        if not args.start_server:
            if args.team:
                if args.name:
                    return args.node_ns, width, height, args.start_server, True if args.team.upper() == 'A' else  False, args.name
                else:
                    parser.error('Missing player name')
            else:
                parser.error('Missing team selection')
        else:
            return args.node_ns, width, height, args.start_server, True, 'server'
    else:
        parser.error('Invalid formatting of resolution')

def generate_launch_description():
    node_ns, width, height, start_server, team_A, player_name = parse_arguments()
    package = 'jps_maze'
    if start_server:
        node_name = 'server_node'
        executable = 'jps_maze_server'
        os.environ['status_topic'] = '/status'
    else:
        node_name = 'client_node' + str(random.randrange(10000))
        executable = 'jps_maze_client'
        if team_A:
            team = 'team_A'
        else:
            team = 'team_B'
        os.environ['status_topic'] = '/' + node_ns +'/' +  team + '/status'
    os.environ['width'] = width
    os.environ['height'] = height
    os.environ['node_ns'] = node_ns
    os.environ['node_name'] = node_name
    os.environ['create_player_topic'] = '/' + node_ns + '/create_player'
    os.environ['move_player_topic'] = '/' + node_ns + '/move_player'
    os.environ['team_A'] = str(team_A)
    os.environ['player_name'] = player_name
    sys.argv = sys.argv[:1]
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
