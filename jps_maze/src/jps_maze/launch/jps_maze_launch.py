import argparse, re, os, subprocess, random
from functools import partial
import logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package

logger = logging.getLogger(name='launch')

def start_daemon(node_name, package, target_port, browser_port, launch_context):
    subprocess.run(args=['forever', 'start', '-a', '--uid', node_name, 'app.js', str(target_port), str(browser_port)], cwd=os.path.join(get_package_share_directory(package_name=package), 'nodeenv'))
def stop_daemon(node_name, package, launch_constext):
    subprocess.run(args=['forever', 'stop', node_name,], cwd=os.path.join(get_package_share_directory(package_name=package), 'nodeenv'))
def parse_arguments():
    parser = argparse.ArgumentParser(prog='jps_maze')
    run_mode_group = parser.add_mutually_exclusive_group(required=True)
    run_mode_group.add_argument('-s', '--server', dest='start_server', action='store_true', help='Start the server node')
    run_mode_group.add_argument('-c', '--client', dest='start_server', action='store_false', help='Start the client node')
    parser.add_argument('-t', '--team', type=str, choices=['A', 'a', 'B', 'b'], dest='team', required=False, help='If client mode is chosen, select the team to play on')
    parser.add_argument('-n', '--name', type=str, dest='name', required=False, help='If client mode is chosen, select the player name')
    parser.add_argument('--ns', '--node_ns', type=str, dest='node_ns', required=False, default='jps_maze', help='Set the namespace to use')
    parser.add_argument('-b', '--board_no', type=int, choices=range(0,2), dest='board_no', required=False, default=0, help='Select the index for the board to use')
    parser.add_argument('-d', '--debug', dest='debug', action='store_true', required=False, default=False, help='Set the log-level to debug for more verbose logging')
    parser.add_argument('--ppt', '--player_per_team', type=int, dest='player_per_team', required=False, default=2, help='Specify the amount of players per team')
    parser.add_argument('--host_name', type=str, dest='host_name', required=False, default='localhost', help='Specify the hostname to connect to for visualisation')
    parser.add_argument('-tp', '--target_port', type=int, dest='target_port', required=False, default=random.randrange(49152, 65535), help='Specify the port the nodejs server awaits data from ros')
    parser.add_argument('-bp', '--browser_port', type=int, dest='browser_port', required=False, default=random.randrange(49152, 65535), help='Specify the port the nodejs server opens for browsers')
    parser.add_argument('--curses', dest='use_curses', action='store_true', required=False, default=False, help='Use curses instead of the nodejs visualizer')
    parser.print_usage()
    args = parser.parse_args(input('Enter options:\n').split())
    if not 0 < args.target_port < 65535:
        parser.error('Invalid port')
    if not args.start_server:
        if args.team:
            if args.name:
                return args.node_ns, args.start_server, True if args.team.upper() == 'A' else  False, args.name, args.board_no, args.debug, args.player_per_team, args.host_name, args.target_port, args.browser_port, args.use_curses
            else:
                parser.error('Missing player name')
        else:
            parser.error('Missing team selection')
    else:
        return args.node_ns, args.start_server, True, 'server', args.board_no, args.debug, args.player_per_team, args.host_name, args.target_port, args.browser_port, args.use_curses

def generate_launch_description():
    node_ns, start_server, team_A, player_name, board_no, debug, player_per_team, host_name, target_port, browser_port, use_curses = parse_arguments()
    logger.info('Parsed arguments')
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
    os.environ['next_round_topic'] = '/' + node_ns + '/next_round'
    os.environ['team_A'] = str(team_A)
    os.environ['player_name'] = player_name
    os.environ['board_path'] = get_share_file_path_from_package(package_name=package, file_name='board' + str(board_no) + '.csv')
    os.environ['player_per_team'] = str(player_per_team)
    os.environ['host_name'] = host_name
    os.environ['target_port'] = str(target_port)
    logger.info('Set up environment variables')
    if debug:
        logger.info('Enabling debug output')
        ros_arguments=['--log-level', 'DEBUG']
    else:
        ros_arguments=[]
    logger.info('Creating node description')
    node = Node(
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
    )
    logger.info('Creating start_handler description')
    start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                LogInfo(
                    msg=['Starting nodejs server daemon']
                ),
                OpaqueFunction(function=partial(start_daemon, node_name, package, target_port, browser_port)),
                LogInfo(
                    msg=['Finished starting nodejs server daemon']
                ),
            ]
        )
    )
    logger.info('Creating stop_handler description')
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                LogInfo(
                 msg=['Stopping nodejs server daemon']
                ),
                OpaqueFunction(function=partial(stop_daemon, node_name, package)),
                LogInfo(
                    msg=['Finished stopping nodejs daemon']
                ),
            ]
        )
    )
    logger.info('Creating curses_starter description')
    curses_starter =  RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start = [
                LogInfo(
                    msg=['Starting curses app']
                ),
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--hide-menubar',
                        '--geometry=66x66 '
                        '-t',
                        node_name + ('' if start_server else ('_team_A' if team_A else '_team_B')),
                        '--zoom=0.75',
                        '--',
                        './jps_maze_curses',
                        str(target_port),
                    ],
                    shell=True,
                    cwd=get_package_share_directory(package_name=package),
                    log_cmd=True,
                ),
            ],
        )
    )
    logger.info('Creating Launch description')
    if host_name == 'localhost' or host_name == '127.0.0.1':
        if use_curses:
            logger.info('Using curses app instead of nodejs')
            launch_description = LaunchDescription(
                [
                    node,
                    curses_starter,
                ]
            )
        else:
            logger.info('Nodejs should run on localhost, so starting automatically')
            launch_description = LaunchDescription(
                [
                    node,
                    start_handler,
                    shutdown_handler,
                ]
            )
    else:
        logger.info('Nodejs should run on different device')
        launch_description = LaunchDescription(
            [
                node,
            ]
        )
    logger.info('Open browser on {}:{}'.format(host_name, browser_port))
    return launch_description
