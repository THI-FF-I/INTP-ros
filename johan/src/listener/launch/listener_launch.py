import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package

node_ns = "INTP_ROS"
node_name = "listener_node"
turtle_name = "_turtlesim_node"

os.environ["node_ns"] = node_ns
os.environ["node_name"] = node_name
os.environ["target_topic"] = "/"+ node_ns + "/talker_node/info"
os.environ["turtle_name"] = turtle_name

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listener',
            namespace=node_ns,
            executable='listener',
            name=node_name,
            parameters=[
                launch_ros.parameter_descriptions.ParameterFile(
                    param_file=get_share_file_path_from_package(package_name= 'listener', file_name='parameters.yaml'),
                    allow_substs=True)
                ]
            ),
        Node(package='turtlesim',
             namespace=node_ns +'/_private',
             executable='turtlesim_node',
             name=turtle_name,
             ros_arguments=[
                 '--log-level',
                 'WARN'
             ],
             remappings=[
                 ('/' + node_ns + '/_private/turtle1/teleport_absolute', turtle_name + '/teleport_absolute'),
             ],
             ),
    ])