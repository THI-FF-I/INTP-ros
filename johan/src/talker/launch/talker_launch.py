import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from ros2launch.api import get_share_file_path_from_package

node_ns = "INTP_ROS"
node_name = "talker_node"

os.environ["node_ns"] = node_ns
os.environ["node_name"] = node_name
os.environ["target_topic"] = "/"+ node_ns + "/"+ node_name + "/info"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker',
            namespace=node_ns,
            executable='talker',
            name=node_name,
            parameters=[
                launch_ros.parameter_descriptions.ParameterFile(
                    param_file=get_share_file_path_from_package(package_name= 'talker', file_name='parameters.yaml'),
                    allow_substs=True)
            ],
        )])