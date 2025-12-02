from launch import LaunchDescription
from launch_ros.actions import Node

#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
#from ament_index_python.packages import get_package_prefix

from ament_index_python.packages import get_package_share_directory
import os

import yaml


def load_yaml(yaml_file):
    try:
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: File '{yaml_file}' not found.")
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file '{yaml_file}': {e}")
    except PermissionError:
        print(f"Error: No permission to read '{yaml_file}'.")
    except Exception as e:
        print(f"Unexpected error while loading '{yaml_file}': {e}")
    return None  # fallback if loading fails

def generate_launch_description():

    ld = LaunchDescription()

    # Declare the 'path' launch argument with a default value
    '''
    path_arg = DeclareLaunchArgument(
        'path',
        default_value=PathJoinSubstitution([
            get_package_prefix('cari_drv_septentrio'),
            'lib', 'cari_drv_septentrio', 'scripts'
        ]),
        description='Path to the septentrio_wrapper.sh script'
    )

    reset_arg = DeclareLaunchArgument(
        'reset_arg',
        default_value="---stophere",
        description='reset driver'
    )
    '''

    pkg_share = get_package_share_directory('cam_lidar_calibration_2')
    config_path = os.path.join(pkg_share, 'config.yaml')

    configuration = load_yaml(config_path)
    if(configuration is None):
        print("Loading default parameters")

        configuration =	{ 
            "camera": {
                "simulation_camera_topic": "/cam/image_color",
                "simulated_image_size": {
                    "height": 1000,
                    "width": 1400
                }

            },
            "lidar": {
                "simulation_lidar_topic": "/lidarsim/points"
            }
        } 

    simulator_bridge_node = Node(
        package="cam_lidar_calibration_2",
        name='simulator_bridge_subscriber',
        executable="simulator_bridge",
        #arguments=[LaunchConfiguration('reset_arg'), '---stophere'],
        parameters=[{
            'camera/simulation_camera_topic': configuration["camera"]["simulation_camera_topic"],
            'lidar/simulation_lidar_topic' : configuration["lidar"]["simulation_lidar_topic"],
            'camera/simulated_image_size/height' : configuration["camera"]["simulated_image_size"]["height"],
            'camera/simulated_image_size/width' : configuration["camera"]["simulated_image_size"]["width"]
        }],
        output='screen',
        respawn=False,
    )

    #ld.add_action(path_arg)
    #ld.add_action(reset_arg)
    ld.add_action(simulator_bridge_node)

    return ld