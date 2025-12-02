from launch import LaunchDescription
from launch_ros.actions import Node
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


    pkg_share = get_package_share_directory('cam_lidar_calibration_2')
    config_path = os.path.join(pkg_share, 'config.yaml')

    configuration = load_yaml(config_path)
    if(configuration is None):
        print("Loading default parameters")

        configuration =	{ 
            "camera": {
                "camera_topic": "/cam/image_color",
                "image_size": {
                    "height": 1000,
                    "width": 1400
                }

            },
            "lidar": {
                "lidar_topic": "/lidarsim/points",
                "maximum_expected_points_number": 60000,
                "lidar_coord_system:": {
                    "right": [0.0,-1.0,0.0],
                    "up": [0.0,0.0,1.0],
                    "front": [1.0,0.0,0.0]
                }
            }
        } 

    calibrator_bridge_node = Node(
        package="cam_lidar_calibration_2",
        name='calib_publisher',
        executable="calibrator_bridge",
        #arguments=[LaunchConfiguration('reset_arg'), '---stophere'],
        parameters=[{
            'camera/camera_topic': configuration["camera"]["camera_topic"],
            'lidar/lidar_topic' : configuration["lidar"]["lidar_topic"],
            'camera/image_size/height' : configuration["camera"]["image_size"]["height"],
            'camera/image_size/width' : configuration["camera"]["image_size"]["width"],
            'lidar/maximum_expected_points_number' : configuration["lidar"]["maximum_expected_points_number"],
            'lidar/lidar_coord_system/right' : configuration["lidar"]["lidar_coord_system"]["right"],
            'lidar/lidar_coord_system/up' : configuration["lidar"]["lidar_coord_system"]["up"],
            'lidar/lidar_coord_system/front' : configuration["lidar"]["lidar_coord_system"]["front"]
        }],
        output='screen',
        respawn=False,
    )

    #ld.add_action(path_arg)
    #ld.add_action(reset_arg)
    ld.add_action(calibrator_bridge_node)

    return ld