import os
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from gym_gazebo_2.utils import launch_helpers

def generate_launch_description():
    """
        Returns ROS2 LaunchDescription object.
    """
    gzserver_only = False

    urdf = os.path.join(get_package_share_directory('mara_description'), 'urdf', 'mara_robot_camera_top.urdf')
    mara = get_package_share_directory('mara_gazebo_plugins')
    install_dir = get_package_prefix('mara_gazebo_plugins')
    ros2_ws_path = os.path.abspath(os.path.join(install_dir, os.pardir))
    MARA_model_path = os.path.join(ros2_ws_path, 'src', 'MARA')
    MARA_plugin_path = os.path.join(ros2_ws_path, 'src', 'MARA', 'mara_gazebo_plugins', 'build')
    world_path = os.path.join(get_package_share_directory('mara_gazebo'), 'worlds', 'empty_speed_up.world')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  (os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + 'share'
                                            + ':' + MARA_model_path)
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + MARA_model_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = (os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
                                            + ':' + MARA_plugin_path)
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + MARA_plugin_path

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if (key.isupper()):
                envs[key] = os.environ[key]
    except Exception as e:
        print("Error with Envs: " + str(e))
        return None

    # Gazebo visual interfaze. GUI/no GUI options.
    gazebo_cmd = "gazebo"
    if gzserver_only:
        gazebo_cmd = "gzserver"

    # Creation of ROS2 LaunchDescription obj.
    ld = LaunchDescription([
        ExecuteProcess(
            cmd=[gazebo_cmd,'--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world_path], output='screen',
            env=envs
        ),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='mara_utils_scripts', node_executable='spawn_entity.py', output='screen'),
        Node(package='hros_cognition_mara_components', node_executable='hros_cognition_mara_components', output='screen',
            arguments=["-motors", install_dir + "/share/hros_cognition_mara_components/link_order.yaml"])
    ])
    return ld