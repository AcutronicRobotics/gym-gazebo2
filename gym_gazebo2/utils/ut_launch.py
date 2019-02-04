import socket
import random
import os
import gym_gazebo2
import pathlib
from datetime import datetime
from billiard import Process
from gym_gazebo2.utils import ut_generic

from launch import LaunchService, LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def start_launch_servide_process(ld):
    """Starts a Launch Service process. To be called from subclasses.

    Args:
         ld : LaunchDescription obj.
    """
    # Create the LauchService and feed the LaunchDescription obj. to it.
    ls = LaunchService()
    ls.include_launch_description(ld)
    p = Process(target=ls.run)
    p.start()

def is_port_in_use(port):
    """Checks if the given port is being used.

    Args:
        port(int): Port number.

    Returns:
        bool: True if the port is being used, False otherwise.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0

def get_exclusive_network_parameters():
    """Creates appropriate values for ROS_DOMAIN_ID and GAZEBO_MASTER_URI.

    Returns:
        Dictionary {ros_domain_id (string), ros_domain_id (string)}
    """

    random_port = random.randint(10000, 15000)
    while is_port_in_use(random_port):
        print("Randomly selected port is already in use, retrying.")
        random_port = random.randint(10000, 15000)

    # Save network segmentation related information in a temporary folder.
    temp_path = '/tmp/gym-gazebo-2/running/'
    pathlib.Path(temp_path).mkdir(parents=True, exist_ok=True)

    # Remove old tmp files.
    ut_generic.clean_old_files(temp_path, ".log", 2)

    filename = datetime.now().strftime('running_since_%H_%M__%d_%m_%Y.log')

    f = open(temp_path + '/' + filename, 'w+')
    f.write(filename + '\nROS_DOMAIN_ID='+str(random_port)+'\nGAZEBO_MASTER_URI=http://localhost:' + str(random_port))
    f.close()

    return {'ros_domain_id':str(random_port),
     'gazebo_master_uri':"http://localhost:" + str(random_port)}

def generate_launch_description_mara(gzclient, real_speed, multi_instance, port):
    """
        Returns ROS2 LaunchDescription object.
        Args:
            real_speed: bool   True if RTF must be set to 1, False if RTF must be set to maximum.
    """
    urdf = os.path.join(get_package_share_directory('mara_description'), 'urdf', 'mara_robot_camera_top.urdf')
    mara = get_package_share_directory('mara_gazebo_plugins')
    install_dir = get_package_prefix('mara_gazebo_plugins')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    if not real_speed:
        world_path = os.path.join(os.path.dirname(gym_gazebo2.__file__), 'worlds', 'empty__state_plugin__speed_up.world')
    else:
        world_path = os.path.join(os.path.dirname(gym_gazebo2.__file__), 'worlds', 'empty__state_plugin.world')

    if port != 11345:  # Default gazebo port
        os.environ["ROS_DOMAIN_ID"] = str(port)
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(port)
        print("******* Manual network segmentation *******")
        print("ROS_DOMAIN_ID=" + os.environ['ROS_DOMAIN_ID'])
        print("GAZEBO_MASTER_URI=" + os.environ['GAZEBO_MASTER_URI'])
        print("")
    elif multi_instance:
        # Exclusive network segmentation, which allows to launch multiple instances of ROS2+Gazebo
        network_params = get_exclusive_network_parameters()
        os.environ["ROS_DOMAIN_ID"] = network_params.get('ros_domain_id')
        os.environ["GAZEBO_MASTER_URI"] = network_params.get('gazebo_master_uri')
        print("******* Exclusive network segmentation *******")
        print("ROS_DOMAIN_ID=" + network_params.get('ros_domain_id'))
        print("GAZEBO_MASTER_URI=" + network_params.get('gazebo_master_uri'))
        print("")

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
    if gzclient:
        gazebo_cmd = "gazebo"
    else:
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
            arguments=["-motors", install_dir + "/share/hros_cognition_mara_components/link_order.yaml"]),
        Node(package='mara_contact_publisher', node_executable='mara_contact_publisher', output='screen', arguments=[urdf])
    ])
    return ld
