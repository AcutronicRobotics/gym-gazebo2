import socket
import random
from multiprocessing import Process

from launch import LaunchService, LaunchDescription
from launch.actions.execute_process import ExecuteProcess


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
    return {'ros_domain_id':str(random_port),
     'gazebo_master_uri':"http://localhost:" + str(random_port)}
