from gym_gazebo2.utils import ut_launch
from ros2pkg.api import get_prefix_path

def generate_launch_description():
    """
        Returns ROS2 LaunchDescription object.
    """
    gzclient = True
    real_speed = False
    multi_instance = False
    port = 11345

    return ut_launch.generate_launch_description_mara(gzclient, real_speed, multi_instance, port)
