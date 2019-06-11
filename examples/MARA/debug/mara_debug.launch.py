from gym_gazebo2.utils import ut_launch
from ros2pkg.api import get_prefix_path

def generate_launch_description():
    """
        Returns ROS2 LaunchDescription object.
    """
    gzclient = True
    realSpeed = False
    multiInstance = False
    port = 11345
    urdf = "reinforcement_learning/mara_robot_train.urdf"
    urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf

    return ut_launch.generateLaunchDescriptionMara(gzclient, realSpeed, multiInstance, port, urdfPath)
