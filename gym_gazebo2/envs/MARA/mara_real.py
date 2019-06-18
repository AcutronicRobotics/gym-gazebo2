import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import os
import psutil
import signal
import sys
import math
import transforms3d as tf3d
from gym import utils, spaces
from gym_gazebo2.utils import ut_launch, ut_generic, ut_mara, ut_math, tree_urdf, general_utils
from gym.utils import seeding

# ROS 2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory # Used for publishing mara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Empty
from ros2pkg.api import get_prefix_path

# Algorithm specific
from PyKDL import ChainJntToJacSolver # For KDL Jacobians

class MARARealEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.realSpeed = args.realSpeed
        self.velocity = args.velocity

        # Set the path of the corresponding URDF file
        urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/reinforcement_learning/mara_robot_gripper_140_camera_run.urdf"

        # Launch mara in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess( ut_launch.launchReal() )

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True

        #############################
        #   Environment hyperparams
        #############################
        # Target, where should the agent reach
        self.targetPosition = np.asarray([-0.40028, 0.095615, 0.72466]) # close to the table
        self.target_orientation = np.asarray([0., 0.7071068, 0.7071068, 0.]) # arrow looking down [w, x, y, z]
        # self.targetPosition = np.asarray([-0.386752, -0.000756, 1.40557]) # easy point
        # self.target_orientation = np.asarray([-0.4958324, 0.5041332, 0.5041331, -0.4958324]) # arrow looking opposite to MARA [w, x, y, z]

        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])

        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])

        # # Topics for the robot publisher and subscriber.
        JOINT_PUBLISHER = '/mara_controller/command'
        JOINT_SUBSCRIBER = '/mara_controller/state'

        # joint names:
        MOTOR1_JOINT = 'motor1'
        MOTOR2_JOINT = 'motor2'
        MOTOR3_JOINT = 'motor3'
        MOTOR4_JOINT = 'motor4'
        MOTOR5_JOINT = 'motor5'
        MOTOR6_JOINT = 'motor6'
        EE_LINK = 'ee_link'

        # Set constants for links
        WORLD = 'world'
        BASE = 'base_robot'
        MARA_MOTOR1_LINK = 'motor1_link'
        MARA_MOTOR2_LINK = 'motor2_link'
        MARA_MOTOR3_LINK = 'motor3_link'
        MARA_MOTOR4_LINK = 'motor4_link'
        MARA_MOTOR5_LINK = 'motor5_link'
        MARA_MOTOR6_LINK = 'motor6_link'
        EE_LINK = 'ee_link'

        JOINT_ORDER = [MOTOR1_JOINT,MOTOR2_JOINT, MOTOR3_JOINT,
                        MOTOR4_JOINT, MOTOR5_JOINT, MOTOR6_JOINT]
        LINK_NAMES = [ WORLD, BASE,
                        MARA_MOTOR1_LINK, MARA_MOTOR2_LINK,
                        MARA_MOTOR3_LINK, MARA_MOTOR4_LINK,
                        MARA_MOTOR5_LINK, MARA_MOTOR6_LINK, EE_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
            'initial_velocities': []
        }
        #############################

        m_jointOrder = copy.deepcopy(JOINT_ORDER)
        m_linkNames = copy.deepcopy(LINK_NAMES)

        # Initialize target end effector position
        self.environment = {
            'jointOrder': m_jointOrder,
            'linkNames': m_linkNames,
            'reset_conditions': reset_condition,
            'tree_path': urdfPath,
            'end_effector_points': EE_POINTS,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = tree_urdf.treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.mara_chain = self.ur_tree.getChain(self.environment['linkNames'][0], self.environment['linkNames'][-1])
        self.numJoints = self.mara_chain.getNrOfJoints()
        # Initialize a KDL Jacobian solver from the chain.
        self.jacSolver = ChainJntToJacSolver(self.mara_chain)

        self.obs_dim = self.numJoints + 6

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        low = -np.pi * np.ones(self.numJoints)
        high = np.pi * np.ones(self.numJoints)
        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        # Seed the environment
        self.seed()

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        while obs_message is None:
            print("obs is empty")
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

        # Collect the end effector points and velocities in cartesian coordinates for the processObservations state.
        # Collect the present joint angles and velocities from ROS for the state.
        lastObservations = ut_mara.processObservations(obs_message, self.environment)
        #Set observation to None after it has been read.
        self._observation_msg = None

        # Get Jacobians from present joint angles and KDL trees
        # The Jacobians consist of a 6x6 matrix getting its from from
        # (joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.getJacobians(lastObservations, self.numJoints, self.jacSolver)
        if self.environment['linkNames'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            translation, rot = general_utils.forwardKinematics(self.mara_chain,
                                                self.environment['linkNames'],
                                                lastObservations[:self.numJoints],
                                                baseLink=self.environment['linkNames'][0], # use the base_robot coordinate system
                                                endLink=self.environment['linkNames'][-1])

            current_quaternion = tf3d.quaternions.mat2quat(rot) #[w, x, y ,z]
            quat_error = tf3d.quaternions.qmult(current_quaternion, tf3d.quaternions.qconjugate(self.target_orientation))

            current_eePos_tgt = np.ndarray.flatten(general_utils.getEePoints(self.environment['end_effector_points'], translation, rot).T)
            eePos_points = current_eePos_tgt - self.targetPosition

            eeVelocities = ut_mara.getEePointsVelocities(ee_link_jacobians, self.environment['end_effector_points'], rot, lastObservations)

            # Concatenate the information that defines the robot state
            # vector, typically denoted asrobot_id 'x'.
            state = np.r_[np.reshape(lastObservations, -1),
                          np.reshape(eePos_points, -1),
                          # np.reshape(quat_error, -1),
                          np.reshape(eeVelocities, -1),]

            return state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """
        self.iterator+=1

        # Execute "action"
        self._pub.publish(ut_mara.getTrajectoryMessage(
            action[:self.numJoints],
            self.environment['jointOrder'],
            self.velocity))

        # Take an observation
        obs = self.take_observation()

        # Fetch the positions of the end-effector which are nr_dof:nr_dof+3
        rewardDist = ut_math.rmseFunc( obs[self.numJoints:(self.numJoints+3)] )
        rewardOrientation = 2 * np.arccos( abs( obs[self.numJoints+3] ) )

        reward = ut_math.computeReward(rewardDist, rewardOrientation, False)

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)

        # Return the corresponding observations, rewards, etc.
        return obs, reward, done, {}

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        # Take an observation
        obs = self.take_observation()

        # Return the corresponding observation
        return obs

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()
