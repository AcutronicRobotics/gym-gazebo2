import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import os
import sys
import math
import transforms3d as tf3d
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_mara, ut_math, tree_urdf, general_utils
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

class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass

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
        self.real_speed = args.real_speed
        self.velocity = args.velocity
        # Set the path of the corresponding URDF file
        URDF_PATH = get_prefix_path("mara_description") + "/share/mara_description/urdf/mara_robot_gripper_140.urdf"

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.obs = None
        self.action_space = None
        self.target_position = None
        self.target_orientation = None
        self.max_episode_steps = 1024
        self.iterator = 0
        self.reset_jnts = True

        #############################
        #   Environment hyperparams
        #############################
        # Target, where should the agent reach
        self.target_position = np.asarray([-0.40028, 0.095615, 0.72466]) # close to the table
        self.target_orientation = np.asarray([0., 0.7071068, 0.7071068, 0.]) # arrow looking down [w, x, y, z]
        # self.target_position = np.asarray([-0.386752, -0.000756, 1.40557]) # easy point
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
        TABLE = 'table'
        BASE = 'base_link'
        BASE_ROBOT = 'base_robot'
        MARA_MOTOR1_LINK = 'motor1_link'
        MARA_MOTOR2_LINK = 'motor2_link'
        MARA_MOTOR3_LINK = 'motor3_link'
        MARA_MOTOR4_LINK = 'motor4_link'
        MARA_MOTOR5_LINK = 'motor5_link'
        MARA_MOTOR6_LINK = 'motor6_link'
        EE_LINK = 'ee_link'

        JOINT_ORDER = [MOTOR1_JOINT,MOTOR2_JOINT, MOTOR3_JOINT,
                        MOTOR4_JOINT, MOTOR5_JOINT, MOTOR6_JOINT]
        LINK_NAMES = [ WORLD, TABLE, BASE, BASE_ROBOT,
                        MARA_MOTOR1_LINK, MARA_MOTOR2_LINK,
                        MARA_MOTOR3_LINK, MARA_MOTOR4_LINK,
                        MARA_MOTOR5_LINK, MARA_MOTOR6_LINK, EE_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        m_joint_order = copy.deepcopy(JOINT_ORDER)
        m_link_names = copy.deepcopy(LINK_NAMES)

        # Initialize target end effector position
        self.environment = {
            'joint_order': m_joint_order,
            'link_names': m_link_names,
            'reset_conditions': reset_condition,
            'tree_path': URDF_PATH,
            'end_effector_points': EE_POINTS,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = tree_urdf.treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.mara_chain = self.ur_tree.getChain(self.environment['link_names'][0], self.environment['link_names'][-1])
        self.num_joints = self.mara_chain.getNrOfJoints()
        # Initialize a KDL Jacobian solver from the chain.
        self.jac_solver = ChainJntToJacSolver(self.mara_chain)

        self.obs_dim = self.num_joints + 10

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        low = -np.pi * np.ones(self.num_joints)
        high = np.pi * np.ones(self.num_joints)
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
        self._observation_msg =  message

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
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

        # Collect the end effector points and velocities in cartesian coordinates for the process_observations state.
        # Collect the present joint angles and velocities from ROS for the state.
        last_observations = ut_mara.process_observations(obs_message, self.environment)
        #Set observation to None after it has been read.
        self._observation_msg = None

        # Get Jacobians from present joint angles and KDL trees
        # The Jacobians consist of a 6x6 matrix getting its from from
        # (joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.get_jacobians(last_observations, self.num_joints, self.jac_solver)
        if self.environment['link_names'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            translation, rot = general_utils.forward_kinematics(self.mara_chain,
                                                self.environment['link_names'],
                                                last_observations[:self.num_joints],
                                                base_link=self.environment['link_names'][0], # make the table as the base to get the world coordinate system
                                                end_link=self.environment['link_names'][-1])

            current_quaternion = tf3d.quaternions.mat2quat(rot) #[w, x, y ,z]
            quat_error = tf3d.quaternions.qmult(current_quaternion, tf3d.quaternions.qconjugate(self.target_orientation))

            current_ee_pos_tgt = np.ndarray.flatten(general_utils.get_ee_points(self.environment['end_effector_points'], translation, rot).T)
            ee_pos_points = current_ee_pos_tgt - self.target_position

            ee_velocities = ut_mara.get_ee_points_velocities(ee_link_jacobians, self.environment['end_effector_points'], rot, last_observations)

            # Concatenate the information that defines the robot state
            # vector, typically denoted asrobot_id 'x'.
            state = np.r_[np.reshape(last_observations, -1),
                          np.reshape(ee_pos_points, -1),
                          np.reshape(quat_error, -1),
                          np.reshape(ee_velocities, -1),]

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
        self._pub.publish(ut_mara.get_trajectory_message(
            action[:self.num_joints],
            self.environment['joint_order'],
            self.velocity))

        # Take an observation
        self.ob = self.take_observation()

        # Fetch the positions of the end-effector which are nr_dof:nr_dof+3
        reward_dist = ut_math.rmse_func( self.ob[self.num_joints:(self.num_joints+3)] )
        reward_orientation = 2 * np.arccos( abs( self.ob[self.num_joints+3] ) )

        reward = ut_math.compute_reward(reward_dist, reward_orientation, False)

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)

        # Return the corresponding observations, rewards, etc.
        return self.ob, reward, done, {}

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

        # Take an observation
        self.ob = self.take_observation()

        # Return the corresponding observation
        return self.ob
