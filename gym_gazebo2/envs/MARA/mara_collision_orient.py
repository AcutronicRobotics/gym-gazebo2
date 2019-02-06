import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import os
import sys
import math
import transforms3d as tf
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_launch, ut_mara, ut_math
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
from multiprocessing import Process
import argparse

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactState
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration

# Algorithm specific
from baselines.agent.scara_arm.tree_urdf import treeFromFile # For KDL Jacobians
from PyKDL import ChainJntToJacSolver # For KDL Jacobians

# from custom baselines repository
from baselines.agent.utility.general_utils import forward_kinematics, get_ee_points

class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass

class MARACollisionOrientEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.real_speed = args.real_speed
        self.velocity = args.velocity
        self.multi_instance = args.multi_instance
        self.port = args.port

        # Launch mara in a new Process
        ut_launch.start_launch_servide_process(
            ut_launch.generate_launch_description_mara(
                self.gzclient, self.real_speed, self.multi_instance, self.port))

        # Wait a bit for the spawn process.
        # TODO, replace sleep function.
        time.sleep(5)

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.obs = None
        self.action_space = None
        self.realgoal = None
        self.max_episode_steps = 1024 # now used in all algorithms
        self.iterator = 0
        self.reset_jnts = True
        self._collision_msg = None

        #############################
        #   Environment hyperparams
        #############################
        # Target, where should the agent reach
        EE_POS_TGT = np.asmatrix([-0.40028, 0.095615, 0.72466])
        EE_ROT_TGT = np.asmatrix([
                                [0.79660969, -0.51571238,  0.31536287],
                                [0.51531424,  0.85207952,  0.09171542],
                                [-0.31601302,  0.08944959,  0.94452874] ])

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

        # Set constants for links
        TABLE = 'table'
        BASE = 'base_link'
        MARA_MOTOR1_LINK = 'motor1_link'
        MARA_MOTOR2_LINK = 'motor2_link'
        MARA_MOTOR3_LINK = 'motor3_link'
        MARA_MOTOR4_LINK = 'motor4_link'
        MARA_MOTOR5_LINK = 'motor5_link'
        MARA_MOTOR6_LINK = 'motor6_link'
        EE_LINK = 'ee_link'

        # EE_LINK = 'ee_link'
        JOINT_ORDER = [MOTOR1_JOINT, MOTOR2_JOINT, MOTOR3_JOINT,
                       MOTOR4_JOINT, MOTOR5_JOINT, MOTOR6_JOINT]
        LINK_NAMES = [TABLE, BASE, MARA_MOTOR1_LINK, MARA_MOTOR2_LINK,
                            MARA_MOTOR3_LINK, MARA_MOTOR4_LINK,
                            MARA_MOTOR5_LINK, MARA_MOTOR6_LINK,
                      EE_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        # Set the path of the corresponding URDF file from "assets"
        URDF_PATH = get_prefix_path("mara_description") + "/share/mara_description/urdf/mara_robot_gripper_140.urdf"

        m_joint_order = copy.deepcopy(JOINT_ORDER)
        m_link_names = copy.deepcopy(LINK_NAMES)
        ee_pos_tgt = EE_POS_TGT
        ee_rot_tgt = EE_ROT_TGT

        # Initialize target end effector position
        self.realgoal = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T)
        self.target_orientation = tf.quaternions.mat2quat(ee_rot_tgt) #[w, x, y, z]

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
        self._sub_coll = self.node.create_subscription(ContactState, '/gazebo_contacts', self.collision_callback, qos_profile=qos_profile_sensor_data)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.scara_chain = self.ur_tree.getChain(self.environment['link_names'][0], self.environment['link_names'][-1])
        self.num_joints = self.scara_chain.getNrOfJoints()
        # Initialize a KDL Jacobian solver from the chain.
        self.jac_solver = ChainJntToJacSolver(self.scara_chain)

        self.obs_dim = self.num_joints + 10

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        low = -np.pi/3.0 * np.ones(self.num_joints)
        high = np.pi/3.0 * np.ones(self.num_joints)
        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        # Spawn Target element in gazebo.
        # node & spawn_cli initialized in super class
        spawn_cli = self.node.create_client(SpawnEntity, '/spawn_entity')

        while not spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        model_xml = "<?xml version=\"1.0\"?> \
                    <robot name=\"myfirst\"> \
                      <link name=\"world\"> \
                      </link>\
                      <link name=\"cylinder0\">\
                        <visual>\
                          <geometry>\
                            <sphere radius=\"0.01\"/>\
                          </geometry>\
                          <origin xyz=\"0 0 0\"/>\
                          <material name=\"rojotransparente\">\
                              <ambient>0.5 0.5 1.0 0.1</ambient>\
                              <diffuse>0.5 0.5 1.0 0.1</diffuse>\
                          </material>\
                        </visual>\
                        <inertial>\
                          <mass value=\"5.0\"/>\
                          <inertia ixx=\"1.0\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1.0\" iyz=\"0.0\" izz=\"1.0\"/>\
                        </inertial>\
                      </link>\
                      <joint name=\"world_to_base\" type=\"fixed\"> \
                        <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\
                        <parent link=\"world\"/>\
                        <child link=\"cylinder0\"/>\
                      </joint>\
                      <gazebo reference=\"cylinder0\">\
                        <material>Gazebo/GreenTransparent</material>\
                      </gazebo>\
                    </robot>"

        pose = Pose()
        pose.position.x = self.realgoal[0]
        pose.position.y = self.realgoal[1]
        pose.position.z = self.realgoal[2]
        pose.orientation.x = self.target_orientation[1]
        pose.orientation.y= self.target_orientation[2]
        pose.orientation.z = self.target_orientation[3]
        pose.orientation.w = self.target_orientation[0]

        #override previous spawn_request element.
        self.spawn_request = SpawnEntity.Request()
        self.spawn_request.name = "target"
        self.spawn_request.xml = model_xml
        self.spawn_request.robot_namespace = ""
        self.spawn_request.initial_pose = pose
        self.spawn_request.reference_frame = "world"

        #ROS2 Spawn Entity
        target_future = spawn_cli.call_async(self.spawn_request)
        rclpy.spin_until_future_complete(self.node, target_future)

        # Seed the environment
        self.seed()

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg =  message

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        if message.collision1_name != message.collision2_name:
                self._collision_msg = message

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg
        if obs_message is None:
            print("Last observation is empty")
            return None
        # Collect the end effector points and velocities in cartesian coordinates for the process_observations state.
        # Collect the present joint angles and velocities from ROS for the state.
        last_observations = ut_mara.process_observations(obs_message, self.environment)
        # Get Jacobians from present joint angles and KDL trees
        # The Jacobians consist of a 6x6 matrix getting its from from
        # (joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.get_jacobians(last_observations, self.num_joints, self.jac_solver)
        if self.environment['link_names'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            translation, rot = forward_kinematics(self.scara_chain,
                                                self.environment['link_names'],
                                                last_observations[:self.num_joints],
                                                base_link=self.environment['link_names'][0],
                                                end_link=self.environment['link_names'][-1])

            current_quaternion = tf.quaternions.mat2quat(rot) #[w, x, y ,z]

            quat_error = ut_math.quaternion_product(current_quaternion, tf.quaternions.qconjugate(self.target_orientation))

            current_ee_tgt = np.ndarray.flatten(get_ee_points(self.environment['end_effector_points'], translation, rot).T)
            ee_points = current_ee_tgt - self.realgoal
            ee_velocities = ut_mara.get_ee_points_velocities(ee_link_jacobians, self.environment['end_effector_points'], rot, last_observations)

            # Concatenate the information that defines the robot state
            # vector, typically denoted asrobot_id 'x'.
            state = np.r_[np.reshape(last_observations, -1),
                          np.reshape(ee_points, -1),
                          np.reshape(quat_error, -1),
                          np.reshape(ee_velocities, -1),]

            return state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def collision(self):
        # Reset if there is a collision
        if self._collision_msg is not None:
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            self._collision_msg = None
            return True
        else:
            return False

    def original_reward_function(self):
        if self.collision():
            reward = - self.reward_dist * 10
            print("Reward (collided) is: ", reward)
        else:
            if self.reward_dist < 0.005:
                reward = 1 + self.reward_dist # Make the reward increase as the distance decreases
                # Include orient reward if and only if it is close enough to the target
                orientation_scale = 0.1
                # Fetch the orientation of the end-effector which are from nr_dof:nr_dof+3 to nr_dof:nr_dof+6
                reward_orient = - orientation_scale * self.reward_orientation
                #scale here the orientation because it should not be the main bias of the reward, position should be

                if reward_orient < 0.005:
                    reward = reward + reward_orient * 10
                    print("Reward is: ", reward)
                else:
                    reward = reward - reward_orient
                    print("Reward (bad orient) is: ", reward)
            else:
                reward = - self.reward_dist
        pass

    def new_reward_function(self):
        alpha = 3
        beta = 3
        gamma = 0.8
        #delta = 20

        distance_reward = (math.exp(-alpha*self.reward_dist)-math.exp(-alpha))/(1-math.exp(-alpha))

        orientation_reward = ((1-math.exp(-beta*abs((self.reward_orientation-math.pi)/math.pi))+gamma)/(1+gamma))

        if self.collision():
            collision_reward = 0
            #print("Collision")
            #collision_reward = delta*(1-math.exp(-self.reward_dist))
        else:
            collision_reward = 0

        if self.reward_dist < 0.05 and self.reward_orientation <0.05:
            done_reward = 10
        else:
            done_reward = 0

        return 2*distance_reward*orientation_reward-2-collision_reward + done_reward

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
        while(self.ob is None):
            print("step: observation is Empty")
            self.ob = self.take_observation()

        # Fetch the positions of the end-effector which are nr_dof:nr_dof+3
        self.reward_dist = ut_math.rmse_func(self.ob[self.num_joints:(self.num_joints+3)])
        self.reward_orientation = 2 * np.arccos(abs(self.ob[self.num_joints+3]))
        #scale here the orientation because it should not be the main bias of the reward, position should be

        #reward = self.original_reward_function()
        reward = self.new_reward_function()

        # Calculate if the env has been solved
        done = bool(self.iterator > self.max_episode_steps)
        #done = False
        # Return the corresponding observations, rewards, etc.
        return self.ob, reward, done, {}

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            # Move to the initial position.
            self._pub.publish(ut_mara.get_trajectory_message(
                self.environment['reset_conditions']['initial_positions'],
                self.environment['joint_order'],
                self.velocity))
            time.sleep(self.velocity)

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        # Return the corresponding observation
        return self.ob
