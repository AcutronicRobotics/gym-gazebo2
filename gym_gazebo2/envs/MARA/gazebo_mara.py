import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import threading # Used for time locks to synchronize position data.
import os
from gym import utils, spaces
from gym_gazebo2.utils import ut_gazebo, ut_generic, ut_launch, ut_mara, ut_math
from gym.utils import seeding
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnEntity
from multiprocessing import Process

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration

# ROS 2 Launch related
import os
import sys
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from launch_ros import get_default_launch_description
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# Algorithm specific
from baselines.agent.scara_arm.tree_urdf import treeFromFile # For KDL Jacobians
from PyKDL import Jacobian, Chain, ChainJntToJacSolver, JntArray # For KDL Jacobians

# from custom baselines repository
from baselines.agent.utility.general_utils import forward_kinematics, get_ee_points, rotation_from_matrix, \
    get_rotation_matrix,quaternion_from_matrix # For getting points and velocities.

class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass

class GazeboMARAEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsMARA()
        self.gzclient = args.gzclient
        self.real_speed = args.real_speed
        self.velocity = args.velocity
        self.multi_instance = args.multi_instance

        # Launch mara in a new Process
        ut_launch.start_launch_servide_process(ut_launch.generate_launch_description_mara(self.gzclient, self.real_speed, self.multi_instance))
        # Wait a bit for the spawn process.
        # TODO, replace sleep function.
        time.sleep(5)

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # TODO: cleanup this variables, remove the ones that aren't used
        # class variables
        self._observation_msg = None
        self.scale = None  # must be set from elsewhere based on observations
        self.bias = None
        self.x_idx = None
        self.obs = None
        self.reward = None
        self.done = None
        self.reward_dist = None
        self.reward_ctrl = None
        self.action_space = None
        self.max_episode_steps = 1000 # now used in all algorithms
        self.iterator = 0
        # default to seconds
        self.slowness = 1000000
        self.slowness_unit = 'nsec'
        self.reset_jnts = True

        self._time_lock = threading.RLock()

        #############################
        #   Environment hyperparams
        #############################
        # target, where should the agent reach
        EE_POS_TGT = np.asmatrix([-0.5087683179567231 - 0.14134294, 0.18728049, (2.03314576-1.4808068867058566)*1.3333])
        EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
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

        URDF_PATH = get_prefix_path("mara_description") + "/share/mara_description/urdf/mara_robot_camera_top.urdf"
        #URDF_PATH = get_prefix_path("mara_description") + "/urdf/mara_demo_camera_top.urdf"

        m_joint_order = copy.deepcopy(JOINT_ORDER)
        m_link_names = copy.deepcopy(LINK_NAMES)
        m_joint_publishers = copy.deepcopy(JOINT_PUBLISHER)
        m_joint_subscribers = copy.deepcopy(JOINT_SUBSCRIBER)
        ee_pos_tgt = EE_POS_TGT
        ee_rot_tgt = EE_ROT_TGT

        # Initialize target end effector position
        ee_tgt = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T)
        self.realgoal = ee_tgt
        self.target_orientation = ee_rot_tgt

        self.environment = {
            # rk changed this to for the mlsh
            # 'ee_points_tgt': ee_tgt,
            'ee_points_tgt': self.realgoal,
            'ee_point_tgt_orient': self.target_orientation,
            'joint_order': m_joint_order,
            'link_names': m_link_names,
            # 'slowness': slowness,
            'reset_conditions': reset_condition,
            'tree_path': URDF_PATH,
            'joint_publisher': m_joint_publishers,
            'joint_subscriber': m_joint_subscribers,
            'end_effector_points': EE_POINTS,
            'end_effector_velocities': EE_VELOCITIES,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory,
                                               JOINT_PUBLISHER,
                                               qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState,
                                                  JOINT_SUBSCRIBER,
                                                  self.observation_callback,
                                                  qos_profile=qos_profile_sensor_data)

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.scara_chain = self.ur_tree.getChain(self.environment['link_names'][0], self.environment['link_names'][-1])
        # Initialize a KDL Jacobian solver from the chain.
        self.jac_solver = ChainJntToJacSolver(self.scara_chain)
        self._observations_stale = [False for _ in range(1)]
        self._currently_resetting = [False for _ in range(1)]
        self.reset_joint_angles = [None for _ in range(1)]
        self.obs_dim = self.scara_chain.getNrOfJoints() + 6#10 # hardcode it for now

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        low = -np.pi/2.0 * np.ones(self.scara_chain.getNrOfJoints())
        high = np.pi/2.0 * np.ones(self.scara_chain.getNrOfJoints())

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
        pose.position.x = EE_POS_TGT[0,0];
        pose.position.y = EE_POS_TGT[0,1];
        pose.position.z = EE_POS_TGT[0,2];
        pose.orientation.x = 0.0;
        pose.orientation.y= 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        
        #override previous spawn_request element.
        self.spawn_request = SpawnEntity.Request()
        self.spawn_request.name = "target"
        self.spawn_request.xml = model_xml
        self.spawn_request.robot_namespace = ""
        self.spawn_request.initial_pose = pose
        self.spawn_request.reference_frame = ""

        #ROS2 Spawn Entity
        future = spawn_cli.call_async(self.spawn_request)

        # Seed the environment
        self.seed()


    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg =  message

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
        # Collect the end effector points and velocities in
        # cartesian coordinates for the process_observationsstate.
        # Collect the present joint angles and velocities from ROS for the state.
        last_observations = ut_mara.process_observations(obs_message, self.environment)
        # # # Get Jacobians from present joint angles and KDL trees
        # # # The Jacobians consist of a 6x6 matrix getting its from from
        # # # (# joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.get_jacobians(last_observations, self.scara_chain.getNrOfJoints(), self.jac_solver)
        if self.environment['link_names'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            # print(self.environment['link_names'][-1])
            trans, rot = forward_kinematics(self.scara_chain,
                                        self.environment['link_names'],
                                        last_observations[:self.scara_chain.getNrOfJoints()],
                                        base_link=self.environment['link_names'][0],
                                        end_link=self.environment['link_names'][-1])

            rotation_matrix = np.eye(4)
            rotation_matrix[:3, :3] = rot
            rotation_matrix[:3, 3] = trans

            # I need this calculations for the new reward function, need to send them back to the run mara or calculate them here
            current_quaternion = quaternion_from_matrix(rotation_matrix)
            tgt_quartenion = quaternion_from_matrix(self.target_orientation)

            A  = np.vstack([current_quaternion, np.ones(len(current_quaternion))]).T

            quat_error = current_quaternion - tgt_quartenion

            current_ee_tgt = np.ndarray.flatten(get_ee_points(self.environment['end_effector_points'],
                                                              trans,
                                                              rot).T)
            ee_points = current_ee_tgt - self.realgoal#self.environment['ee_points_tgt']
            ee_points_jac_trans, _ = ut_mara.get_ee_points_jacobians(ee_link_jacobians,
                                                                   self.environment['end_effector_points'],
                                                                   rot,
                                                                   self.scara_chain.getNrOfJoints())
            ee_velocities = ut_mara.get_ee_points_velocities(ee_link_jacobians,
                                                           self.environment['end_effector_points'],
                                                           rot,
                                                           last_observations)

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

    def wait_for_action(self, action):
        """Receives an action and loops until the robot reaches the pose set by the action.
        
        Note: This function can't be migrated to the ut_mara module since it reads constantly 
        from the observation callback provided by /mara_controller/state.
        """
        action_finished = False
        resetting = False
        trials = 0
        while not action_finished:
            trials += 1
            if trials > 200 and not resetting: #action failed, probably hitting the table.
                print("Can't complete trajectory, setting new trajectory: initial_positions")
                resetting = True
            if resetting:

                # Reset simulation
                reset_cli = self.node.create_client(Empty, '/reset_simulation')
                while not reset_cli.wait_for_service(timeout_sec=1.0):
                    self.node.get_logger().info('service not available, waiting again...')

                reset_future = reset_cli.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self.node, reset_future)

                # Avoid unnecessary pose check.
                break

            rclpy.spin_once(self.node)
            obs_message = self._observation_msg
            if obs_message is not None:
                last_observation = ut_mara.process_observations(obs_message, self.environment)
                action_finished = ut_mara.positions_match(action, last_observation)

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
            action[:self.scara_chain.getNrOfJoints()],
            self.environment['joint_order'],
            self.velocity))

        # Wait until the action is finished.
        self.wait_for_action(action)

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            print("step: observation is Empty")
            self.ob = self.take_observation()

        """NOTE: 
        If MARA is stuck and can't complete the trajectory, the final pose of the trajectory will be the initial position.
        Which means, failing trajectories will receive the same reward as the inital_position pose.
        If the TARGET is UP (close to the inital pose) the MARA could learn to get stuck on purpose and fall into a local minimum.
        A simple solution would be to use a CRASH variable to reduce the reward, and not only be pose-dependant."""
        self.reward_dist = -ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])
        self.reward_orient = - ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)])

        # here we want to fetch the positions of the end-effector which are nr_dof:nr_dof+3
        if(ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])<0.005):
            self.reward = 1 - ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)]) # Make the reward increase as the distance decreases
            print("Reward is: ", self.reward)
        else:
            self.reward = self.reward_dist

        # Calculate if the env has been solved
        done = bool(abs(self.reward_dist) < 0.005) or (self.iterator>self.max_episode_steps)

        # Return the corresponding observations, rewards, etc.
        return self.ob, self.reward, done, {}

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

            # Wait until the action is finished.
            self.wait_for_action(self.environment['reset_conditions']['initial_positions'])

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        # Return the corresponding observation
        return self.ob
