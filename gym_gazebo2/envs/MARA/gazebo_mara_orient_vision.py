"""
MIGRATION TO ROS2 IN PROCESS.

Additional Env specific dependencies:


"""
import gym
gym.logger.set_level(40) # hide warnings
import time
import os
import random
import numpy as np
from gym import utils, spaces
from gym.utils import seeding
from gym_gazebo2.utils import ut_gazebo, ut_generic, ut_launch, ut_mara, ut_math
import copy
import threading # Used for time locks to synchronize position data.

from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import ModelState, LinkState
from gazebo_msgs.msg import EntityState

from std_srvs.srv import Empty
from std_msgs.msg import String, Empty as stdEmpty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose

# ROS 2
import rclpy
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from ros2pkg.api import get_prefix_path
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from baselines.agent.scara_arm.tree_urdf import treeFromFile # For KDL Jacobians
from PyKDL import Jacobian, Chain, ChainJntToJacSolver, JntArray # For KDL Jacobians

import transforms3d as tf

# from custom baselines repository
from baselines.agent.utility.general_utils import forward_kinematics, get_ee_points, rotation_from_matrix, \
    get_rotation_matrix,quaternion_from_matrix # For getting points and velocities.

class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass


class GazeboMARAOrientVisionEnv(gym.Env):
    """
    TODO, description.
    """
    def __init__(self):
        """
        Initialize the SCARA environemnt
            NOTE: This environment uses ROS and interfaces.

            TODO: port everything to ROS 2 natively
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
        self.reset_jnts = True
        self.detect_target_once = 1

        self._time_lock = threading.RLock()

        #############################
        #   Environment hyperparams
        #############################
        # target, where should the agent reach
        EE_POS_TGT = np.asmatrix([-0.53170885, -0.02076771,  0.74240961]) # 200 cm from the z axis some random target at the begining
        EE_ROT_TGT = np.asmatrix([[-0.99500678,  0.09835458, -0.01696725],
                                  [-0.09951792, -0.99061751,  0.09366505],
                                  [-0.00759566,  0.0948859,   0.99545918]])
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])
        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])

        # Topics for the robot publisher and subscriber.
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

        # TODO: fix this and make it relative
        # Set the path of the corresponding URDF file from "assets"
        URDF_PATH = get_prefix_path("mara_description") + "/share/mara_description/urdf/mara_robot_camera_top.urdf"

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
            'reset_conditions': reset_condition,
            'tree_path': URDF_PATH,
            'joint_publisher': m_joint_publishers,
            'joint_subscriber': m_joint_subscribers,
            'end_effector_points': EE_POINTS,
            'end_effector_velocities': EE_VELOCITIES,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        # ROS 2 implementation
        self._pub = self.node.create_publisher(JointTrajectory,
                                                JOINT_PUBLISHER,
                                                qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState,
                                                JOINT_SUBSCRIBER,
                                                self.observation_callback,
                                                qos_profile=qos_profile_sensor_data)
        TARGET_SUBSCRIBER = '/mara/target'
        self._sub_tgt = self.node.create_subscription(Pose,
                                                TARGET_SUBSCRIBER,
                                                self.tgt_callback)

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.scara_chain = self.ur_tree.getChain(self.environment['link_names'][0], self.environment['link_names'][-1])
        # print("nr of jnts: ", self.scara_chain.getNrOfJoints())
        # Initialize a KDL Jacobian solver from the chain.
        self.jac_solver = ChainJntToJacSolver(self.scara_chain)
        #print(self.jac_solver)
        self._observations_stale = [False for _ in range(1)]
        #print("after observations stale")
        self._currently_resetting = [False for _ in range(1)]
        self.reset_joint_angles = [None for _ in range(1)]

        # TODO review with Risto, we might need the first observation for calling step()
        # observation = self.take_observation()
        # assert not done
        # self.obs_dim = observation.size
        """
        obs_dim is defined as:
        num_dof + end_effector_points=3 + end_effector_velocities=3
        end_effector_points and end_effector_velocities is constant and equals 3
        recently also added quaternion to the obs, which has dimension=4
        """
        #
        self.obs_dim = self.scara_chain.getNrOfJoints() + 10 #6 hardcode it for now
        # # print(observation, _reward)

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        # #bounds = self.model.actuator_ctrlrange.copy()
        low = -np.pi/2.0 * np.ones(self.scara_chain.getNrOfJoints())
        high = np.pi/2.0 * np.ones(self.scara_chain.getNrOfJoints())
        # low = -np.pi * np.ones(self.scara_chain.getNrOfJoints())
        # high = np.pi * np.ones(self.scara_chain.getNrOfJoints())
        # low = -np.inf * np.ones(self.scara_chain.getNrOfJoints())
        # high = np.inf * np.ones(self.scara_chain.getNrOfJoints())
        # print("Action spaces: ", low, high)
        self.action_space = spaces.Box(low, high)
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)


        # Migration to ROS2

        #self.add_model_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        #self.add_model_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.add_entity = self.node.create_client(SpawnEntity, '/spawn_entity')
        #self.remove_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.remove_entity = self.node.create_client(DeleteEntity, '/delete_entity')
        #self.set_model_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_entity_state = self.node.create_client(SetEntityState, '/set_entity_state')
        #self.get_model_state = rospy.ServiceProxy('/gazebo/get_entity_state', GetModelState)
        self.get_entity_state = self.node.create_client(GetEntityState, '/get_entity_state')
        #self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        self.assets_path = os.path.join(get_package_share_directory('gazebo_domain_random'), 'assets')

        self.addTarget()

        # Seed the environment
        self._seed()

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg =  message

    def tgt_callback(self,msg):
        # print("Whats the target?: ", msg)
        # self.realgoal is None and self.target_orientation is None:

        if self.detect_target_once is 1:
            print("Get the target from vision, for now just use position.")
            EE_POS_TGT = np.asmatrix([msg.position.x, msg.position.y, msg.position.z])
            rot_quat = np.quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            print("rot_quat: ",rot_quat)
            rot_matrix = tf.quaternions.mat2quat(rot_quat)
            print("rot_matrix: ", rot_matrix)
            EE_ROT_TGT = rot_matrix #np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            # rot_matrix
            # EE_ROT_TGT = np.asmatrix([[0.79660969, -0.51571238,  0.31536287], [0.51531424,  0.85207952,  0.09171542], [-0.31601302,  0.08944959,  0.94452874]]) #rot_matrix#
            EE_POINTS = np.asmatrix([[0, 0, 0]])
            ee_pos_tgt = EE_POS_TGT

            # leave rotation target same since in scara we do not have rotation of the end-effector
            ee_rot_tgt = EE_ROT_TGT
            target1 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T)

            # self.realgoal = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random1, ee_rot_tgt).T)

            self.realgoal = target1
            self.target_orientation = ee_rot_tgt
            print("Predicted target is: ", self.realgoal)
            self.detect_target_once = 0

            model_xml = "<?xml version=\"1.0\"?> \
                        <robot name=\"myfirst\"> \
                          <link name=\"world\"> \
                          </link>\
                          <link name=\"sphere0\">\
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
                            <child link=\"sphere0\"/>\
                          </joint>\
                          <gazebo reference=\"sphere0\">\
                            <material>Gazebo/GreenTransparent</material>\
                          </gazebo>\
                        </robot>"
            robot_namespace = ""
            pose = Pose()
            pose.position.x = EE_POS_TGT[0,0];
            pose.position.y = EE_POS_TGT[0,1];
            pose.position.z = EE_POS_TGT[0,2];

            #Static obstacle (not in original code)
            # pose.position.x = 0.25;#
            # pose.position.y = 0.07;#
            # pose.position.z = 0.0;#

            pose.orientation.x = 0;
            pose.orientation.y= 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;

            while not self.add_entity.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/spawn_entity service not available, waiting again...')

            req = SpawnEntity.Request()
            req.name = "target"
            req.xml = model_xml
            req.robot_namespace = ""
            req.initial_pose = pose
            req.reference_frame = ""

            future = self.add_entity.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

    def addTarget(self):
        # The idea is to add random target in our case rubik cube and the vision system to detect and find the 3D pose of the cube.
        # Open a file: file
        # os.chdir('../assets/urdf/rubik_cube')
        # print("os: ", os)
        file = open(self.assets_path + '/models/sdf/rubik_cube.sdf' ,mode='r')
        # read all lines at once
        model_xml = file.read()
        # close the file
        file.close()

        pose = Pose()

        pose.position.x = -0.5074649153217804#-0.5074649153217804#random.uniform(-0.3, -0.6);
        pose.position.y = random.uniform(-0.02, 0.01)#0.03617460539210797#random.uniform(-0.02, 0.01)        # stay put in Z!!!
        pose.position.z = 0.72#0.72#0.80 #0.72;

        roll = 0.0#random.uniform(-0.2, 0.6)
        pitch = 0.0#random.uniform(-0.2, 0.2)
        yaw = 0.0#-0.3#random.uniform(-0.3, 0.3)
        new_camera_pose = False
        q_rubik = tf.taitbryan.euler2quat(yaw, pitch, roll)
        # print("q_rubik: ", q_rubik.x, q_rubik.y, q_rubik.z, q_rubik.w)

        # q_rubik array Quaternion in w, x, y, z
        pose.orientation.x = q_rubik[1]#q_rubik.x#0.0
        pose.orientation.y = q_rubik[2]#q_rubik.y#0.0
        pose.orientation.z = q_rubik[3]#q_rubik.z#0.0
        pose.orientation.w = q_rubik[0]#q_rubik.w#0.0

        print("Real pose is: ", pose)
        try:
            while not self.add_entity.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/spawn_entity service not available, waiting again...')

            req = SpawnEntity.Request()
            req.name = "puzzle_ball_joints"
            req.xml = model_xml
            req.robot_namespace = ""
            req.initial_pose = pose
            req.reference_frame = ""

            future = self.add_entity.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            print ("service call ok")
        except:
            print('error adding model')

        while not self.set_entity_state.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/set_entity_state service not available, waiting again...')

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "puzzle_ball_joints"
        req.state.pose = pose
        #req.state.twist =
        req.state.reference_frame = "world"

        future = self.set_entity_state.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def removeTarget(self):
        while not self.remove_entity.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/delete_entity service not available, waiting again...')

        req = DeleteEntity.Request()
        req.name = "puzzle_ball_joints"

        future = self.remove_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def removeTargetSphere(self):
        while not self.remove_entity.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/delete_entity service not available, waiting again...')

        req = DeleteEntity.Request()
        req.name = "target"

        future = self.remove_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)


    def setTargetPositions(self, msg):
        """
        The goal is to test with randomized positions which range between the boundries of the H-ROS logo
        """
        print("In randomize target positions.")
        EE_POS_TGT_RANDOM1 = np.asmatrix([np.random.uniform(0.2852485,0.3883636), np.random.uniform(-0.1746508,0.1701576), 0.2868]) # boundry box of the first half H-ROS letters with +-0.01 offset
        # EE_POS_TGT_RANDOM1 = np.asmatrix([np.random.uniform(0.2852485, 0.3883636), np.random.uniform(-0.1746508, 0.1701576), 0.3746]) # boundry box of whole box H-ROS letters with +-0.01 offset
        # EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        EE_ROT_TGT = np.asmatrix([[0.79660969, -0.51571238,  0.31536287], [0.51531424,  0.85207952,  0.09171542], [-0.31601302,  0.08944959,  0.94452874]])
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        ee_pos_tgt_random1 = EE_POS_TGT_RANDOM1

        # leave rotation target same since in scara we do not have rotation of the end-effector
        ee_rot_tgt = EE_ROT_TGT
        target1 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random1, ee_rot_tgt).T)

        # self.realgoal = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random1, ee_rot_tgt).T)

        self.realgoal = target1
        print("randomizeTarget realgoal: ", self.realgoal)

    def randomizeTargetPositions(self):
        """
        The goal is to test with randomized positions which range between the boundries of the H-ROS logo
        """
        print("In randomize target positions.")
        EE_POS_TGT_RANDOM1 = np.asmatrix([np.random.uniform(0.2852485,0.3883636), np.random.uniform(-0.1746508,0.1701576), 0.2868]) # boundry box of the first half H-ROS letters with +-0.01 offset
        EE_POS_TGT_RANDOM2 = np.asmatrix([np.random.uniform(0.2852485,0.3883636), np.random.uniform(-0.1746508,0.1701576), 0.2868]) # boundry box of the H-ROS letters with +-0.01 offset
        # EE_POS_TGT_RANDOM1 = np.asmatrix([np.random.uniform(0.2852485, 0.3883636), np.random.uniform(-0.1746508, 0.1701576), 0.3746]) # boundry box of whole box H-ROS letters with +-0.01 offset
        EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        ee_pos_tgt_random1 = EE_POS_TGT_RANDOM1
        ee_pos_tgt_random2 = EE_POS_TGT_RANDOM2

        # leave rotation target same since in scara we do not have rotation of the end-effector
        ee_rot_tgt = EE_ROT_TGT
        target1 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random1, ee_rot_tgt).T)
        target2 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random2, ee_rot_tgt).T)

        # self.realgoal = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_random1, ee_rot_tgt).T)

        self.realgoal = target1 if np.random.uniform() < 0.5 else target2
        print("randomizeTarget realgoal: ", self.realgoal)

    def randomizeTarget(self):
        print("calling randomize target")

        EE_POS_TGT_1 = np.asmatrix([-0.189383, -0.123176, 0.894476]) # point 1
        EE_POS_TGT_2 = np.asmatrix([-0.359236, 0.0297278, 0.760402]) # point 2
        EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        EE_POINTS = np.asmatrix([[0, 0, 0]])

        ee_pos_tgt_1 = EE_POS_TGT_1
        ee_pos_tgt_2 = EE_POS_TGT_2

        # leave rotation target same since in scara we do not have rotation of the end-effector
        ee_rot_tgt = EE_ROT_TGT

        # Initialize target end effector position
        # ee_tgt = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T)

        target1 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_1, ee_rot_tgt).T)
        target2 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_2, ee_rot_tgt).T)


        """
        This is for initial test only, we need to change this in the future to be more realistic.
        E.g. covered target -> go to other target. This could be implemented for example with vision.
        """
        self.realgoal = target1 if np.random.uniform() < 0.5 else target2
        print("randomizeTarget realgoal: ", self.realgoal)

    def randomizeMultipleTargets(self):
        print("calling randomize multiple target")

        EE_POS_TGT_1 = np.asmatrix([0.3325683, 0.0657366, 0.2868]) # center of O
        EE_POS_TGT_2 = np.asmatrix([0.3305805, -0.1326121, 0.2868]) # center of the H
        EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        EE_POINTS = np.asmatrix([[0, 0, 0]])

        ee_pos_tgt_1 = EE_POS_TGT_1
        ee_pos_tgt_2 = EE_POS_TGT_2

        # leave rotation target same since in scara we do not have rotation of the end-effector
        ee_rot_tgt = EE_ROT_TGT

        # Initialize target end effector position
        # ee_tgt = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T)

        target1 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_1, ee_rot_tgt).T)
        target2 = np.ndarray.flatten(get_ee_points(EE_POINTS, ee_pos_tgt_2, ee_rot_tgt).T)

        """
        This is for initial test only, we need to change this in the future to be more realistic.
        E.g. covered target -> go to other target. This could be implemented for example with vision.
        """
        self.realgoal = target1 if np.random.uniform() < 0.5 else target2
        print("randomizeTarget realgoal: ", self.realgoal)

    def take_observation(self):
        """
        Take observation from the environment and return it.
        TODO: define return type
        """
        # Take an observation
        # done = False

        obs_message = self._observation_msg
        if obs_message is None:
            # print("last_observations is empty")
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
            # #
            rotation_matrix = np.eye(4)
            rotation_matrix[:3, :3] = rot
            rotation_matrix[:3, 3] = trans
            # angle, dir, _ = rotation_from_matrix(rotation_matrix)
            # #
            # current_quaternion = np.array([angle]+dir.tolist())#

            # I need this calculations for the new reward function, need to send them back to the run mara or calculate them here
            current_quaternion = tf.quaternions.mat2quat(rot) #[w, x, y ,z]
            tgt_quartenion = tf.quaternions.mat2quat(self.target_orientation)

            A  = np.vstack([current_quaternion, np.ones(len(current_quaternion))]).T

            #quat_error = np.linalg.lstsq(A, tgt_quartenion)[0]

            quat_error = current_quaternion - tgt_quartenion
            # print("quat_error: ",quat_error)
            # print("self.realgoal: ", self.realgoal)
            # print("curr quat: ", current_quaternion)
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
            #print("quat_error: ", quat_error)
            #print("ee_points:", ee_points)
            return np.r_[np.reshape(last_observations, -1),
                          np.reshape(ee_points, -1),
                          np.reshape(quat_error, -1),
                          np.reshape(ee_velocities, -1),]

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
                while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                    self.node.get_logger().info('/reset_simulation service not available, waiting again...')

                reset_future = self.reset_sim.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self.node, reset_future)

                # Avoid unnecessary pose check.
                break

            rclpy.spin_once(self.node)
            obs_message = self._observation_msg
            if obs_message is not None:
                last_observation = ut_mara.process_observations(obs_message, self.environment)
                action_finished = ut_mara.positions_match(action, last_observation)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - reward
            - done (status)
            - action
            - observation
            - dictionary (#TODO clarify)
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
        # TODO: program this better, check that ob is not None, etc.
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        self.reward_dist = -ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])
        self.reward_orient = - ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)])
        # print("self.reward_orient: ", self.reward_orient)

        #scale here the orientation because it should not be the main bias of the reward, position should be
        orientation_scale = 0.2

        # here we want to fetch the positions of the end-effector which are nr_dof:nr_dof+3
        if(ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])<0.005):
            self.reward = 1 - ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)]) # Make the reward increase as the distance decreases
            print("Reward position is: ", self.reward)
        else:
            self.reward = self.reward_dist

        if(ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)])<0.1):
            self.reward = self.reward +  orientation_scale * (1 -ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)]))
            print("Reward orientation is: ", self.reward)
        else:
            self.reward = self.reward + orientation_scale * ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)])

        # Calculate if the env has been solved
        done = bool(((abs(self.reward_dist) < 0.005) and (abs(self.reward_orient)) < 0.1) or (self.iterator>self.max_episode_steps))

        return self.ob, self.reward, done, {}

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            self._pub.publish(ut_mara.get_trajectory_message(
                self.environment['reset_conditions']['initial_positions'],
                self.environment['joint_order'],
                self.velocity))

            self.wait_for_action(self.environment['reset_conditions']['initial_positions'])

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        # Return the corresponding observation
        return self.ob
