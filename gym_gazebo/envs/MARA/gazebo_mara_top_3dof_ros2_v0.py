import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import threading # Used for time locks to synchronize position data.
import os
from gym import utils, spaces
from gym_gazebo.utils import launch_helpers
from gym.utils import seeding
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnEntity
from multiprocessing import Process

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
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

class GazeboMARATop3DOFv0EnvROS2(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        self.gzserver_only = False # Set to False or comment for the complete server+client option.

        # Launch mara in a new Process
        launch_helpers.start_launch_servide_process(self.generate_launch_description())

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

    def generate_launch_description(self):
        """
            Returns ROS2 LaunchDescription object.
        """
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


        # Exclusive network segmentation, which allows to launch multiple instances of ROS2+Gazebo
        network_params = launch_helpers.get_exclusive_network_parameters()
        os.environ["ROS_DOMAIN_ID"] = network_params.get('ros_domain_id')
        os.environ["GAZEBO_MASTER_URI"] = network_params.get('gazebo_master_uri')
        print("ROS_DOMAIN_ID=" + network_params.get('ros_domain_id'))
        print("GAZEBO_MASTER_URI=" + network_params.get('gazebo_master_uri'))

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
        if self.gzserver_only:
            gazebo_cmd = "gzserver"

        # Creation of ROS2 LaunchDescription obj.
        ld = LaunchDescription([
            ExecuteProcess(
                cmd=[gazebo_cmd, '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen',
                env=envs
            ),
            Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
            Node(package='mara_utils_scripts', node_executable='spawn_entity.py', output='screen'),
            Node(package='hros_cognition_mara_components', node_executable='hros_cognition_mara_components', output='screen',
                arguments=["-motors", install_dir + "/share/hros_cognition_mara_components/link_order.yaml"])
        ])
        return ld

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg =  message

    def get_trajectory_message(self, action, robot_id=0):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        Velocity must be set now. Duration (self.slowness) does not control velocity now.
        """
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = self.environment['joint_order']
        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        action_float = [float(i) for i in action]
        target.positions = action_float
        target.velocities = [1.0]*action.size #rad/s. Real MARA max speed is 1.41

        action_msg.points = [target]
        return action_msg

    def process_observations(self, message, agent, robot_id=0):
        """
        Helper fuinction to convert a ROS message to joint angles and velocities.
        Check for and handle the case where a message is either malformed
        or contains joint values in an order different from that expected observation_callback
        in hyperparams['joint_order']
        """
        if not message:
            print("Message is empty");
            # return None
        else:
            # # Check if joint values are in the expected order and size.
            if message.joint_names != agent['joint_order']:
                # Check that the message is of same size as the expected message.
                if len(message.joint_names) != len(agent['joint_order']):
                    raise MSG_INVALID_JOINT_NAMES_DIFFER

                # Check that all the expected joint values are present in a message.
                # if not all(map(lambda x,y: x in y, message.joint_names,
                #     [self._valid_joint_set[robot_id] for _ in range(len(message.joint_names))])):
                #     raise MSG_INVALID_JOINT_NAMES_DIFFER
                    print("Joints differ")
            return np.array(message.actual.positions) # + message.actual.velocities

    def get_jacobians(self, state, robot_id=0):
        """
        Produce a Jacobian from the urdf that maps from joint angles to x, y, z.
        This makes a 6x6 matrix from 6 joint angles to x, y, z and 3 angles.
        The angles are roll, pitch, and yaw (not Euler angles) and are not needed.
        Returns a repackaged Jacobian that is 3x6.
        """
        # Initialize a Jacobian for self.scara_chain.getNrOfJoints() joint angles by 3 cartesian coords and 3 orientation angles
        jacobian = Jacobian(self.scara_chain.getNrOfJoints())
        # Initialize a joint array for the present self.scara_chain.getNrOfJoints() joint angles.
        angles = JntArray(self.scara_chain.getNrOfJoints())
        # Construct the joint array from the most recent joint angles.
        for i in range(self.scara_chain.getNrOfJoints()):
            angles[i] = state[i]
        # Update the jacobian by solving for the given angles.observation_callback
        self.jac_solver.JntToJac(angles, jacobian)
        # Initialize a numpy array to store the Jacobian.
        J = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])
        # Only want the cartesian position, not Roll, Pitch, Yaw (RPY) Angles
        ee_jacobians = J
        return ee_jacobians

    def get_ee_points_jacobians(self, ref_jacobian, ee_points, ref_rot):
        """
        Get the jacobians of the points on a link given the jacobian for that link's origin
        :param ref_jacobian: 6 x 6 numpy array, jacobian for the link's origin
        :param ee_points: N x 3 numpy array, points' coordinates on the link's coordinate system
        :param ref_rot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
        :return: 3N x 6 Jac_trans, each 3 x 6 numpy array is the Jacobian[:3, :] for that point
                 3N x 6 Jac_rot, each 3 x 6 numpy array is the Jacobian[3:, :] for that point
        """
        ee_points = np.asarray(ee_points)
        ref_jacobians_trans = ref_jacobian[:3, :]
        ref_jacobians_rot = ref_jacobian[3:, :]
        end_effector_points_rot = np.expand_dims(ref_rot.dot(ee_points.T).T, axis=1)
        ee_points_jac_trans = np.tile(ref_jacobians_trans, (ee_points.shape[0], 1)) + \
                                        np.cross(ref_jacobians_rot.T, end_effector_points_rot).transpose(
                                            (0, 2, 1)).reshape(-1, self.scara_chain.getNrOfJoints())
        ee_points_jac_rot = np.tile(ref_jacobians_rot, (ee_points.shape[0], 1))
        return ee_points_jac_trans, ee_points_jac_rot

    def get_ee_points_velocities(self, ref_jacobian, ee_points, ref_rot, joint_velocities):
        """
        Get the velocities of the points on a link
        :param ref_jacobian: 6 x 6 numpy array, jacobian for the link's origin
        :param ee_points: N x 3 numpy array, points' coordinates on the link's coordinate system
        :param ref_rot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
        :param joint_velocities: 1 x 6 numpy array, joint velocities
        :return: 3N numpy array, velocities of each point
        """
        ref_jacobians_trans = ref_jacobian[:3, :]
        ref_jacobians_rot = ref_jacobian[3:, :]
        ee_velocities_trans = np.dot(ref_jacobians_trans, joint_velocities)
        ee_velocities_rot = np.dot(ref_jacobians_rot, joint_velocities)
        ee_velocities = ee_velocities_trans + np.cross(ee_velocities_rot.reshape(1, 3),
                                                       ref_rot.dot(ee_points.T).T)
        return ee_velocities.reshape(-1)

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
        last_observations = self.process_observations(obs_message, self.environment)
        # # # Get Jacobians from present joint angles and KDL trees
        # # # The Jacobians consist of a 6x6 matrix getting its from from
        # # # (# joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = self.get_jacobians(last_observations)
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
            ee_points_jac_trans, _ = self.get_ee_points_jacobians(ee_link_jacobians,
                                                                   self.environment['end_effector_points'],
                                                                   rot)
            ee_velocities = self.get_ee_points_velocities(ee_link_jacobians,
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

    def rmse_func(self, ee_points):
        """
        Computes the Residual Mean Square Error of the difference between current and desired end-effector position
        """
        rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
        return rmse

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - reward
            - done (status)
            - action
            - observation
            - dictionary
        """
        self.iterator+=1

        # Execute "action"
        self._pub.publish(self.get_trajectory_message(action[:self.scara_chain.getNrOfJoints()]))
        # TODO. Wait unlit the action is finished.
        time.sleep(3)

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            print("step: observation is Empty")
            self.ob = self.take_observation()

        self.reward_dist = -self.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])
        self.reward_orient = - self.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+7)])

        #scale here the orientation because it should not be the main bias of the reward, position should be
        orientation_scale = 0.1

        # here we want to fetch the positions of the end-effector which are nr_dof:nr_dof+3
        if(self.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])<0.005):
            self.reward = 1 - self.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)]) # Make the reward increase as the distance decreases
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
            self._pub.publish(self.get_trajectory_message(self.environment['reset_conditions']['initial_positions']))

        # TODO. Wait unlit the action is finished.
        time.sleep(3)

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        # Return the corresponding observation
        return self.ob
