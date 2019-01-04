import gym
import time
import os
import numpy as np
from gym import utils, spaces
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
import copy
import threading # Used for time locks to synchronize position data.

from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetLinkState, GetModelState, SpawnEntity

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import ContactState
from gazebo_msgs.msg import ModelState, LinkState

from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Empty as stdEmpty

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from baselines.agent.scara_arm.tree_urdf import treeFromFile # For KDL Jacobians
from PyKDL import Jacobian, Chain, ChainJntToJacSolver, JntArray # For KDL Jacobians

import cv2

import quaternion as quat

import write_csv as csv_file

# from custom baselines repository
from baselines.agent.utility.general_utils import forward_kinematics, get_ee_points, rotation_from_matrix, \
    get_rotation_matrix,quaternion_from_matrix # For getting points and velocities.

class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass


class GazeboMARATopOrientCollisionv0Env(gym.Env):
    """
    This environment present a modular SCARA robot with a range finder at its
    end pointing towards the workspace of the robot. The goal of this environment is
    defined to reach the center of the "H" or the "O" from the "H-ROS" logo within the worspace.
    This environment uses `slowness=1` and matches the delay between actions/observations
    to this value (slowness). In other words, actions are taken at "1/slowness" rate.

    Reward is determined ... (TODO: describe the heuristic or reward calculation method)
    """
    def __init__(self):
        """
        Initialize the MARA environemnt
            NOTE: This environment uses ROS and interfaces.

            TODO: port everything to ROS 2 natively
        """
        self.gzserver_only = False # Set to False or comment for the complete server+client option.

        # Launch mara in a new Process
        launch_helpers.start_launch_servide_process(self.generate_launch_description())
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
        self.max_episode_steps = 1000 # now used in all algorithms, this should reflect the lstm step size, otherwqise it restarts two times
        self.rand_target_thresh = 40
        self.iterator = 0
        self.reset_iter = 0
        # default to seconds
        self.slowness = 1
        self.slowness_unit = 'sec'
        self.reset_jnts = True
        self._collision_msg = None

        self._time_lock = threading.RLock()

        #############################
        #   Environment hyperparams
        #############################
        # target, where should the agent reach

        EE_POS_TGT = np.asmatrix([-0.40028, 0.095615, 0.72466]) # alex2
        # EE_POS_TGT = np.asmatrix([-0.173762, -0.0124312, 1.60415]) # for testing collision_callback
        # EE_POS_TGT = np.asmatrix([-0.580238, -0.179591, 0.72466]) # rubik touching the bar
        # EE_ROT_TGT = np.asmatrix([[-0.00128296,  0.9999805 ,  0.00611158],
        #                            [ 0.00231397, -0.0061086 ,  0.99997867],
        #                            [ 0.9999965 ,  0.00129708, -0.00230609]])
        # EE_POS_TGT = np.asmatrix([-0.390768, 0.0101776, 0.725335]) # 200 cm from the z axis
        # EE_POS_TGT = np.asmatrix([0.0, 0.001009, 1.64981])
        # EE_POS_TGT = np.asmatrix([-0.4023037912211465, 0.15501116706606247, 0.7238499613771884]) # 200 cm from the z axis

        # # EE_POS_TGT = np.asmatrix([0.3305805, -0.1326121, 0.4868]) # center of the H
        # EE_ROT_TGT = np.asmatrix([[-0.99521107,  0.09689605, -0.01288708],
        #                           [-0.09768035, -0.99077857,  0.09389558],
        #                           [-0.00367013,  0.09470474,  0.99549864]])


        # EE_ROT_TGT = np.asmatrix([[-0.99521107,  0.09689605, -0.01288708],
        #                           [-0.09768035, -0.99077857,  0.09389558],
        #                           [-0.00367013,  0.09470474,  0.99549864]])
        # EE_ROT_TGT = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        EE_ROT_TGT = np.asmatrix([[0.79660969, -0.51571238,  0.31536287], [0.51531424,  0.85207952,  0.09171542], [-0.31601302,  0.08944959,  0.94452874]]) # original orientation
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])
        # Initial joint position
        # INITIAL_JOINTS = np.array([0., 0., 1., 0., 1.57, 0.])
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])
        # Used to initialize the robot, #TODO, clarify this more
        # STEP_COUNT = 2  # Typically 100.
        # slowness = 10000000 # 10 ms, where 1 second is real life simulation
        # slowness = 1000000 # 1 ms, where 1 second is real life simulation
        # slowness = 1 # use >10 for running trained network in the simulation
        # slowness = 10 # use >10 for running trained network in the simulation

        # Topics for the robot publisher and subscriber.
        JOINT_PUBLISHER = '/mara_controller/command'
        JOINT_SUBSCRIBER = '/mara_controller/state'
        RAND_LIGHT_PUBLISHER = '/randomizers/randomizer/light'
        RAND_SKY_PUBLISHER = '/randomizers/randomizer/sky'
        RAND_PHYSICS_PUBLISHER = '/randomizers/randomizer/physics'
        LINK_STATE_PUBLISHER = '/gazebo/set_link_state'
        RAND_OBSTACLES_PUBLISHER = '/randomizers/randomizer/obstacles'

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
            # 'slowness': slowness,
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
        self._sub_coll = self.node.create_subscription(ContactState,
                                                '/gazebo_contacts',
                                                self.collision_callback)
        self._pub_rand_light = self.node.create_publisher(stdEmpty, RAND_LIGHT_PUBLISHER)
        self._pub_rand_sky = self.node.create_publisher(stdEmpty, RAND_SKY_PUBLISHER)
        self._pub_rand_physics = self.node.create_publisher(stdEmpty, RAND_PHYSICS_PUBLISHER)
        self._pub_rand_obstacles = self.node.create_publisher(stdEmpty, RAND_OBSTACLES_PUBLISHER)
        self._pub_link_state = self.node.create_publisher(stdEmpty, LINK_STATE_PUBLISHER)

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
        self.obs_dim = self.scara_chain.getNrOfJoints() + 9#7 #6 hardcode it for now
        # # print(observation, _reward)

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        # #bounds = self.model.actuator_ctrlrange.copy()
        low = -np.pi * np.ones(self.scara_chain.getNrOfJoints())
        high = np.pi * np.ones(self.scara_chain.getNrOfJoints())
        # low = -np.inf * np.ones(self.scara_chain.getNrOfJoints())
        # high = np.inf * np.ones(self.scara_chain.getNrOfJoints())
        self.action_space = spaces.Box(low, high, dtype=np.float32)
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)




        # Migration to ROS2

        # self.add_model_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # self.add_model_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.add_entity = self.node.create_client(SpawnEntity, '/spawn_entity')

        #self.remove_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.remove_entity = self.node.create_client(DeleteEntity, '/delete_entity')

        #self.set_model_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_entity_state = self.node.create_client(SetEntityState, '/set_entity_state')

        #self.get_model_state = rospy.ServiceProxy('/gazebo/get_entity_state', GetModelState)
        self.get_entity_state = self.node.create_client(GetEntityState, '/get_entity_state')

        #self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        robot_namespace = ""
        pose = Pose()
        pose.position.x = EE_POS_TGT[0,0];
        pose.position.y = EE_POS_TGT[0,1];
        pose.position.z = EE_POS_TGT[0,2];
        pose.orientation.x = 0;
        pose.orientation.y= 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
        reference_frame = "world"

        self.assets_path = os.path.abspath(os.path.join(rospkg.RosPack().get_path("gazebo_domain_randomizer"), os.pardir)) + "/assets"

        file_xml = open(self.assets_path + '/models/urdf/target_point.urdf' ,mode='r')
        model_xml = file_xml.read()
        file_xml.close()

        # ROS 1
        # rospy.wait_for_service('/gazebo/spawn_urdf_model')
        # try:
        #     self.add_entity(model_name="target",
        #                         model_xml=model_xml,
        #                         robot_namespace=robot_namespace,
        #                         initial_pose=pose,
        #                         reference_frame=reference_frame)
        # except rospy.ServiceException as e:
        #     print('Error adding urdf model')

        # ROS 2
        while not self.add_entity.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        req = SpawnEntity.Request()
        req.name = "target"
        req.xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = pose
        req.reference_frame = reference_frame

        future = self.add_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        # # self.obj_path = self.assets_path + '/models/sdf/rubik_cube_random.sdf'
        # # self.obj_path = self.assets_path + '/models/sdf/rubik_cube.sdf'
        # self.obj_path = self.assets_path + '/models/sdf/box.sdf' #0.067 0.067 0.067
        # # self.obj_path = self.assets_path + '/models/sdf/cylinder.sdf' # radius = 0.03 length = 0.08
        # # self.obj_path = self.assets_path + '/models/urdf/sphere.urdf' #radius = 0.033
        # file_sdf = open(self.obj_path ,mode='r')
        # model_sdf = file_sdf.read()
        # file_sdf.close()
        #
        # rospy.wait_for_service('/gazebo/spawn_sdf_model')
        # # rospy.wait_for_service('/gazebo/spawn_urdf_model')
        # try:
        #     # self.add_entity(model_name="obj",
        #     self.add_entity(model_name="obj",
        #                         model_xml=model_sdf,
        #                         robot_namespace=robot_namespace,
        #                         initial_pose=pose,
        #                         reference_frame=reference_frame)
        # except rospy.ServiceException as e:
        #     print('Error adding sdf model')

        self._seed()

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        # self._collision_msg = None
        # if message.collision1_name is not message.collision2_name:
        #     if "obj" not in message.collision1_name and "obj" not in message.collision2_name:
        #         if "obj" not in message.collision1_name or "robot::table::table_fixed_joint_lump__mara_work_area_link_collision_4" not in message.collision2_name:
        #             if "robot::motor6_link::motor6_link_fixed_joint_lump__robotiq_arg2f_base_link_collision_1" not in message.collision1_name and  "robot::left_outer_finger::left_outer_finger_collision" not in message.collision2_name:
        #                 self._collision_msg =  message

        if message.collision1_name != message.collision2_name:
            # neither obj nor obstacle colliding with table
            if "obj::" not in message.collision1_name and "obstacle" not in message.collision1_name or "table_fixed_joint_lump__mara_work_area_link_collision_4" not in message.collision2_name:
                # neither obj colliding with obstacle and vice-versa nor objs colliding each other nor obstacles colliding each other
                if "obj::" not in (message.collision1_name and message.collision2_name) and "obstacle" not in (message.collision1_name and message.collision2_name):
                    self._collision_msg = message

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg =  message

    def init_time(self, slowness =1, slowness_unit='sec', reset_jnts=True):
        self.slowness = slowness
        self.slowness_unit = slowness_unit
        self.reset_jnts = reset_jnts
        print("slowness: ", self.slowness)
        print("slowness_unit: ", self.slowness_unit, "type of variable: ", type(slowness_unit))
        print("reset joints: ", self.reset_jnts, "type of variable: ", type(self.reset_jnts))

    def getModelFileType(self, path):
        if path.endswith('.sdf'):
            return "sdf"
        elif path.endswith('.urdf'):
            return "urdf"
        else:
            raise TypeError('the file must be .sdf or .urdf')

    def randomizeSize(self, current_obj_name, shape):
        f = open(self.obj_path,'r+')
        model_xml = f.read()
        f.seek(0)

        min_size = 0.03
        max_size = 0.1

        if shape != 'box' and shape != 'cylinder' and shape != 'sphere':
            raise TypeError("Shape must be 'box', 'cylinder' or 'sphere'")

        if shape == "box":
            size_x = str( round( np.random.uniform(min_size, max_size), 3 ) )
            size_y = str( round( np.random.uniform(min_size, max_size), 3 ) )
            size_z = str( round( np.random.uniform(min_size, max_size), 3 ) )
            # height position of the wooden part of the table + height of the obj
            z = 0.69525 + float(size_z) / 2
        else:
            radius = str( round( np.random.uniform(min_size, max_size/2), 3 ) )
            z = 0.69525 + float(radius)
            if shape == "cylinder":
                length = str( round( np.random.uniform(min_size, max_size), 3 ) )
                z = 0.69525 + float(length) / 2

        model_file = self.getModelFileType(self.obj_path)
        if model_file == "sdf":
            if shape == "box":
                to_replace = model_xml[model_xml.find('<size>')+len('<size>'):model_xml.find('</size>')]
                model_xml = model_xml.replace(to_replace, size_x + ' ' + size_y + ' ' + size_z)
            else:
                to_replace = model_xml[model_xml.find('<radius>')+len('<radius>'):model_xml.find('</radius>')]
                model_xml = model_xml.replace(to_replace, radius)
                if shape == "cylinder":
                    to_replace = model_xml[model_xml.find('<length>')+len('<length>'):model_xml.find('</length>')]
                    model_xml = model_xml.replace(to_replace, length)

        else:
            if shape == "box":
                to_replace = model_xml[model_xml.find('<box ')+len('<box '):model_xml.find('/>')]
                model_xml = model_xml.replace(to_replace, 'size=' + '"' + size_x + ' ' + size_y + ' ' + size_z + '"')
            else:
                to_replace = model_xml[model_xml.find('<sphere ')+len('<sphere '):model_xml.find('/>')]
                model_xml = model_xml.replace(to_replace, 'radius=' + '"' + radius + '"')
                if shape == "cylinder":
                    to_replace = model_xml[model_xml.find('<cylinder ')+len('<cylinder '):model_xml.find('/>')]
                    model_xml = model_xml.replace(to_replace, 'length=' + '"' + length + '" ' + 'radius=' + '"' + radius + '"')

        f.truncate()
        f.write(model_xml)
        f.close()
        self.randomizeObjectType(current_obj_name=current_obj_name, replace=self.obj_path)
        self.randomizeTargetPose(obj_name=current_obj_name, centerPoint=z)

    def random_texture(self):
        material_path = self.assets_path + "/media/materials/scripts/textures.material"
        m = open(material_path,'r')
        textures = []

        for t in m:
            if t.startswith("material "):
                textures.append(t[9:-1])

        rand_texture = np.random.choice(textures)
        return rand_texture, textures

    def randomizeTexture(self, current_obj_name):
        f = open(self.obj_path,'r+')
        model_xml = f.read()
        f.seek(0)

        new_texture, list_textures = self.random_texture()
        for lt in list_textures:
            if lt != str(new_texture):
                model_xml = model_xml.replace(lt, new_texture)

        f.truncate()
        f.write(model_xml)
        f.close()
        self.randomizeObjectType(current_obj_name=current_obj_name, replace=self.obj_path)

    def spawnModel(self, obj_name, obj_path, pose):
        model_file = self.getModelFileType(obj_path)
        obj_file = open(obj_path, mode='r')
        xml = obj_file.read()
        obj_file.close()

        # ROS 1
        # if model_file == "sdf":
        #     rospy.wait_for_service('/gazebo/spawn_sdf_model')
        #     try:
        #         self.add_entity(model_name=obj_name,
        #                             model_xml=xml,
        #                             robot_namespace="",
        #                             initial_pose=pose,
        #                             reference_frame="world")
        #     except rospy.ServiceException as e:
        #         print("Error adding sdf")
        # else:
        #     rospy.wait_for_service('/gazebo/spawn_urdf_model')
        #     try:
        #         self.add_entity(model_name=obj_name,
        #                             model_xml=xml,
        #                             robot_namespace="",
        #                             initial_pose=pose,
        #                             reference_frame="world")
        #     except rospy.ServiceException as e:
        #         print("Error adding urdf")

        # ROS 2
        while not self.add_entity.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        req = SpawnEntity.Request()
        req.name = obj_name
        req.xml = xml
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = "world"

        future = self.add_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)



    def randomizeObjectType(self, current_obj_name=None, list_obj=None, replace=None):
        # obj = ModelState()

        # rospy.wait_for_service('gazebo/get_entity_state')
        # try:
        #     obj = self.get_entity_state("target", '')
        # except rospy.ServiceException as e:
        #     print("Error getting the model state")
        
        # ROS 2
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        req = GetEntityState.Request()
        req.name = 'target'
        req.reference_frame = ''

        obj = self.add_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, obj)


        # rospy.wait_for_service('/gazebo/delete_model')
        # try:
        #     self.remove_entity(current_obj_name)
        # except rospy.ServiceException as e:
        #     print("Error removing model")

        # ROS 2
        while not self.remove_entity.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        req = DeleteEntity.Request()
        req.name = current_obj_name

        future = self.remove_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)


        if replace is None:
            random_obj = np.random.choice(list_obj)
            self.obj_path = random_obj
        else:
            random_obj = replace

        self.spawnModel(current_obj_name, random_obj, obj.pose)

    def randomizeStartPose(self, lower, upper):
        self.environment['reset_conditions']['initial_positions'] = [ np.random.uniform(lower,upper), np.random.uniform(lower,upper),np.random.uniform(lower,upper), np.random.uniform(lower,upper),np.random.uniform(lower,upper),np.random.uniform(lower,upper) ]
        self._pub.publish(self.get_trajectory_message(self.environment['reset_conditions']['initial_positions']))

    def randomizeTargetPose(self, obj_name, centerPoint=False):
        ms = ModelState()
        if not centerPoint:
            EE_POS_TGT = np.asmatrix([ round(np.random.uniform(-0.62713, -0.29082), 5), round(np.random.uniform(-0.15654, 0.15925), 5), self.realgoal[2] ])

            roll = 0.0
            pitch = 0.0
            yaw = np.random.uniform(-1.57, 1.57)
            q = quat.from_euler_angles(roll, pitch, yaw)
            EE_ROT_TGT = quat.as_rotation_matrix(q)
            self.target_orientation = EE_ROT_TGT
            ee_tgt = np.ndarray.flatten(get_ee_points(self.environment['end_effector_points'], EE_POS_TGT, EE_ROT_TGT).T)

            ms.pose.position.x = EE_POS_TGT[0,0]
            ms.pose.position.y = EE_POS_TGT[0,1]
            ms.pose.position.z = EE_POS_TGT[0,2]
            ms.pose.orientation.x = q.x
            ms.pose.orientation.y = q.y
            ms.pose.orientation.z = q.z
            ms.pose.orientation.w = q.w

            if obj_name != "target":
                ms.model_name = obj_name
                rospy.wait_for_service('gazebo/set_model_state')
                try:
                    self.set_entity_state(ms)
                except (rospy.ServiceException) as e:
                    print("Error setting the pose of " + obj_name)

                self.spawnModel(obj_name, self.obj_path, ms.pose)

        else:
            EE_POS_TGT = np.asmatrix([self.realgoal[0], self.realgoal[1], centerPoint])
            ee_tgt = np.ndarray.flatten(get_ee_points(self.environment['end_effector_points'], EE_POS_TGT, self.target_orientation).T)

            ms.pose.position.x = EE_POS_TGT[0,0]
            ms.pose.position.y = EE_POS_TGT[0,1]
            ms.pose.position.z = EE_POS_TGT[0,2]
            ms.pose.orientation.x = 0;
            ms.pose.orientation.y= 0;
            ms.pose.orientation.z = 0;
            ms.pose.orientation.w = 0;

        self._pub_link_state.publish( LinkState(link_name="target_link", pose=ms.pose, reference_frame="world") )
        self.realgoal = ee_tgt

    def get_trajectory_message(self, action, robot_id=0):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = self.environment['joint_order']
        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        action_float = [float(i) for i in action]
        target.positions = action_float
        # These times determine the speed at which the robot moves:
        # it tries to reach the specified target position in 'slowness' time.
        if (self.slowness_unit == 'sec') or (self.slowness_unit is None):
            target.time_from_start.secs = self.slowness
        elif (self.slowness_unit == 'nsec'):
            target.time_from_start.nsecs = self.slowness
        else:
            print("Unrecognized unit. Please use sec or nsec.")

        # Package the single point into a trajectory of points with length 1.
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
        else:
            # # Check if joint values are in the expected order and size.
            if message.joint_names != agent['joint_order']:
                # Check that the message is of same size as the expected message.
                if len(message.joint_names) != len(agent['joint_order']):
                    raise MSG_INVALID_JOINT_NAMES_DIFFER

                # Check that all the expected joint values are present in a message.
                if not all(map(lambda x,y: x in y, message.joint_names,
                    [self._valid_joint_set[robot_id] for _ in range(len(message.joint_names))])):
                    raise MSG_INVALID_JOINT_NAMES_DIFFER
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
        TODO: define return type
        """
        obs_message = self._observation_msg
        if obs_message is None:
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
            trans, rot = forward_kinematics(self.scara_chain,
                                        self.environment['link_names'],
                                        last_observations[:self.scara_chain.getNrOfJoints()],
                                        base_link=self.environment['link_names'][0],
                                        end_link=self.environment['link_names'][-1])

            rotation_matrix = np.eye(4)
            rotation_matrix[:3, :3] = rot
            rotation_matrix[:3, 3] = trans
            # angle, dir, _ = rotation_from_matrix(rotation_matrix)
            # current_quaternion = np.array([angle]+dir.tolist())#

            # # I need this calculations for the new reward function, need to send them back to the run mara or calculate them here
            # current_quaternion = quaternion_from_matrix(rotation_matrix)
            # tgt_quartenion = quaternion_from_matrix(self.target_orientation)

            current_quaternion = quat.from_rotation_matrix(rotation_matrix)
            tgt_quartenion = quat.from_rotation_matrix(self.target_orientation)

            # A  = np.vstack([current_quaternion, np.ones(len(current_quaternion))]).T
            #quat_error = np.linalg.lstsq(A, tgt_quartenion)[0]

            quat_error = current_quaternion * tgt_quartenion.conjugate()
            rot_vec_err = quat.as_rotation_vector(quat_error)

            # convert quat to np arrays
            quat_error = quat.as_float_array(quat_error)

            # RK:  revisit this later, we only take one angle difference here!
            angle_diff = 2 * np.arccos(np.clip(quat_error[..., 0], -1., 1.))

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
                          np.reshape(rot_vec_err, -1),
                          np.reshape(ee_velocities, -1),]
            return state

    def rmse_func(self, ee_points):
        """
        Computes the Residual Mean Square Error of the difference between current and desired end-effector position
        """
        rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
        return rmse

    def _seed(self, seed=None):
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

        self._pub.publish(self.get_trajectory_message(action[:self.scara_chain.getNrOfJoints()]))

        # # Take an observation
        # TODO: program this better, check that ob is not None, etc.
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        self.reward_dist = -self.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])
        # careful we have degrees now so we scale with
        orientation_scale = 0.1
        self.reward_orient = - orientation_scale * self.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+6)])
        #scale here the orientation because it should not be the main bias of the reward, position should be

        if self._collision_msg is not None:
            if self._collision_msg.collision1_name is None:
                raise AttributeError("collision1_name is None")
            if self._collision_msg.collision2_name is None:
                raise AttributeError("collision2_name is None")
            # print(self._collision_msg.collision1_name)
            # print(self._collision_msg.collision2_name)

            self.reward = (self.reward_dist + self.reward_orient) * 5.0
            # self.reward = (self.reward_dist + self.reward_orient) * 6.0
            # print("Reward collision is: ", self.reward)

            # Resets the state of the environment and returns an initial observation.
            # we should avoid this --> huge bottleneck

            # rospy.wait_for_service('/gazebo/reset_simulation')
            # try:
            #     self.reset_sim()

            # ROS 2
            # Still failing
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

            pose = Pose()
            pose.position.x = self.realgoal[0];
            pose.position.y = self.realgoal[1];
            pose.position.z = self.realgoal[2];
            pose.orientation.x = 0;
            pose.orientation.y= 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;
            self._pub_link_state.publish(LinkState(link_name="target_link", pose=pose, reference_frame="world"))

        else:
            # here we want to fetch the positions of the end-effector which are nr_dof:nr_dof+3
            # here is the distance block
            if abs(self.reward_dist) < 0.005:
                self.reward = 1 + self.reward_dist # Make the reward increase as the distance decreases
                self.reset_iter +=1
                print("Reward dist is: ", self.reward)
                if abs(self.reward_orient)<0.005:
                    self.reward = (2 + self.reward + self.reward_orient)*2
                    print("Reward dist + orient is: ", self.reward)
                else:
                    self.reward = self.reward + self.reward_orient
                    print("Reward dist+(orient=>0.01) is: ", self.reward)

            else:
                self.reward = self.reward_dist

        done = bool((abs(self.reward_dist) < 0.001) or (self.iterator>self.max_episode_steps) or (abs(self.reward_orient) < 0.001) )
        # done = bool( (abs(self.reward_dist) < 0.005) or (self.iterator > self.max_episode_steps) )

        # Return the corresponding observations, rewards, etc.
        # TODO, understand better what's the last object to return
        self._collision_msg = None
        return self.ob, self.reward, done, {}

    def goToInit(self):
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()
        # # Go to initial position and wait until it arrives there
        # Wait until the arm is within epsilon of reset configuration.
        self._time_lock.acquire(True, -1)
        with self._time_lock:
            self._currently_resetting = True
        self._time_lock.release()

        if self._currently_resetting:
            epsilon = 1e-3
            reset_action = self.environment['reset_conditions']['initial_positions']
            now_action = self._observation_msg.actual.positions
            du = np.linalg.norm(reset_action-now_action, float(np.inf))
            self._pub.publish(self.get_trajectory_message(self.environment['reset_conditions']['initial_positions']))
            if du > epsilon:
                self._currently_resetting = True
                self._pub.publish(self.get_trajectory_message(self.environment['reset_conditions']['initial_positions']))
                time.sleep(3)

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        # self._pub_rand_light.publish()
        # self._pub_rand_sky.publish()
        # self._pub_rand_physics.publish()
        # self._pub_rand_obstacles.publish()

        if self.reset_iter > self.rand_target_thresh:
            # print("goal is before randomize: ", self.realgoal)
            # print("resseting the iter and randomize target: ", self.reset_iter)
            self.reset_iter = 0
            self.randomizeTargetPose("target")
            # print("self.reset_iter after reset: ", self.reset_iter)
            with open("/tmp/rosrl/targets.txt", 'a') as out:
                out.write( str(self.realgoal) + '\n' )
        # self.randomizeTexture("obj")
        # self.randomizeSize("obj", "box")
        # print("self.reset_iter after reset: ", self.reset_iter)

        # common_path = self.assets_path + "/models/"
        # path_list = [common_path + "sdf/rubik_cube_random.sdf", common_path + "sdf/rubik_cube.sdf",
        #             common_path + "sdf/box.sdf", common_path + "/sdf/cylinder.sdf",
        #             common_path + "urdf/sphere.urdf"]
        # for pl in path_list:
        #     if pl == self.obj_path:
        #         path_list.remove(pl)
        # self.randomizeObjectType("obj", path_list)

        self.iterator = 0

        if self.reset_jnts is True:
            self._pub.publish(self.get_trajectory_message(self.environment['reset_conditions']['initial_positions']))

            # self.randomizeStartPose(-3.13, 3.13)
            # # Generate a new random start pose until it is not a colliding state
            # while self._collision_msg is not None:
            #     self.randomizeStartPose(-3.13, 3.13)

            if (self.slowness_unit == 'sec') or (self.slowness_unit is None):
                time.sleep(int(self.slowness))
            elif(self.slowness_unit == 'nsec'):
                time.sleep(int(self.slowness/1000000000)) # using nanoseconds
            else:
                print("Unrecognized unit. Please use sec or nsec.")

        # Take an observation
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        # self.reset_iter +=1

        # Return the corresponding observation
        return self.ob
