"""
MIGRATION TO ROS2 IN PROCESS.

Additional Env specific dependencies:

gazebo_ros_pkgs (merge from branch ros2_state required)
for now: https://github.com/nzlz/gazebo_ros_pkgs -b ros2_merge_state_time_cmds
"""

import gym
gym.logger.set_level(40) # hide warnings
import time
import os
import numpy as np
from gym import utils, spaces
from gym.utils import seeding
from gym_gazebo.utils import ut_gazebo, ut_generic, ut_launch, ut_mara, ut_math
import copy
import threading # Used for time locks to synchronize position data.

from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import ContactState
from gazebo_msgs.msg import ModelState, LinkState
from gazebo_msgs.msg import EntityState

from std_srvs.srv import Empty
from std_msgs.msg import String, Empty as stdEmpty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing scara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, Vector3, WrenchStamped

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

class GazeboMARAOrientCollisionEnv(gym.Env):
    """
    TODO, description.
    """
    def __init__(self):
        """
        Initialize the MARA environemnt
            NOTE: This environment uses ROS and interfaces.

            TODO: port everything to ROS 2 natively
        """
        # Manage command line args
        args = ut_generic.getArgsMARA()
        self.gzclient = args.gzclient
        self.real_speed = args.real_speed
        self.velocity = args.velocity

        # Launch mara in a new Process
        ut_launch.start_launch_servide_process(ut_launch.generate_launch_description_mara(self.gzclient, self.real_speed))
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
        self.reset_jnts = True
        self._collision_msg = None

        self._time_lock = threading.RLock()

        #############################
        #   Environment hyperparams
        #############################
        # target, where should the agent reach
        EE_POS_TGT = np.asmatrix([-0.40028, 0.095615, 0.72466]) # alex2
        EE_ROT_TGT = np.asmatrix([[0.79660969, -0.51571238,  0.31536287], [0.51531424,  0.85207952,  0.09171542], [-0.31601302,  0.08944959,  0.94452874]]) # original orientation
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])
        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])

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

        robot_namespace = ""
        pose = Pose()
        pose.position.x = EE_POS_TGT[0,0];
        pose.position.y = EE_POS_TGT[0,1];
        pose.position.z = EE_POS_TGT[0,2];
        pose.orientation.x = 0.0;
        pose.orientation.y= 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        reference_frame = "world"

        self.assets_path = os.path.join(get_package_share_directory('gazebo_domain_random'), 'assets')

        file_xml = open(self.assets_path + '/models/urdf/target_point.urdf' ,mode='r')
        model_xml = file_xml.read()
        file_xml.close()

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

        self._seed()

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
            # neither obj nor obstacle colliding with table
            if "obj::" not in message.collision1_name and "obstacle" not in message.collision1_name or "table_fixed_joint_lump__mara_work_area_link_collision_4" not in message.collision2_name:
                # neither obj colliding with obstacle and vice-versa nor objs colliding each other nor obstacles colliding each other
                if "obj::" not in (message.collision1_name and message.collision2_name) and "obstacle" not in (message.collision1_name and message.collision2_name):
                    self._collision_msg = message

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

        model_file = ut_generic.getModelFileType(self.obj_path)
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


    def randomizeObjectType(self, current_obj_name=None, list_obj=None, replace=None):
        # obj = ModelState()
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        req = GetEntityState.Request()
        req.name = 'target'
        req.reference_frame = ''

        obj = self.add_entity.call_async(req)
        rclpy.spin_until_future_complete(self.node, obj)

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

        ut_gazebo.spawnModel(self.node, current_obj_name, random_obj, obj.pose)

    def randomizeStartPose(self, lower, upper):
        self.environment['reset_conditions']['initial_positions'] = [ np.random.uniform(lower,upper), np.random.uniform(lower,upper),np.random.uniform(lower,upper), np.random.uniform(lower,upper),np.random.uniform(lower,upper),np.random.uniform(lower,upper) ]
        self._pub.publish(ut_mara.get_trajectory_message(
            self.environment['reset_conditions']['initial_positions'],
            self.environment['joint_order'],
            self.velocity))

    def randomizeTargetPose(self, obj_name, centerPoint=False):
        ms = ModelState()
        if not centerPoint:
            EE_POS_TGT = np.asmatrix([ round(np.random.uniform(-0.62713, -0.29082), 5), round(np.random.uniform(-0.15654, 0.15925), 5), self.realgoal[2] ])

            roll = 0.0
            pitch = 0.0
            yaw = np.random.uniform(-1.57, 1.57)
            q = tf.taitbryan.euler2quat(yaw, pitch, roll)
            EE_ROT_TGT = tf.quaternions.quat2mat(q)
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
                ut_gazebo.spawnModel(self.node, obj_name, self.obj_path, ms.pose)
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
        last_observations = ut_mara.process_observations(obs_message, self.environment)
        # # # Get Jacobians from present joint angles and KDL trees
        # # # The Jacobians consist of a 6x6 matrix getting its from from
        # # # (# joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.get_jacobians(last_observations, self.scara_chain.getNrOfJoints(), self.jac_solver)
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
            current_quaternion = tf.quaternions.mat2quat(rot) #[w, x, y ,z]
            tgt_quartenion = tf.quaternions.mat2quat(self.target_orientation)

            # A  = np.vstack([current_quaternion, np.ones(len(current_quaternion))]).T
            #quat_error = np.linalg.lstsq(A, tgt_quartenion)[0]

            quat_error = current_quaternion * tgt_quartenion.conjugate()
            rot_vec_err, _ = tf.quaternions.quat2axangle(quat_error)

            # convert quat to np arrays
            # quat_error = np.asarray(quat_error, dtype=np.quaternion).view((np.double, 4))

            # RK:  revisit this later, we only take one angle difference here!
            # angle_diff = 2 * np.arccos(np.clip(quat_error[..., 0], -1., 1.))

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
                          np.reshape(rot_vec_err, -1),
                          np.reshape(ee_velocities, -1),]
            return state

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

        # Execute "action"
        self._pub.publish(ut_mara.get_trajectory_message(
            action[:self.scara_chain.getNrOfJoints()],
            self.environment['joint_order'],
            self.velocity))
        # Wait until the action is finished.
        self.wait_for_action(action)

        # # Take an observation
        # TODO: program this better, check that ob is not None, etc.
        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()

        self.reward_dist = -ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints():(self.scara_chain.getNrOfJoints()+3)])
        # careful we have degrees now so we scale with
        orientation_scale = 0.1
        self.reward_orient = - orientation_scale * ut_math.rmse_func(self.ob[self.scara_chain.getNrOfJoints()+3:(self.scara_chain.getNrOfJoints()+6)])
        #scale here the orientation because it should not be the main bias of the reward, position should be

        if self._collision_msg is not None:
            if self._collision_msg.collision1_name is None:
                raise AttributeError("collision1_name is None")
            if self._collision_msg.collision2_name is None:
                raise AttributeError("collision2_name is None")

            self.reward = (self.reward_dist + self.reward_orient) * 5.0

            # ROS 2
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

        # Return the corresponding observations, rewards, etc.
        self._collision_msg = None
        return self.ob, self.reward, done, {}

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        if self.reset_iter > self.rand_target_thresh:
            # print("goal is before randomize: ", self.realgoal)
            # print("resseting the iter and randomize target: ", self.reset_iter)
            self.reset_iter = 0
            self.randomizeTargetPose("target")
            # print("self.reset_iter after reset: ", self.reset_iter)
            with open("/tmp/rosrl/targets.txt", 'a') as out:
                out.write( str(self.realgoal) + '\n' )

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