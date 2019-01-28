from gym_gazebo2.utils import ut_generic
from gazebo_msgs.srv import SpawnEntity
import rclpy

def spawnModel(node, obj_name, obj_path, pose):
    model_file = ut_generic.getModelFileType(obj_path)
    obj_file = open(obj_path, mode='r')
    xml = obj_file.read()
    obj_file.close()

    # create a new SpawnEntity client
    add_entity = node.create_client(SpawnEntity, '/spawn_entity')

    while not add_entity.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = SpawnEntity.Request()
    req.name = obj_name
    req.xml = xml
    req.robot_namespace = ""
    req.initial_pose = pose
    req.reference_frame = "world"

    future = add_entity.call_async(req)
    rclpy.spin_until_future_complete(node, future)
