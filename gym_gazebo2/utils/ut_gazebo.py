from gazebo_msgs.srv import SpawnEntity
import rclpy

def spawnModel(node, objName, objPath, pose):
    objFile = open(objPath, mode='r')
    xml = objFile.read()
    objFile.close()

    # create a new SpawnEntity client
    addEntity = node.create_client(SpawnEntity, '/spawn_entity')

    while not addEntity.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = SpawnEntity.Request()
    req.name = objName
    req.xml = xml
    req.robot_namespace = ""
    req.initial_pose = pose
    req.reference_frame = "world"

    future = addEntity.call_async(req)
    rclpy.spin_until_future_complete(node, future)

def getTargetSdf():
    modelXml = """<?xml version='1.0'?>
                    <sdf version='1.6'>
                      <model name='target'>
                        <link name='cylinder0'>
                          <pose frame=''>0 0 0 0 0 0</pose>
                          <inertial>
                            <pose frame=''>0 0 0 0 0 0</pose>
                            <mass>5</mass>
                            <inertia>
                              <ixx>1</ixx>
                              <ixy>0</ixy>
                              <ixz>0</ixz>
                              <iyy>1</iyy>
                              <iyz>0</iyz>
                              <izz>1</izz>
                            </inertia>
                          </inertial>
                          <gravity>1</gravity>
                          <velocity_decay/>
                          <self_collide>0</self_collide>
                          <enable_wind>0</enable_wind>
                          <kinematic>0</kinematic>
                          <visual name='cylinder0_visual'>
                            <pose frame=''>0 0 0 0 0 0</pose>
                            <geometry>
                              <sphere>
                                <radius>0.01</radius>
                              </sphere>
                            </geometry>
                            <material>
                              <script>
                                <name>Gazebo/Green</name>
                                <uri>file://media/materials/scripts/gazebo.material</uri>
                              </script>
                              <shader type='pixel'/>
                            </material>
                            <transparency>0.1</transparency>
                            <cast_shadows>1</cast_shadows>
                          </visual>
                        </link>
                        <static>1</static>
                        <allow_auto_disable>1</allow_auto_disable>
                      </model>
                    </sdf>"""
    return modelXml
