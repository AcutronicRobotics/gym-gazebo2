from __future__ import print_function
import PyKDL as kdl
import urdf_parser_py.urdf as urdf

def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.
    :param filename: URDF file path
    """

    with open(filename) as urdfFile:
        return treeFromUrdfModel(urdf.URDF.from_xml_string(urdfFile.read()))

def treeFromParam():
    """
    Construct a PyKDL.Tree from an URDF in a ROS parameter.
    :param param: Parameter name, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_parameter_server())

def treeFromString(xml):
    """
    Construct a PyKDL.Tree from an URDF xml string.
    :param xml: URDF xml string, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_xml_string(xml))

def toKdlPose(pose):
    """
    Helper function that packages a pose structure containing orientation values (roll, pitch, yaw)
    and position values (x, y, z) into a KDL Frame.
    """
    if pose and pose.rpy and len(pose.rpy) == 3 and pose.xyz and len(pose.xyz) == 3:
        frame = kdl.Frame(
            kdl.Rotation.RPY(*pose.rpy),
            kdl.Vector(*pose.xyz))
    else:
        frame = kdl.Frame.Identity()

    return frame

def toKdlInertia(i):
    # kdl specifies the inertia in the reference frame of the link, the urdf
    # specifies the inertia in the inertia reference frame
    origin = toKdlPose(i.origin)
    inertia = i.inertia
    return origin.M * kdl.RigidBodyInertia(
        i.mass, origin.p,
        kdl.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz,
                              inertia.iyz))

def toKdlJoint(jnt):
    #  define a mapping for joints and kdl
    fixed = lambda j, F: kdl.Joint(j.name)
    rotational = lambda j, F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
    translational = lambda j, F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis),
                                           kdl.Joint.TransAxis)

    typeMap = {
        'fixed': fixed,
        'revolute': rotational,
        'continuous': rotational,
        'prismatic': translational,
        'floating': fixed,
        'planar': fixed,
        'unknown': fixed,
        }

    return typeMap[jnt.type](jnt, toKdlPose(jnt.origin))

def addChildrenToTree(robotModel, root, tree):
    """
    Helper function that adds children to a KDL tree.
    """

    # constructs the optional inertia
    inert = kdl.RigidBodyInertia(0)
    if root.inertial:
        inert = toKdlInertia(root.inertial)

    # constructs the kdl joint
    parentJointName, parentLinkName = robotModel.parent_map[root.name]
    parentJoint = robotModel.joint_map[parentJointName]

    # construct the kdl segment
    sgm = kdl.Segment(
        root.name,
        toKdlJoint(parentJoint),
        toKdlPose(parentJoint.origin),
        inert)

    # add segment to tree
    if not tree.addSegment(sgm, parentLinkName):
        return False

    if root.name not in robotModel.child_map:
        return True

    children = [robotModel.link_map[l] for (j, l) in robotModel.child_map[root.name]]

    # recurslively add all children
    for child in children:
        if not addChildrenToTree(robotModel, child, tree):
            return False

    return True

def treeFromUrdfModel(robotModel, quiet=False):
    """
    Construct a PyKDL.Tree from an URDF model from urdf_parser_python.

    :param robotModel: URDF xml string, ``str``
    :param quiet: If true suppress messages to stdout, ``bool``
    """

    root = robotModel.link_map[robotModel.get_root()]

    if root.inertial and not quiet:
        print("The root link %s has an inertia specified in the URDF, but KDL does not support a\
         root link with an inertia. As a workaround, you can add an extra dummy link to your URDF.\
         " % root.name)

    okay = True
    tree = kdl.Tree(root.name)

    #  add all children
    for _, child in robotModel.child_map[root.name]:
        if not addChildrenToTree(robotModel, robotModel.link_map[child], tree):
            okay = False
            break

    return (okay, tree)
