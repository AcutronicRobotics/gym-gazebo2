from __future__ import print_function

import urdf_parser_py.urdf as urdf

import PyKDL as kdl

def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.
    :param filename: URDF file path
    """

    with open(filename) as urdf_file:
        return treeFromUrdfModel(urdf.URDF.from_xml_string(urdf_file.read()))

def treeFromParam(param):
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

def _toKdlPose(pose):
    """
    Helper function that packages a pose structure containing orientation values (roll, pitch, yaw)
    and position values (x, y, z) into a KDL Frame.
    """
    if pose and pose.rpy and len(pose.rpy) == 3 and pose.xyz and len(pose.xyz) == 3:
        return kdl.Frame(
              kdl.Rotation.RPY(*pose.rpy),
              kdl.Vector(*pose.xyz))
    else:
        return kdl.Frame.Identity()

def _toKdlInertia(i):
    # kdl specifies the inertia in the reference frame of the link, the urdf
    # specifies the inertia in the inertia reference frame
    origin = _toKdlPose(i.origin)
    inertia = i.inertia
    return origin.M * kdl.RigidBodyInertia(
            i.mass, origin.p,
            kdl.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz));

def _toKdlJoint(jnt):
    #  define a mapping for joints and kdl
    fixed = lambda j,F: kdl.Joint(j.name)
    rotational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
    translational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.TransAxis)

    type_map = {
            'fixed': fixed,
            'revolute': rotational,
            'continuous': rotational,
            'prismatic': translational,
            'floating': fixed,
            'planar': fixed,
            'unknown': fixed,
            }

    return type_map[jnt.type](jnt, _toKdlPose(jnt.origin))

def _add_children_to_tree(robot_model, root, tree):
    """
    Helper function that adds children to a KDL tree.
    """

    # constructs the optional inertia
    inert = kdl.RigidBodyInertia(0)
    if root.inertial:
        inert = _toKdlInertia(root.inertial)

    # constructs the kdl joint
    parent_joint_name, parent_link_name = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]

    # construct the kdl segment
    sgm = kdl.Segment(
        root.name,
        _toKdlJoint(parent_joint),
        _toKdlPose(parent_joint.origin),
        inert)

    # add segment to tree
    if not tree.addSegment(sgm, parent_link_name):
        return False

    if root.name not in robot_model.child_map:
        return True

    children = [robot_model.link_map[l] for (j,l) in robot_model.child_map[root.name]]

    # recurslively add all children
    for child in children:
        if not _add_children_to_tree(robot_model, child, tree):
            return False

    return True;

def treeFromUrdfModel(robot_model, quiet=False):
    """
    Construct a PyKDL.Tree from an URDF model from urdf_parser_python.

    :param robot_model: URDF xml string, ``str``
    :param quiet: If true suppress messages to stdout, ``bool``
    """

    root = robot_model.link_map[robot_model.get_root()]

    if root.inertial and not quiet:
        print("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." % root.name);

    ok = True
    tree = kdl.Tree(root.name)

    #  add all children
    for joint,child in robot_model.child_map[root.name]:
        if not _add_children_to_tree(robot_model, robot_model.link_map[child], tree):
            ok = False
            break

    return (ok, tree)
