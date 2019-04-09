import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyKDL import Jacobian, JntArray # For KDL Jacobians

def processObservations(message, agent):
    """
    Helper fuinction to convert a ROS message to joint angles and velocities.
    Check for and handle the case where a message is either malformed
    or contains joint values in an order different from that expected observation_callback
    in hyperparams['jointOrder']
    """
    if not message:
        print("Message is empty")
        return None
    else:
        # # Check if joint values are in the expected order and size.
        if message.joint_names != agent['jointOrder']:
            # Check that the message is of same size as the expected message.
            if len(message.joint_names) != len(agent['jointOrder']):
                raise Exception

        return np.array(message.actual.positions) # + message.actual.velocities

def getJacobians(state, numberOfJoints, jacSolver):
    """
    Produce a Jacobian from the urdf that maps from joint angles to x, y, z.
    This makes a 6x6 matrix from 6 joint angles to x, y, z and 3 angles.
    The angles are roll, pitch, and yaw (not Euler angles) and are not needed.
    Returns a repackaged Jacobian that is 3x6.
    """
    # Initialize a Jacobian for self.scara_chain.getNrOfJoints() joint angles by 3 cartesian coords
    # and 3 orientation angles
    jacobian = Jacobian(numberOfJoints)
    # Initialize a joint array for the present self.scara_chain.getNrOfJoints() joint angles.
    angles = JntArray(numberOfJoints)
    # Construct the joint array from the most recent joint angles.
    for i in range(numberOfJoints):
        angles[i] = state[i]
    # Update the jacobian by solving for the given angles.observation_callback
    jacSolver.JntToJac(angles, jacobian)
    # Initialize a numpy array to store the Jacobian.
    jac = np.array([[jacobian[i, j] for j in range(jacobian.columns())] \
        for i in range(jacobian.rows())])
    # Only want the cartesian position, not Roll, Pitch, Yaw (RPY) Angles
    eeJacobians = jac
    return eeJacobians

def getEePointsJacobians(refJacobian, eePoints, refRot, numberOfJoints):
    """
    Get the jacobians of the points on a link given the jacobian for that link's origin
    :param refJacobian: 6 x 6 numpy array, jacobian for the link's origin
    :param eePoints: N x 3 numpy array, points' coordinates on the link's coordinate system
    :param refRot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
    :return: 3N x 6 Jac_trans, each 3 x 6 numpy array is the Jacobian[:3, :] for that point
             3N x 6 Jac_rot, each 3 x 6 numpy array is the Jacobian[3:, :] for that point
    """
    eePoints = np.asarray(eePoints)
    refJacobiansTrans = refJacobian[:3, :]
    refJacobiansRot = refJacobian[3:, :]
    endEffectorPointsRot = np.expand_dims(refRot.dot(eePoints.T).T, axis=1)
    eePointsJacTrans = np.tile(refJacobiansTrans, (eePoints.shape[0], 1)) + \
                                np.cross(refJacobiansRot.T, endEffectorPointsRot).transpose(
                                    (0, 2, 1)).reshape(-1, numberOfJoints)
    eePointsJacRot = np.tile(refJacobiansRot, (eePoints.shape[0], 1))
    return eePointsJacTrans, eePointsJacRot

def getEePointsVelocities(refJacobian, eePoints, refRot, jointVelocities):
    """
    Get the velocities of the points on a link
    :param refJacobian: 6 x 6 numpy array, jacobian for the link's origin
    :param eePoints: N x 3 numpy array, points' coordinates on the link's coordinate system
    :param refRot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
    :param jointVelocities: 1 x 6 numpy array, joint velocities
    :return: 3N numpy array, velocities of each point
    """
    refJacobiansTrans = refJacobian[:3, :]
    refJacobiansRot = refJacobian[3:, :]
    eeVelocitiesTrans = np.dot(refJacobiansTrans, jointVelocities)
    eeVelocitiesRot = np.dot(refJacobiansRot, jointVelocities)
    eeVelocities = eeVelocitiesTrans + np.cross(eeVelocitiesRot.reshape(1, 3),
                                                refRot.dot(eePoints.T).T)
    return eeVelocities.reshape(-1)

def getTrajectoryMessage(action, jointOrder, velocity):
    """
    Helper function.
    Wraps an action vector of joint angles into a JointTrajectory message.
    Velocity must be set now. Duration (self.slowness) does not control velocity now.
    """
    # Set up a trajectory message to publish.
    actionMsg = JointTrajectory()
    actionMsg.joint_names = jointOrder
    # Create a point to tell the robot to move to.
    target = JointTrajectoryPoint()
    actionFloat = [float(i) for i in action]
    target.positions = actionFloat
    target.velocities = [velocity]*action.size

    target.time_from_start.nanosec = 1000000

    actionMsg.points = [target]
    return actionMsg

def positionsMatch(action, lastObservation):
    """
    Compares a given action with the observed position.
    Returns: bool. True if the position is final, False if not.
    """
    acceptedError = 0.01
    for i in range(action.size -1): #lastObservation loses last pose
        if abs(action[i] - lastObservation[i]) > acceptedError:
            return False
    return True
