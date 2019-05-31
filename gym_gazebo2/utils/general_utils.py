""" This file defines general utility functions and classes. """
import math
import numpy as np

# import sys
#
# sys.path.append('/home/rkojcev/ros_python3/devel/lib')
import PyKDL as kdl

class BundleType():
    """
    This class bundles many fields, similar to a record or a mutable
    namedtuple.
    """
    def __init__(self, variables):
        for var, val in variables.items():
            object.__setattr__(self, var, val)

    # Freeze fields so new ones cannot be set.
    def __setattr__(self, key, value):
        if not hasattr(self, key):
            raise AttributeError("%r has no attribute %s" % (self, key))
        object.__setattr__(self, key, value)


def checkShape(value, expectedShape, name=''):
    """
    Throws a ValueError if value.shape != expectedShape.
    Args:
        value: Matrix to shape check.
        expectedShape: A tuple or list of integers.
        name: An optional name to add to the exception message.
    """
    if value.shape != tuple(expectedShape):
        raise ValueError('Shape mismatch %s: Expected %s, got %s' %
                         (name, str(expectedShape), str(value.shape)))


def finiteDifferences(func, inputs, funcOutputShape=(), epsilon=1e-5):
    """
    Computes gradients via finite differences.
    derivative = (func(x+epsilon) - func(x-epsilon)) / (2*epsilon)
    Args:
        func: Function to compute gradient of. Inputs and outputs can be
            arbitrary dimension.
        inputs: Vector value to compute gradient at.
        funcOutputShape: Shape of the output of func. Default is
            empty-tuple, which works for scalar-valued functions.
        epsilon: Difference to use for computing gradient.
    Returns:
        Gradient vector of each dimension of func with respect to each
        dimension of input.
    """
    gradient = np.zeros(inputs.shape+funcOutputShape)
    for idx, _ in np.ndenumerate(inputs):
        testInput = np.copy(inputs)
        testInput[idx] += epsilon
        objD1 = func(testInput)
        assert objD1.shape == funcOutputShape
        testInput = np.copy(inputs)
        testInput[idx] -= epsilon
        objD2 = func(testInput)
        assert objD2.shape == funcOutputShape
        diff = (objD1 - objD2) / (2 * epsilon)
        gradient[idx] += diff
    return gradient


def approxEqual(a01, b01, threshold=1e-5):
    """
    Return whether two numbers are equal within an absolute threshold.
    Returns:
        True if a01 and b01 are equal within threshold.
    """
    return np.all(np.abs(a01 - b01) < threshold)


def extractCondition(hyperparams, m01):
    """
    Pull the relevant hyperparameters corresponding to the specified
    condition, and return a new hyperparameter dictionary.
    """
    return {var: val[m01] if isinstance(val, list) else val
            for var, val in hyperparams.items()}


def getEePoints(offsets, eePos, eeRot):
    """
    Helper method for computing the end effector points given a
    position, rotation matrix, and offsets for each of the ee points.

    Args:
        offsets: N x 3 array where N is the number of points.
        eePos: 1 x 3 array of the end effector position.
        eeRot: 3 x 3 rotation matrix of the end effector.
    Returns:
        3 x N array of end effector points.
    """
    return np.asarray(eeRot.dot(offsets.T) + eePos.T)


def getPosition(tf1, target, source, time):
    """
    Utility function that uses tf to return the position of target
    relative to source at time
    tf1: Object that implements TransformListener
    target: Valid label corresponding to target link
    source: Valid label corresponding to source link
    time: Time given in TF's time structure of secs and nsecs
    """

    # Calculate the quaternion data for the relative position
    # between the target and source.
    # translation, rot = tf1.lookupTransform(target, source, time)
    position, _ = tf1.lookupTransform(source, target, time)
    position = np.asarray(position)

    return position


def getRotationMatrix(angle, direction, point=None):
    """Return matrix to rotate about axis defined by point and direction.

    >>> rot = rotation_matrix(math.pi/2, [0, 0, 1], [1, 0, 0])
    >>> np.allclose(np.dot(R, [0, 0, 0, 1]), [1, -1, 0, 1])
    True
    >>> angle = (random.random() - 0.5) * (2*math.pi)
    >>> direc = np.random.random(3) - 0.5
    >>> point = np.random.random(3) - 0.5
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(-angle, -direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> I = np.identity(4, np.float64)
    >>> np.allclose(I, rotation_matrix(math.pi*2, direc))
    True
    >>> np.allclose(2, np.trace(rotation_matrix(math.pi/2,
    ...                                               direc, point)))
    True

    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    # rotation matrix around unit vector
    rot = np.diag([cosa, cosa, cosa])
    rot += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    rot += np.array([[0.0, -direction[2], direction[1]],
                     [direction[2], 0.0, -direction[0]],
                     [-direction[1], direction[0], 0.0]])
    matrix = np.identity(4)
    matrix[:3, :3] = rot
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        matrix[:3, 3] = point - np.dot(rot, point)
    return matrix


def rotationFromMatrix(matrix):
    """Return rotation angle and axis from rotation matrix.

    >>> angle = (random.random() - 0.5) * (2*math.pi)
    >>> direc = np.random.random(3) - 0.5
    >>> point = np.random.random(3) - 0.5
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> angle, direc, point = rotationFromMatrix(R0)
    >>> R1 = rotation_matrix(angle, direc, point)
    >>> is_same_transform(R0, R1)
    True

    """
    rot = np.array(matrix, dtype=np.float64, copy=False)
    r33 = rot[:3, :3]
    # direction: unit eigenvector of r33 corresponding to eigenvalue of 1
    w00, w01 = np.linalg.eig(r33.T)
    i = np.where(abs(np.real(w00) - 1.0) < 1e-8)[0]
    if not i:
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    direction = np.real(w01[:, i[-1]]).squeeze()
    # point: unit eigenvector of r33 corresponding to eigenvalue of 1
    w00, q00 = np.linalg.eig(rot)
    i = np.where(abs(np.real(w00) - 1.0) < 1e-8)[0]
    if not i:
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    point = np.real(q00[:, i[-1]]).squeeze()
    point /= point[3]
    # rotation angle depending on direction
    cosa = (np.trace(r33) - 1.0) / 2.0
    if abs(direction[2]) > 1e-8:
        sina = (rot[1, 0] + (cosa-1.0)*direction[0]*direction[1]) / direction[2]
    elif abs(direction[1]) > 1e-8:
        sina = (rot[0, 2] + (cosa-1.0)*direction[0]*direction[2]) / direction[1]
    else:
        sina = (rot[2, 1] + (cosa-1.0)*direction[1]*direction[2]) / direction[0]
    angle = math.atan2(sina, cosa)
    return angle, direction, point

def quaternionFromMatrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q00 = quaternionFromMatrix(np.identity(4), True)
    >>> np.allclose(q, [1, 0, 0, 0])
    True
    >>> q00 = quaternionFromMatrix(np.diag([1, -1, -1, 1]))
    >>> np.allclose(q, [0, 1, 0, 0]) or np.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q00 = quaternionFromMatrix(R, True)
    >>> np.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q00 = quaternionFromMatrix(R)
    >>> np.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q00 = quaternionFromMatrix(R)
    >>> np.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q00 = quaternionFromMatrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> is_same_quaternion(quaternionFromMatrix(R, isprecise=False),
    ...                    quaternionFromMatrix(R, isprecise=True))
    True
    >>> R = euler_matrix(0.0, 0.0, np.pi/2.0)
    >>> is_same_quaternion(quaternionFromMatrix(R, isprecise=False),
    ...                    quaternionFromMatrix(R, isprecise=True))
    True

    """
    matrix = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q00 = np.empty((4, ))
        t00 = np.trace(matrix)
        if t00 > matrix[3, 3]:
            q00[0] = t00
            q00[3] = matrix[1, 0] - matrix[0, 1]
            q00[2] = matrix[0, 2] - matrix[2, 0]
            q00[1] = matrix[2, 1] - matrix[1, 2]
        else:
            i, j, k = 0, 1, 2
            if matrix[1, 1] > matrix[0, 0]:
                i, j, k = 1, 2, 0
            if matrix[2, 2] > matrix[i, i]:
                i, j, k = 2, 0, 1
            t00 = matrix[i, i] - (matrix[j, j] + matrix[k, k]) + matrix[3, 3]
            q00[i] = t00
            q00[j] = matrix[i, j] + matrix[j, i]
            q00[k] = matrix[k, i] + matrix[i, k]
            q00[3] = matrix[k, j] - matrix[j, k]
            q00 = q00[[3, 0, 1, 2]]
        q00 *= 0.5 / math.sqrt(t00 * matrix[3, 3])
    else:
        m00 = matrix[0, 0]
        m01 = matrix[0, 1]
        m02 = matrix[0, 2]
        m10 = matrix[1, 0]
        m11 = matrix[1, 1]
        m12 = matrix[1, 2]
        m20 = matrix[2, 0]
        m21 = matrix[2, 1]
        m22 = matrix[2, 2]
        # symmetric matrix k00
        k00 = np.array([[m00-m11-m22, 0.0, 0.0, 0.0],
                        [m01+m10, m11-m00-m22, 0.0, 0.0],
                        [m02+m20, m12+m21, m22-m00-m11, 0.0],
                        [m21-m12, m02-m20, m10-m01, m00+m11+m22]])
        k00 /= 3.0
        # quaternion is eigenvector of k00 that corresponds to largest eigenvalue
        w00, v00 = np.linalg.eigh(k00)
        q00 = v00[[3, 0, 1, 2], np.argmax(w00)]
    if q00[0] < 0.0:
        np.negative(q00, q00)

    # exchange (w, x, y, z) to (x, y, z, w)
    qNew = np.empty(4)
    qNew[:3] = q00[1:]
    qNew[3] = q00[0]
    return qNew

def jointListToKdl(q00):
    """ Return KDL JntArray converted from list q00 """
    if q00 is None:
        return None
    if isinstance(q00, np.matrix) and q00.shape[1] == 0:
        q00 = q00.T.tolist()[0]
    qKdl = kdl.JntArray(len(q00))
    for i, qi0 in enumerate(q00):
        qKdl[i] = qi0
    return qKdl

def jointKdlToList(q00):
    """ Return list converted from KDL JntArray"""
    if q00 is None:
        return None
    return [q00[i] for i in range(q00.rows())]


def forwardKinematics(robotChain, linkNames, q00, baseLink='base', endLink='ee_link'):
    """
    Perform forward kinematics
    Args:
        robotChain: robot's chain object
        linkNames: list of robot link names
        q00: list of joint positions
        baseLink: name of the link regarded as origin
        endLink: name of the link regarded as target
    Returns:
        translation vector and rotation matrix from endLink to baseLink

    """
    baseTrans = doKdlFk(robotChain, q00, linkNames.index(baseLink))
    if baseTrans is None:
        print("FK KDL failure on base transformation.")
    endTrans = doKdlFk(robotChain, q00, linkNames.index(endLink))
    if endTrans is None:
        print("FK KDL failure on end transformation.")
    pose = np.dot(np.linalg.inv(baseTrans), endTrans)
    pos = pose[:3, 3].reshape(1, 3)
    rot = pose[:3, :3]
    return pos, rot

def doKdlFk(robotChain, q00, linkNumber):
    endeffecFrame = kdl.Frame()
    fkKdl = kdl.ChainFkSolverPos_recursive(robotChain)
    kinematicsStatus = fkKdl.JntToCart(jointListToKdl(q00),
                                       endeffecFrame,
                                       linkNumber)
    if kinematicsStatus >= 0:
        p00 = endeffecFrame.p
        matrix = endeffecFrame.M
        return np.array([[matrix[0, 0], matrix[0, 1], matrix[0, 2], p00.x()],
                         [matrix[1, 0], matrix[1, 1], matrix[1, 2], p00.y()],
                         [matrix[2, 0], matrix[2, 1], matrix[2, 2], p00.z()],
                         [0, 0, 0, 1]])
    else:
        return None



def inverseKinematics(robotChain, pos, rot, qGuess=None, minJoints=None, maxJoints=None):
    """
    Perform inverse kinematics
    Args:
        robotChain: robot's chain object
        pos: 1 x 3 or 3 x 1 array of the end effector position.
        rot: 3 x 3 array of the end effector rotation
        qGuess: guess values for the joint positions
        minJoints: minimum value of the position for each joint
        maxJoints: maximum value of the position for each joint
    Returns:
        list of joint positions or None (no solution)
    """
    # print("inside inverse: ", pos, " ; ", rot)
    posKdl = kdl.Vector(pos[0], pos[1], pos[2])
    rotKdl = kdl.Rotation(rot[0, 0], rot[0, 1], rot[0, 2],
                          rot[1, 0], rot[1, 1], rot[1, 2],
                          rot[2, 0], rot[2, 1], rot[2, 2])
    frameKdl = kdl.Frame(rotKdl, posKdl)
    numJoints = robotChain.getNrOfJoints()
    minJoints = -np.pi * np.ones(numJoints) if minJoints is None else minJoints
    maxJoints = np.pi * np.ones(numJoints) if maxJoints is None else maxJoints
    minsKdl = jointListToKdl(minJoints)
    maxsKdl = jointListToKdl(maxJoints)
    fkKdl = kdl.ChainFkSolverPos_recursive(robotChain)
    ikVKdl = kdl.ChainIkSolverVel_pinv(robotChain)
    ikPKdl = kdl.ChainIkSolverPos_NR_JL(robotChain, minsKdl, maxsKdl,
                                        fkKdl, ikVKdl)

    if qGuess is None:
        # use the midpoint of the joint limits as the guess
        lowerLim = np.where(np.isfinite(minJoints), minJoints, 0.)
        upperLim = np.where(np.isfinite(maxJoints), maxJoints, 0.)
        qGuess = (lowerLim + upperLim) / 2.0
        qGuess = np.where(np.isnan(qGuess), [0.]*len(qGuess), qGuess)

    qKdl = kdl.JntArray(numJoints)
    qGuessKdl = jointListToKdl(qGuess)
    if ikPKdl.CartToJnt(qGuessKdl, frameKdl, qKdl) >= 0:
        return jointKdlToList(qKdl)
    else:
        return None
