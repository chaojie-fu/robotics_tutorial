import pybullet as p
import os
import numpy as np
import Helper
import Jacobian

def load():
    # work in the following section to load your robot
    # robotName = 'robotarm.urdf'
    # robotPath = os.path.join('project', 'proj2_baseball', 'rsc', robotName)
    # robotPath = Helper.findURDF(robotName)
    robotInitPos = [0.0, 0.0, 1.1]
    robotInitOrn = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("../rsc/robotarm/urdf/robotarm.urdf", robotInitPos, robotInitOrn,
                         useFixedBase=1)
    return robotId


def generateTraj(robotId, ballPos, targetPos):
    # work in this section, generate your tarjectory as a second order list
    # e.g. traj = [[j_1(t1), j_2(t1), j_3(t1)], [j_1(t2), j_2(t2), j_3(t2)], [j_1(t3), j_2(t3), j_3(t3)], ...]
    # robotId is the Unique body index for your robot
    # ballPos is a list for the baseball position, like [x, y, z]
    # targetPos is a list for the target position, like [x, y, z]
    # do not use the inverse kinematics function of pybullet!!!!!!
    # The following code is a example for a very simple robot

    # rotate in a consistent angular velocity
    traj = []
    numJoints = p.getNumJoints(robotId)
    delta_t = 1 / 2400
    # set an set of initial angle in case of singularity
    pi = 3.14159
    theta = [0.0, pi / 4, pi / 4, pi / 4, pi / 4, 0.0]
    for i in range(240):
        traj.append([theta[0] * i / 240, theta[1] * i / 240, theta[2] * i / 240, 0, theta[3] * i / 240,
                     theta[4] * i / 240, theta[5] * i / 240, -pi / 2, -pi / 2])
    for j in range(2000):
        # set position step here
        delta_p = np.array([[0.0 / 240.], [0.0 / 240.], [0.0 / 480.], [0.0 / 240.], [0.0 / 240.], [0.0 / 240.]])
        Ja = Jacobian.jacobian(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5])
        Ja = np.array(Ja, dtype='float')
        Jainv = np.linalg.inv(Ja)
        delta_theta = np.dot(Jainv, delta_p)
        theta = [theta[0] + delta_theta[0][0],
                 theta[1] + delta_theta[1][0],
                 theta[2] + delta_theta[2][0],
                 theta[3] + delta_theta[3][0],
                 theta[4] + delta_theta[4][0],
                 theta[5] + delta_theta[5][0]
                 ]
        traj.append([theta[0], theta[1], theta[2], 0, theta[3],
                         theta[4], theta[5], -pi / 2, -pi / 2])
    print(traj)
    return traj


def addDebugItems(robotId):
    # add any debug Items you like
    # p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[
    #                    0.5, 0.5, 0.5], parentObjectUniqueId=robotId, parentLinkIndex=3)
    pass
