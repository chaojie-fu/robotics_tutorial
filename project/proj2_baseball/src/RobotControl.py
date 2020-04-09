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
    theta = [0.1, 0.1, 0.1, 0, 0.1, 0.1, 0.1, 0, 0]
    for t in range(1000):
        v_p = [[0], [-0.1], [0], [0], [0], [0]]
        Ja = np.linalg.inv(
            Jacobian.jacobian(theta[0], theta[1], theta[2], theta[4], theta[5], theta[6]))
        v_theta = np.dot(Ja, v_p)
        delta_theta = [v_theta[0][0] * delta_t,
                       v_theta[1][0] * delta_t,
                       v_theta[2][0] * delta_t,
                       0,
                       v_theta[3][0] * delta_t,
                       v_theta[4][0] * delta_t,
                       v_theta[5][0] * delta_t,
                       0,
                       0]
        theta = theta + delta_theta
        print(theta)
        traj.append([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], theta[6], theta[7], theta[8]])
    print(traj)
    return traj


def addDebugItems(robotId):
    # add any debug Items you like
    # p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[
    #                    0.5, 0.5, 0.5], parentObjectUniqueId=robotId, parentLinkIndex=3)
    pass
