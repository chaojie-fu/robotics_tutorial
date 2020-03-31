import pybullet as p
import os
import numpy as np
import Helper


def load():
    # work in the following section to load your robot
    # robotName = 'robotarm.urdf'
    # robotPath = os.path.join('project', 'proj2_baseball', 'rsc', robotName)
    # robotPath = Helper.findURDF(robotName)
    robotInitPos = [0.0, 0.0, 1.1]
    robotInitOrn = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("../rsc/robotarm/urdf/robotarm.urdf",
                         robotInitPos, robotInitOrn)
    return robotId


def generateTraj(robotId, ballPos, targetPos):
    # work in this section, generate your tarjectory as a second order list
    # e.g. traj = [[j_1(t1), j_2(t1), j_3(t1)], [j_1(t2), j_2(t2), j_3(t2)], [j_1(t3), j_2(t3), j_3(t3)], ...]
    # robotId is the Unique body index for your robot
    # ballPos is a list for the baseball position, like [x, y, z]
    # targetPos is a list for the target position, like [x, y, z]
    # do not use the inverse kinematics function of pybullet!!!!!!
    # The following code is a example for a very simple robot

    traj = []
    numJoints = p.getNumJoints(robotId)

    for i in range(numJoints):
        angle = [1.57, 0, 0, 0, 0, 1.57, 1.57, 0, 1.57]

    for i in range(100):
        traj.append(angle)

    return traj


def addDebugItems(robotId):
    # add any debug Items you like
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[
                       0.5, 0.5, 0.5], parentObjectUniqueId=robotId, parentLinkIndex=3)
    # pass
