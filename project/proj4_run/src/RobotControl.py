import pybullet as p
import Helper
import time
import numpy as np
from MPC_controller import Walk


def loadRobot(initPos):
    robotName = 'MarioTheRobot.urdf'
    initOrn = [1.0, 0.0, 0.0, 0.0]
    return p.loadURDF(Helper.findURDF(robotName), initPos, initOrn)


def generateTraj(robotId):
    # work in this function to make a plan before actual control
    # the output can be in any data structure you like
    steps = 1000

    fy = []
    theta = []
    x = []

    for i in range(steps):
        fy.append(np.pi / 2)
        theta.append(-np.pi / 4)
        x.append(0)

    # plan = [reference_theta, reference_x, reference_fy
    plan = [np.array(theta), np.array(x), np.array(fy)]
    # plan = AnswerByTA.generateTraj(robotId)
    return plan


def realTimeControl(robotId, plan, count):
    # work in this function to calculate real time control signal
    # the output should be a list of two float
    start = time.time()

    plan = np.transpose(plan)
    reference = plan[count: count+5]
    reference = np.transpose(reference)
    state = getstate(robotId)
    mpc = Walk()
    controlSignal = mpc.Solve(state, reference)
    # controlSignal = [0, 0]
    if count % 10 == 0:
        print('Count   ', count)
        print('reference', reference)
        print('state', state)
        print('M1, M2', controlSignal)
        end = time.time()
        print('time', end - start)
        print()
    # controlSignal = AnswerByTA.realTimeControl(robotId, plan)
    count += 1
    return controlSignal


def addDebugItems(robotId):
    # work in this function to add any debug visual items you need
    pass


def getstate(robotId):
    name = ['engine1', 'engine2']
    motors = []
    jointState_0 = []

    for jointId in range(p.getNumJoints(robotId)):
        jointName = p.getJointInfo(robotId, jointId)[1]
        if jointName.decode() in name:
            motors.append(jointId)
    jointState_0 = p.getJointState(robotId, motors[0])

    pos_wheel = p.getLinkState(robotId, 5, 1)
    pos_leg = p.getLinkState(robotId, 4, 1)
    Euler_leg = p.getEulerFromQuaternion(pos_leg[1])

    theta = Euler_leg[1]
    v_theta = pos_leg[7][1]
    fy = jointState_0[0]
    v_fy = jointState_0[1]
    x = pos_wheel[0][0]
    v_x = pos_wheel[6][0]

    return [theta, fy, x, v_theta, v_fy, v_x]