import pybullet as p
import numpy as np
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
    pi = 3.14159265358979323846264338327950288419716939937510
    traj = []
    # numJoints = p.getNumJoints(robotId)
    Ball_x = ballPos[0]
    Ball_y = ballPos[1]

    # calculate the polar angle of ball
    if True:
        if Ball_y >= 0:
            if Ball_x == 0:
                Ball_theta = pi / 2
            elif Ball_x < 0:
                Ball_theta = np.arctan(Ball_y / Ball_x) + pi
            else:
                Ball_theta = np.arctan(Ball_y / Ball_x)
        else:
            if Ball_x == 0:
                Ball_theta = 3 * pi / 2
            elif Ball_x < 0:
                Ball_theta = np.arctan(Ball_y / Ball_x) + pi
            else:
                Ball_theta = np.arctan(Ball_y / Ball_x) + 2 * pi

    # set an set of initial angle in case of singularity

    theta = [Ball_theta + pi / 2, 0.0, 3 * pi / 4, 3 * pi / 4, pi / 2, 0]
    for i in range(240):
        traj.append([theta[0], theta[1] * i / 240, theta[2] * i / 240, 0, theta[3] * i / 240,
                     theta[4] * i / 240, theta[5] * i / 240, 0.0, 0.0])
    # move the catcher towards beyond the ball
    # set position step here

    step = 240

    if True:
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]
        theta5 = theta[3]
        theta6 = theta[4]
        theta7 = theta[5]

        WRA = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
        WPA = np.array([[0], [0], [215.2]])
        WtA = np.concatenate((WRA, WPA), axis=1)
        WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

        ARB = np.array([[1, 0, 0], [0, np.cos(theta2), -np.sin(theta2)], [0, np.sin(theta2), np.cos(theta2)]])
        APB = np.array([[162.4], [0], [0]])
        AtB = np.concatenate((ARB, APB), axis=1)
        ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

        BRC = np.array([[1, 0, 0], [0, np.cos(theta3), -np.sin(theta3)], [0, np.sin(theta3), np.cos(theta3)]])
        BPC = np.array([[-162.4], [0], [351]])
        BtC = np.concatenate((BRC, BPC), axis=1)
        BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

        CRD = np.array([[1, 0, 0], [0, np.cos(theta5), -np.sin(theta5)], [0, np.sin(theta5), np.cos(theta5)]])
        CPD = np.array([[0], [0], [351.2]])
        CtD = np.concatenate((CRD, CPD), axis=1)
        CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

        DRE = np.array([[np.cos(theta6), -np.sin(theta6), 0], [np.sin(theta6), np.cos(theta6), 0], [0, 0, 1]])
        DPE = np.array([[162.4], [0], [0]])
        DtE = np.concatenate((DRE, DPE), axis=1)
        DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

        ERF = np.array([[1, 0, 0], [0, np.cos(theta7), -np.sin(theta7)], [0, np.sin(theta7), np.cos(theta7)]])
        EPF = np.array([[0], [0], [162.4]])
        EtF = np.concatenate((ERF, EPF), axis=1)
        ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

        PO = np.array([[0], [0], [0], [1]])

        pOF = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO)))))) + [[0.0], [0.0], [1100.0], [0.0]]
        CurrentX = pOF[0][0] / 1000
        print('CurrentX: ')
        print(CurrentX)
        print('ballPos: ')
        print(ballPos[0])
        CurrentY = pOF[1][0] / 1000
        for i in range(step):
            delta_p = np.array([[(ballPos[0] - CurrentX) * 1000 / step], [(ballPos[1] - CurrentY) * 1000 / step], [0.0 * 1000 / step], [0.0 / 240.], [0.0 / 240.], [0.0 / 240.]])
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
            traj.append([theta[0], theta[1], theta[2], 0, theta[3], theta[4], theta[5], 0.0, 0.0])

    if True:
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]
        theta5 = theta[3]
        theta6 = theta[4]
        theta7 = theta[5]

        WRA = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
        WPA = np.array([[0], [0], [215.2]])
        WtA = np.concatenate((WRA, WPA), axis=1)
        WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

        ARB = np.array([[1, 0, 0], [0, np.cos(theta2), -np.sin(theta2)], [0, np.sin(theta2), np.cos(theta2)]])
        APB = np.array([[162.4], [0], [0]])
        AtB = np.concatenate((ARB, APB), axis=1)
        ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

        BRC = np.array([[1, 0, 0], [0, np.cos(theta3), -np.sin(theta3)], [0, np.sin(theta3), np.cos(theta3)]])
        BPC = np.array([[-162.4], [0], [351]])
        BtC = np.concatenate((BRC, BPC), axis=1)
        BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

        CRD = np.array([[1, 0, 0], [0, np.cos(theta5), -np.sin(theta5)], [0, np.sin(theta5), np.cos(theta5)]])
        CPD = np.array([[0], [0], [351.2]])
        CtD = np.concatenate((CRD, CPD), axis=1)
        CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

        DRE = np.array([[np.cos(theta6), -np.sin(theta6), 0], [np.sin(theta6), np.cos(theta6), 0], [0, 0, 1]])
        DPE = np.array([[162.4], [0], [0]])
        DtE = np.concatenate((DRE, DPE), axis=1)
        DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

        ERF = np.array([[1, 0, 0], [0, np.cos(theta7), -np.sin(theta7)], [0, np.sin(theta7), np.cos(theta7)]])
        EPF = np.array([[0], [0], [162.4]])
        EtF = np.concatenate((ERF, EPF), axis=1)
        ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

        PO = np.array([[0], [0], [0], [1]])

        pOF = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO)))))) + [[0.0], [0.0],
                                                                                                  [1100.0], [0.0]]
        CurrentX = pOF[0][0] / 1000
        print('CurrentX: ')
        print(CurrentX)
        print('ballPos: ')
        print(ballPos[0])
        CurrentY = pOF[1][0] / 1000
        for i in range(step):
            delta_p = np.array([[(ballPos[0] - CurrentX) * 1000 / step], [(ballPos[1] - CurrentY) * 1000 / step], [0.0 * 1000 / step], [0.0 / 240.], [0.0 / 240.], [0.0 / 240.]])
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
            traj.append([theta[0], theta[1], theta[2], 0, theta[3], theta[4], theta[5], 0.0, 0.0])

    if True:
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]
        theta5 = theta[3]
        theta6 = theta[4]
        theta7 = theta[5]

        WRA = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
        WPA = np.array([[0], [0], [215.2]])
        WtA = np.concatenate((WRA, WPA), axis=1)
        WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

        ARB = np.array([[1, 0, 0], [0, np.cos(theta2), -np.sin(theta2)], [0, np.sin(theta2), np.cos(theta2)]])
        APB = np.array([[162.4], [0], [0]])
        AtB = np.concatenate((ARB, APB), axis=1)
        ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

        BRC = np.array([[1, 0, 0], [0, np.cos(theta3), -np.sin(theta3)], [0, np.sin(theta3), np.cos(theta3)]])
        BPC = np.array([[-162.4], [0], [351]])
        BtC = np.concatenate((BRC, BPC), axis=1)
        BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

        CRD = np.array([[1, 0, 0], [0, np.cos(theta5), -np.sin(theta5)], [0, np.sin(theta5), np.cos(theta5)]])
        CPD = np.array([[0], [0], [351.2]])
        CtD = np.concatenate((CRD, CPD), axis=1)
        CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

        DRE = np.array([[np.cos(theta6), -np.sin(theta6), 0], [np.sin(theta6), np.cos(theta6), 0], [0, 0, 1]])
        DPE = np.array([[162.4], [0], [0]])
        DtE = np.concatenate((DRE, DPE), axis=1)
        DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

        ERF = np.array([[1, 0, 0], [0, np.cos(theta7), -np.sin(theta7)], [0, np.sin(theta7), np.cos(theta7)]])
        EPF = np.array([[0], [0], [162.4]])
        EtF = np.concatenate((ERF, EPF), axis=1)
        ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

        PO = np.array([[0], [0], [0], [1]])

        pOF = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO)))))) + [[0.0], [0.0],
                                                                                                  [1100.0], [0.0]]
        CurrentX = pOF[0][0] / 1000
        print('CurrentX: ')
        print(CurrentX)
        print('ballPos: ')
        print(ballPos[0])
        CurrentY = pOF[1][0] / 1000
        CurrentZ = pOF[2][0] / 1000
        for i in range(step):
            delta_p = np.array([[(ballPos[0] - CurrentX) * 1000 / step], [(ballPos[1] - CurrentY) * 1000 / step], [(1.25 - CurrentZ) * 1000 / step], [0.0 / 240.], [0.0 / 240.], [0.0 / 240.]])
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
            traj.append([theta[0], theta[1], theta[2], 0, theta[3], theta[4], theta[5], 0.0, 0.0])

    for i in range(100):
        traj.append([theta[0], theta[1], theta[2], 0, theta[3], theta[4], theta[5], pi / 2 * 0.70, pi / 2 * 0.70])

    for i in range(1000):
        delta_p = np.array([[0], [0], [10 / 240], [0.0 / 240.], [0.0 / 240.], [0.0 / 240.]])
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
        traj.append([theta[0], theta[1], theta[2], 0, theta[3], theta[4], theta[5], pi / 2 * 0.70, pi / 2 * 0.70])

    return traj


def addDebugItems(robotId):
    # add any debug Items you like
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[
                       0.5, 0.5, 0.5], parentObjectUniqueId=robotId, parentLinkIndex=6)
    pass
