import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

theta1 = 0
theta2 = 0
theta3 = 0
# theta4 = 0
theta5 = 0
theta6 = 0
theta7 = 0
pi = 3.14159265

limit = np.linspace(-pi, pi, 10)
Result = [[0], [0], [0], [0]]

for theta1 in limit:
    for theta2 in limit:
        for theta3 in limit:
            for theta5 in limit:
                for theta6 in limit:
                    for theta7 in limit:
                        WRA = np.array(
                            [[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0],
                             [0, 0, 1]])
                        WPA = np.array([[0], [0], [215.2]])
                        WtA = np.concatenate((WRA, WPA), axis=1)
                        WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

                        ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)],
                                        [0, math.sin(theta2), math.cos(theta2)]])
                        APB = np.array([[162.4], [0], [0]])
                        AtB = np.concatenate((ARB, APB), axis=1)
                        ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

                        BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)],
                                        [0, -math.sin(theta3), math.cos(theta3)]])
                        BPC = np.array([[-162.4], [0], [351]])
                        BtC = np.concatenate((BRC, BPC), axis=1)
                        BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

                        CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)],
                                        [0, math.sin(theta5), math.cos(theta5)]])
                        CPD = np.array([[0], [0], [351.2]])
                        CtD = np.concatenate((CRD, CPD), axis=1)
                        CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

                        DRE = np.array(
                            [[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0],
                             [0, 0, 1]])
                        DPE = np.array([[162.4], [0], [0]])
                        DtE = np.concatenate((DRE, DPE), axis=1)
                        DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

                        ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)],
                                        [0, math.sin(theta7), math.cos(theta7)]])
                        EPF = np.array([[0], [0], [162.4]])
                        EtF = np.concatenate((ERF, EPF), axis=1)
                        ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

                        PO = np.array([[0], [0], [0], [1]])

                        pOF = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
                        Result = np.concatenate((Result, pOF), axis=1)


x = Result[0, :]
y = Result[1, :]
z = Result[2, :]

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)

ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
plt.show()
