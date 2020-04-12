import numpy as np

pi = 3.1415926
theta1 = 0
theta2 = 0
theta3 = 0
theta5 = 0
theta6 = 0
theta7 = 0

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
oOF = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[1], [0], [0]]))))))

print(pOF)
