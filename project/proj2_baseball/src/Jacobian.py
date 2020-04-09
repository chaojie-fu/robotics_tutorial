import numpy as np
import math


def jacobian(theta1, theta2, theta3, theta5, theta6, theta7, dx=1e-8):
    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    pOF_origin = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, [[0], [0], [0], [1]]))))))
    oOF_origin = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta1
    theta1temp = theta1 + dx

    WRA = np.array([[math.cos(theta1temp), -math.sin(theta1temp), 0], [math.sin(theta1temp), math.cos(theta1temp), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta1 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta1 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta2
    theta2temp = theta2 + dx

    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2temp), -math.sin(theta2temp)], [0, math.sin(theta2temp), math.cos(theta2temp)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta2 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta2 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta3
    theta3temp = theta3 + dx

    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3temp), math.sin(theta3temp)], [0, -math.sin(theta3temp), math.cos(theta3temp)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta3 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta3 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta5
    theta5temp = theta5 + dx

    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5temp), -math.sin(theta5temp)], [0, math.sin(theta5temp), math.cos(theta5temp)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta5 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta5 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta6
    theta6temp = theta6 + dx

    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6temp), -math.sin(theta6temp), 0], [math.sin(theta6temp), math.cos(theta6temp), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7), -math.sin(theta7)], [0, math.sin(theta7), math.cos(theta7)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta6 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta6 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    # pOF + partial theta7
    theta7temp = theta7 + dx

    WRA = np.array([[math.cos(theta1), -math.sin(theta1), 0], [math.sin(theta1), math.cos(theta1), 0], [0, 0, 1]])
    WPA = np.array([[0], [0], [215.2]])
    WtA = np.concatenate((WRA, WPA), axis=1)
    WTA = np.concatenate((WtA, [[0, 0, 0, 1]]), axis=0)

    ARB = np.array([[1, 0, 0], [0, math.cos(theta2), -math.sin(theta2)], [0, math.sin(theta2), math.cos(theta2)]])
    APB = np.array([[162.4], [0], [0]])
    AtB = np.concatenate((ARB, APB), axis=1)
    ATB = np.concatenate((AtB, [[0, 0, 0, 1]]), axis=0)

    BRC = np.array([[1, 0, 0], [0, math.cos(theta3), math.sin(theta3)], [0, -math.sin(theta3), math.cos(theta3)]])
    BPC = np.array([[-162.4], [0], [351]])
    BtC = np.concatenate((BRC, BPC), axis=1)
    BTC = np.concatenate((BtC, [[0, 0, 0, 1]]), axis=0)

    CRD = np.array([[1, 0, 0], [0, math.cos(theta5), -math.sin(theta5)], [0, math.sin(theta5), math.cos(theta5)]])
    CPD = np.array([[0], [0], [351.2]])
    CtD = np.concatenate((CRD, CPD), axis=1)
    CTD = np.concatenate((CtD, [[0, 0, 0, 1]]), axis=0)

    DRE = np.array([[math.cos(theta6), -math.sin(theta6), 0], [math.sin(theta6), math.cos(theta6), 0], [0, 0, 1]])
    DPE = np.array([[162.4], [0], [0]])
    DtE = np.concatenate((DRE, DPE), axis=1)
    DTE = np.concatenate((DtE, [[0, 0, 0, 1]]), axis=0)

    ERF = np.array([[1, 0, 0], [0, math.cos(theta7temp), -math.sin(theta7temp)], [0, math.sin(theta7temp), math.cos(theta7temp)]])
    EPF = np.array([[0], [0], [162.4]])
    EtF = np.concatenate((ERF, EPF), axis=1)
    ETF = np.concatenate((EtF, [[0, 0, 0, 1]]), axis=0)

    PO = np.array([[0], [0], [0], [1]])

    pOF_theta7 = np.dot(WTA, np.dot(ATB, np.dot(BTC, np.dot(CTD, np.dot(DTE, np.dot(ETF, PO))))))
    oOF_theta7 = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, np.dot(ERF, [[0], [0], [1]]))))))

    return [[(pOF_theta1[0][0] - pOF_origin[0][0])/dx, (pOF_theta2[0][0] - pOF_origin[0][0])/dx,
             (pOF_theta3[0][0] - pOF_origin[0][0])/dx, (pOF_theta5[0][0] - pOF_origin[0][0])/dx,
             (pOF_theta6[0][0] - pOF_origin[0][0])/dx, (pOF_theta7[0][0] - pOF_origin[0][0])/dx],

            [(pOF_theta1[1][0] - pOF_origin[1][0])/dx, (pOF_theta2[1][0] - pOF_origin[1][0])/dx,
             (pOF_theta3[1][0] - pOF_origin[1][0])/dx, (pOF_theta5[1][0] - pOF_origin[1][0])/dx,
             (pOF_theta6[1][0] - pOF_origin[1][0])/dx, (pOF_theta7[1][0] - pOF_origin[1][0])/dx],

            [(pOF_theta1[2][0] - pOF_origin[2][0])/dx, (pOF_theta2[2][0] - pOF_origin[2][0])/dx,
             (pOF_theta3[2][0] - pOF_origin[2][0])/dx, (pOF_theta5[2][0] - pOF_origin[2][0])/dx,
             (pOF_theta6[2][0] - pOF_origin[2][0])/dx, (pOF_theta7[2][0] - pOF_origin[2][0])/dx],

            [(oOF_theta1[0][0] - oOF_origin[0][0])/dx, (oOF_theta2[0][0] - oOF_origin[0][0])/dx,
             (oOF_theta3[0][0] - oOF_origin[0][0])/dx, (oOF_theta5[0][0] - oOF_origin[0][0])/dx,
             (oOF_theta6[0][0] - oOF_origin[0][0])/dx, (oOF_theta7[0][0] - oOF_origin[0][0])/dx],

            [(oOF_theta1[1][0] - oOF_origin[1][0])/dx, (oOF_theta2[1][0] - oOF_origin[1][0])/dx,
             (oOF_theta3[1][0] - oOF_origin[1][0])/dx, (oOF_theta5[1][0] - oOF_origin[1][0])/dx,
             (oOF_theta6[1][0] - oOF_origin[1][0])/dx, (oOF_theta7[1][0] - oOF_origin[1][0])/dx],

            [(oOF_theta1[2][0] - oOF_origin[2][0])/dx, (oOF_theta2[2][0] - oOF_origin[2][0])/dx,
             (oOF_theta3[2][0] - oOF_origin[2][0])/dx, (oOF_theta5[2][0] - oOF_origin[2][0])/dx,
             (oOF_theta6[2][0] - oOF_origin[2][0])/dx, (oOF_theta7[2][0] - pOF_origin[2][0])/dx]]


# an example
# Ja = jacobian(theta1=0.1, theta2=0.1, theta3=0.1, theta5=0.1, theta6=0.1, theta7=0.1)
# p = [[216.7], [24.7], [217.77], [0.0297], [-0.1962], [0.9801]]
#
# print(Ja)
# print('\n')
# delta_theta = [[0.01], [0.01], [0.01], [0.01], [0.01], [0.01]]
# delta_p = np.dot(Ja, delta_theta)
# print(delta_p)
# print('\n')
# print(p + delta_p)
