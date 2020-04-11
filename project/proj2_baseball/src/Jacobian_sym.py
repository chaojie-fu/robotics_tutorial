import sympy as sy
from sympy import *
import numpy as np


def jacobian_sym(t1, t2, t3, t5, t6, t7):
    theta1, theta2, theta3, theta5, theta6, theta7 = sy.symbols('theta1 theta2 theta3 theta5 theta6 theta7')
    t1dot, t2dot, t3dot, t5dot, t6dot, t7dot = sy.symbols('t1dot t2dot t3dot t5dot t6dot t7dot')

    WRA = Matrix([[sy.cos(theta1), -sy.sin(theta1), 0], [sy.sin(theta1), sy.cos(theta1), 0], [0, 0, 1]])
    ARB = Matrix([[1, 0, 0], [0, sy.cos(theta2), -sy.sin(theta2)], [0, sy.sin(theta2), sy.cos(theta2)]])
    BRC = Matrix([[1, 0, 0], [0, sy.cos(theta3), sy.sin(theta3)], [0, -sy.sin(theta3), sy.cos(theta3)]])
    CRD = Matrix([[1, 0, 0], [0, sy.cos(theta5), -sy.sin(theta5)], [0, sy.sin(theta5), sy.cos(theta5)]])
    DRE = Matrix([[sy.cos(theta6), -sy.sin(theta6), 0], [sy.sin(theta6), sy.cos(theta6), 0], [0, 0, 1]])
    ERF = Matrix([[1, 0, 0], [0, sy.cos(theta7), -sy.sin(theta7)], [0, sy.sin(theta7), sy.cos(theta7)]])
    WRF = WRA * ARB * BRC * CRD * DRE * ERF

    l1 = 215.2
    l2 = 162.4
    l31 = -162.4
    l32 = 351.0
    l4 = 351.2
    l5 = 162.4
    l6 = 162.4

    vw = Matrix([[0], [0], [0]])
    ww = Matrix([[0], [0], [0]])

    va = WRA.T * (vw + ww.cross(Matrix([[0], [0], [l1]])))
    wa = WRA.T * ww + Matrix([[0], [0], [t1dot]])

    vb = ARB.T * (va + wa.cross(Matrix([[l2], [0], [0]])))
    wb = ARB.T * wa + Matrix([[t2dot], [0], [0]])

    vc = BRC.T * (vb + wb.cross(Matrix([[l31], [0], [l32]])))
    wc = BRC.T * wb + Matrix([[t3dot], [0], [0]])

    vd = CRD.T * (vc + wc.cross(Matrix([[0], [0], [l4]])))
    wd = CRD.T * wc + Matrix([[t5dot], [0], [0]])

    ve = DRE.T * (vd + wd.cross(Matrix([[l5], [0], [0]])))
    we = DRE.T * wd + Matrix([[0], [0], [t6dot]])

    vf = ERF.T * (ve + we.cross(Matrix([[0], [0], [l6]])))
    wf = ERF.T * we + Matrix([[t7dot], [0], [0]])

    vfn = vf.subs([(theta1, t1), (theta2, t2), (theta3, t3), (theta5, t5), (theta6, t6), (theta7, t7)])
    wfn = wf.subs([(theta1, t1), (theta2, t2), (theta3, t3), (theta5, t5), (theta6, t6), (theta7, t7)])

    Ja00 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja00 = Ja00.det()
    Ja01 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja01 = Ja01.det()
    Ja02 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja02 = Ja02.det()
    Ja03 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja03 = Ja03.det()
    Ja04 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja04 = Ja04.det()
    Ja05 = (Matrix([[1, 0, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja05 = Ja05.det()

    Ja10 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja10 = Ja10.det()
    Ja11 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja11 = Ja11.det()
    Ja12 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja12 = Ja12.det()
    Ja13 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja13 = Ja13.det()
    Ja14 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja14 = Ja14.det()
    Ja15 = (Matrix([[0, 1, 0]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja15 = Ja15.det()

    Ja20 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja20 = Ja20.det()
    Ja21 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja21 = Ja21.det()
    Ja22 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja22 = Ja22.det()
    Ja23 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja23 = Ja23.det()
    Ja24 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja24 = Ja24.det()
    Ja25 = (Matrix([[0, 0, 1]]) * vfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja25 = Ja25.det()

    Jav = np.array([[Ja00, Ja01, Ja02, Ja03, Ja04, Ja05],
                    [Ja10, Ja11, Ja12, Ja13, Ja14, Ja15],
                    [Ja20, Ja21, Ja22, Ja23, Ja24, Ja25]])

    Ja30 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja30 = Ja30.det()
    Ja31 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja31 = Ja31.det()
    Ja32 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja32 = Ja32.det()
    Ja33 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja33 = Ja33.det()
    Ja34 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja34 = Ja34.det()
    Ja35 = (Matrix([[1, 0, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja35 = Ja35.det()

    Ja40 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja40 = Ja40.det()
    Ja41 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja41 = Ja41.det()
    Ja42 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja42 = Ja42.det()
    Ja43 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja43 = Ja43.det()
    Ja44 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja44 = Ja44.det()
    Ja45 = (Matrix([[0, 1, 0]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja45 = Ja45.det()

    Ja50 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 1), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja50 = Ja50.det()
    Ja51 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 1), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja51 = Ja51.det()
    Ja52 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 1), (t5dot, 0), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja52 = Ja52.det()
    Ja53 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 1), (t6dot, 0), (t7dot, 0)])).evalf()
    Ja53 = Ja53.det()
    Ja54 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 1), (t7dot, 0)])).evalf()
    Ja54 = Ja54.det()
    Ja55 = (Matrix([[0, 0, 1]]) * wfn.subs(
        [(t1dot, 0), (t2dot, 0), (t3dot, 0), (t5dot, 0), (t6dot, 0), (t7dot, 1)])).evalf()
    Ja55 = Ja55.det()

    Jaw = np.array([[Ja30, Ja31, Ja32, Ja33, Ja34, Ja35],
                    [Ja40, Ja41, Ja42, Ja43, Ja44, Ja45],
                    [Ja50, Ja51, Ja52, Ja53, Ja54, Ja55]])

    WRA = np.array([[np.cos(t1), -np.sin(t1), 0], [np.sin(t1), np.cos(t1), 0], [0, 0, 1]])
    ARB = np.array([[1, 0, 0], [0, np.cos(t2), -np.sin(t2)], [0, np.sin(t2), np.cos(t2)]])
    BRC = np.array([[1, 0, 0], [0, np.cos(t3), np.sin(t3)], [0, -np.sin(t3), np.cos(t3)]])
    CRD = np.array([[1, 0, 0], [0, np.cos(t5), -np.sin(t5)], [0, np.sin(t5), np.cos(t5)]])
    DRE = np.array([[np.cos(t6), -np.sin(t6), 0], [np.sin(t6), np.cos(t6), 0], [0, 0, 1]])
    ERF = np.array([[1, 0, 0], [0, np.cos(t7), -np.sin(t7)], [0, np.sin(t7), np.cos(t7)]])
    WRF = np.dot(WRA, np.dot(ARB, np.dot(BRC, np.dot(CRD, np.dot(DRE, ERF)))))

    Jav = np.dot(WRF, Jav)
    Jaw = np.dot(WRF, Jaw)

    Ja = np.concatenate((Jav, Jaw))
    return [[Ja00, Ja01, Ja02, Ja03, Ja04, Ja05],
            [Ja10, Ja11, Ja12, Ja13, Ja14, Ja15],
            [Ja20, Ja21, Ja22, Ja23, Ja24, Ja25],
            [Ja30, Ja31, Ja32, Ja33, Ja34, Ja35],
            [Ja40, Ja41, Ja42, Ja43, Ja44, Ja45],
            [Ja50, Ja51, Ja52, Ja53, Ja54, Ja55]]
