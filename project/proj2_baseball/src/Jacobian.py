import numpy as np


def jacobian(theta1, theta2, theta3, theta5, theta6, theta7):
    l2 = 162.4
    l31 = -162.4
    l32 = 351.0
    l4 = 351.2
    l5 = 162.4
    l6 = 162.4

    A1 = -l32 * np.sin(theta2)
    A2 = l2 * np.cos(theta2) * np.cos(theta3) + l31 * np.sin(theta2) * np.cos(theta3) + \
         l2 * np.sin(theta2) * np.sin(theta3) - l31 * np.sin(theta2) * np.sin(theta3)
    A3 = l2 * np.cos(theta2) * np.sin(theta3) + l31 * np.sin(theta2) * np.sin(theta3) - \
         l2 * np.sin(theta2) * np.sin(theta3) + l31 * np.sin(theta2) * np.cos(theta3)
    B2 = l32 * np.sin(theta3)
    B3 = -l32 * np.cos(theta3)
    D1 = np.sin(theta2) * np.cos(theta3) - np.cos(theta2) * np.sin(theta3)
    D2 = np.sin(theta2) * np.sin(theta3) + np.cos(theta2) * np.cos(theta3)

    A4 = A1 + D1 * l4
    A5 = A2 * np.cos(theta5) - l4 * np.cos(theta5) + A3 * np.sin(theta5)
    A6 = -A2 * np.sin(theta5) + l4 * np.sin(theta5) + A3 * np.cos(theta5)
    B5 = B2 * np.cos(theta5) + B3 * np.sin(theta5)
    B6 = -B2 * np.sin(theta5) + B3 * np.cos(theta5)
    E5 = -l4 * np.cos(theta5)
    E6 = l4 * np.sin(theta5)
    D3 = D1 * np.cos(theta5) + D2 * np.sin(theta5)
    D4 = -D1 * np.sin(theta5) + D2 * np.cos(theta5)

    A7 = A4 * np.cos(theta6) + A5 * np.sin(theta6) + D4 * l5 * np.sin(theta6)
    A8 = -A4 * np.sin(theta6) + A5 * np.cos(theta6) + D4 * l5 * np.cos(theta6)
    A9 = A6 - D3 * l5
    B7 = B5 * np.sin(theta6)
    B8 = B5 * np.cos(theta6)
    B9 = B6
    E7 = E5 * np.sin(theta6)
    E8 = E5 * np.cos(theta6)
    E9 = E6
    G1 = np.cos(theta6) + D3 * np.sin(theta6)
    G2 = -np.sin(theta6) + D3 * np.cos(theta6)
    G3 = D4
    H1 = np.cos(theta6)
    H2 = -np.sin(theta6)
    I1 = np.cos(theta6)
    I2 = -np.sin(theta6)

    Ja00 = A7 + G2 * l6
    Ja01 = B7
    Ja02 = E7 + H2 * l6
    Ja03 = I2 * l6
    Ja04 = 0
    Ja05 = 0

    Ja10 = A8 * np.cos(theta7) - G1 * l6 * np.cos(theta7) + A9 * np.sin(theta7)
    Ja11 = B8 * np.cos(theta7) + B9 * np.sin(theta7)
    Ja12 = -H1 * l6 * np.cos(theta7) + E8 * np.cos(theta7) + E9 * np.sin(theta7)
    Ja13 = -I1 * l6 * np.sin(theta7)
    Ja14 = 0
    Ja15 = 0

    Ja20 = -A8 * np.sin(theta7) + G1 * l6 * np.sin(theta7) + A9 * np.sin(theta7)
    Ja21 = -B8 * np.sin(theta7) + B9 * np.cos(theta7)
    Ja22 = H1 * l6 * np.sin(theta7) - E8 * np.sin(theta7) + E9 * np.sin(theta7)
    Ja23 = I1 * l6 * np.sin(theta7)
    Ja24 = 0
    Ja25 = 0

    Ja30 = G1
    Ja31 = 0
    Ja32 = H1
    Ja33 = I1
    Ja34 = 0
    Ja35 = 1

    Ja40 = G2 * np.cos(theta7) + G3 * np.sin(theta7)
    Ja41 = 0
    Ja42 = H2 * np.cos(theta7)
    Ja43 = I2 * np.cos(theta7)
    Ja44 = np.sin(theta7)
    Ja45 = 0

    Ja50 = -G2 * np.sin(theta7) + G3 * np.cos(theta7)
    Ja51 = 0
    Ja52 = -H2 * np.sin(theta7)
    Ja53 = -I2 * np.sin(theta7)
    Ja54 = np.cos(theta7)
    Ja55 = 0

    return [[Ja00, Ja01, Ja02, Ja03, Ja04, Ja05],
            [Ja10, Ja11, Ja12, Ja13, Ja14, Ja15],
            [Ja20, Ja21, Ja22, Ja23, Ja24, Ja25],
            [Ja30, Ja31, Ja32, Ja33, Ja34, Ja35],
            [Ja40, Ja41, Ja42, Ja43, Ja44, Ja45],
            [Ja50, Ja51, Ja52, Ja53, Ja54, Ja55]]
