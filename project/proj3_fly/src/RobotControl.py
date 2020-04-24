import pybullet as p
import numpy as np
from scipy.optimize import minimize
import math


def generateTraj(robotId):
    # work in this function to make a plan before actual control
    # the output can be in any data structure you like
    t = []
    t_step = 1 / 240.0
    n_max = 480
    for i in range(n_max + 1):
        t.append(i * t_step)

    plan = []
    v = 10
    pi = 3.14159
    for i in range(n_max + 1):
        x = -16.0 + i * v * t_step
        y = 0.0
        theta = pi / 12.0
        v_x = v
        v_y = 0
        v_theta = 0
        plan.append([x, v_x, y, v_y, theta, v_theta])
    return plan


def realTimeControl(robotId, plan, n, real_state, real_u):
    # work in this function to calculate real time control signal
    # the output should be a list of two float

    # return u in format [u[0], u[1], real_state, real_u]
    u = optimal(robotId, plan, n, real_state, real_u)
    return u


def addDebugItems():
    # work in this function to add any debug visual items you need
    pass


def getCondition(robotId):
    linkState = []
    for j in range(5):
        position = p.getLinkState(robotId, j, 1)
        linkState.append(position[4])
        if j == 2:
            v_x = position[6][0]
            v_y = position[6][2]  # 世界坐标系中的z坐标
        if j == 4:
            v_theta = position[7][1]
    x = linkState[2][0]
    y = linkState[2][2]  # 世界坐标系中的z坐标
    if (linkState[3][0] - linkState[4][0]) != 0:
        theta = np.arctan((linkState[4][2] - linkState[3][2]) / (linkState[3][0] - linkState[4][0]))
    else:
        theta = math.pi / 2  # 当分母为零时，角度为pi/2
    if (linkState[4][0] - linkState[3][0]) > 0:
        theta = theta + math.pi  # 当四电机转到三电机前面时，即已经转过半圈了
    return [x, v_x, y, v_y, theta, v_theta]


def optimal(robotId, plan, n, real_state, real_u):
    t_step = 1 / 240.0
    m = 10
    iyy = 0.68
    b = 0.3
    g = 10
    Qi = 10
    Ri = 1
    para = tuple([n])

    def objective(u, args=para):
        # calculate objective function
        t_max = para[0]
        obj_error = 0
        obj_control = 0
        for i in range(t_max + 1):
            # plan state at i + 1 time point
            x_ref = np.array(plan[i + 1])

            # desired state at i + 1 time point
            x_real_state = real_state[i]
            x_desire = np.array([
                t_step * x_real_state[1] + x_real_state[0],
                t_step * np.sin(x_real_state[5]) / m * (u[0] + u[1]) + x_real_state[1],
                t_step * x_real_state[3] + x_real_state[2],
                t_step * (np.cos(x_real_state[5]) / m * (u[0] + u[1]) - g) + x_real_state[3],
                t_step * x_real_state[5] + x_real_state[4],
                t_step * b / iyy * (u[1] - u[0]) + x_real_state[5]
            ])
            obj_error = obj_error + np.linalg.norm(x_ref - x_desire)

        for i in range(t_max + 1):
            obj_control = obj_control + np.linalg.norm([real_u[i][0], real_u[i][1]])

        obj = obj_error * Qi + obj_control * Ri
        return obj

    def constraint1(u):
        return 100.0 - u[0]

    def constraint2(u):
        return u[0] + 100.0

    def constraint3(u):
        return 100.0 - u[1]

    def constraint4(u):
        return u[1] + 100.0

    # initial guesses
    n = 2
    u0 = np.zeros(2)
    u0[0] = 50.0
    u0[1] = 50.0

    # optimize
    con1 = {'type': 'ineq', 'fun': constraint1}
    con2 = {'type': 'ineq', 'fun': constraint2}
    con3 = {'type': 'ineq', 'fun': constraint3}
    con4 = {'type': 'ineq', 'fun': constraint4}
    cons = ([con1, con2, con3, con4])
    solution = minimize(objective, u0, method='SLSQP', constraints=cons)
    u = solution.x
    print(u)
    return u