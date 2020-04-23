import pybullet as p
# import AnswerByTA


def generateTraj(robotId):
    # work in this function to make a plan before actual control
    # the output can be in any data structure you like
    t = []
    t_step = 1 / 240.0
    n = 4800
    for i in range(n):
        t.append(i * t_step)
    [u1, u2, x, y, theta, v_x, v_y, v_theta] = GlobalTraj()
    plan = []
    # generate plan in format like below:
    # [[x[0], y[0], theta[0], v_x[0], v_y[0], v_theta[0], u1[0]],
    #  [x[1], y[1], theta[1], v_x[1], v_y[1], v_theta[1], u2[0]],
    #  ......
    #  [x[n - 1], y[n - 1], theta[n - 1], v_x[n - 1], v_y[n - 1], v_theta[n - 1], u1[n - 1], u2[n - 1]]
    # ]
    for i in range(n):
        plan.append([x[i], y[i], theta[i], v_x[i], v_y[i], v_theta[i], u1[i], u2[i]])
    return plan


def realTimeControl(robotId, plan, n):
    # work in this function to calculate real time control signal
    # the output should be a list of two float
    # n represents current time step (starts with 0)

    # calculate Global trajectory and control output
    Global = plan
    # get current state
    ref = getCondition(robotId)

    uff1 = Global[0][n]
    uff2 = Global[1][n]

    ufe = ErrorControl(Global[2][n] - ref[0], Global[3][n] - ref[1], Global[4][n] - ref[2],
                       Global[5][n] - ref[3], Global[6][n] - ref[4], Global[7][n] - ref[5])
    ufe1 = ufe[0]
    ufe2 = ufe[1]

    u1 = uff1 + ufe1
    u2 = uff2 + ufe2
    controlSignal = [u1, u2]
    # controlSignal = AnswerByTA.realTimeControl(robotId, plan)
    return controlSignal


def addDebugItems():
    # work in this function to add any debug visual items you need
    pass


def GlobalTraj():
    # time line
    # change t_step and n to change time line
    t = []
    t_step = 1 / 240.0
    n = 4800
    for i in range(n):
        t.append(i * t_step)
    # generate global trajectory and control output here using optimal method


    return [u1, u2, x, y, theta, v_x, v_y, v_theta]


def ErrorControl(e_x, e_y, e_theta, e_v_x, e_v_y, e_v_theta, e_u1, e_u2):
    # generate feedback control output

    return [ufb1, ufb2]


def getCondition(robotId):

    return [x, y, theta, v_x, v_y, v_theta]
