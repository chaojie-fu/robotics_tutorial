import numpy as np
from pyomo.environ import *  # 导入该模块所有函数
from pyomo.dae import *


class Walk:
    def __init__(self, state, reference):
        # state be a 1 * 8 array
        # reference be a 4 * predict_step array, which concludes theta fy and x reference
        m = ConcreteModel()
        length4reference = np.size(reference, 1)
        m.predict_step = RangeSet(1, length4reference)
        m.minus_1_predict_step = RangeSet(1, length4reference - 1)
        m.minus_2_predict_step = RangeSet(1, length4reference - 2)
        m.State = RangeSet(1, 6)

        # Parameters
        # weight of item in objective function
        # {0: theta position error, 1: fy position error,2: x position error,
        #  3: moment value, 4: rate of moment's change}
        m.wg = Param(RangeSet(0, 4), initialize={0: 10, 1: 10, 2: 1, 3: 0.00001, 4: 0})

        m.dt = 1/240.0
        m.mb = 7
        m.M = 2
        m.Ib = 0.5
        m.Ir = 0.5
        m.g = 10
        m.l1 = 0.5
        m.l2 = 0.5
        m.r = 0.2

        m.s0 = Param(m.State,
                     initialize={1: state[0], 2: state[1], 3: state[2], 4: state[3], 5: state[4], 6: state[5]})

        ref4theta = {}
        for i in m.predict_step:
            ref4theta[i] = reference[0][i - 1]
        m.reference_theta = Param(m.predict_step, initialize=ref4theta)

        ref4fy = {}
        for i in m.predict_step:
            ref4fy[i] = reference[1][i - 1]
        m.reference_fy = Param(m.predict_step, initialize=ref4fy)

        ref4x = {}
        for i in m.predict_step:
            ref4x[i] = reference[2][i - 1]
        m.reference_x = Param(m.predict_step, initialize=ref4x)

        # Variables
        # m.s: {1: theta, 2: fy, 3: x, 4: v_theta, 5: v_fy, 6: v_x}
        m.s = Var(m.State, m.predict_step)
        m.M1 = Var(m.predict_step, bounds=(-250, 250))
        m.M2 = Var(m.predict_step, bounds=(-250, 250))

        # Constraints
        def rule_s(model, i):
            return m.s[i, 1] == m.s0[i]
        m.s0_update = Constraint(m.State, rule=rule_s)

        def rule_theta(model, i):
            return m.s[1, i + 1] == m.s[1, i] + m.s[4, i] * m.dt
        m.theta_update = Constraint(m.minus_1_predict_step, rule=rule_theta)

        def rule_fy(model, i):
            return m.s[2, i + 1] == m.s[2, i] + m.s[5, i] * m.dt
        m.fy_update = Constraint(m.minus_1_predict_step, rule=rule_fy)

        def rule_x(model, i):
            return m.s[3, i + 1] == m.s[3, i] + m.s[6, i] * m.dt
        m.x_update = Constraint(m.minus_1_predict_step, rule=rule_x)

        def rule_v_theta(model, i):
            # return (m.M1[i]+m.M2[i]) * m.dt == \
            #        m.mb * ((m.s[4, i+1] - m.s[4, i]) * (m.Ib / m.mb + m.l1 ** 2) +
            #                (m.s[5, i+1] - m.s[5, i]) * (m.Ib / m.mb + m.l1 * m.l2 * cos(m.s[2, i])) +
            #                (m.s[6, i+1] - m.s[6, i]) * m.l1 * cos(m.s[1, i]) +
            #                (m.l1 * m.s[6, i] * m.s[5, i] * cos(m.s[1, i] + m.s[2, i]) -
            #                 m.l1 * m.l2 * (m.s[5, i]) ** 2 * sin(m.s[2, i]) - m.l1 * m.g * sin(m.s[1, i])) * m.dt)
            return m.M1[i] + m.M2[i] == m.Ib * (m.s[5, i + 1] - m.s[5, i]) / m.dt + \
                   m.Ib * (m.s[4, i + 1] - m.s[4, i]) / m.dt - m.g * m.l1 * m.mb * sin(m.s[1, i]) + \
                   m.l1 * m.l1 * m.mb * (m.s[4, i + 1] - m.s[4, i]) / m.dt - \
                   m.l1 * m.l2 * m.mb * sin(m.s[2, i]) * ((m.s[2, i + 1] - m.s[2, i]) / m.dt)**2 + \
                   m.l1 * m.l2 * m.mb * cos(m.s[2, i]) * ((m.s[5, i + 1] - m.s[5, i]) / m.dt) + \
                   m.l1 * m.mb * cos(m.s[1, i]) * ((m.s[6, i + 1] - m.s[6, i]) / m.dt) + \
                   m.l2 * m.mb * sin(m.s[1, i] + m.s[2, i]) * ((m.s[2, i + 1] - m.s[2, i]) / m.dt) * \
                   ((m.s[3, i + 1] - m.s[3, i]) / m.dt)
        m.v_theta_update = Constraint(m.minus_1_predict_step, rule=rule_v_theta)

        def rule_v_fy(model, i):
            # return -m.M1[i]*m.dt == \
            #        m.mb*((m.s[4, i+1]-m.s[4, i])*(m.Ib/m.mb+m.l1*m.l2*cos(m.s[2, i])) +
            #              (m.s[5, i+1]-m.s[5, i])*(m.Ib/m.mb+m.l2**2) +
            #              (m.s[6, i+1]-m.s[6, i])*m.l2*cos(m.s[2, i]+m.s[1, i]) -
            #              (m.l2 * sin(m.s[1, i]+m.s[2, i])*m.s[4, i] * m.s[6, i] + m.g*m.l2*sin(m.s[2, i]))*m.dt)
            return -m.M1[i] == m.Ib * ((m.s[5, i + 1] - m.s[5, i]) / m.dt) + \
                   m.Ib * ((m.s[4, i + 1] - m.s[4, i]) / m.dt) - m.g * m.l2 * m.mb * sin(m.s[2, i]) + \
                   m.l1 * m.l2 * m.mb * cos(m.s[2, i]) * ((m.s[4, i + 1] - m.s[4, i]) / m.dt) + \
                   m.l2 * m.l2 * m.mb * ((m.s[5, i + 1] - m.s[5, i]) / m.dt) - \
                   m.l2 * m.mb * sin(m.s[2, i] + m.s[1, i]) * ((m.s[1, i + 1] - m.s[1, i]) / m.dt) * \
                   ((m.s[3, i + 1] - m.s[3, i]) / m.dt) + \
                   m.l2 * m.mb * cos(m.s[1, i] + m.s[2, i]) * ((m.s[6, i + 1] - m.s[6, i]) / m.dt)
        m.v_fy_update = Constraint(m.minus_1_predict_step, rule=rule_v_fy)

        def rule_v_x(model, i):
            # return -m.M2[i]*m.dt/m.r == \
            #        m.mb*((m.s[4, i+1]-m.s[4, i])*(m.l1*cos(m.s[1, i])) +
            #              (m.s[5, i+1]-m.s[5, i])*(m.l2*cos(m.s[1, i]+m.s[2, i])) +
            #              (m.s[6, i+1]-m.s[6, i])*(m.M+m.mb+m.Ir/m.r**2)/m.mb -
            #              (m.l1*sin(m.s[1, i])*m.s[4, i]**2 +
            #               m.l2*sin(m.s[1, i]+m.s[2, i])*((m.s[4, i] +m.s[5, i])*m.s[5, i]))*m.dt)
            return -m.M2[i] / m.r == m.Ir / m.r / m.r * ((m.s[6, i + 1] - m.s[6, i]) / m.dt) + \
                   (m.M * ((m.s[6, i + 1] - m.s[6, i]) / m.dt) +
                    m.mb * (-m.l1 * sin(m.s[1, i]) * ((m.s[1, i + 1] - m.s[1, i]) / m.dt)**2 +
                            m.l1 * cos(m.s[1, i]) * ((m.s[4, i + 1] - m.s[4, i]) / m.dt) -
                            m.l2 * (((m.s[2, i + 1] - m.s[2, i]) / m.dt) + ((m.s[1, i + 1] - m.s[1, i]) / m.dt)) *
                            sin(m.s[1, i] + m.s[2, i]) * ((m.s[2, i + 1] - m.s[2, i]) / m.dt) +
                            m.l2 * cos(m.s[1, i] + m.s[2, i]) * ((m.s[5, i + 1] - m.s[5, i]) / m.dt) +
                            ((m.s[6, i + 1] - m.s[6, i]) / m.dt)))
        m.v_x_update = Constraint(m.minus_1_predict_step, rule=rule_v_x)

        # Objective function
        m.referenceObj = sum((m.s[1, k]-m.reference_theta[k]) ** 2 * m.wg[0] +
                             (m.s[2, k]-m.reference_fy[k]) ** 2 * m.wg[1] +
                             (m.s[3, k]-m.reference_x[k]) ** 2 * m.wg[2] for k in m.minus_1_predict_step)

        m.M1obj = m.wg[3]*sum(m.M1[k]**2 for k in m.minus_1_predict_step)
        m.M2obj = m.wg[3]*sum(m.M2[k]**2 for k in m.minus_1_predict_step)

        m.sM1obj = m.wg[4]*sum((m.M1[k+1]-m.M1[k])**2 for k in m.minus_2_predict_step)
        m.sM2obj = m.wg[4]*sum((m.M2[k+1]-m.M2[k])**2 for k in m.minus_2_predict_step)

        m.obj = Objective(expr=m.referenceObj + m.M1obj+m.M2obj+m.sM1obj+m.sM2obj, sense=minimize)

        self.iN = m  # .create_instance()

    def Solve(self):
        SolverFactory('ipopt').solve(self.iN)

        M1 = -self.iN.M1[1]()
        M2 = self.iN.M2[1]()
        return [M1, M2]


class Fly:
    def __init__(self, state, reference):
        # state be a 1 * 8 array
        # reference be a 3 * predict_step array, which concludes theta fy and x reference
        m = ConcreteModel()
        length4reference = np.size(reference, 1)
        m.predict_step = RangeSet(1, length4reference)
        m.minus_1_predict_step = RangeSet(1, length4reference - 1)
        m.minus_2_predict_step = RangeSet(1, length4reference - 2)
        m.State = RangeSet(1, 8)

        # TODO: check Parameter Part
        # Parameters
        # weight of item in objective function
        # {0: theta position error, 1: fy position error,2: x position error,
        #  3: moment value, 4: rate of moment's change}
        m.wg = Param(RangeSet(0, 4), initialize={0: 10, 1: 10, 2: 1, 3: 0.00001, 4: 0})

        m.dt = 1 / 240.0
        m.mb = 7
        m.M = 2
        m.Ib = 0.5
        m.Ir = 0.5
        m.g = 10
        m.l1 = 0.5
        m.l2 = 0.5
        m.r = 0.2

        m.s0 = Param(m.State, initialize={1: state[0], 2: state[1], 3: state[2], 4: state[3],
                                          5: state[4], 6: state[5], 7: state[6], 8: state[7]})

        ref4theta = {}
        for i in m.predict_step:
            ref4theta[i] = reference[0][i - 1]
        m.reference_theta = Param(m.predict_step, initialize=ref4theta)

        ref4fy = {}
        for i in m.predict_step:
            ref4fy[i] = reference[1][i - 1]
        m.reference_fy = Param(m.predict_step, initialize=ref4fy)

        ref4x = {}
        for i in m.predict_step:
            ref4x[i] = reference[2][i - 1]
        m.reference_x = Param(m.predict_step, initialize=ref4x)

        ref4y = {}
        for i in m.predict_step:
            ref4y[i] = reference[3][i - 1]
        m.reference_x = Param(m.predict_step, initialize=ref4y)

        # Variables
        # m.s: {1: theta, 2: fy, 3: x, 4: v_theta, 5: v_fy, 6: v_x, 7: y, 8: v_y}
        m.s = Var(m.State, m.predict_step)
        m.M1 = Var(m.predict_step, bounds=(-250, 250))
        m.M2 = Var(m.predict_step, bounds=(-250, 250))

        # Constraints
        def rule_s(model, i):
            return m.s[i, 1] == m.s0[i]
        m.s0_update = Constraint(m.State, rule=rule_s)

        def rule_theta(model, i):
            return m.s[1, i + 1] == m.s[1, i] + m.s[4, i] * m.dt
        m.theta_update = Constraint(m.minus_1_predict_step, rule=rule_theta)

        def rule_fy(model, i):
            return m.s[2, i + 1] == m.s[2, i] + m.s[5, i] * m.dt
        m.fy_update = Constraint(m.minus_1_predict_step, rule=rule_fy)

        def rule_x(model, i):
            return m.s[3, i + 1] == m.s[3, i] + m.s[6, i] * m.dt
        m.x_update = Constraint(m.minus_1_predict_step, rule=rule_x)

        def rule_y(model, i):
            return m.s[7, i + 1] == m.s[7, i] + m.s[8, i] * m.dt
        m.y_update = Constraint(m.minus_1_predict_step, rule=rule_y)

        def rule_v_x(model, i):
            return


