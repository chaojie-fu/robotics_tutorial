import numpy as np
from pyomo.environ import *  # 导入该模块所有函数
from pyomo.dae import *

N = 5  # forward predict steps
ns = 6  # state numbers / here: {0: theta, 1: fy, 2: x, 3: v_theta, 4: v_fy, 5: v_x}
na = 2  # actuator numbers /here: {0: M1, 1: M2}
dt = 1/240


class Walk(object):
    def __init__(self):
        m = ConcreteModel()

        m.predict_step = RangeSet(0, N-1)
        m.minus_1_predict_step = RangeSet(0, N-2)
        m.minus_2_predict_step = RangeSet(0, N-3)

        # Parameters
        # weight of item in objective function
        m.wg = Param(RangeSet(0, 2), initialize={
            0: 5000000, 1: 1, 2: 1}, mutable=True)

        m.dt = Param(initialize=dt, mutable=True)
        m.mb = Param(initialize=7, mutable=True)
        m.M = Param(initialize=2, mutable=True)
        m.Ib = Param(initialize=0.5, mutable=True)
        m.Ir = Param(initialize=0.5, mutable=True)
        m.g = Param(initialize=10, mutable=True)
        m.l1 = Param(initialize=0.5, mutable=True)
        m.l2 = Param(initialize=0.5, mutable=True)
        m.r = Param(initialize=0.2, mutable=True)

        m.s0 = Param(RangeSet(  # 初始值
            0, ns-1), initialize={0: 0., 1: 0., 2: 0., 3: 0., 4: 0., 5: 0.}, mutable=True)
        m.reference_x = Param(  # 参考x(t)
            RangeSet(0, N-1), initialize={0: 0., 1: 0., 2: 0., 3: 0, 4: 0}, mutable=True)
        m.reference_theta = Param(  # 参考theta(t)
            RangeSet(0, N-1), initialize={0: -0.1, 1: -0.1, 2: -0.1, 3: -0.1, 4: -0.1}, mutable=True)
        m.reference_fy = Param(  # 参考x(t)
            RangeSet(0, N-1), initialize={0: 0.2, 1: 0.2, 2: 0.2, 3: 0.2, 4: 0.2}, mutable=True)

        # Variables
        m.s = Var(RangeSet(0, ns-1), m.predict_step)
        m.M1 = Var(m.minus_1_predict_step, bounds=(-250, 250))
        m.M2 = Var(m.minus_1_predict_step, bounds=(-250, 250))

        # {0: theta, 1: fy, 2: x, 3: v_theta, 4: v_fy, 5: v_x}
        # Constraints

        # TODO
        # check the rule=lambda syntax
        m.s0_update = Constraint(
            RangeSet(0, ns-1), rule=lambda m, i: m.s[i, 0] == m.s0[i])

        m.theta_update = Constraint(m.predict_step, rule=lambda m, k:
                                    m.s[0, k+1] == m.s[0, k] +
                                    m.s[3, k]*m.dt
                                    if k < N-1 else Constraint.Skip)
        m.fy_update = Constraint(m.predict_step, rule=lambda m, k:
                                 m.s[1, k+1] == m.s[1, k] +
                                 m.s[4, k]*m.dt
                                 if k < N-1 else Constraint.Skip)
        m.x_update = Constraint(m.predict_step, rule=lambda m, k:
                                m.s[2, k+1] == m.s[2, k] +
                                m.s[5, k]*m.dt
                                if k < N-1 else Constraint.Skip)

        m.v_theta_update = Constraint(m.predict_step, rule=lambda m, k:
                                      (m.M1[k]+m.M2[k])*m.dt == m.mb*((m.s[3, k+1]-m.s[3, k])*(m.Ib/m.mb+m.l1**2) +
                                                                      (m.s[4, k+1]-m.s[4, k])*(m.Ib/m.mb+m.l1*m.l2*cos(m.s[1, k])) +
                                                                      (m.s[5, k+1]-m.s[5, k])*m.l1*cos(m.s[0, k]) +
                                                                      (m.l1*m.s[5, k]*m.s[4, k]*cos(m.s[0, k]+m.s[1, k]) -
                                                                       m.l1*m.l2*(m.s[4, k]) ** 2 * sin(m.s[1, k]) -
                                                                       m.l1*m.g*sin(m.s[0, k]))*m.dt)
                                      if k < N-1 else Constraint.Skip)
        m.v_fy_update = Constraint(m.predict_step, rule=lambda m, k:
                                   m.M1[k]*m.dt == m.mb*((m.s[3, k+1]-m.s[3, k])*(m.Ib/m.mb+m.l1*m.l2*cos(m.s[1, k])) +
                                                         (m.s[4, k+1]-m.s[4, k])*(m.Ib/m.mb+m.l2**2) +
                                                         (m.s[5, k+1]-m.s[5, k])*m.l2*cos(m.s[1, k]+m.s[0, k]) -
                                                         (m.l2 * sin(m.s[0, k]+m.s[1, k])*m.s[3, k] *
                                                          m.s[5, k]+m.g*m.l2*sin(m.s[1, k]))*m.dt)
                                   if k < N-1 else Constraint.Skip)
        m.v_x_update = Constraint(m.predict_step, rule=lambda m, k:
                                  -m.M2[k]*m.dt/m.r == m.mb*((m.s[3, k+1]-m.s[3, k])*(m.l1*cos(m.s[0, k])) +
                                                             (m.s[4, k+1]-m.s[4, k])*(m.l2*cos(m.s[0, k]+m.s[1, k])) +
                                                             (m.s[5, k+1]-m.s[5, k])*(m.M+m.mb+m.Ir/m.r**2)/m.mb -
                                                             (m.l1*sin(m.s[0, k])*m.s[3, k]**2+m.l2*sin(m.s[0, k]+m.s[1, k])*((m.s[3, k]+m.s[4, k])*m.s[4, k]))*m.dt)
                                  if k < N-1 else Constraint.Skip)
        # Objective function
        m.referenceobj = m.wg[0]*sum((m.s[2, k] - m.reference_x[k]) ** 2 +
                                     (m.s[0, k] - m.reference_theta[k]) ** 2 +
                                     (m.s[1, k] - m.reference_fy[k]) ** 2 for k in m.predict_step)

        m.M1obj = m.wg[1]*sum(m.M1[k]**2 for k in m.minus_1_predict_step)
        m.M2obj = m.wg[1]*sum(m.M2[k]**2 for k in m.minus_1_predict_step)

        m.sM1obj = m.wg[2]*sum((m.M1[k+1]-m.M1[k])**2 for k in m.minus_2_predict_step)
        m.sM2obj = m.wg[2]*sum((m.M2[k+1]-m.M2[k])**2 for k in m.minus_2_predict_step)

        m.obj = Objective(expr=m.referenceobj +
                          m.M1obj+m.M2obj+m.sM1obj+m.sM2obj, sense=minimize)

        self.iN = m  # .create_instance()

    def Solve(self, state, reference):
        # self.iN.s0.reconstruct(
        #     {0: state[0], 1: state[1], 2: state[2], 3: state[3], 4: state[4], 5: state[5]})  # 输入当前状态值
        # self.iN.reference_theta.reconstruct(
        #     {0: reference[0, 0], 1: reference[0, 1], 2: reference[0, 2], 2: reference[0, 3], 2: reference[0, 4]})
        # self.iN.reference_x.reconstruct(
        #     {0: reference[1, 0], 1: reference[1, 1], 2: reference[1, 2], 2: reference[0, 3], 2: reference[0, 4]})
        # self.iN.reference_fy.reconstruct(
        #     {0: reference[2, 0], 1: reference[2, 1], 2: reference[2, 2], 2: reference[0, 3], 2: reference[0, 4]})
        # self.iN.s0_update.reconstruct()  # 将当前状态值作为迭代初始值

        self.iN.s0.reconstruct(
            {0: state[0], 1: state[1], 2: state[2], 3: state[3], 4: state[4]})  # 输入当前状态值
        self.iN.reference_theta.reconstruct(
            {0: reference[0, 0], 1: reference[0, 1], 3: reference[0, 2], 4: reference[0, 3]})
        self.iN.reference_x.reconstruct(
            {0: reference[1, 0], 1: reference[1, 1], 3: reference[1, 2], 4: reference[0, 3]})
        self.iN.reference_fy.reconstruct(
            {0: reference[2, 0], 1: reference[2, 1], 3: reference[2, 2], 4: reference[0, 3]})
        self.iN.s0_update.reconstruct()  # 将当前状态值作为迭代初始值

        SolverFactory('ipopt').solve(self.iN)

        M1 = self.iN.M1[0]()
        M2 = self.iN.M2[0]()
        return [M1, M2]
