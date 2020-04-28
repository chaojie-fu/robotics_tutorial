import scipy.io as sio
import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d

# getPlan()
# 输入：steps 模拟步数
# 输出：plan  参考轨迹


def getPlan(steps):

    # 载入data.mat(把这个问路径改成你的data.mat路径即可)
    data = sio.loadmat(
        r'data.mat')

    # print('scipy读取三维矩阵的初步结果: \n%s\n' % data)
    t = data['t']
    state = data['z']
    u = data['u']

    x = state[0, ...]
    y = state[1, ...]
    theta = -state[2, ...]
    v_x = state[3, ...]
    v_y = state[4, ...]
    v_theta = state[5, ...]
    t = t[0, ...]

    # 求插值函数
    f_x = interp1d(t, x, kind='cubic')  # x 三次样条插值函数
    print(f_x)
    f_y = interp1d(t, y, kind='cubic')  # y
    f_theta = interp1d(t, theta, kind='cubic')
    f_v_x = interp1d(t, v_x, kind='cubic')
    f_v_y = interp1d(t, v_y, kind='cubic')
    f_v_theta = interp1d(t, v_theta, kind='cubic')

    t_pred = np.linspace(0, t[-1], num=steps)  # 时间点

    # 得到plan
    plan = []
    for i in range(steps):
        plan.append([f_x(t_pred[i]), f_v_x(t_pred[i]), f_y(t_pred[i]), f_v_y(t_pred[i]),
                     f_theta(t_pred[i]), f_v_theta(t_pred[i])])
    return plan
