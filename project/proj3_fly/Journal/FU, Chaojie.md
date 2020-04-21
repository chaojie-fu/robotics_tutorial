# 实验记录

[TOC]

## 2020.04.17

### 代码阅读

#### Env.py文件

23行：关节摩擦力为0.01；

25行：使用位置控制？

38行：限制马达最大输出为100；

#### HextechUAV.urdf

84行：过中心的长轴为x轴；

#### RobotControl.py

路径在仿真之前给出；

控制量在仿真过程中实时生成；

controlSignal 各取50时平衡；

连接体质量8，左右引擎质量各为1；



## 2020.04.20

### realTimeControl（robotId, plan）

从代码结构上来看，在获取了robotId的情况下就能够获取无人机在该时刻的所有状态。

可能用到的函数为 `getJointState` 和 `getLinkState` 。

能够获取的有效信息暂时有：（`getJointState` 似乎并没有什么用）：

<div align = "center">
    <img src = "pics\getLinkState.png" width = 700>
</div>



因此，需要解决的问题有二：

1. 控制无人机从一个目标点移动到相邻的一个目标点；（对速度等没有严格的要求，但是提高速度会提高控制算法的精度）
2. 在控制过程中的误差控制（使用反馈解决，但是用什么反馈还需要实际效果的检验）



## 2020.04.21

### 待解决的问题

1. `loadURDF`的过程，pybullet 仿真引擎好像是会重新计算转动惯量的，但是在代码中没有体现这一点。需要查阅相关文档



### 无人机的动力学

详见`笔记本-第三次实验-模型动力学`部分。

关于转动惯量等动力学参数：[http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model](http://wiki.ros.org/urdf/Tutorials/Adding Physical and Collision Properties to a URDF Model)；

动力学部分已基本完成。



### PID控制

基本公式：

若定义 $u(t)$ 为控制输入，则PID算法的控制输入可以用下面的方式表示
$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d\frac{d}{dt}e(t)
$$
其中，$e(t)$ 为误差值（设定值减测量值）。

问题有以下几个：

1. 控制量不止一个，需要所有的控制量都进行单独的PID控制吗？
2. 控制的目标是什么？是需要根据轨迹移动。但是怎么细化这个目标，便于控制？（比如设定好姿态角，质心速度等）
3. 控制的滞后问题可能会对问题有较大的影响。