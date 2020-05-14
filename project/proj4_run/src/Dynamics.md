# 地面动力学与空中动力学

## 地面动力学

$$
Q_x = 
\frac{1.0 I_{R} \frac{d^{2}}{d t^{2}} x{\left(t \right)} + r^{2} \left(1.0 M \frac{d^{2}}{d t^{2}} x{\left(t \right)} + m_{b} \left(- l_{1} \sin{\left(\theta{\left(t \right)} \right)} \left(\frac{d}{d t} \theta{\left(t \right)}\right)^{2} + l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - l_{2} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} + l_{2} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + \frac{d^{2}}{d t^{2}} x{\left(t \right)}\right)\right)}{r^{2}}
$$



$$
Q_\theta = 
1.0 I_{b} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + 1.0 I_{b} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - 1.0 g l_{1} m_{b} \sin{\left(\theta{\left(t \right)} \right)} + 1.0 l_{1}^{2} m_{b} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - 1.0 l_{1} l_{2} m_{b} \sin{\left(\phi{\left(t \right)} \right)} \left(\frac{d}{d t} \phi{\left(t \right)}\right)^{2} + 1.0 l_{1} l_{2} m_{b} \cos{\left(\phi{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + 1.0 l_{1} m_{b} \cos{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} x{\left(t \right)} + 1.0 l_{2} m_{b} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} \frac{d}{d t} x{\left(t \right)}
$$



$$
Q_\phi = 
I_{b} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + I_{b} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - g l_{2} m_{b} \sin{\left(\phi{\left(t \right)} \right)} + l_{1} l_{2} m_{b} \cos{\left(\phi{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} + l_{2}^{2} m_{b} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} - l_{2} m_{b} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} \frac{d}{d t} x{\left(t \right)} + l_{2} m_{b} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} x{\left(t \right)}
$$



广义力如下：
$$
\begin{cases}
Q_x = -M_2 / r\\
Q_\theta = M_1 + M_2\\
Q_\phi = -M_1\\
\end{cases}
$$
若需要将上式输入到python中，使用替换命令替换如下python输出即可:

```python
Q_x = (1.0*I_R*Derivative(x(t), (t, 2)) + r**2*(1.0*M*Derivative(x(t), (t, 2)) + m_b*(-l_1*sin(theta(t))*Derivative(theta(t), t)**2 + l_1*cos(theta(t))*Derivative(theta(t), (t, 2)) - l_2*(Derivative(phi(t), t) + Derivative(theta(t), t))*sin(phi(t) + theta(t))*Derivative(phi(t), t) + l_2*cos(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) + Derivative(x(t), (t, 2)))))/r**2

Q_theta = 1.0*I_b*Derivative(phi(t), (t, 2)) + 1.0*I_b*Derivative(theta(t), (t, 2)) - 1.0*g*l_1*m_b*sin(theta(t)) + 1.0*l_1**2*m_b*Derivative(theta(t), (t, 2)) - 1.0*l_1*l_2*m_b*sin(phi(t))*Derivative(phi(t), t)**2 + 1.0*l_1*l_2*m_b*cos(phi(t))*Derivative(phi(t), (t, 2)) + 1.0*l_1*m_b*cos(theta(t))*Derivative(x(t), (t, 2)) + 1.0*l_2*m_b*sin(phi(t) + theta(t))*Derivative(phi(t), t)*Derivative(x(t), t)

Q_phi = I_b*Derivative(phi(t), (t, 2)) + I_b*Derivative(theta(t), (t, 2)) - g*l_2*m_b*sin(phi(t)) + l_1*l_2*m_b*cos(phi(t))*Derivative(theta(t), (t, 2)) + l_2**2*m_b*Derivative(phi(t), (t, 2)) - l_2*m_b*sin(phi(t) + theta(t))*Derivative(theta(t), t)*Derivative(x(t), t) + l_2*m_b*cos(phi(t) + theta(t))*Derivative(x(t), (t, 2))
```



### 等式的化简

简化变量，消除时间t：
$$
\begin{align}
Q_x &= 
\frac{I_{R}}{r^2} \frac{d^{2}}{d t^{2}} x + ( M \frac{d^{2}}{d t^{2}} x + m_{b} (- l_{1} \sin{(\theta )} (\frac{d}{d t} \theta)^{2} + l_{1} \cos{(\theta )} \frac{d^{2}}{d t^{2}} \theta - l_{2} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) \sin{(\phi + \theta )} \frac{d}{d t} \phi + l_{2} \cos{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi + \frac{d^{2}}{d t^{2}} x))\\

Q_\theta &= 
I_{b} \frac{d^{2}}{d t^{2}} \phi +  I_{b} \frac{d^{2}}{d t^{2}} \theta -  g l_{1} m_{b} \sin{(\theta )} +  l_{1}^{2} m_{b} \frac{d^{2}}{d t^{2}} \theta -  l_{1} l_{2} m_{b} \sin{(\phi )} (\frac{d}{d t} \phi)^{2} +  l_{1} l_{2} m_{b} \cos{(\phi )} \frac{d^{2}}{d t^{2}} \phi +  l_{1} m_{b} \cos{(\theta )} \frac{d^{2}}{d t^{2}} x +  l_{2} m_{b} \sin{(\phi + \theta )} \frac{d}{d t} \phi \frac{d}{d t} x\\

Q_\phi &= 
I_{b} \frac{d^{2}}{d t^{2}} \phi + I_{b} \frac{d^{2}}{d t^{2}} \theta - g l_{2} m_{b} \sin{(\phi )} + l_{1} l_{2} m_{b} \cos{(\phi )} \frac{d^{2}}{d t^{2}} \theta + l_{2}^{2} m_{b} \frac{d^{2}}{d t^{2}} \phi - l_{2} m_{b} \sin{(\phi + \theta )} \frac{d}{d t} \theta \frac{d}{d t} x + l_{2} m_{b} \cos{(\phi + \theta )} \frac{d^{2}}{d t^{2}} x

\end{align}
$$






## 空中动力学

$$
Q_x = 
1.0 M \frac{d^{2}}{d t^{2}} x{\left(t \right)} + m_{b} \left(- l_{1} \sin{\left(\theta{\left(t \right)} \right)} \left(\frac{d}{d t} \theta{\left(t \right)}\right)^{2} + l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \log{\left(l_{2} \right)} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + \frac{d^{2}}{d t^{2}} x{\left(t \right)}\right)
$$



$$
Q_y =
M g + 1.0 M \frac{d^{2}}{d t^{2}} y{\left(t \right)} + g m_{b} - 1.0 m_{b} \left(l_{1} \sin{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} + l_{1} \cos{\left(\theta{\left(t \right)} \right)} \left(\frac{d}{d t} \theta{\left(t \right)}\right)^{2} + l_{2} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} + l_{2} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} - \frac{d^{2}}{d t^{2}} y{\left(t \right)}\right)
$$



$$
Q_{\theta} =
I_{b} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + I_{b} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - g l_{1} m_{b} \sin{\left(\theta{\left(t \right)} \right)} - g l_{2} m_{b} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + l_{1}^{2} m_{b} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} + \frac{l_{1} l_{2} m_{b} \sin{\left(\phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)} \left(\frac{d}{d t} \phi{\left(t \right)}\right)^{2}}{2} - \frac{l_{1} l_{2} m_{b} \sin{\left(\phi{\left(t \right)} \right)} \left(\frac{d}{d t} \phi{\left(t \right)}\right)^{2}}{2} - \frac{l_{1} l_{2} m_{b} \cos{\left(\phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)}}{2} + \frac{l_{1} l_{2} m_{b} \cos{\left(\phi{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)}}{2} + \frac{l_{1} l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} m_{b} \log{\left(l_{2} \right)} \cos{\left(\phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)}}{2} + \frac{l_{1} l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} m_{b} \log{\left(l_{2} \right)} \cos{\left(\phi{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)}}{2} - \frac{l_{1} l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} m_{b} \sin{\left(\phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)}}{2} - \frac{l_{1} l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} m_{b} \sin{\left(\phi{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)}}{2} - l_{1} m_{b} \sin{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} y{\left(t \right)} + l_{1} m_{b} \cos{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} x{\left(t \right)} - \frac{l_{2}^{2} m_{b} \sin{\left(2 \phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)} \left(\frac{d}{d t} \phi{\left(t \right)}\right)^{2}}{2} + l_{2} m_{b} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} \frac{d}{d t} y{\left(t \right)} + \frac{l_{2}^{2 \frac{d}{d t} \phi{\left(t \right)}} m_{b} \sin{\left(2 \phi{\left(t \right)} + 2 \theta{\left(t \right)} \right)}}{2} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} m_{b} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} x{\left(t \right)}
$$



$$
Q_{\phi} =
I_{b} \left(\frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + \frac{d^{2}}{d t^{2}} \theta{\left(t \right)}\right) - g l_{2} m_{b} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} - 1.0 m_{b} \left(l_{2} \left(l_{1} \sin{\left(\theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} + l_{2} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} - \frac{d}{d t} y{\left(t \right)}\right) \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} - l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + \frac{d}{d t} x{\left(t \right)}\right) \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)}\right) + m_{b} \left(l_{2} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \left(l_{1} \sin{\left(\theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} + l_{2} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} - \frac{d}{d t} y{\left(t \right)}\right) \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + l_{2} \left(l_{1} \sin{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} + l_{1} \cos{\left(\theta{\left(t \right)} \right)} \left(\frac{d}{d t} \theta{\left(t \right)}\right)^{2} + l_{2} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d}{d t} \phi{\left(t \right)} + l_{2} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} - \frac{d^{2}}{d t^{2}} y{\left(t \right)}\right) \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} - l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \left(l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + \frac{d}{d t} x{\left(t \right)}\right) \log{\left(l_{2} \right)} \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d}{d t} \theta{\left(t \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + \frac{d}{d t} x{\left(t \right)}\right) \log{\left(l_{2} \right)}^{2} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(- l_{1} \sin{\left(\theta{\left(t \right)} \right)} \left(\frac{d}{d t} \theta{\left(t \right)}\right)^{2} + l_{1} \cos{\left(\theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \theta{\left(t \right)} - l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \left(\frac{d}{d t} \phi{\left(t \right)} + \frac{d}{d t} \theta{\left(t \right)}\right) \sin{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} + l_{2}^{\frac{d}{d t} \phi{\left(t \right)}} \log{\left(l_{2} \right)} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)} \frac{d^{2}}{d t^{2}} \phi{\left(t \right)} + \frac{d^{2}}{d t^{2}} x{\left(t \right)}\right) \log{\left(l_{2} \right)} \cos{\left(\phi{\left(t \right)} + \theta{\left(t \right)} \right)}\right)
$$



$$
Q_{\alpha} =
1.0 I_{R} \frac{d^{2}}{d t^{2}} \alpha{\left(t \right)}
$$



若需要将上式输入到python中，使用替换命令替换如下python输出即可:

```python
Q_x = 1.0*M*Derivative(x(t), (t, 2)) + m_b*(-l_1*sin(theta(t))*Derivative(theta(t), t)**2 + l_1*cos(theta(t))*Derivative(theta(t), (t, 2)) - l_2**Derivative(phi(t), t)*(Derivative(phi(t), t) + Derivative(theta(t), t))*sin(phi(t) + theta(t)) + l_2**Derivative(phi(t), t)*log(l_2)*cos(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) + Derivative(x(t), (t, 2)))

Q_y = M*g + 1.0*M*Derivative(y(t), (t, 2)) + g*m_b - 1.0*m_b*(l_1*sin(theta(t))*Derivative(theta(t), (t, 2)) + l_1*cos(theta(t))*Derivative(theta(t), t)**2 + l_2*(Derivative(phi(t), t) + Derivative(theta(t), t))*cos(phi(t) + theta(t))*Derivative(phi(t), t) + l_2*sin(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) - Derivative(y(t), (t, 2)))

Q_theta = I_b*Derivative(phi(t), (t, 2)) + I_b*Derivative(theta(t), (t, 2)) - g*l_1*m_b*sin(theta(t)) - g*l_2*m_b*sin(phi(t) + theta(t)) + l_1**2*m_b*Derivative(theta(t), (t, 2)) + l_1*l_2*m_b*sin(phi(t) + 2*theta(t))*Derivative(phi(t), t)**2/2 - l_1*l_2*m_b*sin(phi(t))*Derivative(phi(t), t)**2/2 - l_1*l_2*m_b*cos(phi(t) + 2*theta(t))*Derivative(phi(t), (t, 2))/2 + l_1*l_2*m_b*cos(phi(t))*Derivative(phi(t), (t, 2))/2 + l_1*l_2**Derivative(phi(t), t)*m_b*log(l_2)*cos(phi(t) + 2*theta(t))*Derivative(phi(t), (t, 2))/2 + l_1*l_2**Derivative(phi(t), t)*m_b*log(l_2)*cos(phi(t))*Derivative(phi(t), (t, 2))/2 - l_1*l_2**Derivative(phi(t), t)*m_b*sin(phi(t) + 2*theta(t))*Derivative(phi(t), t)/2 - l_1*l_2**Derivative(phi(t), t)*m_b*sin(phi(t))*Derivative(phi(t), t)/2 - l_1*m_b*sin(theta(t))*Derivative(y(t), (t, 2)) + l_1*m_b*cos(theta(t))*Derivative(x(t), (t, 2)) - l_2**2*m_b*sin(2*phi(t) + 2*theta(t))*Derivative(phi(t), t)**2/2 + l_2*m_b*cos(phi(t) + theta(t))*Derivative(phi(t), t)*Derivative(y(t), t) + l_2**(2*Derivative(phi(t), t))*m_b*sin(2*phi(t) + 2*theta(t))/2 + l_2**Derivative(phi(t), t)*m_b*sin(phi(t) + theta(t))*Derivative(x(t), t)

Q_phi = I_b*(Derivative(phi(t), (t, 2)) + Derivative(theta(t), (t, 2))) - g*l_2*m_b*sin(phi(t) + theta(t)) - 1.0*m_b*(l_2*(l_1*sin(theta(t))*Derivative(theta(t), t) + l_2*sin(phi(t) + theta(t))*Derivative(phi(t), t) - Derivative(y(t), t))*cos(phi(t) + theta(t))*Derivative(phi(t), t) - l_2**Derivative(phi(t), t)*(l_1*cos(theta(t))*Derivative(theta(t), t) + l_2**Derivative(phi(t), t)*cos(phi(t) + theta(t)) + Derivative(x(t), t))*sin(phi(t) + theta(t))) + m_b*(l_2*(Derivative(phi(t), t) + Derivative(theta(t), t))*(l_1*sin(theta(t))*Derivative(theta(t), t) + l_2*sin(phi(t) + theta(t))*Derivative(phi(t), t) - Derivative(y(t), t))*cos(phi(t) + theta(t)) + l_2*(l_1*sin(theta(t))*Derivative(theta(t), (t, 2)) + l_1*cos(theta(t))*Derivative(theta(t), t)**2 + l_2*(Derivative(phi(t), t) + Derivative(theta(t), t))*cos(phi(t) + theta(t))*Derivative(phi(t), t) + l_2*sin(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) - Derivative(y(t), (t, 2)))*sin(phi(t) + theta(t)) - l_2**Derivative(phi(t), t)*(Derivative(phi(t), t) + Derivative(theta(t), t))*(l_1*cos(theta(t))*Derivative(theta(t), t) + l_2**Derivative(phi(t), t)*cos(phi(t) + theta(t)) + Derivative(x(t), t))*log(l_2)*sin(phi(t) + theta(t)) + l_2**Derivative(phi(t), t)*(l_1*cos(theta(t))*Derivative(theta(t), t) + l_2**Derivative(phi(t), t)*cos(phi(t) + theta(t)) + Derivative(x(t), t))*log(l_2)**2*cos(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) + l_2**Derivative(phi(t), t)*(-l_1*sin(theta(t))*Derivative(theta(t), t)**2 + l_1*cos(theta(t))*Derivative(theta(t), (t, 2)) - l_2**Derivative(phi(t), t)*(Derivative(phi(t), t) + Derivative(theta(t), t))*sin(phi(t) + theta(t)) + l_2**Derivative(phi(t), t)*log(l_2)*cos(phi(t) + theta(t))*Derivative(phi(t), (t, 2)) + Derivative(x(t), (t, 2)))*log(l_2)*cos(phi(t) + theta(t)))

Q_alpha = 1.0*I_R*Derivative(alpha(t), (t, 2))
```



广义力如下：
$$
\begin{cases}
Q_x = 0\\
Q_y = 0\\
Q_\theta = M_1 + M_2\\
Q_\phi = -M_1\\
Q_\alpha = -M_2
\end{cases}
$$



### 等式的化简

$$
\begin{align}
Q_x &= 
 M \frac{d^{2}}{d t^{2}} x + m_{b} (- l_{1} \sin{(\theta )} (\frac{d}{d t} \theta)^{2} + l_{1} \cos{(\theta )} \frac{d^{2}}{d t^{2}} \theta - l_{2}^{\frac{d}{d t} \phi} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) \sin{(\phi + \theta )} + l_{2}^{\frac{d}{d t} \phi} \log{(l_{2} )} \cos{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi + \frac{d^{2}}{d t^{2}} x)\\

Q_y &=
M g +  M \frac{d^{2}}{d t^{2}} y + g m_{b} -  m_{b} (l_{1} \sin{(\theta )} \frac{d^{2}}{d t^{2}} \theta + l_{1} \cos{(\theta )} (\frac{d}{d t} \theta)^{2} + l_{2} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) \cos{(\phi + \theta )} \frac{d}{d t} \phi + l_{2} \sin{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi - \frac{d^{2}}{d t^{2}} y)\\

Q_{\theta} &=
I_{b} \frac{d^{2}}{d t^{2}} \phi + I_{b} \frac{d^{2}}{d t^{2}} \theta - g l_{1} m_{b} \sin{(\theta )} - g l_{2} m_{b} \sin{(\phi + \theta )} + l_{1}^{2} m_{b} \frac{d^{2}}{d t^{2}} \theta + \frac{l_{1} l_{2} m_{b} \sin{(\phi + 2 \theta )} (\frac{d}{d t} \phi)^{2}}{2} - \frac{l_{1} l_{2} m_{b} \sin{(\phi )} (\frac{d}{d t} \phi)^{2}}{2} - \frac{l_{1} l_{2} m_{b} \cos{(\phi + 2 \theta )} \frac{d^{2}}{d t^{2}} \phi}{2} + \frac{l_{1} l_{2} m_{b} \cos{(\phi )} \frac{d^{2}}{d t^{2}} \phi}{2} + \frac{l_{1} l_{2}^{\frac{d}{d t} \phi} m_{b} \log{(l_{2} )} \cos{(\phi + 2 \theta )} \frac{d^{2}}{d t^{2}} \phi}{2} + \frac{l_{1} l_{2}^{\frac{d}{d t} \phi} m_{b} \log{(l_{2} )} \cos{(\phi )} \frac{d^{2}}{d t^{2}} \phi}{2} - \frac{l_{1} l_{2}^{\frac{d}{d t} \phi} m_{b} \sin{(\phi + 2 \theta )} \frac{d}{d t} \phi}{2} - \frac{l_{1} l_{2}^{\frac{d}{d t} \phi} m_{b} \sin{(\phi )} \frac{d}{d t} \phi}{2} - l_{1} m_{b} \sin{(\theta )} \frac{d^{2}}{d t^{2}} y + l_{1} m_{b} \cos{(\theta )} \frac{d^{2}}{d t^{2}} x - \frac{l_{2}^{2} m_{b} \sin{(2 \phi + 2 \theta )} (\frac{d}{d t} \phi)^{2}}{2} + l_{2} m_{b} \cos{(\phi + \theta )} \frac{d}{d t} \phi \frac{d}{d t} y + \frac{l_{2}^{2 \frac{d}{d t} \phi} m_{b} \sin{(2 \phi + 2 \theta )}}{2} + l_{2}^{\frac{d}{d t} \phi} m_{b} \sin{(\phi + \theta )} \frac{d}{d t} x\\

Q_{\phi} &=
I_{b} (\frac{d^{2}}{d t^{2}} \phi + \frac{d^{2}}{d t^{2}} \theta) - g l_{2} m_{b} \sin{(\phi + \theta )} -  m_{b} (l_{2} (l_{1} \sin{(\theta )} \frac{d}{d t} \theta + l_{2} \sin{(\phi + \theta )} \frac{d}{d t} \phi - \frac{d}{d t} y) \cos{(\phi + \theta )} \frac{d}{d t} \phi - l_{2}^{\frac{d}{d t} \phi} (l_{1} \cos{(\theta )} \frac{d}{d t} \theta + l_{2}^{\frac{d}{d t} \phi} \cos{(\phi + \theta )} + \frac{d}{d t} x) \sin{(\phi + \theta )}) + m_{b} (l_{2} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) (l_{1} \sin{(\theta )} \frac{d}{d t} \theta + l_{2} \sin{(\phi + \theta )} \frac{d}{d t} \phi - \frac{d}{d t} y) \cos{(\phi + \theta )} + l_{2} (l_{1} \sin{(\theta )} \frac{d^{2}}{d t^{2}} \theta + l_{1} \cos{(\theta )} (\frac{d}{d t} \theta)^{2} + l_{2} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) \cos{(\phi + \theta )} \frac{d}{d t} \phi + l_{2} \sin{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi - \frac{d^{2}}{d t^{2}} y) \sin{(\phi + \theta )} - l_{2}^{\frac{d}{d t} \phi} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) (l_{1} \cos{(\theta )} \frac{d}{d t} \theta + l_{2}^{\frac{d}{d t} \phi} \cos{(\phi + \theta )} + \frac{d}{d t} x) \log{(l_{2} )} \sin{(\phi + \theta )} + l_{2}^{\frac{d}{d t} \phi} (l_{1} \cos{(\theta )} \frac{d}{d t} \theta + l_{2}^{\frac{d}{d t} \phi} \cos{(\phi + \theta )} + \frac{d}{d t} x) \log{(l_{2} )}^{2} \cos{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi + l_{2}^{\frac{d}{d t} \phi} (- l_{1} \sin{(\theta )} (\frac{d}{d t} \theta)^{2} + l_{1} \cos{(\theta )} \frac{d^{2}}{d t^{2}} \theta - l_{2}^{\frac{d}{d t} \phi} (\frac{d}{d t} \phi + \frac{d}{d t} \theta) \sin{(\phi + \theta )} + l_{2}^{\frac{d}{d t} \phi} \log{(l_{2} )} \cos{(\phi + \theta )} \frac{d^{2}}{d t^{2}} \phi + \frac{d^{2}}{d t^{2}} x) \log{(l_{2} )} \cos{(\phi + \theta )})\\

Q_{\alpha} &=
 I_{R} \frac{d^{2}}{d t^{2}} \alpha
\end{align}
$$

