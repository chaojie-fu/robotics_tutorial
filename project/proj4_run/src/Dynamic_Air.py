from sympy import *
from sympy.physics.mechanics import *

# Dynamic in the air
M = symbols('M')
mb = symbols('m_b')
IR = symbols('I_R')
Ib = symbols('I_b')

g = symbols('g')

l1 = symbols('l_1')
l2 = symbols('l_2')
r = symbols('r')

x = dynamicsymbols('x')
x_d = dynamicsymbols('x', 1)
x_dd = dynamicsymbols('x', 2)
y = dynamicsymbols('y')
y_d = dynamicsymbols('y', 1)
y_dd = dynamicsymbols('y', 2)
theta = dynamicsymbols('theta')
theta_d = dynamicsymbols('theta', 1)
theta_dd = dynamicsymbols('theta', 2)
phi = dynamicsymbols('phi')
phi_d = dynamicsymbols('phi', 1)
phi_dd = dynamicsymbols('phi', 2)
alpha = dynamicsymbols('alpha')
alpha_d = dynamicsymbols('alpha', 1)
alpha_dd = dynamicsymbols('alpha', 2)

kin_energy_air = 0.5 * M * x_d**2 + 0.5 * M * y_d**2 + 0.5 * IR * alpha_d**2 + \
    0.5 * mb * ((x_d + l1 * theta_d * cos(theta) + l2 ** phi_d * cos(theta + phi))**2 +
                (y_d - l1 * theta_d * sin(theta) - l2 * phi_d * sin(theta + phi))**2) +\
    0.5 * Ib * (theta_d + phi_d)**2
pot_energy_air = M * g * y + mb * g * (y + l1 * cos(theta) + l2 * cos(theta + phi))
L_air = kin_energy_air - pot_energy_air

Q_x_air = diff(diff(L_air, x_d), 't') - diff(L_air, x)
Q_y_air = diff(diff(L_air, y_d), 't') - diff(L_air, y)
Q_theta_air = diff(diff(L_air, theta_d), 't') - diff(L_air, theta)
Q_phi_air = diff(diff(L_air, phi_d), 't') - diff(L_air, phi)
Q_alpha_air = diff(diff(L_air, alpha_d), 't') - diff(L_air, alpha)

print("Q_x =", simplify(Q_x_air))
print("Q_y =", simplify(Q_y_air))
print("Q_theta =", simplify(Q_theta_air))
print("Q_phi =", simplify(Q_phi_air))
print("Q_alpha =", simplify(Q_alpha_air))

print(latex(simplify(Q_x_air)))
print(latex(simplify(Q_y_air)))
print(latex(simplify(Q_theta_air)))
print(latex(simplify(Q_phi_air)))
print(latex(simplify(Q_alpha_air)))