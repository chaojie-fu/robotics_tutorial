from sympy import *
from sympy.physics.mechanics import *

# Dynamic on the plane
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
theta = dynamicsymbols('theta')
theta_d = dynamicsymbols('theta', 1)
theta_dd = dynamicsymbols('theta', 2)
phi = dynamicsymbols('phi')
phi_d = dynamicsymbols('phi', 1)
phi_dd = dynamicsymbols('phi', 2)

kin_energy_plane = 0.5 * M * x_d**2 + 0.5 * IR * (x_d / r)**2 + \
    0.5 * mb * ((x_d + l1 * theta_d * cos(theta) + l2 * phi_d * cos(theta + phi))**2 +
                 (l1 * theta_d * sin(theta) + l2 * phi_d * sin(theta + phi))**2) + \
    0.5 * Ib * (theta_d + phi_d)**2
pot_energy_plane = mb * g * (l1 * cos(theta) + l2 * cos(phi))
L_plane = kin_energy_plane - pot_energy_plane

Q_x_plane = diff(diff(L_plane, x_d), 't') - diff(L_plane, x)
Q_theta_plane = diff(diff(L_plane, theta_d), 't') - diff(L_plane, theta)
Q_phi_plane = diff(diff(L_plane, phi_d), 't') - diff(L_plane, phi)

print("Q_x =", simplify(Q_x_plane))
print("Q_theta =", simplify(Q_theta_plane))
print("Q_phi =", simplify(Q_phi_plane))

print(latex(simplify(Q_x_plane)))
print(latex(simplify(Q_theta_plane)))
print(latex(simplify(Q_phi_plane)))

