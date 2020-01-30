import frccontrol as fct
from math import sin, cos
import numpy as np


def pexp(twist):
    dx = twist[0, 0]
    dy = twist[1, 0]
    dtheta = twist[2, 0]
    sinTheta, cosTheta = sin(dtheta), cos(dtheta)

    if abs(dtheta) < 1E-9:
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
        c = 0.5 * dtheta
    else:
        s = sinTheta / dtheta
        c = (1 - cosTheta) / dtheta

    dx = dx * s - dy * c
    dy = dx * c + dy * s

    x = dx * cosTheta - dy * sinTheta
    y = dx * sinTheta + dy * cosTheta

    return np.array([[x], [y], [dtheta]])


# dt = 0.01
# x = [[x, y, theta]]^T, u = [[v_l, v_r]]^T
def f(x, u):
    mat = np.array([[1. / 2, 1. / 2], [0, 0], [-1. / 2, 1. / 2]])
    twist = mat @ u
    return pexp(twist) / 0.01


jacobianA = fct.numerical_jacobian_x(3, 3, f, np.zeros((3, 1)), np.zeros((2, 1)))
print(jacobianA)

jacobianB = fct.numerical_jacobian_u(3, 2, f, np.zeros((3, 1)), np.zeros((2, 1)))
print(jacobianB)
