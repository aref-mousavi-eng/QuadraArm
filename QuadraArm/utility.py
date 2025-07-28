import numpy as np
import math


def cosd(angle):
    return math.cos(angle*math.pi/180)


def sind(angle):
    return math.sin(angle*math.pi/180)


def atan2d(y, x):
    return (math.atan2(y, x)*180)/math.pi


def acosd(x):
    return (math.acos(x)*180)/math.pi


def T_i_to_i_minus_1(theta_i, d_i, r_i, alpha_i):
    T = np.array([[cosd(theta_i), -sind(theta_i)*cosd(alpha_i),  sind(theta_i)*sind(alpha_i), r_i*cosd(theta_i)],
                  [sind(theta_i),  cosd(theta_i)*cosd(alpha_i), -cosd(theta_i)*sind(alpha_i), r_i*sind(theta_i)],
                  [      0      ,         sind(alpha_i)       ,         cosd(alpha_i)       ,        d_i       ],
                  [      0      ,              0              ,              0              ,         1        ]])
    return T


def get_DH(thetas, d0, d1, l1, l2, l3, l4):
    DH = np.array([[        0,  d1, -d0,   0],
                   [thetas[0],  l1,   0,  90],
                   [thetas[1],   0,  l2,   0],
                   [thetas[2],   0,  l3,   0],
                   [thetas[3],   0,  l4,   0]])
    return DH


def add_lists(l1, l2):
    return [xi + yi for xi, yi in zip(l1, l2)]


def subtract_lists(l1, l2):
    return [xi - yi for xi, yi in zip(l1, l2)]
