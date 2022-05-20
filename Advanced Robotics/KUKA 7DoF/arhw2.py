# -*- coding: utf-8 -*-
"""ARHW2.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1afm_WgDAnO1QHHwRxooiwKyolLZI8ShO
"""

from operator import pos
import numpy as np
from math import *
import matplotlib.pyplot as plt


def Rx(q):
    T = np.array([[1, 0, 0, 0],
                  [0, np.cos(q), -np.sin(q), 0],
                  [0, np.sin(q), np.cos(q), 0],
                  [0, 0, 0, 1]])
    return T


def Ry(q):
    T = np.array([[np.cos(q), 0, np.sin(q), 0],
                  [0, 1, 0, 0],
                  [-np.sin(q), 0, np.cos(q), 0],
                  [0, 0, 0, 1]])
    return T


def Rz(q):
    T = np.array([[np.cos(q), -np.sin(q), 0, 0],
                  [np.sin(q), np.cos(q), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return T


def Tx(x):
    T = np.array([[1, 0, 0, x],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return T


def Ty(y):
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, y],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return T


def Tz(z):
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])
    return T


def dRx(q):
    T = np.array([[0, 0, 0, 0],
                  [0, -np.sin(q), -np.cos(q), 0],
                  [0, np.cos(q), -np.sin(q), 0],
                  [0, 0, 0, 0]])
    return T


def dRy(q):
    T = np.array([[-np.sin(q), 0, np.cos(q), 0],
                  [0, 0, 0, 0],
                  [-np.cos(q), 0, -np.sin(q), 0],
                  [0, 0, 0, 0]])
    return T


def dRz(q):
    T = np.array([[-np.sin(q), -np.cos(q), 0, 0],
                  [np.cos(q), -np.sin(q), 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])
    return T


def dTx(x):
    T = np.array([[0, 0, 0, 1],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])
    return T


def dTy(y):
    T = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])
    return T


def dTz(z):
    T = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, 0]])
    return T

def FK(q, links):
    T = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    return T

def GetJacobianColumn(dT, T_inv):
    dT = np.linalg.multi_dot([dT, T_inv])
    return np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

def JacobianVirtual(q, links):
    T = FK(q, links)
    T[0:3, 3] = 0
    T_inv = np.transpose(T)

    dT = np.linalg.multi_dot([dTy(q[0]),
                            Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    dT = np.linalg.multi_dot([dT, T_inv])
    J1 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                              Tz(links[0]),
                             dRz(q[1]),                   
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])


    dT = np.linalg.multi_dot([dT, T_inv])
    J2 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             dRy(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])


    dT = np.linalg.multi_dot([dT, T_inv])
    J3 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             dRy(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    dT = np.linalg.multi_dot([dT, T_inv])
    J4 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             dRz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    dT = np.linalg.multi_dot([dT, T_inv])
    J5 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             dRy(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    dT = np.linalg.multi_dot([dT, T_inv])
    J6 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    dT = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             dRz(q[6])])

    dT = np.linalg.multi_dot([dT, T_inv])
    J7 = np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

    J = np.hstack([J1, J2, J3, J4, J5, J6, J7])
    return J

ax = plt.axes(projection='3d')


def PlotFK(q, links, color='b', linewidth=3):
    pos0 = [0, 0, 0]

    T = Ty(q[0])

    pos1 = T[0:3, 3]

    T = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1])])

    pos2 = T[0:3, 3]

    T = np.linalg.multi_dot([Ty(q[0]),
                            Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2])])
    pos3 = T[0:3, 3]

    T = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3])])

    pos4 = T[0:3, 3]

    T = np.linalg.multi_dot([Ty(q[0]),
                             Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4])])

    pos5 = T[0:3, 3]

    T = np.linalg.multi_dot([Ty(q[0]),
                            Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5])])

    pos6 = T[0:3, 3]


    T = np.linalg.multi_dot([Ty(q[0]),
                            Tz(links[0]),
                             Rz(q[1]),
                             Tz(links[1]),
                             Ry(q[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Tz(-links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    pos7 = T[0:3, 3]

    # print(f"End-effector pos: {pos7}")

    x = [pos0[0], pos1[0], pos2[0], pos3[0], pos4[0], pos5[0], pos6[0], pos7[0]]
    y = [pos0[1], pos1[1], pos2[1], pos3[1], pos4[1], pos5[1], pos6[1], pos7[1]]
    z = [pos0[2], pos1[2], pos2[2], pos3[2], pos4[2], pos5[2], pos6[2], pos7[2]]

    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_zlim(0, 2)

    ax.plot3D(x, y, z, linewidth=linewidth, c=color)

    return pos7

# Print first fk
def WeightedPseudoInv(q_current, weighs):
    i = 0
    error = [10, 10, 10, 10, 10, 10, 10]

    while abs(sum(error[0:3])) > 0.01 or i < 2:
        r_current = FK(q_current, link_length)

        r_current = np.hstack([r_current[0:3, 3], [0, 0, 0]])

        error = r_global - r_current
        # print(f"[{i}] Error sum: {round(sum(error[0:3]), 4)}")
        d_error = error / 100

        jac = JacobianVirtual(q_current, link_length)
        # print(jac.shape)

        J_wgh_1 = np.linalg.multi_dot([np.linalg.pinv(weighs), np.transpose(jac)])
        J_wgh_2 = np.linalg.multi_dot([jac, np.linalg.pinv(weighs), np.transpose(jac)])

        J_wgh = np.linalg.multi_dot([J_wgh_1, np.linalg.pinv(J_wgh_2)])

        delta_q = np.dot(J_wgh, d_error)
        # print(delta_q.shape)

        q_current = q_current + delta_q
        i += 1

        if i % 20 == 0:
            PlotFK(q_current, link_length, color="black", linewidth=0.2)

    return q_current


link_length = [0.675, 0.35, 1.15, 1.2, 0.041, 0.24]

# Weighted pseudo inv
pose = PlotFK([-0.45, np.pi / 6, np.pi /5, np.pi / 4, np.pi , np.pi / 3, np.pi / 4], link_length, color="blue")

r_global = np.array([pose[0], pose[1], pose[2], 0, 0, 0])
q_start = np.array([pi, pi, pi, pi, pi, pi, pi])
weighs_pseudo_inv = np.diag([1, 1, 1, 1, 1, 1, 1])

q_final = WeightedPseudoInv(q_start, weighs_pseudo_inv)
PlotFK(q_final, link_length, 'r')

# ax.set_title('Weighted PI')
# plt.show()

def DLS(q_current, r_global, links_length):
    i = 0
    error = [10, 10, 10, 10, 10, 10]

    nu = 0.1
    Im = np.ones(6)

    #for i in range (0, 1000):
    while abs(sum(error[0:3])) > 0.01 or i < 2:
        r_current = FK(q_current, link_length)
        r_current = np.hstack([r_current[0:3, 3], [0, 0, 0]])

        error = r_global - r_current
        # print(f"[{i}] Error sum: {round(sum(error[0:3]), 4)}")
        d_error = error / 100

        J = JacobianVirtual(q_current, links_length)
        J_pinv = np.dot(np.transpose(J), np.linalg.pinv(np.dot(J, np.transpose(J))+nu**2 * Im))

        delta_q = np.dot(J_pinv, d_error)

        q_current = q_current + delta_q
        i += 1

        if i % 20 == 0:
            PlotFK(q_current, links_length, color="black", linewidth=0.2)

    return q_current

# # Damped least squares
# PlotFK([np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2], link_length, color="blue")

# r_global = np.array([-0.4, 0.4, 0.466, 0, 0, 0])
# q_start = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

q_final = DLS(q_start, r_global, link_length)
PlotFK(q_final, link_length, 'r')
ax.set_title('Damped least squares')
plt.show()


# plt.show()
