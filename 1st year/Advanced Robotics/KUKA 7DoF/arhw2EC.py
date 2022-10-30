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
                             Tz(links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    return T

def GetJacobianColumn(dT, T_inv):
    dT = np.linalg.multi_dot([dT, T_inv])
    return np.vstack([dT[0, 3], dT[1, 3], dT[2, 3], dT[2, 1], dT[0, 2], dT[1, 0]])

def Jacobian_virtual(q, links, theta):
    T = FK(q, links)
    T[0:3, 3] = 0
    T_inv = np.transpose(T)

    J = []

                                    # Tz(d1),                   # active joint
                                    # dTz(theta[0]),            # virtual spring
                                    # Rz(q1),                   # passive joint
                                    # Tx(l1),                   # rigid link
                                    # T_3D(theta[1:7]),         # virtual spring
                                    # Rz(q2),                   # passive joint
                                    # Tx(l2),                   # rigid link
                                    # T_3D(theta[7:13]),        # virtual spring
                                    # Rz(q3),                   # passive joint
                                    # T_platform

    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             dTy(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])])

    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J1 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])

    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             dRz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J2 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])
    

    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             dRy(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J3 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])

    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             dRy(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J4 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             dRz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J5 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])
    

    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             dRy(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local, T_inv])
    J6 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                             Ty(theta[0]),        # virtual spring     
                             Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             dRx(theta[6])]) 
    Td_global = np.linalg.multi_dot([Td_local,T_inv])
    J7 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])

    J = np.transpose(np.vstack([J1, J2, J3, J4, J5, J6, J7]))   
    return J

def getRandomNoise():
    return np.random.normal(loc=0, scale=1e-5)


def FK_vjm(q, links, theta):
  
  T = np.linalg.multi_dot([Ty(q[0]),            # active joint
                           Ty(theta[0]),        # virtual spring     
                           Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             Ry(q[3]),
                             Ry(theta[3]),
                             Tz(links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 

  return T


# Initial parameters
experiments = 30
points_per_experiment = 100

link_length = [0.200, 0.250, 0.250, 0.100, 0.1, 0.1]
theta = np.array([pi, pi, pi, pi, pi, pi, pi])

# Create stiffness matrix before calibration
JointStiffness = np.array([1e+6, 0.56e+6, 0.3e+6, 0.43e+6, 2.8e+6, 3.2e+6, 2.1e+6])
JointStiffness = np.diag(JointStiffness)
print("Joint stiffness \n{0}\n".format(JointStiffness))

first_term = np.zeros((7, 7))
second_term = np.zeros(7)


for i in range(experiments):
    q_r = np.random.uniform(-np.pi, np.pi, 6)
    q_p = np.random.uniform(-1.5, 1.5, 1)
    q_revolute = qs = np.hstack([q_p, q_r])

    W = np.random.uniform(-1000, 1000, 6)

    jTheta = Jacobian_virtual(q_revolute, link_length, theta)

    dt = np.linalg.multi_dot([jTheta, np.linalg.inv(JointStiffness), np.transpose(jTheta), W]) + getRandomNoise()

    jTheta = jTheta[0:6, :]
    dt = dt[0:6]
    W = W[0:6]

    A = np.zeros(jTheta.shape)

    for i_jac in range(jTheta.shape[1]):
        j = jTheta[:, i_jac]
        A[:, i_jac] = np.outer(j, j).dot(W)

    first_term = first_term + np.transpose(A).dot(A)
    second_term = second_term + np.transpose(A).dot(dt)

delta_x_hat = np.linalg.inv(first_term).dot(second_term)

stiffness = np.divide(1, delta_x_hat)
print("Recalculated joint stiffness: \n{0}\n".format(stiffness))

# new stiffness matrix
JointStiffness = np.diag(stiffness)

print("Recalculated joint stiffness diag: \n{0}\n".format(JointStiffness))