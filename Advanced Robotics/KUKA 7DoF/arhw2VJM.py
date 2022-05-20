from operator import pos
import numpy as np
from math import *
import matplotlib.pyplot as plt
from numpy.linalg import inv

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
                             Tz(-links[3]),
                             Rz(q[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Tz(links[5]),
                             Rz(q[6])])

    return T

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
                             Tz(-links[3]),
                             Rz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])]) 

  return T

def Jacobian_virtual2(q, links, theta):
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


def Jacobian_virtual(q, links, theta, H):
    
    H = np.asarray(H)
    R = H[0:3, 0:3]
    R = np.linalg.inv(R)
    R_prime = np.hstack([R, [[0], [0], [0]] ])
    
    R_prime = np.vstack([R_prime, [0, 0, 0, 1]])
    
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

    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
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
    Td_global = np.linalg.multi_dot([Td_local,R_prime])
    J7 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])

    J = np.transpose(np.vstack([J1, J2, J3, J4, J5, J6, J7]))   
    return J


def Jacobian_passive(q, links, theta, H):
    
    H = np.asarray(H)
    R = H[0:3, 0:3]
    R = np.linalg.inv(R)
    R_prime = np.hstack([R, [[0], [0], [0]] ])
    
    R_prime = np.vstack([R_prime, [0, 0, 0, 1]])

    ## first joint ##
    Td_local = np.linalg.multi_dot([dTy(q[0]),            # active joint
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
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J1 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    ## second joint ##
    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                           Ty(theta[0]),        # virtual spring     
                           Tz(links[0]),        # rigid link
                             dRz(q[1]),          # active joint
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
            
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J2 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])
    

    ## third joint ##
    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                           Ty(theta[0]),        # virtual spring     
                           Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             dRy(q[2]),
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
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J3 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])
    
    ## third joint ##
    Td_local = np.linalg.multi_dot([Ty(q[0]),            # active joint
                           Ty(theta[0]),        # virtual spring     
                           Tz(links[0]),        # rigid link
                             Rz(q[1]),          # active joint
                             Rz(theta[1]),      # virtual spring
                             Tz(links[1]),
                             Ry(q[2]),
                             Ry(theta[2]),
                             Tz(links[2]),
                             dRy(q[3]),
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
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J4 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    ## third joint ##
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
                             dRz(q[4]),
                             Rz(theta[4]),
                             Tz(links[4]),
                             Ry(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])])
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J5 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    ## third joint ##
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
                             dRy(q[5]),
                             Ry(theta[5]),
                             Tz(links[5]),
                             Rz(q[6]),
                             Rx(theta[6])])
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J6 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])


    ## third joint ##
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
                             dRz(q[6]),
                             Rx(theta[6])])
    
    Td_global = np.linalg.multi_dot([Td_local, R_prime])
    J7 = np.block([Td_global[0, 3], Td_global[1, 3], Td_global[2, 3], Td_global[2, 1], Td_global[0, 2], Td_global[1, 0]])

    J = np.transpose(np.vstack([J1, J2, J3, J4, J5, J6, J7]))
    return J  


thetas = np.zeros(7, dtype=float)
d=0.15
K_a = 10**6
params = { 'A':np.pi*(d**2)/4, 'E':70*10**9, 'G':25.2*10**9, 'l':1, 'Ix':np.pi*(d**4)/64, 'Iy':np.pi*(d**4)/64, 'Iz':np.pi*(d**4)/64, 'Ip':np.pi*(d**4)/32}


qs = [0.2, np.pi / 4, np.pi / 3, np.pi / 2, np.pi / 3, np.pi / 2, np.pi / 4]
link_length = [0.200, 0.250, 0.250, 0.100, 0.1, 0.1]


H = FK_vjm(qs, link_length, thetas)
J_passive = Jacobian_passive(qs, link_length, thetas, H)
J_virtual = Jacobian_virtual(qs, link_length, thetas, H)

def KThetaLeg(K_active, E, G, d, link):
    K0 = np.zeros(7, dtype=float)
    K0[0] = K_active

    zeros_6_1 = np.zeros((6,1), dtype=float)

    K1 = elementStiffness22(E, G, d[0], link[0])
    K1 = np.hstack([zeros_6_1, K1])

    K = np.vstack([K0, K1])
    return K

def elementStiffness22(E, G, d, link):
    S = np.pi*(d**2)/4
    Iy = np.pi*(d**4)/64
    Iz = np.pi*(d**4)/64
    J = Iy + Iz
    
    K = np.array([[E*S/link,                 0,                 0,        0,                 0,                0],
                  [       0, 12*E*Iz/(link**3),                 0,        0,                 0, -6*E*Iz/(link**2)],
                  [       0,                 0, 12*E*Iy/(link**3),        0, 6*E*Iy/(link**2),                0],
                  [       0,                 0,                 0, G*J/link,                 0,                0],
                  [       0,                 0, 6*E*Iy/(link**2),        0,       4*E*Iy/link,                0],
                  [       0, -6*E*Iz/(link**2),                 0,        0,                 0,      4*E*Iz/link]], dtype=float)
    
    return K

E = 7.0000e+10 # Young's modulus
G = 2.5500e+10 # shear modulus
d = np.array([0.15, 0.15], dtype=float) # links diameter

K_active = 1000000 # actuator stiffness
K_theta = KThetaLeg(K_active, E, G, d, link_length)

Kc0 = np.linalg.inv(np.linalg.multi_dot([J_virtual, np.linalg.inv(K_theta), np.transpose(J_virtual)]))
Kc = Kc0 - np.linalg.multi_dot([Kc0, J_passive, np.linalg.inv(np.linalg.multi_dot([np.transpose(J_passive), Kc0, J_passive])), np.transpose(J_passive), Kc0])
print(Kc)