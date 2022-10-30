import numpy as np
import matplotlib.pyplot as plt


### Transformations ###

def Rx(q):
    T = np.array([[1,         0,          0, 0],
                  [0, np.cos(q), -np.sin(q), 0],
                  [0, np.sin(q),  np.cos(q), 0],
                  [0,         0,          0, 1]], dtype=float)
    return T


def dRx(q):
    T = np.array([[0,          0,          0, 0],
                  [0, -np.sin(q), -np.cos(q), 0],
                  [0,  np.cos(q), -np.sin(q), 0],
                  [0,          0,          0, 0]], dtype=float)
    return T


def Ry(q):
    T = np.array([[np.cos(q), 0, np.sin(q), 0],
                  [0, 1,         0, 0],
                  [-np.sin(q), 0, np.cos(q), 0],
                  [0, 0,         0, 1]], dtype=float)
    return T


def dRy(q):
    T = np.array([[-np.sin(q), 0,  np.cos(q), 0],
                  [0, 0,          0, 0],
                  [-np.cos(q), 0, -np.sin(q), 0],
                  [0, 0,          0, 0]], dtype=float)
    return T


def Rz(q):
    T = np.array([[np.cos(q), -np.sin(q), 0, 0],
                  [np.sin(q),  np.cos(q), 0, 0],
                  [0,          0, 1, 0],
                  [0,          0, 0, 1]], dtype=float)
    return T


def dRz(q):
    T = np.array([[-np.sin(q), -np.cos(q), 0, 0],
                  [np.cos(q), -np.sin(q), 0, 0],
                  [0,          0,  0, 0],
                  [0,          0,  0, 0]], dtype=float)
    return T


def Tx(x):
    T = np.array([[1, 0, 0, x],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)
    return T


def dTx(x):
    T = np.array([[0, 0, 0, 1],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]], dtype=float)
    return T


def Ty(y):
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, y],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)
    return T


def dTy(y):
    T = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]], dtype=float)
    return T


def Tz(z):
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]], dtype=float)
    return T


def dTz(z):
    T = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, 0]], dtype=float)
    return T


### Beam stiffness ###

def elementStiffness11(E, G, d, link):
    S = np.pi*(d**2)/4
    Iy = np.pi*(d**4)/64
    Iz = np.pi*(d**4)/64
    J = Iy + Iz

    K = np.array([[E*S/link,                 0,                 0,        0,                 0,                0],
                  [0, 12*E*Iz/(link**3),                 0,
                   0,                 0, 6*E*Iz/(link**2)],
                  [0,                 0, 12*E*Iy /
                      (link**3),        0, -6*E*Iy/(link**2),                0],
                  [0,                 0,                 0, G*J /
                      link,                 0,                0],
                  [0,                 0, -6*E*Iy /
                      (link**2),        0,       4*E*Iy/link,                0],
                  [0,  6*E*Iz/(link**2),                 0,        0,                 0,      4*E*Iz/link]], dtype=float)

    return K


def elementStiffness12(E, G, d, link):
    S = np.pi*(d**2)/4
    Iy = np.pi*(d**4)/64
    Iz = np.pi*(d**4)/64
    J = Iy + Iz

    K = np.array([[-E*S/link,                 0,                 0,        0,                 0,                0],
                  [0, -12*E*Iz/(link**3),                 0,
                   0,                 0, 6*E*Iz/(link**2)],
                  [0,                 0, -12*E*Iy /
                      (link**3),        0, -6*E*Iy/(link**2),                0],
                  [0,                 0,                 0, -G *
                      J/link,                 0,                0],
                  [0,                 0, 6*E*Iy /
                      (link**2),        0,       2*E*Iy/link,                0],
                  [0,  -6*E*Iz/(link**2),                 0,        0,                 0,      2*E*Iz/link]], dtype=float)

    return K


def elementStiffness22(E, G, d, link):
    S = np.pi*(d**2)/4
    Iy = np.pi*(d**4)/64
    Iz = np.pi*(d**4)/64
    J = Iy + Iz

    K = np.array([[E*S/link,                 0,                 0,        0,                 0,                0],
                  [0, 12*E*Iz/(link**3),                 0,
                   0,                 0, -6*E*Iz/(link**2)],
                  [0,                 0, 12*E*Iy /
                      (link**3),        0, 6*E*Iy/(link**2),                0],
                  [0,                 0,                 0, G*J /
                      link,                 0,                0],
                  [0,                 0, 6*E*Iy /
                      (link**2),        0,       4*E*Iy/link,                0],
                  [0, -6*E*Iz/(link**2),                 0,        0,                 0,      4*E*Iz/link]], dtype=float)

    return K


class DeltaRobot:
    def __init__(self, f, L, l):
        self.f = f
        self.L = L
        self.l = l
        self.R = [np.eye(4), Rz(2*np.pi/3), Rz(-2*np.pi/3)]

    def forwardLeg(self, R, q, theta=np.zeros(13)):
        return (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                Rx(theta[0]) @  # 1 DOF virtual spring
                Ty(-self.L) @
                # 6 DOF virtual spring
                Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                )

    def forward(self, qs, theta):
        T = []
        for i in range(len(qs)):
            T.append(self.forwardLeg(self.R[i], qs[i], theta[i]))
        return T

    def inverseLeg(self, J1, J3):
        J1x, J1y, J1z = J1
        J3x, J3y, J3z = J3

        x1, x2 = J1y, J3y
        y1, y2 = J1z, J3z
        r1, r2 = self.L, np.sqrt(self.l**2 - J3x**2)
        d = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        l = (r1**2 - r2**2 + d**2)/(2*d)
        h = np.sqrt(r1**2 - l**2)

        J2x = 0
        J2y = l/d * (x2-x1) + h/d * (y2 - y1) + x1
        J2z = l/d * (y2 - y1) - h/d * (x2 - x1) + y1
        q1 = np.arctan2(J2z-J1z, J2y-J1y) + np.pi

        a = np.sqrt((J1y - J3y)**2 + (J1z - J3z)**2)
        b = np.sqrt((J2y - J3y)**2 + (J2z - J3z)**2)
        alpha = np.arccos((b**2 + a**2 - self.L**2)/(2 * a * b))
        beta = np.arccos((self.L**2 + a**2 - b**2)/(2 * self.L*a))
        q2 = alpha + beta

        a = np.sqrt((J1y - J3y)**2 + (J1z - J3z)**2)
        b = np.sqrt((J2y - J3y)**2 + (J2z - J3z)**2)
        q3 = np.arcsin(J3x/self.l)

        return q1, q2, q3

    def inverse(self, x0, y0, z0):
        J3_1 = [x0, y0, z0]
        J3_2 = J3_1 @ Rz(2*np.pi/3)[:3, :3]
        J3_3 = J3_1 @ Rz(-2*np.pi/3)[:3, :3]

        J1 = [0, -self.f/(2*np.sqrt(3)), 0]
        qs = []
        qs.append(self.inverseLeg(J1, J3_1))
        qs.append(self.inverseLeg(J1, J3_2))
        qs.append(self.inverseLeg(J1, J3_3))
        return qs

    def transformStiffness(self, qs):
        Q = []
        for i in range(len(qs)):
            toLink1 = self.R[i] @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(qs[i][0])
            rotationLink1 = toLink1[0:3, 0:3]

            toLink2 = toLink1 @ Ty(-self.L) @ Rx(qs[i][1]) @ Rz(qs[i][2])
            rotationLink2 = toLink2[0:3, 0:3]

            zeros = np.zeros((3, 3), dtype=float)

            Q1 = np.vstack([np.hstack([rotationLink1,         zeros]),
                            np.hstack([zeros, rotationLink1])])

            Q2 = np.vstack([np.hstack([rotationLink2,         zeros]),
                            np.hstack([zeros, rotationLink2])])

            Q.append([Q1, Q2])
        return Q

    ### VJM section ###

    def JacobianPassiveLeg(self, T_fk, R, q, theta):
        T_fk[0:3, 3] = 0
        inv_T_fk = np.transpose(T_fk)

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ dRx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J1 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        dRx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J2 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ dRz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J3 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        J = np.hstack([J1, J2, J3])
        return J

    def JacobianPassive(self, qs, theta):
        T_fk = self.forward(qs, theta)

        Jq = []
        for leg in range(len(qs)):
            J = self.JacobianPassiveLeg(
                T_fk[leg], self.R[leg], qs[leg], theta[leg])
            Jq.append(J)
        return Jq

    def JacobianThetaLeg(self, T_fk, R, q, theta):
        T_fk[0:3, 3] = 0
        inv_T_fk = np.transpose(T_fk)

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        dRx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J1 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        dTx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J2 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ dTy(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J3 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ dTz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J4 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ dRx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J5 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ dRy(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J6 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ dRz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J7 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        dTx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                         ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J8 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ dTy(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                         ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J9 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ dTz(theta[9]) @ Rx(theta[10]
                                                                         ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J10 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ dRx(theta[10]
                                                                         ) @ Ry(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J11 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ dRy(theta[11]) @ Rz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J12 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        dT_leg_local = (R @ Ty(-self.f/(2*np.sqrt(3))) @ Rx(q[0]) @
                        Rx(theta[0]) @  # 1 DOF virtual spring
                        Ty(-self.L) @
                        # 6 DOF virtual spring
                        Tx(theta[1]) @ Ty(theta[2]) @ Tz(theta[3]) @ Rx(theta[4]) @ Ry(theta[5]) @ Rz(theta[6]) @
                        Rx(q[1]) @ Rz(q[2]) @ Ty(-self.l) @
                        Tx(theta[7]) @ Ty(theta[8]) @ Tz(theta[9]) @ Rx(theta[10]
                                                                        ) @ Ry(theta[11]) @ dRz(theta[12])  # 6 DOF virtual spring
                        )
        dT_leg = dT_leg_local @ inv_T_fk
        J13 = np.vstack([dT_leg[0, 3], dT_leg[1, 3], dT_leg[2, 3],
                        dT_leg[2, 1], dT_leg[0, 2], dT_leg[1, 0]])

        J = np.hstack([J1, J2, J3, J4, J5, J6, J7, J8, J9, J10, J11, J12, J13])
        return J

    def JacobianTheta(self, qs, theta):
        T_fk = self.forward(qs, theta)

        Jtheta = []
        for leg in range(len(qs)):
            J = self.JacobianThetaLeg(
                T_fk[leg], self.R[leg], qs[leg], theta[leg])
            Jtheta.append(J)
        return Jtheta


### VJM section ###

def KThetaLeg(K_active, E, G, d, link):
    K0 = np.zeros(13, dtype=float)
    K0[0] = K_active

    zeros_6_1 = np.zeros((6, 1), dtype=float)
    zeros_6_6 = np.zeros((6, 6), dtype=float)

    K1 = elementStiffness22(E, G, d[0], link[0])
    K1 = np.hstack([zeros_6_1, K1, zeros_6_6])

    K2 = elementStiffness22(E, G, d[1], link[1])
    K2 = np.hstack([zeros_6_1, zeros_6_6, K2])

    K = np.vstack([K0, K1, K2])
    return K


def Kc_VJM(Ktheta, Jq, Jtheta):
    Kc_total = []
    for i in range(len(Ktheta)):
        Kc0 = np.linalg.inv(np.linalg.multi_dot(
            [Jtheta[i], np.linalg.inv(Ktheta[i]), np.transpose(Jtheta[i])]))
        Kc = Kc0 - np.linalg.multi_dot([Kc0, Jq[i], np.linalg.inv(np.linalg.multi_dot(
            [np.transpose(Jq[i]), Kc0, Jq[i]])), np.transpose(Jq[i]), Kc0])
        Kc_total.append(Kc)

    Kc_total = Kc_total[0] + Kc_total[1] + Kc_total[2]
    return Kc_total


### MSA section ###

lambda_r_12 = np.array([[1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]], dtype=float)
lambda_e_12 = np.array([0, 0, 0, 1, 0, 0], dtype=float)


lambda_r_34 = np.array([[1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0]], dtype=float)
lambda_p_34 = np.array([[0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 1]], dtype=float)


def Kc_MSA(Q, K_active, K1, K2, lambda_e_12, lambda_r_12, lambda_r_34, lambda_p_34):
    Kc = []
    for i in range(len(Q)):
        # Rigid connection 1
        # Equation 1
        eq1 = np.hstack([np.zeros((6, 6*5), dtype=float),
                        np.eye(6, dtype=float), np.zeros((6, 6*4), dtype=float)])
        Q1 = Q[i][0]
        K1_11 = np.linalg.multi_dot([Q1, K1[0][0], np.transpose(Q1)])
        K1_12 = np.linalg.multi_dot([Q1, K1[0][1], np.transpose(Q1)])
        K1_21 = np.linalg.multi_dot([Q1, K1[1][0], np.transpose(Q1)])
        K1_22 = np.linalg.multi_dot([Q1, K1[1][1], np.transpose(Q1)])

        # Flexible link 2-3
        # Equation 2
        eq2 = np.hstack([np.zeros((6, 6*1), dtype=float), -np.eye(6, dtype=float), np.zeros((6, 6*3), dtype=float),
                        np.zeros((6, 6*1), dtype=float), K1_11, K1_12, np.zeros((6, 6*2), dtype=float)])

        # Equation 3
        eq3 = np.hstack([np.zeros((6, 6*2), dtype=float), -np.eye(6, dtype=float), np.zeros((6, 6*2), dtype=float),
                        np.zeros((6, 6*1), dtype=float), K1_21, K1_22, np.zeros((6, 6*2), dtype=float)])

        Q2 = Q[i][1]
        K2_11 = np.linalg.multi_dot([Q2, K2[0][0], np.transpose(Q2)])
        K2_12 = np.linalg.multi_dot([Q2, K2[0][1], np.transpose(Q2)])
        K2_21 = np.linalg.multi_dot([Q2, K2[1][0], np.transpose(Q2)])
        K2_22 = np.linalg.multi_dot([Q2, K2[1][1], np.transpose(Q2)])

        # Flexible link 3-4
        # Equation 4
        eq4 = np.hstack([np.zeros((6, 6*3), dtype=float), -np.eye(6, dtype=float), np.zeros((6, 6*1), dtype=float),
                        np.zeros((6, 6*3), dtype=float), K2_11, K2_12])

        # Equation 5
        eq5 = np.hstack([np.zeros((6, 6*4), dtype=float), -np.eye(6, dtype=float),
                         np.zeros((6, 6*3), dtype=float), K2_21, K2_22])

        # Elastic joint 1-2
        # Equation 6
        eq6 = np.hstack([np.zeros((5, 6*5), dtype=float),
                         lambda_r_12, -lambda_r_12, np.zeros((5, 6*3), dtype=float)])

        # Equation 7
        eq7 = np.hstack([np.eye(6, dtype=float), np.eye(
            6, dtype=float), np.zeros((6, 6*8), dtype=float)])

        # Equation 8
        eq8 = np.hstack([lambda_e_12, np.zeros((6*4), dtype=float),
                        K_active * lambda_e_12, -K_active*lambda_e_12, np.zeros((6*3), dtype=float)])

        # Passive joint 3-4
        # Equation 9
        eq9 = np.hstack([np.zeros((4, 6*5), dtype=float),
                         np.zeros((4, 6*2), dtype=float), lambda_r_34, -lambda_r_34, np.zeros((4, 6*1), dtype=float)])

        # Equation 10
        eq10 = np.hstack([np.zeros((4, 6*2), dtype=float), lambda_r_34, lambda_r_34,
                          np.zeros((4, 6*6), dtype=float)])

        # Equation 11
        eq11 = np.hstack([np.zeros((2, 6*2), dtype=float), lambda_p_34,
                          np.zeros((2, 6*7), dtype=float)])
        # Equation 12
        eq12 = np.hstack([np.zeros((2, 6*3), dtype=float), lambda_p_34,
                          np.zeros((2, 6*6), dtype=float)])

        # External loading equation
        # Equation 13
        eq13 = np.hstack([np.zeros((6, 6*4), dtype=float), -np.eye(6, dtype=float),
                          np.zeros((6, 6*5), dtype=float)])

        # Aggregated matrix
        agg = np.vstack([eq1, eq2, eq3, eq4, eq5, eq6, eq7,
                        eq8, eq9, eq10, eq11, eq12, eq13])
        A = agg[0:54, 0:54]
        B = agg[0:54, 54:]
        C = agg[54:, 0:54]
        D = agg[54:, 54:]

        K_leg = D - np.linalg.multi_dot([C, np.linalg.inv(A), B])
        Kc.append(K_leg)
    Kc = Kc[0] + Kc[1] + Kc[2]
    return Kc


### Plotting ###

def plotDeflection(x, y, z, deflection, space):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlim3d(space[0][0], space[0][1])
    ax.set_ylim3d(space[1][0], space[1][1])
    ax.set_zlim3d(space[2][0], space[2][1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    r = [0, 1]
    X, Y = np.meshgrid(r, r)
    ones = np.ones(4).reshape(2, 2)
    zeros = np.zeros(4).reshape(2, 2)
    ax.plot_wireframe(X, Y, ones, alpha=0.5, color='slategray')
    ax.plot_wireframe(X, Y, zeros, alpha=0.5, color='slategray')
    ax.plot_wireframe(X, zeros, Y, alpha=0.5, color='slategray')
    ax.plot_wireframe(X, ones, Y, alpha=0.5, color='slategray')
    ax.plot_wireframe(ones, X, Y, alpha=0.5, color='slategray')
    ax.plot_wireframe(zeros, X, Y, alpha=0.5, color='slategray')

    cmap = plt.cm.get_cmap('RdGy_r', 12)
    cmap = ax.scatter3D(x, y, z, c=deflection, cmap=cmap, s=50)

    plt.colorbar(cmap)
    ax.view_init(elev=30, azim=-40)
    plt.show()


def is_singular(qs):
    for i in range(len(qs)):
        for j in range(len(qs[i])):
            if np.isnan(qs[i][j]):
                return True
    return False


if __name__ == "__main__":
    space_x = [-150, 150]
    space_y = [-150, 150]
    space_z = [-400, -200]
    space = [space_x, space_y, space_z]
    link = np.array([170, 300], dtype=float)  # links length
    d = np.array([3, 3], dtype=float)  # links diameter

    delta = DeltaRobot(f=200*np.sqrt(3), L=link[0], l=link[1])

    theta = np.zeros(13, dtype=float)
    theta = [theta, theta, theta]

    K_active = 100000  # actuator stiffness
    E = 7.0000e+10  # Young's modulus
    G = 2.5500e+10  # shear modulus

    # VJM parameters
    Ktheta = KThetaLeg(K_active, E, G, d, link)
    Ktheta = [Ktheta, Ktheta, Ktheta]

    # MSA parameters
    K1_11 = elementStiffness11(E, G, d[0], link[0])
    K1_12 = elementStiffness12(E, G, d[0], link[0])
    K1_21 = np.transpose(K1_12)
    K1_22 = elementStiffness22(E, G, d[0], link[0])
    K1 = [[K1_11, K1_12], [K1_21, K1_22]]

    K2_11 = elementStiffness11(E, G, d[1], link[1])
    K2_12 = elementStiffness12(E, G, d[1], link[1])
    K2_21 = np.transpose(K2_12)
    K2_22 = elementStiffness22(E, G, d[1], link[1])
    K2 = [[K2_11, K2_12], [K2_21, K2_22]]

    # Applied force
    F = np.array([[1000], [0], [0], [0], [0], [0]], dtype=float)

    method = 'MSA'

    xScatter = np.array([])
    yScatter = np.array([])
    zScatter = np.array([])
    dScatter = np.array([])

    for z in np.arange(space_z[0], space_z[1], 30):
        xData = np.array([])
        yData = np.array([])
        zData = np.array([])
        dData = np.array([])
        print("z:", z)

        for x in np.arange(space_x[0], space_x[1], 30):
            for y in np.arange(space_y[0], space_y[1], 30):
                p_global = np.array([x, y, z], dtype=float)
                qs = delta.inverse(x, y, z)
                if is_singular(qs):
                    continue

                if method == 'VJM':
                    Jq = delta.JacobianPassive(qs, theta)
                    Jtheta = delta.JacobianTheta(qs, theta)
                    Kc = Kc_VJM(Ktheta, Jq, Jtheta)
                elif method == 'MSA':
                    Q = delta.transformStiffness(qs)
                    Kc = Kc_MSA(Q, K_active, K1, K2,
                                lambda_e_12, lambda_r_12, lambda_r_34, lambda_p_34)

                # Get deflection
                dt = np.linalg.inv(Kc).dot(F)
                deflection = np.sqrt(dt[0]**2 + dt[1]**2 + dt[2]**2)

                xData = np.append(xData, x)
                yData = np.append(yData, y)
                zData = np.append(zData, z)
                dData = np.append(dData, deflection)

        xScatter = np.append(xScatter, xData)
        yScatter = np.append(yScatter, yData)
        zScatter = np.append(zScatter, zData)
        dScatter = np.append(dScatter, dData)

    print('Range: ', np.min(dScatter), np.max(dScatter))
    plotDeflection(xScatter, yScatter, zScatter, dScatter, space)
