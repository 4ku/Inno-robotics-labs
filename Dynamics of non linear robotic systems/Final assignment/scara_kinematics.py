from matrix import *


class ScaraKinematics():

    def extract_vectors_from_trans(trans):
        x, y, z = trans[0:3, 3]
        p = [x, y, z]
        v1 = trans[0:3, 0]
        v2 = trans[0:3, 1]
        v3 = trans[0:3, 2]

        return p, [v1, v2, v3]


    def __init__(self):
        super().__init__()

        self.Lbase, self.L1, self.L2, self.L3 = 1, 1, 1, 1
        p_tool = [1, 1, 1]
        q1, q2, q4, q5, q6, q7 = 0, 0, 0, 0, 0, 0

        self.Tbase = Tz(self.Lbase)
        self.R1 = Rz(q1)
        self.T12 = Tx(self.L1)
        self.R2 = Rz(q2)
        self.T23 = Tx(self.L2)
        self.T35 = Tz(-self.L3-q4)
        self.R5 = Ry(q5)
        self.R6 = Rz(q6)
        self.R7 = Rx(q7)
        self.T7_tool = T(p_tool)

        self.A_res = self.Tbase * self.R1 * self.T12 * self.R2 * \
            self.T23 * self.T35 * self.R5 * self.R6 * self.R7 * self.T7_tool

    def forward(self, q1, q2, q4, q5, q6, q7):

        self.R1 = Rz(q1)
        self.R2 = Rz(q2)
        self.T35 = Tz(-self.L3-q4)
        self.R5 = Ry(q5)
        self.R6 = Rz(q6)
        self.R7 = Rx(q7)

        A1 = self.Tbase * self.R1
        A2 = self.T12 * self.R2
        A3 = self.T23
        Awrist = self.T35 * self.R5 * self.R6 * self.R7
        Atool = self.T7_tool

        A = [A1, A2, A3, Awrist, Atool]
        states = [Matrix(np.eye(4))]
        for a in A:
            states.append(states[-1] * a)

        self.A_res = states[-1]

        return states

    def inverse(self, A_res):
        A07 = A_res * np.linalg.inv(self.T7_tool.val)
        position, rotations = ScaraKinematics.extract_vectors_from_trans(A07.val)
        px, py, pz = position

        cos_q2 = (px**2 + py**2 - (self.L1**2 + self.L2**2)) / \
            (2*self.L1*self.L2)
        cos_q2 = np.clip(cos_q2, -1.0, 1.0)
        sin_q2 = (1-cos_q2**2)**0.5

        alpha = np.arctan2(py, px)
        beta = np.arctan2(self.L2*sin_q2, self.L1 + self.L2*cos_q2)
        q1 = alpha - beta
        q2 = np.arctan2(sin_q2, cos_q2)
        q4 = self.Lbase - self.L3 - pz

        A05 = self.Tbase * Rz(q1) * self.T12 * Rz(q2) * \
            self.T23 * Tz(-self.L3-q4)
        R57 = np.linalg.inv(A05.val) * A07
        R57 = R57.val
        if R57[0][0] == 0 and R57[2][0] == 0:
            if R57[1][0] < 0:
                q6 = -np.pi/2
            else:
                q6 = np.pi/2
            q7 = 0
            q5 = np.arctan2(R57[0][2], R57[2][2])
        else:
            q5 = np.arctan2(-R57[2][0], R57[0][0])
            q7 = np.arctan2(-R57[1][2], R57[1][1])
            q6 = np.arctan2(np.cos(q7) * R57[1][0], R57[1][1])
        return q1, q2, q4, q5, q6, q7
