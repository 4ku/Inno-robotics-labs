from dubins_path import DubinsPath, SimpleCar
import numpy as np
import math
from matplotlib.patches import Ellipse
from copy import deepcopy
from math import sqrt
import matplotlib.pyplot as plt
from math import sqrt, tan, cos, sin, atan2
from numpy.random import randn
from numpy import array, sqrt
from IPython.display import display
import sympy
sympy.init_printing(use_latex='mathjax')


class EKFLocalization():
    def __init__(self, dt, vel, std_vel, std_steer,
                 std_range, std_bearing, dim_x=3, dim_z=2, dim_u=2):
        self.dt = dt
        self.vel = vel
        self.std_vel = std_vel
        self.std_steer = std_steer
        self.std_range = std_range
        self.std_bearing = std_bearing

        self.get_linearized_motion_model()
        self.subs = {self._x: 0, self._y: 0, self._vt: 0,
                     self._wt: 0, self._dt: dt, self._theta: 0}

        self.x = np.zeros((dim_x, 1))  # state
        self.P = np.diag([.1, .1, .1])        # uncertainty covariance
        self.R = np.eye(dim_z)        # state uncertainty
        self.Q = np.eye(dim_x)        # process uncertainty
        self.y = np.zeros((dim_z, 1))  # residual
        self.z = np.array([None]*dim_z)
        self.K = np.zeros(self.x.shape)  # kalman gain
        self._I = np.eye(dim_x)

        self.R[0, 0] = std_range**2
        self.R[1, 1] = std_bearing**2

    def get_linearized_motion_model(self):
        x, y, theta, vt, wt, dt = sympy.symbols(
            'x, y, theta, v_t, omega_t, delta_t')
        f = sympy.Matrix([
            [x - (vt/wt)*sympy.sin(theta) + (vt/wt)*sympy.sin(theta + wt*dt)],
            [y + (vt/wt)*sympy.cos(theta) - (vt/wt)*sympy.cos(theta + wt*dt)],
            [theta+wt*dt]
        ])
        self._x, self._y, self._theta, self._vt, self._wt, self._dt = x, y, theta, vt, wt, dt
        state = sympy.Matrix([x, y, theta])
        control = sympy.Matrix([vt, wt])
        self.F_j = f.jacobian(state)
        self.V_j = f.jacobian(control)

        # In case angular velocity equal to 0
        f = sympy.Matrix([
            [x + dt*vt*sympy.cos(theta)],
            [y + dt*vt*sympy.sin(theta)],
            [theta]
        ])
        self.F_j0 = f.jacobian(state)
        self.V_j0 = f.jacobian(control)

    def x_forward(self, x, u, dt):
        if u[1] == 0:
            x_plus = np.array([
                x[0] + dt*u[0]*np.cos(x[2]),
                x[1] + dt*u[0]*np.sin(x[2]),
                x[2]
            ])
        else:
            r = u[0]/u[1]
            theta = x[2]
            rotation = x[2] + u[1]*dt
            x_plus = np.array([x[0] + -r*sin(theta) + r*sin(rotation),
                               x[1] + r*cos(theta) - r*cos(rotation),
                               x[2] + u[1]*dt])
        return x_plus

    def get_linearized_measurement_model(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
        dist = sqrt(hyp)
        Hx = array([[dist], [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        H = array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
                  [(py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
        return Hx, H

    def residual(self, a, b):
        """ compute residual (a-b) between measurements containing 
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

    def predict(self, u):
        self.x = self.x_forward(self.x, u, self.dt)
        self.subs[self._x] = self.x[0, 0]
        self.subs[self._y] = self.x[1, 0]
        self.subs[self._theta] = self.x[2, 0]
        self.subs[self._vt] = u[0]
        self.subs[self._wt] = u[1]

        if u[1] == 0:
            F = array(self.F_j0.evalf(subs=self.subs)).astype(float)
            V = array(self.V_j0.evalf(subs=self.subs)).astype(float)
        else:        
            F = array(self.F_j.evalf(subs=self.subs)).astype(float)
            V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance in the control space
        M = array([[self.std_vel**2, 0],  [0, self.std_steer**2]])

        self.P = F @ self.P @ F.T + V @ M @ V.T + self.Q

    def ekf_update(self, z, landmarks):
        Hx, H = self.get_linearized_measurement_model(self.x, landmarks)
        PHT = np.dot(self.P, H.T)
        self.K = PHT.dot(np.linalg.inv(np.dot(H, PHT) + self.R))
        self.y = self.residual(z, Hx)
        self.x = self.x + np.dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # P = (I-KH)P is the optimal gain
        I_KH = self._I - np.dot(self.K, H)
        self.P = np.dot(I_KH, self.P).dot(I_KH.T) + \
            np.dot(self.K, self.R).dot(self.K.T)
        self.z = deepcopy(z)

    def z_landmark(self, lmark, sim_pos):
        x, y = sim_pos[0, 0], sim_pos[1, 0]
        d = np.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)
        a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
        z = np.array([[d + randn()*self.std_range],
                      [a + randn()*self.std_bearing]])
        return z

    def covariance_ellipse(self, P, deviations=1):
        U, s, _ = np.linalg.svd(P)
        orientation = math.atan2(U[1, 0], U[0, 0])
        width = deviations * math.sqrt(s[0])
        height = deviations * math.sqrt(s[1])
        if height > width:
            raise ValueError('width must be greater than height')
        return (orientation, width, height)

    def plot_covariance_ellipse(self, mean, cov, std=None, facecolor='b', edgecolor='g', alpha=0.7, ls='solid'):
        ellipse = self.covariance_ellipse(cov)
        ax = plt.gca()
        angle = np.degrees(ellipse[0])
        width = ellipse[1] * 2.
        height = ellipse[2] * 2.
        e = Ellipse(xy=mean, width=std*width, height=std*height, angle=angle,
                    facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, lw=2, ls=ls)
        ax.add_patch(e)
        x, y = mean
        plt.scatter(x, y, marker='+', color=edgecolor)
        a = ellipse[0]
        h, w = height/4, width/4
        plt.plot([x, x + h*cos(a+np.pi/2)], [y, y + h*sin(a+np.pi/2)])
        plt.plot([x, x + w*cos(a)], [y, y + w*sin(a)])

    def run_localization(self, dub_path, landmarks,
                         step=10, ellipse_step=2000, ylim=None, iteration_num=1000):
        self.x = array([dub_path.start_pos]).T
        sim_pos = self.x.copy()
        u, _ = dub_path.get_control(sim_pos[:2, 0])

        plt.figure()
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)
        sim_Pos = []
        filtered_pos = []
        for i in range(iteration_num):
            sim_pos = self.x_forward(sim_pos, u, self.dt)  # simulate robot
            sim_Pos.append(sim_pos)

            u, finished = dub_path.get_control(sim_pos[:2, 0])
            if finished:
                print("End is reached")
                break

            if i % step == 0:
                self.predict(u=u)
                if i % ellipse_step == 0:
                    self.plot_covariance_ellipse(
                        (self.x[0, 0], self.x[1, 0]), self.P[0:2, 0:2], std=6, facecolor='k', alpha=0.3)

                for lmark in landmarks:
                    z = self.z_landmark(lmark, sim_pos)
                    self.ekf_update(z, lmark)
                filtered_pos.append(self.x)
                if i % ellipse_step == 0:
                    self.plot_covariance_ellipse(
                        (self.x[0, 0], self.x[1, 0]), self.P[0:2, 0:2], std=6, facecolor='g', alpha=0.8)

        sim_Pos = np.array(sim_Pos)
        filtered_pos = np.array(filtered_pos)
        plt.plot(sim_Pos[:, 0], sim_Pos[:, 1], color='k', label = "Simulated", lw=2)
        plt.plot(filtered_pos[:, 0, 0], filtered_pos[:,1, 0], color='y', label = "Filtered", lw=2)

        plt.scatter(dub_path.start_pos[0], dub_path.start_pos[1], s=60, color='r')
        plt.scatter(dub_path.end_pos[0], dub_path.end_pos[1], s=60, color='b')
        plt.axis('equal')
        plt.title("EKF Robot localization")
        plt.legend()
        if ylim is not None:
            plt.ylim(*ylim)
        plt.show()
        return ekf

start_pos = [0, 0, -np.pi/2]
end_pos = [20, 20, -np.pi/2]
car = SimpleCar(v=2, l=1.5, max_phi=np.pi / 10)
dub_path = DubinsPath(car, start_pos, end_pos)

ekf = EKFLocalization(dt=0.1, vel=car.v, std_vel=5.1, std_steer=np.radians(1),
                      std_range=0.3, std_bearing=0.1)

landmarks = array([[5, 30], [5, -30], [-5, 0]])
ekf.run_localization(dub_path, landmarks,
                     ellipse_step=70, iteration_num=500)
