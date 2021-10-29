import numpy as np
from numpy import pi
from matplotlib import pyplot as plt
from point import Point
FREQ = 3


class Joint:
    def __init__(self, positions, dq_max, ddq_max, trajectory_junction_ratio=0, do_numerical_control=False):
        self.positions = positions
        self.dq_max = dq_max
        self.ddq_max = ddq_max
        self.trajectory_junction_ratio = trajectory_junction_ratio

        # Format of one trajectory: [t0, t1, T, tf]
        self.trajectories = []
        self.calculate_path(t0=0)
        if do_numerical_control:
            self.numerical_control()


# -----------------------------------------------------------------------
# ----------------------- Additional functions --------------------------
# -----------------------------------------------------------------------


    def __recalculate_positions_after_junction(junction_point, trajectory, qf, dq_max, ddq_max):
        t0_point, t1_point, T_point, tf_point = trajectory
        t, q, v, a = Point.get_point_to_point_plot_line(
            junction_point, t1_point)
        t1_point.q = q[-1]
        dq = qf - q[-1]
        dq_max = np.sign(dq) * abs(dq_max)
        ddq_max = np.sign(dq) * abs(ddq_max)

        T = t1_point.t + (dq - 0.5 * ddq_max *
                          (t1_point.t - t0_point.t)**2)/dq_max
        T_point.t = T
        T_point.q = q[-1] + (T - t1_point.t) * dq_max
        tf_point.t = T + t1_point.t - t0_point.t

    def recalculate_velocities_and_positions(self):
        # Recalculate velocities
        for i in range(len(self.trajectories)):
            trajectory = self.trajectories[i]
            new_v = (self.positions[i+1] - self.positions[i]
                     )/(trajectory[2].t - trajectory[0].t)
            trajectory[1].v = new_v
            trajectory[2].v = new_v

        # Recalculate positions
        for i in range(len(self.trajectories)):
            trajectory = self.trajectories[i]
            dq_m = trajectory[1].v
            ddq_m = (trajectory[1].v - trajectory[0].v) / \
                (trajectory[1].t - trajectory[0].t)

            if self.trajectory_junction_ratio > 0 and i > 0:
                Joint.__recalculate_positions_after_junction(
                    self.trajectories[i-1][2], trajectory, self.positions[i+1], dq_m, ddq_m)
            else:
                trajectory[1].q = trajectory[0].q + \
                    (0.5*ddq_m*(trajectory[1].t - trajectory[0].t)**2)
                trajectory[2].q = trajectory[1].q + trajectory[1].v * \
                    (trajectory[2].t - trajectory[1].t)

    def get_path_points(self):
        points = [point for trajectory in self.trajectories for point in trajectory]
        if self.trajectory_junction_ratio == 0:
            return points

        path_points = [points[0]]
        for i in range(len(points)):
            if i % 4 == 1 or i % 4 == 2:
                path_points.append(points[i])
        path_points.append(points[-1])

        return path_points

    def get_all_points_in_time_order(self):
        points = []
        points.append(self.trajectories[0][0])
        points.append(self.trajectories[0][1])
        points.append(self.trajectories[0][2])
        for i in range(1, len(self.trajectories)):
            points.append(self.trajectories[i][0])
            points.append(self.trajectories[i-1][3])
            points.append(self.trajectories[i][1])
            points.append(self.trajectories[i][2])
        points.append(self.trajectories[-1][3])
        return points

    def __plot(self, X, Y, index, name, vlines=[], hlines=[]):
        plt.subplot(1, 3, index)
        plt.plot(X, Y, linewidth=2, label=name, c='b')
        plt.xlabel('t (s)', fontsize=18)
        plt.ylabel(f'{name}(t) ($\degree$/s)', fontsize=18)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([min(X), max(X)])
        plt.ylim([min(Y), 1.1 * max(Y)])
        if vlines:
            plt.vlines(vlines, min(Y), 1.1 * max(Y),
                       linestyles='--', linewidth=2)
        if hlines:
            plt.hlines(hlines, min(Y), 1.1 * max(Y),
                       linestyles='--', linewidth=2)

    def get_path_points_for_plot(self):
        points = self.get_path_points()

        T, Q, V, A = [], [], [], []
        for i in range(1, len(points)):
            t, q, v, a = Point.get_point_to_point_plot_line(
                points[i-1], points[i])
            T = np.concatenate((T, t))
            Q = np.concatenate((Q, q))
            V = np.concatenate((V, v))
            A = np.concatenate((A, a))

        return T, Q, V, A


# -----------------------------------------------------------------------
# -------------------------- Main functions -----------------------------
# -----------------------------------------------------------------------


    def calculate_trajectory(q0, qf, dq_m, ddq_m, t0):
        dq = abs(qf-q0)
        # triangular check
        c = np.sqrt(dq*ddq_m)

        if c <= dq_m:
            t1 = np.sqrt(dq/ddq_m)
            dq_m = ddq_m * t1
            T = t1
            tf = 2*t1
        else:
            t1 = dq_m/ddq_m
            T = dq/dq_m
            tf = T+t1

        dq_m *= np.sign(qf-q0)
        ddq_m *= np.sign(qf-q0)

        t0_point = Point(t0, q0, 0)
        t1_point = Point(t0 + t1, q0 + (0.5*ddq_m*t1**2),
                         dq_m)
        T_point = Point(t0 + T, q0 + (0.5*ddq_m*t1**2) +
                        dq_m*(T-t1), dq_m)
        tf_point = Point(t0 + tf, qf, 0)
        return t0_point, t1_point, T_point, tf_point

    def calculate_path(self, t0=0):
        self.trajectories = []
        for i in range(1, len(self.positions)):
            t0_point, t1_point, T_point, tf_point = Joint.calculate_trajectory(
                self.positions[i-1], self.positions[i], self.dq_max, self.ddq_max, t0)

            # Recalculate points for trajectory junction
            if self.trajectory_junction_ratio > 0 and len(self.trajectories):
                traj = [t0_point, t1_point, T_point, tf_point]
                Joint.__recalculate_positions_after_junction(
                    self.trajectories[-1][2], traj, self.positions[i], self.dq_max, self.ddq_max)

            self.trajectories.append([t0_point, t1_point, T_point, tf_point])
            t0 = T_point.t + (1-self.trajectory_junction_ratio) * \
                (tf_point.t - T_point.t)

    def numerical_control(self):
        # Get points in correct order
        points = self.get_all_points_in_time_order()

        # Recalculate times
        prev_time, prev_time_modified = 0, 0
        for point in points:
            t_modified = prev_time_modified + \
                np.ceil((point.t - prev_time)/FREQ) * FREQ
            prev_time = point.t
            point.t = t_modified
            prev_time_modified = t_modified

        self.recalculate_velocities_and_positions()

    def synchronization(joints):
        points = []
        for joint in joints:
            joint_points = joint.get_all_points_in_time_order()
            points.append(joint_points)

        res_point_times = [points[0][0].t]
        for i in range(1, len(points[0])):
            max_time = 0
            for j in range(len(points)):
                max_time = max(max_time, points[j][i].t - points[j][i-1].t)

            res_point_times.append(res_point_times[-1] + max_time)

        for i in range(len(res_point_times)):
            for j in range(len(points)):
                points[j][i].t = res_point_times[i]

        for joint in joints:
            joint.recalculate_velocities_and_positions()

    def plot_path_points(self, print_points=False, figsize=(16, 5), convert_to_degrees=True):
        if print_points:
            points = self.get_path_points()
            Point.print_points(points)
            # Point.print_points([point for trajectory in self.trajectories for point in trajectory])

        T, Q, V, A = self.get_path_points_for_plot()

        if convert_to_degrees:
            def degrees(rad): return rad/pi * 180
            Q = list(map(degrees, Q))
            V = list(map(degrees, V))
            A = list(map(degrees, A))

        plt.figure(figsize=figsize)
        self.__plot(T, Q, 1, 'q')
        self.__plot(T, V, 2, 'v')
        self.__plot(T, A, 3, 'a')
        plt.show()
