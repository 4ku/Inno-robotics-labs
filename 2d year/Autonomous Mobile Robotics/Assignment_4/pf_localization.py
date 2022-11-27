import numpy as np
import scipy.stats
from numpy.random import uniform
import matplotlib.pyplot as plt
from math import cos, sin, atan2
from numpy.random import randn

from dubins_path import DubinsPath, SimpleCar


class PFLocalization():
    def __init__(self, dt):
        self.dt = dt

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

    def create_gaussian_particles(self, mean, std, N):
        particles = np.empty((N, 3))
        particles[:, 0] = mean[0] + (randn(N) * std[0])
        particles[:, 1] = mean[1] + (randn(N) * std[1])
        particles[:, 2] = mean[2] + (randn(N) * std[2])
        particles[:, 2] %= 2 * np.pi
        return particles

    def create_uniform_particles(self, x_range, y_range, hdg_range, N):
        particles = np.empty((N, 3))
        particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
        particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
        particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
        particles[:, 2] %= 2 * np.pi
        return particles

    def predict(self, particles, u, std, dt=1.):
        N = len(particles)
        if u[1] == 0:
            particles[:, 0] += dt*u[0]*np.cos(particles[:, 2])
            particles[:, 1] += dt*u[0]*np.sin(particles[:, 2])
            particles[:, 2] += (randn(N) * std[0])
            particles[:, 2] %= 2 * np.pi
        else:
            r = u[0]/u[1]
            theta = particles[:, 2]
            rotation = particles[:, 2] + u[1]*dt
            particles[:, 0] = particles[:, 0] + - \
                r*np.sin(theta) + r*np.sin(rotation)
            particles[:, 1] = particles[:, 1] + r * \
                np.cos(theta) - r*np.cos(rotation)
            particles[:, 2] = particles[:, 2] + u[1]*dt + (randn(N) * std[0])
            particles[:, 2] %= 2 * np.pi

        return particles

    def update(self, particles, weights, x, R, landmarks):
        weights.fill(1.)
        NL = len(landmarks)
        z = (np.linalg.norm(landmarks - x, axis=1) + (randn(NL) * R))
        for i, landmark in enumerate(landmarks):
            distance = np.linalg.norm(particles[:, 0:2]-landmark, axis=1)
            weights *= scipy.stats.norm(distance, R).pdf(z[i])

        weights += 1.e-300
        weights /= sum(weights)
        return weights

    def importance_sampling(self, particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights.fill(1.0 / len(weights))
        return weights

    def estimate(self, particles, weights):
        pos = particles[:, 0:2]
        mean = np.average(pos, weights=weights, axis=0)
        var = np.average((pos - mean)**2, weights=weights, axis=0)
        return mean, var

    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    def resample_particles(self, weights):
        N = len(weights)
        positions = (np.random.random() + np.arange(N)) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes

    def run_localization(self, dub_path, N, landmarks, iteration_num, sensor_std_err=.1, initial_x=None):
        plt.figure()
        # create particles and weights
        if initial_x is not None:
            particles = self.create_gaussian_particles(
                mean=initial_x, std=(5, 5, np.pi/4), N=N)
        else:
            particles = self.create_uniform_particles(
                (-20, 20), (-20, 20), (0, 6.28), N)
        weights = np.zeros(N)

        xs = []
        sim_pos = np.array([dub_path.start_pos]).T
        u, _ = dub_path.get_control(sim_pos[:2, 0])

        plt.scatter(particles[:, 0], particles[:, 1], alpha=0.1, color='b')
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=30)
        track = []
        for i in range(iteration_num):
            sim_pos = self.x_forward(sim_pos, u, self.dt)  # simulate robot
            track.append(sim_pos)

            u, finished = dub_path.get_control(sim_pos[:2, 0])
            if finished:
                print("End is reached")
                break

            particles = self.predict(
                particles, u=u, std=(.02, .05), dt=self.dt)
            # incorporate measurements
            weights = self.update(particles, weights,  sim_pos.flatten()[
                                  0:2], R=sensor_std_err, landmarks=landmarks)
            if self.neff(weights) < N/2:
                indexes = self.resample_particles(weights)
                weights = self.importance_sampling(particles, weights, indexes)

            mu, var = self.estimate(particles, weights)
            xs.append(mu)

        xs = np.array(xs)
        track = np.array(track)
        p1 = plt.scatter(track[:, 0], track[:, 1], color='k', s=10)
        p2 = plt.scatter(xs[:, 0], xs[:, 1], marker='s', s=1, color='r')
        plt.legend([p1, p2], ['Real', 'PF'], loc=4, numpoints=1)
        plt.xlim((-10, 30))
        plt.show()


start_pos = [0, 0, -np.pi/2]
end_pos = [20, 20, -np.pi/2]
car = SimpleCar(v=2, l=1.5, max_phi=np.pi / 10)
dub_path = DubinsPath(car, start_pos, end_pos)

dt = 0.01
landmarks = np.array([[5, 30], [5, -30], [-5, 0]])
pfl = PFLocalization(dt)
# pfl.run_localization(100, landmarks, initial_x=(1,1, np.pi/4), iteration_num=500)
pfl.run_localization(dub_path, 500, landmarks, initial_x=start_pos , iteration_num=3000)
