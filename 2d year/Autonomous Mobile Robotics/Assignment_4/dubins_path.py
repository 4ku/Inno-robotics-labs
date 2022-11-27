from math import tan, atan2, acos, pi
import numpy as np

from utils import transform, directional_theta, distance


class Params:
    """ Store parameters for different dubins paths. """

    def __init__(self, d):

        self.d = d      # dubins type
        self.t1 = None  # first tangent point
        self.t2 = None  # second tangent point
        self.c1 = None  # first center point
        self.c2 = None  # second center point
        self.len = None  # total travel distance


class SimpleCar:
    def __init__(self, v=2, l=1.5, max_phi=pi/5):
        self.l = float(l)  # length
        self.v = v  # velocity
        self.max_phi = max_phi
        self.r = self.l / tan(self.max_phi)

class DubinsPath:
    """
    Consider four dubins paths
    - LSL
    - LSR
    - RSL
    - RSR
    and find the shortest obstacle-free one.
    """

    def __init__(self, car, start_pos, end_pos, method='LSR'):

        self.car = car
        self.r = self.car.r

        # turn left: 1, turn right: -1
        self.direction = {
            'LSL': [1, 1],
            'LSR': [1, -1],
            'RSL': [-1, 1],
            'RSR': [-1, -1]
        }

        self.start_pos = start_pos
        self.end_pos = end_pos

        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos

        self.s = np.array(start_pos[:2])
        self.e = np.array(end_pos[:2])

        self.lc1 = transform(x1, y1, 0, self.r, theta1, 1)
        self.rc1 = transform(x1, y1, 0, self.r, theta1, 2)
        self.lc2 = transform(x2, y2, 0, self.r, theta2, 1)
        self.rc2 = transform(x2, y2, 0, self.r, theta2, 2)

        solutions = {'LSL': self._LSL(), 'LSR': self._LSR(),
                     'RSL': self._RSL(), 'RSR': self._RSR()}
        self.route = self.get_route(solutions[method])
        
        self.path_seg_ind = 0
        self.prev_d = np.inf

    def get_route(self, s):
        """ Get the route of dubins path. """
        w = self.car.v / self.car.r
        w1 = w if s.d[0] == 1 else -w
        w2 = w if s.d[1] == 1 else -w
        
        W = [w1, 0, w2]
        goal = [s.t1, s.t2, self.end_pos]
        
        return list(zip(goal, W))

    def get_current_target(self):
        return self.route[self.path_seg_ind][0][:2]

    def get_control(self, x):
        d = np.linalg.norm(self.get_current_target() - x)
        if self.prev_d < d or d < 0.2:
            if self.path_seg_ind == 2:
                return None, True
            self.path_seg_ind += 1
            d = np.linalg.norm(self.get_current_target() - x)
        w = self.route[self.path_seg_ind][1]
        u = np.array([self.car.v, w])
        self.prev_d = d
        return u, False

    def get_params(self, dub, c1, c2, t1, t2):
        """ Calculate the dubins path length. """

        v1 = self.s - c1
        v2 = t1 - c1
        v3 = t2 - t1
        v4 = t2 - c2
        v5 = self.e - c2

        delta_theta1 = directional_theta(v1, v2, dub.d[0])
        delta_theta2 = directional_theta(v4, v5, dub.d[1])

        arc1 = abs(delta_theta1*self.r)
        tangent = np.linalg.norm(v3)
        arc2 = abs(delta_theta2*self.r)

        theta = self.start_pos[2] + delta_theta1

        dub.t1 = t1.tolist() + [theta]
        dub.t2 = t2.tolist() + [theta]
        dub.c1 = c1
        dub.c2 = c2
        dub.len = arc1 + tangent + arc2

        return dub

    def _LSL(self):

        lsl = Params(self.direction['LSL'])

        cline = self.lc2 - self.lc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) - acos(0)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta, 1)

        lsl = self.get_params(lsl, self.lc1, self.lc2, t1, t2)

        return lsl

    def _LSR(self):

        lsr = Params(self.direction['LSR'])

        cline = self.rc2 - self.lc1
        R = np.linalg.norm(cline) / 2

        if R < self.r:
            return None

        theta = atan2(cline[1], cline[0]) - acos(self.r/R)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta+pi, 1)

        lsr = self.get_params(lsr, self.lc1, self.rc2, t1, t2)

        return lsr

    def _RSL(self):

        rsl = Params(self.direction['RSL'])

        cline = self.lc2 - self.rc1
        R = np.linalg.norm(cline) / 2

        if R < self.r:
            return None

        theta = atan2(cline[1], cline[0]) + acos(self.r/R)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta+pi, 1)

        rsl = self.get_params(rsl, self.rc1, self.lc2, t1, t2)

        return rsl

    def _RSR(self):

        rsr = Params(self.direction['RSR'])

        cline = self.rc2 - self.rc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) + acos(0)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta, 1)

        rsr = self.get_params(rsr, self.rc1, self.rc2, t1, t2)

        return rsr


