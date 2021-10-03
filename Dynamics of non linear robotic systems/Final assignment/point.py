import numpy as np


class Point:
    def __init__(self, t, q, v):
        self.t = t
        self.q = q
        self.v = v

    def get_point_to_point_plot_line(point1, point2):
        if point1.t == point2.t:
            return [], [], [], []
        num_ticks = 200
        t = np.linspace(point1.t, point2.t, num_ticks)
        a = (point2.v - point1.v)/(point2.t - point1.t)
        b = (point1.v*point2.t - point2.v*point1.t)/(point2.t - point1.t)

        q = point1.q + 0.5 * a * t**2 + b * t - \
            (0.5 * a * point1.t**2 + b * point1.t)
        v = a * t + b
        a = np.full(num_ticks, a)

        return t, q, v, a

    def print_points(points):
        template = "{0:>10}|{1:>10}|{2:>10}|{3:>10}"
        print("Points:")
        print(template.format("№", "t", "q", "v"))
        template = "{0:10}|{1:10.3f}|{2:10.3f}|{3:10.3f}"
        for i, point in enumerate(points):
            print(template.format(i, point.t, point.q, point.v))