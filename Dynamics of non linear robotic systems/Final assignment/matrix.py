import numpy as np


class Matrix:
    # This class is required for handy matrix multiplication.

    __array_ufunc__ = None

    def __init__(self, val):
        self.val = val

    def __mul__(self, b):
        if isinstance(b, Matrix):
            val = np.matmul(self.val, b.val)
        else:
            val = np.matmul(self.val, b)
        return Matrix(val)

    def __rmul__(self, a):
        if isinstance(a, Matrix):
            val = np.matmul(a.val, self.val)
        else:
            val = np.matmul(a, self.val)
        return Matrix(val)


class Rx(Matrix):
    # Rotation around x axis

    def __init__(self, q):
        rot = np.zeros((4, 4), dtype=np.float64)
        rot[3, 3] = 1
        rot[0, 0] = 1
        rot[1, :] = [0, np.cos(q), -np.sin(q), 0]
        rot[2, :] = [0, np.sin(q), np.cos(q), 0]
        self.val = rot


class Ry(Matrix):
    # Rotation around y axis

    def __init__(self, q):
        rot = np.zeros((4, 4), dtype=np.float64)
        rot[3, 3] = 1
        rot[:3, :3] = [[np.cos(q), 0, np.sin(q)],
                       [0, 1, 0],
                       [-np.sin(q), 0, np.cos(q)]
                       ]
        self.val = rot


class Rz(Matrix):
    # Rotation around z axis
    def __init__(self, q):
        rot = np.zeros((4, 4), dtype=np.float64)
        rot[3, 3] = 1
        rot[0, :] = [np.cos(q), -np.sin(q), 0, 0]
        rot[1, :] = [np.sin(q), np.cos(q), 0, 0]
        rot[2, 2] = 1
        self.val = rot


class T(Matrix):
    # Transition matrix
    def __init__(self, vector):
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[0:3, 3] = vector
        np.fill_diagonal(mat, 1)
        self.val = mat


class Tx(Matrix):
    # Transition matrix along x axis
    def __init__(self, q):
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[0, 3] = q
        np.fill_diagonal(mat, 1)
        self.val = mat


class Ty(Matrix):
    # Transition matrix along y axis
    def __init__(self, q):
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[1, 3] = q
        np.fill_diagonal(mat, 1)
        self.val = mat


class Tz(Matrix):
    # Transition matrix along z axis
    def __init__(self, q):
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[2, 3] = q
        np.fill_diagonal(mat, 1)
        self.val = mat
