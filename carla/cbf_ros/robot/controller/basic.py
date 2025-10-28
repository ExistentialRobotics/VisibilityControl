from env.simple_env import Environment
import numpy as np
from env.config import epsilon_s, alpha_fov, radius


class BasicController:
    def __init__(self, env: Environment):
        self.motion_model = None
        self.sensing = None
        self.env = env
        self.ref_input = None

    def c(self, x, ydot, ref, d, rdrx, rdry):
        alpha = 3
        mat = -(rdrx @ self.F(x) + rdrx @ self.G(x) @ ref + rdry @ ydot)
        return mat + alpha * self.hS(d) * np.ones(np.shape(mat))

    def c2(self, r, grad, grady, ydot, sdf, x):
        alf = 3.0
        theta = x[2]
        f = np.array([[0], [0], [0]])
        g = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
        lfh = -grad * f
        lgh = -grad * g
        dhdt = -grady * ydot
        h = -sdf
        c = lfh + lgh * r + dhdt + alf * h
        return c

    @staticmethod
    def F(x):
        return np.array([[0], [0], [0]])

    @staticmethod
    def G(x):
        theta = x[2]
        return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])

    def hS(self, d):
        return -d - epsilon_s * self.min_distance(alpha_fov, radius)

    @staticmethod
    def min_distance(alpha, radius):
        return radius * np.sin(alpha) / (2 + 2 * np.sin(0.5 * alpha))

    def L_Gh(self, x, rdrx):
        return (rdrx @ self.G(x)).reshape(1, 2)

    def u_weighted(self, x, rdrx, c):
        p = 1
        la = np.array([[1, 0], [0, 1 / np.sqrt(p)]])
        return -c * la @ la @ self.L_Gh(x, rdrx).T / np.linalg.norm(self.L_Gh(x, rdrx) @ la)

    def solve(self, x, y, ydot, ref):
        rdrx, rdry, signed_d = self.env.polygon_sdf_grad_lse(x, y, True)
        u = np.copy(ref)
        c = self.c(x, ydot, ref, signed_d, rdrx, rdry)
        u_bar = self.u_weighted(x, rdrx, c)
        gamma = 1 if signed_d > 15 else 5
        if c < 0:
            u[0] += gamma * u_bar[0]
            u[1] += gamma * u_bar[1]
        return self.crop(u)

    @staticmethod
    def crop(u):
        u[0] = min(max(u[0], 0), 20)
        u[1] = min(max(u[1], -1.75), 1.75)
        return u
