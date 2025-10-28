from abc import ABC, abstractmethod
import numpy as np
import autograd.numpy as jnp

class Agent(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def f(self, t, x, v, w):
        """Motion model"""
        pass

    @abstractmethod
    def f_x(self, state, control):
        """Derivative wrt state"""
        pass

    @abstractmethod
    def f_u(self, state, control):
        """Derivative wrt control"""
        pass

    @abstractmethod
    def G(self, state):
        pass


class Unicycle(Agent):
    def __init__(self):
        super().__init__()
        pass

    def G(self, state):
        _, _, theta = state
        return np.array([
            [np.cos(theta), 0],
            [np.sin(theta), 0],
            [0, 1]
        ])

    def G_jnp(self, state):
        _, _, theta = state
        return jnp.array([
            [jnp.cos(theta), 0],
            [jnp.sin(theta), 0],
            [0, 1]
        ])

    def f_jnp(self, x, v, w):
        u = jnp.array([v,w])
        return jnp.dot(self.G_jnp(x), u)

    def f(self, t, x, v, w):
        """Unicycle model"""
        u = np.array([v, w])
        return self.G(x) @ u

    def f_x(self, state, control):
        """Derivate wrt state"""
        _, _, theta = state
        v, w = control
        return np.array([
            [0, 0, -v * np.sin(theta)],
            [0, 0, v * np.cos(theta)],
            [0, 0, 0]
        ])

    def f_u(self, state, control):
        """Derivate wrt control"""
        return self.G(state)



class Diff_Drive_Euler(Agent):
    def __init__(self, dt):
        super().__init__()
        self.dt = dt

    def G(self, state):
        _, _, theta = state
        return np.array([
            [self.dt * np.cos(theta), 0],
            [self.dt * np.sin(theta), 0],
            [0, self.dt]
        ])

    def G_jnp(self, state):
        _, _, theta = state
        return jnp.array([
            [self.dt * jnp.cos(theta), 0],
            [self.dt * jnp.sin(theta), 0],
            [0, self.dt]
        ])

    def f_jnp(self, x, v, w):
        u = jnp.array([v,w])
        return jnp.dot(self.G_jnp(x), u)

    def f(self, t, x, v, w):
        """Unicycle model"""
        u = np.array([v, w])
        y = x + self.G(x) @ u
        # keep theta between 0 and 2pi
        y[2] = (y[2] % (2 * np.pi) + 2 * np.pi) % (2 * np.pi)
        # y[2] = y[2] if y[2] >=0 else y[2] + 2*np.pi
        return y

    def f_x(self, state, control):
        """Derivate wrt state"""
        _, _, theta = state
        v, w = control
        return np.array([
            [0, 0, -self.dt * v * np.sin(theta)],
            [0, 0, self.dt * v * np.cos(theta)],
            [0, 0, 0]
        ])

    def f_u(self, state, control):
        """Derivate wrt control"""
        return self.G(state)
