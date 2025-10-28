from abc import abstractmethod
from robot.controller.mpc.agent import Agent
import numpy as np
import autograd.numpy as jnp
from autograd import grad, jacobian
from utils.utils import use_autodiff


class Constraint:
    """Abstract class for constraints."""

    def __init__(self) -> None:
        self.polygon = None

    @abstractmethod
    def h(self, state):
        pass

    @abstractmethod
    def h_x(self, state):
        pass

    @abstractmethod
    def h_xx(self, state):
        pass

    @abstractmethod
    def h_u(self, state, control):
        pass


class CBFConstraint:
    """Convert a constraint to a CBF constraint."""

    def __init__(
            self,
            constraint: Constraint,
            agent: Agent,
            epsilon,
            alpha
    ) -> None:
        self.constraint = constraint
        self.obstacle_type = type(constraint).__name__

        self.agent = agent
        self.alpha = alpha
        self.epsilon = epsilon

        self.f = self.agent.f
        self.f_jnp = self.agent.f_jnp
        self.f_x = self.agent.f_x
        self.f_u = self.agent.f_u

        self.h = self.constraint.h
        self.h_x = self.constraint.h_x
        self.h_u = self.constraint.h_u
        self.h_xx = self.constraint.h_xx

    def eval_h(self, state, tgt_state, control):
        if self.obstacle_type == 'PolygonConstraint':
            # gradient of barrier fcn wrt agent pose for static polygon is 2x1, append 0 to make 3x1
            # since gradient wrt theta = 0 for SDFs
            # start = time.time()
            # once gradients of barrier are evaluated, we don't need autograd arrays anymore
            state_jax = jnp.array(state)
            h_x_eval = np.append(self.h_x(state_jax), 0)
            h_xx_eval = self.h_xx(state_jax)
            # time_taken_grad = time.time() - start
            # print('Grad compute time: ', time_taken_grad)
            h_eval = self.h(state)
            # time_taken_sdf = time.time() - start - time_taken_grad
        elif self.obstacle_type == 'FOVConstraint':
            # polygon = self.constraint.get_visible_region(state)
            # start = time.time()
            if use_autodiff:
                state = jnp.array(state)

            h_x_eval = self.h_x(state, tgt_state)
            # print('Time for gradient: ', time.time() - start)
            # sdf(y) evaluated wrt target state but fov calculated wrt agent state
            # start = time.time()
            h_xx_eval = self.h_xx(state, tgt_state)
            # print('Time for hessian: ', time.time() - start)
            h_eval = self.h(state, tgt_state)
        else:
            h_x_eval = self.h_x(state)
            h_eval = self.h(state)
            h_xx_eval = self.h_xx(state)

        return h_eval, h_x_eval, h_xx_eval

    def g(self, rbt_state, control, h_eval, h_x_eval):
        v, w = control
        # NOTE: @ f only works with unicycle model where x_dot = G(x)u i.e. not x_dot = f(x) + G(x)u
        # in general need h_x @ G(x) @ u
        # todo: check return statement with @G@u
        # return (-h_x_eval @ self.f(0, rbt_state, v, w)) - self.alpha * h_eval
        return (-h_x_eval @ self.agent.G(rbt_state) @ control) - self.alpha * h_eval

    def g_x(self, rbt_state, control, h_x_eval, h_xx_eval):
        v, w = control
        # check_f = jnp.dot(self.f(0, state, v, w), h_xx_eval)
        # check_h_x = jnp.dot(h_x_eval, self.f_x(state, control))
        # return np.dot(-h_xx_eval, self.f(0, rbt_state, v, w)) - \
        #     np.dot(h_x_eval, self.f_x(rbt_state, control)) - \
        #     self.alpha * h_x_eval
        return (-h_xx_eval @ self.agent.G(rbt_state) @ control) - \
            np.dot(h_x_eval, self.f_x(rbt_state, control)) - \
            self.alpha * h_x_eval

    def g_u(self, rbt_state, control, h_x_eval):
        return np.dot(-h_x_eval, self.f_u(rbt_state, control))


class CircularConstraint(Constraint):
    """Circular constraint"""

    def __init__(self, radius, x_coord, y_coord) -> None:
        # init from parent class to set polygon to None for circle constraint
        super(CircularConstraint, self).__init__()
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.radius = radius
        self.obstacle = jnp.array([x_coord, y_coord, 0])
        self.Lambda = jnp.array(
            [
                [1 / (radius ** 2), 0, 0],
                [0, 1 / (radius ** 2), 0],
                [0, 0, 0]
            ]
        )

    def h(self, state):
        return (state - self.obstacle).T @ self.Lambda @ (state - self.obstacle) - 1

    def h_x(self, state):
        # return 2 * (state - self.obstacle).T @ self.Lambda
        return grad(self.h)(state)

    def h_xx(self, state):
        # return 2 * self.Lambda
        return jacobian(self.h_x)(state)

    def h_u(self, state, control):
        return jnp.zeros((1, 2))


class PolygonConstraint(Constraint):
    def __init__(self, polygon: list) -> None:
        super(PolygonConstraint, self).__init__()
        self.polygon = polygon

    def h(self, state):
        """Barrier function (SDF) for an arbitrary polygon"""
        N = len(self.polygon) - 1
        e = self.polygon[1:] - self.polygon[:-1]
        v = state[:2] - self.polygon[:-1]
        pq = v - e * jnp.clip((v[:, 0] * e[:, 0] + v[:, 1] * e[:, 1]) / (e[:, 0] * e[:, 0] + e[:, 1] * e[:, 1]), 0,
                              1).reshape(N, -1)
        d = jnp.min(pq[:, 0] * pq[:, 0] + pq[:, 1] * pq[:, 1])
        wn = 0
        for i in range(N):
            i2 = int(jnp.mod(i + 1, N))
            cond1 = 0 <= v[i, 1]
            cond2 = 0 > v[i2, 1]
            val3 = jnp.cross(e[i], v[i])
            wn += 1 if cond1 and cond2 and val3 > 0 else 0
            wn -= 1 if ~cond1 and ~cond2 and val3 < 0 else 0
        sign = 1 if wn == 0 else -1
        return jnp.sqrt(d) * sign

    def h_x(self, state):
        # always uses autodiff since its fast since no need for raytracing for sdf of robot wrt obstacle
        return grad(self.h)(state[0:2])

    def h_xx(self, state):
        z = jnp.zeros((3, 3))
        z[0:2, 0:2] = jacobian(self.h_x)(state[0:2])
        return z

    def h_u(self, state, control):
        return jnp.zeros((1,2))
        # return jnp.append(grad(self.h, argnum=1)(state[0:2], control), 0)


class FOVConstraint(Constraint):
    def __init__(self, env) -> None:
        super(FOVConstraint, self).__init__()
        self.env = env

    def h(self, state, tgt_state):
        """Barrier function (SDF) for an arbitrary polygonal FoV calculated wrt agent state
        but evaluated wrt target state. Sign flipped since 'safe' set is inside polygon"""
        # get the occluded FoV polygon in world frame
        polygon = self.env.get_visible_region_jnp(state)

        N = len(polygon) - 1
        e = polygon[1:] - polygon[:-1]
        v = tgt_state[:2] - polygon[:-1]
        pq = v - e * jnp.clip((v[:, 0] * e[:, 0] + v[:, 1] * e[:, 1]) / (e[:, 0] * e[:, 0] + e[:, 1] * e[:, 1]), 0,
                              1).reshape(N, -1)
        d = jnp.min(pq[:, 0] * pq[:, 0] + pq[:, 1] * pq[:, 1])
        wn = 0
        for i in range(N):
            i2 = int(jnp.mod(i + 1, N))
            cond1 = 0 <= v[i, 1]
            cond2 = 0 > v[i2, 1]
            val3 = jnp.cross(e[i], v[i])
            wn += 1 if cond1 and cond2 and val3 > 0 else 0
            wn -= 1 if ~cond1 and ~cond2 and val3 < 0 else 0
        sign = 1 if wn == 0 else -1
        return -jnp.sqrt(d) * sign

    def h_x(self, state, tgt_state):
        if use_autodiff:
            return grad(self.h)(state)
        else:
            # get visible region via raytracing, then computes sdf(FoV, target)
            return self.env.new_fd_grad(state, tgt_state)


    def h_xx(self, state, tgt_state):
        if use_autodiff:
            return jacobian(grad(self.h)(state))(state)
        else:
            return self.env.new_fd_hess(state, tgt_state)


    def h_u(self, state, control):
        return


if __name__ == "__main__":
    psi = 1.0472
    radius = 100
    y_future = jnp.array([158.46240802, 3.59904008, 1.91283549])
    state = jnp.array([142.46241225, 0.94630009, 1.9])
    fov_constraint = FOVConstraint(y_future, psi, radius)
    print('finding h...')
    polygon = fov_constraint.get_visible_region(state)
    h = fov_constraint.h(state)
    print('finding grad h...')
    grad_h = fov_constraint.h_x(state)
    print(h, grad_h)
    # constr = CBFConstraint(fov_constraint, agent=Unicycle(), alpha=50, epsilon=100)