import json
import matplotlib.pyplot as plt
import numpy as np
import os
import time
from scipy.integrate import solve_ivp, trapezoid
from tqdm import tqdm
import uuid
import autograd.numpy as jnp
import importlib
from robot.controller.mpc.agent import Unicycle
import robot.controller.mpc.constraint
import yaml

importlib.reload(robot.controller.mpc.constraint)
from robot.controller.mpc.constraint import Constraint, CircularConstraint, CBFConstraint, PolygonConstraint
import robot.controller.mpc.loss

importlib.reload(robot.controller.mpc.loss)
from robot.controller.mpc.loss import ControlNormLoss, QuadracticLoss

params_filename = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "params/params_compare.yaml"))
with open(os.path.join(params_filename)) as f:
    params = yaml.load(f, Loader=yaml.FullLoader)
K = params['K']


class Simulation:
    def __init__(self, agent, init_pos, loss='control_norm', final_pos=None) -> None:
        self.dt = 0.1
        self.num = 4
        self.init_pos = init_pos
        self.final_pos = final_pos if final_pos is not None else np.array([0, 0, 0])
        self.K = K
        self.T = 6

        self.states = np.zeros((3, self.T + 1))
        self.states_cont = np.zeros((3, (self.T + 1) * self.num))
        self.states[:, 0] = self.init_pos
        # initialize controls to 0 (or to r(x))
        self.controls = np.zeros((2, self.T))  # np.array([np.ones(self.T)*10,np.zeros(self.T)])

        self.lambda_t = np.zeros((3, (self.T + 1) * self.num))

        gamma = 1
        self.Q = gamma * np.eye(3)
        self.Q_bar = np.diag([10, 10, 0])
        self.R_bar = np.eye(2) * 1
        self.ref = np.array([np.ones(self.T) * 10, np.zeros(self.T)])

        if loss == 'quadratic':
            self.loss_fn = QuadracticLoss(self.Q, self.Q_bar, self.R_bar, self.ref, self.final_pos)
        elif loss == 'control_norm':
            self.loss_fn = ControlNormLoss(self.R_bar, self.ref)
        else:
            pass

        self.agent = agent

        self.constraints: list[Constraint] = []

        self.env=None


    def add_constraint(
            self,
            constraint: Constraint,
            alpha,
            epsilon
    ):
        cbf_constraint = CBFConstraint(constraint, self.agent, alpha, epsilon)
        self.constraints.append(cbf_constraint)

    def lambda_t_dot(self, t, lambda_t, v, w):
        # print('solving ivp')
        idx = np.floor(t * self.num / self.dt).astype(int)
        state = self.states_cont[:, idx]
        control = np.array([v, w])
        control_jax = jnp.array(control)
        state_jax = jnp.array(state)
        deriv = - self.agent.f_x(state, control).T @ lambda_t - self.loss_fn.p_x_con(state)

        for constraint in self.constraints:
            # print('getting constraint')
            # h_eval_0, h_x_eval_0, h_xx_eval_0 = constraint.eval_h(state_jax, control_jax)
            h_eval = self.env.sdf(state, constraint.constraint.target_state)
            h_x_eval = -self.env.new_fd_grad(state, constraint.constraint.target_state)
            h_xx_eval = -self.env.new_fd_hess(state, constraint.constraint.target_state)
            constr_val = constraint.epsilon / constraint.g(state_jax, control_jax, h_eval, h_x_eval) * \
                         constraint.g_x(state_jax, control_jax, h_x_eval, h_xx_eval).T
            deriv += constr_val
        return -deriv

    def update_next_state(self, t):
        """
    Find the next cont states between t and t+1,
    Pass t=t+k for MPC
    """
        v, w = self.controls[:, t]
        sol = solve_ivp(
            self.agent.f,
            [(t) * self.dt, (t + 1) * self.dt],
            self.states[:, t],
            args=(v, w),
            t_eval=np.linspace((t) * self.dt, (t + 1) * self.dt, self.num),
            max_step=self.dt / self.num,
            method="LSODA"
        )
        # set the discrete state to the final value of the integrated ODE sim over the time period
        self.states[:, t + 1] = sol.y[:, -1]
        self.states_cont[:, (t) * self.num: (t + 1) * self.num] = sol.y

    def update_previous_adjoint(self, t):
        """
    Find the prev cont adjoint states between t, t-1,
    Pass t=t+k for MPC
    """
        t0 = time.time()
        v, w = self.controls[:, t]
        sol = solve_ivp(
            self.lambda_t_dot,
            [(t) * self.dt, (t + 1) * self.dt],
            self.lambda_t[:, (t + 1) * self.num],
            args=(v, w),
            t_eval=np.linspace((t) * self.dt, (t + 1) * self.dt, self.num),
            max_step=self.dt / self.num,
            method="LSODA"
        )
        # print('solved ivp')
        solll = sol.y
        self.lambda_t[:, (t) * self.num: (t + 1) * self.num] = sol.y[:, ::-1]
        self.lambda_t[:, (t) * self.num] = self.loss_fn.p_x_dis(self.states[:, t]) + \
                                           self.lambda_t[:, (t) * self.num]
        print(time.time() - t0)

    def get_control_gradients(self, t, env):
        deriv_u_k = np.zeros((2, self.K))
        states_cont = jnp.array(self.states_cont)
        controls = jnp.array(self.controls)
        for k in range(self.K):
            if t + k + 1 >= self.T:
                break

            integral = np.zeros((2, self.num))
            for i in range((t + k) * self.num, (t + k + 1) * self.num):
                # aa = (t + k) *self.num, (t + k + 1) * self.num
                # lamd = self.lambda_t[:, i].T
                # G_model = self.agent.G(self.states_cont[:, i])
                int_val = self.lambda_t[:, i].T @ self.agent.G(self.states_cont[:, i])
                for constraint in self.constraints:
                    h_eval = self.env.sdf(states_cont[:, i], constraint.constraint.target_state)
                    h_x_eval = -self.env.new_fd_grad(states_cont[:, i], constraint.constraint.target_state)
                    # h_xx_eval = -self.env.new_fd_hess(states_cont[:, i], constraint.constraint.target_state)
                    # h_eval_0, h_x_eval_0, h_xx_eval_0 = constraint.eval_h(states_cont[:, i], controls[:, t + k])
                    int_val += (-1 * constraint.epsilon / constraint.g(states_cont[:, i],
                                                                       controls[:, t + k], h_eval, h_x_eval)
                                * constraint.g_u(states_cont[:, i],
                                                 controls[:, t + k], h_x_eval))
                ixx = i - (t + k) * self.num
                integral[:, i - (t + k) * self.num] = int_val

            deriv_u_k[:, k] = trapezoid(integral, dx=self.dt / self.num) + \
                              self.loss_fn.p_u_dis(self.controls[:, t + k].T, t + k)
        return deriv_u_k

    def take_one_step_forward(self, t):
        v, w = self.controls[:, t]
        self.controls[:, t + 1:] = np.zeros_like(self.controls[:, t + 1:])
        sol = solve_ivp(
            self.agent.f,
            [(t) * self.dt, (t + 1) * self.dt],
            self.states[:, t],
            args=(v, w),
            t_eval=np.linspace((t) * self.dt, (t + 1) * self.dt, self.num),
            max_step=self.dt / self.num,
            method="LSODA"
        )
        self.states[:, t + 1] = sol.y[:, -1]
        self.states_cont[:, (t) * self.num: (t + 1) * self.num] = sol.y
        self.states[:, t + 2:] = np.zeros_like(self.states[:, t + 2:])
        self.states_cont[:, (t + 1) * self.num + 1:] = np.zeros_like(
            self.states_cont[:, (t + 1) * self.num + 1:])

    def find_current_control(self, t, env, epochs, delta):
        self.env=env

        for c in range(epochs):
            print('Epoch #: ', c)
            print('Forward pass...')
            # Forward
            for k in range(self.K):
                if t + k + 1 >= self.T:
                    break
                self.update_next_state(t + k)

            # Backward
            print('Backward pass...')
            start = time.time()
            for k in range(self.K - 1, -1, -1):
                if t + k + 1 >= self.T:
                    break  # should be continue??
                self.update_previous_adjoint(t + k)
            print('Time taken for single backward pass: ', time.time() - start)

            # Gradient computation
            print('Computing gradients of control vector...')
            start = time.time()
            deriv_u_k = self.get_control_gradients(t, env)
            print('Time taken for controls gradient: ', time.time() - start)
            print('Controls gradient: ', deriv_u_k)

            # Update
            valid = self.controls[:, t: t + self.K].shape[1]
            self.controls[:, t: t + self.K] -= delta * deriv_u_k[:, :valid]
            print('new controls: ', self.controls[:, t: t + self.K])
            print('new lambdas: ', self.lambda_t)

        loss_val = self.loss_fn.p(self.states[:, t], self.controls[:, t], t)
        grad_val = np.linalg.norm(deriv_u_k[:, 0])
        # return self.controls[:,t], loss_val, grad_val
        return self.controls, loss_val, grad_val

    def simulate(self, epochs, delta):
        self.loss_values = []
        self.gradient_values = []

        pb = tqdm(range(self.T))
        for t in pb:
            control, loss_val, grad_val = self.find_current_control(t, epochs, delta)
            pb.set_description(f"Loss: {loss_val}, Gradient: {grad_val}")
            self.loss_values.append(loss_val)
            self.gradient_values.append(grad_val)

            # Take the one step
            self.take_one_step_forward(t)

    def plot_and_save_results(self):
        uid = uuid.uuid1()
        os.makedirs(f'./results/{uid}/', exist_ok=True)
        plt.plot(self.states_cont[0, :-self.num], self.states_cont[1, :-self.num])
        config = {
            "init_pos": self.init_pos.tolist(),
            "K": self.K
        }
        count = 0
        for cbf_constraint in self.constraints:
            constraint = cbf_constraint.constraint
            count += 1
            if isinstance(constraint, CircularConstraint):
                plt.gca().add_patch(
                    plt.Circle((constraint.x_coord, constraint.y_coord), constraint.radius,
                               color='r', fill=False))
                config[f"constraint-{count}"] = {
                    "radius": constraint.radius,
                    "x_coord": constraint.x_coord,
                    "y_coord": constraint.y_coord,
                    "alpha": cbf_constraint.alpha,
                    "epsilon": cbf_constraint.epsilon
                }
            elif isinstance(constraint, PolygonConstraint):
                plt.gca().plot(constraint.polygon[:, 0], constraint.polygon[:, 1])

        plt.grid()
        plt.savefig(f'./results/{uid}/trajectory.png')
        with open(f'./results/{uid}/config.json', 'w') as fp:
            json.dump(config, fp)
        print(f"Saved results to results/{uid}/")
        plt.show()

    def test_unicycle(self, t):
        v, w = np.array([10, 0.5])
        v, w = np.array([])
        sol = solve_ivp(
            self.agent.f,
            [(t) * self.dt, (t + 1) * self.dt],
            np.array([5, 5, 0.3]),
            args=(v, w),
            t_eval=np.linspace((t) * self.dt, (t + 1) * self.dt, self.num),
            max_step=self.dt / self.num,
            method="LSODA"
        )
        return sol.y[:, -1]


def simulate_one_constraint_case(polygon):
    sim = Simulation(agent=Unicycle(), init_pos=np.array([-5, -5, 0]),
                     loss='control_norm', final_pos=np.array([4, -6, 0]))
    constraint1 = PolygonConstraint(polygon)
    # constraint1 = CircularConstraint(
    #     radius=1,
    #     x_coord=-4,
    #     y_coord=-4
    # )
    sim.add_constraint(constraint1, alpha=50, epsilon=100)
    sim.simulate(epochs=8, delta=0.05)
    sim.plot_and_save_results()
    return sim


def simulate_two_constraint_case():
    sim = Simulation(agent=Unicycle(), init_pos=np.array([-5, -5, 0]))
    constraint1 = CircularConstraint(
        radius=2,
        x_coord=-2,
        y_coord=-2,
    )
    sim.add_constraint(constraint1, alpha=10, epsilon=100)

    constraint2 = CircularConstraint(
        radius=2,
        x_coord=-2,
        y_coord=-7,
    )
    sim.add_constraint(constraint2, alpha=5, epsilon=50)

    sim.simulate(epochs=10, delta=0.01)
    sim.plot_and_save_results()

    sim.loss_fn.plot_loss()


if __name__ == "__main__":
    poly = jnp.array([[4.15058908, 1.8007605],
                      [2.6075482, -7.],
                      [2., -7.],
                      [1.85917585, -7.57628843],
                      [-2.2330149, -4.14926839],
                      [-1., -3.],
                      [0.36793827, 0.41984566],
                      [4.15058908, 1.8007605]])
    sim = simulate_one_constraint_case(poly)
    # simulate_two_constraint_case()
