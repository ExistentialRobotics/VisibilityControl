import json
import time

import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.integrate import solve_ivp, trapezoid
from tqdm import tqdm
import uuid

from robot.controller.mpc.agent import Unicycle
from robot.controller.mpc.constraint import Constraint, CircularConstraint, CBFConstraint
from robot.controller.mpc.loss import QuadracticLoss, ControlNormLoss


class Simulation:
    def __init__(self, agent, init_pos, loss='control_norm', final_pos=None) -> None:
        """default uses control norm loss, can also pass loss='quadratic'"""
        self.T = 50
        self.dt = 0.1
        self.num = 10
        self.init_pos = init_pos
        self.final_pos = final_pos if final_pos is not None else np.array([0, 0, 0])
        self.K = 10

        self.states = np.zeros((3, self.T + 1))
        self.states_cont = np.zeros((3, (self.T + 1) * self.num))
        self.states[:, 0] = self.init_pos
        self.controls = np.zeros((2, self.T))

        self.lambda_t = np.zeros((3, (self.T + 1) * self.num))

        gamma = 1
        self.Q = gamma * np.eye(3)
        self.Q_bar = np.diag([10, 10, 0])
        self.R_bar = np.eye(2) * 1
        if loss == 'quadratic':
            self.loss_fn = QuadracticLoss(self.Q, self.Q_bar, self.R_bar, self.final_pos)
        elif loss == 'control_norm':
            self.loss_fn = ControlNormLoss(self.R_bar)
        else:
            pass
        self.agent = agent

        self.constraints: list[Constraint] = []

    def add_constraint(
        self,
        constraint: Constraint,
        alpha,
        epsilon
      ):
        cbf_constraint = CBFConstraint(constraint, self.agent, alpha, epsilon)  
        self.constraints.append(cbf_constraint)

    def lambda_t_dot(self, t, lambda_t, v, w):
        idx = np.floor(t * self.num / self.dt).astype(int)
        state = self.states_cont[:, idx]
        control = np.array([v, w])

        deriv = - self.agent.f_x(state, control).T @ lambda_t - self.loss_fn.p_x_con(state)
        for constraint in self.constraints:
            deriv += constraint.epsilon / constraint.g(state, control) * constraint.g_x(state, control).T
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
        t0=time.time()
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
        self.lambda_t[:, (t) * self.num: (t + 1) * self.num] = sol.y[:, ::-1]
        self.lambda_t[:, (t) * self.num] = self.loss_fn.p_x_dis(self.states[:, t]) + self.lambda_t[:, (t) * self.num]
        print(time.time()-t0)

    def get_control_gradients(self, t):
        deriv_u_k = np.zeros((2, self.K))
        for k in range(self.K):
            if t + k + 1 >= self.T:
                break

            integral = np.zeros((2, self.num))
            for i in range((t + k) * self.num, (t + k + 1) * self.num):
                int_val = self.lambda_t[:, i].T @ self.agent.G(self.states_cont[:, i])
                for constraint in self.constraints:
                    int_val += (-1 * constraint.epsilon / constraint.g(self.states_cont[:, i], self.controls[:, t + k])
                                * constraint.g_u(self.states_cont[:, i], self.controls[:, t + k]))
                integral[:, i - (t + k) * self.num] = int_val

            deriv_u_k[:, k] = trapezoid(integral, dx=self.dt / self.num) + self.loss_fn.p_u_dis(self.controls[:, t + k].T)
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
        self.states_cont[:, (t + 1) * self.num + 1:] = np.zeros_like(self.states_cont[:, (t + 1) * self.num + 1:])

    def find_current_control(self, t, epochs, delta):
        for _ in range(epochs):
            # Forward
            for k in range(self.K):
                if t + k + 1 >= self.T:
                    break
                self.update_next_state(t + k)

            # Backward
            for k in range(self.K - 1, -1, -1):
                if t + k + 1 >= self.T:
                    break
                self.update_previous_adjoint(t + k)

            # Gradient computation
            deriv_u_k = self.get_control_gradients(t)

            # Update
            valid = self.controls[:, t: t + self.K].shape[1]
            self.controls[:, t: t + self.K] -= delta * deriv_u_k[:, :valid]
            
        loss_val = self.loss_fn.p(self.states[:, t], self.controls[:, t])
        grad_val = np.linalg.norm(deriv_u_k[:, 0])
        return self.controls[:,t], loss_val, grad_val
    
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
            plt.gca().add_patch(
                plt.Circle((constraint.x_coord, constraint.y_coord), constraint.radius, color='r', fill=False))
            config[f"constraint-{count}"] = {
                "radius": constraint.radius,
                "x_coord": constraint.x_coord,
                "y_coord": constraint.y_coord,
                "alpha": cbf_constraint.alpha,
                "epsilon": cbf_constraint.epsilon
            }
        plt.grid()
        plt.savefig(f'./results/{uid}/trajectory.png')
        with open(f'./results/{uid}/config.json', 'w') as fp:
            json.dump(config, fp)
        print(f"Saved results to results/{uid}/")
        plt.show()


def simulate_one_constraint_case():
    sim = Simulation(agent=Unicycle(), init_pos=np.array([-5, -5, 0]), final_pos=np.array([-1, -10, 0]))
    constraint1 = CircularConstraint(
        radius=1,
        x_coord=-4,
        y_coord=-4
    )
    sim.add_constraint(constraint1, alpha=50, epsilon=100)
    sim.simulate(epochs=10, delta=0.01)
    sim.plot_and_save_results()


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
    simulate_one_constraint_case()
    # simulate_two_constraint_case()
