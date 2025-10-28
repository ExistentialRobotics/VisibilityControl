import numpy as np
import os
import time
from tqdm import tqdm
import autograd.numpy as jnp
import importlib
import robot.controller.mpc.constraint
import yaml
importlib.reload(robot.controller.mpc.constraint)
from copy import deepcopy
from robot.controller.mpc.constraint import Constraint, CBFConstraint
import robot.controller.mpc.loss
importlib.reload(robot.controller.mpc.loss)
from robot.controller.mpc.loss import ControlNormLoss, QuadracticLoss, SDFLoss, SigmoidSDFLoss, IfElseSigmoidSDFLoss

params_filename = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "params/params_compare.yaml"))
with open(os.path.join(params_filename)) as f:
    params = yaml.load(f, Loader=yaml.FullLoader)
K = params['K']
w_scale_factor = params['w_scale_factor']
use_autodiff = params['use_autodiff']
use_planner = params['use_planner']
add_ctrl_penalty = params['add_control_deviation_penalty']
use_omega_constraint = params['use_omega_constraint']
use_lin_vel_constraint = params['use_lin_vel_constraint']
OMEGA_LB = params['omega_lb']
OMEGA_UB = params['omega_ub']
V_LB = params['v_lb']
V_UB = params['v_ub']
ctrl_penalty_scale_v = params['ctrl_penalty_scale_v']
ctrl_penalty_scale_w = params['ctrl_penalty_scale_w']

class Simulation:
    def __init__(
            self,agent,init_pos,dt,horizon,tgt_future_traj,loss,ref_ctrl,final_pos=None,env=None) -> None:
        self.dt = dt
        self.init_pos = init_pos
        self.final_pos = final_pos if final_pos is not None else np.array([0, 0, 0])
        self.K = K
        self.T = horizon

        self.states = np.zeros((3, self.T + 1))
        self.states[:, 0] = self.init_pos

        self.controls = ref_ctrl
        self.lambda_t = np.zeros((3, self.T + 1))
        self.ref = deepcopy(self.controls)

        gamma = 1
        self.Q = gamma * np.eye(3)
        self.Q_bar = np.diag([1, 1, 0])
        self.R_bar = np.diag([ctrl_penalty_scale_v, ctrl_penalty_scale_w])
        self.env = env

        if loss == 'quadratic':
            self.loss_fn = QuadracticLoss(self.Q, self.Q_bar, self.R_bar, self.ref, self.final_pos)
        elif loss == 'control_norm':
            self.loss_fn = ControlNormLoss(self.R_bar, self.ref)
        elif loss == 'SDFLoss':
            self.loss_fn = SDFLoss(self.env)
        elif loss == 'SigmoidSDFLoss':
            self.loss_fn = SigmoidSDFLoss(self.env)
        elif loss == 'IfElseSigmoidSDFLoss':
            self.loss_fn = IfElseSigmoidSDFLoss(self.env)

        self.agent = agent

        self.constraints: list[Constraint] = []
        self.tgt_future_traj = tgt_future_traj

    def add_constraint(
            self,
            constraint: Constraint,
            epsilon,
            alpha=1,
            is_cbf: bool = True
    ):
        if is_cbf:
            constraint = CBFConstraint(constraint, self.agent, alpha, epsilon)
        else:
            constraint.epsilon = epsilon
        self.constraints.append(constraint)

    def update_next_state(self, t):
        """
    Find the next state using the motion model
    """
        v, w = self.controls[:, t]
        # motion model for next state
        self.states[:, t + 1] = self.agent.f(t, self.states[:, t], v, w)

    def update_previous_adjoint(self, t):
        """
    Find the prev cont adjoint states between t, t-1,
    Pass t=t+k for MPC
    """
        v, w = self.controls[:, t]
        p_x_dis = self.loss_fn.p_x_dis((self.states[:, t], self.tgt_future_traj[:, t]))
        disc_term = p_x_dis + self.agent.f_x(self.states[:, t], (v, w)).T @ self.lambda_t[:, t + 1]
        # print('SDF Gradient: ', p_x_dis)
        for constraint in self.constraints:
            # print('getting constraint')
            # h_eval = self.env.sdf(state_jax, curr_tgt_state)
            # h_x_eval = self.env.new_fd_grad(state_jax, curr_tgt_state)
            # h_xx_eval = self.env.new_fd_hess(state_jax, curr_tgt_state)
            h_eval, h_x_eval, h_xx_eval = constraint.eval_h(self.states[:, t], self.tgt_future_traj[:, t], self.controls[:, t])
            constr_val = 0
            if isinstance(constraint, CBFConstraint):
                # print(constraint.obstacle_type, h_x_eval, h_xx_eval)
                constr_val = constraint.epsilon / constraint.g(self.states[:, t], self.controls[:, t], h_eval, h_x_eval) * \
                             constraint.g_x(self.states[:, t], self.controls[:, t], h_x_eval, h_xx_eval).T
            else:
                # constr_val = constraint.epsilon / (h_eval + 50) * \
                #             h_x_eval.T
                # constr_val = np.array([0, 0, -2 * (curr_tgt_state[2] - state_jax[2])])
                pass
            disc_term -= constr_val

        self.lambda_t[:, t] = disc_term
        return disc_term[2]

    def get_control_gradients(self, t):
        deriv_u_k = np.zeros((2, self.K))
        for k in range(self.K):
            if t + k + 1 >= self.T:
                break
            constr_val = np.zeros((2,))

            for constraint in self.constraints:
                # h_eval = self.env.sdf(self.states[:, t + k], curr_tgt_state)
                # h_x_eval = self.env.new_fd_grad(self.states[:, t + k], curr_tgt_state)
                # h_xx_eval = self.env.new_fd_hess(self.states[:, t + k], curr_tgt_state)

                # could be a polygonal obstacle constraint or an FOV Constraint; converted to jax inside eval_h if its polygonConstraint
                h_eval, h_x_eval, h_xx_eval = constraint.eval_h(self.states[:, t + k], self.tgt_future_traj[:, t + k], self.controls[:, t + k])

                if isinstance(constraint, CBFConstraint):
                    constr_val += (-1 * constraint.epsilon / constraint.g(self.states[:, t + k],
                                                                          self.controls[:, t + k], h_eval, h_x_eval)
                                   * constraint.g_u(self.states[:, t + k],
                                                    self.controls[:, t + k], h_x_eval))
                else:
                    constr_val = 0

            if add_ctrl_penalty: # not used with control norm loss
                self.loss_fn.add_ctrl_norm(self.R_bar,  self.controls[:, t + k].T, self.ref[:, k])

            # add_ctrl_norm above updates the control norm object inside IfElseLoss;
            # call to p_u_dis below doesn't need external arguments
            if isinstance(self.loss_fn, ControlNormLoss):
                self.loss_fn.control = self.controls[:,t+k]
                self.loss_fn.ref = self.ref[:, k]

            # dbg_p_u = self.loss_fn.p_u_dis()
            # print("dL/du:", dbg_p_u, "dL/du lambda term: ", self.agent.f_u(self.states[:, t + k], self.controls[:, t + k]).T @ self.lambda_t[:,
            #                                                                                      t + k + 1])
            deriv_u_k[:, k] = constr_val + self.loss_fn.p_u_dis() + \
                              self.agent.f_u(self.states[:, t + k], self.controls[:, t + k]).T @ self.lambda_t[:,
                                                                                                 t + k + 1]
            # print(self.lambda_t[:,t + k + 1])
        return deriv_u_k

    def find_current_control(self, t, epochs, delta, arr_control_init=None):
        # update the global control array with passed in argument
        if use_planner and arr_control_init is not None:
            self.controls[:, t: t + self.K] = arr_control_init
            print('control init being updated')
        else:
            pass
        # ref control input to use with control norm loss term
        self.ref = deepcopy(self.controls[:, t: t + self.K])
        # need to update the ref and control signal is using control norm loss as each iteration has a new reference input
        if isinstance(self.loss_fn, ControlNormLoss):
            self.loss_fn.ref = self.ref
            self.loss_fn.control = self.controls[:, t: t + self.K]

        loss_fn_deriv_theta_k = np.zeros((epochs, self.K))
        euc_dist_loss_active_ind = 0
        for c in tqdm(range(epochs)):
            # Forward
            for k in range(self.K):
                if t + k + 1 >= self.T:
                    break
                self.update_next_state(t + k)

            # Backward
            start = time.time()
            for k in range(self.K, -1, -1):
                if t + k + 1 >= self.T:
                    break
                # set lambda_K based on terminal x_K
                if k == self.K:
                    curr_tgt_state = self.tgt_future_traj[:, t + k]
                    self.lambda_t[:, t + k] = self.loss_fn.p_x_dis((self.states[:, t + k], curr_tgt_state))
                    loss_fn_deriv_theta_k[c,k-1] = deepcopy(self.lambda_t[2, t + k])
                else:
                    loss_fn_deriv_theta_k[c,k-1] = self.update_previous_adjoint(t + k)

                # only for visualization purposes
                # if self.loss_fn.euclidean_dist_offset(self.states[:, t + k], self.tgt_future_traj[:, t + k]) > 0:
                #     euc_dist_loss_active_ind = 1
            # print('Time taken for single backward pass: ', time.time() - start)

            # Gradient computation
            # print('Computing gradients of control vector...')
            start = time.time()
            deriv_u_k = self.get_control_gradients(t)
            # print('Time taken for controls gradient: ', time.time() - start)
            # print('Controls gradient: ', deriv_u_k)

            # Update
            valid = self.controls[:, t: t + self.K].shape[1]
            self.controls[:, t: t + self.K] -= delta * deriv_u_k[:, :valid]
            # print("||optimized_control - planner_reference||_2: ", np.linalg.norm(self.controls[:, t: t + self.K] - self.ref))

            if use_omega_constraint:
                # first, wrap between -pi and pi
                # self.controls[1, t: t + self.K] = (self.controls[1, t: t + self.K] + np.pi) % (2 * np.pi) - np.pi
                self.controls[1, t: t + self.K] = np.clip(self.controls[1, t: t + self.K], OMEGA_LB, OMEGA_UB)
            if use_lin_vel_constraint:
                self.controls[0, t: t + self.K] = np.clip(self.controls[0, t: t + self.K], V_LB, V_UB)

            # wrap around of omega portion of controls
            # self.controls[1, :] = np.where(self.controls[1, :] >= 0, np.abs(self.controls[1, :]) % 2 * np.pi,
            #                                -1 * (np.abs(self.controls[1, :]) % 2 * np.pi))

            # print('new controls: ', self.controls[:, t: t + self.K])
            # # print('new lambdas: ', self.lambda_t[:,t:t+self.K])
            # print('l2-norm of abs change in w from prev epoch: ',
            #       np.linalg.norm(np.abs(self.controls[1, t:t+self.K] - prev_controls[1, :])))
        # print('Updated adjoint: ', self.lambda_t[:, t:t + self.K])
        # loss_val = self.loss_fn.p((self.states[:, t], self.target))

        grad_val = np.linalg.norm(deriv_u_k[:, 0])
        loss_fn_deriv_theta_k_first_control = np.max(np.abs(loss_fn_deriv_theta_k[:, 0]))
        # print(np.abs(loss_fn_deriv_theta_k), np.abs(loss_fn_deriv_theta_k[:, 0]))

        self.lambda_t[:, :] = 0
        return (self.controls[:, t], grad_val,
                loss_fn_deriv_theta_k_first_control,
                deriv_u_k[:, :valid])#, euc_dist_loss_active_ind


if __name__ == "__main__":
    poly = jnp.array([[4.15058908, 1.8007605],
                      [2.6075482, -7.],
                      [2., -7.],
                      [1.85917585, -7.57628843],
                      [-2.2330149, -4.14926839],
                      [-1., -3.],
                      [0.36793827, 0.41984566],
                      [4.15058908, 1.8007605]])
