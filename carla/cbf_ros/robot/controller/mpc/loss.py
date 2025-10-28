from abc import abstractmethod, ABC
import matplotlib.pyplot as plt
from scipy.special import erf
import numpy as np
import yaml
from utils.utils import sigmoid
from autograd import grad
import os

params_filename = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "params/params_compare.yaml"))
with open(os.path.join(params_filename)) as f:
    params = yaml.load(f, Loader=yaml.FullLoader)
use_autodiff = params['use_autodiff']
use_exp_loss = params['use_exp_loss']
use_fov_circular = params['use_fov_circular']
sig_k = params['sig_k']
sig_bdry_r_offset = params['sig_bdry_r_offset']
use_euclidean_dist = params['use_euclidean_dist']
add_ctrl_penalty = params['add_control_deviation_penalty']

class Loss(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def p(self, state, control):
        """Loss value"""
        pass

    @abstractmethod
    def p_x_con(self, state):
        """Derivate of the continuous loss wrt state"""
        pass

    @abstractmethod
    def p_x_dis(self, state):
        """Derivative of the discrete loss wrt state"""
        pass


class QuadracticLoss(Loss):
    def __init__(self, Q, Q_bar, R_bar, ref, final_pos=None):
        self.Q = Q
        self.Q_bar = Q_bar
        self.R_bar = R_bar
        self.ref = ref
        self.loss_record = []

        if final_pos is None:
            self.final_pos = np.array([0, 0, 0])
        else:
            self.final_pos = final_pos

    def cont_loss(self, states):
        """Continuous Time loss"""
        try:
            # If states is a matrix of form N X k
            loss = np.diag((states - self.final_pos[None, :]).T @ self.Q @ (states - self.final_pos[None, :]))
        except ValueError:
            # If state is just a single state of dimension k
            loss = (states - self.final_pos).T @ self.Q @ (states - self.final_pos)
        return np.sum(loss)

    def discrete_loss(self, states):
        try:
            loss = np.diag((states - self.final_pos[None, :]).T @ self.Q_bar @ (states - self.final_pos[None, :]))
        except ValueError:
            loss = (states - self.final_pos).T @ self.Q_bar @ (states - self.final_pos)
        return np.sum(loss)

    def control_loss(self, control, ix):
        try:
            loss = np.diag((control - self.ref[:, ix]).T @ self.R_bar @ (control - self.ref[:, ix]))
        except ValueError:
            loss = (control - self.ref[:, ix]).T @ self.R_bar @ (control - self.ref[:, ix])
        return np.sum(loss)

    def p(self, states, control, ix):
        """Value of the loss function."""
        self.loss_record.append([self.cont_loss(states), self.discrete_loss(states), self.control_loss(control, ix)])
        return self.cont_loss(states) + self.discrete_loss(states) + self.control_loss(control, ix)

    def p_x_dis(self, state):
        return 2 * self.Q_bar @ (state - self.final_pos)

    def p_x_con(self, state):
        return 2 * self.Q @ (state - self.final_pos)

    def p_u_dis(self, control, ix):
        """Derivative of discrete loss wrt control"""
        return 2 * (control - self.ref[:, ix]) @ self.R_bar

    def plot_loss(self):
        self.loss_record = np.array(self.loss_record)
        plt.plot(self.loss_record[:, 0])
        plt.title('cont_loss')
        plt.show()
        plt.plot(self.loss_record[:, 1])
        plt.title('discrete_loss')
        plt.show()
        plt.plot(self.loss_record[:, 2])
        plt.title('control_loss')
        plt.show()


class ControlNormLoss(Loss):
    def __init__(self, R_bar=np.zeros((2,2)), control=np.array([0,0]), ref=np.array([0,0])):
        self.R_bar = R_bar
        self.control = control
        self.ref = ref
        self.loss_record = []

    def p(self):
        """Loss value"""
        try:
            loss = np.diag((self.control - self.ref).T @ self.R_bar @ (self.control - self.ref))
        except ValueError:
            loss = (self.control - self.ref).T @ self.R_bar @ (self.control - self.ref)
        return np.sum(loss)

    def p_x_con(self, state):
        """Derivate of the continuous loss wrt state"""
        return 0

    def p_x_dis(self, state):
        """Derivative of the discrete loss wrt state"""
        return 0

    def p_u_dis(self):
        """Derivative of discrete loss wrt control"""
        return 2 * (self.control - self.ref) @ self.R_bar


class SDFLoss(Loss):
    def __init__(self, env, epsilon=10):
        self.env = env
        self.epsilon = epsilon

    def p(self, state, control):
        """Loss value"""
        rbt, tgt = state
        if use_exp_loss:
            return np.exp(self.env.sdf(rbt, tgt) / self.epsilon)
        else:
            return self.env.sdf(rbt, tgt)

    def p_x_con(self, state):
        """Derivative of the continuous loss wrt state"""
        pass

    def p_x_dis(self, state):
        """Derivate of the discrete loss wrt state"""
        rbt, tgt = state
        if use_fov_circular:
            if use_exp_loss:
                return np.exp(self.env.sdf_circular(rbt, tgt) / self.epsilon) * \
                    self.epsilon * self.env.grad_sdf_circular(rbt, tgt)
            else:
                return self.env.grad_sdf_circular(rbt, tgt)
        else:
            if use_exp_loss:
                if use_autodiff:
                    return (np.exp(self.env.sdf_jnp(rbt, tgt) / self.epsilon) *
                            self.env.autodiff_grad(rbt, tgt))
                else:
                    # note for exponential - use chain rule i.e. d/dx e^sdf = sdf' e^sdf
                    return (np.exp(self.env.sdf(rbt, tgt) / self.epsilon) *
                        self.env.new_fd_grad(rbt, tgt))  #- 2 * np.array([0, 0, tgt[2] - rbt[2]]))
            else:
                if use_autodiff:
                    return self.env.autodiff_grad(rbt, tgt)
                else:
                    return self.env.new_fd_grad(rbt, tgt)

    def p_u_dis(self, control):
        """Derivative of discrete loss wrt control"""
        return 0

class SigmoidSDFLoss(Loss):
    def __init__(self, env):
        self.env = env

    def p(self, state):
        """Loss value"""
        rbt, tgt = state
        return sigmoid(sig_k, self.env.sdf(rbt, tgt))

    def p_x_con(self, state):
        """Derivative of the continuous loss wrt state"""
        pass

    def p_x_dis(self, state):
        """Derivative of the discrete loss wrt state"""
        rbt, tgt = state
        if use_autodiff:
            return self.env.autodiff_grad_sig(rbt, tgt)
        else:
            return self.env.new_fd_grad_sig(rbt, tgt)

    def p_u_dis(self, control):
        """Derivative of discrete loss wrt control"""
        return 0

class IfElseSigmoidSDFLoss(Loss):
    def __init__(self, env):
        self.env = env
        self.control_loss_adapter = ControlNormLoss()

    def add_ctrl_norm(self, R_bar, control, ref):
        # controlNorm loss object inits with 0s in control and ref; this fcn updates those values
        # this must be called everytime a new control and reference is passed in i.e. inside every planning loop
        self.control_loss_adapter.R_bar = R_bar
        self.control_loss_adapter.control = control
        self.control_loss_adapter.ref = ref

    def euclidean_dist_offset(self, rbt, tgt):
        return np.linalg.norm(rbt - tgt) - (self.env.r + sig_bdry_r_offset)

    def p(self, state):
        """If/else loss function: sdf/euclidean distance"""
        rbt, tgt = state
        # note when the loss switches to sdf, it is not sigmoid(sdf); can test sigmoid later
        if use_euclidean_dist:
            loss = np.linalg.norm(rbt - tgt)
        else:
            loss = sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt)) * np.linalg.norm(rbt - tgt) + \
        (1 - sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt))) * self.env.sdf(rbt, tgt)

        return loss + self.control_loss_adapter.p()

    def p_autodiff(self, rbt, tgt):
        return sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt)) * np.linalg.norm(rbt - tgt) + \
            (1 - sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt))) * self.env.sdf_jnp(rbt, tgt)

    def p_x_con(self, state):
        """Derivative of the continuous loss wrt state"""
        pass

    def fd_if_else(self, state):
        rbt, tgt = state
        delta = 10e-2
        delta_t = delta / 10
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        # since self.p() expects a (rbt,tgt) tuple, add the extra brackets in the args below
        df_dx = (self.p((rbt + X[0], tgt)) - self.p((rbt + X[1], tgt))) / (2 * delta)
        df_dy = (self.p((rbt + X[2], tgt)) - self.p((rbt + X[3], tgt))) / (2 * delta)
        df_dt = (self.p((rbt + X[4], tgt)) - self.p((rbt + X[5], tgt))) / (2 * delta_t)
        return np.array([df_dx, df_dy, df_dt])

    def analytical_if_else_grad(self, state):
        rbt,tgt = state
        return 2*sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt))*(rbt-tgt) + 2*np.linalg.norm(rbt - tgt)*sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt))*(1-sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt)))*(rbt-tgt) + \
            (1-sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt)))*self.env.new_fd_grad(rbt, tgt) - 2*self.env.sdf(rbt, tgt)*sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt))*(1-sigmoid(sig_k, self.euclidean_dist_offset(rbt, tgt)))*(rbt-tgt)
    def p_x_dis(self, state):
        """Derivative of the discrete loss wrt state"""
        rbt, tgt = state
        if use_euclidean_dist:
            return 2*(rbt - tgt)
        if use_autodiff:
            return grad(self.p_autodiff)(rbt, tgt)
        else:
            return self.analytical_if_else_grad(state)
            # return self.fd_if_else(state)

    def p_u_dis(self):
        """Derivative of discrete loss wrt control"""
        return 0 + self.control_loss_adapter.p_u_dis()
