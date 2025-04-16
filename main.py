import argparse
import os
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import yaml
from pathlib import Path


from pursuer.controller.cbf_qp import CbfController
# from pursuer.planner.planner_main import Planner
from world.env.simple_env import Environment

project_root = Path(__file__).resolve().parent
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
parser = argparse.ArgumentParser()
args = parser.parse_args()


def run_sim(params_filename):
    assert os.path.exists(params_filename)
    with open(os.path.join(params_filename)) as f:
        params = yaml.load(f, Loader=yaml.FullLoader)

    horizon = params['horizon']
    tau = params['tau']
    radius = params['FoV']['radius']
    psi = params['FoV']['psi']
    epsilon_s = params['epsilon_s']
    render = params['render']

    env = Environment(horizon=horizon, tau=tau, psi=psi, radius=radius, epsilon_s=epsilon_s)

    env.reset()
    done = False
    x_record = []
    y_record = []
    u_record = []
    ydot_record = []
    SDF_record = []
    i = 0
    ref = np.array([[18], [0]])
    u = np.copy(ref)
    cbf = CbfController(env)
    # mpt = Planner()
    ref_mpt = None
    planner_cd = 20
    while not done:
        y, ydot, x = env.update(u)
        # if i % planner_cd == 0:
        #     _, ref_mpt = mpt.get_next_state_cbf(x_cvx, y)
        # if ref_mpt is not None and i < len(ref_mpt):
        #     u_r = ref_mpt[i].reshape(2, 1)
        # else:
        u_r = ref
        time_init = time.time()
        u, obs = cbf.solvecvx(x, y, ydot, u_r, u)
        print(f"Frame rate: {round(1/(time.time() - time_init))}")
        i = (i + 1) % planner_cd
        env.cv_render(x, y, obs)
    env.close()
    if not render:
        y_record = np.array(y_record)
        x_record = np.array(x_record)
        plt.plot(y_record[:, 0], y_record[:, 1])
        plt.plot(x_record[:, 0], x_record[:, 1])
        np.savez('../data/target_traj_2D', y_record)
        np.savez('../data/robot_traj_2D', x_record)
        np.savez('../data/u_record', u_record)
        np.savez('../data/ydot_record', ydot_record)
        np.savez('../data/SDF_record', SDF_record)
        plt.show()


if __name__ == '__main__':
    run_sim(params_filename=os.path.join(project_root, 'params/params_compare.yaml'))
