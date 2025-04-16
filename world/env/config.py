import os
import yaml
from pathlib import Path

project_root = Path(__file__).resolve().parents[2]
params_filename = os.path.join(project_root, 'params/params_compare.yaml')

assert os.path.exists(params_filename)
with open(os.path.join(params_filename)) as f:
    params = yaml.load(f, Loader=yaml.FullLoader)

K = params['K']
horizon = params['horizon']
tau = params['tau']
radius = params['FoV']['radius']
psi = params['FoV']['psi']
epsilon_s = params['epsilon_s']
render = params['render']
dt = params['dt']
use_planner = params['use_planner']
ctrl_penalty_scale_v = params['ctrl_penalty_scale_v']
ctrl_penalty_scale_w = params['ctrl_penalty_scale_w']
use_qp = params['QP']['use_qp']
pen_smooth_v = params['QP']['pen_smooth_v']
pen_smooth_w = params['QP']['pen_smooth_w']
use_penalty_smoothing = params['QP']['use_penalty_smoothing']
obstacle_path = params['obstacle_path']
omega_lb, omega_ub, v_lb, v_ub = params['omega_lb'], params['omega_ub'], params['v_lb'], params['v_ub']
alpha_fov = params['QP']['alpha_fov']
alpha_obs = params['QP']['alpha_obs']
obstacle_path = params['obstacle_path']
make_rounded = params['make_rounded']
map_file_path = os.path.join(project_root, 'resources', params['map_path'])
use_autodiff = params['use_autodiff']
sig_k = params['sig_k']
obs_incl_cond = params['QP']['obs_incl_cond']
use_fov_slack = params['QP']['use_fov_slack']
slack_delta = params['QP']['slack_delta']
planner_max_time = params['Planner']['planner_max_time']
