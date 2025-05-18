import os
import yaml
from pathlib import Path

project_root = Path(__file__).resolve().parents[2]
params_filename = os.path.join(project_root, 'params/sim_configs.yaml')

assert os.path.exists(params_filename)
with open(os.path.join(params_filename)) as f:
    params = yaml.load(f, Loader=yaml.FullLoader)

radius = params['FoV']['radius']
psi = params['FoV']['psi']
epsilon_s = params['epsilon_s']
render = params['render']
use_planner = params['Planner']['use_planner']
omega_lb, omega_ub, v_lb, v_ub = params['omega_lb'], params['omega_ub'], params['v_lb'], params['v_ub']
alpha_fov = params['QP']['alpha_fov']
map_file_path = os.path.join(project_root, 'resources', params['map_path'])
planner_max_time = params['Planner']['planner_max_time']
tau = params['tau']
mpt_model_path = params['Planner']['mpt_model_path']
mpt_enable = params['Planner']['mpt_enable']
base_planner = params['Planner']['base_planner']
