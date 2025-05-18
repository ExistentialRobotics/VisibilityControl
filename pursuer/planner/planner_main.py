import json
import time
import threading
import cv2
import numpy as np
import torch

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import util as ou
except ImportError:
    raise ImportError("Importing OMPL failed")
from math import sin, cos
from utils.utils import map2world, world2map, visible_region_omni, polygon_sdf, normalize_angle
from pursuer.planner.transformer import Models
from os import path as osp
from pursuer.planner.mpt_tools import get_patch
from world.env.config import *
from world.env.maps.map import Map

sim_map = Map().map
robot_radius = 0.2
carLength = 0.3
cam_alpha = np.pi / 3
cam_radius = 100

map_origin = np.array([[490], [488], [np.pi / 2]])
map_ratio = 2

# Planning parameters
space = ob.SE2StateSpace()
# Set the bounds
bounds = ob.RealVectorBounds(2)
bounds.setLow(-200)
bounds.setHigh(200)
space.setBounds(bounds)

cspace = oc.RealVectorControlSpace(space, 2)
cbounds = ob.RealVectorBounds(2)
cbounds.setLow(0, v_lb)
cbounds.setHigh(0, v_ub)
cbounds.setLow(1, omega_lb)
cbounds.setHigh(1, omega_ub)
cspace.setBounds(cbounds)
ss = oc.SimpleSetup(cspace)
si = ob.SpaceInformation(space)

ou.setLogLevel(ou.LOG_WARN)
seed = 42


def kinematicUnicycleODE(x, u, xdot):
    xdot[0] = u[0] * cos(x[2])
    xdot[1] = u[0] * sin(x[2])
    xdot[2] = u[1]


class VisibilityGoal(ob.Goal):
    def __init__(self, space, goal_state, sim_map):
        super().__init__(space)
        self.goal_state = goal_state.get()
        self.sim_map = sim_map
        target_w = np.array([self.goal_state.getX(), self.goal_state.getY(), self.goal_state.getYaw()]).reshape((3, 1))
        target_d = world2map(map_origin, map_ratio, target_w.reshape((3, 1))).squeeze()
        self.omni_polygon = visible_region_omni(target_d, 100, 200, sim_map)
        self.omni_polygon = (map2world(map_origin, map_ratio, self.omni_polygon.T)[0:2, :]).T

    def isSatisfied(self, state):
        # Check if the goal condition is met
        satisfied = self.is_goal_met(state, self.goal_state)
        return satisfied

    def is_goal_met(self, state, target):
        same_pos = bool(np.sqrt((state.getX() - target.getX()) ** 2 + (state.getY() - target.getY()) ** 2) < 80)
        in_fov, facing_target = False, False
        if same_pos:
            robot_w = np.array([state.getX(), state.getY(), state.getYaw()]).reshape((3, 1))
            in_fov = polygon_sdf(self.omni_polygon, robot_w[0:2].T) < -15
            facing_target = np.abs(np.arctan2(target.getY() - state.getY(),
                                              target.getX() - state.getX()) - state.getYaw()) < 0.5 * np.pi / 3

            # if in_fov and facing_target: print(getSDF(state, target, sim_map))
        return bool(in_fov and facing_target)


class MotionCost(ob.OptimizationObjective):
    def __init__(self, si):
        super(MotionCost, self).__init__(si)

    def motionCost(self, s1, s2):
        # This method can be used to define the cost of a motion
        # For simplicity, let's just use Euclidean distance as an example
        return ob.Cost(np.sqrt((s1.getX() - s2.getX()) ** 2 + (s1.getY() - s2.getY()) ** 2))


class ValidityChecker(ob.StateValidityChecker):
    """A class to check if an obstacle is in collision or not.
    """

    def __init__(self, si, CurMap, mpt_mask, res=0.05, robot_radius=robot_radius, ):
        """
        Initialize the class object, with the current maps and mask generated
        from the transformer model.
        :param si: an object of type ompl.base.SpaceInformation
        :param CurMap: A np.array with the current maps.
        :param MapMask: Areas of the maps to be masked.
        """
        super().__init__(si)
        self.size = CurMap.shape
        self.map = CurMap
        self.map_origin = np.array([[len(self.map) // 2], [len(self.map[0]) // 2], [np.pi / 2]])
        self.patch_map = mpt_mask

    def isValid(self, state):
        """
        Check if the given state is valid.
        :param state: An ob.State object to be checked.
        :returns bool: True if the state is valid.
        """
        x, y = state.getX(), state.getY()
        pix_dim = world2map(self.map_origin, 2, np.array([[x], [y], [0]])).astype(np.int16)
        if pix_dim[0] < 0 or pix_dim[0] >= self.size[0] or pix_dim[1] < 0 or pix_dim[1] >= self.size[1]:
            return False
        return bool(self.map[pix_dim[0], pix_dim[1]][0]) and bool(self.patch_map[pix_dim[0], pix_dim[1]][0])


def get_path(start, goal, sim_map, mpt_mask, step_time=0.1, max_time=300.):
    '''
    Plan a path given the start, goal and patch_map.
    :param start: The SE(2) co-ordinate of start position
    :param goal: The SE(2) co-ordinate of goal position
    :param step_time: The time step for the planner.
    :param max_time: The maximum time to plan
    :param exp: If true, the planner switches between exploration and exploitation.
    returns tuple: Returns path array, time of solution, number of vertices, success.
    '''
    # Tried importance sampling, but seems like it makes not much improvement
    # over rejection sampling.

    StartState = ob.State(space)
    StartState().setX(start[0])
    StartState().setY(start[1])
    StartState().setYaw(start[2])

    GoalState = ob.State(space)
    GoalState().setX(goal[0])
    GoalState().setY(goal[1])
    GoalState().setYaw(goal[2])

    # setup validity checker
    ValidityCheckerObj = ValidityChecker(si, sim_map, mpt_mask)

    ss.setStateValidityChecker(ValidityCheckerObj)

    ss.setOptimizationObjective(MotionCost(si))
    ss.clear()
    ss.clearStartStates()
    ss.addStartState(StartState)
    ss.setGoal(VisibilityGoal(si, GoalState, sim_map))

    ode = oc.ODE(kinematicUnicycleODE)
    odeSolver = oc.ODEBasicSolver(ss.getSpaceInformation(), ode)
    propagator = oc.ODESolver.getStatePropagator(odeSolver)
    ss.setStatePropagator(propagator)
    ss.getSpaceInformation().setPropagationStepSize(0.2)
    ss.getSpaceInformation().setMinMaxControlDuration(5, 5)

    # planner = oc.SST(ss.getSpaceInformation())
    planner = oc.RRT(ss.getSpaceInformation())

    ss.setPlanner(planner)
    time = step_time
    ss.solve(step_time)
    solved = False

    path = []
    controls = []
    while time < max_time:
        ss.solve(step_time)
        time += step_time
        if ss.haveExactSolutionPath():
            print("Found Solution")
            solved = True
            path = np.array([[ss.getSolutionPath().getState(i).getX(),
                              ss.getSolutionPath().getState(i).getY(),
                              ss.getSolutionPath().getState(i).getYaw()]
                             for i in range(ss.getSolutionPath().getStateCount())])
            controls = np.array([[ss.getSolutionPath().getControl(i)[0],
                                  ss.getSolutionPath().getControl(i)[1]]
                                 for i in range(ss.getSolutionPath().getControlCount())])
            # cost = ss.getSolutionPath().asGeometric().cost(getBalancedObjective(si, GoalState)).value()
            # print("Cost: ", cost)
            break
    plannerData = ob.PlannerData(si)
    planner.getPlannerData(plannerData)
    numVertices = plannerData.numVertices()

    ss.clear()
    ss.clearStartStates()
    return controls, path, time, numVertices, solved


def load_model(model_path):
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    modelFolder = model_path
    modelFile = osp.join(modelFolder, f'model_params.json')
    model_param = json.load(open(modelFile))
    transformer = Models.Transformer(**model_param)
    transformer.to(device)
    # checkpoint = torch.load(osp.join(modelFolder, f'model_weights.pkl'), map_location=torch.device('cpu'))
    checkpoint = torch.load(osp.join(modelFolder, f'model_weights.pkl'))
    transformer.load_state_dict(checkpoint['state_dict'])
    transformer.eval()
    return transformer


def planner_single(start, goal, if_mpt=False, model=None):
    cols, rows = sim_map.shape
    small_map = cv2.resize(sim_map, (cols // 2, rows // 2))
    small_map = (small_map / np.max(small_map)).astype(np.uint8)
    start = start.reshape((3,))
    goal = goal.reshape((3,))
    start_m = world2map(map_origin, map_ratio, start).squeeze()[0:2]
    goal_m = world2map(map_origin, map_ratio, goal).squeeze()[0:2]
    time_inference = time.time()
    if if_mpt:
        patch_map, _ = get_patch(model,
                                 (0.5 * start_m).astype(np.int16)[::-1],
                                 (0.5 * goal_m).astype(np.int16)[::-1],
                                 small_map)
        fs_patch_map = cv2.resize(patch_map, sim_map.shape[::-1]) * 255
    else:
        fs_patch_map = np.ones_like(sim_map)
    # print('MPT takes: '+str(round(time.time()-time_inference, 5))+'s for inferencing')
    actions, trajectory, planner_time, numVer, success = get_path(start, goal, sim_map, fs_patch_map, max_time=planner_max_time)
    if len(actions) == 0:
        actions = None
    traj_m = None
    if success:
        traj_m = world2map(map_origin, map_ratio, trajectory.T)[0:2].T.astype(np.int16)
        start = trajectory[1]
        start[2] = normalize_angle(start[2])
    else:
        trajectory = start.reshape((1, 3))
    return start, trajectory, success, actions


class PlannerThread:
    def __init__(self, model_path='pursuer/planner/models/final_models/point_robot',
                 loop_delay: float = 0.01):
        """
        model_path   Path to your transformer model
        loop_delay   Seconds to sleep between planning attempts
        """
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._start = None  # numpy array [x,y,yaw]
        self._goal = None
        self._result = None  # tuple from planner_single
        self._shutdown = False

        # load model once
        self._model = load_model(model_path)

        # how long to wait between replans
        self._delay = loop_delay

        # start background thread
        self._thread = threading.Thread(target=self._planning_loop, daemon=True)
        self._thread.start()

    def set_start_goal(self, start: np.ndarray, goal: np.ndarray):
        """
        Atomically update start+goal for the planner to pick up on its next iteration.
        """
        with self._cv:
            self._start = start.copy()
            self._goal = goal.copy()
            # wake up the thread if it's still waiting for the first problem
            self._cv.notify()

    def get_result(self):
        """
        Fetch the most recent planning result.
        Returns the tuple (new_start, trajectory, success, actions) or None.
        """
        with self._lock:
            return self._result

    def shutdown(self):
        """
        Signal the planning thread to exit cleanly, then join.
        """
        with self._cv:
            self._shutdown = True
            self._cv.notify()
        self._thread.join()

    def _planning_loop(self):
        """
        Wait for first start/goal, then continuously plan.
        """
        # wait until set_start_goal is called at least once
        with self._cv:
            self._cv.wait_for(lambda: self._start is not None or self._shutdown)
            if self._shutdown:
                return
        last_success = [[], []]
        last_success_index = 1
        while True:
            with self._lock:
                s = self._start
                g = self._goal

            # run your planner_single; returns (new_start, trajectory, success, actions)
            _, trajectory, success, actions = planner_single(s, g, if_mpt=True, model=self._model)
            if success:
                with self._lock:
                    self._result = (trajectory, actions)

            # check shutdown flag
            with self._cv:
                if self._shutdown:
                    return

            # brief pause to avoid 100% CPU spin
            time.sleep(self._delay)

