import threading
import time
import numpy as np


class Invader:
    def __init__(self, trajectory_file, update_rate_hz=300):
        self.trajectory = self._load_trajectory(trajectory_file)
        self.update_rate = update_rate_hz
        self.current_index = 0
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.current_state = self.trajectory[0] if len(self.trajectory) > 0 else np.zeros(6)

    def _load_trajectory(self, file_path):
        data = np.load(file_path)['arr_0']  # expects an Nx6 array
        if data.ndim != 2 or data.shape[1] != 6:
            raise ValueError("Trajectory file must be Nx6 array: [x, y, theta, xdot, ydot, thetadot]")
        return data

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_trajectory)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _run_trajectory(self):
        while self.running and self.current_index < len(self.trajectory):
            with self.lock:
                self.current_state = self.trajectory[self.current_index]
                self.current_index += 1
            time.sleep(1.0 / self.update_rate)

    def get_pose(self):
        with self.lock:
            return self.current_state[:3]  # (x, y, theta)

    def get_velocity(self):
        with self.lock:
            return self.current_state[3:]  # (xdot, ydot, thetadot)

    def get_full_state(self):
        with self.lock:
            return self.current_state.copy()  # (x, y, theta, xdot, ydot, thetadot)
