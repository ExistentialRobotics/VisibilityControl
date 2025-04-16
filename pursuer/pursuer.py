import numpy as np
import threading
import time


class Pursuer:
    def __init__(self, init_pose=np.array([[0.0], [0.0], [0.0]]), dt=0.1):
        self.pose = np.array(init_pose, dtype=float)  # x, y, theta
        self.dt = dt
        self.v = 0.0
        self.omega = 0.0
        self.lock = threading.Lock()
        self.sensors = []
        self.running = False
        self.thread = None

    def set_control(self, v, omega):
        """Set the control input (linear and angular velocity)."""
        with self.lock:
            self.v = v
            self.omega = omega

    def get_pose(self):
        with self.lock:
            return self.pose.copy()

    def attach_sensor(self, sensor):
        """Attach a sensor (must have a `update(robot_pose)` method)."""
        self.sensors.append(sensor)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _run(self):
        while self.running:
            with self.lock:
                x, y, theta = self.pose
                dx = self.v * np.cos(theta) * self.dt
                dy = self.v * np.sin(theta) * self.dt
                dtheta = self.omega * self.dt
                self.pose = np.array([[x + dx], [y + dy], [theta + dtheta]])

                # Update sensors
                for sensor in self.sensors:
                    sensor.update(self.pose)

            time.sleep(self.dt)
