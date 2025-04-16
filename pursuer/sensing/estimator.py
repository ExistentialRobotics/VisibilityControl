import numpy as np

class EKFEstimator:
    def __init__(self, name="EKFTracker", dt=0.1):
        self.name = name
        self.dt = dt

        # State: [x, y, theta]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1  # Initial covariance

        self.Q = np.diag([0.05, 0.05, 0.01])  # Process noise
        self.R = np.diag([0.2, 0.2, 0.1])     # Observation noise

    def predict(self, target_velocity):
        """Perform EKF prediction using target velocity (vx, vy, omega)."""
        vx, vy, omega = target_velocity.flatten()
        theta = self.x[2, 0]

        # Motion model (discrete): x_{t+1} = x_t + v*dt
        dx = np.array([
            [vx * self.dt],
            [vy * self.dt],
            [omega * self.dt]
        ])

        self.x += dx

        # Jacobian of motion model w.r.t. state is identity
        F = np.eye(3)
        self.P = F @ self.P @ F.T + self.Q

    def update_with_observation(self, observed_pose):
        """EKF update with direct observation of the pose."""
        z = observed_pose.reshape((3, 1))  # observation
        H = np.eye(3)                      # Observation model (identity)
        y = z - H @ self.x                # Innovation
        S = H @ self.P @ H.T + self.R     # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    def get_estimated_pose(self):
        return self.x.flatten()  # (x, y, theta)

    def get_covariance(self):
        return self.P.copy()

    def update(self, robot_pose=None):
        # Optional: attach to robot for time-stepping (not required)
        pass
