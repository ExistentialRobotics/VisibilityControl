import numpy as np

class AugmentedEKF:
    def __init__(self, initial_covariance, process_noise_cov, observation_noise_cov):
        self.P = initial_covariance
        self.Q = process_noise_cov
        self.R = observation_noise_cov

    def predict(self, state, dt):
        """
        Prediction step for EKF
        :param state: Target state {x,y,theta,u,v}
        :param dt: Time step (s) between current and previous target detection
        :return: np.ndarray: Predicted state
        """

        # Extract state components for readability
        x, y, theta, u, v = state

        # Predict next state based on current state and unknown velocities
        predicted_x = x + u * dt
        predicted_y = y + v * dt

        # Update state covariance with process noise
        # F is now the Jacobian of the motion model including v and omega in the state
        F = self.calculate_jacobian(state, dt)
        self.P = F @ self.P @ F.T + self.Q
        # print("Inside prediction step: ", predicted_x, predicted_y, theta, u, v)

        return np.array([predicted_x, predicted_y, theta, u, v])

    def update(self, state, z):

        # Observation model; observe position and velocity (using finite differences)
        H = np.array([[1, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0],
                      [0, 0, 1, 0, 0],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])  # Expanded observation matrix

        # Observation residual
        # print("Inside update step: ", state, z)
        z_pred = H @ state  # Predicted observation from predicted state
        y = z - z_pred
        # print("Y: ", y)

        # Handle angle normalization for theta observation residual
        y[2] = ((y[2] + np.pi) % (2 * np.pi)) - np.pi

        # Residual covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.pinv(S)

        # State update
        updated_state = state + K @ y
        # Ensure theta remains within -pi to pi
        updated_state[2] = ((updated_state[2] + np.pi) % (2 * np.pi)) - np.pi

        # Covariance update
        I = np.eye(len(updated_state))  # Identity matrix
        self.P = (I - K @ H) @ self.P
        # print(self.P)

        return updated_state


    def calculate_jacobian(self, state, dt):
        # Calculate the Jacobian matrix of the augmented state vector
        x, y, theta, u, v = state

        # constant velocity motion model
        return np.array([
            [1, 0, 0, dt, 0],
            [0, 1, 0, 0, dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])