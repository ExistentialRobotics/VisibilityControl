class DepthCamera:
    def __init__(self, name="DepthCamera"):
        self.name = name
        self.latest_pose = None

    def update(self, robot_pose):
        self.latest_pose = robot_pose
        print(f"[{self.name}] Captured at pose: {robot_pose}")
