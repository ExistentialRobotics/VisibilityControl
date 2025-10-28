from env.config import map_file_path
from cv2 import threshold, imread, IMREAD_GRAYSCALE, THRESH_BINARY

class Map:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        # Initialize the instance (only called once)
        self.map_file_path = map_file_path
        _, self.map = threshold(imread(map_file_path, IMREAD_GRAYSCALE), 1, 255, THRESH_BINARY)

    def __call__(self):
        return self.map
