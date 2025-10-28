import carla
import numpy as np
import time
import random
import pickle


def visualize_path_only(client, trajectory, color, z_height=0.1):
    """
    Visualizes a trajectory in CARLA without a vehicle.

    Args:
        client (carla.Client): The CARLA client.
        trajectory (numpy.ndarray): The trajectory points as an n*2 NumPy array.
        z_height (float): The height at which to place the trajectory points.
    """
    world = client.get_world()

    # Convert NumPy array to CARLA locations
    locations = [carla.Location(x=float(point[0]), y=-float(point[1]), z=z_height) for point in trajectory]

    # Draw the trajectory in the environment
    for i in range(len(locations) - 1):
        start = locations[i]
        end = locations[i + 1]
        world.debug.draw_line(start, end, thickness=0.75, color=color, life_time=24.0)


def main():
    # Connect to a running CARLA instance
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    with open('CARLA_data_everything_updated2.pkl', 'rb') as file:
        data = pickle.load(file)

    # Define a simple trajectory as an n*2 NumPy array
    rbt = np.array(data[4])
    tgt = np.array(data[5])

    # Visualize the trajectory
    visualize_path_only(client, tgt, carla.Color(0, 0, 16))
    visualize_path_only(client, rbt, carla.Color(32, 0, 0))
    # Allow some time to view before cleanup
    time.sleep(150)

    # Clean up
    vehicle.destroy()


if __name__ == "__main__":
    main()
