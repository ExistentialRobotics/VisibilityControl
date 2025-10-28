import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import pickle

import pandas as pd


# Load the data from the CSV file
file_path = 'demo_full.pkl'
# data = pd.read_csv(file_path)
with open('demo_full.pkl', 'rb') as file:
    data = pickle.load(file)
# Sample data (replace these lists with your actual data)
time_stamps = np.array(data[0])
time_stamps = time_stamps-time_stamps[0]
values = -np.array(data[1])

# Create figure and axis
fig, ax = plt.subplots(figsize=(10, 3), dpi=100)
line, = ax.plot([], [], lw=2)

# Set the axis limits
ax.set_xlim(min(time_stamps), max(time_stamps))
ax.set_ylim(-1.0, 60.0)
plt.xlabel('Time (seconds)', fontsize=18)  # Larger font size for axis labels
plt.ylabel('SDF (meters)', fontsize=18)
plt.axhline(0, color='red', linestyle='--')
plt.xticks(fontsize=12)  # Larger font size for tick labels
plt.yticks(fontsize=12)
plt.tight_layout()


# Initialization function for the plot
def init():
    line.set_data([], [])
    return line,

# Update function for the animation
def update(frame):
    print(frame)
    # Update the data in the line for each frame
    line.set_data(time_stamps[:frame], values[:frame])
    ax.set_ylim(min(values[:frame+1]) - 0.1, max(values[:frame+1]) + 0.1)
    return line,

# Create animation object
ani = FuncAnimation(fig, update, frames=len(time_stamps), init_func=init, blit=True)

# Save the animation as MP4 file
ani.save('SDF_all.mp4', writer='ffmpeg')

# Display the animation (optional)
plt.show()
