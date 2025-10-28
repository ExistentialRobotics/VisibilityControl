import pickle

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data from the CSV file
file_path = 'demo_full.pkl'
# data = pd.read_csv(file_path)
with open('demo_full.pkl', 'rb') as file:
    data = pickle.load(file)

# Assuming that:
# - The first row (top row) contains timestamps in seconds.
# - The second row contains 'SDF' values (assuming it's at index 0).
# - The fourth row contains 'Obstacle Distance' values (assuming it's at index 2).

# Extracting timestamps from the columns and converting them to NumPy array
timestamps = np.array(data[0])
timestamps = timestamps-timestamps[0]

# Extracting the 'SDF' and 'Obstacle Distance' values and converting to NumPy arrays
h = -np.array(data[1])  # Assuming line 2 is row index 0 for 'SDF'
# obstacle_distance = np.array(data.iloc[2])  # Assuming line 4 is row index 2 for 'Obstacle Distance'

# Plotting
plt.figure(figsize=(10, 3), dpi=600)

# Plot SDF
plt.plot(timestamps, h)

# Plot Obstacle Distance
# plt.plot(timestamps, obstacle_distance, label='Obstacle Distance')

# Adding horizontal line at y=0 in red dashed style
plt.axhline(0, color='red', linestyle='--')

# statuses = ['Target Tracked', 'Target Lost', 'Relocated']
# indices = [98, 183, 248]
# for index, status in zip(indices, statuses):
#     plt.axvline(timestamps[index], color='blue', linestyle='--', alpha=0.5, label=f'{status} ({timestamps[index]:.2f} sec)')

# Customizing the plot
plt.xlabel('Time (seconds)', fontsize=18)  # Larger font size for axis labels
plt.ylabel('SDF (meters)', fontsize=18)
plt.xticks(fontsize=16)  # Larger font size for tick labels
plt.yticks(fontsize=16)
# plt.legend(fontsize=16)  # Larger font size for legend
# plt.grid(True)
plt.tight_layout()

# Show the plot
plt.show()
# plt.savefig(f'{file_path[:-4]}.png', format='png', dpi=600, bbox_inches='tight')