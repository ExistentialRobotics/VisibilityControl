import datetime

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec

from env.config import *

obstacles_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, obstacle_path))
obstacles = np.load(obstacles_file_path, allow_pickle=True)

fig = plt.figure(figsize=(10, 12))
gs = GridSpec(3, 1, height_ratios=[3, 0.8, 0.8])
ax1 = plt.subplot(gs[0])
ax2 = plt.subplot(gs[1])
ax3 = plt.subplot(gs[2])

for obstacle in obstacles:
    ax1.fill(obstacle[:, 0], obstacle[:, 1])
rbt_line = ax1.plot([], [], linestyle='dashed', color='red', label="Robot Trajectory")[0]
vis_region_line = ax1.plot([], [], linestyle='dashed', color='orange', label="FOV")[0]
patches = []
tgt_line = ax1.plot([], [], linestyle='dashed', color='blue', label="Target")[0]
tgt_ort_line = ax1.plot([], [], linestyle='dotted', color='green', label="Target orientation", alpha=0.5)[0]
rbt_ort_line = ax1.plot([], [], linestyle='dotted', color='green', label="Robot orientation", alpha=0.5)[0]
# future_tgt_scat = ax1.scatter([], [], color="tab:blue", label="Target Future Trajectory", s=1, alpha=0.5)
ax3_line = ax3.plot([], [], linestyle='-', color='green', label="Angular Velocity (rad/s)")[0]
ax3.set(xlim=[0, horizon], ylim=[-3.14, 3.14], xlabel='Time', ylabel='Angular Velocity (rad/s)')
ax1.set(xlim=[-200, 200], ylim=[-150, 150], xlabel='X', ylabel='Y')
sdf_line = ax2.plot([], [], linestyle='-', color='blue', label="SDF value")[0]
text = ax1.text(-170, 170, [], fontsize=10, ha='right', va='bottom')
text2 = ax1.text(-170, 170, [], fontsize=10, ha='right', va='top')
ax2.set(xlim=[0, horizon], ylim=[0, 1], xlabel='Time', ylabel='SDF value')
planner_result = ax1.plot([], [], linestyle='-', color='blue', label="Planner", alpha=0.7)[0]


def find_new_points(x, y, theta, radius=50):
    ex = x + np.cos(theta) * radius
    ey = y + np.sin(theta) * radius
    return ex, ey


# add patches to arg list in 5th position for circular sdf, comment out vis_region lines in function and uncomment ax1.add_patch
def update_frame(frame, x_record, y_record, fov_record, tgt_future_traj, sdf_record, omega_record, planner_record):
    if use_planner: planner_result.set_data(planner_record[frame][:, 0], planner_record[frame][:, 1])
    rbt_line.set_data(x_record[:, :frame + 1][0], x_record[:, :frame + 1][1])
    tgt_line.set_data(y_record[:, :frame + 1][0], y_record[:, :frame + 1][1])
    ymin3, ymax3 = ax3.get_ylim()
    ax3.set_ylim(min(ymin3, min(omega_record[:frame + 1])), max(ymax3, max(omega_record[:frame + 1])))
    ax3_line.set_data(list(range(frame + 1)), omega_record[:frame + 1])
    vis_region = fov_record[frame]
    vis_region_line.set_data(vis_region[:, 0], vis_region[:, 1])
    tgt_ort_x, tgt_ort_y = find_new_points(y_record[0, frame], y_record[1, frame], y_record[2, frame])
    tgt_ort_line.set_data([y_record[0, frame], tgt_ort_x], [y_record[1, frame], tgt_ort_y])
    rbt_ort_x, rbt_ort_y = find_new_points(x_record[0, frame], x_record[1, frame], x_record[2, frame])
    rbt_ort_line.set_data([x_record[0, frame], rbt_ort_x], [x_record[1, frame], rbt_ort_y])
    # future_tgt_scat.set_offsets(tgt_future_traj[:2, frame:frame + K].T)
    ymin, ymax = ax2.get_ylim()
    ax2.set_ylim(min(ymin, min(sdf_record[:frame + 1])), max(ymax, max(sdf_record[:frame + 1])))
    sdf_line.set_data(list(range(frame + 1)), sdf_record[:frame + 1])
    text.set_text(str(frame))
    return


def animate(data_folder, i, x_record, y_record, fov_record, tgt_future_traj, sdf_record, omega_record, planner_record):
    ani = animation.FuncAnimation(
        fig=fig,
        func=update_frame,
        fargs=(x_record, y_record, fov_record, tgt_future_traj, sdf_record, omega_record, planner_record),
        frames=i,
        interval=100
    )
    save_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    ani.save(os.path.join(data_folder, './simulations/traj_{x}.mp4'.format(x=str(save_time))))
