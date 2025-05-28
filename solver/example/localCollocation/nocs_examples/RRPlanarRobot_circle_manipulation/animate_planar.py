import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
import os

# --- Load data ---
file_path = 'RRPlanarRobot_circle_manipulation_solution.txt'
data = pd.read_csv(file_path, sep='\t')

# --- Parameters ---
L1 = L2 = L3 = 1.0
circle_radius = 0.25

# --- Forward kinematics ---
def forward_kinematics(q1, q2, q3):
    x0, y0 = 0, 0
    x1 = x0 + L1 * np.cos(q1)
    y1 = y0 + L1 * np.sin(q1)
    x2 = x1 + L2 * np.cos(q1 + q2)
    y2 = y1 + L2 * np.sin(q1 + q2)
    x3 = x2 + L3 * np.cos(q1 + q2 + q3)
    y3 = y2 + L3 * np.sin(q1 + q2 + q3)
    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

# --- Setup figure ---
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)
ax.set_title("3-Link Manipulator and Object")

# Add origin axes
ax.axhline(0, color='gray', lw=1)
ax.axvline(0, color='gray', lw=1)

# Remove grid
# ax.grid(True)

# Ellipsoid links
link_colors = ['red', 'green', 'blue']
links = [patches.Ellipse((0, 0), width=1.0, height=0.2, angle=0, color=link_colors[i]) for i in range(3)]
for link in links:
    ax.add_patch(link)

# Circle object
circle = plt.Circle((0, 0), circle_radius, color='black', fill=False, lw=2)
ax.add_patch(circle)

# Circle position label
circle_label = ax.text(0, 0, '', fontsize=10, color='black', ha='center', va='bottom')

# --- Utility: update ellipse between two points ---
def update_ellipse(ellipse, p1, p2):
    center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = np.hypot(dx, dy)
    angle = np.degrees(np.arctan2(dy, dx))
    ellipse.set_center(center)
    ellipse.width = length
    ellipse.angle = angle

# --- Animation update function ---
def update(frame):
    q1, q2, q3 = data.loc[frame, ['q1', 'q2', 'q3']]
    xc, yc = data.loc[frame, ['q4', 'q5']]
    points = forward_kinematics(q1, q2, q3)

    # Update links
    for i in range(3):
        update_ellipse(links[i], points[i], points[i + 1])

    # Update circle
    circle.center = (xc, yc)

    # Update label
    circle_label.set_position((xc, yc + circle_radius + 0.1))
    circle_label.set_text(f"({xc:.2f}, {yc:.2f})")

    return (*links, circle, circle_label)

# --- Create animation ---
ani = animation.FuncAnimation(
    fig, update, frames=len(data), blit=True, interval=30, repeat=False
)

# --- Save animation as GIF ---
ani.save('manipulator_animation.gif', writer=PillowWriter(fps=30))

# --- Save individual frames as images ---
output_dir = 'frames'
os.makedirs(output_dir, exist_ok=True)

for frame in range(len(data)):
    update(frame)
    plt.savefig(f'{output_dir}/frame_{frame:04d}.png')

plt.show()
