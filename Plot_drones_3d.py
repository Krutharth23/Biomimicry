#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import numpy as np
import time
import subprocess
from swarm_tracker import SwarmTracker

# --- CONFIGURATION ---
TARGET_DRONES = [
    "iris_with_gimbal", "iris_with_gimbal1", 
    "iris_with_gimbal2", "iris_with_gimbal3", 
    "iris_with_gimbal4"
]
HISTORY_LEN = 30 # Trail length

# Goal Sphere Params (For visualization only)
SPHERE_CENTER = (0, 0, 15)
SPHERE_RADIUS = 8.0

def get_world_name():
    try:
        res = subprocess.check_output(["gz", "topic", "-l"], text=True)
        for t in res.split('\n'):
            if "dynamic_pose/info" in t:
                return t.split('/')[2]
    except: return None

def generate_sphere_wireframe(center, radius):
    """Generates 3D coordinates for a sphere wireframe"""
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = radius * np.cos(u) * np.sin(v) + center[0]
    y = radius * np.sin(u) * np.sin(v) + center[1]
    z = radius * np.cos(v) + center[2]
    return x, y, z

def main():
    world = get_world_name()
    if not world: return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()

    print(f"3D Plotter attached to world: {world}")
    while not tracker.is_active():
        time.sleep(1)

    # --- SETUP 3D PLOT ---
    plt.ion()
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Arena Settings
    ax.set_title("3D Swarm Real-time View")
    ax.set_xlabel("East (X)")
    ax.set_ylabel("North (Y)")
    ax.set_zlabel("Altitude (Z)")
    
    # Set fixed limits so the view doesn't jump around
    LIMIT = 20
    ax.set_xlim(-LIMIT, LIMIT)
    ax.set_ylim(-LIMIT, LIMIT)
    ax.set_zlim(0, 30)

    # Draw Goal Sphere (Wireframe)
    sx, sy, sz = generate_sphere_wireframe(SPHERE_CENTER, SPHERE_RADIUS)
    ax.plot_wireframe(sx, sy, sz, color='gray', alpha=0.2, linewidth=0.5)
    
    # Draw Ground
    xx, yy = np.meshgrid(range(-20, 21, 5), range(-20, 21, 5))
    zz = np.zeros_like(xx)
    ax.plot_wireframe(xx, yy, zz, color='green', alpha=0.1)

    # Initialize Drone Graphics
    drone_gfx = {}
    colors = plt.cm.get_cmap('tab10', len(TARGET_DRONES))

    for i, name in enumerate(TARGET_DRONES):
        drone_gfx[name] = {
            'x': deque(maxlen=HISTORY_LEN),
            'y': deque(maxlen=HISTORY_LEN),
            'z': deque(maxlen=HISTORY_LEN),
            'scat': ax.plot([], [], [], 'o', color=colors(i), markersize=6, label=name)[0],
            'line': ax.plot([], [], [], '-', color=colors(i), alpha=0.5)[0],
            'shadow': ax.plot([], [], [], 'x', color='gray', alpha=0.3)[0] # Ground shadow
        }

    ax.legend(loc='upper right')
    
    print("Plotter Running. Rotate window with mouse.")

    try:
        while True:
            positions = tracker.get_positions()

            for name in TARGET_DRONES:
                if name in positions:
                    pos = positions[name]
                    gfx = drone_gfx[name]

                    # Append history
                    gfx['x'].append(pos['x'])
                    gfx['y'].append(pos['y'])
                    gfx['z'].append(pos['z'])

                    # Update Scatter (Head)
                    gfx['scat'].set_data([pos['x']], [pos['y']])
                    gfx['scat'].set_3d_properties([pos['z']])

                    # Update Trail
                    gfx['line'].set_data(list(gfx['x']), list(gfx['y']))
                    gfx['line'].set_3d_properties(list(gfx['z']))

                    # Update Shadow (Project to Z=0)
                    gfx['shadow'].set_data([pos['x']], [pos['y']])
                    gfx['shadow'].set_3d_properties([0])

            # Draw
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nPlotter Closed.")
        plt.close()

if __name__ == "__main__":
    main()