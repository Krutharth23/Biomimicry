#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
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
HISTORY_LEN = 50 # How many past points to show (trails)

def get_world_name():
    try:
        res = subprocess.check_output(["gz", "topic", "-l"], text=True)
        for t in res.split('\n'):
            if "dynamic_pose/info" in t:
                return t.split('/')[2]
    except: return None

def main():
    # 1. Setup Tracker
    world = get_world_name()
    if not world:
        print("Error: Gazebo not running or topic not found.")
        return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()

    print(f"Plotter attached to world: {world}")
    print("Waiting for data...")
    while not tracker.is_active():
        time.sleep(1)

    # 2. Setup Matplotlib
    plt.ion() # Interactive mode on
    fig = plt.figure(figsize=(10, 8))
    gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])

    # -- Subplot 1: XY Map (Top Down) --
    ax_map = fig.add_subplot(gs[0])
    ax_map.set_title("Swarm Position (XY Plane)")
    ax_map.set_xlabel("East (Y) - Meters")
    ax_map.set_ylabel("North (X) - Meters")
    ax_map.grid(True)
    ax_map.set_xlim(-15, 15)
    ax_map.set_ylim(-15, 15)
    ax_map.set_aspect('equal')

    # Draw the target circle for reference
    circle = plt.Circle((0, 0), 10.0, color='g', fill=False, linestyle='--', label='Target Radius')
    ax_map.add_patch(circle)

    # -- Subplot 2: Altitude (Side View) --
    ax_alt = fig.add_subplot(gs[1])
    ax_alt.set_title("Altitude (Z)")
    ax_alt.set_ylabel("Height (m)")
    ax_alt.set_ylim(0, 15) # Assuming target is 10m
    ax_alt.grid(True)

    # Initialize Data Structures
    # Dictionary to store lines/scatters for each drone
    drone_gfx = {}
    colors = plt.cm.get_cmap('tab10', len(TARGET_DRONES))

    for i, name in enumerate(TARGET_DRONES):
        drone_gfx[name] = {
            'trail_x': deque(maxlen=HISTORY_LEN),
            'trail_y': deque(maxlen=HISTORY_LEN),
            'trail_z': deque(maxlen=HISTORY_LEN),
            'time': deque(maxlen=HISTORY_LEN),
            # Map Plot Elements
            'scat': ax_map.plot([], [], 'o', color=colors(i), markersize=8)[0],
            'line': ax_map.plot([], [], '-', color=colors(i), alpha=0.5)[0],
            'text': ax_map.text(0, 0, f"D{i}", fontsize=9),
            # Alt Plot Elements
            'alt_line': ax_alt.plot([], [], '-', color=colors(i), label=f"D{i}")[0]
        }

    ax_alt.legend(loc='upper right', ncol=5, fontsize='small')
    
    start_time = time.time()

    try:
        while True:
            # Get Snapshot of positions
            positions = tracker.get_positions()
            current_t = time.time() - start_time

            for name in TARGET_DRONES:
                if name in positions:
                    pos = positions[name]
                    gfx = drone_gfx[name]

                    # Append Data
                    gfx['trail_x'].append(pos['x'])
                    gfx['trail_y'].append(pos['y'])
                    gfx['trail_z'].append(pos['z'])
                    gfx['time'].append(current_t)

                    # Update Map (XY)
                    gfx['scat'].set_data([pos['x']], [pos['y']]) # Must be sequence
                    gfx['line'].set_data(gfx['trail_x'], gfx['trail_y'])
                    gfx['text'].set_position((pos['x'] + 0.5, pos['y'] + 0.5))

                    # Update Altitude (Time vs Z)
                    gfx['alt_line'].set_data(gfx['time'], gfx['trail_z'])

            # Dynamic Scaling for Altitude Time Axis
            ax_alt.set_xlim(max(0, current_t - 10), current_t + 1)
            
            # Draw
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            # Throttle loop to spare CPU for the simulator
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nPlotter Closed.")
        plt.close()

if __name__ == "__main__":
    main()