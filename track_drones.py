#!/usr/bin/env python3

import time
import sys
import subprocess
from swarm_tracker import SwarmTracker

# --- CONFIGURATION ---
# Only these specific names will be displayed
TARGET_DRONES = [
    "iris_with_gimbal",
    "iris_with_gimbal1",
    "iris_with_gimbal2",
    "iris_with_gimbal3",
    "iris_with_gimbal4"
]

def find_pose_topic():
    """Runs 'gz topic -l' to find the correct dynamic_pose topic."""
    try:
        result = subprocess.check_output(["gz", "topic", "-l"], text=True)
        topics = result.split('\n')
        for topic in topics:
            if "dynamic_pose/info" in topic:
                parts = topic.split('/')
                if len(parts) > 2 and parts[1] == 'world':
                    return parts[2]
    except Exception:
        return None
    return None

def main():
    # 1. Auto-detect World Name
    world_name = find_pose_topic()
    if not world_name:
        print("Error: Could not find Gazebo topic. Is the simulation running?")
        return

    print(f"Detected World: {world_name}")
    print(f"Tracking Targets: {TARGET_DRONES}")

    # 2. Start Tracker
    # We pass the list to the tracker so it ignores other data internally
    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world_name)
    tracker.start()

    print("Waiting for data stream...")
    while not tracker.is_active():
        time.sleep(1)

    # 3. Display Loop
    try:
        while True:
            positions = tracker.get_positions()
            
            # Clear Screen
            print("\033[H\033[J", end="") 
            print("-" * 65)
            print(f"{'DRONE NAME':<20} | {'X (m)':<10} | {'Y (m)':<10} | {'ALT (m)':<10}")
            print("-" * 65)

            found_count = 0
            
            # Loop through our SPECIFIC target list to maintain order
            for name in TARGET_DRONES:
                if name in positions:
                    pos = positions[name]
                    print(f"{name:<20} | {pos['x']:<10.2f} | {pos['y']:<10.2f} | {pos['z']:<10.2f}")
                    found_count += 1
                else:
                    # Optional: Print placeholder if drone is missing from Gazebo
                    print(f"{name:<20} | {'--':<10} | {'--':<10} | {'--':<10}")

            print("-" * 65)
            
            if found_count == 0:
                print("Waiting for targets to appear...")
            
            time.sleep(0.2) 

    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()