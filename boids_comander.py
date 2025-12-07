#!/usr/bin/env python3

import time
import json
import socket
import math
import subprocess
import numpy as np
from swarm_tracker import SwarmTracker

# --- CONFIGURATION ---
TARGET_DRONES = [
    "iris_with_gimbal", "iris_with_gimbal1", 
    "iris_with_gimbal2", "iris_with_gimbal3", 
    "iris_with_gimbal4"
]
MANAGER_IP = 'localhost'
MANAGER_PORT = 14000

# --- BOIDS PARAMETERS (TUNED FOR SAFETY) ---
PERCEPTION_RADIUS = 8.0  # Increased: See neighbors from further away
PROTECTED_RANGE = 5.0    # Increased: Start separating at 5 meters
MAX_SPEED = 2.5          # Reduced: Slower flight gives more time to react

# Weights (Tuned to prioritize Anti-Collision)
W_SEPARATION = 2       # HIGHEST PRIORITY: Don't crash
W_COHESION = 1         # LOW PRIORITY: Don't clump together
W_ALIGNMENT = 1.0        # Medium: Fly roughly the same way
W_MIGRATION = 1.0        # High: Go to the circle

# Mission Goals
CIRCLE_RADIUS = 10.0     # Increased: 20m Diameter Circle
TARGET_ALTITUDE = 10.0

def get_world_name():
    try:
        res = subprocess.check_output(["gz", "topic", "-l"], text=True)
        for t in res.split('\n'):
            if "dynamic_pose/info" in t:
                return t.split('/')[2]
    except: return None

class Boid:
    def __init__(self, name):
        self.name = name
        self.pos = np.array([0.0, 0.0]) # x, y
        self.vel = np.array([0.0, 0.0]) 
        self.alt = 0.0

def calculate_migration_force(pos):
    """
    Returns a vector that pushes the drone to orbit (0,0).
    """
    # Vector from center to drone
    x, y = pos[0], pos[1]
    dist = math.sqrt(x**2 + y**2)
    if dist < 0.1: dist = 0.1

    # Tangent Vector (Orbit logic)
    tan_x = -y / dist 
    tan_y =  x / dist 

    # Radial Vector (Pull to radius)
    err_r = CIRCLE_RADIUS - dist
    rad_x = (x / dist) * err_r 
    rad_y = (y / dist) * err_r 

    return np.array([tan_x + rad_x, tan_y + rad_y])

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    world = get_world_name()
    if not world: return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()
    
    # Initialize Boid Objects
    boids = {name: Boid(name) for name in TARGET_DRONES}

    print(f"Boids Commander Active on world: {world}")
    print("Waiting for data...")
    while not tracker.is_active():
        time.sleep(1)
    print("Swarm Acquired. Flocking started.")

    try:
        while True:
            raw_positions = tracker.get_positions()
            packet = {}

            # 1. UPDATE STATE
            valid_boids = []
            for name in TARGET_DRONES:
                if name in raw_positions:
                    p = raw_positions[name]
                    # Gazebo X=East, Y=North
                    boids[name].pos = np.array([p['x'], p['y']]) 
                    boids[name].alt = p['z']
                    valid_boids.append(boids[name])

            # 2. CALCULATE BOIDS RULES
            for me in valid_boids:
                v_sep = np.array([0.0, 0.0])
                v_coh = np.array([0.0, 0.0])
                
                neighbors = 0
                avg_pos = np.array([0.0, 0.0])

                for other in valid_boids:
                    if other == me: continue
                    
                    dist = np.linalg.norm(me.pos - other.pos)

                    if dist < PERCEPTION_RADIUS:
                        neighbors += 1
                        
                        # --- RULE 1: SEPARATION (IMPROVED) ---
                        if dist < PROTECTED_RANGE:
                            diff = me.pos - other.pos
                            # Inverse Square Law: Force gets huge as dist gets small
                            # Added a small epsilon (0.01) to prevent divide by zero
                            weight = 1.0 / (dist * dist + 0.01)
                            v_sep += diff * weight

                        # Accumulate for Cohesion
                        avg_pos += other.pos

                if neighbors > 0:
                    # --- RULE 2: COHESION ---
                    avg_pos /= neighbors
                    v_coh = avg_pos - me.pos
                    if np.linalg.norm(v_coh) > 0:
                        v_coh = (v_coh / np.linalg.norm(v_coh))

                # --- RULE 3: MIGRATION ---
                v_mig = calculate_migration_force(me.pos)

                # 3. SUM VECTORS & WEIGHTS
                total_force = (v_sep * W_SEPARATION) + \
                              (v_coh * W_COHESION) + \
                              (v_mig * W_MIGRATION)
                
                # Update "Velocity"
                cmd_vel = total_force

                # Clamp Speed
                speed = np.linalg.norm(cmd_vel)
                if speed > MAX_SPEED:
                    cmd_vel = (cmd_vel / speed) * MAX_SPEED

                # 4. VERTICAL & COORDINATE CONVERSION
                err_z = TARGET_ALTITUDE - me.alt
                vz_cmd = -1 * (err_z * 0.8) # Lower Z gain for smoother altitude
                
                # Gazebo [X, Y] -> MAVSDK [North, East]
                # Gazebo X = East, Gazebo Y = North
                vel_north = cmd_vel[1] 
                vel_east  = cmd_vel[0]
                
                yaw = math.degrees(math.atan2(vel_east, vel_north))

                # Find index
                idx = TARGET_DRONES.index(me.name)
                packet[str(idx)] = [vel_north, vel_east, vz_cmd, yaw]

            # 5. SEND
            if packet:
                sock.sendto(json.dumps(packet).encode(), (MANAGER_IP, MANAGER_PORT))
            
            # Print Status
            print("\033[H\033[J", end="") 
            print("BOIDS ACTIVE (Collision Avoidance Mode)")
            print(f"Goal: Radius {CIRCLE_RADIUS}m, Alt {TARGET_ALTITUDE}m")
            print(f"Separation Weight: {W_SEPARATION} (Range: {PROTECTED_RANGE}m)")

            time.sleep(0.05) # 20Hz

    except KeyboardInterrupt:
        print("Stopping.")

if __name__ == "__main__":
    main()