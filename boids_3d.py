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

# --- 3D BOIDS PARAMETERS ---
PERCEPTION_RADIUS = 8.0  
PROTECTED_RANGE = 4.0    
MAX_SPEED = 2.5          

# Weights
W_SEPARATION = 5.0       # Strong separation to prevent crashes
W_COHESION = 0.2         # Low cohesion to prevent clumping
W_ALIGNMENT = 1.0        
W_MIGRATION = 1.2        # Pull towards the Sphere
W_FLOOR = 8.0            # Strong force to avoid hitting the ground

# Mission Goals (A Sphere in the sky)
SPHERE_CENTER = np.array([0.0, 0.0, 15.0]) # Center at 15m altitude
SPHERE_RADIUS = 8.0      

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
        # 3D Vectors now!
        self.pos = np.array([0.0, 0.0, 0.0]) 
        self.vel = np.array([0.0, 0.0, 0.0]) 

def calculate_sphere_migration(pos):
    """
    Pushes the drone onto the surface of a 3D Sphere.
    Also adds a tangent force to make them swirl around it.
    """
    # Vector from Sphere Center to Drone
    rel_pos = pos - SPHERE_CENTER
    dist = np.linalg.norm(rel_pos)
    
    if dist < 0.1: dist = 0.1
    
    # 1. Radial Force: Pull/Push to surface
    err_r = SPHERE_RADIUS - dist
    # Normal vector pointing out from center
    normal = rel_pos / dist
    f_radial = normal * err_r
    
    # 2. Swirl Force (Cross Product)
    # We cross the normal with an "Up" vector to get a horizontal swirl
    up = np.array([0.0, 0.0, 1.0])
    f_swirl = np.cross(normal, up)
    
    # Combine: Pull to surface + Swirl around it
    return f_radial + (f_swirl * 1.5)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    world = get_world_name()
    if not world: return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()
    
    boids = {name: Boid(name) for name in TARGET_DRONES}

    print(f"3D Boids Commander Active on world: {world}")
    print("Waiting for data...")
    while not tracker.is_active():
        time.sleep(1)
    print("Swarm Acquired. 3D Flocking started.")

    try:
        while True:
            raw_positions = tracker.get_positions()
            packet = {}

            # 1. UPDATE STATE (3D)
            valid_boids = []
            for name in TARGET_DRONES:
                if name in raw_positions:
                    p = raw_positions[name]
                    # Gazebo X=East, Y=North, Z=Up
                    boids[name].pos = np.array([p['x'], p['y'], p['z']]) 
                    valid_boids.append(boids[name])

            # 2. CALCULATE FORCES
            for me in valid_boids:
                v_sep = np.array([0.0, 0.0, 0.0])
                v_coh = np.array([0.0, 0.0, 0.0])
                v_floor = np.array([0.0, 0.0, 0.0])
                
                neighbors = 0
                avg_pos = np.array([0.0, 0.0, 0.0])

                for other in valid_boids:
                    if other == me: continue
                    
                    # 3D Distance
                    dist = np.linalg.norm(me.pos - other.pos)

                    if dist < PERCEPTION_RADIUS:
                        neighbors += 1
                        
                        # --- 3D SEPARATION ---
                        # If neighbor is ABOVE me, this pushes me DOWN.
                        # If neighbor is BELOW me, this pushes me UP.
                        if dist < PROTECTED_RANGE:
                            diff = me.pos - other.pos
                            weight = 1.0 / (dist * dist + 0.01)
                            v_sep += diff * weight

                        avg_pos += other.pos

                if neighbors > 0:
                    # --- 3D COHESION ---
                    avg_pos /= neighbors
                    v_coh = avg_pos - me.pos
                    if np.linalg.norm(v_coh) > 0:
                        v_coh = (v_coh / np.linalg.norm(v_coh))

                # --- 3D MIGRATION (Sphere) ---
                v_mig = calculate_sphere_migration(me.pos)

                # --- GROUND FLOOR SAFETY ---
                # If altitude < 2m, push UP hard
                if me.pos[2] < 3.0:
                    # The closer to 0, the harder the push
                    push = (3.0 - me.pos[2]) 
                    v_floor = np.array([0.0, 0.0, push])

                # 3. SUM VECTORS
                total_force = (v_sep * W_SEPARATION) + \
                              (v_coh * W_COHESION) + \
                              (v_mig * W_MIGRATION) + \
                              (v_floor * W_FLOOR)
                
                cmd_vel = total_force

                # Clamp Speed
                speed = np.linalg.norm(cmd_vel)
                if speed > MAX_SPEED:
                    cmd_vel = (cmd_vel / speed) * MAX_SPEED

                # 4. COORDINATE CONVERSION (CRITICAL)
                # Gazebo: X=East, Y=North, Z=Up
                # MAVSDK: N=North, E=East, D=Down
                
                vel_east  = cmd_vel[0] # Gazebo X
                vel_north = cmd_vel[1] # Gazebo Y
                vel_up    = cmd_vel[2] # Gazebo Z
                
                # MAVSDK NED requires Down velocity
                # If Gazebo says "Go Up (+)", MAVSDK needs "Go Down (-)"
                vel_down = -vel_up 

                # Yaw: Face movement direction
                yaw = math.degrees(math.atan2(vel_east, vel_north))

                # Store
                idx = TARGET_DRONES.index(me.name)
                packet[str(idx)] = [vel_north, vel_east, vel_down, yaw]

            # 5. SEND
            if packet:
                sock.sendto(json.dumps(packet).encode(), (MANAGER_IP, MANAGER_PORT))
            
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("Stopping.")

if __name__ == "__main__":
    main()