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

# --- FIGURE 8 PARAMETERS ---
SCALE = 40.0           # Width of the Figure 8 (Meters)
CYCLE_DURATION = 30.0  # Time to complete one full loop (Seconds)
TARGET_ALTITUDE = 15.0 # Height of the flight path

# --- BOIDS WEIGHTS ---
# We need strong migration to keep up with the moving target
W_SEPARATION = 2.5    
W_COHESION = 2.0      
W_ALIGNMENT = .6     
W_MIGRATION = 2.5     # High weight: Chase the Rabbit!

PERCEPTION_RADIUS = 8.0
PROTECTED_RANGE = 4.0
MAX_SPEED = 4.5       # Increased speed to allow chasing

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
        self.pos = np.array([0.0, 0.0, 0.0]) 

def get_virtual_leader_pos(t_sec):
    """
    Calculates the position of the 'Virtual Rabbit' at time t.
    Shape: Lemniscate of Bernoulli (Figure 8)
    """
    # Normalize time to 0..2PI cycle
    t = (t_sec % CYCLE_DURATION) / CYCLE_DURATION * (2 * math.pi)
    
    # Parametric Equations
    # Note: We swap X/Y here to align with North/East nicely
    denom = 1 + math.sin(t)**2
    x = (SCALE * math.cos(t)) / denom
    y = (SCALE * math.cos(t) * math.sin(t)) / denom
    
    return np.array([x, y, TARGET_ALTITUDE])

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    world = get_world_name()
    if not world: return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()
    
    boids = {name: Boid(name) for name in TARGET_DRONES}

    print(f"Figure 8 Commander Active on world: {world}")
    while not tracker.is_active():
        time.sleep(1)
    print("Swarm Acquired. Chasing the pattern.")

    start_time = time.time()

    try:
        while True:
            # Time tick
            current_t = time.time() - start_time
            
            # 1. Update Virtual Leader Position
            leader_pos = get_virtual_leader_pos(current_t)

            # 2. Update Drone States
            raw_positions = tracker.get_positions()
            packet = {}
            valid_boids = []
            
            for name in TARGET_DRONES:
                if name in raw_positions:
                    p = raw_positions[name]
                    boids[name].pos = np.array([p['x'], p['y'], p['z']]) 
                    valid_boids.append(boids[name])

            # 3. Calculate Forces
            for me in valid_boids:
                v_sep = np.array([0.0, 0.0, 0.0])
                v_coh = np.array([0.0, 0.0, 0.0])
                
                neighbors = 0
                avg_pos = np.array([0.0, 0.0, 0.0])

                for other in valid_boids:
                    if other == me: continue
                    dist = np.linalg.norm(me.pos - other.pos)

                    if dist < PERCEPTION_RADIUS:
                        neighbors += 1
                        if dist < PROTECTED_RANGE:
                            diff = me.pos - other.pos
                            weight = 1.0 / (dist * dist + 0.01)
                            v_sep += diff * weight
                        avg_pos += other.pos

                if neighbors > 0:
                    avg_pos /= neighbors
                    v_coh = (avg_pos - me.pos)
                    if np.linalg.norm(v_coh) > 0:
                        v_coh = v_coh / np.linalg.norm(v_coh)

                # --- MIGRATION (Chase the Leader) ---
                # Vector from Drone -> Virtual Leader
                v_mig = leader_pos - me.pos
                # Normalize (Direction only)
                if np.linalg.norm(v_mig) > 0:
                    v_mig = v_mig / np.linalg.norm(v_mig)

                # 4. Sum Forces
                total_force = (v_sep * W_SEPARATION) + \
                              (v_coh * W_COHESION) + \
                              (v_mig * W_MIGRATION)
                
                cmd_vel = total_force
                speed = np.linalg.norm(cmd_vel)
                if speed > MAX_SPEED:
                    cmd_vel = (cmd_vel / speed) * MAX_SPEED

                # 5. Coordinate Conversion (Gazebo -> NED)
                vel_east  = cmd_vel[0]
                vel_north = cmd_vel[1]
                vel_up    = cmd_vel[2]
                vel_down  = -vel_up 

                yaw = math.degrees(math.atan2(vel_east, vel_north))
                idx = TARGET_DRONES.index(me.name)
                packet[str(idx)] = [vel_north, vel_east, vel_down, yaw]

            if packet:
                sock.sendto(json.dumps(packet).encode(), (MANAGER_IP, MANAGER_PORT))
            
            # Print Status
            print("\033[H\033[J", end="") 
            print("FIGURE 8 PATTERN ACTIVE")
            print(f"Leader Pos: [{leader_pos[0]:.1f}, {leader_pos[1]:.1f}, {leader_pos[2]:.1f}]")
            print(f"Cycle Time: {current_t % CYCLE_DURATION:.1f} / {CYCLE_DURATION}s")

            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("Stopping.")

if __name__ == "__main__":
    main()