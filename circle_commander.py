#!/usr/bin/env python3

import time
import json
import socket
import math
import subprocess
from swarm_tracker import SwarmTracker

# --- CONFIGURATION ---
TARGET_DRONES = [
    "iris_with_gimbal", "iris_with_gimbal1", 
    "iris_with_gimbal2", "iris_with_gimbal3", 
    "iris_with_gimbal4"
]
MANAGER_IP = 'localhost'
MANAGER_PORT = 14000

# Circle Parameters
CENTER_NORTH = 0  # Center of circle (Meters North of Origin)
CENTER_EAST = 0   # Center of circle (Meters East of Origin)
RADIUS = 5.0
SPEED = 2.0
ALTITUDE = 10.0
GAIN_P = 0.5      # Lowered gain for smoother convergence
GAIN_Z = 1.0

def get_world_name():
    try:
        res = subprocess.check_output(["gz", "topic", "-l"], text=True)
        for t in res.split('\n'):
            if "dynamic_pose/info" in t:
                return t.split('/')[2]
    except: return None

def calculate_velocity(gz_x, gz_y, gz_z):
    """
    Converts Gazebo (ENU) position to MAVSDK (NED) Velocity.
    """
    # --- 1. COORDINATE CONVERSION (CRITICAL FIX) ---
    # Gazebo X = East, Gazebo Y = North
    # MAVSDK wants: X = North, Y = East
    current_north = gz_y
    current_east  = gz_x
    current_alt   = gz_z # Gazebo Z is Up (Altitude)

    # --- 2. Calculate Geometry in NED Frame ---
    rel_n = current_north - CENTER_NORTH
    rel_e = current_east - CENTER_EAST
    dist = math.sqrt(rel_n**2 + rel_e**2)
    if dist < 0.1: dist = 0.1

    # Tangential Vector (Orbit Counter-Clockwise)
    # Tangent of (n, e) is (-e, n)
    tan_n = -rel_e / dist * SPEED
    tan_e =  rel_n / dist * SPEED

    # Radial Vector (Pull to radius)
    err_r = RADIUS - dist
    rad_n = (rel_n / dist) * err_r * GAIN_P
    rad_e = (rel_e / dist) * err_r * GAIN_P

    # Combine
    vx_north = tan_n + rad_n
    vy_east  = tan_e + rad_e

    # --- 3. Vertical Logic ---
    # Target(10) - Current(2) = Error(8).
    # We want to go UP. In NED, UP is Negative Z.
    # So we want negative velocity.
    err_z = ALTITUDE - current_alt
    vz_down = -1 * (err_z * GAIN_Z)

    # Safety Clamps
    vz_down = max(-3.0, min(2.0, vz_down)) # Max Up 3m/s, Max Down 2m/s

    # --- 4. Yaw Calculation ---
    # Face the direction of travel (North/East)
    yaw = math.degrees(math.atan2(vy_east, vx_north))

    return [vx_north, vy_east, vz_down, yaw]

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    world = get_world_name()
    if not world: 
        print("Error: Gazebo topic not found.")
        return

    tracker = SwarmTracker(target_drones=TARGET_DRONES, world_name=world)
    tracker.start()
    
    print(f"Commander Linked to World: {world}")
    print("Waiting for drone data...")

    while not tracker.is_active():
        time.sleep(1)

    print("Swarm Acquired. Starting Logic.")

    try:
        while True:
            positions = tracker.get_positions()
            packet = {}

            # Print Debug Table
            print("\033[H\033[J", end="") 
            print("-" * 90)
            print(f"{'DRONE':<20} | {'GAZ X(E)':<8} | {'GAZ Y(N)':<8} | {'CMD N':<8} | {'CMD E':<8} | {'CMD D':<8}")
            print("-" * 90)

            for i, name in enumerate(TARGET_DRONES):
                if name in positions:
                    pos = positions[name]
                    
                    # Calculate
                    vel = calculate_velocity(pos['x'], pos['y'], pos['z'])
                    packet[str(i)] = vel
                    
                    # Print (Vel[0]=North, Vel[1]=East)
                    print(f"{name:<20} | {pos['x']:<8.1f} | {pos['y']:<8.1f} | {vel[0]:<8.2f} | {vel[1]:<8.2f} | {vel[2]:<8.2f}")

            if packet:
                sock.sendto(json.dumps(packet).encode(), (MANAGER_IP, MANAGER_PORT))

            time.sleep(0.05) # 20Hz

    except KeyboardInterrupt:
        print("\nStopping.")

if __name__ == "__main__":
    main()