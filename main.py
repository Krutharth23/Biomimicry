#!/usr/bin/env python3

import asyncio
import json
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

# --- CONFIGURATION ---
NUM_DRONES = 5
START_UDP_PORT = 14550
START_GRPC_PORT = 50051
COMMAND_PORT = 14000

# Store the latest velocity command for each drone
# Format: {index: [vx, vy, vz, yaw]}
# Default to "Hover" (0,0,0,0)
latest_commands = {i: [0.0, 0.0, 0.0, 0.0] for i in range(NUM_DRONES)}

async def connect_drone(index):
    udp_port = START_UDP_PORT + (index * 10)
    grpc_port = START_GRPC_PORT + index
    
    print(f"[Drone {index}] Connecting (UDP:{udp_port})...")
    # spawn unique mavsdk_server for each
    drone = System(port=grpc_port)
    await drone.connect(system_address=f"udp://:{udp_port}")

    print(f"[Drone {index}] Waiting for Link...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print(f"[Drone {index}] Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    
    # Arm and Takeoff
    try:
        print(f"[Drone {index}] Arming...")
        await drone.action.arm()
        print(f"[Drone {index}] Taking Off...")
        await drone.action.takeoff()
        # Wait for initial climb to finish before starting offboard
        await asyncio.sleep(10) 

        # Send 0 setpoint before starting offboard
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()
        print(f"[Drone {index}] Offboard Active.")
    except Exception as e:
        print(f"[Drone {index}] Setup Error: {e}")

    return drone

async def main():
    # 1. Connect
    connect_tasks = [connect_drone(i) for i in range(NUM_DRONES)]
    drones = await asyncio.gather(*connect_tasks)

    # 2. Setup Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('localhost', COMMAND_PORT))
    sock.setblocking(False)
    
    print("-" * 50)
    print(f"MANAGER READY. Holding {NUM_DRONES} drones in Offboard.")
    print("-" * 50)

    # 3. Control Loop (20Hz)
    while True:
        # A. UPDATE COMMANDS (Non-blocking receive)
        try:
            while True: # Drain the socket buffer to get the freshest packet
                data, addr = sock.recvfrom(4096)
                command_dict = json.loads(data.decode())
                
                # Update our global memory of what each drone SHOULD be doing
                for drone_idx_str, vel_data in command_dict.items():
                    idx = int(drone_idx_str)
                    if idx < NUM_DRONES:
                        latest_commands[idx] = vel_data
        except BlockingIOError:
            pass # No new data, that's fine, keep using old data
        except Exception as e:
            print(f"Packet Error: {e}")

        # B. SEND COMMANDS TO DRONES
        # We MUST send this every loop iteration to keep the failsafe from triggering
        tasks = []
        for i, drone in enumerate(drones):
            vx, vy, vz, yaw = latest_commands[i]
            tasks.append(
                drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vx, vy, vz, yaw)
                )
            )
        
        # Fire all commands at once
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

        # 20Hz update rate
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(main())