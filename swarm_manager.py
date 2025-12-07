#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

# --- CONFIGURATION ---
NUM_DRONES = 5
START_UDP_PORT = 14550  # ArduPilot SITL default start
START_GRPC_PORT = 50051 # MAVSDK internal server start

async def connect_drone(index):
    """
    Connects to a single drone and returns the System object.
    """
    # 1. Calculate Ports
    # UDP: 14550, 14560, 14570... (Standard ArduPilot increments)
    udp_port = START_UDP_PORT + (index * 10)
    # gRPC: 50051, 50052, 50053... (Must be unique for each drone instance)
    grpc_port = START_GRPC_PORT + index
    
    system_address = f"udp://:{udp_port}"
    
    print(f"[Drone {index}] Initializing (UDP: {udp_port}, gRPC: {grpc_port})...")
    
    # 2. Initialize System with unique gRPC port
    # FIX: Removed 'mavsdk_server_address="localhost"'. 
    # This ensures MAVSDK spawns a NEW server process for this drone.
    drone = System(port=grpc_port)
    
    await drone.connect(system_address=system_address)

    print(f"[Drone {index}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {index}] Connected!")
            break

    print(f"[Drone {index}] Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Drone {index}] GPS OK.")
            break
            
    return drone

async def run_mission(drone, index):
    """
    The flight logic for a single drone.
    """
    try:
        print(f"[Drone {index}] Arming...")
        await drone.action.arm()

        print(f"[Drone {index}] Taking off...")
        await drone.action.takeoff()
        
        # Wait for takeoff to finish (approx 10s)
        await asyncio.sleep(10)

        # --- OFFBOARD MANEUVERS ---
        print(f"[Drone {index}] Starting Offboard Mode...")
        
        # 1. Send an initial setpoint (Required before starting offboard)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"[Drone {index}] Offboard start failed: {e}")
            return

        
        vel_x = 2.0
        print(f"[Drone {index}] Moving NORTH")
       

        await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_x, 0.0, 0.0, 0.0))
        await asyncio.sleep(5)

        # 3. Stop
        print(f"[Drone {index}] Stopping")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(2)

        # 4. Stop Offboard
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass

        # 5. Land
        print(f"[Drone {index}] Landing...")
        await drone.action.land()

    except Exception as e:
        print(f"[Drone {index}] Error: {e}")

async def main():
    # 1. CONNECT PHASE
    print("-- Connecting to swarm...")
    connect_tasks = [connect_drone(i) for i in range(NUM_DRONES)]
    
    # Run all connections concurrently
    # This will now start 5 separate backend servers automatically
    drones = await asyncio.gather(*connect_tasks)
    
    print("-- All drones connected and ready!")
    await asyncio.sleep(2)

    # 2. MISSION PHASE
    print("-- executing swarm mission...")
    mission_tasks = [run_mission(drones[i], i) for i in range(NUM_DRONES)]
    # Run all missions concurrently
    await asyncio.gather(*mission_tasks)
    
    print("-- Mission Complete.")

if __name__ == "__main__":
    asyncio.run(main())
