#!/bin/bash

# Configuration
NUM_DRONES=5

echo "---------------------------------------------------"
echo "Cleaning up old processes..."
echo "---------------------------------------------------"
pkill -f sim_vehicle.py
pkill -f mavproxy.py
pkill -f mavsdk_server # Ensure no old servers are blocking ports

echo "---------------------------------------------------"
echo "Launching $NUM_DRONES ArduPilot SITL Instances..."
echo "---------------------------------------------------"

for i in $(seq 0 $(($NUM_DRONES - 1)))
do
    # Calculate SysID (1-based)
    SYS_ID=$(($i + 1))
    
    echo "Spawning Drone Instance $i (SysID $SYS_ID)..."
    
    # -I$i handles the port increments (14550, 14560...)
    gnome-terminal --tab --title="SITL Drone $i" -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I$i --sysid=$SYS_ID; exec bash"
    
    # Stagger launches slightly to reduce CPU spike
    sleep 2
done

echo "---------------------------------------------------"
echo "Simulator Ready!"
echo "Run 'python3 swarm_manager.py' in a new terminal."
echo "---------------------------------------------------"
