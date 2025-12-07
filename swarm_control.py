#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import sys
import os
import time

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist

class SwarmDrone(Node):
    def __init__(self, index, total_drones):
        self.index = index
        self.total_drones = total_drones
        self.ns = f"uav{index}"
        super().__init__(f'swarm_control_{index}', namespace=self.ns)

        # QoS Profile for MAVROS (Best Effort is often safer for telemetry)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, 
            'mavros/state', 
            self.state_cb, 
            qos_profile
        )

        # Publishers
        self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

    def state_cb(self, msg):
        self.current_state = msg

    def wait_for_services(self):
        self.get_logger().info(f"Waiting for MAVROS services...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Arming service not available, waiting...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("SetMode service not available, waiting...")

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm(self, should_arm):
        req = CommandBool.Request()
        req.value = should_arm
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_vel(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        self.vel_pub.publish(msg)

# --- Helper function for file synchronization ---
def wait_for_swarm_sync(node):
    flag_dir = "/tmp"
    my_flag = os.path.join(flag_dir, f"drone_ready_{node.index}")
    
    # Create my flag
    with open(my_flag, 'w') as f:
        f.write("ready")
    
    node.get_logger().info("Takeoff complete. Waiting for swarm...")

    all_ready = False
    while not all_ready and rclpy.ok():
        ready_count = 0
        for i in range(node.total_drones):
            if os.path.exists(os.path.join(flag_dir, f"drone_ready_{i}")):
                ready_count += 1
        
        if ready_count == node.total_drones:
            all_ready = True
            node.get_logger().info("All drones ready! Proceeding...")
        else:
            time.sleep(1)
            # Must spin to keep heartbeats alive
            rclpy.spin_once(node, timeout_sec=0.1)

def main():
    # Args parsing
    idx = 0
    count = 1
    if len(sys.argv) > 1:
        idx = int(sys.argv[1])
    if len(sys.argv) > 2:
        count = int(sys.argv[2])

    rclpy.init()
    
    drone = SwarmDrone(idx, count)
    drone.wait_for_services()

    # 1. Wait for Connection
    drone.get_logger().info("Waiting for FCU connection...")
    while not drone.current_state.connected and rclpy.ok():
        rclpy.spin_once(drone, timeout_sec=0.1)
    drone.get_logger().info("FCU Connected!")

    # 2. Warmup setpoints
    for _ in range(20):
        drone.publish_vel(0,0,0)
        rclpy.spin_once(drone, timeout_sec=0.1)
        time.sleep(0.05)

    # 3. Set Mode GUIDED
    drone.get_logger().info("Requesting GUIDED mode...")
    while drone.current_state.mode != "GUIDED" and rclpy.ok():
        drone.set_mode("GUIDED")
        time.sleep(1.0)
        rclpy.spin_once(drone, timeout_sec=0.1)

    # 4. Arm
    drone.get_logger().info("Requesting ARM...")
    while not drone.current_state.armed and rclpy.ok():
        drone.arm(True)
        time.sleep(1.0)
        rclpy.spin_once(drone, timeout_sec=0.1)

    # 5. Takeoff (Vel Z)
    drone.get_logger().info("Taking off (Vel Z=2.0)...")
    start_time = time.time()
    while (time.time() - start_time) < 5.0 and rclpy.ok():
        drone.publish_vel(0, 0, 2.0)
        rclpy.spin_once(drone, timeout_sec=0.1)
        time.sleep(0.05)
    
    # Hover
    drone.publish_vel(0,0,0)
    
    # 6. SYNC
    wait_for_swarm_sync(drone)

    # 7. Maneuver: North
    drone.get_logger().info("Moving North...")
    start_time = time.time()
    while (time.time() - start_time) < 4.0 and rclpy.ok():
        drone.publish_vel(2.0, 0, 0) # X is forward in ENU/Body frame usually
        rclpy.spin_once(drone, timeout_sec=0.1)
        time.sleep(0.05)

    # 8. Maneuver: South
    drone.get_logger().info("Moving South...")
    start_time = time.time()
    while (time.time() - start_time) < 4.0 and rclpy.ok():
        drone.publish_vel(-2.0, 0, 0)
        rclpy.spin_once(drone, timeout_sec=0.1)
        time.sleep(0.05)

    # Land
    drone.get_logger().info("Landing...")
    drone.set_mode("LAND")

    # Keep node alive until Ctrl+C
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        pass
    
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
