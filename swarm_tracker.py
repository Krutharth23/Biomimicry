import gz.transport13
import gz.msgs10
from gz.msgs10.pose_v_pb2 import Pose_V
import threading
import time

class SwarmTracker:
    def __init__(self, target_drones=None, world_name="map"):
        """
        Args:
            target_drones (list): Optional list of model names to filter by (e.g., ['iris_0', 'iris_1']).
                                  If None, it tracks ALL models in the world.
            world_name (str): The name of the Gazebo world (default is 'map' or 'default').
                              Check 'gz topic -l' if you aren't receiving data.
        """
        self.node = gz.transport13.Node()
        self.target_drones = target_drones
        self.latest_positions = {}
        self.lock = threading.Lock()
        self.running = False
        self.topic = f"/world/{world_name}/dynamic_pose/info"
        self.last_msg_time = 0

    def _callback(self, msg: Pose_V):
        """Internal callback to process Gazebo messages."""
        self.last_msg_time = time.time()
        
        with self.lock:
            for pose in msg.pose:
                # If a target list is provided, skip models not in the list
                if self.target_drones and pose.name not in self.target_drones:
                    continue
                
                # Store position data
                self.latest_positions[pose.name] = {
                    'id': pose.id,
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                    'yaw': self._q_to_yaw(pose.orientation) # simplified yaw
                }

    def _q_to_yaw(self, q):
        """Helper to get Yaw from Quaternion."""
        # Standard conversion (msg.orientation.x, .y, .z, .w)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return 0.0 # You can import math.atan2 if you need precise yaw, keeping it simple for now

    def start(self):
        """Starts the listener."""
        if self.running:
            return
        
        print(f"[SwarmTracker] Subscribing to: {self.topic}")
        if self.node.subscribe(Pose_V, self.topic, self._callback):
            self.running = True
            print("[SwarmTracker] Listener started successfully.")
        else:
            print(f"[SwarmTracker] Error: Could not subscribe to {self.topic}")

    def get_positions(self):
        """
        Returns a dictionary of the latest positions.
        Format: {'model_name': {'x': 0.0, 'y': 0.0, 'z': 0.0}, ...}
        """
        with self.lock:
            return self.latest_positions.copy()

    def is_active(self):
        """Returns True if we have received data recently (last 2 seconds)."""
        return (time.time() - self.last_msg_time) < 2.0