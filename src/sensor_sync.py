import time
import threading
from collections import deque

class SensorSync:
    """
    Synchronizes camera frames with telemetry (IMU, Attitude, Odometry) data.
    Keeps a sliding window of recent telemetry and pairs the closest one temporally
    when a new camera frame is received.
    """
    def __init__(self, max_buffer_size=100, max_time_diff=0.05):
        self.max_buffer_size = max_buffer_size
        self.max_time_diff = max_time_diff  # 50ms tolerance
        
        self.telemetry_buffer = deque(maxlen=max_buffer_size)
        self.synced_data = deque(maxlen=max_buffer_size)
        self.lock = threading.Lock()

    def add_telemetry(self, telemetry_dict):
        """Add latest telemetry reading with current timestamp."""
        with self.lock:
            # Create a snapshot of the telemetry dict
            telemetry_snapshot = {
                "timestamp": time.time(),
                "imu": dict(telemetry_dict.get("imu", {})),
                "attitude": dict(telemetry_dict.get("attitude", {})),
                "odometry": dict(telemetry_dict.get("odometry", {})),
                "altitude": telemetry_dict.get("altitude", 0.0),
                "speed": telemetry_dict.get("speed", 0.0),
                "flight_mode": telemetry_dict.get("flight_mode", "UNKNOWN"),
                "armed": telemetry_dict.get("armed", False)
            }
            self.telemetry_buffer.append(telemetry_snapshot)

    def add_frame(self, frame):
        """Add a camera frame and sync it with the closest telemetry reading."""
        frame_time = time.time()
        closest_telem = None
        min_diff = float('inf')

        with self.lock:
            # Find the telemetry reading closest in time to this frame
            for telem in self.telemetry_buffer:
                diff = abs(frame_time - telem["timestamp"])
                if diff < min_diff:
                    min_diff = diff
                    closest_telem = telem
            
            if closest_telem and min_diff <= self.max_time_diff:
                synced_pair = {
                    "frame": frame,
                    "frame_time": frame_time,
                    "telemetry": closest_telem,
                    "time_diff": min_diff
                }
                self.synced_data.append(synced_pair)
                return synced_pair
            
        return None

    def get_latest_synced(self):
        """Retrieve the most recent synchronized frame-telemetry pair."""
        with self.lock:
            if self.synced_data:
                return self.synced_data[-1]
            return None
