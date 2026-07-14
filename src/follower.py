import asyncio
import time
import numpy as np

# Estimated physical heights of targets in meters
TARGET_HEIGHTS = {
    "red_sphere": 0.4,
    "blue_cube": 0.5,
    "green_cylinder": 0.5,
    "yellow_cylinder": 0.5,
    "person": 1.8,
    "car": 1.5
}

class FollowController:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.active = False
        self.target_distance = 3.0  # meters
        
        # Tuned Proportional (P) Constants for faster response
        self.kp_yaw = 0.35  # Increased to track targets moving laterally more aggressively
        self.kp_fwd = 0.25  # Reduced to prevent back-and-forth oscillation
        self.kp_alt = 0.8   # Increased to close the altitude gap much quicker
        self.kp_alt_pixel = 0.005 # For vertical centering based on image pixels
        self.max_fwd_speed = 1.0   # Maximum forward speed (m/s)
        
        # Exponential Moving Average (EMA) smoothing factors for velocity (0.0 to 1.0)
        # Lower value = smoother but delayed; Higher = faster, less filtered
        self.alpha_yaw = 0.40  # Increased for faster turning response
        self.alpha_fwd = 0.1   # Smooth forward/backward speed
        self.alpha_alt = 0.30  # Increased for snappier altitude adjustments
        
        # Current smoothed velocity states
        self.cmd_yaw = 0.0
        self.cmd_fwd = 0.0
        self.cmd_alt = 0.0
        
        self.frame_width = 640
        self.frame_height = 480
        
        self.last_detection_time = 0
        self.lost_timeout = 2.0  # seconds before declaring target lost
        
        # Search State Machine
        self.state = "TRACKING" # TRACKING, SEARCHING, or HOLDING
        self.search_alt = 0.5
        self.search_phase = "CLIMBING"
        self.search_timer = 0
        self.yaw_speed_search = 22.5 # deg/s (half speed)
        self.rotation_time = (360.0 * 1) / self.yaw_speed_search # time for one 360 rotation
        self.move_speed = 1.0 # m/s for square pattern
        self.move_time = 10.0 / self.move_speed # 10 meters distance
        self.square_side = -1 # -1: initial rotation, 0: Fwd, 1: Left, 2: Back, 3: Right
        self.hold_start_time = 0
        self.last_err_x = 0.0
        self.last_err_alt = 0.0
        self.log_file = None
        
    def _log_data(self, target_class):
        if self.log_file:
            try:
                with open(self.log_file, "a") as f:
                    f.write(f"{time.time()},{self.state},{target_class},{self.last_err_x:.3f},{self.last_err_alt:.3f},{self.cmd_yaw:.3f},{self.cmd_alt:.3f},{self.kp_yaw:.3f},{self.kp_alt:.3f},{self.alpha_yaw:.3f},{self.alpha_alt:.3f}\n")
            except Exception:
                pass

    def start(self):
        self.active = True
        self.state = "TRACKING"
        self.last_detection_time = time.time()
        # Initialize logging
        try:
            import os
            os.makedirs("logs", exist_ok=True)
            self.log_file = f"logs/pid_tuning_{int(time.time())}.csv"
            with open(self.log_file, "w") as f:
                f.write("timestamp,state,target_class,err_x,err_alt,cmd_yaw,cmd_alt,kp_yaw,kp_alt,alpha_yaw,alpha_alt\n")
            print(f"Started logging PID data to {self.log_file}")
        except Exception as e:
            print(f"Failed to initialize PID log file: {e}")
            self.log_file = None
        
    def stop(self):
        self.active = False
        self.log_file = None
        
    async def update(self, detection_info, target_class="person"):
        if not self.active or not self.drone.connected:
            return
            
        current_time = time.time()
        current_alt = self.drone.telemetry["altitude"]
        
        # Calculate dynamic target altitude based on object height (1.2x height)
        # Ensure it is at least 0.5m for safety.
        target_alt = max(2.0, 1.0 * TARGET_HEIGHTS.get(target_class, 1.0))
        
        if detection_info:
            if self.state == "SEARCHING":
                # Reset search commands to avoid carrying search velocities into tracking
                self.cmd_yaw = 0.0
                self.cmd_fwd = 0.0
                self.cmd_alt = 0.0
                
            self.state = "TRACKING"
            self.last_detection_time = current_time
            
            # Extract detection data
            cx, cy = detection_info["center"]
            dist = detection_info["distance"]
            
            # 1. Yaw error (X-axis pixel)
            # Center of the 640px frame is 320. 
            # We use a tight 40-pixel deadzone (300 to 340) to keep the object perfectly centered.
            # We calculate error relative to these bounds to avoid sudden ramping when crossing the threshold.
            left_bound = 290
            right_bound = 320
            
            if cx < left_bound:
                # Object is too far left (< 240). `err_x_track` will be negative.
                # Command will be positive (rotate right / clockwise).
                err_x_track = (cx - left_bound)
            elif cx > right_bound:
                # Object is too far right (> 440). `err_x_track` will be positive.
                # Command will be negative (rotate left / counter-clockwise).
                err_x_track = (cx - right_bound)
            else:
                # Object is perfectly in the 240-440 zone
                err_x_track = 0.0
                
            self.last_err_x = err_x_track
            raw_yaw_cmd = err_x_track * self.kp_yaw
            
            # 2. Alt Control: Keep dynamic altitude based on target class (1.2 * height)
            # No longer using Y-pixel centering (err_y) to adjust altitude
            err_alt = target_alt - current_alt
            self.last_err_alt = err_alt
            raw_down_cmd = -err_alt * self.kp_alt
            
            # 3. Distance error (forward/back)
            # Softer alignment criteria to prevent forward/backward command from dropping to 0 on minor deviations
            yaw_aligned = (abs(err_x_track) < 40.0)
            alt_aligned = (abs(err_alt) < 0.5)
            
            if yaw_aligned and alt_aligned:
                err_dist = dist - self.target_distance
                raw_fwd_cmd = err_dist * self.kp_fwd
            else:
                raw_fwd_cmd = 0.0
                
            # 4. Apply Exponential Moving Average (EMA) Filter for buttery smooth movement
            self.cmd_yaw = (self.alpha_yaw * raw_yaw_cmd) + ((1 - self.alpha_yaw) * self.cmd_yaw)
            self.cmd_fwd = (self.alpha_fwd * raw_fwd_cmd) + ((1 - self.alpha_fwd) * self.cmd_fwd)
            self.cmd_alt = (self.alpha_alt * raw_down_cmd) + ((1 - self.alpha_alt) * self.cmd_alt)
            
            # 5. Clamp commands for strict safety and smoothness (using adjustable max_fwd_speed)
            yaw_out = np.clip(self.cmd_yaw, -30.0, 30.0)      # Reduced max rotation speed
            fwd_out = np.clip(self.cmd_fwd, -self.max_fwd_speed * 0.5, self.max_fwd_speed)
            down_out = np.clip(self.cmd_alt, -0.5, 0.5)       # Reduced max vertical speed
            
            # Floor safety constraint: Do not fly below 0.5m altitude during tracking
            # In NED, positive down_out means descending
            if current_alt < 0.5 and down_out > 0.0:
                down_out = 0.0
            
            # Send command
            await self.drone.send_velocity_command(
                forward_m_s=fwd_out,
                right_m_s=0.0,
                down_m_s=down_out,
                yawspeed_deg_s=yaw_out
            )
            self._log_data(target_class)
            
        else:
            # Target lost
            # Gradually slow down to a hover rather than jerking to a stop
            self.cmd_yaw = self.cmd_yaw * 0.8
            self.cmd_fwd = self.cmd_fwd * 0.8
            self.cmd_alt = self.cmd_alt * 0.8
            
            if current_time - self.last_detection_time > self.lost_timeout:
                # Reset smoothed commands
                self.cmd_yaw = 0.0
                self.cmd_fwd = 0.0
                self.cmd_alt = 0.0
                if self.state == "TRACKING":
                    self.state = "SEARCHING"
                    self.search_alt = target_alt
                    self.search_phase = "CLIMBING"
                    self.square_side = -1
                
                if self.state == "SEARCHING":
                    err_alt = self.search_alt - current_alt
                    self.last_err_alt = err_alt
                    self.last_err_x = 0.0
                    down_cmd = -err_alt * self.kp_alt
                    down_cmd = np.clip(down_cmd, -1.0, 1.0)
                    
                    if self.search_phase == "CLIMBING":
                        if abs(err_alt) < 0.3: # Reached target altitude
                            self.search_phase = "ROTATING"
                            self.search_timer = current_time
                        else:
                            # Climb to search altitude
                            await self.drone.send_velocity_command(0.0, 0.0, down_cmd, 0.0)
                            
                    elif self.search_phase == "ROTATING":
                        if current_time - self.search_timer > self.rotation_time:
                            # Finished rotations, move to next side of the square
                            self.search_phase = "MOVING"
                            self.search_timer = current_time
                            self.square_side = (self.square_side + 1) % 4
                            await self.drone.send_velocity_command(0.0, 0.0, down_cmd, 0.0)
                        else:
                            # Rotate while maintaining altitude
                            await self.drone.send_velocity_command(0.0, 0.0, down_cmd, self.yaw_speed_search)
                            
                    elif self.search_phase == "MOVING":
                        if current_time - self.search_timer > self.move_time:
                            # Finished moving, rotate again
                            self.search_phase = "ROTATING"
                            self.search_timer = current_time
                            await self.drone.send_velocity_command(0.0, 0.0, down_cmd, 0.0)
                        else:
                            # Move along the square pattern
                            fwd, right = 0.0, 0.0
                            if self.square_side == 0:
                                fwd = self.move_speed   # Forward 10m
                            elif self.square_side == 1:
                                right = -self.move_speed # Left 10m
                            elif self.square_side == 2:
                                fwd = -self.move_speed  # Back 10m
                            elif self.square_side == 3:
                                right = self.move_speed  # Right 10m
                                
                            await self.drone.send_velocity_command(fwd, right, down_cmd, 0.0)
            self._log_data(target_class)
