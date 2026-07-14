import numpy as np
import cv2
import math
import threading

class MapBuilder:
    def __init__(self, map_size_m=40, resolution_m=0.1):
        """
        map_size_m: total size of the map in meters (40m x 40m)
        resolution_m: grid cell size (0.1m/cell -> 400x400 grid)
        """
        self.map_size = map_size_m
        self.resolution = resolution_m
        
        self.grid_size = int(self.map_size / self.resolution)
        
        # Bayesian Log-Odds grid (starts at 0 -> 50% probability)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        
        # Log-odds update values
        self.l_free = -0.4    # Log-odds for free space
        self.l_occ = 0.85     # Log-odds for occupied space
        self.l_max = 5.0      # Max cap for occupancy
        self.l_min = -5.0     # Min cap for free space

        # Map origin is at the center (index 200, 200)
        self.origin = (self.grid_size // 2, self.grid_size // 2)
        
        self.lock = threading.Lock()
        self.path_history = []

    def reset(self):
        """Reset the map grid and path history."""
        with self.lock:
            self.grid.fill(0.0)
            self.path_history.clear()

    def save_map(self, filepath, pose, yaw):
        """Save the current map visualization to a file."""
        import os
        img = self.get_map_image(pose, yaw)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        cv2.imwrite(filepath, img)

    def _world_to_grid(self, x, y):
        """Convert real-world coordinates (meters) to grid indices.
        World X is North, World Y is East.
        Grid X is East (+X is right), Grid Y is North (-Y is up).
        """
        gx = int(y / self.resolution) + self.origin[0]
        gy = int(-x / self.resolution) + self.origin[1]
        return gx, gy

    def update_map(self, pose, kp_matched, frame_shape, telemetry, detection_info=None):
        """
        Project matched keypoints to the ground and update the occupancy grid.
        pose: (x, y, z) from VO
        kp_matched: list of matched keypoints from VO
        frame_shape: (height, width) of camera image
        telemetry: dict containing altitude and attitude
        """
        drone_x, drone_y, drone_z = pose
        drone_gx, drone_gy = self._world_to_grid(drone_x, drone_y)
        
        # Save position to path history if moved significantly (>15cm) to prevent infinite growth
        with self.lock:
            if not self.path_history or math.hypot(drone_x - self.path_history[-1][0], drone_y - self.path_history[-1][1]) > 0.15:
                self.path_history.append((drone_x, drone_y))
        
        # We only project if we are reasonably high up
        alt = telemetry.get('altitude', 0.0)
        if alt < 0.2:
             return
             
        # Drone attitude (degrees)
        pitch = telemetry.get('attitude', {}).get('pitch', 0)
        roll = telemetry.get('attitude', {}).get('roll', 0)
        yaw = telemetry.get('attitude', {}).get('yaw', 0)
        
        r_rad = math.radians(roll)
        p_rad = math.radians(pitch)
        y_rad = math.radians(yaw)
        
        cos_r, sin_r = math.cos(r_rad), math.sin(r_rad)
        cos_p, sin_p = math.cos(p_rad), math.sin(p_rad)
        cos_y, sin_y = math.cos(y_rad), math.sin(y_rad)
        
        # Camera tilt is 15 degrees down on the drone body frame
        tilt_rad = math.radians(-15.0)
        cos_t, sin_t = math.cos(tilt_rad), math.sin(tilt_rad)
        
        h, w = frame_shape
        focal_length = 277.19 # from CAMERA_MATRIX

        # If we have a valid object detection, explicitly project and mark it as occupied
        if detection_info and detection_info.get("distance", -1) > 0:
            u, v = detection_info["center"]
            dist = detection_info["distance"]
            
            # Project the center ray
            nx = (u - w/2) / focal_length
            ny = (v - h/2) / focal_length
            
            bx = cos_t * 1.0 + sin_t * ny
            by = -nx
            bz = sin_t * 1.0 - cos_t * ny
            
            x1 = bx
            y1 = by * cos_r - bz * sin_r
            z1 = by * sin_r + bz * cos_r
            
            x2 = x1 * cos_p + z1 * sin_p
            y2 = y1
            z2 = -x1 * sin_p + z1 * cos_p
            
            wx = x2 * cos_y - y2 * sin_y
            wy = x2 * sin_y + y2 * cos_y
            
            # Normalize the ray vector (length is L = sqrt(1.0 + nx^2 + ny^2))
            L = math.sqrt(1.0 + nx*nx + ny*ny)
            wx_unit = wx / L
            wy_unit = wy / L
            
            # Relative coordinates in world
            dx_w = dist * wx_unit
            dy_w = dist * wy_unit
            
            end_x = drone_x + dx_w
            end_y = drone_y + dy_w
            end_gx, end_gy = self._world_to_grid(end_x, end_y)
            
            # Mark it occupied in the grid
            with self.lock:
                # Occupy a circle around the obstacle (radius 0.35m)
                radius_cells = max(1, int(0.35 / self.resolution))
                for dy in range(-radius_cells, radius_cells + 1):
                    for dx in range(-radius_cells, radius_cells + 1):
                        if dx*dx + dy*dy <= radius_cells*radius_cells:
                            gx = end_gx + dx
                            gy = end_gy + dy
                            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                                self.grid[gy, gx] += self.l_occ * 2.0  # High confidence occupancy boost
                                self.grid[gy, gx] = min(self.grid[gy, gx], self.l_max)

        # Sweep 10 rays across a narrow 10-degree horizontal wedge (-5 to +5 degrees)
        fov_deg = 4
        half_fov = fov_deg // 2
        bins = [8.0] * fov_deg

        for m in kp_matched:
            u, v = m.pt
            
            # Normal pixel coordinates (no roll correction needed here, handled by 3D rotation)
            nx = (u - w/2) / focal_length
            ny = (v - h/2) / focal_length
            
            # 1. Ray vector in unpitched camera frame: X right, Y down, Z forward
            # Unpitched camera ray in drone body coordinates: [1.0, -nx, -ny]
            # 2. Rotate ray around Y body axis by camera tilt (-15 degrees)
            bx = cos_t * 1.0 + sin_t * ny
            by = -nx
            bz = sin_t * 1.0 - cos_t * ny
            
            # 3. Rotate ray by drone roll (around X axis)
            x1 = bx
            y1 = by * cos_r - bz * sin_r
            z1 = by * sin_r + bz * cos_r
            
            # 4. Rotate ray by drone pitch (around Y axis)
            x2 = x1 * cos_p + z1 * sin_p
            y2 = y1
            z2 = -x1 * sin_p + z1 * cos_p
            
            # 5. Rotate ray by drone yaw (around Z axis)
            wx = x2 * cos_y - y2 * sin_y
            wy = x2 * sin_y + y2 * cos_y
            wz = z2
            
            # Only consider rays pointing down to the ground
            if wz >= -0.05:
                continue
                
            # Intersect ray with ground plane (z = 0 relative to takeoff)
            s = -alt / wz
            distance = math.hypot(s * wx, s * wy)
            
            if distance > 8.0:
                continue # ignore points beyond 8m
                
            # Relative coordinates and bearing
            dx_w = s * wx
            dy_w = s * wy
            
            bearing = math.atan2(dy_w, dx_w)
            rel_bearing_rad = bearing - y_rad
            # Normalize bearing to [-pi, pi]
            rel_bearing_rad = (rel_bearing_rad + math.pi) % (2 * math.pi) - math.pi
            rel_bearing_deg = math.degrees(rel_bearing_rad)
            
            # Map to FOV bins
            bin_idx = int(rel_bearing_deg + half_fov)
            if 0 <= bin_idx < fov_deg:
                if distance < bins[bin_idx]:
                    bins[bin_idx] = distance

        # Raycast all bins to either map the obstacle or clear 8m of free space
        with self.lock:
            for i in range(fov_deg):
                rel_angle_rad = math.radians(i - half_fov)
                angle_rad = y_rad + rel_angle_rad
                r = bins[i]
                
                # Calculate ray endpoint
                end_x = drone_x + r * math.cos(angle_rad)
                end_y = drone_y + r * math.sin(angle_rad)
                end_gx, end_gy = self._world_to_grid(end_x, end_y)
                
                # Raycast free space from drone to endpoint
                self._raycast_free(drone_gx, drone_gy, end_gx, end_gy)
                
                # If there was a real obstacle closer than 8m, mark endpoint as occupied
                if r < 7.95:
                    # Occupy a 1.0m wide circle (radius 0.5m) around the obstacle
                    radius_cells = int(0.5 / self.resolution)
                    for dy in range(-radius_cells, radius_cells + 1):
                        for dx in range(-radius_cells, radius_cells + 1):
                            if dx*dx + dy*dy <= radius_cells*radius_cells:
                                gx = end_gx + dx
                                gy = end_gy + dy
                                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                                    self.grid[gy, gx] += self.l_occ
                                    self.grid[gy, gx] = min(self.grid[gy, gx], self.l_max)
                else:
                    # Explicitly mark the 8m endpoint as free space as well
                    if 0 <= end_gx < self.grid_size and 0 <= end_gy < self.grid_size:
                        self.grid[end_gy, end_gx] += self.l_free
                        self.grid[end_gy, end_gx] = max(self.grid[end_gy, end_gx], self.l_min)
                
    def _raycast_free(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to mark space as free."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    if x != x1 or y != y1: # Don't overwrite the obstacle itself
                        self.grid[y, x] += self.l_free
                        self.grid[y, x] = max(self.grid[y, x], self.l_min)
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    if x != x1 or y != y1:
                        self.grid[y, x] += self.l_free
                        self.grid[y, x] = max(self.grid[y, x], self.l_min)
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

    def get_map_image(self, pose, yaw=0.0):
        """Returns a visualized RGB image of the map with the drone pose overlay."""
        with self.lock:
            # Convert log odds to probabilities 0.0 - 1.0
            prob = 1.0 - (1.0 / (1.0 + np.exp(self.grid)))
            
            # Map probabilities to grayscale pixels (0 is black/obstacle, 255 is white/free, 127 is gray/unknown)
            img = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.uint8)
            
            # Unknown -> gray
            img[:] = (127, 127, 127)
            
            # Free -> white
            img[prob < 0.4] = (255, 255, 255)
            
            # Occupied -> black
            img[prob > 0.6] = (0, 0, 0)
            
            # Draw path history
            if len(self.path_history) > 1:
                pts = []
                for pt_x, pt_y in self.path_history:
                    pgx, pgy = self._world_to_grid(pt_x, pt_y)
                    pts.append([pgx, pgy])
                pts = np.array(pts, dtype=np.int32)
                # Cyan line for path (BGR: (255, 255, 0))
                cv2.polylines(img, [pts], isClosed=False, color=(255, 255, 0), thickness=2)

            # Draw drone
            drone_x, drone_y, _ = pose
            dx, dy = self._world_to_grid(drone_x, drone_y)
            if 0 <= dx < self.grid_size and 0 <= dy < self.grid_size:
                # Draw modern simple chevron (stealth pointer) representing drone direction
                yaw_rad = math.radians(yaw)
                sin_y = math.sin(yaw_rad)
                cos_y = math.cos(yaw_rad)
                
                # Tip of the chevron (pointing forward)
                tip_x = int(dx + 12 * sin_y)
                tip_y = int(dy - 12 * cos_y)
                
                # Rear left corner (yaw - 135 degrees)
                yaw_left = yaw_rad - math.radians(135.0)
                left_x = int(dx + 8 * math.sin(yaw_left))
                left_y = int(dy - 8 * math.cos(yaw_left))
                
                # Rear right corner (yaw + 135 degrees)
                yaw_right = yaw_rad + math.radians(135.0)
                right_x = int(dx + 8 * math.sin(yaw_right))
                right_y = int(dy - 8 * math.cos(yaw_right))
                
                # Assemble polygon vertices (chevron shape: Tip -> Rear Left -> Center -> Rear Right)
                chevron_pts = np.array([[tip_x, tip_y], [left_x, left_y], [dx, dy], [right_x, right_y]], dtype=np.int32)
                
                # Draw filled modern chevron (electric blue) with a crisp white border
                cv2.fillPoly(img, [chevron_pts], (255, 100, 0))
                cv2.polylines(img, [chevron_pts], isClosed=True, color=(255, 255, 255), thickness=1)
                
            return img
