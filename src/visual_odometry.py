import cv2
import numpy as np
import math

class VisualOdometry:
    def __init__(self, camera_matrix):
        self.camera_matrix = camera_matrix
        self.orb = cv2.ORB_create(nfeatures=500)
        # FLANN based matcher for ORB (using LSH index)
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None
        
        # Accumulate global pose (start at origin)
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))

    def process_frame(self, frame_data):
        """
        frame_data: dict containing 'frame' (image array) and 'telemetry' (dict with odometry)
        Returns: current position (x, y, z) and rotation matrix R
        """
        frame = frame_data['frame']
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Extract features
        kp, des = self.orb.detectAndCompute(gray, None)
        
        if self.prev_frame is None or des is None or self.prev_des is None:
            self.prev_frame = gray
            self.prev_kp = kp
            self.prev_des = des
            pose, R = self._get_pose()
            return pose, R, [], []

        # Match features
        try:
            matches = self.flann.knnMatch(self.prev_des, des, k=2)
        except Exception as e:
            # Fallback if FLANN fails due to too few features
            pose, R = self._get_pose()
            return pose, R, [], []

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
        
        if len(good_matches) < 20:
            # Not enough features, skip frame
            self.prev_frame = gray
            self.prev_kp = kp
            self.prev_des = des
            pose, R = self._get_pose()
            return pose, R, good_matches, kp

        # Get matched points
        pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches])

        # Calculate Essential Matrix
        E, mask = cv2.findEssentialMat(pts2, pts1, self.camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        
        if E is None or E.shape != (3, 3):
             # Invalid essential matrix
             self.prev_frame = gray
             self.prev_kp = kp
             self.prev_des = des
             pose, R = self._get_pose()
             return pose, R, good_matches, kp

        # Decompose Essential Matrix to get Rotation (R) and Translation (t) direction
        _, R, t, mask = cv2.recoverPose(E, pts2, pts1, self.camera_matrix)

        # Recover Scale using Odometry
        scale = self._get_scale_from_odometry(frame_data['telemetry'])
        
        if scale > 0.001:  # Only update if there was meaningful movement
            # Update global translation and rotation
            self.cur_t = self.cur_t + scale * self.cur_R.dot(t)
            self.cur_R = R.dot(self.cur_R)

        # Update previous frame data
        self.prev_frame = gray
        self.prev_kp = kp
        self.prev_des = des

        pose, R = self._get_pose()
        return pose, R, good_matches, kp

    def _get_scale_from_odometry(self, telemetry):
        """Calculates distance moved since last frame using EKF2 Odometry."""
        if not hasattr(self, 'prev_odom_pos'):
            self.prev_odom_pos = telemetry['odometry']
            return 0.0
            
        cur_pos = telemetry['odometry']
        dx = cur_pos['x'] - self.prev_odom_pos['x']
        dy = cur_pos['y'] - self.prev_odom_pos['y']
        dz = cur_pos['z'] - self.prev_odom_pos['z']
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        self.prev_odom_pos = cur_pos
        return distance

    def _get_pose(self):
        """Return current estimated position and rotation."""
        return (self.cur_t[0][0], self.cur_t[1][0], self.cur_t[2][0]), self.cur_R
