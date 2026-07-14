import asyncio
import math
import time
import numpy as np

class Explorer:
    def __init__(self, drone, map_builder, state):
        self.drone = drone
        self.map_builder = map_builder
        self.state = state
        self.active = False
        self.task = None
        
        self.area_size = 20.0 # 20x20m
        self.step_size = 2.0  # distance between sweep lanes
        
    async def start(self):
        if self.active: return
        self.active = True
        
        # Disable follow mode if active
        self.state["follow_active"] = False
        self.state["manual_active"] = False
        
        # Ensure we are in OFFBOARD flight mode
        if "OFFBOARD" not in self.drone.telemetry.get("flight_mode", "").upper():
            success = await self.drone.start_offboard()
            if not success:
                print("Explorer: Failed to start offboard mode")
                self.active = False
                return
                
        self.task = asyncio.create_task(self._explore_loop())
        print("Explorer started: Sweeping 20x20m area.")

    def stop(self):
        self.active = False
        if self.task:
            self.task.cancel()
            self.task = None
        print("Explorer stopped.")

    def _is_obstacle_ahead(self, pose, yaw):
        """Check the map grid to see if there is an obstacle 2 meters ahead."""
        drone_x, drone_y, _ = pose
        
        # Point 2 meters ahead
        check_dist = 2.0
        ahead_x = drone_x + check_dist * math.cos(math.radians(yaw))
        ahead_y = drone_y + check_dist * math.sin(math.radians(yaw))
        
        with self.map_builder.lock:
            gx, gy = self.map_builder._world_to_grid(ahead_x, ahead_y)
            if 0 <= gx < self.map_builder.grid_size and 0 <= gy < self.map_builder.grid_size:
                log_odds = self.map_builder.grid[gy, gx]
                prob = 1.0 - (1.0 / (1.0 + math.exp(log_odds)))
                if prob > 0.6: # Obstacle
                    return True
        return False
        
    def _body_to_local(self, dx_body, dy_body):
        curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
        curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
        curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
        yaw_rad = math.radians(curr_yaw)
        dx_local = dx_body * math.cos(yaw_rad) - dy_body * math.sin(yaw_rad)
        dy_local = dx_body * math.sin(yaw_rad) + dy_body * math.cos(yaw_rad)
        return curr_x + dx_local, curr_y + dy_local

    async def _goto_position(self, target_x, target_y, target_alt, target_yaw=None, tolerance=0.3, yaw_tolerance=5.0, timeout=None):
        if timeout is None:
            # Estimate timeout: 4s per meter, plus 8s base
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            dist = math.hypot(target_x - curr_x, target_y - curr_y)
            timeout = max(8.0, dist * 4.0)
            
        print(f"Explorer: Heading to X={target_x:.2f}, Y={target_y:.2f}, Alt={target_alt:.2f}, Yaw={target_yaw} (timeout: {timeout:.1f}s)")
        start_time = time.time()
        
        while self.active and self.drone.connected:
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_alt = self.drone.telemetry.get("altitude", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            
            err_x = target_x - curr_x
            err_y = target_y - curr_y
            err_alt = target_alt - curr_alt
            
            dist = math.hypot(err_x, err_y)
            
            err_yaw = 0.0
            if target_yaw is not None:
                err_yaw = (target_yaw - curr_yaw + 180) % 360 - 180
                
            elapsed = time.time() - start_time
            if dist < tolerance and abs(err_alt) < 0.25 and (target_yaw is None or abs(err_yaw) < yaw_tolerance):
                print(f"Explorer: Arrived at target: dist={dist:.2f}m, alt_err={err_alt:.2f}m")
                break
            elif elapsed > timeout:
                print(f"Explorer: Timeout ({timeout:.1f}s) reached. Proceeding. dist={dist:.2f}m")
                break
                
            kp_pos = 0.8
            kp_alt = 1.0
            kp_yaw = 0.8
            
            vx_local = np.clip(err_x * kp_pos, -1.5, 1.5)
            vy_local = np.clip(err_y * kp_pos, -1.5, 1.5)
            
            curr_yaw_rad = math.radians(curr_yaw)
            vx_body = vx_local * math.cos(curr_yaw_rad) + vy_local * math.sin(curr_yaw_rad)
            vy_body = -vx_local * math.sin(curr_yaw_rad) + vy_local * math.cos(curr_yaw_rad)
            
            vz_body = np.clip(-err_alt * kp_alt, -1.0, 1.0)
            
            vyaw_body = 0.0
            if target_yaw is not None:
                vyaw_body = np.clip(err_yaw * kp_yaw, -30.0, 30.0)
                
            await self.drone.send_velocity_command(vx_body, vy_body, vz_body, vyaw_body)
            await asyncio.sleep(0.05)
            
        await self.drone.send_velocity_command(0, 0, 0, 0)
        await asyncio.sleep(0.5)

    async def _explore_loop(self):
        try:
            # 1. RTH check: check first if drone is already present (not armed or at home/grounded)
            is_armed = self.drone.telemetry.get("armed", False)
            alt = self.drone.telemetry.get("altitude", 0.0)
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            
            if is_armed and (math.hypot(curr_x, curr_y) > 1.5 or alt > 0.3):
                print("Explorer: Drone is active/away from home. Initiating RTH first...")
                await self.drone.rtl()
                while self.active and self.drone.telemetry.get("armed", False):
                    print("Explorer: Waiting for drone to land and disarm...")
                    await asyncio.sleep(1.0)
                print("Explorer: Drone landed and disarmed. Proceeding.")
                await asyncio.sleep(2.0)
            
            # 2. Arm and Takeoff to 5m
            if not self.drone.telemetry.get("armed", False):
                print("Explorer: Arming drone...")
                await self.drone.arm()
                await asyncio.sleep(1.0)
                
            print("Explorer: Taking off to 5.0m...")
            await self.drone.takeoff(5.0)
            # Wait up to 15 seconds to reach takeoff altitude or stabilize
            for _ in range(30):
                if not self.active:
                    break
                alt = self.drone.telemetry.get("altitude", 0.0)
                if abs(alt - 5.0) < 0.6:
                    print(f"Explorer: Reached takeoff altitude: {alt:.2f}m")
                    break
                await asyncio.sleep(0.5)
                
            # Transition to OFFBOARD mode (retry up to 5 times)
            print("Explorer: Starting offboard mode...")
            success = False
            for i in range(5):
                if not self.active:
                    break
                success = await self.drone.start_offboard()
                if success:
                    break
                print(f"Explorer: Offboard start failed, retrying {i+1}/5...")
                await asyncio.sleep(1.0)
                
            if not success:
                print("Explorer: Failed to start offboard mode after retries")
                self.active = False
                return
                
            # 3. Go back 8 m (maintain 5.0m alt, keep current yaw)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            tx, ty = self._body_to_local(-8.0, 0.0)
            await self._goto_position(tx, ty, 5.0, curr_yaw)
            
            # Reset map builder to clean it for a fresh start at the new search origin
            print("Explorer: Reached search origin (8m back). Resetting map for a fresh start.")
            self.map_builder.reset()
            
            # 4. Fly at distance 0.5m from ground (descend to 0.5m altitude)
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(curr_x, curr_y, 0.5, curr_yaw)
            
            # 5. Go left 8m (maintain 0.5m alt, keep current yaw)
            tx, ty = self._body_to_local(0.0, -8.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(tx, ty, 0.5, curr_yaw)
            
            # 6. Rotate 90 deg clockwise
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            target_yaw = (curr_yaw + 90.0 + 180) % 360 - 180
            await self._goto_position(curr_x, curr_y, 0.5, target_yaw)
            
            # 7. Move left 16m
            tx, ty = self._body_to_local(0.0, -16.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(tx, ty, 0.5, curr_yaw)
            
            # 8. Rotate 90 deg clockwise
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            target_yaw = (curr_yaw + 90.0 + 180) % 360 - 180
            await self._goto_position(curr_x, curr_y, 0.5, target_yaw)
            
            # 9. Move left 16m
            tx, ty = self._body_to_local(0.0, -16.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(tx, ty, 0.5, curr_yaw)
            
            # 10. Rotate 90 deg clockwise
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            target_yaw = (curr_yaw + 90.0 + 180) % 360 - 180
            await self._goto_position(curr_x, curr_y, 0.5, target_yaw)
            
            # 11. Move left 16m
            tx, ty = self._body_to_local(0.0, -16.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(tx, ty, 0.5, curr_yaw)
            
            # 12. Rotate 90 deg clockwise
            curr_x = self.drone.telemetry.get("odometry", {}).get("x", 0.0)
            curr_y = self.drone.telemetry.get("odometry", {}).get("y", 0.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            target_yaw = (curr_yaw + 90.0 + 180) % 360 - 180
            await self._goto_position(curr_x, curr_y, 0.5, target_yaw)
            
            # 13. Move left 8m
            tx, ty = self._body_to_local(0.0, -8.0)
            curr_yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            await self._goto_position(tx, ty, 0.5, curr_yaw)
            
            # 14. Save this map
            pose_dict = self.state.get("slam_pose", {"x": 0.0, "y": 0.0, "z": 0.0})
            pose = (pose_dict["x"], pose_dict["y"], pose_dict["z"])
            yaw = self.drone.telemetry.get("attitude", {}).get("yaw", 0.0)
            import time
            map_filename = f"logs/map_{int(time.time())}_final.png"
            self.map_builder.save_map(map_filename, pose, yaw)
            print(f"Explorer: Final map saved to {map_filename}")
            
            # 15. RTH like this
            print("Explorer: Returning to launch (RTH)...")
            await self.drone.rtl()
            while self.active and self.drone.telemetry.get("armed", False):
                print("Explorer: Waiting for drone to land and disarm...")
                await asyncio.sleep(1.0)
            print("Explorer: Returned to Home and landed successfully.")
            
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Explorer loop error: {e}")
        finally:
            self.active = False
            self.state["explore_active"] = False
            if self.drone.connected:
                asyncio.create_task(self.drone.send_velocity_command(0, 0, 0, 0))
