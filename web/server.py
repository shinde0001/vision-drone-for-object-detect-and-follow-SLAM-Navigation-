import cv2
import asyncio
import json
import os
import sys
from pathlib import Path
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
import time
import random
import math

# Add src to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.camera_stream import CameraStream
from src.detector import Detector
from src.drone_controller import DroneController
from src.follower import FollowController
from src.sensor_sync import SensorSync
from src.visual_odometry import VisualOdometry
from src.map_builder import MapBuilder
from src.explorer import Explorer
import numpy as np

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Static files
web_dir = os.path.dirname(os.path.abspath(__file__))

# Global state
camera = CameraStream(port=5600)
detector = Detector()
drone = DroneController(system_address="udp://:14540")
follower = FollowController(drone)
sensor_sync = SensorSync()

# Camera intrinsics from SDF
CAMERA_MATRIX = np.array([
    [277.19, 0,      160.5],
    [0,      277.19, 120.5],
    [0,      0,      1.0  ]
])
vo = VisualOdometry(CAMERA_MATRIX)
map_builder = MapBuilder()

state = {
    "target_class": "person",
    "follow_active": False,
    "explore_active": False,
    "latest_detection": None,
    "manual_active": False,
    "manual_velocity": {
        "forward": 0.0,
        "right": 0.0,
        "down": 0.0,
        "yaw": 0.0
    },
    "last_manual_time": 0.0,
    "target_behavior": {
        "active": False,
        "wall_size": 15.0,
        "speed": 1.0,
        "current_pos": {"x": 5.0, "y": 5.0},
        "target_pos": {"x": 5.0, "y": 5.0}
    }
}

explorer = Explorer(drone, map_builder, state)
@app.on_event("startup")
async def startup_event():
    # Start camera thread
    camera.start()
    
    # Connect to drone in background
    asyncio.create_task(drone.connect())
    
    # Start control loop
    asyncio.create_task(control_loop())
    
    # Start target movement loop
    asyncio.create_task(target_movement_loop())
    
    # Start sensor sync telemetry loop
    asyncio.create_task(sync_telemetry_loop())

async def sync_telemetry_loop():
    """Background loop to feed telemetry into SensorSync at 50Hz"""
    while True:
        try:
            if drone.connected:
                sensor_sync.add_telemetry(drone.telemetry)
        except Exception as e:
            print(f"Error in sync_telemetry_loop: {e}")
        await asyncio.sleep(0.02)  # 50Hz

async def control_loop():
    """Main loop for sending offboard commands based on detections or manual controls"""
    while True:
        try:
            if drone.connected:
                if state["follow_active"]:
                    await follower.update(state["latest_detection"], state["target_class"])
                elif state["manual_active"]:
                    # Timeout fail-safe (1.5 seconds)
                    if time.time() - state["last_manual_time"] > 1.5:
                        state["manual_active"] = False
                        state["manual_velocity"] = {"forward": 0.0, "right": 0.0, "down": 0.0, "yaw": 0.0}
                        print("Manual control timeout: stopping drone")
                        await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
                    else:
                        v = state["manual_velocity"]
                        await drone.send_velocity_command(v["forward"], v["right"], v["down"], v["yaw"])
        except Exception as e:
            print(f"Error in control loop: {e}")
        await asyncio.sleep(0.05)  # 20Hz control loop

async def target_movement_loop():
    """Background loop to update target model position in Gazebo"""
    dt = 0.5  # update every 0.5 seconds
    
    # Map UI/YOLO classes to Gazebo model names
    gazebo_models = {
        "person": "person_standing",
        "car": "car",
        "red_sphere": "red_sphere",
        "blue_cube": "blue_cube",
        "green_cone": "green_cone",
        "yellow_cylinder": "yellow_cylinder"
    }

    # Map model names to their correct ground Z-heights
    model_heights = {
        "person_standing": 0.7,
        "car": 0.4,
        "red_sphere": 0.3,
        "blue_cube": 0.25,
        "green_cone": 0.3,
        "yellow_cylinder": 0.3
    }

    model_defaults = {
        "person_standing": (5.0, 5.0),
        "car": (-5.0, -5.0),
        "red_sphere": (5.0, 0.0),
        "blue_cube": (0.0, 5.0),
        "green_cone": (-5.0, 0.0),
        "yellow_cylinder": (0.0, -5.0)
    }

    last_target_model = None

    while True:
        try:
            beh = state["target_behavior"]
            target_cls = state["target_class"]
            gz_model = gazebo_models.get(target_cls, target_cls)
            z_val = model_heights.get(gz_model, 0.3)
            
            # Reset coordinates when changing targets to prevent sudden teleports
            if last_target_model != gz_model:
                last_target_model = gz_model
                default_pos = model_defaults.get(gz_model, (0.0, 0.0))
                beh["current_pos"] = {"x": default_pos[0], "y": default_pos[1]}
                beh["target_pos"] = {"x": default_pos[0], "y": default_pos[1]}

            if beh["active"]:
                speed = beh["speed"]
                wall_size = beh["wall_size"]
                curr = beh["current_pos"]
                tgt = beh["target_pos"]
                
                # If close to target destination, pick a new one
                dist = math.hypot(tgt["x"] - curr["x"], tgt["y"] - curr["y"])
                if dist < 0.2:
                    half = wall_size / 2.0
                    tgt["x"] = random.uniform(-half, half)
                    tgt["y"] = random.uniform(-half, half)
                else:
                    # Move towards target
                    dx = (tgt["x"] - curr["x"]) / dist
                    dy = (tgt["y"] - curr["y"]) / dist
                    move_dist = min(speed * dt, dist)
                    curr["x"] += dx * move_dist
                    curr["y"] += dy * move_dist
                
                # Update gazebo model using subprocess
                # We run this in the background to avoid blocking the event loop
                process = await asyncio.create_subprocess_exec(
                    "gz", "model", "-m", gz_model, 
                    "-x", str(curr["x"]), "-y", str(curr["y"]), "-z", str(z_val),
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )
                stdout, stderr = await process.communicate()
                if process.returncode != 0:
                    print(f"Error updating Gazebo model {gz_model}: {stderr.decode().strip()}")
            else:
                # If not active, enforce position so it ignores collisions/physics (acts static)
                curr = beh["current_pos"]
                process = await asyncio.create_subprocess_exec(
                    "gz", "model", "-m", gz_model, 
                    "-x", str(curr["x"]), "-y", str(curr["y"]), "-z", str(z_val),
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )
                stdout, stderr = await process.communicate()
                if process.returncode != 0:
                    print(f"Error updating Gazebo model {gz_model}: {stderr.decode().strip()}")
        except Exception as e:
            print(f"Error in target movement loop: {e}")
        await asyncio.sleep(dt)

def generate_video():
    """Generator for MJPEG stream"""
    frame_count = 0
    while True:
        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.1)
            continue
            
        frame_count += 1
        
        # Run detection
        detector.set_target(state["target_class"])
        annotated_frame, detection_info = detector.process_frame(frame)
        
        # Sync frame with telemetry
        synced_data = sensor_sync.add_frame(frame)
        if synced_data and frame_count % 3 == 0:  # ~10 FPS for VO to save CPU
            # Get smooth EKF2 pose from telemetry
            odom = synced_data['telemetry']['odometry']
            ekf_pose = (odom["x"], odom["y"], odom["z"])
            state["slam_pose"] = {"x": odom["x"], "y": odom["y"], "z": odom["z"]}
            
            # Run Visual Odometry to find keypoint matches
            pose, R, matches, kp = vo.process_frame(synced_data)
            
            # Update Map in background thread using smooth EKF2 pose (always run to clear free space)
            import threading
            matched_kps = [kp[m.trainIdx] for m in matches] if len(matches) > 0 else []
            threading.Thread(target=map_builder.update_map, args=(ekf_pose, matched_kps, frame.shape[:2], synced_data['telemetry'], detection_info)).start()
        
        # Update global state for the control loop
        state["latest_detection"] = detection_info
        
        # Draw HUD Diagnostic Info on the video frame
        try:
            # Draw semi-transparent background at the top
            overlay = annotated_frame.copy()
            cv2.rectangle(overlay, (0, 0), (640, 50), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)

            # Draw tracking state and calculated errors
            is_active = state["follow_active"]
            status_str = "ACTIVE" if is_active else "INACTIVE"
            state_str = follower.state if is_active else "IDLE"
            
            err_x = getattr(follower, 'last_err_x', 0.0)
            err_alt = getattr(follower, 'last_err_alt', 0.0)
            cmd_yaw = getattr(follower, 'cmd_yaw', 0.0)
            cmd_alt = getattr(follower, 'cmd_alt', 0.0)
            
            text_line1 = f"AUTO: {status_str} | STATE: {state_str} | TARGET: {state['target_class'].upper()}"
            text_line2 = f"YAW: {err_x:+.1f}px (cmd: {cmd_yaw:+.2f}) | ALT: {err_alt:+.2f}m (cmd: {cmd_alt:+.2f})"
            
            cv2.putText(annotated_frame, text_line1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if is_active else (100, 100, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, text_line2, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0) if is_active else (200, 200, 200), 1, cv2.LINE_AA)
        except Exception as hud_err:
            print(f"Error rendering HUD: {hud_err}")

        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        # Small sleep to limit frame rate slightly (aiming for ~20-30 FPS)
        time.sleep(0.03)

@app.get("/map_image")
async def get_map_image():
    """Return the current SLAM map as a JPEG."""
    pose_dict = state.get("slam_pose", {"x": 0.0, "y": 0.0, "z": 0.0})
    pose = (pose_dict["x"], pose_dict["y"], pose_dict["z"])
    yaw = drone.telemetry.get("attitude", {}).get("yaw", 0.0)
    img = map_builder.get_map_image(pose, yaw)
    ret, buffer = cv2.imencode('.jpg', img)
    if not ret:
        return Response(status_code=500)
    return Response(content=buffer.tobytes(), media_type="image/jpeg")

@app.get("/")
async def get_index():
    with open(os.path.join(web_dir, "index.html"), "r") as f:
        return HTMLResponse(content=f.read())

@app.get("/style.css")
async def get_css():
    with open(os.path.join(web_dir, "style.css"), "r") as f:
        return HTMLResponse(content=f.read(), media_type="text/css")

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_video(), media_type="multipart/x-mixed-replace; boundary=frame")

# --- API Endpoints ---

@app.post("/api/target")
async def set_target(payload: dict):
    try:
        target = payload.get("target")
        if target:
            state["target_class"] = target
            return {"success": True, "target": target}
        return {"success": False, "error": "No target provided"}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/arm")
async def arm_drone():
    try:
        await drone.arm()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/disarm")
async def disarm_drone():
    try:
        await drone.disarm()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/takeoff")
async def takeoff_drone(payload: dict):
    try:
        alt = payload.get("altitude", 5.0)
        await drone.takeoff(altitude=float(alt))
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/land")
async def land_drone():
    try:
        state["follow_active"] = False
        await drone.land()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/follow/start")
async def start_follow():
    try:
        if not drone.telemetry["armed"]:
            return {"success": False, "error": "Drone not armed"}
            
        success = await drone.start_offboard()
        if success:
            state["follow_active"] = True
            explorer.stop()
            follower.start()
            return {"success": True}
        return {"success": False, "error": "Failed to start offboard mode"}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/follow/stop")
async def stop_follow():
    try:
        state["follow_active"] = False
        follower.stop()
        await drone.stop_offboard()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/slam/explore")
async def start_explore():
    try:
        # Save old map to logs folder and reset map
        pose_dict = state.get("slam_pose", {"x": 0.0, "y": 0.0, "z": 0.0})
        pose = (pose_dict["x"], pose_dict["y"], pose_dict["z"])
        yaw = drone.telemetry.get("attitude", {}).get("yaw", 0.0)
        old_map_filename = f"logs/old_map_{int(time.time())}.png"
        map_builder.save_map(old_map_filename, pose, yaw)
        print(f"Old map saved to {old_map_filename}")
        map_builder.reset()

        state["explore_active"] = True
        follower.stop()
        asyncio.create_task(explorer.start())
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/slam/stop_explore")
async def stop_explore():
    try:
        state["explore_active"] = False
        explorer.stop()
        await drone.stop_offboard()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/manual/move")
async def manual_move(payload: dict):
    try:
        if not drone.connected:
            return {"success": False, "error": "Drone is disconnected"}
        if not drone.telemetry["armed"]:
            return {"success": False, "error": "Drone is not armed"}
        
        # Disable follow/explore if active
        if state["follow_active"]:
            state["follow_active"] = False
            follower.stop()
        if state.get("explore_active"):
            state["explore_active"] = False
            explorer.stop()
        
        # Ensure we are in OFFBOARD flight mode
        if "OFFBOARD" not in drone.telemetry["flight_mode"].upper():
            success = await drone.start_offboard()
            if not success:
                return {"success": False, "error": "Failed to start offboard mode"}
        
        # Update manual velocity and timestamp
        state["manual_velocity"] = {
            "forward": float(payload.get("forward", 0.0)),
            "right": float(payload.get("right", 0.0)),
            "down": float(payload.get("down", 0.0)),
            "yaw": float(payload.get("yaw", 0.0))
        }
        state["last_manual_time"] = time.time()
        state["manual_active"] = True
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/rtl")
async def rtl_drone():
    try:
        state["follow_active"] = False
        state["manual_active"] = False
        await drone.rtl()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/target/behavior")
async def set_target_behavior(payload: dict):
    try:
        if "active" in payload:
            state["target_behavior"]["active"] = payload["active"]
        if "wall_size" in payload:
            state["target_behavior"]["wall_size"] = float(payload["wall_size"])
        if "speed" in payload:
            state["target_behavior"]["speed"] = float(payload["speed"])
        return {"success": True, "behavior": state["target_behavior"]}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.post("/api/follower/parameters")
async def set_follower_parameters(payload: dict):
    try:
        if "kp_yaw" in payload:
            follower.kp_yaw = float(payload["kp_yaw"])
        if "kp_fwd" in payload:
            follower.kp_fwd = float(payload["kp_fwd"])
        if "kp_alt" in payload:
            follower.kp_alt = float(payload["kp_alt"])
        if "alpha_yaw" in payload:
            follower.alpha_yaw = float(payload["alpha_yaw"])
        if "alpha_fwd" in payload:
            follower.alpha_fwd = float(payload["alpha_fwd"])
        if "alpha_alt" in payload:
            follower.alpha_alt = float(payload["alpha_alt"])
        if "max_fwd_speed" in payload:
            follower.max_fwd_speed = float(payload["max_fwd_speed"])
        return {
            "success": True, 
            "parameters": {
                "kp_yaw": follower.kp_yaw,
                "kp_fwd": follower.kp_fwd,
                "kp_alt": follower.kp_alt,
                "alpha_yaw": follower.alpha_yaw,
                "alpha_fwd": follower.alpha_fwd,
                "alpha_alt": follower.alpha_alt,
                "max_fwd_speed": follower.max_fwd_speed
            }
        }
    except Exception as e:
        return {"success": False, "error": str(e)}

# --- WebSocket for live Telemetry ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = {
                "telemetry": drone.telemetry,
                "detection": state["latest_detection"],
                "target_class": state["target_class"],
                "follow_active": state["follow_active"],
                "explore_active": state.get("explore_active", False),
                "drone_connected": drone.connected,
                "target_behavior": state["target_behavior"],
                "slam_pose": state.get("slam_pose", {"x": 0, "y": 0, "z": 0}),
                "follower_params": {
                    "kp_yaw": follower.kp_yaw,
                    "kp_fwd": follower.kp_fwd,
                    "kp_alt": follower.kp_alt,
                    "alpha_yaw": follower.alpha_yaw,
                    "alpha_fwd": follower.alpha_fwd,
                    "alpha_alt": follower.alpha_alt,
                    "max_fwd_speed": follower.max_fwd_speed
                }
            }
            await websocket.send_json(data)
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
