# 🛸 Autonomous Vision-Based Drone Follower

An advanced, offline-capable closed-loop visual servoing system that enables an autonomous drone (PX4 SITL / Gazebo Classic) to detect, track, and follow dynamic targets in real-time. The system integrates deep learning (`YOLOv8`), traditional computer vision (`HSV Color Masking`), flight controller integration (`MAVSDK`), and a modern `FastAPI` control dashboard.

---

## 🏗️ System Architecture

```mermaid
graph TD
    subgraph Gazebo Classic 11 / PX4 SITL
        iris[Iris Quadcopter + FPV Cam] -->|UDP Stream: Port 5600| gst[GStreamer Pipeline]
        sim[Physics World / Targets] <-->|Subprocess API / gz model| srv[FastAPI Web Server]
    end

    subgraph Computer Vision Pipeline
        gst -->|Raw Frames| cv_proc[Frame Processor]
        cv_proc -->|COCO Targets: Person/Car| yolo[YOLOv8 Inference]
        cv_proc -->|Geometric Primitives| hsv[HSV Color Tracker]
    end

    subgraph Control Loop (Visual Servoing)
        yolo & hsv -->|BBox & Estimated Distance| pid[PID Tracking Controller]
        pid -->|Yaw & Velocity Commands| mav[MAVSDK Controller API]
        mav -->|Flight Commands: UDP 14540| iris
    end

    subgraph User Interface
        srv -->|WebSocket Telemetry & MJPEG Stream| ui[Glassmorphism Web UI]
        ui -->|API Requests: Control & Parameters| srv
    end
```

---

## 🚀 Key Features

### 1. Dual-Path Detection Pipeline (`src/detector.py`)
* **True YOLO Path**: Real-time object detection using `YOLOv8n` for complex, textured classes (`person`, `car`). Targets are rendered using realistic, high-fidelity 3D meshes (adapted from the Gazebo database) to maximize detection confidence.
* **HSV Color Path**: Low-latency, robust HSV thresholding and contour tracking for geometric primitives (`red_sphere`, `blue_cube`, `green_cone`, `yellow_cylinder`).
* **Camera Projection Distance Estimation**:
  Uses thin-lens projection geometry to estimate target distance dynamically:
  $$\text{Distance} = \frac{\text{Physical Height (m)} \times \text{Camera Focal Length (px)}}{\text{Bounding Box Height (px)}}$$
  *(Focal length calibrated to $277.19$ pixels based on the simulation camera matrix)*.

### 2. Closed-Loop PD Flight Control (`src/follower.py`)
* **Prioritized Two-Stage Servo**:
  1. **Rotational & Altitude Centering**: Drone yaws and adjusts altitude to align the target's center within a deadzone vertical band ($240\text{px} - 440\text{px}$ in the $640\times480$ frame) to ensure stability before forward progression.
  2. **Range Tracking**: Adjusts forward pitch velocity ($V_x$) proportionally to maintain the user-configured tracking distance (default: $3.0\text{m}$).
* **Anti-Windup & Smooth Control**: Features bounds-based error calculations and command rate-limiters to eliminate sudden high-frequency flight oscillations.

### 3. Dynamic Obstacle Avoidance for Targets (`web/server.py`)
* The background target movement loop features a **dynamic collision avoidance engine** for random target path planning.
* Targets map physical exclusion zones based on double their bounding radius ($2 \times r$):
  * Hatchback (`car`): $r = 1.5\text{m}$ (exclusion zone: $3.0\text{m}$)
  * Human (`person`): $r = 0.4\text{m}$ (exclusion zone: $0.8\text{m}$)
  * Primitives: $r = 0.3\text{m}$ (exclusion zone: $0.6\text{m}$)
* If a path step intersects another object's exclusion zone, the path planner aborts the current vector and dynamically recalculates a clear direction vector.

### 4. Interactive Glassmorphism Dashboard (`web/`)
* **Real-time Video Feed**: Low-latency MJPEG streaming with overlay showing:
  * Dynamic YOLO / HSV bounding boxes.
  * Real-time estimated range.
  * Real-time flight telemetry (altitude, battery, GPS connection quality).
* **Control Center**: Dynamic target selection grid, flight parameters tuning UI, live state displays, and flight-state overrides.

---

## 🛠️ Requirements & Dependencies

* **Operating System**: Ubuntu 20.04 / 22.04 LTS
* **Simulator**: Gazebo Classic 11 with PX4 Autopilot SITL (`~/PX4-Autopilot`)
* **GStreamer**: Standard Linux Gstreamer development libraries
* **Python Dependencies**:
  * `opencv-python`
  * `ultralytics` (YOLOv8 framework)
  * `mavsdk` (MAVLink Developer Suite)
  * `fastapi`, `uvicorn` (Dashboard server)
  * `numpy`

---

## ⚙️ Setup & Execution

### 1. Verification of Simulator Path
Ensure that your PX4 SITL repository is cloned under `${HOME}/PX4-Autopilot`. The startup script automatically compiles and binds the SITL dependencies.

### 2. Run the Unified Launcher
A single, self-healing script cleans up orphan simulator instances, sources all ROS 2 and Gazebo paths, initializes local environment paths, and launches both simulation and server:

```bash
chmod +x start_all.sh
./start_all.sh
```

### 3. Open the Dashboard
Navigate to `http://localhost:8080` in your web browser.

---

## 📁 Repository Structure

```
├── README.md                           # Documentation
├── start_all.sh                        # Master bash orchestrator script
├── launch_vision.sh                    # Handles Gazebo, PX4 SITL & Drone spawn
├── gazebo/
│   ├── worlds/
│   │   └── vision_targets.world       # World file with target spawn coordinates
│   └── models/                         # Custom models (Person mesh, Hatchback mesh, shapes)
├── src/
│   ├── camera_stream.py                # GStreamer frame receiver thread
│   ├── detector.py                     # Dual-path object detector (YOLOv8/HSV)
│   ├── drone_controller.py             # MAVSDK abstraction interface
│   └── follower.py                     # Flight controller tracking logic
├── web/
│   ├── server.py                       # FastAPI server & telemetry router
│   └── templates/                      # Dashboard UI templates
└── models/
    └── yolov8n.pt                      # Bundled offline YOLOv8 weights
```

---

## 🔒 Safety and Failure Modes
* **Loss of Target**: If the target exits the camera FOV, the drone stops forward velocity and initiates a slow horizontal yaw scan to relocate the object.
* **Geofence Enforcement**: Target movement and drone movements are bound within a user-defined coordinate cage (default: $10\text{m} \times 10\text{m}$).
