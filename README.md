# 🤖 IoT Level-4 Autonomous Spatial Mapping Rover

An autonomous 4-wheel differential-drive rover using an ESP32 edge node to stream ultrasonic + IMU telemetry over UDP to a ROS 2 backend, which performs dead-reckoning odometry and ray-traced occupancy grid mapping, visualized on an HTML5 web dashboard and RViz2.

---

## System Architecture

```
[ESP32 Edge Node]
  └── HC-SR04 x3 + MPU6050
  └── UDP stream → port 5000 → "yaw,left,front,right"

[ROS 2 Backend — WSL2 Ubuntu Jazzy]
  └── rover_bridge_node  — UDP ingestion + odometry + TF
  └── mapper_node        — Bresenham ray-tracing → OccupancyGrid
  └── rosbridge          — WebSocket gateway port 9090

[Visualization]
  └── RViz2              — 3D view (path + sensor cones + map)
  └── Web Dashboard      — 2D occupancy map + live telemetry
```

---

## Hardware

| Component | Model | Notes |
|---|---|---|
| Microcontroller | ESP32 DevKit | Edge node |
| Motor Driver | L298N x2 | 4x BO gear motors |
| Sonar | HC-SR04 x3 | Front, Left, Right |
| IMU | MPU6050 | Yaw integration |
| Battery | 18650 2S | 7.4V pack |
| Chassis | 255×150mm | Wheelbase ~123mm |

---

## Software Requirements

### ROS 2 Backend (WSL2 Ubuntu)
```bash
sudo apt install ros-jazzy-rosbridge-suite
sudo apt install ros-jazzy-tf2-web-republisher
```

### Web Dashboard
Download these files and place in `web/` folder:
- `three.js` — https://cdn.jsdelivr.net/npm/three@0.89.0/build/three.min.js
- `ros3d.js` — https://cdn.jsdelivr.net/npm/ros3d/build/ros3d.min.js

---

## Project Structure

```
swarm-iot-rover/
├── firmware/
│   └── esp32wifi_UDP.ino       # ESP32 Arduino firmware
├── ros2_ws/src/swarm_bot/
│   ├── swarm_bot/
│   │   ├── rover_bridge_node.py  # UDP → ROS topics
│   │   ├── mapper_node.py        # Ray-tracing occupancy grid
│   │   └── room_simulator.py     # Hardware-free simulation
│   ├── launch/
│   │   ├── telemetry_launch.py   # Real robot mode
│   │   └── sim_launch.py         # Simulation mode
│   ├── setup.py
│   └── package.xml
└── web/
    └── index.html              # Web dashboard
```

---

## Running the Project

### Simulation Mode (no hardware needed)
```bash
# Terminal 1 — ROS backend
cd ~/swarm_ws && source install/setup.bash
ros2 launch swarm_bot sim_launch.py

# Terminal 2 — Web server (Windows PowerShell)
cd E:\path\to\web
python -m http.server 8000
```
Open `http://localhost:8000` in browser.

### Real Robot Mode (ESP32 connected)
```bash
# Flash firmware/esp32wifi_UDP.ino to ESP32
# Set your WiFi credentials and laptop IP in the firmware

# Terminal 1 — ROS backend
cd ~/swarm_ws && source install/setup.bash
ros2 launch swarm_bot telemetry_launch.py

# Terminal 2 — Web server
python -m http.server 8000
```

### RViz2 (optional)
```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
# Fixed Frame: odom
# Add: /map (Map), /odom (Odometry), /sensor/front|left|right (Range)
```

---

## Configuration

### ESP32 Firmware (`firmware/esp32wifi_UDP.ino`)
Edit these values before flashing:
```cpp
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* targetIP = "YOUR_LAPTOP_IP";  // WSL2 host IP
const int   udpPort  = 5000;
```

### Key Constants
| Parameter | Value | Location |
|---|---|---|
| Obstacle distance | 25 cm | ESP32 firmware |
| Critical distance | 12 cm | ESP32 firmware |
| Cruise speed | 125 PWM | ESP32 firmware |
| Map resolution | 5 cm/cell | mapper_node.py |
| Map size | 200×200 cells = 10m×10m | mapper_node.py |
| Wheelbase | 0.123 m | rover_bridge_node.py |

---

## Known Limitations

- Dead-reckoning drift accumulates over time (no SLAM loop closure)
- HC-SR04 echo pins output 5V to 3.3V ESP32 pins (no level shifter)
- L298N 5V regulator near capacity under full motor load
- Velocity constant needs empirical calibration per battery charge level

---

## Author

**Rahul Kumar Mishra**  
GitHub: [@avyaya-s](https://github.com/avyaya-s)