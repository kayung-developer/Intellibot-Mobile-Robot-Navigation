# Intellibot Autonomous Navigation

![Version](https://img.shields.io/badge/version-5.0.0-blue.svg) ![Python](https://img.shields.io/badge/python-3.9%2B-yellow.svg) ![Status](https://img.shields.io/badge/status-Production-green.svg) ![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)

**Intellibot** is a high-fidelity autonomous mobile robot simulator featuring a service-oriented architecture, advanced path planning algorithms, and a futuristic "Command Center" dashboard. It simulates a differential drive robot with realistic physics, LIDAR sensor fusion, dynamic costmap generation, and automated recovery behaviors similar to professional ROS (Robot Operating System) navigation stacks.

---

## 🌟 Key Features

### 🧠 Professional Navigation Stack
*   **Global Planner:** Uses **A* Algorithm** combined with **Costmap Inflation**. Walls emit a "high cost" field, forcing the robot to plan paths that naturally curve around obstacles rather than hugging edges.
*   **Path Smoothing:** Implements **B-Spline interpolation** (`scipy`) to convert jagged grid movements into organic, curved trajectories.
*   **Local Controller:** Features a **Pure Pursuit** controller with dynamic lookahead for smooth path following.
*   **Dynamic Re-planning:** Real-time obstacle detection via LIDAR triggers immediate path recalculation.

### 🛡️ Deep Recovery Behaviors
Unlike basic simulators, Intellibot V5 features a robust state machine for handling "stuck" scenarios (e.g., getting trapped in a corner):
1.  **Stuck Detection:** Monitors velocity vs. positional change over time.
2.  **J-Turn Maneuver:** Executes a high-speed reverse followed by a distinct rotation to reorient sensors and break free from local minima.
3.  **Map Cleansing:** Forces a global map update and a brand new plan after recovery.

### 🔋 Energy Management
*   **Battery Physics:** Simulated drain based on motor usage and idle CPU load.
*   **Auto-Docking:** One-click autonomous navigation to the nearest charging station.
*   **Charging Zones:** Visual detection (Green Zones) and automatic charging state switching.

### 🖥️ Command Center UI
*   **Real-time Telemetry:** Sparkline graphs for Velocity and Battery levels.
*   **Sensor Fusion Display:** Raw LIDAR point cloud visualization (Radar Scope).
*   **Interactive Map:** Click-to-nav, Fog of War (walls), and real-time raycasting visuals.
*   **Console Logging:** Scrolling terminal for system events and debugging.

---

## 🛠️ Architecture

The system follows a decoupled **Client-Server** model running asynchronously within a single Python process:

1.  **Backend (FastAPI & Uvicorn):**
    *   **Environment Service:** Manages the grid, static walls, and dynamic obstacles.
    *   **Planner Service:** Handles A* logic and B-Spline interpolation.
    *   **Robot Entity:** Simulates physics (inertia, kinematics) and control loops.
2.  **Communication Layer:**
    *   **WebSockets:** Streams high-frequency telemetry (60Hz) to the UI.
    *   **REST API:** Handles transactional commands (Set Goal, Manual Override).
3.  **Frontend (CustomTkinter):**
    *   Renders the visual state, graphs, and captures user input.

---

## 🚀 Installation

### Prerequisites
*   Python 3.9 or higher.

### Dependencies
Install the required packages using pip:

```bash
pip install fastapi uvicorn numpy requests websockets customtkinter pydantic scipy pyinstaller
```

---

## 🕹️ Usage

### Running from Source
1.  Navigate to the project directory.
2.  Run the main script:
    ```bash
    python robotics.py
    ```

### Controls
*   **Set Navigation Goal:** **Left-Click** anywhere on the black map area. The robot will plan a curved path and execute it.
*   **Auto-Dock:** Click the **"RETURN TO DOCK"** button. The robot will find the closest charger (Green Zones) and navigate there.
*   **Manual Override:** Use the **On-Screen Arrow Pad** to take control. 
    *   *Note: Manual input disables autonomous mode immediately.*

### Interpreting the UI
*   **Cyan Line:** The calculated B-Spline path.
*   **Red Rays:** LIDAR rays hitting an obstacle.
*   **Green Zones:** Charging stations.
*   **Radar (Bottom Right):** Top-down raw sensor view relative to the robot.

---

## 📦 Building a Windows Executable

To create a standalone `.exe` file that runs without Python installed, use **PyInstaller**. 

**Important:** We must use the `--collect-all` flag to ensure `customtkinter` theme assets are packaged correctly.

1.  Open your terminal/command prompt.
2.  Run the following command:

```bash
pyinstaller --noconsole --onefile --collect-all customtkinter --name IntellibotV5 intellibot.py
```

3.  Once finished, check the **`dist/`** folder for `Intellibot.exe`.

---

## 📝 Configuration

You can tweak the physics and behavior in the `Config` class at the top of the `intellibot_pro_v5.py` file:

```python
class Config:
    MAX_LIN_VEL = 2.2       # Max Speed
    INERTIA = 0.85          # 0.0 (Instant) to 0.99 (Heavy Slide)
    INFLATION_RADIUS = 2.5  # Safety distance cost around walls
    LIDAR_RAYS = 72         # Sensor resolution
    REPLAN_TRIGGER_DIST = 3.5 # Distance to detect obstacles ahead
    # ...
```

---

## 📄 License

This project is licensed under the MIT License.

---

**Developed for the Intellibot Project.**
*Precision Robotics & Autonomous Systems Simulation.*
