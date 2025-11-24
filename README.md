# Intellibot Autonomous Navigation 

![Version](https://img.shields.io/badge/version-5.0.0-blue.svg) ![Python](https://img.shields.io/badge/python-3.9%2B-yellow.svg) ![Status](https://img.shields.io/badge/status-Production-green.svg) ![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)

**Intellibot** is a high-fidelity autonomous mobile robot simulator featuring a service-oriented architecture, advanced path planning algorithms, and a futuristic "Command Center" dashboard. It simulates a differential drive robot with realistic physics, LIDAR sensor fusion, dynamic costmap generation, and automated recovery behaviors similar to ROS (Robot Operating System) navigation stacks.

---

## 🌟 Key Features

### 🧠 Advanced Navigation Stack
*   **Global Planner:** A* Algorithm with **Costmap Inflation** (robots prefer staying away from walls).
*   **Path Smoothing:** B-Spline trajectory generation for organic, curved movement using `scipy`.
*   **Local Controller:** Pure Pursuit algorithm with dynamic lookahead for smooth path following.
*   **Dynamic Re-planning:** Real-time obstacle detection triggers immediate path recalculation.

### 🛡️ Smart Recovery Behaviors
Unlike basic simulators, Intellibot V5 features a robust state machine for handling "stuck" scenarios:
1.  **Stuck Detection:** Monitors velocity vs. positional change.
2.  **Deep Recovery Maneuver:** Performs a high-speed reverse followed by a "J-Turn" to reorient the sensors.
3.  **Map Cleansing:** Forces a global map update and re-plan after recovery.

### 🔋 Energy Management
*   **Battery Physics:** Simulated drain based on motor usage and idle CPU load.
*   **Auto-Docking:** One-click autonomous navigation to the nearest charging station.
*   **Charging Zones:** Visual detection and state switching when docked.

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
    *   Renders the visual state and captures user input.

---

## 🚀 Installation

### Prerequisites
*   Python 3.9 or higher.

### Dependencies
Install the required packages using pip:

```bash
pip install fastapi uvicorn numpy requests websockets customtkinter pydantic scipy
```

---

## 🕹️ Usage

1.  **Start the System:**
    Run the main script. The backend server will initialize in a background thread, followed by the GUI.
    ```bash
    python robotics.py
    ```

2.  **Controls:**
    *   **Set Navigation Goal:** Left-Click anywhere on the map (black area). The robot will plan a curved path and execute it.
    *   **Auto-Dock:** Click the **"RETURN TO DOCK"** button. The robot will find the closest charger (Green Zones) and navigate there.
    *   **Manual Override:** Use the On-Screen Arrow Pad to take control. *Note: Manual input disables autonomous mode immediately.*

3.  **Interpreting the UI:**
    *   **Cyan Line:** The calculated B-Spline path.
    *   **Red Rays:** LIDAR detection hits (obstacles).
    *   **Green Zones:** Charging stations.
    *   **Radar (Bottom Right):** Top-down raw sensor view relative to the robot.

---

## 🧠 Technical Deep Dive

### The "Costmap" Concept
In V5, the environment isn't just "0" (Free) or "1" (Occupied). We implement an **Inflation Radius**.
*   **Walls:** Cost = Infinite.
*   **Near Walls:** Cost = High.
*   **Open Space:** Cost = Low.
The A* planner uses these costs to generate paths that naturally curve around corners with a safety margin, rather than hugging walls and clipping edges.

### Recovery Logic (The "J-Turn")
If the robot's wheels are spinning but position isn't changing (`dist_moved < threshold`), the `stuck_timer` increments. If it exceeds 1.5 seconds:
1.  **Phase 1:** Set target linear velocity to `-1.5` (Reverse).
2.  **Phase 2:** Set angular velocity to `1.5` (Turn) while reversing.
3.  **Phase 3:** Clear current path -> Reset Planner -> Calculate new route from new orientation.

---

## 📝 Configuration

You can tweak the physics and behavior in the `Config` class at the top of the file:

```python
class Config:
    MAX_LIN_VEL = 2.2       # Max Speed
    INERTIA = 0.85          # 0.0 (Instant) to 0.99 (Heavy Slide)
    INFLATION_RADIUS = 2.5  # Safety distance from walls
    LIDAR_RAYS = 72         # Sensor resolution
    # ...
```

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

**Developed for the Intellibot Project.**
*Precision Robotics & Autonomous Systems Simulation.*
