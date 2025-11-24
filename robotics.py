import tkinter
import customtkinter as ctk
import asyncio
import threading
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import numpy as np
import math
import heapq
import time
import json
import requests
from contextlib import asynccontextmanager
from collections import deque
from datetime import datetime
from scipy.interpolate import splprep, splev
import websockets

# --- CONFIGURATION ---
class Config:
    APP_TITLE = "INTELLIBOT"
    # Colors
    THEME_PRIMARY = "#00F0FF"    
    THEME_SECONDARY = "#7000FF"  
    THEME_WARNING = "#FFCC00"    
    THEME_DANGER = "#FF003C"     
    THEME_SUCCESS = "#00FF9D"    
    BG_COLOR = "#050505"         
    PANEL_COLOR = "#0F111A"      
    
    # Physics & Map
    MAP_WIDTH = 40
    MAP_HEIGHT = 25
    TICK_RATE = 0.05
    ROBOT_RADIUS = 0.4
    
    # Navigation Tuning
    MAX_LIN_VEL = 2.2
    MAX_ANG_VEL = 3.5
    INERTIA = 0.85 
    ARRIVAL_TOLERANCE = 0.5
    
    # Obstacle Handling
    REPLAN_TRIGGER_DIST = 3.5   
    INFLATION_RADIUS = 2.5      # INCREASED: Forces wider turns around walls
    WALL_COST = 50.0            # High cost for being near walls
    
    # Sensors
    LIDAR_RAYS = 72
    LIDAR_FOV = 200
    LIDAR_RANGE = 12
    
    # Energy
    BATTERY_CAPACITY = 100.0
    CHARGE_RATE = 0.5
    DRAIN_MOVE = 0.03
    CHARGING_STATIONS = [(2, 2, 3, 3), (35, 20, 3, 3)]

CONFIG = Config()

# --- DATA MODELS ---
class Point(BaseModel):
    x: float
    y: float

class RobotStateDTO(BaseModel):
    x: float
    y: float
    theta: float
    v_lin: float
    battery: float
    status: str
    is_charging: bool
    is_autonomous: bool

class SystemPacket(BaseModel):
    robot: RobotStateDTO
    lidar: dict
    path: list[list[float]]
    logs: list[str]
    map_updates: list[tuple]

# --- SERVICES ---

class MathUtils:
    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def smooth_path_bspline(path_coords):
        if len(path_coords) < 3: return path_coords
        try:
            clean_path = [path_coords[0]]
            for i in range(1, len(path_coords)):
                if path_coords[i] != path_coords[i-1]:
                    clean_path.append(path_coords[i])
            if len(clean_path) < 3: return clean_path
            
            x, y = [p[0] for p in clean_path], [p[1] for p in clean_path]
            tck, u = splprep([x, y], s=0.5, k=2)
            u_new = np.linspace(u.min(), u.max(), len(clean_path) * 4)
            x_new, y_new = splev(u_new, tck)
            return list(zip(x_new, y_new))
        except: return path_coords

class EnvironmentService:
    def __init__(self):
        self.width = CONFIG.MAP_WIDTH
        self.height = CONFIG.MAP_HEIGHT
        self.grid = np.zeros((self.height, self.width), dtype=int)
        self.costmap = np.ones((self.height, self.width), dtype=float)
        self.new_obstacles = []
        
        # Initial Walls
        self.add_block(10, 5, 2, 8)
        self.add_block(25, 12, 8, 2)
        self.add_block(5, 18, 5, 2)
        self.add_block(18, 5, 8, 2) 

    def add_block(self, x, y, w, h):
        for r in range(y, min(y+h, self.height)):
            for c in range(x, min(x+w, self.width)):
                self.grid[r, c] = 1
                self._inflate_cost(c, r)

    def mark_obstacle(self, x, y):
        ix, iy = int(x), int(y)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            if self.grid[iy, ix] == 0:
                self.grid[iy, ix] = 1
                self._inflate_cost(ix, iy)
                self.new_obstacles.append((ix, iy))
                return True
        return False

    def _inflate_cost(self, x, y):
        # Creates a "Heat Map" around walls so path planner avoids them
        r = int(CONFIG.INFLATION_RADIUS)
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                ny, nx = y+dy, x+dx
                if 0 <= ny < self.height and 0 <= nx < self.width:
                    if self.grid[ny, nx] == 0:
                        dist = math.sqrt(dx**2 + dy**2)
                        # Linear falloff cost
                        if dist <= r:
                            cost_penalty = (1.0 - (dist/r)) * CONFIG.WALL_COST
                            self.costmap[ny, nx] = max(self.costmap[ny, nx], 1.0 + cost_penalty)

    def is_occupied(self, x, y):
        if not (0 <= x < self.width and 0 <= y < self.height): return True
        return self.grid[int(y), int(x)] == 1
    
    def get_cost(self, x, y):
        if not (0 <= x < self.width and 0 <= y < self.height): return float('inf')
        return self.costmap[int(y), int(x)]
    
    def get_nearest_charger(self, rx, ry):
        best_dist = float('inf')
        best_pt = (rx, ry)
        for (cx, cy, w, h) in CONFIG.CHARGING_STATIONS:
            center_x, center_y = cx + w/2, cy + h/2
            dist = math.sqrt((rx-center_x)**2 + (ry-center_y)**2)
            if dist < best_dist:
                best_dist = dist
                best_pt = (center_x, center_y)
        return best_pt

class PathPlannerService:
    def __init__(self, env: EnvironmentService):
        self.env = env

    def plan(self, start: tuple, goal: tuple):
        start_cell = (int(start[0]), int(start[1]))
        goal_cell = (int(goal[0]), int(goal[1]))
        
        # Escape Logic: If robot is "in" a wall (due to radius), find nearest safe spot
        if self.env.is_occupied(*start):
            start_cell = self._find_nearest_free(start_cell)
            if not start_cell: return None
        
        if self.env.is_occupied(*goal): return None

        frontier = []
        heapq.heappush(frontier, (0, start_cell))
        came_from = {start_cell: None}
        cost_so_far = {start_cell: 0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell: break

            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.env.width and 0 <= nxt[1] < self.env.height): continue
                if self.env.grid[nxt[1], nxt[0]] == 1: continue

                # Weighted A*: Movement Cost * Inflation Cost
                cell_cost = self.env.get_cost(nxt[0], nxt[1])
                move_cost = math.sqrt(dx**2 + dy**2)
                new_cost = cost_so_far[current] + (move_cost * cell_cost)
                
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + math.sqrt((goal_cell[0]-nxt[0])**2 + (goal_cell[1]-nxt[1])**2)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal_cell not in came_from: return None
        
        path = []
        curr = goal_cell
        while curr:
            path.append((curr[0] + 0.5, curr[1] + 0.5))
            curr = came_from[curr]
        path.reverse()
        return MathUtils.smooth_path_bspline(path)

    def _find_nearest_free(self, cell):
        q = deque([cell])
        visited = {cell}
        while q:
            curr = q.popleft()
            if not self.env.is_occupied(curr[0], curr[1]): return curr
            if len(visited) > 50: return None
            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
                nxt = (curr[0]+dx, curr[1]+dy)
                if 0<=nxt[0]<self.env.width and 0<=nxt[1]<self.env.height and nxt not in visited:
                    visited.add(nxt); q.append(nxt)
        return None

class RobotEntity:
    def __init__(self):
        self.x = 3.5
        self.y = 3.5
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.battery = CONFIG.BATTERY_CAPACITY
        self.is_autonomous = False
        self.is_charging = False
        self.status = "IDLE"
        
        self.current_path = []
        self.path_index = 0
        self.final_goal = None
        self.logs = deque(maxlen=5)
        
        self.stuck_timer = 0
        self.last_pos = (0,0)
        self.recovery_mode = False
        self.recovery_step = 0
        self.replan_cooldown = 0

    def update_physics(self, dt, env: EnvironmentService):
        # Charging
        in_charger = False
        for (cx, cy, w, h) in CONFIG.CHARGING_STATIONS:
            if cx <= self.x <= cx+w and cy <= self.y <= cy+h:
                in_charger = True; break
        
        if in_charger and abs(self.v) < 0.1:
            self.is_charging = True
            self.battery = min(CONFIG.BATTERY_CAPACITY, self.battery + CONFIG.CHARGE_RATE)
            if self.battery >= 99.9 and self.battery < 100.0: self.log("INFO: Fully Charged")
        else:
            self.is_charging = False
            drain = (abs(self.v) * CONFIG.DRAIN_MOVE) + 0.001
            self.battery = max(0, self.battery - drain)

        # Physics
        alpha = CONFIG.INERTIA
        self.v = self.v * alpha + self.target_v * (1 - alpha)
        self.w = self.w * alpha + self.target_w * (1 - alpha)

        if self.battery > 0:
            new_x = self.x + self.v * math.cos(self.theta) * dt
            new_y = self.y + self.v * math.sin(self.theta) * dt
            if not env.is_occupied(new_x, new_y):
                self.x, self.y = new_x, new_y
            else:
                self.v = 0 # Collision stop
            self.theta = MathUtils.normalize_angle(self.theta + (self.w * dt))
        
        # Stuck Logic (Only if autonomous and supposed to be moving)
        dist_moved = math.sqrt((self.x - self.last_pos[0])**2 + (self.y - self.last_pos[1])**2)
        if self.is_autonomous and not self.recovery_mode and dist_moved < 0.02 and self.target_v > 0.1:
            self.stuck_timer += dt
        else:
            self.stuck_timer = 0
            self.last_pos = (self.x, self.y)
            
        if self.replan_cooldown > 0: self.replan_cooldown -= 1

    def run_autonomous_control(self, lidar_data, planner: PathPlannerService, env: EnvironmentService):
        if not self.current_path or not self.final_goal: return

        # --- DEEP RECOVERY MODE ---
        if self.stuck_timer > 1.5 or self.recovery_mode:
            self.execute_deep_recovery()
            return

        # --- OBSTACLE MAPPING & REPLAN ---
        path_blocked = False
        angles = np.array(lidar_data['angles'])
        dists = np.array(lidar_data['distances'])
        
        # Map Updates
        for i, dist in enumerate(dists):
            if dist < CONFIG.REPLAN_TRIGGER_DIST:
                hit_x = self.x + math.cos(angles[i]) * dist
                hit_y = self.y + math.sin(angles[i]) * dist
                env.mark_obstacle(hit_x, hit_y)

        # Check Path Intersection
        check_limit = min(self.path_index + 10, len(self.current_path))
        for i in range(self.path_index, check_limit):
            wx, wy = self.current_path[i]
            # Check against high cost areas (inflation), not just hard walls
            if env.is_occupied(wx, wy):
                path_blocked = True
                break

        if path_blocked and self.replan_cooldown == 0:
            self.log("WARN: Obstacle! Re-routing...")
            self.target_v = 0
            # Plan from current spot to final goal
            new_path = planner.plan((self.x, self.y), self.final_goal)
            
            if new_path:
                self.current_path = new_path
                self.path_index = 0
                self.replan_cooldown = 20 
                self.status = "RE-ROUTING"
                return
            else:
                self.log("CRITICAL: Blocked. Initiating Reverse.")
                self.stuck_timer = 5.0 # Force recovery
                return

        # --- PATH FOLLOWING ---
        self.status = "AUTONOMOUS"
        lookahead_idx = min(self.path_index + 3, len(self.current_path)-1)
        target = self.current_path[lookahead_idx]
        
        dist_to_wp = math.sqrt((self.x - self.current_path[self.path_index][0])**2 + 
                               (self.y - self.current_path[self.path_index][1])**2)
        if dist_to_wp < 1.0 and self.path_index < len(self.current_path) - 1:
            self.path_index += 1
            
        dist_to_final = math.sqrt((self.x - self.current_path[-1][0])**2 + (self.y - self.current_path[-1][1])**2)
        if dist_to_final < CONFIG.ARRIVAL_TOLERANCE:
            self.target_v = 0; self.target_w = 0; self.is_autonomous = False
            self.log("SUCCESS: Destination Reached")
            self.status = "IDLE"
            return

        goal_dx = target[0] - self.x
        goal_dy = target[1] - self.y
        desired_heading = math.atan2(goal_dy, goal_dx)
        heading_error = MathUtils.normalize_angle(desired_heading - self.theta)
        
        self.target_w = np.clip(heading_error * 4.0, -CONFIG.MAX_ANG_VEL, CONFIG.MAX_ANG_VEL)
        turn_penalty = max(0.1, 1.0 - (abs(heading_error) / 1.5))
        self.target_v = np.clip(dist_to_final, 0, CONFIG.MAX_LIN_VEL) * turn_penalty

    def execute_deep_recovery(self):
        """
        Phase 1: Reverse Straight for distance
        Phase 2: J-Turn (Reverse + Turn) to change orientation
        Phase 3: Reset & Force Re-plan
        """
        self.recovery_mode = True
        self.status = "DEEP RECOVERY"
        
        # Phase 1: Deep Reverse (approx 2 sec)
        if self.recovery_step < 40: 
            self.target_v = -1.5 # Fast reverse
            self.target_w = 0.0
        
        # Phase 2: J-Turn (Swing nose away) (approx 1 sec)
        elif self.recovery_step < 60:
            self.target_v = -1.0
            self.target_w = 1.5 # Hard turn while backing
            
        # Phase 3: Finish
        else:
            self.target_v = 0
            self.target_w = 0
            self.recovery_mode = False
            self.recovery_step = 0
            self.stuck_timer = 0
            # CRITICAL: Clear path to force a brand new plan from this new location/angle
            self.current_path = [] 
            self.log("INFO: Repositioned. Calculating new route.")
            return

        self.recovery_step += 1

    def log(self, msg: str):
        t_str = datetime.now().strftime("%H:%M:%S")
        self.logs.append(f"[{t_str}] {msg}")

# --- BACKEND ---
env_service = EnvironmentService()
planner_service = PathPlannerService(env_service)
robot = RobotEntity()

async def simulation_loop():
    while True:
        t_start = time.time()
        
        lidar_data = simulate_lidar()
        if robot.is_autonomous: 
            robot.run_autonomous_control(lidar_data, planner_service, env_service)
        robot.update_physics(CONFIG.TICK_RATE, env_service)
        
        map_updates = list(env_service.new_obstacles)
        env_service.new_obstacles.clear()
        
        packet = SystemPacket(
            robot=RobotStateDTO(
                x=robot.x, y=robot.y, theta=robot.theta, v_lin=robot.v,
                battery=robot.battery,
                status=robot.status,
                is_charging=robot.is_charging, is_autonomous=robot.is_autonomous
            ),
            lidar=lidar_data, path=robot.current_path, logs=list(robot.logs),
            map_updates=map_updates
        )
        await manager.broadcast(packet.model_dump_json())
        robot.logs.clear()
        await asyncio.sleep(max(0, CONFIG.TICK_RATE - (time.time() - t_start)))

def simulate_lidar():
    angles = np.linspace(robot.theta - math.radians(CONFIG.LIDAR_FOV/2),
                         robot.theta + math.radians(CONFIG.LIDAR_FOV/2), CONFIG.LIDAR_RAYS)
    dists = []
    for angle in angles:
        dx, dy = math.cos(angle), math.sin(angle)
        d, step = 0, 0.5
        while d < CONFIG.LIDAR_RANGE:
            d += step
            if env_service.is_occupied(robot.x + dx*d, robot.y + dy*d): break
        dists.append(d)
    return {"angles": angles.tolist(), "distances": dists}

# --- API ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(simulation_loop())
    yield
    task.cancel()

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
manager = type("Manager", (), {"active_connections": [], "broadcast": None})()

class ConnectionManager:
    def __init__(self): self.active_connections = []
    async def connect(self, ws): await ws.accept(); self.active_connections.append(ws)
    def disconnect(self, ws): 
        if ws in self.active_connections: self.active_connections.remove(ws)
    async def broadcast(self, msg):
        for c in self.active_connections[:]:
            try: await c.send_text(msg)
            except: self.disconnect(c)
manager = ConnectionManager()

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True: await ws.receive_text()
    except: manager.disconnect(ws)

@app.post("/api/goal")
async def set_goal(pt: Point):
    path = planner_service.plan((robot.x, robot.y), (pt.x, pt.y))
    if path:
        robot.current_path = path; robot.path_index = 0; robot.is_autonomous = True
        robot.final_goal = (pt.x, pt.y) 
        robot.log(f"CMD: Navigating to ({pt.x:.1f}, {pt.y:.1f})")
    else: robot.log("ERR: Destination Unreachable")

@app.post("/api/dock")
async def auto_dock():
    target = env_service.get_nearest_charger(robot.x, robot.y)
    path = planner_service.plan((robot.x, robot.y), target)
    if path:
        robot.current_path = path; robot.path_index = 0; robot.is_autonomous = True
        robot.final_goal = target
        robot.log("CMD: Initiating Docking Sequence")
    else: robot.log("ERR: Charger Unreachable")

@app.post("/api/manual")
async def manual(v: float, w: float):
    robot.is_autonomous = False; robot.target_v = v; robot.target_w = w

# --- FRONTEND ---
class Sparkline(ctk.CTkCanvas):
    def __init__(self, master, color, **kwargs):
        super().__init__(master, bg=CONFIG.PANEL_COLOR, highlightthickness=0, **kwargs)
        self.color = color; self.data = deque(maxlen=40)
        self.w, self.h = 0, 0
        self.bind("<Configure>", lambda e: setattr(self, 'w', e.width) or setattr(self, 'h', e.height))
    def update_graph(self, val):
        self.data.append(val); self.delete("all")
        if len(self.data) < 2 or self.h == 0: return
        min_v, max_v = min(self.data), max(self.data)
        rng = max_v - min_v if max_v != min_v else 1
        step = self.w / (len(self.data)-1)
        pts = []
        for i, d in enumerate(self.data): pts.extend([i*step, self.h - ((d - min_v)/rng * self.h)])
        self.create_line(pts, fill=self.color, width=2, smooth=True)

class ModernRobotApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title(CONFIG.APP_TITLE)
        self.geometry("1400x900")
        ctk.set_appearance_mode("Dark")
        self.configure(fg_color=CONFIG.BG_COLOR)
        self.setup_ui()
        threading.Thread(target=lambda: uvicorn.run(app, host="127.0.0.1", port=8000, log_level="critical"), daemon=True).start()
        threading.Thread(target=self.ws_client, daemon=True).start()

    def setup_ui(self):
        self.grid_columnconfigure(1, weight=1); self.grid_rowconfigure(0, weight=1)
        pnl_left = ctk.CTkFrame(self, width=280, fg_color=CONFIG.PANEL_COLOR, corner_radius=10)
        pnl_left.grid(row=0, column=0, sticky="ns", padx=10, pady=10); pnl_left.grid_propagate(False)

        ctk.CTkLabel(pnl_left, text="INTELLIBOT", font=("Orbitron", 26, "bold"), text_color=CONFIG.THEME_PRIMARY).pack(pady=(20, 0))
        ctk.CTkLabel(pnl_left, text="NAV STACK V5 (FINAL)", font=("Arial", 10, "bold"), text_color="gray").pack(pady=(0, 20))
        self.lbl_status = ctk.CTkLabel(pnl_left, text="SYSTEM: STANDBY", font=("Consolas", 14, "bold"), text_color="white")
        self.lbl_status.pack(pady=10)
        
        pad = ctk.CTkFrame(pnl_left, fg_color="transparent"); pad.pack(pady=20)
        btn_style = {"width": 60, "height": 60, "font": ("Arial", 24), "fg_color": "#20222E", "hover_color": "#2D3040"}
        ctk.CTkButton(pad, text="▲", command=lambda: self.api_req("/api/manual", v=1.5, w=0), **btn_style).grid(row=0, column=1, padx=4, pady=4)
        ctk.CTkButton(pad, text="◄", command=lambda: self.api_req("/api/manual", v=0, w=1.5), **btn_style).grid(row=1, column=0, padx=4, pady=4)
        ctk.CTkButton(pad, text="■", command=lambda: self.api_req("/api/manual", v=0, w=0), **btn_style, text_color=CONFIG.THEME_DANGER).grid(row=1, column=1, padx=4, pady=4)
        ctk.CTkButton(pad, text="►", command=lambda: self.api_req("/api/manual", v=0, w=-1.5), **btn_style).grid(row=1, column=2, padx=4, pady=4)
        ctk.CTkButton(pad, text="▼", command=lambda: self.api_req("/api/manual", v=-1.5, w=0), **btn_style).grid(row=2, column=1, padx=4, pady=4)

        ctk.CTkButton(pnl_left, text="RETURN TO DOCK", fg_color=CONFIG.THEME_SUCCESS, text_color="black", hover_color="#00CC7A",
                      command=lambda: requests.post("http://127.0.0.1:8000/api/dock")).pack(fill="x", padx=20, pady=10)

        ctk.CTkLabel(pnl_left, text="SYSTEM LOGS", font=("Arial", 11, "bold"), text_color="gray").pack(pady=(20,5), anchor="w", padx=20)
        self.console = ctk.CTkTextbox(pnl_left, height=200, fg_color="#000", text_color=CONFIG.THEME_PRIMARY, font=("Consolas", 11))
        self.console.pack(fill="x", padx=10, pady=5)

        pnl_center = ctk.CTkFrame(self, fg_color="#000", corner_radius=10, border_width=2, border_color="#222")
        pnl_center.grid(row=0, column=1, sticky="nsew", padx=(0,10), pady=10)
        self.map_cv = tkinter.Canvas(pnl_center, bg="#080808", highlightthickness=0)
        self.map_cv.pack(fill="both", expand=True)
        self.map_cv.bind("<Button-1>", self.on_map_click)
        
        self.hud_frame = ctk.CTkFrame(pnl_center, fg_color="transparent"); self.hud_frame.place(relx=0.02, rely=0.02)
        ctk.CTkLabel(self.hud_frame, text="LIVE SENSOR FEED", font=("Arial", 10, "bold"), text_color=CONFIG.THEME_DANGER).pack(anchor="w")
        self.lbl_uptime = ctk.CTkLabel(self.hud_frame, text="T+00:00:00", font=("Consolas", 12), text_color="white")
        self.lbl_uptime.pack(anchor="w")

        pnl_right = ctk.CTkFrame(self, width=250, fg_color=CONFIG.PANEL_COLOR, corner_radius=10)
        pnl_right.grid(row=0, column=2, sticky="ns", padx=(0,10), pady=10); pnl_right.grid_propagate(False)
        ctk.CTkLabel(pnl_right, text="TELEMETRY", font=("Orbitron", 16), text_color="white").pack(pady=20)
        ctk.CTkLabel(pnl_right, text="VELOCITY", font=("Arial", 10, "bold")).pack(anchor="w", padx=20)
        self.graph_vel = Sparkline(pnl_right, color=CONFIG.THEME_PRIMARY, height=60, width=210); self.graph_vel.pack(pady=5, padx=20)
        ctk.CTkLabel(pnl_right, text="BATTERY LEVEL", font=("Arial", 10, "bold")).pack(anchor="w", padx=20, pady=(10,0))
        self.graph_bat = Sparkline(pnl_right, color=CONFIG.THEME_SUCCESS, height=60, width=210); self.graph_bat.pack(pady=5, padx=20)
        ctk.CTkLabel(pnl_right, text="LIDAR RADAR", font=("Arial", 10, "bold")).pack(anchor="w", padx=20, pady=(20,0))
        self.radar_cv = tkinter.Canvas(pnl_right, width=210, height=210, bg="#000", highlightthickness=0); self.radar_cv.pack(pady=10)

    def ws_client(self):
        loop = asyncio.new_event_loop(); asyncio.set_event_loop(loop)
        async def run():
            while True:
                try:
                    async with websockets.connect("ws://127.0.0.1:8000/ws") as ws:
                        while True:
                            self.after(0, self.render, json.loads(await ws.recv()))
                except: time.sleep(1)
        loop.run_until_complete(run())

    def render(self, data):
        r, l, p, logs = data['robot'], data['lidar'], data['path'], data['logs']
        self.lbl_status.configure(text=f"STATUS: {r['status']}", text_color=CONFIG.THEME_DANGER if "RECOVERY" in r['status'] else CONFIG.THEME_PRIMARY)
        self.graph_vel.update_graph(r['v_lin']); self.graph_bat.update_graph(r['battery'])
        if logs:
            self.console.configure(state="normal")
            for log in logs: self.console.insert("end", log + "\n")
            self.console.see("end"); self.console.configure(state="disabled")

        cv = self.map_cv; cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height()
        sx, sy = w / CONFIG.MAP_WIDTH, h / CONFIG.MAP_HEIGHT
        
        for x in range(0, w, int(sx)*2): cv.create_line(x, 0, x, h, fill="#111")
        for y in range(0, h, int(sy)*2): cv.create_line(0, y, w, y, fill="#111")
        
        env = env_service
        for y in range(env.height):
            for x in range(env.width):
                if env.grid[y, x] == 1: cv.create_rectangle(x*sx, y*sy, (x+1)*sx, (y+1)*sy, fill="#333", outline="#444")
        
        for (cx, cy, cw, ch) in CONFIG.CHARGING_STATIONS:
            cv.create_rectangle(cx*sx, cy*sy, (cx+cw)*sx, (cy+ch)*sy, outline=CONFIG.THEME_SUCCESS, fill="#002200", width=2, dash=(2,2))

        if p:
            pts = [(px*sx, py*sy) for px, py in p]
            if len(pts) > 1: cv.create_line(pts, fill=CONFIG.THEME_SECONDARY, width=2, smooth=True)

        rx, ry = r['x']*sx, r['y']*sy
        for i in range(0, len(l['angles']), 4):
            lx = rx + math.cos(l['angles'][i]) * l['distances'][i] * sx
            ly = ry + math.sin(l['angles'][i]) * l['distances'][i] * sx
            col = CONFIG.THEME_DANGER if l['distances'][i] < 4.0 else "#1A2E35"
            cv.create_line(rx, ry, lx, ly, fill=col)
            
        col = CONFIG.THEME_SUCCESS if r['is_charging'] else CONFIG.THEME_PRIMARY
        cv.create_oval(rx-sx*0.4, ry-sx*0.4, rx+sx*0.4, ry+sx*0.4, fill=col, outline="white", width=2)
        
        rcv = self.radar_cv; rcv.delete("all"); cw, ch = 210, 210
        rcv.create_oval(10, 10, cw-10, ch-10, outline="#333")
        for i, d in enumerate(l['distances']):
            if d < CONFIG.LIDAR_RANGE * 0.9:
                a = l['angles'][i] - r['theta'] - math.pi/2
                px = cw/2 + math.cos(a) * d * 8; py = ch/2 + math.sin(a) * d * 8
                rcv.create_rectangle(px, py, px+2, py+2, fill=CONFIG.THEME_DANGER, outline="")

    def api_req(self, ep, **params):
        try: requests.post(f"http://127.0.0.1:8000{ep}", params=params)
        except: pass
    def on_map_click(self, e):
        w, h = self.map_cv.winfo_width(), self.map_cv.winfo_height()
        requests.post("http://127.0.0.1:8000/api/goal", json={"x": (e.x/w)*CONFIG.MAP_WIDTH, "y": (e.y/h)*CONFIG.MAP_HEIGHT})

if __name__ == "__main__":
    ModernRobotApp().mainloop()
