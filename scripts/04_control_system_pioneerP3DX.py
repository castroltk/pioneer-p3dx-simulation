"""
Author: Joaquín Castro Suárez
Date: 05/12/2026
Robot: Pioneer p3dx v.1

⚠️ DEPRECATED: This version uses Tkinter and has been replaced.
Please use 05_control_System_pioneer_v.2.py (DearPyGui) for better performance.

OBJECTIVE:

    

REQUIREMENTS:
    1. CoppeliaSim open.
    2. The simulation does NOT need to be running yet.
    3. pip install matplotlib
    4. pip install coppeliasim-zmqremoteapi-client


Estructura
    class PIDController:
        ...

    class Odometria:
        ...

    class Robot:
        ...

    class SonarSensor:
        ...

    def compute_setpoints(left_dist, right_dist):   # ← aquí, fuera de todo
        ...

    class Dashboard:
        ...

"""
#========================================================================
import sys
import math
import time
import threading
import tkinter as tk
from collections import deque
import matplotlib 
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

#=======================================================================

#=======================================================================
#Conection to copeliaSim
#=======================================================================

try:
    #
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require("sim")
    SIM_MODE = False
    print("[OK] connected to coppeliaSim")
    sim.startSimulation()
    print("[OK] simulation started")
    

except Exception as e:
    SIM_MODE = True
    sim      = None
    del client
    print(f"[WARNING] simulation mode — no robot: {e}")
    print(f"Error searching for object: {e}")
    print("Possible causes:")
    print("- The object name in the scene is different.")
    print("- The scene is not loaded in CoppeliaSim.")
    print("- The robot is not in the scene's root.")
    print("Check the 'Scene Hierarchy' panel in CoppeliaSim")
    print("and adjust the paths in the code.")


#=======================================================================
#robot physical parameters - pioneer P3DX
#=======================================================================

wheel_radius    = 0.0975    #meters --> wheel radius
axle_length     = 0.330     # meters — estimated from official body width 381mm total minus, wheel offset from chassis edge.
Digital_twin    = 0.5      #seconds per cycly

# PID tuning
kp = 0.5
ki = 0.0
kd = 0.0

# target speed
setpoint = 0.5   # m/s

# obstacle thresholds
safe_distance     = 1.0   # meters
Warning_distance  = 0.5
cristical_distance= 0.2

# motor health
# motor health monitoring thresholds
# based on Pioneer P3-DX official max speed: 1.2 m/s
#0.0975 m --> wheel radius
max_effort      = 2   # rad/s:- —> derived from (1.2 m/s) / (0.0975 m) wheel radius
effort_window   = 20
alert_threshold = 0.85

# chart history
max_point = 300

# sonar layout — Pioneer P3-DX indexes start at 0
FRONT_SONARS = [3, 4, 5, 6, 7]
LEFT_SONARS  = [0, 1, 2, 3]
RIGHT_SONARS = [7, 8, 9, 10]
#=======================================================================

#=======================================================================
class PIDController:
    def __init__(self, kp, ki, kd, name="motor"):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.name       = name
        self.prev_error = 0.0
        self.integral   = 0.0 #save error over time
        self.alert_active   = False

        #________________________________________________________________________________
        # Stores a sliding window of recent effort values to monitor performance 
        # or detect stalls. Automatically discards the oldest entry when EFFORT_WINDOW 
        # is exceeded, ensuring O(1) efficiency and fixed memory usage.
        self.effort_history = deque(
        maxlen=effort_window) #Double-Ended Queue
        #________________________________________________________________________________
        
#________________________________________________________________________________
    def compute(self, setpoint, measured, dt):
        # if setpoint is zero, reset and return 0
        # prevents oscillation around zero
        if abs(setpoint) < 0.01:
            self.integral   = 0.0
            self.prev_error = 0.0
            return 0.0


        error          = setpoint - measured
        self.integral += error * dt

        # anti-windup — prevent overshooting on sudden setpoint change
        max_integral  = 1.0

        # Anti-windup: Clamps integral term to prevent excessive error accumulation.
        # Format: math.clamp(variable, min_limit, max_limit)
        self.integral = max(-1, min(1, self.integral))

        # Standard discrete derivative: calculates the rate of change of the error.
        # Formula: (current_error - previous_error) / time_step
        # Calculates the rate of change between the current error (k) 
        # and the previous error (k-1) over the time step (dt).
        # Formula: derivative = (e[k] - e[k-1]) / dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error

        # PID Control Law: Sums the Proportional, Integral, and Derivative terms
        # to calculate the final control output.
        output = ((self.kp * error) +
                (self.ki * self.integral) +
                (self.kd * derivative))

        
        output = max(-12.3, min(12.3, output))
        self._monitor_health(output)
        return output
#________________________________________________________________________________
    def _monitor_health(self, output):
        effort = abs(output) / max_effort
        self.effort_history.append(effort)

        
        #only runs when self.effort_history = deque(maxlen=effort_window) is full (20 samples)
        if len(self.effort_history) == effort_window:
            # health alert: on above 85%, off below 59.5% — hysteresis prevents flickering
            avg = sum(self.effort_history) / effort_window
            if avg > alert_threshold and not self.alert_active:
                self.alert_active = True
                print(f"[ALERT] {self.name}: high effort ({avg*100:.1f}%) — possible wear")
            elif avg < alert_threshold * 0.7:   #alert_threshold * 0.7 = 0.85 * 0.7 = 0.595 = 59.5%
                self.alert_active = False

    # returns 0.0 if no data yet, otherwise current effort average as percentage
    def get_health(self):
        if not self.effort_history:
            return 0.0
        return sum(self.effort_history) / len(self.effort_history) * 100


#=======================================================================

#=======================================================================
class Odometria:
    def __init__(self):
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.dist  = 0.0

    def actualizar(self, v_izq, v_der, dt):
        v = (v_der + v_izq) / 2
        w = (v_der - v_izq) / axle_length

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # To normalize ANY angle (like 450), you MUST pass its y_coordinate = sine(A) and x_coordinate = cosine(A).
        # Format: math.atan2(y_coordinate, x_coordinate)
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Cumulative distance traveled based on linear velocity (v).
        self.dist  += abs(v) * dt
        return v, w

    def reset(self):
        self.x = self.y = self.theta = self.dist = 0.0


#=======================================================================

#=======================================================================
class Robot:
    def __init__(self):
        self.connected = False
        if not SIM_MODE:
            try:
                self.motor_left  = sim.getObject('/PioneerP3DX/leftMotor')
                self.motor_right = sim.getObject('/PioneerP3DX/rightMotor')
                self.connected   = True
                print("[OK] motors found")
            except Exception as e:
                print(f"[ERROR] motors not found: {e}")

    def set_velocidades(self, vel_left, vel_right):
        # receives rad/s
        if self.connected:
            sim.setJointTargetVelocity(self.motor_left,  vel_left)
            sim.setJointTargetVelocity(self.motor_right, vel_right)

    def get_velocidades(self):
        # returns m/s — None if not connected
        if self.connected:
            v_left  = sim.getJointVelocity(self.motor_left)  * wheel_radius
            v_right = sim.getJointVelocity(self.motor_right) * wheel_radius
            return v_left, v_right
        return None, None

    def stop(self):
        self.set_velocidades(0, 0)


#=======================================================================

#=======================================================================
class SonarSensor:
    def __init__(self):
        self.handles = []
        if not SIM_MODE:
            for i in range(0, 16):
                handle = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
                self.handles.append(handle)

    def read_sides(self):
        if SIM_MODE:
            return safe_distance, safe_distance

        left_min  = float('inf')
        right_min = float('inf')

        for i, handle in enumerate(self.handles):
            # Read proximity sensor: 'result' indicates detection, 'distance' is the range.
            # The underscores (_) ignore the 3D point, object handle, and surface normal.
            result, distance, _, _, _ = sim.readProximitySensor(handle)
            if result:
                if i in LEFT_SONARS:
                    left_min  = min(left_min,  distance)
                else:
                    right_min = min(right_min, distance)

        return left_min, right_min

#=======================================================================

#=======================================================================
def compute_setpoints(left_dist, right_dist):
    front_dist = min(left_dist, right_dist)

    if front_dist > safe_distance:
        return setpoint, setpoint

    elif front_dist > Warning_distance:
        if left_dist > right_dist:
            return setpoint * 0.3, setpoint
        else:
            return setpoint, setpoint * 0.3

    elif front_dist > cristical_distance:
        if left_dist > right_dist:
            return -setpoint * 0.5, setpoint
        else:
            return setpoint, -setpoint * 0.5
    else:
        return 0.0, 0.0

#=======================================================================

#=======================================================================

class Dashboard:
    def __init__(self, root):
        self.root      = root
        self.odom      = Odometria()
        self.robot     = Robot()
        self.sonar     = SonarSensor()
        self.pid_left  = PIDController(kp, ki, kd, name="left motor")
        self.pid_right = PIDController(kp, ki, kd, name="right motor")
        self.active    = False
        self.t_start   = time.time()
        self.arrow_robot = None

        # data history for every chart
        self.hist = {
            'time'        : deque(maxlen=max_point),
            'v'           : deque(maxlen=max_point),
            'w'           : deque(maxlen=max_point),
            'theta'       : deque(maxlen=max_point),
            'sp_left'     : deque(maxlen=max_point),
            'v_left'      : deque(maxlen=max_point),
            'v_right'     : deque(maxlen=max_point),
            'health_left' : deque(maxlen=max_point),
            'health_right': deque(maxlen=max_point),
            'sonar_left'  : deque(maxlen=max_point),
            'sonar_right' : deque(maxlen=max_point),
            'x'           : deque(maxlen=max_point),
            'y'           : deque(maxlen=max_point),
        }

        self._build_ui()
        self._start_loop()

    # ─────────────────────────────────────────
    def _build_ui(self):
        self.root.title("Pioneer P3-DX — Control Dashboard")
        self.root.configure(bg="#1e1e2e")

        BG     = "#1e1e2e"
        CARD   = "#2a2a3e"
        ACCENT = "#1D9E75"
        TEXT   = "#cdd6f4"
        MUTED  = "#6c7086"
        DANGER = "#f38ba8"
        AMBER  = "#fab387"
        BLUE   = "#89b4fa"
        
    

        # ── top bar ──────────────────────────────────
        top = tk.Frame(self.root, bg=BG, padx=12, pady=8)
        top.pack(fill="x")

        tk.Label(top, text="Pioneer P3-DX",
                 font=("Courier New", 14, "bold"),
                 bg=BG, fg=ACCENT).pack(side="left")

        mode  = "● ROBOT REAL" if self.robot.connected else "● SIMULATION MODE"
        color = ACCENT if self.robot.connected else AMBER
        tk.Label(top, text=mode, font=("Courier New", 10),
                 bg=BG, fg=color).pack(side="left", padx=16)

        self.lbl_time = tk.Label(top, text="t = 0.00 s",
                                  font=("Courier New", 10), bg=BG, fg=MUTED)
        self.lbl_time.pack(side="right")

        # ── metrics row ──────────────────────────────
        mf = tk.Frame(self.root, bg=BG, padx=12)
        mf.pack(fill="x")

        metrics = [
            ("linear v",  "lbl_v",     "m/s",  ACCENT),
            ("angular ω", "lbl_w",     "rad/s", ACCENT),
            ("position x","lbl_x",     "m",     BLUE),
            ("position y","lbl_y",     "m",     BLUE),
            ("heading θ", "lbl_theta", "°",     AMBER),
            ("distance",  "lbl_dist",  "m",     AMBER),
            ("health L",  "lbl_hl",    "%",     ACCENT),
            ("health R",  "lbl_hr",    "%",     ACCENT),
        ]

        for label, attr, unit, color in metrics:
            card = tk.Frame(mf, bg=CARD, padx=8, pady=6)
            card.pack(side="left", padx=3, pady=6, fill="x", expand=True)
            tk.Label(card, text=label, font=("Courier New", 8),
                     bg=CARD, fg=MUTED).pack()
            lbl = tk.Label(card, text="0.00",
                           font=("Courier New", 12, "bold"),
                           bg=CARD, fg=color)
            lbl.pack()
            tk.Label(card, text=unit, font=("Courier New", 8),
                     bg=CARD, fg=MUTED).pack()
            setattr(self, attr, lbl)

        # ── main area ────────────────────────────────
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        # left column — controls
        col_left = tk.Frame(main, bg=BG, width=280)
        col_left.pack(side="left", fill="y", padx=(0, 8))
        col_left.pack_propagate(False)

        # sliders
        ctrl = tk.Frame(col_left, bg=CARD, padx=10, pady=10)
        ctrl.pack(fill="x", pady=(0, 6))
        tk.Label(ctrl, text="MOTOR CONTROL",
                 font=("Courier New", 9), bg=CARD, fg=MUTED).pack(anchor="w")

        self.var_left  = tk.DoubleVar(value=0.0)
        self.var_right = tk.DoubleVar(value=0.0)
        self.var_scale = tk.DoubleVar(value=1.0)

        def make_slider(parent, text, var, color):
            f = tk.Frame(parent, bg=CARD)
            f.pack(fill="x", pady=3)
            tk.Label(f, text=text, width=9, anchor="w",
                     bg=CARD, fg=TEXT, font=("Courier New", 9)).pack(side="left")
            tk.Scale(f, from_=-3.0, to=3.0, resolution=0.1,
                     orient="horizontal", variable=var,
                     bg=CARD, fg=color, troughcolor="#3a3a5e",
                     highlightthickness=0, sliderrelief="flat",
                     font=("Courier New", 8), length=130,
                     command=lambda e: self._apply_velocities()
                     ).pack(side="left")
            lbl = tk.Label(f, text="0.0", width=5, anchor="e",
                           bg=CARD, fg=color, font=("Courier New", 9, "bold"))
            lbl.pack(side="left")
            return lbl

        self.lbl_sl = make_slider(ctrl, "◀ left",  self.var_left,  ACCENT)
        self.lbl_sr = make_slider(ctrl, "▶ right", self.var_right, AMBER)

        f_scale = tk.Frame(ctrl, bg=CARD)
        f_scale.pack(fill="x", pady=3)
        tk.Label(f_scale, text="scale", width=9, anchor="w",
                 bg=CARD, fg=MUTED, font=("Courier New", 9)).pack(side="left")
        tk.Scale(f_scale, from_=0.1, to=3.0, resolution=0.1,
                 orient="horizontal", variable=self.var_scale,
                 bg=CARD, fg=MUTED, troughcolor="#3a3a5e",
                 highlightthickness=0, sliderrelief="flat",
                 font=("Courier New", 8), length=130
                 ).pack(side="left")

        # presets
        pre = tk.Frame(col_left, bg=CARD, padx=10, pady=8)
        pre.pack(fill="x", pady=(0, 6))
        tk.Label(pre, text="PRESETS",
                 font=("Courier New", 9), bg=CARD, fg=MUTED).pack(anchor="w", pady=(0, 4))

        presets = [
            ("▲ forward",  1.5,  1.5), ("▼ reverse", -1.5, -1.5),
            ("↻ turn R",   1.5, -1.5), ("↺ turn L",  -1.5,  1.5),
            ("⟳ spin R",   2.0, -2.0), ("⟲ spin L",  -2.0,  2.0),
        ]

        row1 = tk.Frame(pre, bg=CARD); row1.pack()
        row2 = tk.Frame(pre, bg=CARD); row2.pack(pady=3)

        def make_btn(parent, text, l, r):
            def cmd():
                self.var_left.set(l); self.var_right.set(r)
                self._apply_velocities()
            tk.Button(parent, text=text, command=cmd,
                      bg="#3a3a5e", fg=TEXT, relief="flat",
                      padx=6, pady=3, font=("Courier New", 8),
                      activebackground="#4a4a6e"
                      ).pack(side="left", padx=2)

        for t, l, r in presets[:3]: make_btn(row1, t, l, r)
        for t, l, r in presets[3:]: make_btn(row2, t, l, r)

        tk.Button(pre, text="■ STOP", command=self._stop,
                  bg=DANGER, fg="#1e1e2e", relief="flat",
                  padx=12, pady=5, font=("Courier New", 9, "bold")
                  ).pack(fill="x", pady=(6, 0))

        # sonar display
        sf = tk.Frame(col_left, bg=CARD, padx=10, pady=8)
        sf.pack(fill="x", pady=(0, 6))
        tk.Label(sf, text="SONAR DISTANCES",
                 font=("Courier New", 9), bg=CARD, fg=MUTED).pack(anchor="w")
        row = tk.Frame(sf, bg=CARD); row.pack(fill="x", pady=4)
        tk.Label(row, text="left",  bg=CARD, fg=MUTED,
                 font=("Courier New", 8)).pack(side="left")
        self.lbl_sonar_l = tk.Label(row, text="---", bg=CARD, fg=BLUE,
                                     font=("Courier New", 11, "bold"))
        self.lbl_sonar_l.pack(side="left", padx=6)
        tk.Label(row, text="right", bg=CARD, fg=MUTED,
                 font=("Courier New", 8)).pack(side="left", padx=(16, 0))
        self.lbl_sonar_r = tk.Label(row, text="---", bg=CARD, fg=BLUE,
                                     font=("Courier New", 11, "bold"))
        self.lbl_sonar_r.pack(side="left", padx=6)

        # obstacle avoidance toggle
        self.avoid_var = tk.BooleanVar(value=False)
        av = tk.Frame(col_left, bg=CARD, padx=10, pady=8)
        av.pack(fill="x", pady=(0, 6))
        tk.Label(av, text="OBSTACLE AVOIDANCE",
                 font=("Courier New", 9), bg=CARD, fg=MUTED).pack(anchor="w")
        tk.Checkbutton(av, text="enable auto-avoidance",
                       variable=self.avoid_var,
                       bg=CARD, fg=TEXT, selectcolor="#3a3a5e",
                       font=("Courier New", 9),
                       activebackground=CARD).pack(anchor="w", pady=4)

        # generated code display
        cf = tk.Frame(col_left, bg=CARD, padx=10, pady=8)
        cf.pack(fill="x", pady=(0, 6))
        tk.Label(cf, text="COPPELIASIM CODE",
                 font=("Courier New", 9), bg=CARD, fg=MUTED).pack(anchor="w")
        self.lbl_code = tk.Label(cf,
                                  text="setJointTargetVelocity(left,  0.0)\nsetJointTargetVelocity(right, 0.0)",
                                  font=("Courier New", 8), bg=CARD, fg=ACCENT,
                                  justify="left")
        self.lbl_code.pack(anchor="w", pady=4)

        tk.Button(col_left, text="↺ reset odometry", command=self._reset,
                  bg="#3a3a5e", fg=MUTED, relief="flat",
                  font=("Courier New", 8)).pack(fill="x", pady=(0, 4))

        # right column — matplotlib charts
        col_right = tk.Frame(main, bg=BG)
        col_right.pack(side="left", fill="both", expand=True)

        fig = plt.figure(figsize=(10, 7), facecolor="#1e1e2e")
        gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.4)

        # gridspec layout:
        # gs[0:2, 0] = rows 0 and 1, column 0 → tall trajectory plot
        # gs[0, 1]   = row 0, column 1        → linear velocity
        # gs[0, 2]   = row 0, column 2        → angular velocity
        # etc.
        self.ax_traj   = fig.add_subplot(gs[0:2, 0])
        self.ax_v      = fig.add_subplot(gs[0, 1])
        self.ax_w      = fig.add_subplot(gs[0, 2])
        self.ax_theta  = fig.add_subplot(gs[1, 1])
        self.ax_sonar  = fig.add_subplot(gs[1, 2])
        self.ax_wheels = fig.add_subplot(gs[2, 0])
        self.ax_health = fig.add_subplot(gs[2, 1])
        self.ax_pid    = fig.add_subplot(gs[2, 2])

        chart_cfg = [
            (self.ax_traj,   "XY trajectory",         "x (m)",    "y (m)"),
            (self.ax_v,      "linear velocity v(t)",  "time (s)", "m/s"),
            (self.ax_w,      "angular velocity w(t)", "time (s)", "rad/s"),
            (self.ax_theta,  "heading theta(t)",      "time (s)", "degrees"),
            (self.ax_sonar,  "sonar distances",       "time (s)", "meters"),
            (self.ax_wheels, "wheel velocities",      "time (s)", "m/s"),
            (self.ax_health, "motor health",          "time (s)", "%"),
            (self.ax_pid,    "setpoint vs measured",  "time (s)", "m/s"),
        ]

        for ax, title, xlabel, ylabel in chart_cfg:
            ax.set_facecolor("#2a2a3e")
            ax.set_title(title, fontsize=8, color="#cdd6f4", pad=4)
            ax.set_xlabel(xlabel, fontsize=7, color="#6c7086")
            ax.set_ylabel(ylabel, fontsize=7, color="#6c7086")
            ax.tick_params(colors="#6c7086", labelsize=6)
            ax.grid(True, color="#3a3a5e", linewidth=0.4)
            for spine in ax.spines.values():
                spine.set_edgecolor("#3a3a5e")

        self.ax_traj.set_aspect("equal")

        # initialize chart lines
        self.line_traj,      = self.ax_traj.plot([], [], color="#1D9E75", lw=1.5)
        self.dot_robot,      = self.ax_traj.plot([], [], "o", color="#fab387", ms=8)
        self.line_v,         = self.ax_v.plot([], [], color="#1D9E75", lw=1.2)
        self.line_w,         = self.ax_w.plot([], [], color="#fab387", lw=1.2)
        self.line_theta,     = self.ax_theta.plot([], [], color="#f38ba8", lw=1.2)
        self.line_sonar_l,   = self.ax_sonar.plot([], [], color="#89b4fa", lw=1.2, label="left")
        self.line_sonar_r,   = self.ax_sonar.plot([], [], color="#fab387", lw=1.2, label="right")
        self.line_wheel_l,   = self.ax_wheels.plot([], [], color="#1D9E75", lw=1.2, label="left")
        self.line_wheel_r,   = self.ax_wheels.plot([], [], color="#fab387", lw=1.2, label="right")
        self.line_health_l,  = self.ax_health.plot([], [], color="#1D9E75", lw=1.2, label="left")
        self.line_health_r,  = self.ax_health.plot([], [], color="#f38ba8", lw=1.2, label="right")
        self.line_sp,        = self.ax_pid.plot([], [], color="#1D9E75", lw=1.0, linestyle="--", label="setpoint")
        self.line_meas,      = self.ax_pid.plot([], [], color="#89b4fa", lw=1.2, label="measured")

        for ax in [self.ax_sonar, self.ax_wheels, self.ax_health, self.ax_pid]:
            ax.legend(fontsize=6, facecolor="#2a2a3e",
                      edgecolor="#3a3a5e", labelcolor="#cdd6f4")

        self.canvas_mpl = FigureCanvasTkAgg(fig, master=col_right)
        self.canvas_mpl.get_tk_widget().pack(fill="both", expand=True)

    # ─────────────────────────────────────────
    def _apply_velocities(self):
        scale = self.var_scale.get()
        l = self.var_left.get()
        r = self.var_right.get()
        self.lbl_sl.config(text=f"{l:+.1f}")
        self.lbl_sr.config(text=f"{r:+.1f}")
        self.robot.set_velocidades(l * scale, r * scale)
        self.lbl_code.config(
            text=f"setJointTargetVelocity(left,  {l*scale:.2f})\n"
                 f"setJointTargetVelocity(right, {r*scale:.2f})")

    def _stop(self):
        self.var_left.set(0.0)
        self.var_right.set(0.0)
        self._apply_velocities()
        self.robot.stop()

    def _reset(self):
        self.odom.reset()
        for key in self.hist:
            self.hist[key].clear()
        self.t_start = time.time()

    # ─────────────────────────────────────────
    def _start_loop(self):
        self.active = True
        # data loop runs in separate thread so UI never freezes
        threading.Thread(target=self._data_loop, daemon=True).start()
        self._update_ui()

    def _data_loop(self):
        # thread 1 — reads sensors, computes PID, updates odometry
        while self.active:
            t = time.time() - self.t_start

            # 1. read sonars
            sonar_l, sonar_r = self.sonar.read_sides()

            # 2. decide setpoints
            if self.avoid_var.get():
                sp_left, sp_right = compute_setpoints(sonar_l, sonar_r)
            else:
                scale    = self.var_scale.get()
                sp_left  = self.var_left.get()  * scale * wheel_radius
                sp_right = self.var_right.get() * scale * wheel_radius

            # 3. read wheel velocities
            v_left, v_right = self.robot.get_velocidades()
            if v_left is None:
                v_left  = sp_left  * 0.85
                v_right = sp_right * 0.90

            # 4. PID correction for each wheel independently
            corr_l = self.pid_left.compute(sp_left,  v_left,  Digital_twin)
            corr_r = self.pid_right.compute(sp_right, v_right, Digital_twin)

            # 5. convert m/s → rad/s and send
            self.robot.set_velocidades(corr_l / wheel_radius,
                                       corr_r / wheel_radius)

            # 6. update odometry
            v, w = self.odom.actualizar(v_left, v_right, Digital_twin)

            # 7. store history for charts
            self.hist['time'].append(t)
            self.hist['v'].append(v)
            self.hist['w'].append(w)
            self.hist['theta'].append(math.degrees(self.odom.theta))
            self.hist['sp_left'].append(sp_left)
            self.hist['v_left'].append(v_left)
            self.hist['v_right'].append(v_right)
            self.hist['health_left'].append(self.pid_left.get_health())
            self.hist['health_right'].append(self.pid_right.get_health())
            self.hist['sonar_left'].append(sonar_l)
            self.hist['sonar_right'].append(sonar_r)
            self.hist['x'].append(self.odom.x)
            self.hist['y'].append(self.odom.y)

            time.sleep(Digital_twin)

    def _update_ui(self):
        # thread 2 — reads history and refreshes UI every 100ms
        if not self.active:
            return

        t  = list(self.hist['time'])
        xs = list(self.hist['x'])
        ys = list(self.hist['y'])

        # update metric labels
        if self.hist['v']:
            self.lbl_v.config(text=f"{self.hist['v'][-1]:+.3f}")
            self.lbl_w.config(text=f"{self.hist['w'][-1]:+.3f}")
            self.lbl_theta.config(text=f"{self.hist['theta'][-1]:+.1f}")
            self.lbl_sonar_l.config(text=f"{self.hist['sonar_left'][-1]:.2f} m")
            self.lbl_sonar_r.config(text=f"{self.hist['sonar_right'][-1]:.2f} m")
            self.lbl_hl.config(text=f"{self.hist['health_left'][-1]:.1f}")
            self.lbl_hr.config(text=f"{self.hist['health_right'][-1]:.1f}")

        self.lbl_x.config(text=f"{self.odom.x:+.3f}")
        self.lbl_y.config(text=f"{self.odom.y:+.3f}")
        self.lbl_dist.config(text=f"{self.odom.dist:.3f}")
        self.lbl_time.config(text=f"t = {time.time()-self.t_start:.2f} s")

        if len(t) > 1:
            # trajectory + heading arrow
            self.line_traj.set_data(xs, ys)
            self.dot_robot.set_data([xs[-1]], [ys[-1]])

            if self.arrow_robot:
                self.arrow_robot.remove()
            theta = self.odom.theta
            dx = 0.15 * math.cos(theta)
            dy = 0.15 * math.sin(theta)
            self.arrow_robot = self.ax_traj.annotate(
                "", xy=(xs[-1]+dx, ys[-1]+dy),
                xytext=(xs[-1], ys[-1]),
                arrowprops=dict(arrowstyle="->", color="#fab387", lw=1.5))
            self.ax_traj.relim()
            self.ax_traj.autoscale_view()

            # all time-series charts
            pairs = [
                (self.line_v,       'v'),
                (self.line_w,       'w'),
                (self.line_theta,   'theta'),
                (self.line_sonar_l, 'sonar_left'),
                (self.line_sonar_r, 'sonar_right'),
                (self.line_wheel_l, 'v_left'),
                (self.line_wheel_r, 'v_right'),
                (self.line_health_l,'health_left'),
                (self.line_health_r,'health_right'),
                (self.line_sp,      'sp_left'),
                (self.line_meas,    'v_left'),
            ]
            for line, key in pairs:
                line.set_data(t, list(self.hist[key]))

            for ax in [self.ax_v, self.ax_w, self.ax_theta, self.ax_sonar,
                       self.ax_wheels, self.ax_health, self.ax_pid]:
                ax.relim()
                ax.autoscale_view()

            self.canvas_mpl.draw_idle()

        self.root.after(200, self._update_ui)

    def close(self):
        self.active = False
        self.robot.stop()

        if not SIM_MODE and sim:
            try:
                sim.stopSimulation()
                print("[OK] simulation stopped")
            except Exception as e:
                print(f"[WARNING] could not stop simulation: {e}")
        
        self.root.destroy()

#=======================================================================
#-------------------main--------------------------------
#=======================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app  = Dashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()





    
