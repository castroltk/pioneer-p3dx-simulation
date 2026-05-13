"""
Author: Joaquín Castro Suárez
Date: 05/12/2026
Robot: Pioneer p3dx v.2

Description: Optimized version of the PID controller GUI.
            Migrated from Tkinter to DearPyGui (DPG) for
            better performance, lower resource usage and
            real-time rendering capabilities.
            Replaces: 04_pid_controller.py

OBJECTIVE:
    Control and monitor the Pioneer P3-DX mobile robot through a real-time
    dashboard. The system integrates PID motor control, odometry, and sonar-based
    obstacle avoidance to analyze robot behavior during navigation.

    This project is under active development — future modules will expand
    localization, mapping, and autonomous navigation capabilities.

REQUIREMENTS:
    1. CoppeliaSim open.
    2. The simulation does NOT need to be running yet.
    3. pip install dearpygui
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

    def compute_setpoints(left_dist, right_dist):
        ...

    class Dashboard:
        ...

"""
#========================================================================
import math
import time
import threading
from collections import deque
import dearpygui.dearpygui as dpg
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

#=======================================================================
# connection to CoppeliaSim
#=======================================================================
try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim    = client.require("sim")
    SIM_MODE = False
    print("[OK] connected to coppeliaSim")
    sim.startSimulation()
    print("[OK] simulation started")

except Exception as e:
    SIM_MODE = True
    sim      = None
    print(f"[WARNING] simulation mode — no robot: {e}")
    print("Possible causes:")
    print("- The object name in the scene is different.")
    print("- The scene is not loaded in CoppeliaSim.")
    print("- The robot is not in the scene's root.")

#=======================================================================
# robot physical parameters - pioneer P3DX
#=======================================================================
wheel_radius     = 0.0975   # meters --> wheel radius
axle_length      = 0.330    # meters — estimated from official body width 381mm total minus wheel offset from chassis edge.
Digital_twin     = 0.1      # seconds per cycle

# PID tuning — Pioneer P3-DX validated values
# tested in CoppeliaSim 4.10 — Digital_twin = 0.1s — setpoint = 0.5 m/s
# kp: base response | ki: steady-state correction | kd: oscillation damping
kp = 0.9    # stable forward motion without oscillation
ki = 0.001  # minimal integral — avoids windup from wheel slip
kd = 0.0001 # light derivative — smooths without bouncing

# target speed
setpoint = 0.5   # m/s

# obstacle thresholds
safe_distance      = 1.0   # meters
Warning_distance   = 0.5
cristical_distance = 0.2

# motor health monitoring thresholds
# based on Pioneer P3-DX official max speed: 1.2 m/s
# 0.0975 m --> wheel radius
max_effort      = 12.3  # rad/s — derived from (1.2 m/s) / (0.0975 m) wheel radius
effort_window   = 20    # number of samples in sliding window (20 × 0.1s = 2 seconds)
alert_threshold = 0.85  # 85% of max effort triggers alert

# chart history
max_point = 300

# sonar layout — Pioneer P3-DX indexes start at 0
FRONT_SONARS = [3, 4, 5, 6, 7]
LEFT_SONARS  = [0, 1, 2, 3]
RIGHT_SONARS = [7, 8, 9, 10]

sim_lock = threading.Lock()

#=======================================================================
class PIDController:
    def __init__(self, kp, ki, kd, name="motor"):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.name       = name
        self.prev_error = 0.0
        self.integral   = 0.0  # save error over time
        self.alert_active = False

        #________________________________________________________________________________
        # Stores a sliding window of recent effort values to monitor performance
        # or detect stalls. Automatically discards the oldest entry when EFFORT_WINDOW
        # is exceeded, ensuring O(1) efficiency and fixed memory usage.
        self.effort_history = deque(maxlen=effort_window)  # Double-Ended Queue
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

        # Anti-windup: Clamps integral term to prevent excessive error accumulation.
        self.integral = max(-1, min(1, self.integral))

        # Standard discrete derivative: calculates the rate of change of the error.
        # Formula: derivative = (e[k] - e[k-1]) / dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID Control Law
        output = ((self.kp * error) +
                  (self.ki * self.integral) +
                  (self.kd * derivative))

        # clamp output to motor physical limits
        output = max(-12.3, min(12.3, output))

        self._monitor_health(output)
        return output

    def _monitor_health(self, output):
        effort = abs(output) / max_effort
        self.effort_history.append(effort)

        # only runs when deque is full (20 samples)
        if len(self.effort_history) == effort_window:
            # health alert: on above 85%, off below 59.5% — hysteresis prevents flickering
            avg = sum(self.effort_history) / effort_window
            if avg > alert_threshold and not self.alert_active:
                self.alert_active = True
                print(f"[ALERT] {self.name}: high effort ({avg*100:.1f}%) — possible wear")
            elif avg < alert_threshold * 0.7:
                self.alert_active = False

    def get_health(self):
        # returns 0.0 if no data yet, otherwise current effort average as percentage
        if not self.effort_history:
            return 0.0
        return sum(self.effort_history) / len(self.effort_history) * 100

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

        # To normalize ANY angle (like 450), you MUST pass its
        # y_coordinate = sine(A) and x_coordinate = cosine(A).
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Cumulative distance traveled based on linear velocity (v).
        self.dist  += abs(v) * dt
        return v, w

    def reset(self):
        self.x = self.y = self.theta = self.dist = 0.0

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
            try:
                with sim_lock:
                    sim.setJointTargetVelocity(self.motor_left,  vel_left)
                    sim.setJointTargetVelocity(self.motor_right, vel_right)
            except Exception as e:
                print(f"[WARNING] lost connection to CoppeliaSim: {e}")
                self.connected = False

    def get_velocidades(self):
        # returns m/s — None if not connected
        if self.connected:
            try:
                with sim_lock:
                    v_left  = sim.getJointVelocity(self.motor_left)  * wheel_radius
                    v_right = sim.getJointVelocity(self.motor_right) * wheel_radius
                return v_left, v_right
            except Exception as e:
                print(f"[WARNING] lost connection to CoppeliaSim: {e}")
                self.connected = False
        return None, None

    def stop(self):
        self.set_velocidades(0, 0)

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
            try:
                with sim_lock:    # ← agregar aquí
                    result, distance, _, _, _ = sim.readProximitySensor(handle)
                if result:
                    if i in LEFT_SONARS:
                        left_min  = min(left_min,  distance)
                    else:
                        right_min = min(right_min, distance)
            except Exception as e:
                print(f"[WARNING] sonar read error: {e}")
                return safe_distance, safe_distance

        return left_min, right_min

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
class Dashboard:
    def __init__(self):
        self.odom      = Odometria()
        self.robot     = Robot()
        self.sonar     = SonarSensor()
        self.pid_left  = PIDController(kp, ki, kd, name="left motor")
        self.pid_right = PIDController(kp, ki, kd, name="right motor")
        self.active    = False
        self.t_start   = time.time()
        self.avoid     = False

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

    #=======================================================================
    def _build_ui(self):
        dpg.create_context()

        # ── theme ──────────────────────────────────────────────────────
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg,      (30,  30,  46))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg,       (42,  42,  62))
                dpg.add_theme_color(dpg.mvThemeCol_Button,        (58,  58,  94))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (74,  74,  110))
                dpg.add_theme_color(dpg.mvThemeCol_SliderGrab,    (29,  158, 117))
                dpg.add_theme_color(dpg.mvThemeCol_Text,          (205, 214, 244))
                dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (29,  158, 117))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 6)

        # ── main window ────────────────────────────────────────────────
        with dpg.window(tag="main", no_title_bar=True, no_scrollbar=True):

            # top bar
            with dpg.group(horizontal=True):
                dpg.add_text("Pioneer P3-DX", color=(29, 158, 117))
                mode  = "● ROBOT REAL" if self.robot.connected else "● SIMULATION MODE"
                color = (29, 158, 117) if self.robot.connected else (250, 179, 135)
                dpg.add_text(mode, color=color)
                dpg.add_text("", tag="lbl_time")

            dpg.add_separator()

            # ── metrics row ────────────────────────────────────────────
            with dpg.group(horizontal=True):
                for tag, label, unit, color in [
                    ("m_v",     "linear v",   "m/s",   (29,  158, 117)),
                    ("m_w",     "angular w",  "rad/s", (29,  158, 117)),
                    ("m_x",     "position x", "m",     (137, 180, 250)),
                    ("m_y",     "position y", "m",     (137, 180, 250)),
                    ("m_theta", "heading theta",  "°",     (250, 179, 135)),
                    ("m_dist",  "distance",   "m",     (250, 179, 135)),
                    ("m_hl",    "health L",   "%",     (29,  158, 117)),
                    ("m_hr",    "health R",   "%",     (29,  158, 117)),
                ]:
                    with dpg.child_window(width=148, height=70, no_scrollbar=True):
                        dpg.add_text(label, color=(108, 112, 134))
                        dpg.add_text("0.00", tag=tag, color=color)
                        dpg.add_text(unit,   color=(108, 112, 134))

            dpg.add_separator()

            # ── main area ──────────────────────────────────────────────
            with dpg.group(horizontal=True):

                # left column — controls
                with dpg.child_window(width=280, height=-1, no_scrollbar=True):

                    dpg.add_text("MOTOR CONTROL", color=(108, 112, 134))
                    dpg.add_text(" left",  color=(205, 214, 244))
                    dpg.add_slider_float(tag="sl_left",  min_value=-3.0, max_value=3.0,
                                         default_value=0.0, width=240,
                                         callback=self._apply_velocities)

                    dpg.add_text(" right", color=(205, 214, 244))
                    dpg.add_slider_float(tag="sl_right", min_value=-3.0, max_value=3.0,
                                         default_value=0.0, width=240,
                                         callback=self._apply_velocities)

                    dpg.add_text("scale",   color=(108, 112, 134))
                    dpg.add_slider_float(tag="sl_scale", min_value=0.1,  max_value=3.0,
                                         default_value=1.0, width=240)

                    dpg.add_separator()
                    dpg.add_text("PRESETS", color=(108, 112, 134))

                    with dpg.group(horizontal=True):
                        dpg.add_button(label=" forward", width=85,
                                       callback=lambda: self._preset(1.5,   1.5))
                        dpg.add_button(label=" reverse", width=85,
                                       callback=lambda: self._preset(-1.5, -1.5))

                    with dpg.group(horizontal=True):
                        dpg.add_button(label=" turn R",  width=85,
                                       callback=lambda: self._preset(1.5,  -1.5))
                        dpg.add_button(label=" turn L",  width=85,
                                       callback=lambda: self._preset(-1.5,  1.5))

                    with dpg.group(horizontal=True):
                        dpg.add_button(label=" spin R",  width=85,
                                       callback=lambda: self._preset(2.0,  -2.0))
                        dpg.add_button(label=" spin L",  width=85,
                                       callback=lambda: self._preset(-2.0,  2.0))

                    dpg.add_button(label="  STOP", width=175,
                                   callback=self._stop)

                    with dpg.theme() as stop_theme:
                        with dpg.theme_component(dpg.mvButton):
                            dpg.add_theme_color(dpg.mvThemeCol_Button, (243, 139, 168))
                            dpg.add_theme_color(dpg.mvThemeCol_Text,   (30,  30,  46))
                    dpg.bind_item_theme(dpg.last_item(), stop_theme)

                    dpg.add_separator()
                    dpg.add_text("SONAR DISTANCES", color=(108, 112, 134))
                    with dpg.group(horizontal=True):
                        dpg.add_text("L:", color=(108, 112, 134))
                        dpg.add_text("---", tag="sonar_l", color=(137, 180, 250))
                        dpg.add_text("R:", color=(108, 112, 134))
                        dpg.add_text("---", tag="sonar_r", color=(137, 180, 250))

                    dpg.add_separator()
                    dpg.add_text("OBSTACLE AVOIDANCE", color=(108, 112, 134))
                    dpg.add_checkbox(label="enable auto-avoidance",
                                     tag="chk_avoid",
                                     callback=lambda s, a: setattr(self, 'avoid', a))

                    dpg.add_separator()
                    dpg.add_text("COPPELIASIM CODE", color=(108, 112, 134))
                    dpg.add_text(
                        "setJointTargetVelocity(left,  0.0)\nsetJointTargetVelocity(right, 0.0)",
                        tag="lbl_code", color=(29, 158, 117))

                    dpg.add_separator()
                    dpg.add_button(label=" reset odometry", width=175,
                                   callback=self._reset)

                # right column — charts
                with dpg.child_window(height=-1, no_scrollbar=True):

                    with dpg.group(horizontal=True):

                        # XY trajectory — tall left
                        with dpg.plot(label="XY trajectory", height=380, width=280):
                            dpg.add_plot_axis(dpg.mvXAxis, label="x (m)", tag="ax_tx")
                            with dpg.plot_axis(dpg.mvYAxis, label="y (m)", tag="ax_ty"):
                                dpg.add_line_series(    [], [], label="path",  tag="line_traj")
                                dpg.add_scatter_series( [], [], label="robot", tag="dot_robot")

                        with dpg.group():
                            with dpg.plot(label="linear velocity v(t)", height=185, width=400):
                                dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_vx")
                                with dpg.plot_axis(dpg.mvYAxis, label="m/s", tag="ax_vy"):
                                    dpg.add_line_series([], [], label="v", tag="line_v")

                            with dpg.plot(label="angular velocity w(t)", height=185, width=400):
                                dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_wx")
                                with dpg.plot_axis(dpg.mvYAxis, label="rad/s", tag="ax_wy"):
                                    dpg.add_line_series([], [], label="w", tag="line_w")

                    with dpg.group(horizontal=True):

                        with dpg.plot(label="heading theta(t)", height=185, width=280):
                            dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_thx")
                            with dpg.plot_axis(dpg.mvYAxis, label="degrees", tag="ax_thy"):
                                dpg.add_line_series([], [], label="θ", tag="line_theta")

                        with dpg.plot(label="sonar distances", height=185, width=280):
                            dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_sx")
                            with dpg.plot_axis(dpg.mvYAxis, label="meters", tag="ax_sy"):
                                dpg.add_line_series([], [], label="left",  tag="line_sl")
                                dpg.add_line_series([], [], label="right", tag="line_sr")

                        with dpg.plot(label="wheel velocities", height=185, width=280):
                            dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_wvx")
                            with dpg.plot_axis(dpg.mvYAxis, label="m/s", tag="ax_wvy"):
                                dpg.add_line_series([], [], label="left",  tag="line_wl")
                                dpg.add_line_series([], [], label="right", tag="line_wr")

                    with dpg.group(horizontal=True):

                        with dpg.plot(label="motor health", height=185, width=280):
                            dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_hx")
                            with dpg.plot_axis(dpg.mvYAxis, label="%", tag="ax_hy"):
                                dpg.add_line_series([], [], label="left",  tag="line_hl")
                                dpg.add_line_series([], [], label="right", tag="line_hr")

                        with dpg.plot(label="setpoint vs measured", height=185, width=400):
                            dpg.add_plot_axis(dpg.mvXAxis, label="time (s)", tag="ax_px")
                            with dpg.plot_axis(dpg.mvYAxis, label="m/s", tag="ax_py"):
                                dpg.add_line_series([], [], label="setpoint", tag="line_sp")
                                dpg.add_line_series([], [], label="measured",  tag="line_ms")

        dpg.bind_theme(global_theme)
        dpg.create_viewport(title="Pioneer P3-DX — Control Dashboard",
                            width=1280, height=820)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)

    #=======================================================================
    def _apply_velocities(self):
        scale = dpg.get_value("sl_scale")
        l     = dpg.get_value("sl_left")
        r     = dpg.get_value("sl_right")
        self.robot.set_velocidades(l * scale, r * scale)
        dpg.set_value("lbl_code",
                      f"setJointTargetVelocity(left,  {l*scale:.2f})\n"
                      f"setJointTargetVelocity(right, {r*scale:.2f})")

    def _preset(self, l, r):
        dpg.set_value("sl_left",  l)
        dpg.set_value("sl_right", r)
        self._apply_velocities()

    def _stop(self):
        dpg.set_value("sl_left",  0.0)
        dpg.set_value("sl_right", 0.0)
        self._apply_velocities()
        self.robot.stop()

    def _reset(self):
        self.odom.reset()
        for key in self.hist:
            self.hist[key].clear()
        self.t_start = time.time()

    #=======================================================================
    def _data_loop(self):
        # thread 1 — reads sensors, computes PID, updates odometry
        while self.active:
            t = time.time() - self.t_start

            # 1. read sonars
            sonar_l, sonar_r = self.sonar.read_sides()

            # 2. decide setpoints
            if self.avoid:
                sp_left, sp_right = compute_setpoints(sonar_l, sonar_r)
            else:
                scale    = dpg.get_value("sl_scale")
                sp_left  = dpg.get_value("sl_left")  * scale * wheel_radius
                sp_right = dpg.get_value("sl_right") * scale * wheel_radius

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
            self.hist['sonar_left'].append(sonar_l if sonar_l != float('inf') else 0)
            self.hist['sonar_right'].append(sonar_r if sonar_r != float('inf') else 0)
            self.hist['x'].append(self.odom.x)
            self.hist['y'].append(self.odom.y)

            time.sleep(Digital_twin)

    def _update_charts(self):
        # thread 2 — updates UI every 200ms
        while self.active:
            t = list(self.hist['time'])

            if len(t) > 1:
                xs = list(self.hist['x'])
                ys = list(self.hist['y'])

                # update metrics
                dpg.set_value("m_v",     f"{self.hist['v'][-1]:+.3f}")
                dpg.set_value("m_w",     f"{self.hist['w'][-1]:+.3f}")
                dpg.set_value("m_x",     f"{self.odom.x:+.3f}")
                dpg.set_value("m_y",     f"{self.odom.y:+.3f}")
                dpg.set_value("m_theta", f"{self.hist['theta'][-1]:+.1f}")
                dpg.set_value("m_dist",  f"{self.odom.dist:.3f}")
                dpg.set_value("m_hl",    f"{self.hist['health_left'][-1]:.1f}")
                dpg.set_value("m_hr",    f"{self.hist['health_right'][-1]:.1f}")
                dpg.set_value("lbl_time",f"t = {time.time()-self.t_start:.1f} s")

                sl = self.hist['sonar_left'][-1]
                sr = self.hist['sonar_right'][-1]
                dpg.set_value("sonar_l", f"{sl:.2f} m")
                dpg.set_value("sonar_r", f"{sr:.2f} m")

                # update charts
                dpg.set_value("line_traj",  [xs, ys])
                dpg.set_value("dot_robot",  [[xs[-1]], [ys[-1]]])
                dpg.set_value("line_v",     [t, list(self.hist['v'])])
                dpg.set_value("line_w",     [t, list(self.hist['w'])])
                dpg.set_value("line_theta", [t, list(self.hist['theta'])])
                dpg.set_value("line_sl",    [t, list(self.hist['sonar_left'])])
                dpg.set_value("line_sr",    [t, list(self.hist['sonar_right'])])
                dpg.set_value("line_wl",    [t, list(self.hist['v_left'])])
                dpg.set_value("line_wr",    [t, list(self.hist['v_right'])])
                dpg.set_value("line_hl",    [t, list(self.hist['health_left'])])
                dpg.set_value("line_hr",    [t, list(self.hist['health_right'])])
                dpg.set_value("line_sp",    [t, list(self.hist['sp_left'])])
                dpg.set_value("line_ms",    [t, list(self.hist['v_left'])])

                # auto fit axes
                for ax in ["ax_tx","ax_ty","ax_vx","ax_vy","ax_wx","ax_wy",
                           "ax_thx","ax_thy","ax_sx","ax_sy","ax_wvx","ax_wvy",
                           "ax_hx","ax_hy","ax_px","ax_py"]:
                    dpg.fit_axis_data(ax)

            time.sleep(0.2)

    #=======================================================================
    def run(self):
        self._build_ui()
        self.active = True

        threading.Thread(target=self._data_loop,     daemon=True).start()
        threading.Thread(target=self._update_charts, daemon=True).start()

        # main render loop — DearPyGui controls the window
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

        # cleanup on close — replaces tkinter's close()
        self.active = False
        self.robot.stop()
        if not SIM_MODE and sim:
            try:
                sim.stopSimulation()
                print("[OK] simulation stopped")
            except Exception as e:
                print(f"[WARNING] could not stop simulation: {e}")
        dpg.destroy_context()

#=======================================================================
#-------------------main--------------------------------
#=======================================================================
if __name__ == "__main__":
    app = Dashboard()
    app.run()