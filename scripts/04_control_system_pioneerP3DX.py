"""
Author: Joaquín Castro Suárez
Date: 05/12/2026
Robot: Pioneer p3dx

OBJECTIVE:

    

REQUIREMENTS:
    1. CoppeliaSim open.
    2. The simulation does NOT need to be running yet.
    3. pip install matplotlib
    4. pip install coppeliasim-zmqremoteapi-client

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

wheel_radius    = 0.0975    #meters
axle_length     = 0.330     #meters
Digital_twin    = 0.05      #seconds per cycly

# PID tuning
kp = 1.2    
ki = 0.1
kd = 0.05

# target speed
setpoint = 0.5   # m/s

# obstacle thresholds
safe_distance     = 1.0   # meters
Warning_distance  = 0.5
cristical_distance= 0.2

# motor health
max_effort      = 2.5
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
        error          = setpoint - measured
        self.integral += error * dt

        # anti-windup — prevent overshooting on sudden setpoint change
        max_integral  = 1.0

        # Anti-windup: Clamps integral term to prevent excessive error accumulation.
        # Format: math.clamp(variable, min_limit, max_limit)
        math.fclamp(self.integral, -max_integral, max_integral)

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

        self._monitor_health(output)
        return output
#________________________________________________________________________________

#=======================================================================

#=======================================================================

    
