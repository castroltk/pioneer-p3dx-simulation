"""
Make the robot MOVE FORWARD IN A STRAIGHT LINE.

Autor:   Joaquin Castro Suarez
Fecha:   09/05/2026
Robot:   Pioneer p3dx

OBJECTIVE:
    Apply the fundamental concept:
    Both motors at the same speed = robot moves straight.

CONCEPT:
    setJointTargetVelocity(motor, speed)
    - speed in rad/s
    - positive = rotation in one direction
    - negative = rotation in the other direction
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# ==========CONECTION============
client = RemoteAPIClient()
sim = client.getObject('sim')

# =================HANDLE============================================
robot = sim.getObject('/PioneerP3DX')
motor_left = sim.getObject('/PioneerP3DX/leftMotor')
motor_right = sim.getObject('/PioneerP3DX/rightMotor')

ultrasonicSensor_list= []
for i in range(0, 16):
    sensor = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
    ultrasonicSensor_list.append(sensor)
    #print(f"Sensor {i:2d} found. Handle {sensor}")

sim.startSimulation()   
pos_start = sim.getObjectPosition(robot, sim.handle_world)

speed = 2.0   # rad/s en cada rueda

sim.setJointTargetVelocity(motor_left, speed)
sim.setJointTargetVelocity(motor_right, speed)






time.sleep(10)
sim.stopSimulation()

