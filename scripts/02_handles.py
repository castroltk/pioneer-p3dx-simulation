'''
Second test: Obtaining references (handles) to the Pioneer objects.

Autor:   Joaquin Castro Suarez
Fecha:   09/05/2026
Robot:   Pioneer p3dx

OBJECTIVE:
    To confirm that we can access the robot, its motors, and its sensors.
    If it fails here, it's a problem with NAMES or PATHS in the scene.

'''



from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

print("\nLooking for Pioneer p3dx items...\n")

#Lookinkg for Pioneer items
try:
    robot = sim.getObject("/PioneerP3DX")
    print(f"Robot found. Handle: {robot}")
    robot_path = "/PioneerP3DX"

    rightMotor = sim.getObject(f"{robot_path}/rightMotor")
    print(f"Right Motor found. Handle: {rightMotor}")

    leftMotor = sim.getObject(f'{robot_path}/leftMotor')
    print(f"Left Motor found. Handle: {leftMotor}")


    ultrasonicSensor_list= []
    for i in range(0, 16):
        handleSensor = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        ultrasonicSensor_list.append(handleSensor)
        print(f"Ultra sonic senor found. Handle {handleSensor}")
        
    
except Exception as e:
    print(f"\nError searching for object: {e}")
    print("\nPossible causes:")
    print("\n- The object name in the scene is different.")
    print("\n- The scene is not loaded in CoppeliaSim.")
    print("\n- The robot is not in the scene's root.")
    print("\nCheck the 'Scene Hierarchy' panel in CoppeliaSim")
    print("\nand adjust the paths in the code.")