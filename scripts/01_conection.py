"""
Primer test: conectar con CoppeliaSim y verificar comunicacion.

Autor:   Joaquin Castro Suarez
Fecha:   09/05/2026
Robot:   Pioneer p3dx

OBJETIVO:
    Confirmar que Python puede hablar con CoppeliaSim.
    No mueve nada, solo lee informacion.

REQUISITOS:
    1. CoppeliaSim abierto.
    2. La simulacion NO necesita estar corriendo todavia.
=============================================================================
Author: Joaquín Castro Suárez
Date: 05/09/2026
Robot: Pioneer p3dx

OBJECTIVE:

To confirm that Python can communicate with CoppeliaSim.

It doesn't move anything, it only reads information.

REQUIREMENTS:

1. CoppeliaSim open.

2. The simulation does NOT need to be running yet.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Connection to CoppeliaSim")

#Crear cleinte y obtener el namespace 'sim'
client = RemoteAPIClient()
sim = client.getObject("sim")

print("Succesful connetion!!")

#Time simulation
tiempo_sim = sim.getSimulationTime()

print(f"tiempo actual de la simulacion: {tiempo_sim} segundos")

state = sim.getSimulationState()
states = {
    0: "STOP",
    8: "PAUSe",
    16: "RUNNING"

}

print(f"Estado de la simulacion: {states.get(state, 'Desconocido')}")

print("\n TEST COMPLETE.")
















