"""
Script detective: Lists ALL objects in the scene with their paths.
Useful when you don't know the exact names.

Autor: Joaquin Castro Suarez
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

print("Listando todos los objetos de la escena:\n")
print("-" * 60)

# Get all objects in the scene
all_objects = sim.getObjectsInTree(sim.handle_scene)

for handle in all_objects:
    # Obtener el alias (nombre simple) del objeto
    try:
        alias = sim.getObjectAlias(handle, 1)  # 1 = ruta completa
        print(f"  Handle: {handle:4d}  ->  {alias}")
    except:
        pass

print("-" * 60)
print(f"\nTotal de objetos: {len(all_objects)}")