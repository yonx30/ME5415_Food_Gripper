def add_marker(rootNode, i:int, position:list, scale:float, color:list=[1,1,0]):
    # Add sphere to pickup
    marker = rootNode.addChild(f'marker{i}')

    # Solver/time integrators to calculate system positions and velocities at each time ste
    #visualization
    markerVisu = marker.addChild('markerVisu')
    markerVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj')
    markerVisu.addObject('OglModel', name='Visual', src='@loader', translation=position, color=color, scale=scale)


def add_cube(rootNode, position:list, mass:float, scale:float):
    # Add cube to pickup
    cube = rootNode.addChild('cube')

    # Solver/time integrators to calculate system positions and velocities at each time step
    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('SparseLDLSolver', name='linearSolver')
    cube.addObject('MechanicalObject', template='Rigid3', position=position+[0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=mass)
    cube.addObject('UncoupledConstraintCorrection')

    #collision
    cubeCollis = cube.addChild('cubeCollis')

    cubeCollis.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj', triangulate=True,  scale=scale+0.1)

    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping') # Maps DOFs of cube mesh to a rigid SOFA object (use other types of mapping for soft objects)

    #visualization
    cubeVisu = cube.addChild('cubeVisu')

    cubeVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj')

    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], scale=scale) # Visual mesh *slightly* smaller than actual object model/collision mesh
    cubeVisu.addObject('RigidMapping')

def add_sphere(rootNode, position:list, mass:float, scale:float):
    # Add sphere to pickup
    sphere = rootNode.addChild('sphere')

    # Solver/time integrators to calculate system positions and velocities at each time step
    sphere.addObject('EulerImplicitSolver', name='odesolver')
    sphere.addObject('SparseLDLSolver', name='linearSolver')
    sphere.addObject('MechanicalObject', template='Rigid3', position=position+[0, 0, 0, 1])
    sphere.addObject('UniformMass', totalMass=mass)
    sphere.addObject('UncoupledConstraintCorrection')

    #collision
    sphereCollis = sphere.addChild('sphereCollis')

    sphereCollis.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj', triangulate=True,  scale=scale+0.1) # Make collision hitbox slightly bigger than visual
    # sphereCollis.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.obj', triangulate=True,  scale=1)
    # sphereCollis.addObject('MeshSTLLoader', name='loader', filename='details/data/mesh/Sphere.stl', triangulate=True,  scale=1)

    sphereCollis.addObject('MeshTopology', src='@loader')
    sphereCollis.addObject('MechanicalObject')
    sphereCollis.addObject('TriangleCollisionModel')
    sphereCollis.addObject('LineCollisionModel')
    sphereCollis.addObject('PointCollisionModel')
    sphereCollis.addObject('RigidMapping') # Maps DOFs of sphere mesh to a rigid SOFA object (use other types of mapping for soft objects)

    #visualization
    sphereVisu = sphere.addChild('sphereVisu')

    sphereVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj')
    # sphereVisu.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.obj')
    # sphereVisu.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.stl')

    sphereVisu.addObject('OglModel', name='Visual', src='@loader', color=[1.0, 1.0, 0.0], scale=scale)
    sphereVisu.addObject('RigidMapping')