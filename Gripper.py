import math
from PickObjects import add_marker

youngModulusFingers = 590 # 590 MPa
youngModulusStiffLayerFingers = 5500

scale = 1e3

radius = 30
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian
zHeight = 100
translateFinger1 = f'0 0 {zHeight}'
translateFinger2 = str(radius + radius*math.sin(angle1-math.pi/2)) + ' ' + str(radius*math.cos(angle1-math.pi/2)) + f' {zHeight}'
translateFinger3 = str(radius + radius*math.sin(angle2-math.pi/2)) + ' ' + str(radius*math.cos(angle2-math.pi/2)) + f' {zHeight}'
translations= [translateFinger1,translateFinger2, translateFinger3]
angles=[0,angle1, angle2]

cadFilePath = 'CAD/'


def add_gripper(rootNode, numGrippers:int=3):
    # add_marker(rootNode, 1, [-20, -10, 90], 5.0)
    # add_marker(rootNode, 2, [5, 10, 110], 5.0)

    gripper = rootNode.addChild('gripper')

    for i in range(numGrippers):
        # Add numGrippers fingers for the gripper
        rotation = [-90, 0, 360 - angles[i]*180/math.pi] # In gripping position

        # rotation = [0, 0, 360 - angles[i]*180/math.pi] # Flat

        finger = gripper.addChild(f'finger{i+1}')
        # Add solvers
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        # Load mesh
        finger.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        # Add mechanical properties
        finger.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass= 0.0001)
        # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations 
        finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.3,  youngModulus=youngModulusFingers)#, drawAsEdges=True)

        # Use a bounding box Region of Interest to get the topmost points in the gripper only
        box = [-20, -10, zHeight-2, 5, 10, zHeight+5]
        finger.addObject('BoxROI', name='boxROI', box=box) # Basically bounding box of model
        finger.addObject('BoxROI', name='boxROISubTopo', box=[-30, -10, 0, 20, 10, 130], strict=False)

        # Fix the topmost points of the gripper
        if i == 0:
            finger.addObject('FixedProjectiveConstraint', name='fixedpoint', indices='@boxROI.indices')
        else:
            finger.addObject('FixedProjectiveConstraint', name='fixedpoint', indices='@../finger1/boxROI.indices') # Ensures the same top points are fixed for other grippers

        finger.addObject('LinearSolverConstraintCorrection', name='preconditioner') # Solves the correction due to effect of pneunet cavity on finger

        # Constitutuve law of stiff layer
        modelSubTopo = finger.addChild('modelSubTopo')
        if i == 0:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        else:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@../../finger1/boxROISubTopo.tetrahedraInROI', name='container')
        # Forcefield for stiffer layer deformation
        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers))



        # Pneumatic chamber/cavity 
        cavity = finger.addChild(f'cavity')
        # cavity.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+f'cavity.stl',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+f'cavitywhole.vtk',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity') # Has to be mechancial object so it has DOFs/can be deformed

        # This is the heart of the pneumatic actuator; it directly imparts the pressure force onto the mesh container (pneunetCavity) which can be configured
        # Can either define this by pressure or volume growth
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0, triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False) # Maps the deformation of the cavity mesh to finger 

        # Add collisions to finger 
        collisionFinger = finger.addChild('collisionFinger')

        if i < 2:
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger_hooked.stl', translation = translations[i], rotation=rotation)
        else:
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger_hooked.stl', translation = translations[i], rotation=rotation)
        
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo') # Creates a topology using the mesh loaded by STL loader above
        collisionFinger.addObject('MechanicalObject', name='collisMech')
        collisionFinger.addObject('TriangleCollisionModel', selfCollision=False) # Collision using the mesh defined in mesh topology
        collisionFinger.addObject('LineCollisionModel', selfCollision=False) # Self collision turned off, since finger is (unlikely) to collide with itself
        collisionFinger.addObject('PointCollisionModel', selfCollision=False)
        collisionFinger.addObject('BarycentricMapping') # Maps collision mesh to finger 

        # Add a visual object to visualise finger
        modelVisu = finger.addChild('visu')
        # modelVisu.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger.stl')
        modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        modelVisu.addObject('OglModel', src='@loader', color=[0.4, 0.4, 0.4, 0.6])#color=[0.7, 0.7, 0.7, 0.6])
        modelVisu.addObject('BarycentricMapping')


        # #  Add a visual object to visualise fingernail with different colour
        modelVisu = finger.addChild('fingernail') #fingernail.addChild('visu')
        if i < 2:
            modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'fingernail_hooked.vtk', translation = translations[i], rotation=rotation)
        else:
            modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'fingernail_hooked.vtk', translation = translations[i], rotation=rotation)
        modelVisu.addObject('OglModel', src='@loader', color=[1.0, 0.8, 0.0, 1.0])
        modelVisu.addObject('BarycentricMapping')


