import math
from PickObjects import add_marker

youngModulusFingers = 590
youngModulusStiffLayerFingers = 1500

scale = 1e3

radius = 30
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian
zHeight = 85
translateFinger1 = f'0 0 {zHeight}'
translateFinger2 = str(radius + radius*math.sin(angle1-math.pi/2)) + ' ' + str(radius*math.cos(angle1-math.pi/2)) + f' {zHeight}'
translateFinger3 = str(radius + radius*math.sin(angle2-math.pi/2)) + ' ' + str(radius*math.cos(angle2-math.pi/2)) + f' {zHeight}'
translations= [translateFinger1,translateFinger2, translateFinger3]
angles=[0,angle1, angle2]

cadFilePath = 'C:/Users/yonx3/OneDrive - National University of Singapore/Documents/NUS Masters/ME5415 Advanced Soft Robotics/Design Project/CAD/'


def add_gripper(rootNode):
    # add_marker(rootNode, 1, [-20, -10, 90], 5.0)
    # add_marker(rootNode, 2, [5, 10, 110], 5.0)

    gripper = rootNode.addChild('gripper')


    # controller = gripper.addChild('controller') # Controls movement of gripper
    # controller.addObject('MechanicalObject', name='rigidParticle', template='Vec3d', position=f'{radius} 0 {zHeight+5} 0 0 0 1', showObject='1', showObjectScale='0.5')
    # controller.addObject('UniformMass', totalMass= 0.0001)
    # controller.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
    # controller.addObject('SparseLDLSolver', name='preconditioner')

    # controller.addObject('RigidMapping', input='@controller', output='@rigidParticle')

    # controllerMap = controller.addChild('mappingVec')
    # controllerMap.addObject('MechanicalObject', name='particleVec', template='Vec3d')
    # controllerMap.addObject('RigidMapping', input='@../rigidParticle', output='@particleVec')

    for i in range(3):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        rotation = [-90, 0, 360 - angles[i]*180/math.pi]

        # controller = gripper.addChild(f'finger{i+1}Controller')
        # controller.addObject('MechanicalObject', name='rigidParticle', template='Vec3d', translation = translations[i], rotation=rotation, showObject='1', showObjectScale='0.5')
        # controller.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'sphere.vtk', rotation=rotation, translation = translations[i])
        # controller.addObject('MeshTopology', src='@loader', name='container')
        # controller.addObject('MechanicalObject', name='rigidParticle', template='Vec3d', showIndices=False, showIndicesScale=4e-5)


        finger = gripper.addChild(f'finger{i+1}')
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        finger.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        finger.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass= 0.0001)
        
        # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations 
        finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.3,  youngModulus=youngModulusFingers)#, drawAsEdges=True)

        # box = [-30, 0, 90, 10, 0, 110]
        # box = [-500,-500,-500,500,500,500]
        box = [-20, -10, zHeight-2, 5, 10, zHeight+5]
        finger.addObject('BoxROI', name='boxROI', box=box) # Basically bounding box of model
        finger.addObject('BoxROI', name='boxROISubTopo', box=[-30, -10, 0, 20, 10, 130], strict=False)

        # finger.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False) # Maps the finger to its parent controller

        # gripper.addObject('AttachProjectiveConstraint', template='Vec3d', object1='@controller/rigidParticle', indices1='0', 
        #                   object2=f'@finger{i+1}/tetras', indices2='10', constraintFactor='1', twoWay='False')
        # gripper.addObject('RigidMapping', template='Rigid3d', input='@controller/rigidParticle', output='@finger{i+1}/boxROI.indices')

        # gripper.addObject('AttachProjectiveConstraint', template='Vec3d', object1='@controller/rigidParticle', indices1='0', 
        #                   object2=f'@finger{i+1}/tetras', indices2='@boxROI.indices', constraintFactor='1', twoWay='False')


        # Very large stiffness to essentially fix object in space, using the boxROI as a fixing point
        if i == 0:
            # finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)
            # finger.addObject('RestShapeSpringsForceField', points='@boxROI.pointsInROI', stiffness=1e12, angularStiffness=1e12)
            # finger.addObject('RestShapeSpringsForceField', points='@boxROI.positions[10]', stiffness=1, angularStiffness=1e12)
            finger.addObject('FixedProjectiveConstraint', name='fixedpoint', indices='@boxROI.indices') #, mstate="1")
            # print(finger.fixedpoint.bbox)
        else:
            # finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12, angularStiffness=1e12)
            finger.addObject('FixedProjectiveConstraint', name='fixedpoint', indices='@../finger1/boxROI.indices')

        finger.addObject('LinearSolverConstraintCorrection', name='preconditioner')#solverName='preconditioner') # Solves the correction due to effect of pneunet cavity on finger

        ##########################################
        # Sub topology						   #
        ##########################################
        # Constitutuve law of stiff layer
        modelSubTopo = finger.addChild('modelSubTopo')
        if i == 0:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        else:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@../../finger1/boxROISubTopo.tetrahedraInROI', name='container')
        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers))


        ##########################################
        # Constraint							 #
        ##########################################
        # Pneumatic chamber/cavity 
        # for i in range(1,3):
        cavity = finger.addChild(f'cavity')
        # cavity.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+f'cavity.stl',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+f'cavitywhole.vtk',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity') # Has to be mechancial object so it has DOFs/can be deformed

        # This is the heart of the pneumatic actuator; it directly imparts the pressure force onto the mesh container (pneunetCavity) which can be configured
        # Can either define this by pressure or volume growth
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0, triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False) # Maps the deformation of the cavity mesh to finger 

        ##########################################
        # Collision							  #
        ##########################################

        collisionFinger = finger.addChild('collisionFinger')
        collisionFinger.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger.stl', translation = translations[i], rotation=rotation)
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo') # Loads mesh from STL loader above
        collisionFinger.addObject('MechanicalObject', name='collisMech')
        collisionFinger.addObject('TriangleCollisionModel', selfCollision=False) # Collision using hte mesh defined in mesh topology
        collisionFinger.addObject('LineCollisionModel', selfCollision=False) # Self collision turned off, since finger is (unlikely) to collide with itself
        collisionFinger.addObject('PointCollisionModel', selfCollision=False)
        collisionFinger.addObject('BarycentricMapping') # Maps finger to cavity 


        ##########################################
        # Visualization						  #
        ##########################################
        modelVisu = finger.addChild('visu')
        # modelVisu.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger.stl')
        modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6])
        modelVisu.addObject('BarycentricMapping')




