import math

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500

scale = 1e3

radius = 70
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian
translateFinger1 = '-100 0 100'
translateFinger2 = '-100 ' + str(radius + radius*math.sin(angle1-math.pi/2)) + ' ' + str(radius*math.cos(angle1-math.pi/2))
translateFinger3 = '-100 ' + str(radius + radius*math.sin(angle2-math.pi/2)) + ' ' + str(radius*math.cos(angle2-math.pi/2))
translations= [translateFinger1,translateFinger2, translateFinger3]
angles=[0,angle1, angle2]

cadFilePath = 'C:/Users/yonx3/OneDrive - National University of Singapore/Documents/NUS Masters/ME5415 Advanced Soft Robotics/Design Project/CAD/'


def add_gripper(rootNode):

    for i in range(1):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        rotation = [-90, 0, 360 - angles[i]*180/math.pi]
        finger = rootNode.addChild('finger'+str(i+1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        finger.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'cavity.vtk', rotation=rotation, translation = translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass= 0.04)
        
        # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations 
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=youngModulusFingers)#, drawAsEdges=True)

        finger.addObject('BoxROI', name='boxROI', box=[-500, -500, 90, 500, 500, 100]) # Basically bounding box of model
        finger.addObject('BoxROI', name='boxROISubTopo', box=[-500, -500, -100, 500, 500, 90], strict=False)

        # Very large stiffness to essentially fix object in space, using the boxROI as a fixing point
        if i == 0:
            finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)
        else:
            finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12, angularStiffness=1e12)

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
        cavity = finger.addChild('cavity')
        cavity.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'cavity.stl',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity') # Has to be mechancial object so it has DOFs/can be deformed

        # This is the heart of the pneumatic actuator; it directly imparts the pressure force onto the mesh container (pneunetCavity) which can be configured
        # Can either define this by pressure or volume growth
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001, triangles='@topo.triangles', valueType='pressure')
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
        modelVisu.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger.stl')
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation = translations[i], rotation=rotation)
        modelVisu.addObject('BarycentricMapping')

