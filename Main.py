import Sofa
import Sofa.Gui
from GripperController import WholeGripperController
from PickObjects import *
# from Gripper import add_gripper


import math

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1000

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

    # add_marker(rootNode, 1, [-20, 0, 90], 5.0)
    # add_marker(rootNode, 2, [5, 0, 110], 5.0)
    
    add_marker(rootNode, 1, [-20, -10, 90], 5.0)
    add_marker(rootNode, 2, [5, 10, 110], 5.0)


    for i in range(3):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        rotation = [-90, 0, 360 - angles[i]*180/math.pi]
        finger = rootNode.addChild('finger'+str(i+1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        finger.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass= 0.0001)
        
        # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations 
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=youngModulusFingers)#, drawAsEdges=True)

        # box = [-30, 0, 90, 10, 0, 110]
        # box = [-500,-500,-500,500,500,500]
        box = [-20, -10, zHeight-2, 5, 10, zHeight+5]
        finger.addObject('BoxROI', name='boxROI', box=box) # Basically bounding box of model
        finger.addObject('BoxROI', name='boxROISubTopo', box=[-30, -10, 0, 20, 10, 130], strict=False)

        # topPoint = [10.000000000000007, 22.9588190858165, 100.0]
        # points = finger.boxROI.positionsfinger.boxROI.indices

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
        cavity.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+f'cavity.vtk',translation = translations[i], rotation=rotation)
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




def add_plugins(rootNode):
    '''Loads in required plugins to root node'''
    pluginNode = rootNode.addChild('PluginNode')
    pluginNode.addObject('RequiredPlugin', name='SoftRobots')
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Solver", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False) # Needed to load meshes such as stl and obj
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.StateContainer", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Shader", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Setting", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.Spring", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Dynamic", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.Linear", printLog=False)
    pluginNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective", printLog=False)
    return pluginNode    

def add_pipelines(rootNode):
    rootNode.addObject('FreeMotionAnimationLoop') # Animation pipeline, builds system incl constraints 
    rootNode.addObject('CompositingVisualLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=5.0')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1, angleCone=0.0)
    rootNode.addObject('BackgroundSetting', color=[1, 1, 1], listening='1')
    rootNode.addObject('OglSceneFrame', style='CubesCones', alignment='TopRight')    

def add_plane(rootNode):
    planeNode = rootNode.addChild('Plane')

    # Load mesh using the appropriate filetype loader
    planeNode.addObject('MeshOBJLoader', name='loader', filename='mesh/floorFlat.obj', triangulate=True, rotation=[90, 0, 0], scale=10, translation=[0, 0, 0]) # mesh/ folder items universally accessible in SOFA
    # planeNode.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Surface.obj', triangulate=True, rotation=[0, 0, 0], scale=5, translation=[0, 0, 0])#, position=[0, 0, 10, 0, 0, 0, 1])
    # planeNode.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Surface.Stl', triangulate=True, rotation=[0, 0, 270], scale=5, translation=[-122, 0, 0])

    planeNode.addObject('MeshTopology', src='@loader')
    planeNode.addObject('MechanicalObject', src='@loader')

    # Add collision models to the plane to prevent object from falling through. Specified to not move during simulation
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)
    planeNode.addObject('LineCollisionModel', simulated=False, moving=False)
    planeNode.addObject('PointCollisionModel', simulated=False, moving=False)
    planeNode.addObject('OglModel',name='Visual', src='@loader', color=[1, 0, 0, 1])
    return rootNode

def add_camera(rootNode, position:list, orientation:list):
    # rootNode.addObject('InteractiveCamera', name='Cam', position=f'{position[0]} {position[1]} {position[2]}', 
    #                    lookAt="0 0 0", orientation=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    rootNode.addObject('InteractiveCamera', name='Cam', position=f'{position[0]} {position[1]} {position[2]}', 
                       lookAt=f"{position[0]} 0 0", distance=f'{position[0]} {position[1]} {position[2]}')#orientation=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    # lighting = rootNode.addChild('lighting')
    # lighting.addObject('LightManager')
    # # lighting.addObject('PositionalLight', name='light2', color='256 256 256', attenuation="0.1", position='-100 0 50')
    # lighting.addObject('DirectionalLight', name='light3', color='0 0 1', direction="0 0 -1")


def createScene(rootNode):
    add_plugins(rootNode)
    add_pipelines(rootNode)
    add_plane(rootNode)
    add_sphere(rootNode, [30, 0, 50], 0.001, 15)
    # add_cube(rootNode, -200, 00, 100, 0.001, 6)
    add_camera(rootNode, [-100, -500, 50], orientation=[-100,0,0])
    add_gripper(rootNode)

    controller = WholeGripperController(name="controller", node=rootNode, pressureLimits=(-1, 2))
    rootNode.addObject(controller)

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.findData('gravity').value=[0, 0, -981]


    return rootNode



def main():
    # Call the SOFA function to create the root node
    root = Sofa.Core.Node("root")

    # Call the createScene function, as runSofa does
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Launch the GUI (qt or qglviewer)
    Sofa.Gui.GUIManager.Init("myscene", "qt")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)

    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root) # Runs the continuous main animation loop
    Sofa.Gui.GUIManager.closeGUI()

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()