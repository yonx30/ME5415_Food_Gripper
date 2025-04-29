import Sofa
import Sofa.Gui
from GripperController import WholeGripperController
from PickObjects import *
from Gripper import add_gripper

cadFilePath = 'C:/Users/yonx3/OneDrive - National University of Singapore/Documents/NUS Masters/ME5415 Advanced Soft Robotics/Design Project/CAD/'

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
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=1.0')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1, angleCone=0.0)
    rootNode.addObject('BackgroundSetting', color='1 1 1', listening='1')
    rootNode.addObject('OglSceneFrame', style='CubesCones', alignment='TopRight')    

def add_plane(rootNode):
    planeNode = rootNode.addChild('Plane')

    # Load mesh using the appropriate filetype loader
    # planeNode.addObject('MeshOBJLoader', name='loader', filename='mesh/floorFlat.obj', triangulate=True, rotation=[90, 0, 0], scale=10, translation=[0, 0, 0]) # mesh/ folder items universally accessible in SOFA
    planeNode.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'square_grid.stl', triangulate=True, rotation=[0, 0, 0], scale=2, translation=[0, 0, -10])

    planeNode.addObject('MeshTopology', src='@loader')
    planeNode.addObject('MechanicalObject', src='@loader')

    # Add collision models to the plane to prevent object from falling through. 
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=True)
    planeNode.addObject('LineCollisionModel', simulated=False, moving=True)
    planeNode.addObject('PointCollisionModel', simulated=False, moving=True)
    planeVisu = planeNode.addChild('visu')
    planeVisu.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'square_grid.stl', triangulate=True, rotation=[0, 0, 0], scale=2, translation=[0, 0, -10])
    # planeVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/floorFlat.obj', triangulate=True, rotation=[90, 0, 0], scale=10, translation=[0, 0, 0])

    # planeVisu.addObject('OglModel', src='@loader', color=[0.3, 0.3, 0.4, 1])
    planeVisu.addObject('OglModel', src='@loader', color=[1.0, 0.3, 0.0, 1])
    planeVisu.addObject('BarycentricMapping')
    return rootNode

def add_camera(rootNode, position:list):
    rootNode.addObject('InteractiveCamera', name='camera', position=f'{position[0]} {position[1]} {position[2]}', 
                       lookAt=f"{position[0]} 0 0", distance=f'{position[0]} {position[1]} {position[2]}')
    # lighting = rootNode.addChild('lighting')
    # lighting.addObject('LightManager')
    # # lighting.addObject('PositionalLight', name='light2', color='256 256 256', attenuation="0.1", position='-100 0 50')
    # lighting.addObject('DirectionalLight', name='light3', color='0 0 1', direction="0 0 -1")


def createScene(rootNode):
    add_plugins(rootNode)
    add_pipelines(rootNode)
    add_plane(rootNode)
    # add_sphere(rootNode, [30, 0, 20], 0.0001, 15)
    add_cube(rootNode, [24, 0, 20], 1.0, 6)
    add_camera(rootNode, [-100, -5000, 50])
    add_gripper(rootNode)
    # add_sausage(rootNode, [-20, -60, 10], 0.01) # 100g
    # add_meatball(rootNode, [30, -5, 5], 0.02) # 20g
    # add_brocolli(rootNode, [25, 0, 5], 0.02) # 20g
    # add_carrot(rootNode, [30, -100, 5], 0.1) # 100g
    # add_green_beans(rootNode, [15, 0, 5], 0.0000132) # 132mg
    # add_spaghetti(rootNode, [15, 0, 1], 0.001) # 1g
    # add_cookie(rootNode, [10, 0, 5], 0.005) # 5
    # add_orangeJuice(rootNode, [30, 0, 0], 0.300) # 300g (incl juice weight)


    controller = WholeGripperController(name="controller", node=rootNode, pressureLimits=(-1, 5), inflateIncrement=0.2, moveIncrement=1.0)
    rootNode.addObject(controller)

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.findData('gravity').value=[0, 0, -9810]

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