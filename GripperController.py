import Sofa.Core
from Sofa.constants import *
import math
import numpy as np


def moveRestPos(rest_pos, dx, dy, dz):
    '''Returns the moved position'''
    out = []
    for i in range(0, len(rest_pos)):
        out += [[rest_pos[i][0] + dx, rest_pos[i][1] + dy, rest_pos[i][2] + dz]]
    return out


# def rotateRestPos(rest_pos, rx, centerPosY, centerPosZ):
#     out = []
#     for i in range(0, len(rest_pos)):
#         newRestPosY = (rest_pos[i][1] - centerPosY) * math.cos(rx) - (rest_pos[i][2] - centerPosZ) * math.sin(
#             rx) + centerPosY
#         newRestPosZ = (rest_pos[i][1] - centerPosY) * math.sin(rx) + (rest_pos[i][2] - centerPosZ) * math.cos(
#             rx) + centerPosZ
#         out += [[rest_pos[i][0], newRestPosY, newRestPosZ]]
#     return out

class WholeGripperController(Sofa.Core.Controller):
    '''Controller class to control gripper movements'''
    def __init__(self, node, pressureLimits:tuple=(-1.0,1.5), inflateIncrement:float=0.05, moveIncrement:float=1.0, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = node
        self.pressureLimits = pressureLimits
        
        self.constraints = []
        self.dofs = []

        for i in range(1, 4):
            self.dofs.append(self.node.gripper.getChild(f'finger{i}').getMechanicalState())
            self.constraints.append(self.node.gripper.getChild(f'finger{i}').cavity.SurfacePressureConstraint)

        self.plane = self.node.Plane.getMechanicalState()

        print("Controller loaded!")

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        self.inflateIncrement = inflateIncrement
        self.moveIncrement = moveIncrement

        return

    def onAnimateBeginEvent(self, event): # called at each begin of animation step
        pass


    def onAnimateEndEvent(self, event): # called at each end of animation step
        pass


    def onKeypressedEvent(self, e):
        if e["key"] == Sofa.constants.Key.space:
            for i in range(3):
                pressureValue = self.constraints[i].value.value[0] + self.inflateIncrement
                if pressureValue > self.pressureLimits[1]:
                    pressureValue = self.pressureLimits[1]
                self.constraints[i].value = [pressureValue]
            print(f"Inflate: {pressureValue}")

        if e["key"] == Sofa.constants.Key.minus:
            for i in range(3):
                pressureValue = self.constraints[i].value.value[0] - self.inflateIncrement
                if pressureValue < self.pressureLimits[0]:
                    pressureValue = self.pressureLimits[0]
                self.constraints[i].value = [pressureValue]
            print(f"Deflate: {pressureValue}")

        elif e["key"] == Sofa.constants.Key.uparrow:
            results = moveRestPos(self.plane.position.value, 0.0, 0.0, -self.moveIncrement/2)
            self.plane.position.value = results

            results = np.array(self.node.camera.position.value) + np.array([0.0, 0.0, -self.moveIncrement/2])
            self.node.camera.position.value = list(results)


        elif e["key"] == Sofa.constants.Key.downarrow:
            results = moveRestPos(self.plane.position.value, 0.0, 0.0, self.moveIncrement/2)
            self.plane.position.value = results

            results = np.array(self.node.camera.position.value) + np.array([0.0, 0.0, self.moveIncrement/2])
            self.node.camera.position.value = list(results)

        elif e["key"] == Sofa.constants.Key.leftarrow:
            results = moveRestPos(self.plane.position.value, self.moveIncrement, 0.0, 0.0)
            self.plane.position.value = results

            results = np.array(self.node.camera.position.value) + np.array([self.moveIncrement/2, 0.0, 0.0])
            self.node.camera.position.value = list(results)

        elif e["key"] == Sofa.constants.Key.rightarrow:
            results = moveRestPos(self.plane.position.value, -self.moveIncrement, 0.0, 0.0)
            self.plane.position.value = results

            results = np.array(self.node.camera.position.value) + np.array([-self.moveIncrement/2, 0.0, 0.0])
            self.node.camera.position.value = list(results)

        # # Direct rotation
        # elif e["key"] == "A":
        #     for i in range(3):
        #         results = rotateRestPos(self.dofs[i].rest_position.value, math.pi / 16, self.centerPosY,
        #                                 self.centerPosZ)
        #         self.dofs[i].rest_position.value = results
        #     self.rotAngle = self.rotAngle + math.pi / 16

        # # Indirect rotation
        # elif e["key"] == "Q":
        #     for i in range(3):
        #         results = rotateRestPos(self.dofs[i].rest_position.value, -math.pi / 16, self.centerPosY,
        #                                 self.centerPosZ)
        #         self.dofs[i].rest_position.value = results
        #     self.rotAngle = self.rotAngle - math.pi / 16
