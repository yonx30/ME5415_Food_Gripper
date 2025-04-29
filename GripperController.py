#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
from Sofa.constants import *
import math
import numpy as np


def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(0, len(rest_pos)):
        out += [[rest_pos[i][0] + dx, rest_pos[i][1] + dy, rest_pos[i][2] + dz]]
    return out


def rotateRestPos(rest_pos, rx, centerPosY, centerPosZ):
    out = []
    for i in range(0, len(rest_pos)):
        newRestPosY = (rest_pos[i][1] - centerPosY) * math.cos(rx) - (rest_pos[i][2] - centerPosZ) * math.sin(
            rx) + centerPosY
        newRestPosZ = (rest_pos[i][1] - centerPosY) * math.sin(rx) + (rest_pos[i][2] - centerPosZ) * math.cos(
            rx) + centerPosZ
        out += [[rest_pos[i][0], newRestPosY, newRestPosZ]]
    return out


class WholeGripperController(Sofa.Core.Controller):

    def __init__(self, node, pressureLimits:tuple, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = node
        self.pressureLimits = pressureLimits
        
        self.constraints = []
        self.dofs = []
        for i in range(1, 4):
            self.dofs.append(self.node.getChild('finger' + str(i)).getMechanicalState())
            self.constraints.append(self.node.getChild('finger' + str(i)).cavity.SurfacePressureConstraint)
        print("Controller loaded!")
        # self.dofs.append(kw["finger"].getMechanicalState())
        # self.constraints.append(kw["finger"].Cavity.SurfacePressureConstraint)

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        return

    def onAnimateBeginEvent(self, event): # called at each begin of animation step
        pass


    def onAnimateEndEvent(self, event): # called at each end of animation step
        pass


    def onKeypressedEvent(self, e):

        # arr = np.array(self.node.finger1.getObject('tetras').findData('position').value)
        # print(arr[:,0].min(), arr[:,0].max(), arr[:,1].min(), arr[:,1].max(), arr[:,2].min(), arr[:,2].max())

        # arr = self.node.finger1.boxROI.findData('position').value
        # arr = np.array(self.node.finger1.boxROI.position.toList())
        # print(arr)
        # print(arr[:,0].min(), arr[:,0].max(), arr[:,1].min(), arr[:,1].max(), arr[:,2].min(), arr[:,2].max())
        # print(self.node.finger1.boxROI.indices.toList())

        # arr = np.array(self.node.finger1.boxROISubTopo.position.toList())
        # print(arr)
        # print(arr[:,0].min(), arr[:,0].max(), arr[:,1].min(), arr[:,1].max(), arr[:,2].min(), arr[:,2].max())
        # print(arr[:,2].argmax())
        # print(self.node.finger1.boxROI.pointsInROI.toList())
        # print(self.node.finger1.boxROI.indices.toList())
        
        increment = 0.05

        # print("dsads")
        # for i in range(3):
        #     pressureValue = self.constraints[i].value.value[0] + increment
        #     if pressureValue > 1.5:
        #         pressureValue = 1.5
        #     self.constraints[i].value = [pressureValue]


        if e["key"] == Sofa.constants.Key.space:
            for i in range(3):
                pressureValue = self.constraints[i].value.value[0] + increment
                if pressureValue > self.pressureLimits[1]:
                    pressureValue = self.pressureLimits[1]
                self.constraints[i].value = [pressureValue]
            print(f"Inflate: {pressureValue}")

        if e["key"] == Sofa.constants.Key.minus:
            for i in range(3):
                pressureValue = self.constraints[i].value.value[0] - increment
                if pressureValue < self.pressureLimits[0]:
                    pressureValue = self.pressureLimits[0]
                self.constraints[i].value = [pressureValue]
            print(f"Deflate: {pressureValue}")

        elif e["key"] == Sofa.constants.Key.uparrow:
            for i in range(3):
                results = moveRestPos(self.dofs[i].rest_position.value, 3.0, 0.0, 0.0)
                self.dofs[i].rest_position.value = results

        elif e["key"] == Sofa.constants.Key.downarrow:
            for i in range(3):
                results = moveRestPos(self.dofs[i].rest_position.value, -3.0, 0.0, 0.0)
                self.dofs[i].rest_position.value = results

        elif e["key"] == Sofa.constants.Key.leftarrow:
            dy = 3.0 * math.cos(self.rotAngle)
            dz = 3.0 * math.sin(self.rotAngle)
            for i in range(3):
                results = moveRestPos(self.dofs[i].rest_position.value, 0.0, dy, dz)
                self.dofs[i].rest_position.value = results
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        elif e["key"] == Sofa.constants.Key.rightarrow:
            dy = -3.0 * math.cos(self.rotAngle)
            dz = -3.0 * math.sin(self.rotAngle)
            for i in range(3):
                results = moveRestPos(self.dofs[i].rest_position.value, 0.0, dy, dz)
                self.dofs[i].rest_position.value = results
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # Direct rotation
        elif e["key"] == "A":
            for i in range(3):
                results = rotateRestPos(self.dofs[i].rest_position.value, math.pi / 16, self.centerPosY,
                                        self.centerPosZ)
                self.dofs[i].rest_position.value = results
            self.rotAngle = self.rotAngle + math.pi / 16

        # Indirect rotation
        elif e["key"] == "Q":
            for i in range(3):
                results = rotateRestPos(self.dofs[i].rest_position.value, -math.pi / 16, self.centerPosY,
                                        self.centerPosZ)
                self.dofs[i].rest_position.value = results
            self.rotAngle = self.rotAngle - math.pi / 16
