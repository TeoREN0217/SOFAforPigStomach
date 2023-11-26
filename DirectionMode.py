import Sofa.Core
import SofaRuntime
import numpy as np
from math import cos,atan,acos,sqrt,sin,pi
from splib3.numerics import Vec3, Quat,sdiv
from scipy.spatial.transform import Rotation as R
from splib3.objectmodel import setData
from splib3.animation import animate, AnimationManager
import os
import time
#Screenshot
import win32gui
# from PyQt5.QtWidgets import QApplication
# from PyQt5.QtGui import *
import sys
# from ArduinoInterface_730 import Controller
import Sofa.constants.Key as Key
import math
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'
Img_path = os.path.dirname(os.path.abspath(__file__))+'/img/'
##########################################
# Some Settings                          #
##########################################
#Setting CableActuator position:
#first point radius
length1 = 4
#other point radius
cablelength=4

#Drawing Circle: 
radius=40
modelX=0
modelY=0
modelZ=0
#TargetPoint position in inverse mode: 
target_position=[0, 20,100]
# target_position=[0, -55,160]
print_flag=0
#EffectorPoint on the robot:
robot_decisionpoint=[[0., 0., 60]]
#Static patient position
patient_position=[0,-50,140,0,0,0,1]
#Using rigidify function, groupIndices is rigid part
print_flag=0

groupIndices=[[0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 738, 739, 740, 741, 742, 743, 744, 745, 746, 747, 748, 749, 750, 751, 752, 753, 754, 755, 756, 757, 758, 759, 760, 761, 762, 763, 764, 765, 766, 767, 768, 769, 770, 771, 772, 773, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 826, 827, 828, 829, 1159, 1163]]  
#For ScreenShot


import Sofa.Core
import Sofa.constants.Key as Key

# import socket
# udp_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)


def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(0,len(rest_pos)) :
        out += [[rest_pos[i][0]+dx, rest_pos[i][1]+dy, rest_pos[i][2]+dz]]
    return out


def rotateRestPos(rest_pos, rz, centerPosX, centerPosY):
    out = []
    for i in range(len(rest_pos)):
        newRestPosX = (rest_pos[i][0] - centerPosX)*math.cos(rz) - (rest_pos[i][1] - centerPosY)*math.sin(rz) + centerPosX
        newRestPosY = (rest_pos[i][0] - centerPosX)*math.sin(rz) + (rest_pos[i][1] - centerPosY)*math.cos(rz) + centerPosY
        out.append([newRestPosX, newRestPosY, rest_pos[i][2]])
    return out

class ThroatController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable1 = args[0]
        self.cable2 = args[1]
        self.cable3 = args[2]
        self.node =   args[3]
        self.rigidBaseMO =args[4]

        self.Simulation =self.node.getChild("Simulation")
        self.dofs=self.Simulation.getChild("Throat").dofs
        # self.rig=self.Simulation.getChild("RigidifiedStructure")
        # self.dofs=self.rig.getChild("RigidParts").RigidifiedParticules.dofs
        
        self.name = "ThroatController"
        self.slidingVar=0
        self.rate = 1
        self.angularRate =0.5
        self.centerPosX = 0
        self.centerPosY = 0
        self.rotAngle = 0
        return 
    
    def _extracted_from_onKeypressedEvent_10(self, qOld, posA, angularRate):
        qNew = Quat.createFromEuler([0., angularRate, 0.], 'rxzy')
        qNew.normalize()
        qNew.rotateFromQuat(qOld)
        for i in range(4):
            posA[0][i+3] = qNew[i]    

            
    def onKeypressedEvent(self, e):
        if e["key"] == Key.uparrow:  # uparrow
            # with self.rigidBaseMO.rest_position.writeable() as posA:
            #     posA[0][0] -= self.rate
            results = moveRestPos(self.dofs.rest_position.value, 0.0, 0.0, 1.0)
            self.slidingVar += 1.0
            self.dofs.rest_position.value = results
        if e["key"] ==Key.downarrow:  # downarrow
            results = moveRestPos(self.dofs.rest_position.value, 0.0, 0.0, -1.0)
            self.slidingVar -= 1.0
            self.dofs.rest_position.value = results
        if e["key"] ==Key.leftarrow:  # rightarrow 
            results = rotateRestPos(self.dofs.rest_position.value, math.pi / 16, self.centerPosX, self.centerPosY)
            self.dofs.rest_position.value = results
            self.rotAngle = self.rotAngle + math.pi/16 
        if e["key"] ==Key.rightarrow:  # leftarrow
            results = rotateRestPos(self.dofs.rest_position.value, -math.pi / 16, self.centerPosX, self.centerPosY)
            self.dofs.rest_position.value = results
            self.rotAngle = self.rotAngle - math.pi/16


##########################################
# Rigidify function                      #
##########################################
def getBarycenter(selectedPoints):
    poscenter = [0., 0., 0.]
    if len(selectedPoints) != 0:
            poscenter = sdiv(sum(selectedPoints), float(len(selectedPoints)))
    return poscenter
def Rigidify(targetObject, sourceObject, groupIndices, frames=None, name=None):
    if frames is None:
        frames = [[0., 0., 0.]]*len(groupIndices)

    assert len(groupIndices) == len(frames), "size mismatch."

    if name is None:
        name = sourceObject.name

    # sourceObject.reinit()
    ero = targetObject.addChild(name)

    allPositions = sourceObject.container.position.value
    allIndices =list(range(len(allPositions)))

    rigids = []
    indicesMap = []

    def mfilter(si, ai, pts):
            tmp = []
            for i in ai:
                    if i in si:
                            tmp.append(pts[i])
            return tmp

    # get all the points from the source.
    selectedIndices = []
    for i in range(len(groupIndices)):
            selectedPoints = mfilter(groupIndices[i], allIndices, allPositions)
            if len(frames[i]) == 3:
                    orientation = Quat.createFromEuler(frames[i], inDegree=True)
                    poscenter = getBarycenter(selectedPoints)
            elif len(frames[i]) == 4:
                    orientation = frames[i]
                    poscenter = getBarycenter(selectedPoints)
            elif len(frames[i]) == 6:
                    orientation = Quat.createFromEuler([frames[i][3], frames[i][4], frames[i][5]], inDegree=True)
                    poscenter = [frames[i][0], frames[i][1], frames[i][2]]
            elif len(frames[i]) == 7:
                    orientation = [frames[i][3], frames[i][4], frames[i][5], frames[i][6]]
                    poscenter = [frames[i][0], frames[i][1], frames[i][2]]
            else:
                    Sofa.msg_error("Do not understand the size of a frame.")

            rigids.append(poscenter + list(orientation))

            selectedIndices += map(lambda x: x, groupIndices[i])
            indicesMap += [i] * len(groupIndices[i])

    otherIndices = list(filter(lambda x: x not in selectedIndices, allIndices))
    Kd = {v: None for k, v in enumerate(allIndices)}
    Kd.update({v: [0, k] for k, v in enumerate(otherIndices)})
    Kd.update({v: [1, k] for k, v in enumerate(selectedIndices)})
    indexPairs = [v for kv in Kd.values() for v in kv]

    freeParticules = ero.addChild("DeformableParts")
    freeParticules.addObject("MechanicalObject", template="Vec3", name="dofs",
                                position=[allPositions[i] for i in otherIndices])

    rigidParts = ero.addChild("RigidParts")
    rigidParts.addObject("MechanicalObject", template="Rigid3", name="dofs", reserve=len(rigids), position=rigids)

    rigidifiedParticules = rigidParts.addChild("RigidifiedParticules")
    rigidifiedParticules.addObject("MechanicalObject", template="Vec3", name="dofs",
                                        position=[allPositions[i] for i in selectedIndices])
    rigidifiedParticules.addObject("RigidMapping", name="mapping", globalToLocalCoords=True, rigidIndexPerPoint=indicesMap)

    if "solver" in sourceObject.objects:
        sourceObject.removeObject(sourceObject.solver)
    if "integration" in sourceObject.objects:
        sourceObject.removeObject(sourceObject.integration)
    if "correction" in sourceObject.objects:
        sourceObject.removeObject(sourceObject.correction)

    sourceObject.addObject("SubsetMultiMapping", name="mapping", template="Vec3,Vec3",
                            input=[freeParticules.dofs.getLinkPath(),rigidifiedParticules.dofs.getLinkPath()],
                            output=sourceObject.dofs.getLinkPath(),
                            indexPairs=indexPairs)

    rigidifiedParticules.addChild(sourceObject)
    freeParticules.addChild(sourceObject)
    return ero
##########################################
# CableGeometry positions of every point #
##########################################
cableGeometry1 = [[ -length1,0, 10.],[-cablelength,0, 13],[-cablelength,0, 15],[-cablelength,0, 18],
                  [-cablelength,0, 20],[-cablelength,0, 23],[-cablelength,0, 25],[-cablelength,0, 28],
                  [-cablelength,0,30],[-cablelength,0,33],[-cablelength,0,35],[-cablelength,0,38],
                  [-cablelength,0, 40],[-cablelength,0,43],[-cablelength,0,45],[-cablelength,0,48],
                  [-cablelength,0,50],[-cablelength,0,53],[-cablelength,0,55],[-cablelength,0,58],[-cablelength,0,60]]

cableGeometry2 = [[ length1*cos(pi/3),-length1*cos(pi/6), 10.],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 13],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 15],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 18],
                  [cablelength*cos(pi/3),-cablelength*cos(pi/6), 20],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 23],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 25],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 28],
                  [cablelength*cos(pi/3),-cablelength*cos(pi/6),30],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 33],[cablelength*cos(pi/3),-cablelength*cos(pi/6),35],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 38],
                  [cablelength*cos(pi/3),-cablelength*cos(pi/6), 40],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 43],[cablelength*cos(pi/3),-cablelength*cos(pi/6),45],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 48],
                  [cablelength*cos(pi/3),-cablelength*cos(pi/6),50],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 53],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 55],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 58],[cablelength*cos(pi/3),-cablelength*cos(pi/6), 60]]

cableGeometry3 = [[ length1*cos(pi/3),length1*cos(pi/6), 10.],[cablelength*cos(pi/3),cablelength*cos(pi/6), 13],[cablelength*cos(pi/3),cablelength*cos(pi/6), 15],[cablelength*cos(pi/3),cablelength*cos(pi/6), 18],
                  [cablelength*cos(pi/3),cablelength*cos(pi/6), 20],[cablelength*cos(pi/3),cablelength*cos(pi/6), 23],[cablelength*cos(pi/3),cablelength*cos(pi/6), 25],[cablelength*cos(pi/3),cablelength*cos(pi/6), 28],
                  [cablelength*cos(pi/3),cablelength*cos(pi/6),30],[cablelength*cos(pi/3),cablelength*cos(pi/6), 33],[cablelength*cos(pi/3),cablelength*cos(pi/6),35],[cablelength*cos(pi/3),cablelength*cos(pi/6), 38],
                  [cablelength*cos(pi/3),cablelength*cos(pi/6), 40],[cablelength*cos(pi/3),cablelength*cos(pi/6), 43],[cablelength*cos(pi/3),cablelength*cos(pi/6),45],[cablelength*cos(pi/3),cablelength*cos(pi/6), 48],
                  [cablelength*cos(pi/3),cablelength*cos(pi/6),50],[cablelength*cos(pi/3),cablelength*cos(pi/6), 53],[cablelength*cos(pi/3),cablelength*cos(pi/6), 55],[cablelength*cos(pi/3),cablelength*cos(pi/6), 60]]

##########################################
# Setting a target in Inverse mode       #
##########################################
def effectorTarget(parentNode, position=target_position):
    target = parentNode.addChild('Target')
    target.addObject('EulerImplicitSolver', firstOrder=True)
    target.addObject('CGLinearSolver')
    target.addObject('MechanicalObject', name='dofs', position=position, showObject=True, showObjectScale=2, drawMode=2, showColor=[1., 1., 1., 1.])
    target.addObject('UncoupledConstraintCorrection')
    return target
def effectorTarget2(parentNode, position=[0., -80,170]):
    target2 = parentNode.addChild('Target2')
    target2.addObject('EulerImplicitSolver', firstOrder=True)
    target2.addObject('CGLinearSolver')
    target2.addObject('MechanicalObject', name='dofs', position=position, showObject=True, showObjectScale=2, drawMode=2, showColor=[1., 1., 1., 1.])
    target2.addObject('UncoupledConstraintCorrection')
    return target2
##########################################
# Throat                                 #
##########################################
class Throat():
    #Setting basic properties about your throat robot
    def __init__(self, parentNode, youngModulus=8000, poissonRatio=0.45, totalMass=0.07):  
        self.node = parentNode.addChild('Throat')
        #Loading vtk file and some necessary steps
        self.node.addObject('MeshVTKLoader', name='loader', filename=path+'simpleModel2.vtk')
        self.node.addObject('MeshTopology', src='@loader', name='container')
        self.node.addObject('MechanicalObject', name='dofs', template='Vec3')
        self.node.addObject('UniformMass', totalMass=totalMass)
        self.node.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=poissonRatio,  youngModulus=youngModulus)
        #Adding cables to your robot
        self.__addCables()


    def __addCables(self):
        #Define a cable      
        #1.add a child node
        cable1 = self.node.addChild("cable1")
        #2.create a mech object and set position
        cable1.addObject("MechanicalObject", name='dof1', position=cableGeometry1)
        #3.add actuator for inverse mode and set params
        cable1.addObject('CableActuator', template='Vec3', name='CableActuator',
                        indices=list(range(len(cableGeometry1))),
                        maxPositiveDisp='50',
                        maxDispVariation='1.5',
                        hasPullPoint='0',
                        minForce=0)
        #4.mapping
        cable1.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

        cable2 = self.node.addChild("cable2")
        cable2.addObject("MechanicalObject", name='dof2', position=cableGeometry2)
        cable2.addObject('CableActuator', template='Vec3', name='CableActuator',
                        indices=list(range(len(cableGeometry2))),
                        maxPositiveDisp='50',
                        maxDispVariation='1.5',
                        hasPullPoint='0',
                        minForce=0)
        cable2.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

        cable3 = self.node.addChild("cable3")
        cable3.addObject("MechanicalObject", name='dof3', position=cableGeometry3)
        cable3.addObject('CableActuator', template='Vec3', name='CableActuator',
                        indices=list(range(len(cableGeometry3))),
                        hasPullPoint='0',
                        maxPositiveDisp='50',
                        maxDispVariation='1.5',
                        minForce=0)
        cable3.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

        
    #Adding collision model to the your robot 
    def addCollisionModel(self, selfCollision=False):
        throatColli = self.node.addChild('CollisionModel')
        throatColli.addObject('MeshSTLLoader', name='loader', filename=path+'simpleModel2.stl')
        throatColli.addObject('MeshTopology', src='@loader')
        throatColli.addObject('MechanicalObject')
        #different collision model 
        #the models in different will have interaction
        throatColli.addObject('TriangleCollisionModel', contactStiffness=100, group=1,selfCollision=False)
        throatColli.addObject('LineCollisionModel' , contactStiffness=100, group=1,selfCollision=False)
        throatColli.addObject('PointCollisionModel', contactStiffness=100, group=1 ,selfCollision=False)
        throatColli.addObject('BarycentricMapping')
        
    #Adding visual model to the your robot 
    def addVisualModel(self, color):
        throatVisu = self.node.addChild('VisualModel')
        throatVisu.addObject('MeshSTLLoader', filename=path+'simpleModel2.stl')
        throatVisu.addObject('OglModel', color=color)
        throatVisu.addObject('BarycentricMapping')
    
    #Adding an effectors to your robot: It's the decision point 
    def addEffectors(self, target, position,rootNode):
        #Using drawing circle mode, just using this animation and activate the animate
        effectors = self.node.addChild('Effectors')
        effectors.addObject('MechanicalObject', position=position,showVectors=False,showVectorsScale=1)
        effectors.addObject('PositionEffector', indices=list(range(len(position))), effectorGoal=target_position)
        effectors.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
        effectors.addObject('Monitor', indices=0, showTrajectories=True,showPositions=False,TrajectoriesColor=[0,0,1,1],sizeFactor=10)

def __regidify(throat,parentNode):
    parentNode=parentNode
    frames = None
    deformableObject=throat
    rigidifiedstruct = Rigidify(parentNode, deformableObject, groupIndices=groupIndices, frames=frames,name="RigidifiedStructure")

def createScene(rootNode):
    #RequiredPlugins
    rootNode.addObject('RequiredPlugin', pluginName=["SoftRobots.Inverse",'SoftRobots','SofaValidation','Sofa.GL.Component.Rendering2D','CImgPlugin','Sofa.Component.SolidMechanics.Spring','SofaMeshCollision','ArticulatedSystemPlugin','Sofa.Component.Mapping.MappedMatrix','Sofa.Component.Topology.Container.Dynamic','SofaSparseSolver','SofaPreconditioner','SofaPython3','SofaConstraint',
                                                     'SofaImplicitOdeSolver','SofaLoader','SofaSimpleFem','SofaBoundaryCondition','SofaEngine',
                                                     'SofaOpenglVisual','Sofa.Component.IO.Mesh','Sofa.Component.LinearSolver.Direct','Sofa.Component.LinearSolver.Iterative',
                                                     'Sofa.Component.Mass','Sofa.Component.ODESolver.Backward','Sofa.Component.SceneUtility',
                                                     'Sofa.Component.StateContainer','Sofa.Component.Topology.Container.Constant','Sofa.Component.Visual',
                                                     'Sofa.GL.Component.Rendering3D','Sofa.Component.SolidMechanics.FEM.Elastic','Sofa.Component.AnimationLoop',
                                                     'Sofa.Component.Collision.Detection.Algorithm','Sofa.Component.Collision.Detection.Intersection',
                                                     'Sofa.Component.Collision.Geometry','Sofa.Component.Collision.Response.Contact','Sofa.Component.Constraint.Lagrangian.Correction',
                                                     'Sofa.Component.Constraint.Projective','Sofa.Component.Mapping.Linear','Sofa.Component.Engine.Select',
                                                     'Sofa.Component.Setting','Sofa.Component.Constraint.Lagrangian.Solver','Sofa.Component.Constraint.Lagrangian.Model'])

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='showBehavior')
    #Set gravity of your scene
    rootNode.gravity = [0., -9.810, 0]
    rootNode.addObject(AnimationManager(rootNode))
    rootNode.addObject('FreeMotionAnimationLoop')    
    rootNode.addObject('QPInverseProblemSolver',epsilon=1e-1)
    simulation = rootNode.addChild('Simulation')
    #Necessary things
    simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    simulation.addObject('ShewchukPCGLinearSolver', name='linearSolver', iterations=500, tolerance=1.0e-18, preconditioners='precond')
    simulation.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d", name='precond')
    simulation.addObject('GenericConstraintCorrection', solverName='precond')

    #Define your robot, add visual model and collision model
    throat = Throat(simulation)
    throat.addVisualModel(color=[1., 1., 1., 0.8])
    throat.addCollisionModel()
    
    #Regidify your robot into two parts
    __regidify(simulation.Throat,simulation)xDispVariation=1.5,maxPositiveDisp=40,maxNegativeDisp=-40)
    
    simulation.addObject('MechanicalMatrixMapper',
                                template='Vec3,Rigid3',
                                name="RigidAndDeformableCoupling",
                                object1=simulation.RigidifiedStructure.DeformableParts.dofs.getLinkPath(),
                                object2=simulation.RigidifiedStructure.RigidParts.dofs.getLinkPath(),
                                nodeToParse=simulation.RigidifiedStructure.DeformableParts.Throat.getLinkPath())
    
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContact", responseParams="mu=0")
    rootNode.addObject('LocalMinDistance', alarmDistance=2, contactDistance=0.5)
    
    #Set a target for your robot and activate your effector on robot
    target = effectorTarget(rootNode)
    throat.addEffectors(target=target.dofs.getData('position').getLinkPath(), position=robot_decisionpoint, rootNode=rootNode)
    
    simulation.Throat.addObject(ThroatController(rootNode.Simulation.Throat.cable1,rootNode.Simulation.Throat.cable2,rootNode.Simulation.Throat.cable3,rootNode,rootNode.Simulation.RigidifiedStructure.RigidParts.dofs))
    FixedBox=simulation.Throat.addChild('FixedBox')
    FixedBox.addObject('BoxROI', name='BoxROI', box=[[8, 8, 0], [-8, -8, 10]], drawBoxes=True,doUpdate=False)
    FixedBox.addObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')
