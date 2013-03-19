#-
# ==========================================================================
# Copyright (C) 1995 - 2006 Autodesk, Inc. and/or its licensors.  All 
# rights reserved.
#
# The coded instructions, statements, computer programs, and/or related 
# material (collectively the "Data") in these files contain unpublished 
# information proprietary to Autodesk, Inc. ("Autodesk") and/or its 
# licensors, which is protected by U.S. and Canadian federal copyright 
# law and by international treaties.
#
# The Data is provided for use exclusively by You. You have the right 
# to use, modify, and incorporate this Data into other products for 
# purposes authorized by the Autodesk software license agreement, 
# without fee.
#
# The copyright notices in the Software and this entire statement, 
# including the above license grant, this restriction and the 
# following disclaimer, must be included in all copies of the 
# Software, in whole or in part, and all derivative works of 
# the Software, unless such copies or derivative works are solely 
# in the form of machine-executable object code generated by a 
# source language processor.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. 
# AUTODESK DOES NOT MAKE AND HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED 
# WARRANTIES INCLUDING, BUT NOT LIMITED TO, THE WARRANTIES OF 
# NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR 
# PURPOSE, OR ARISING FROM A COURSE OF DEALING, USAGE, OR 
# TRADE PRACTICE. IN NO EVENT WILL AUTODESK AND/OR ITS LICENSORS 
# BE LIABLE FOR ANY LOST REVENUES, DATA, OR PROFITS, OR SPECIAL, 
# DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES, EVEN IF AUTODESK 
# AND/OR ITS LICENSORS HAS BEEN ADVISED OF THE POSSIBILITY 
# OR PROBABILITY OF SUCH DAMAGES.
#
# ==========================================================================
#+

# import maya.cmds as cmds
# cmds.createNode("transform", name="ccdNode1")
# cmds.createNode("mesh", name="ccdNodeShape1", parent="ccdNode1")
# cmds.sets("ccdNodeShape1", add="initialShadingGroup")
# cmds.createNode("spccdNode", name="ccdNodeNode1")
# cmds.connectAttr("time1.outTime", "ccdNodeNode1.time")
# cmds.connectAttr("ccdNodeNode1.outputMesh", "ccdNodeShape1.inMesh")



cmds.createNode("transform", name="joint4")
cmds.connectAttr("time1.outTime", "joint4.time")
# cmds.createNode("spccdNode", name="ccdNodeNode1")
# cmds.connectAttr("time1.outTime", "ccdNodeNode1.time")
# cmds.connectAttr("ccdNodeNode1.outputMesh", "ccdNodeShape1.inMesh")


''' GET POSITION OF JOINT ''' # =====> cmds.getAttr('joint4.translate')
#To get position of joint in WS-> pos = cmds.getAttr(<objectName>.translate)
#return value is an array of size 1 containing an array of float values
# x-coord = pos[0][0]
# y-coord = pos[0][1]
# z-coord = pos[0][1]

''' SET POSITION OF JOINT ''' # =====> cmds.setAttr('joint4.translate',)
# setAttr object.translateY 10.5;
# or:
# setAttr object.translate -1.5 10.5 0;



import sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx
import Leap, string, math

kPluginNodeName = "spCCDNode"
kPluginNodeId = OpenMaya.MTypeId(0x8700B)

class ccdNode(OpenMayaMPx.MPxNode):
    time = OpenMaya.MObject()
    outputMesh = OpenMaya.MObject()

    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def createMesh(self, tempTime, outData):
        frame = int(tempTime.asUnits(OpenMaya.MTime.kFilm))
        if frame is 0:
            frame = 1

        cubeSize = 0.5 * float(frame % 10)

        numFaces = 6
        numVertices = 8
        numFaceConnects = 24

        vtx_1 = OpenMaya.MFloatPoint(-cubeSize, -cubeSize, -cubeSize)
        vtx_2 = OpenMaya.MFloatPoint( cubeSize, -cubeSize, -cubeSize)
        vtx_3 = OpenMaya.MFloatPoint( cubeSize, -cubeSize,  cubeSize)
        vtx_4 = OpenMaya.MFloatPoint(-cubeSize, -cubeSize,  cubeSize)
        vtx_5 = OpenMaya.MFloatPoint(-cubeSize,  cubeSize, -cubeSize)
        vtx_6 = OpenMaya.MFloatPoint(-cubeSize,  cubeSize,  cubeSize)
        vtx_7 = OpenMaya.MFloatPoint( cubeSize,  cubeSize,  cubeSize)
        vtx_8 = OpenMaya.MFloatPoint( cubeSize,  cubeSize, -cubeSize)

        points = OpenMaya.MFloatPointArray()
        points.setLength(8)
        points.set(vtx_1, 0)
        points.set(vtx_2, 1)
        points.set(vtx_3, 2)
        points.set(vtx_4, 3)
        points.set(vtx_5, 4)
        points.set(vtx_6, 5)
        points.set(vtx_7, 6)
        points.set(vtx_8, 7)

        faceConnects = OpenMaya.MIntArray()
        faceConnects.setLength(numFaceConnects)
        faceConnects.set(0, 0)
        faceConnects.set(1, 1)
        faceConnects.set(2, 2)
        faceConnects.set(3, 3)
        faceConnects.set(4, 4)
        faceConnects.set(5, 5)
        faceConnects.set(6, 6)
        faceConnects.set(7, 7)
        faceConnects.set(3, 8)
        faceConnects.set(2, 9)
        faceConnects.set(6, 10)
        faceConnects.set(5, 11)
        faceConnects.set(0, 12)
        faceConnects.set(3, 13)
        faceConnects.set(5, 14)
        faceConnects.set(4, 15)
        faceConnects.set(0, 16)
        faceConnects.set(4, 17)
        faceConnects.set(7, 18)
        faceConnects.set(1, 19)
        faceConnects.set(1, 20)
        faceConnects.set(7, 21)
        faceConnects.set(6, 22)
        faceConnects.set(2, 23)

        faceCounts = OpenMaya.MIntArray()
        faceCounts.setLength(6)
        faceCounts.set(4, 0)
        faceCounts.set(4, 1)
        faceCounts.set(4, 2)
        faceCounts.set(4, 3)
        faceCounts.set(4, 4)
        faceCounts.set(4, 5)

        meshFS = OpenMaya.MFnMesh()
        newMesh = meshFS.create(numVertices, numFaces, points, faceCounts, faceConnects, outData)

        return newMesh

    def compute(self, plug, data):
        if plug == ccdNode.outputMesh:
            timeData = data.inputValue(ccdNode.time)
            tempTime = timeData.asTime()

            outputHandle = data.outputValue(ccdNode.outputMesh)

            dataCreator = OpenMaya.MFnMeshData()
            newOutputData = dataCreator.create()

            self.createMesh(tempTime, newOutputData)

            outputHandle.setMObject(newOutputData)
            data.setClean(plug)
        else:
            return OpenMaya.kUnknownParameter

def nodeCreator():
    return OpenMayaMPx.asMPxPtr( ccdNode() )

def nodeInitializer():
    unitAttr = OpenMaya.MFnUnitAttribute()
    typedAttr = OpenMaya.MFnTypedAttribute()

    ccdNode.time = unitAttr.create("time", "tm", OpenMaya.MFnUnitAttribute.kTime, 0.0)
    #ccdNode.outputMesh = typedAttr.create("outputMesh", "out", OpenMaya.MFnData.kMesh)

    ccdNode.addAttribute(ccdNode.time)
    #ccdNode.addAttribute(ccdNode.outputMesh)

    ccdNode.attributeAffects(ccdNode.time, ccdNode.outputMesh)


# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeName, kPluginNodeId, nodeCreator, nodeInitializer)
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( kPluginNodeId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeName )
        raise


JOINTS = {
    # 'joint1': {
    #     'base-pos': Leap.Vector(0, 0, 0),
    #     'pos': Leap.Vector(0, 0, 0),
    #     'rot': Leap.Vector(0, 90, 0),
    #     'name': 'palm'
    # },
    # 'joint2': {
    #     'base-pos': Leap.Vector(0, 0, -5),
    #     'pos': Leap.Vector(0, 0, -5),
    #     'rot': Leap.Vector(0, 0, 0),
    #     'name': 'knuckle'
    # },
    # 'joint3': {
    #     'base-pos': Leap.Vector(0, 0, -10),
    #     'pos': Leap.Vector(0, 0, -10),
    #     'rot': Leap.Vector(0, 0, 0),
    #     'name': 'mid-joint'
    # },
    'joint4': {
        'base-pos': Leap.Vector(0, 0, -15),
        'pos': Leap.Vector(0, 0, -15),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'tip-joint'
    },
    'joint5': {
        'base-pos': Leap.Vector(0, 0, -20),
        'pos': Leap.Vector(0, 0, -20),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'end-effector'
    }
}

#CCD algorithm - with a targetPos
def perform_ccd(self, targetTipPos):
    print "ccd test"
    effector = 'joint5'
    effectorPos = JOINTS[effector]['pos']
    error = effectorPos.distance_to(targetTipPos)
    iterations = 4
    while (error > 0.5 or iterations > 0):
        i = 4
        while (i > 1):
            #Loop through each joint and update the joint positions
            #Pe - position of the end effector 
            pEnd = JOINTS[effector]['pos']

            #Pc - distance between the joint position and end effector position
            jointKey = 'joint'+str(i)
            pBase = JOINTS[jointKey]['pos']

            #pT - target position
            pT = targetTipPos

            #peToPc
            pE_pC = (pEnd-pBase).normalized
            pT_pC = (pT-pBase).normalized
            #Angle of rotation
            try:  
                theta = math.acos(Leap.Vector.dot(pE_pC, pT_pC))
            except ValueError, e:
                break

            #Axis of Rotation
            rAxis = Leap.Vector.cross(pE_pC, pT_pC)
            #Get Rotation Matrix from Axis & Angle
            rotMat = Leap.Matrix(rAxis, theta)
            rMat = rotMat.to_array_3x3()

            phi = math.atan2(rMat[6], rMat[7]) * Leap.RAD_TO_DEG
            theta = math.acos(rMat[8]) * Leap.RAD_TO_DEG
            psi = -1*math.atan2(rMat[2], rMat[5]) * Leap.RAD_TO_DEG

            #Update pBase by rotating it
            #pBase = rotMat.transform_point(pBase)
            #JOINTS[jointKey]['pos'] = pBase
            #Update the position of our end effector
            JOINTS[effector]['pos'] = rotMat.transform_point(pEnd)

            #Update rotation for with the current rot current
            for j in range(i, 4):
                rotKey = 'joint' + str(j)
                JOINTS[jointKey]['pos'] =  rotMat.transform_point(JOINTS[jointKey]['pos'])


           # maya.rotate(jointKey, phi, theta, psi)
            
            #Move to next Joint
            i -= 1
        #Check error
        effectorPos = JOINTS[effector]['pos']
        error = effectorPos.distance_to(targetTipPos)
        iterations -= 1
    #Turn back on frame updates
    #Update pos for each joint in maya
    for i in range(2, 5):
        jointKey = 'joint' + str(i)
        pos = JOINTS[jointKey]['pos']
        maya.move(jointKey, pos.x, pos.y, pos.z)
    self.is_peforming_ccd = False
    #time.sleep(1)  
