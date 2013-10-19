#!/usr/bin/python
# -*- coding: utf-8 -*- 
'''
'' PAALM Plug-in  - for tracking hand data using the Leap Motion Controller
'' @author: Michael Rivera
'' @date: 10/17/2013
''  
'''

# System Imports
import sys, string, time, ast
import math as Math

# Maya Imports
import maya
import maya.cmds as cmds
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx
import pymel.all as pm
from pymel.core import * 

# PyMel Data Types for Matrices, Quaternions etc
import pymel.core.datatypes as dt

# Handles function callbacks for commands
from functools import partial

# Leap Imports
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture



###############################################################
'''############################################################
'' ## PAALM OPTIONS ###########################################
'' ############################################################
'''

## Degrees-to-Radians ##
DEG_TO_RAD = Math.pi/180

## Epsilon for float comparison ##
EPSILON = 0.001

## PAALM Configuration ##
CONFIG = {
    'JOINT_ANGLE_MULTIPLIER': 0.5,
    'INITIAL_MAX_TIME': 30000,
    'MAX_TIME_INCREMENT': 200,
    'MIN_TIME': 1,
    'TIME_INCREMENT': 15,
    'PALM_INDEX': 9999,
    'MIN_CLEAR_FRAME': 2
}

## Defines if we use direction vectors from the Leap. ##
## If false, we use raw positions ##
USE_LEAP_DIRECTION = True

## Defines if we should use CCD for IK Targets ##
USE_CDD = False

## Defines the max Distance for CCD thresholding ## 
CCD_DISTANCE_THRESHOLD = 0.1

## Defines if we move from 0->MaxJountNum for CCD opposed ##
## to decrement joint numbers ##
CCD_USE_INCREASE_JOINT_NUM = False

## Defines if a Character is being manipulated ##
USE_CHARACTER = True

## Defines whether we use quaternions for rotations ##
USE_QUATERNIONS = True

############################################################
#################### FINGER TEST CONFIG ####################
############################################################
''' 
'' DO NOT EDIT THESE CONFIGS BELOW
'''
## List of Fingers Being Used for IK ##
PAALM_FINGERS = []

## List of Hands Being Used for IK ##
PAALM_HANDS = []

## Is Tracking Hands from command port ##
IS_TRACKING = True

## Is Tracking Palm from command port ##
IS_TRACKING_PALM_POSITION = True

## The type assigned to the Hand Demo Joint Set up ##
HAND_DEMO_TYPE = -1

## The type assigned to the Character Demo Joint Set up ##
CHARACTER_DEMO_TYPE = -2

## The type assigned to the Octopus Demo Joint Set up ##
OCTOPUS_DEMO_TYPE = -3

## Active Demo Type - the type of demo that is being used ##
ACTIVE_DEMO_TYPE = HAND_DEMO_TYPE

## Maya Command Port ##
MAYA_PORT = 6001
MAYA_PORT_NAME = ':6001'

'''#############################################################
'' ############## SOCKET COMMAND PORT FOR MAYA #################
'' #############################################################
'''
#create Command port - sets port up to receive data on the mel function for receiveTipPos
# This then forwards the data to the python function
def open_command_port():
    #Check if our port is already open if it isn't open it
    for i in pm.commandPort(q=True,lp=True):
        if (i == MAYA_PORT_NAME):
            #Port already exists
            print 'PAALM Port Exists!'
            return
    #Otherwise open the port`
    pm.commandPort(name=MAYA_PORT_NAME)
    print 'PAALM Port Opened!'

def close_command_port():
    pm.commandPort(close=True, name=MAYA_PORT_NAME)

'''############################################################
'' ## PAALM JOINT SYSTEMS / IK  ###############################
'' ############################################################
'''


############################################################
############################ HAND ##########################
############################################################
class Hand(object):
    def __init__(self, handID, fingers=[], palm=None):
        self.handID = handID
        self.fingers = fingers
        self.palm = palm

    # Returns the output that is returned when using the print function
    def __str__(self):
        return '<# HandID:\'%s\' # %s #>' % (self.handID, self.fingerList)
    
    def __repr__(self):
        return self.__str__()

    # Sets a list of fingers to this hand
    def set_fingers(self, fingers):
        self.fingers = fingers

    # Adds a finger to this fingers list
    def add_finger(self, finger):
        self.fingers.append(finger) 

    # Returns true if the hand has at least 1 finger
    def has_fingers(self):
        return (len(self.fingers) > 0)

    #Sets the palm on the hand
    def set_palm(self, palm):
        if (isinstance(palm, Joint)):
            self.palm = palm

    #Returns true if this hand has a palm
    def has_palm(self):
        return self.palm is not None

    def clear_keyframes(self, currentMaxTime=0):
        if (currentMaxTime == 0):
            currentMaxTime = get_max_time()
        for finger in self.fingers:
            finger.clear_keyframes(currentMaxTime)
                #Clear All the key frames for every joint
        #pm.refresh(force=True)
        if (self.has_palm):
            self.palm.clear_keyframes(currentMaxTime)


    # #Compute length between the palm and wrist
    # def calculate_palm_to_wrist_length(self):
    #     if (self.has_palm() and self.has_wrist()):
    #         palmToWristLength = (self.palm.get_position()-self.wrist.get_position()).magnitude
    #         return palmToWristLength
    #     else:
    #         #No length
    #         return 0

    #Set the target positions for their corresponding fingers by 
    # def update_target_positions(self, targetPositions):
    #     #For each Target position perform CCD on that finger
    #     for tp in targetPositions:
    #         fingerIndex = tp['fingerIndex']
    #         #Check if the finger is valid for this hand
    #         if (fingerIndex < len(self.fingerList)):
    #             #Get the finger corresponding to this index
    #             finger = self.fingerList[fingerIndex]
    #             #Set the target position for this finger
    #             finger.set_target_position(tp)
    #         else:
    #             #Do nothing for that index
    #             print 'Invalid finger index'


############################################################
########################### FINGER #########################
############################################################
class Finger(object):
    def __init__(self, mayaID, target=None, jointList=[]):
        self.mayaID = mayaID
        self.length = 0
        self.hasTarget = False
        self.target = None
        self.jointList = []

        self.add_joints(jointList)
        self.set_target(target)
        self.calculate_length()
        #self.currentTime = 0


    #Returns the output that is returned when using the print function
    def __str__(self):
        return '<# FingerID:\'%s\' # %s #>' % (self.mayaID, self.jointList)
    
    def __repr__(self):
        return self.__str__()

    #Returns the joint list associated with this finger
    def get_joints(self):
        return self.jointList

    #Returns the number of joints in the finger
    def get_num_joints(self):
        return len(self.get_joints())

    #Adds a joint to this finger's joint list - returns true is successful
    def add_joint(self, joint):
        #print isinstance(joint, Joint)
        if (isinstance(joint, Joint)):
            self.jointList.append(joint)
            self.calculate_length()
            return True
        else:
            return False

    #Adds a list of joints to this finger
    def add_joints(self, joints):
        success = True
        for joint in joints:
            success = self.add_joint(joint) and success
        self.calculate_length()
        return success


    #Sets a key frame on all of the joints for this finger at the current time
    def set_keyframe(self, currentTime):
        #Get the Current Time
        #currentTime = get_current_time()
        for joint in self.jointList:
            joint.set_keyframe(currentTime)

        #Clear target keyframes
        if (self.has_target()):
            self.target.set_keyframe(currentTime)
    

    #Clears all key frames on the finger's joints from
    def clear_keyframes(self, currentMaxTime=0):
        #currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
        #self.currentTime = 1
        if (currentMaxTime == 0):
            currentMaxTime = get_max_time()
        for joint in self.jointList:
            #print joint
            joint.clear_keyframes(currentMaxTime)
                #Clear All the key frames for every joint
        #pm.refresh(force=True)
        if (self.has_target()):
            self.target.clear_keyframes(currentMaxTime)


    #Sets the target object that is in maya for this finger
    def set_target(self, target):
        if (isinstance(target, Target)):
            self.target = target
            self.hasTarget = True
        else:
            self.target = None
            self.hasTarget = False
        #Update all joint and target attributes
        self.update()
        self.calculate_length()

    #Returns true if there is a target assigned to this finger
    def has_target(self):
        return self.hasTarget

    #Returns the target object
    def get_target(self):
        if (self.has_target()):
            return self.target
        else:
            print 'No Target on Finger: %s' % self
            return None

    #Gets the target position or a zero vector if the target is not set
    def get_target_position(self):
        if (self.has_target()):
            return self.target.get_position()
        else:
            print 'No target defined'
            #TODO: Return the end effector's current position
            return Leap.Vector(0,0,0)
    #Sets the target position for the finger 
    def set_target_position(self, position):
        if (self.has_target()):
            self.target.set_position(position)
            #self.calculate_length()
            return True
        else:
            print "Cannot Set Target Position!"
            return False 
            
    #Gets the effector of the joint chain (last joint in the chain)
    def get_effector(self):
        joints = self.get_joints()
        numJoints = self.get_num_joints()
        if (numJoints > 0):
            return joints[numJoints-1]
        else:
            print 'No Effector!'
            return None


   

    #Updates all joint and target positiings for this finger 
    def update(self):
        #Update all joints
        for joint in self.get_joints():
            # print 'joint-update: %s' % joint
            joint.update()
        #Update the Target
        if self.has_target():
            # print 'target-update'
            self.target.update() 
    
    #Calculate the distance between the palm and the first joint of the finger
    def calculate_palm_distance(self,palm):
        numJoints = self.get_num_joints()
        if (numJoints > 0):
            palmDistance = (palm.get_position() - self.jointList[0]).magnitude
            return palmDistance
        else:
            return 0
    #Gets the length of the finger with respect the segments between the ordered joints
    def get_length(self):
        return self.calculate_length()

    #Calculate the length of the finger
    def calculate_length(self):
        self.update()
        # wristPos =  palmPos = pm.xform('hand4_joint1', query=True, ws=True, t=True)
        # wristPos =  Leap.Vector(wristPos[0], wristPos[1], wristPos[2])
        # #print wristPos

        # palmPos = pm.xform('hand4_joint1|joint2', query=True, ws=True, t=True) #//in an array [x,y,z]
        # palmPos =  Leap.Vector(palmPos[0], palmPos[1], palmPos[2])
        # #print palmPos

        # #Get initial palm-to-finger length
        # isFirst = True

        # #Sum of Segment lengths
        # chainLength = 0

        # #Include palm and wrist vect mag
        # chainLength = (palmPos - wristPos).magnitude

        chainLength = 0 #### REMOVE THIS AND UNCOMMENT TOP when ready
        #print chainLength

        #Get the number of joints 
        numJoints = self.get_num_joints()
        #Now get length of vectors between sequences of two joints. If we do not
        #Have a number of joints > 1, there can be no segments 
        if (numJoints > 1) :
            for index in range(0, numJoints-1):
                #print index
                firstKey = index
                secondKey = index + 1

                firstJoint = self.jointList[index]
                secondJoint = self.jointList[index+1]

                #print str(firstJoint) + ' - ' + str(secondJoint)
                #Get the segment length between the two joints
                fingerSegment = secondJoint.get_position() - firstJoint.get_position()
                #print fingerSegment
                segmentLength = fingerSegment.magnitude
               # print segmentLength
                #We need to include the firstJoint and its connect to the palm
                # if (isFirst):
                #     #print 'isFirst'
                #     jointSegment = firstJoint.get_position-palmPos
                #     segmentLength += jointSegment.magnitude
                #     isFirst = False

                #print segmentLength
                #Add to the chainLength
                chainLength += segmentLength
    
        #Add an extension so the mapping length is not directly on the sphere
        # chainExtension = 5
        # JOINT_CHAIN_LENGTH +=  chainExtension

        #Set our new Chain length
        self.length = chainLength
        return chainLength


    #CCD algorithm - with a targetPos
    # While distance from effector to target > threshold and numloops<max
        #   Take current bone
        #   Build vector V1 from bone pivot to effector
        #   Build vector V2 from bone pivot to target
        #   Get the angle between V1 and V2
        #   Get the rotation direction
        #   Apply a differential rotation to the current bone
        #   If it is the base node then the new current bone is the last bone in the chain
        #   Else the new current bone is the previous one in the chain
    #End while
    def perform_ccd(self, keyframeTime, palmToWristLength=0):
        #No ccd if no target
        if (self.target is None) or (self.get_num_joints() < 2):
            print 'No Target to Perform CCD On!'
            return False
        else:            

            #Number of iterations to perform per finger
            iterations = 40

            #Update positions for all joints
            self.update()

            #Get the joints and the number of them
            joints = self.get_joints()
            numJoints = self.get_num_joints()

            #Last joint is not the effector so second to last
            currentJointNum = numJoints-2
            maxJointNum = numJoints-2
            minJointNum = 0

            if CCD_USE_INCREASE_JOINT_NUM:
                currentJointNum = minJointNum

            #Effector Position and the Target Position
            effector = self.get_effector()
            target = self.get_target()
            targetTipPos = target.get_position()
            effectorPos = effector.get_position()

            #Distance of the effector to the Target
            distance = (targetTipPos - effectorPos).magnitude


            #Set a Key Frame for the finger
            self.set_keyframe(keyframeTime)
            #CCD Main Loop
            while (distance > CCD_DISTANCE_THRESHOLD and iterations > 0):

                #print distance

                #Joint Angle Multiplier for scaling / smoothness
                if (distance > 1):
                    CONFIG['JOINT_ANGLE_MULTIPLIER'] = 1.0
                else:
                    CONFIG['JOINT_ANGLE_MULTIPLIER'] = 0.4

                #Current Joint & 
                joint = joints[currentJointNum]

                #Update our joint rotation based on the effector Position and the Target Effector Pos
                joint.update_rotation(effectorPos, targetTipPos)
                
                #New Effector Position
                effector.update()
                effectorPos = effector.get_position()

            
                #Update our distance from the target Point
                oldDistance = distance
                distance = (targetTipPos - effectorPos).magnitude
            
                #Check if we have only moved a minimal amount of distance 
                if (Math.fabs(oldDistance-distance) < EPSILON):
                    #Stop CCD
                    break 

                #Decrement Iteration
                iterations = iterations - 1
                
                ############################################
                ######### Keep Joint Num in Bounds #########
                ############################################
                if CCD_USE_INCREASE_JOINT_NUM:
                    #Move to next joint closest to the TP
                    currentJointNum = currentJointNum + 1
                    if currentJointNum > maxJointNum:
                        #keep the joint nums in bounds
                        currentJointNum = minJointNum
                else:
                    #Move onto next joint in the chain
                    currentJointNum = currentJointNum - 1
                    #print currentJointNum
                    #Cycle look if jointNum goes out of bound
                    if (currentJointNum < minJointNum):
                        currentJointNum = maxJointNum
                ############################################

                    #Update all positions of joints in this finger
                self.update()
              
            return True

############################################################
########################### JOINT ##########################
############################################################
class Joint(object):
    #Create a joint
    def __init__(self, mayaID):
        self.mayaID = mayaID
        self.pos = Leap.Vector(0,0,0)
        self.rot = Leap.Vector(0,0,0)
        self.update()

    #Returns the output that is returned when using the print function
    def __str__(self):
        return 'JointID:\'%s\'' % (self.mayaID)

    def __repr__(self):
        return self.__str__()

    #Retrieves updated position and rotation information from Maya
    def update(self):
        #Get position of joint in Maya (world-space)
        mayaPos = pm.xform(self.mayaID, query=True, ws=True, t=True) #//in an array [x,y,z] 
        #Update position reference for joint
        self.pos = Leap.Vector(mayaPos[0], mayaPos[1], mayaPos[2])
        
        #Get Rotatation of joint in Maya
        mayaRot = pm.xform(self.mayaID, query=True, rotation=True)   
        self.rot = Leap.Vector(mayaRot[0], mayaRot[1], mayaRot[2])


    #Get the position of this joint in Maya 
    def get_position(self):
        return self.pos

    #Set the position of this joint in Maya 
    def set_position(self, position):
        pm.xform(self.mayaID, t=(position[0],position[1], position[2]), ws=True)
        self.pos = position
        return True

    #Set the rotation on this joint in Maya
    def set_rotation(self):
        print 'unimplemented rot'

    #Adds an animation key frame to the joint at at it current attributes
    def set_keyframe(self, currentTime):
        pm.setKeyframe(self.mayaID, t=currentTime)

    def clear_keyframes(self, currentMaxTime):
        #currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
        #print currentMaxTime
        pm.cutKey(self.mayaID, time=(CONFIG['MIN_CLEAR_FRAME'], currentMaxTime), option='keys')

    #Get Composite Matrix - include all rotation, translation values
    def get_composite_matrix(self):
        mVals = pm.xform(self.mayaID, query=True, matrix=True, ws=True)

        #Construct matrix from values of our composite matrix
        mat = [ [ float(mVals[0]),  float(mVals[1]),  float(mVals[2]), float(mVals[3])  ], 
                [ float(mVals[4]),  float(mVals[5]),  float(mVals[6]), float(mVals[7])  ],
                [ float(mVals[8]),  float(mVals[9]), float(mVals[10]), float(mVals[11]) ],
                [ float(mVals[12]), float(mVals[13]), float(mVals[14]), float(mVals[15]) ]  ]

        #Turn mat into a transformation Matrix
        mat = dt.TransformationMatrix(dt.Matrix(mat))

        return mat
    #Set Composition Matrix 
    def set_composite_matrix(self, mat):
        #Convert into a maya matrix
        mat = mat.asMatrix()                                   
        matAsFloatTuple = ( mat.a00, mat.a01, mat.a02, mat.a03, 
                            mat.a10, mat.a11, mat.a12, mat.a13,
                            mat.a20, mat.a21, mat.a22, mat.a23,
                            mat.a30, mat.a31, mat.a32, mat.a33
                            )
        #Set the composition matrix on the object
        pm.xform(self.mayaID, ws=True, matrix=matAsFloatTuple) #Removed Euler euler=True because it was rotating about local axes
        #pm.refresh(force=True)


    #Return this joint's current position in World Space
    def get_rot(self):
        return self.rot

    #Computes an optimal rotation for this joint to point towards the targetPosition
    #Applys the rotation via updating the composite matrix
    def update_rotation(self, effectorPos, targetPos):
        #Position of the joiny
        jointPos = self.get_position()

        #The Two Vectors of Interest
        V1 = (effectorPos - jointPos)
        V2 = (targetPos - jointPos)


        if USE_QUATERNIONS:
            #Get Axis & Angle - Returns vector and angle in degrees
            axisAngle = pm.angleBetween(v1=(V1[0], V1[1], V1[2]), v2=(V2[0], V2[1], V2[2]))            

            #Make a quaternion from the axis and the angle
            axis = dt.Vector(axisAngle[0], axisAngle[1], axisAngle[2])

            #Convert Angle to Degrees
            angle =  (axisAngle[3]*DEG_TO_RAD)

            #Apply magic number to the joint to prevent moving too sharply to the targetPoint
            angle *= CONFIG['JOINT_ANGLE_MULTIPLIER'] #For radian 
            rotQuat = dt.Quaternion(angle, axis)

            #Get current matrix - CURRENTLY DOES NOT INCORPORATE SCALE (cutting off values)
            mat = self.get_composite_matrix()

            #Apply Rotation to the matrix
            mat.addRotationQuaternion(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w, space='world')   #object space
            
            #Set the composite matrix for the joint
            self.set_composite_matrix(mat)
        else: 
            #Get Axis & Angle - Returns vector and angle in degrees
            angles = pm.angleBetween(euler=True, v1=(V1[0], V1[1], V1[2]), v2=(V2[0], V2[1], V2[2])) 
            pm.rotate(self.mayaID, rotate=(angles[0],  angles[1], angles[2]))
    #TODO DEFINE FUNCTION FOR CREATING A TARGET POINT In
    #MAYA FOR THIS FINGER on INIT

    #Set Limit of X rotation on Joint
    def set_rotation_limit_x(self, minX, maxX):
        pm.joint(self.mayaID, edit=True, limitX=(minX, maxX))

    #Set Limit of Y rotation on Joint
    def set_rotation_limit_y(self, minY, maxY):
        pm.joint(self.mayaID, edit=True, limitY=(minY, maxY))

    #Set Limit of Z rotation on Joint
    def set_rotation_limit_z(self, minZ, maxZ):
        pm.joint(self.mayaID, edit=True, limitZ=(minZ, maxZ))

    def set_rotation_limits(self, minX, maxX, minY, maxY, minZ, maxZ):
        pm.joint(self.mayaID, edit=True, limitX=(minX, maxX), limitY=(minY, maxY), limitZ=(minZ, maxZ))

############################################################
################## END EFFECTOR (JOINT) ####################
############################################################

class EndEffector(Joint):
    
    def __init__(self, mayaID):
        super(EndEffector, self).__init__(mayaID)

    #Returns the output that is returned when using the print function
    def __str__(self):
        return 'EndEffectorID: %s' % (self.mayaID)

    def __repr__(self):
        return self.__str__()


############################################################
################## TARGET (JOINT/SPHERE) ###################
############################################################

class Target(Joint):
    def __init__(self, mayaID):
        super(Target, self).__init__(mayaID)
    
    #Returns the output that is returned when using the print function
    def __str__(self):
        return  '<# TargetID: \'%s\'' % (self.mayaID)

    def __repr__(self):
        return self.__str__()


    def set_position(self, position):
        super(Target, self).set_position(position)
        self.set_keyframe(get_current_time())

    def update(self):
        #Get the Current Target Position
        mayaPos = pm.xform(self.mayaID, query=True, t=True, ws=True)
        self.pos = Leap.Vector(mayaPos[0], mayaPos[1], mayaPos[2])
        #PRoject to be in range of the armLength
        armLength = 50 #sum of the arm lengths

        sphereCenter = Leap.Vector(0,0,0)
        #self.pos = project_point(self.pos, sphereCenter, armLength)
        #print TARGET_POINT

    #Projects a point on to the Sphere whose radius is the armLenght
    def project_point(point, sphereCenter, armLength):
        sPoint = point - sphereCenter
        #Magnitude of the point
        pMag = sPoint.magnitude

        if (pMag <= armLength):
            #This point is within our sphere so we need not project it
            #print "in range returning same point"
            return point
        #Vector to projected Point
        Q = (point / pMag) * armLength
        #The point projected onto the Sphere
        projectPt = Q + sphereCenter
        return projectPt

    #TODO CREATE SPHERE IN MAYA FOR FINGER

###########################################################
########################### PALM ##########################
###########################################################
class Palm(Joint):
    '''
    '' Create a joint
    '''
    def __init__(self, mayaID):
        self.mayaID = mayaID
        self.pos = Leap.Vector(0,0,0)
        self.rot = Leap.Vector(0,0,0)
        self.update()

    '''
    '' Returns the output that is returned when using the print function
    '''
    def __str__(self):
        return 'PalmID:\'%s\'' % (self.mayaID)

    '''
    '' Returns the string representation of this object
    '''
    def __repr__(self):
        return self.__str__()

    ''' 
    '' Retrieves updated position and rotation information from Maya
    '''
    def update(self):
        # Get position of joint in Maya (world-space)
        mayaPos = pm.xform(self.mayaID, query=True, ws=True, t=True) #//in an array [x,y,z] 
        # Update position reference for joint
        self.pos = Leap.Vector(mayaPos[0], mayaPos[1], mayaPos[2])
        
        # Get Rotatation of joint in Maya
        mayaRot = pm.xform(self.mayaID, query=True, rotation=True)
        
        # Set rotation
        self.rot = Leap.Vector(mayaRot[0], mayaRot[1], mayaRot[2])


    '''
    '' Get the position of this joint in Maya 
    '''
    def get_position(self):
        return self.pos

    '''
    '' Set the position of this joint in Maya 
    '''
    def set_position(self, position):
        pm.xform(self.mayaID, t=(position[0], position[1], position[2]), ws=True)
        self.pos = position
        return True

    '''
    '' Return this joint's current position in World Space
    '''
    def get_rot(self):
        return self.rot

#############################################################
############## ANIMATION TIME FUNCTIONALITY #################
#############################################################

'''
'' Returns the current time in maya
'''
def get_current_time():
    return cmds.currentTime(query=True)

'''
'' Sets the current time in maya
'''
def set_current_time(newTime):
    cmds.currentTime(newTime, edit=True)

'''
'' Gets the config time Increment
'''
def get_time_increment():
    return CONFIG['TIME_INCREMENT']

'''
'' Increments the time kept in the maya by the TIME_INCREMENT
'''
def increment_current_time(timeIncrement=0):
    if (timeIncrement == 0):
        timeIncrement = get_time_increment()
    # Update the current time by the increment
    newTime = get_current_time() + timeIncrement
    set_current_time(newTime)
    
    # Check if we need to increase the max time buffer
    maxTimeBuffer =  get_max_time()-10
    if (newTime >= maxTimeBuffer):
        increase_max_time()

'''
'' Returns the current max time from our config
'''
def get_max_time():
    return pm.playbackOptions(query=True, maxTime=True)

'''
'' Sets the maxTime in maya
'''
def set_max_time(newTime):
    pm.playbackOptions(maxTime=newTime)

def get_max_time_increment():
    return CONFIG['MAX_TIME_INCREMENT']

'''
'' Increases the max time in maya
'''
def increase_max_time(timeIncrement=get_max_time_increment()):
    newMaxTime = get_max_time() + timeIncrement
    pm.playbackOptions(loop='once', maxTime=newMaxTime)
 

'''
'' Resets the keyframes for all joints in the Structure
'' And resets the current time key and max time
'''
def reset_time():
    #Clear All the key frames for every joint
    currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
    
    # Clear keyframes for all hands 
    clear_keyframes()   

    # Reset time 
    set_current_time(CONFIG['MIN_TIME'])
    set_max_time(get_current_time() + get_max_time_increment())

'''
'' Clears the keyframes for all linked items
'''
def clear_keyframes():
    # Clear key frames for all hands
    for hand in PAALM_HANDS:
       hand.clear_keyframes()
    # Increment key frame time
    pm.refresh(force=True)
    increment_current_time()




#############################################################
################## PORT RECEIVING FUNCTIONS #################
#############################################################
'''
'' Interfacing with a command port that sends target positon updates for
'' each finger. <targetQueue> is an array of dictionaries with each one 
'' containing a finger 'id', a 'length_ratio', and and 'dir' 
'' (the directional vector), and if enabled, palm position data
'''
def update_IK_targets(targetQueue):
    # Don't Update targets if we aren't tracking
    if (not IS_TRACKING):
        print 'NOT TRACKING'
        return

    #Right Hand
    global PAALM_HANDS 
    rightHand = PAALM_HANDS[0]
    rightPalm = rightHand.palm
    numFingers = len(rightHand.fingers)
    count = 0
    octoCount = 0
    
    # Assume using direction
    for target in targetQueue:
        # Handle the Demo Type
        if (ACTIVE_DEMO_TYPE == OCTOPUS_DEMO_TYPE):
            # Octopuse Demo
            handle_octo_input(rightHand.fingers, target, octoCount)
            if (octoCount == 0 or octoCount == 1):
                octoCount = octoCount + 1
            else:
                octoCount = octoCount + 2
            continue
        elif (ACTIVE_DEMO_TYPE == CHARACTER_DEMO_TYPE and USE_CHARACTER and len(targetQueue) > 2):
            # Character Demo
            count = count + 1
            if (count > 2):
                break

        # Update the palm position, if necessary
        if (target['id'] == CONFIG['PALM_INDEX']):
            if (not IS_TRACKING_PALM_POSITION):
                # Don't track palm if we are supposed to
                continue
            palmNormal = target['normal']
            # Update the target palm position
            palmPosition = target['position']
            rightPalm.set_position(palmPosition)
            continue
        else:
            # Otherwise, update the position of the current position
            fingerID = target['id']

            #There is an id to match out finger id in our maya hand
            #if (fingerID < numFingers):

            #Get the target Direction & lengthRatio
            tDir = target['dir']
            targetDir = Leap.Vector(tDir[0], tDir[1], tDir[2])
            targetLengthRatio = target['length_ratio']

            #Index Finger and Base Position and end joint
            finger = rightHand.fingers[fingerID]
            fingerBaseJoint = finger.get_joints()[0]
            fingerBasePosition = fingerBaseJoint.get_position()
            
            #End of finger
            fingerEnd = finger.get_effector()
            fingerEndPosition = fingerEnd.get_position()
            
            #Get Length of the Finger
            fingerLength = finger.get_length()

            #Get the ratio of finger length from the Leap (with the baseline) and use it for our Maya Length
            targetLength = fingerLength * targetLengthRatio

            # print 'Maya Finger Length : %s' % fingerLength
            # print 'Leap Length Ratio: %s' % targetLengthRatio
            # print 'Target Distance: %s' % targetLength

            #Create a vector from the direction vector scale by the targetDistance
            #Add the base joint position to get out  target point location
            newTargetPos =  fingerBasePosition + ((targetDir) * targetLength)
            # print 'Target Position: %s' % newTargetPos

            #Set the Target Position 
            finger.set_target_position(newTargetPos)

    # Perform Inverse Kinematics with currentTime
    currentTime = get_current_time()
    # Set keyframe for palm
    rightPalm.set_keyframe(currentTime)
    
    # Set Key frame for a finger
    for finger in rightHand.fingers:
        #finger.perform_ccd(ccdTime)
        finger.set_keyframe(currentTime)
        
        #Refresh Display
        pm.refresh(force=True)
    #Increase Animation Timer
    increment_current_time()

'''
'' Function for dealing with octopus input 
'''
def handle_octo_input(rightHandFingers, target, octoCount):
        maxRange = 2
        if (octoCount == 0 or octoCount == 1):
            maxRange = 1
        else:
            print 'cool'
        print 'OC: ' +str(octoCount)
        #There is an id to match out finger id in our maya hand
        #if (fingerID < numFingers):
        for i in range(0, maxRange):
            #print i 
            fingerID = octoCount + i
            print fingerID
            #Get the target Direction & lengthRatio
            tDir = target['dir']
            targetDir = Leap.Vector(tDir[0], tDir[1], tDir[2])
            targetLengthRatio = target['length_ratio']

            #Index Finger and Base Position and end joint
            finger = rightHand[fingerID]
            fingerBaseJoint = finger.get_joints()[0]
            fingerBasePosition = fingerBaseJoint.get_position()
            
            #End of finger
            fingerEnd = finger.get_effector()
            fingerEndPosition = fingerEnd.get_position()
            
            #Get Length of the Finger
            fingerLength = finger.get_length()

            #Get the ratio of finger length from the Leap (with the baseline) and use it for our Maya Length
            targetLength = fingerLength*targetLengthRatio

            # print 'Maya Finger Length : %s' % fingerLength
            # print 'Leap Length Ratio: %s' % targetLengthRatio
            # print 'Target Distance: %s' % targetLength

            #Create a vector from the direction vector scale by the targetDistance
            #Add the base joint position to get out  target point location
            newTargetPos =  fingerBasePosition + ((targetDir) * targetLength)
            # print 'Target Position: %s' % newTargetPos

            #Set the Target Position 
            finger.set_target_position(newTargetPos)


############################################################################


'''############################################################
'' ## Calibration / Leap Controller Listening #################
'' ############################################################
'''

'''############################################################
'' ## MAYA UI ELEMENTS ########################################
'' ############################################################
'''

############################################################
###  Custom Menu Window for PAALM (Buttons) ################
############################################################
class PAALMEditorWindow(object):

    ''' 
    '' Creates the Editor Window 
    '''
    def __init__(self):
        global PAALM_EDITOR_WINDOW
        # Create an editor window
        self.name = PAALM_EDITOR_WINDOW_NAME
        self.window = None
        self.layout = None

        # Delete the old window by name if it exists
        if (self.exists()):
            cmds.deleteUI(self.name)

        # Build the window and layout
        self.construct()

        # Store global reference for window
        PAALM_EDITOR_WINDOW = self

    '''
    '' Returns true if this editor window already exists
    '''
    def exists(self):
        return cmds.window(self.name, query=True, exists=True)

    '''
    '' Constructs the window and layout
    '''
    def construct(self):
        self.window =cmds.window(
            self.name,
            menuBar=True, 
            width=450, 
            height=450, 
            title='PAALM Editor',
            maximizeButton=False,
            docTag='PAALM Editor',
            sizeable=False,
            retain=True)
        self.layout = cmds.columnLayout(parent=self.window)

        # Reset button for clearing keyframes
        cmds.button(
            label='Reset', 
            parent=self.layout, 
            command= pm.Callback(reset_time))

        # Calibrate Button
        cmds.button(
            label='Calibrate', 
            parent=self.layout, 
            command=pm.Callback(init_calibration))
               

        # Start Tracking
        cmds.button(
            label='Open Port', 
            parent=self.layout, 
            command=pm.Callback(init_tracking))

        # Stop Tracking
        cmds.button(
            label='Toggle Tracking', 
            parent=self.layout, 
            command=pm.Callback(toggle_tracking))

                # Stop Tracking
        cmds.button(
            label='Hand Demo', 
            parent=self.layout, 
            command=pm.Callback(init_demo, HAND_DEMO_TYPE))

    '''
    '' Shows the editor window
    '''
    def show(self):
        # Make the window if none exists
        if (self.exists()):
            # Show the window if it already exists
            cmds.showWindow(self.name)
        else:
            # Show the button window
            self.construct()
            cmds.showWindow(self.window)


##############################################################
###  Custom Drop Down Menu for PAALM  ########################
##############################################################

class PAALMDropDownMenu(object):

    '''
    '' Initialize the PAALM DropDown Menu
    '''
    def __init__(self):
        gMainWindow = maya.mel.eval('$temp1=$gMainWindow')
        self.name = PAALM_DROP_DOWN_MENU_NAME
        # Delete the old window by name if it exists
        if (self.exists()):
            cmds.deleteUI(self.name)

        dropDownMenu = cmds.menu(
            PAALM_DROP_DOWN_MENU_NAME, 
            label=PAALM_DROP_DOWN_MENU_LABEL, 
            parent=gMainWindow, 
            tearOff=True)
        cmds.menuItem(
            label='Show Editor', 
            parent=dropDownMenu,
            command=pm.Callback(self.show_editor_window))
        cmds.menuItem(
            label='About',
            parent=dropDownMenu,
            command=pm.Callback(self.show_about_page))
        #cmds.menuItem(divider=True)
        cmds.menuItem(
            label='Quit',
            parent=dropDownMenu,
            command=pm.Callback(self.quit))

    '''
    '' Returns true if this menu exists in Maya's top option menu
    '''
    def exists(self):
        return cmds.menu(self.name, query=True, exists=True)


    ''' 
    '' Shows the PAALM Editor Window 
    '''
    def show_editor_window(self):
        PAALM_EDITOR_WINDOW.show()

    '''
    '' Show the About / PAALM Blog
    '''
    def show_about_page(self):
        cmds.showHelp(PAALM_ABOUT_WEBSITE, absolute=True)

    '''
    '' Quit paalm tracking
    '''
    def quit(self):
        stop_tracking()
        print 'No Longer Tracking!'


    
#######################################################
''' 
#############################################################
## GLOBALS UI FXNS ##########################################
#############################################################
'''


#######################################################
'''
#############################################################
## PAALM TRACKING FUNCTIONS #################################
#############################################################
'''

'''
'' Initializes a leap listener and tracker for getting hand data
'''
def init_tracking(self=None):
    # Stop tracking if we are currently
    open_command_port()
    
    # Add the listener to receive events from the controller
    #LEAP_CONTROLLER.add_listener(PAALM_LEAP_LISTENER)

    # Keep this process running until Q is pressed to quit
    print 'Tracking hand gestures!'

'''
'' Temporarily pause hand tracking
'''
def toggle_tracking(self=None):
    global IS_TRACKING
    # Toggle sending palm data
    wasTracking = IS_TRACKING
    IS_TRACKING = not IS_TRACKING
    trackingMsg = 'Tracking Started'
    if (wasTracking): 
        trackingMsg = 'Tracking Paused'
    print trackingMsg

'''
'' Temporarily stop sending palm tracking datah
'''
def toggle_palm_tracking(self=None):
    global IS_TRACKING_PALM_POSITION
    # Toggle sending palm data
    wasTrackingPalm = IS_TRACKING_PALM_POSITION
    IS_TRACKING_PALM_POSITION = not IS_TRACKING_PALM_POSITION
    trackingMsg = 'Tracking Started'
    if (wasTrackingPalm): 
        trackingMsg = 'Tracking Paused'
    print trackingMsg


'''
'' Terminates hand tracking
'''
def stop_tracking(self=None):
    close_command_port()
    print 'Removed Tracking Listener'


'''
'' Calibrates hand tracking data based on finger lengths and
'' a spread open palm
'''
def init_calibration(self=None):
    # TODO remove
    print 'Hold your hand above the Leap Motion Controller with Open Palms'
    #PAALM_LEAP_LISTENER.reset_calibration()

'''
'' Initializes a demo type for IK Targets
'''
def init_demo(demoType):
    if (demoType == HAND_DEMO_TYPE):
        IK_HAND_TEST()
    elif (demoType == CHARACTER_DEMO_TYPE):
        #IK_CHARACTER_TEST()
        print 'Character Demo'
    elif (demoType == OCTOPUS_DEMO_TYPE):
        #IK_OCTO_TEST()
        print 'Octopus Demo'





#######################################################
'''
#############################################################
## GLOBALS VARS  ############################################
#############################################################
'''

## Author Information ##
PAALM_AUTHOR = 'Michael Rivera'
PAALM_AUTHOR_WEBSITE = 'http://mikeriv.com'
PAALM_ABOUT_WEBSITE = 'http://projectpaalm.blogspot.com'

## Refers to MEL command that starts the PAALM Plugin ##
PAALM_COMMAND_NAME = 'paalm'


## Unique Name of the PAALM Editor Window in memory ##
PAALM_EDITOR_WINDOW_NAME = 'PAALMEditor'

## Reference to the PAALM Editor Window ##
PAALM_EDITOR_WINDOW = PAALMEditorWindow()

## Unique name of the drop down menu in memory ##
PAALM_DROP_DOWN_MENU_NAME = 'PAALMMENU'

## Label of the drop down menu ##
PAALM_DROP_DOWN_MENU_LABEL = 'PAALM'

## Reference to the PAALM Drop Down Menu ##
PAALM_DROP_DOWN_MENU = PAALMDropDownMenu()

##############################################################
'''
##############################################################
## Maya Plug-in Set up #######################################
##############################################################
'''

''' 
'' PAALM Command Class 
'''
class PAALM(OpenMayaMPx.MPxCommand):

    ''' 
    '' Constructor 
    '''
    def __init__(self):
        OpenMayaMPx.MPxCommand.__init__(self)

    ''' 
    '' Execution of the Command 
    '''
    def doIt(self, args):
        # Convert our string to a dictionary
        targetQueue = ast.literal_eval(args.asString(0))
        # Now update targets using the dictionary
        update_IK_targets(targetQueue)

''' 
'' Creates an instance of our command 
'''
def paalm_command_creator():
    return OpenMayaMPx.asMPxPtr(PAALM())

''' 
'' Initialize the plug-in when Maya loads it 
'''
def initializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.registerCommand(PAALM_COMMAND_NAME, paalm_command_creator)
    except:
        sys.stderr.write('Failed to register command: ' + PAALM_COMMAND_NAME)

''' 
'' Uninitialize the plug-in when Maya un-loads it 
'''
def uninitializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.deregisterCommand(PAALM_COMMAND_NAME)
        stop_tracking()
    except:
        sys.stderr.write('Failed to unregister command: ' + PAALM_COMMAND_NAME)


############################################################
############################################################
#################### IK HANDLE TEST ########################
############################################################


''' 
'' Defines the Joint Structure for PAALM Hand Demo
'''
def IK_HAND_TEST():
    global CONFIG, PAALM_FINGERS, PAALM_HANDS

    ##### THUMB FINGER ####
    thumbFinger = Finger('R-ThumbFinger')
    
    # Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j3 = Joint('joint3')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    # Proximal Interphalangeal Joint (PIJ) & Constraints
    j4 = Joint('joint4')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    # Distal Interphalangeal Joint (DIJ) & Constrains
    j5 = Joint('joint5')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    # Target Position
    thumbTarget = Target('thumbIKHandle')

    # Add ordered joint chain to finger 1
    thumbFinger.add_joints([j3,j4,j5])

    # Set the target for the finger
    thumbFinger.set_target(thumbTarget)

    # Clear all set key frames
    PAALM_FINGERS.append(thumbFinger)

    #### INDEX FINGER #####
    indexFinger = Finger('R-IndexFinger')
    
    # Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j6 = Joint('joint6')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    # Proximal Interphalangeal Joint (PIJ) & Constraints
    j7 = Joint('joint7')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    # Distal Interphalangeal Joint (DIJ) & Constrains
    j8 = Joint('joint8')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    # End Effector Cannot move at all
    j9 = Joint('joint9')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    # Target Position
    indexTarget = Target('indexIKHandle')

    # Add ordered joint chain to finger 1
    indexFinger.add_joints([j6,j7,j8,j9])

    # Set the target for the finger
    indexFinger.set_target(indexTarget)

    # Clear all set key frames
    PAALM_FINGERS.append(indexFinger)

    #finger1.perform_ccd()


    ##### MIDDLE FINGER #####
    middleFinger = Finger('R-MiddleFinger')
    
    # Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j10 = Joint('joint10')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    # Proximal Interphalangeal Joint (PIJ) & Constraints
    j11 = Joint('joint11')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    # Distal Interphalangeal Joint (DIJ) & Constrains
    j12 = Joint('joint12')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    # End Effector Cannot move at all
    j13 = Joint('joint13')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    # Target Position
    middleTarget = Target('middleIKHandle')

    # Add ordered joint chain to finger 1
    middleFinger.add_joints([j10,j11,j12,j13])

    # Set the target for the finger
    middleFinger.set_target(middleTarget)

    # Clear all set key frames
    middleFinger.clear_keyframes()
    PAALM_FINGERS.append(middleFinger)

    ##### RING FINGER #####
    ringFinger = Finger('R-RingFinger')
    
    # Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j14 = Joint('joint14')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    # Proximal Interphalangeal Joint (PIJ) & Constraints
    j15 = Joint('joint15')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    # Distal Interphalangeal Joint (DIJ) & Constrains
    j16 = Joint('joint16')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    # End Effector Cannot move at all
    j17 = Joint('joint17')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    # Target Position
    ringTarget = Target('ringIKHandle')

    # Add ordered joint chain to finger 1
    ringFinger.add_joints([j14,j15,j16,j17])

    # Set the target for the finger
    ringFinger.set_target(ringTarget)

    # Clear all set key frames
    PAALM_FINGERS.append(ringFinger)

    ##### PINKY FINGER #####
    pinkyFinger = Finger('R-PinkyFinger')
    
    # Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j18 = Joint('joint18')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    # Proximal Interphalangeal Joint (PIJ) & Constraints
    j19 = Joint('joint19')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    # Distal Interphalangeal Joint (DIJ) & Constrains
    j20 = Joint('joint20')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    # End Effector Cannot move at all
    j21 = Joint('joint21')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    # Target Position
    pinkyTarget = Target('pinkyIKHandle')

    # Add ordered joint chain to finger 1
    pinkyFinger.add_joints([j18,j19,j20,j21])

    # Set the target for the finger
    pinkyFinger.set_target(pinkyTarget)

    # Add pinky to list of finger
    PAALM_FINGERS.append(pinkyFinger)

    #### PALM #####
    palm = Palm('palm_joint')

    ### Make Right Hand from Fingers and palm ###
    rightHand = Hand('Right-Hand', PAALM_FINGERS, palm)

    ## Add right hand to list of hands
    PAALM_HANDS.append(rightHand)

    # Reset all time and keyframes
    reset_time()
