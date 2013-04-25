import pymel.all as pm
from pymel.core import * 
import sys, Leap, string
import math as Math


#import pymel.api as pm
#PyMel Data Types for Matrices, Quaternions etc
import pymel.core.datatypes as dt
#import pymel.core.animation as pm 



############################################################
#################### CONSTANT VARS ####################
############################################################
DEG_TO_RAD = Math.pi/180
EPSILON = 0.001
CURRENT_TIME_KEY = 'CURRENT_TIME'


############################################################
#################### CONFIGURATION VARS ####################
############################################################

CONFIG = {
    'JOINT_ANGLE_MULTIPLIER': 0.5,
    'CURRENT_TIME': 1,                   #Current Animation time
    'INITIAL_MAX_TIME': 30000,
    'MAX_TIME': 60000,
    'MIN_TIME': 1,
    'TIME_INCREMENT': 15
}

#############################################################
############## SOCKET COMMAND PORT FOR MAYA #################
#############################################################
#create Command port - sets port up to receive data on the mel function for receiveTipPos
# This then forwards the data to the python function
def open_command_port():
    #Check if our port is already open if it isn't open it
    for i in pm.commandPort(q=True,lp=True):
        if (i ==':6001'):
            #Port already exists
            return
    #Otherwise open the port`
    pm.commandPort(name=':6001')


############################################################
############################ HAND ##########################
############################################################
class Hand(object):
    def __init(self, mayaID, fingerList=[], palm=None, wrist=None):
        self.mayaID = mayaID
        self.addFingers(fingerList)
        self.hasWrist = False
        self.hasPalm = False
        self.set_wrist(wrist)
        self.set_palm(palm)

    #Returns the output that is returned when using the print function
    def __str__(self):
        return '<# HandID:\'%s\' # %s #>' % (self.mayaID, self.fingerList)
    
    def __repr__(self):
        return self.__str__()

    #Adds a list of fingers to this hand
    def add_fingers(self, fingerList):
        success = True
        for finger in fingerList:
            success = self.addFinger(finger) and success
        return success

    #Adds a finger to this hand
    def add_finger(self, finger):
        if (isinstance(finger, Joint)):
            self.fingerList.append(finger)
            return True
        else:
            return False

    #Sets the wrist on the hand
    def set_wrist(self, wrist):
        if (isinstance(wrist, Joint)):
            self.wrist = wrist
            self.hasWrist = True
        else:
            self.hasWrist = False
    
    #Returns true if this hand has a wrist
    def has_wrist(self):
        return self.hasWrist

    #Sets the palm on the hand
    def set_palm(self, palm):
        if (isinstance(palm, Joint)):
            self.palm = palm
            self.hasPalm = True
        else:
            self.hasPalm = False

    #Returns true if this hand has a palm
    def has_palm(self):
        return self.hasPalm

    #Compute length between the palm and wrist
    def calculate_palm_to_wrist_length(self):
        if (self.has_palm() and self.has_wrist()):
            palmToWristLength = (self.palm.get_position()-self.wrist.get_position()).magnitude
            return palmToWristLength
        else:
            #No length
            return 0

    def perform_ccd(self):
        #For each finger 
        print 'unimplemented ccd for hand'
        

    #Set the target positions for their corresponding fingers by 
    def update_target_positions(self, targetPositions):
        #For each Target position perform CCD on that finger
        for tp in targetPositions:
            fingerIndex = tp['fingerIndex']
            #Check if the finger is valid for this hand
            if (fingerIndex < len(self.fingerList)):
                #Get the finger corresponding to this index
                finger = self.fingerList[fingerIndex]
                #Set the target position for this finger
                finger.set_target_position(tp)
            else:
                #Do nothing for that index
                print 'Invalid finger index'


############################################################
########################### FINGER #########################
############################################################
class Finger(object):
    def __init__(self, mayaID, target=None, jointList=[]):
        self.mayaID = mayaID
        self.length = 0
        self.hasTarget = False
        self.jointList = []
        self.add_joints(jointList)
        self.set_target(target)
        self.calculate_length()
        self.currentTime = 0


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
        #Increment the Global Time
        self.increment_time()
    
    #Increment the finger keyframe counter
    def increment_time(self):
        self.currentTime += CONFIG['TIME_INCREMENT']

    #Clears all key frames on the finger's joints from
    def clear_keyframes(self, currentMaxTime=CONFIG['MAX_TIME']):

        #currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
        #CONFIG['MAX_TIME'] = currentMaxTime
        
        self.currentTime = 1

        for joint in self.jointList:
            #print joint
            joint.clear_keyframes(currentMaxTime)
                #Clear All the key frames for every joint
        #pm.refresh(force=True)
        if (self.has_target()):
            self.target.clear_keyframes(currentMaxTime)


        CONFIG[CURRENT_TIME_KEY] = CONFIG['MIN_TIME']
        CONFIG['MAX_TIME'] = CONFIG['INITIAL_MAX_TIME']
        #print CONFIG['MAX_TIME']
        pm.playbackOptions(maxTime= CONFIG['MAX_TIME'])

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
            #TODO: REturn the end effector's current position
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
            palmDistance = (palm.get_position()-self.jointList[0]).magnitude
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
        # chainLength = 0;

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
                secondKey = index +1

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
        # chainExtension = 5;
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
            self.currentTime = keyframeTime
            #Distance Threshold for ending the ccd
            DISTANCE_THRESHOLD = 0.1
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

            #formerly perform_ccd(self,targetTipPos)
            #Config bool that tells us if we should be incrementing jointnums from 1
            #If false we decrement from the max joint joint
            USE_INCREASE_JOINT_NUM = False
            if USE_INCREASE_JOINT_NUM:
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
            while (distance > DISTANCE_THRESHOLD and iterations > 0):

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
                if USE_INCREASE_JOINT_NUM:
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
        pm.cutKey(self.mayaID, time=(1, currentMaxTime), option='keys')

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
        pm.refresh(force=True)


    #Return this joint's current position in World Space
    def get_rot(self):
        return self.rot

    #Computes an optimal rotation for this joint to point towards the targetPosition
    #Applys the rotation via updating the composite matrix
    def update_rotation(self, effectorPos, targetPos):
        #Position of the joiny
        jointPos = self.get_position()

        #The Two Vectors of Interest
        V1 = (effectorPos-jointPos)
        V2 = (targetPos-jointPos)

        QUATS = True
        if QUATS:
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
        sPoint = point-sphereCenter
        #Magnitude of the point
        pMag = sPoint.magnitude

        if (pMag <= armLength):
            #This point is within our sphere so we need not project it
            #print "in range returning same point"
            return point
        #Vector to projected Point
        Q = (point/pMag)*armLength
        #The point projected onto the Sphere
        projectPt = Q + sphereCenter
        return projectPt

    #TODO CREATE SPHERE IN MAYA FOR FINGER


#############################################################
############## ANIMATION TIME FUNCTIONALITY #################
#############################################################

def get_current_time():
    return CONFIG[CURRENT_TIME_KEY]

def get_max_time():
    return CONFIG['MAX_TIME']

def increment_time():
    CONFIG[CURRENT_TIME_KEY] = CONFIG[CURRENT_TIME_KEY] + CONFIG['TIME_INCREMENT']
    pm.playbackOptions(loop='once', maxTime=CONFIG['MAX_TIME'])
    currentTime = get_current_time()
    maxTimeBuffer =  get_max_time()-10
    if (currentTime >= maxTimeBuffer):
        increase_max_time()

def increase_max_time():
    CONFIG['MAX_TIME'] = CONFIG['MAX_TIME'] + 200
    pm.playbackOptions(maxTime=CONFIG['MAX_TIME'])


def reset_time(): 
    #Clear All the key frames for every joint
    currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
    #print 'woo'
    #print currentMaxTime
    for hand in HANDS: 
        #print jointKey 
        #pm.cutKey(jointKey, time=(1, currentMaxTime), option='keys')
        for finger in hand:
            finger.clear_keyframes()
            pm.refresh(force=True)
    CONFIG[CURRENT_TIME_KEY] = CONFIG['MIN_TIME']
    CONFIG['MAX_TIME'] = CONFIG['INITIAL_MAX_TIME']
    #pm.playbackOptions(maxTime= CONFIG['INITIAL_MAX_TIME'])


#############################################################
################## PORT RECEIVING FUNCTIONS #################
#############################################################
#Interfacing with a command port that sends target positon updates for
# each finger. <targetQueue> is an array of dictionaries with each one 
# containing a finger 'id', a 'length_ratio', and and 'dir' 
# (the directional vector)
def receive_target_queue(targetQueue):
    #Right Hand
    rightHand = HANDS[0]
    numFingers = len(rightHand)
    count = 0
    octoCount = 0
    #Assume using direction
    for target in targetQueue:

        if (DEMO_TYPE == 3):
            #Octopus Demo
            handle_octo_input(rightHand, target, octoCount)
            octoCount = octoCount + 1
            continue
        elif (DEMO_TYPE == 2 and USE_CHARACTER and len(targetQueue) > 2):
            count = count + 1
            if (count > 2):
                break;

        
        fingerID = target['id']

        #There is an id to match out finger id in our maya hand
        #if (fingerID < numFingers):

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

    #Perform CCD now :)
    ccdTime = get_current_time()
    for finger in rightHand:
        #finger.perform_ccd(ccdTime)
        #Refresh Display
        finger.set_keyframe(ccdTime)
        pm.refresh(force=True)

    #Increase Animation Timer
    increment_time()


def handle_octo_input(rightHand, target, octoCount):
        fingerID = octoCount
        maxRange = 2
        if (octoCount == 4):
            maxRange = 1

        #There is an id to match out finger id in our maya hand
        #if (fingerID < numFingers):
        for i in range(0, maxRange):

            fingerID = octoCount + i
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


############################################################
################### MAIN (USED FOR TESTING) ################
############################################################
#Main Loop for initialization
def main():
    #Close open ports & open our current port
    open_command_port()

    #Initialize all joint positions
    #init_joint_positions()

    #Reset the animation time
    reset_time()

    #Perform CCD (for testing)
    perform_ccd() 

if __name__ == '__main__':
    print 'main!'
    #main()


############################################################
####################### FINGER TEST ########################
############################################################

FINGERS = []
HANDS = []

############################################################
#################### IK HANDLE TEST ########################
############################################################

def IK_FINGER_TEST():
    #Close open ports & open our current port
    open_command_port()

    ##### THUMB FINGER ####
    thumbFinger = Finger('R-ThumbFinger')
    #Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j3 = Joint('joint3')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    #Proximal Interphalangeal Joint (PIJ) & Constraints
    j4 = Joint('joint4')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    #Distal Interphalangeal Joint (DIJ) & Constrains
    j5 = Joint('joint5')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    #Target Position
    thumbTarget = Target('thumbIKHandle')

    #Add ordered joint chain to finger 1
    thumbFinger.add_joints([j3,j4,j5])

    #Set the target for the finger
    thumbFinger.set_target(thumbTarget)

    #Clear all set key frames
    FINGERS.append(thumbFinger)

    #### INDEX FINGER #####
    indexFinger = Finger('R-IndexFinger')
    #Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j6 = Joint('joint6')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    #Proximal Interphalangeal Joint (PIJ) & Constraints
    j7 = Joint('joint7')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    #Distal Interphalangeal Joint (DIJ) & Constrains
    j8 = Joint('joint8')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    #End Effector Cannot move at all
    j9 = Joint('joint9')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    #Target Position
    indexTarget = Target('indexIKHandle')


    #Add ordered joint chain to finger 1
    indexFinger.add_joints([j6,j7,j8,j9])

    #Set the target for the finger
    indexFinger.set_target(indexTarget)

    #Clear all set key frames
    FINGERS.append(indexFinger)

    #finger1.perform_ccd()


    ##### MIDDLE FINGER #####
    middleFinger = Finger('R-MiddleFinger')
    #Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j10 = Joint('joint10')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    #Proximal Interphalangeal Joint (PIJ) & Constraints
    j11 = Joint('joint11')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    #Distal Interphalangeal Joint (DIJ) & Constrains
    j12 = Joint('joint12')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    #End Effector Cannot move at all
    j13 = Joint('joint13')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    #Target Position
    middleTarget = Target('middleIKHandle')

    #Add ordered joint chain to finger 1
    middleFinger.add_joints([j10,j11,j12,j13])

    #Set the target for the finger
    middleFinger.set_target(middleTarget)

    #Clear all set key frames
    middleFinger.clear_keyframes()
    FINGERS.append(middleFinger)

    ##### RING FINGER #####
    ringFinger = Finger('R-RingFinger')
    #Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j14 = Joint('joint14')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    #Proximal Interphalangeal Joint (PIJ) & Constraints
    j15 = Joint('joint15')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    #Distal Interphalangeal Joint (DIJ) & Constrains
    j16 = Joint('joint16')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    #End Effector Cannot move at all
    j17 = Joint('joint17')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    #Target Position
    ringTarget = Target('ringIKHandle')

    #Add ordered joint chain to finger 1
    ringFinger.add_joints([j14,j15,j16,j17])

    #Set the target for the finger
    ringFinger.set_target(ringTarget)

    #Clear all set key frames
    FINGERS.append(ringFinger)

    ##### PINKY FINGER #####
    pinkyFinger = Finger('R-PinkyFinger')
    #Knuckle - Metacarpal Phalangeal Joint (MPJ) & Constraints
    j18 = Joint('joint18')
    #j6.set_rotation_limits(0, 0, -40, 60, -11, 11)
    
    #Proximal Interphalangeal Joint (PIJ) & Constraints
    j19 = Joint('joint19')
    #j7.set_rotation_limits(0, 0, 0, 0, 0, 90)
     
    #Distal Interphalangeal Joint (DIJ) & Constrains
    j20 = Joint('joint20')
    #j8.set_rotation_limits(0, 0, 0, 0, 0, 90)

    #End Effector Cannot move at all
    j21 = Joint('joint21')
    #j9.set_rotation_limits(0, 0, 0, 0, 0, 0)
    
    #Target Position
    pinkyTarget = Target('pinkyIKHandle')

    #Add ordered joint chain to finger 1
    pinkyFinger.add_joints([j18,j19,j20,j21])

    #Set the target for the finger
    pinkyFinger.set_target(pinkyTarget)

    #Clear all set key frames
    FINGERS.append(pinkyFinger)

    ## ADD ALL FINGERS as a HAND
    HANDS.append(FINGERS)

    #reset_time()
    for finger in FINGERS:
        #Set initial keyframes
        #print 'settting key frame'
        finger.clear_keyframes()
        finger.set_keyframe(1)
    increment_time()



###################################################################
#################### IK_CHARACTER HANDLE TEST ########################
###################################################################

def IK_CHARACTER_TEST():
    #Close open ports & open our current port
    open_command_port()

    ##### Left Leg ####
    leftLeg = Finger('L-Leg')
   # l0 = Joint('Aragor:LeftHips_Dummy')
   # l1 = Joint('Aragor:LeftUpLeg')
    l2 = Joint('Aragor:LeftUpLegRoll')
    l3 = Joint('Aragor:LeftLeg')
    l4 = Joint('Aragor:LeftLegRoll')
    l5 = Joint('Aragor:LeftFoot')
    l6 = Joint('Aragor:LeftToeBase')
    l7 = Joint('Aragor:LeftToes_End')

    #Target Position
    leftLegTarget = Target('LeftLegIKHandle')

    #Add Joint Chain to the Leg
    leftLeg.add_joints([l2,l3,l4,l5,l6,l7])

    #Set the target for the finger
    leftLeg.set_target(leftLegTarget)

    #Clear all set key frames
    FINGERS.append(leftLeg)

    ##### Right Leg ####
    rightLeg = Finger('R-Leg') 
   # r0 = Joint('Aragor:RightHips_Dummy')
   # r1 = Joint('Aragor:RightUpLeg')
    r2 = Joint('Aragor:RightUpLegRoll')
    r3 = Joint('Aragor:RightLeg')
    r4 = Joint('Aragor:RightLegRoll')
    r5 = Joint('Aragor:RightFoot')
    r6 = Joint('Aragor:RightToeBase')
    r7 = Joint('Aragor:RightToes_End')

    #Target Position
    rightLegTarget = Target('RightLegIKHandle')

    #Add Joint Chain to the Leg
    rightLeg.add_joints([r2,r3,r4,r5, r6, r7])

    #Set the target for the finger
    rightLeg.set_target(rightLegTarget)

    #Clear all set key frames
    FINGERS.append(rightLeg)


    ## ADD ALL FINGERS as a HAND
    HANDS.append(FINGERS)

    #reset_time()
    for finger in FINGERS:
        #Set initial keyframes
        #print 'settting key frame'
        finger.clear_keyframes()
        finger.set_keyframe(1)
    increment_time()


###################################################################
#################### IK_OCTO HANDLE TEST ########################
###################################################################

def IK_OCTO_TEST():
    #Close open ports & open our current port
    open_command_port()
    ##################### Leg 1 ##################
    frLeg = Finger('FR-Leg')
    fr0 = Joint('joint18')
    fr1 = Joint('joint19')
    fr2 = Joint('joint20')
    fr3 = Joint('joint21')

    #Target Position
    frTarget = Target('FRLeg')

    #Add Joint Chain to the Leg
    frLeg.add_joints([fr0,fr1,fr2,fr3])

    #Set the target for the finger
    frLeg.set_target(frLegTarget)

    #Clear all set key frames
    FINGERS.append(frLeg)
    ##############################################
    

    ##################### Leg 2 ##################
    flLeg = Finger('FL-Leg')
    fl0 = Joint('joint14')
    fl1 = Joint('joint15')
    fl2 = Joint('joint16')
    fl3 = Joint('joint17')

    #Target Position
    flTarget = Target('FLLeg')

    #Add Joint Chain to the Leg
    flLeg.add_joints([fl0,fl1,fl2,fl3])

    #Set the target for the finger
    flLeg.set_target(flTarget)

    #Clear all set key frames
    FINGERS.append(flLeg)
    ##############################################


    ##################### Leg 3 ##################
    slLeg = Finger('SL-Leg')
    sl0 = Joint('joint10')
    sl1 = Joint('joint11')
    sl2 = Joint('joint12')
    sl3 = Joint('joint13')

    #Target Position
    slTarget = Target('SLLeg')

    #Add Joint Chain to the Leg
    slLeg.add_joints([sl0,sl1,sl2,sl3])

    #Set the target for the finger
    slLeg.set_target(slTarget)

    #Clear all set key frames
    FINGERS.append(slLeg)


    ##################### Leg 4 ##################
    blLeg = Finger('BL-Leg')
    bl0 = Joint('joint6')
    bl1 = Joint('joint7')
    bl2 = Joint('joint8')
    bl3 = Joint('joint9')

    #Target Position
    blTarget = Target('BLLeg')

    #Add Joint Chain to the Leg
    blLeg.add_joints([bl0,bl1,bl2,bl3])

    #Set the target for the finger
    blLeg.set_target(blTarget)

    #Clear all set key frames
    FINGERS.append(blLeg)


    ##################### Leg 5 ##################
    bmlLeg = Finger('BML-Leg')
    bml0 = Joint('joint2')
    bml1 = Joint('joint3')
    bml2 = Joint('joint4')
    bml3 = Joint('joint5')

    #Target Position
    bmlTarget = Target('BMLLeg')

    #Add Joint Chain to the Leg
    bmlLeg.add_joints([bml0,bml1,bml2,bml3])

    #Set the target for the finger
    bmlLeg.set_target(bmlTarget)

    #Clear all set key frames
    FINGERS.append(bmlLeg)

    ##################### Leg 6 ##################
    bmrLeg = Finger('BMR-Leg')
    bmr0 = Joint('joint30')
    bmr1 = Joint('joint31')
    bmr2 = Joint('joint32')
    bmr3 = Joint('joint33')

    #Target Position
    bmrTarget = Target('BMRLeg')

    #Add Joint Chain to the Leg
    bmrLeg.add_joints([bmr0,bmr1,bmr2,bmr3])

    #Set the target for the finger
    bmrLeg.set_target(bmrTarget)

    #Clear all set key frames
    FINGERS.append(bmrLeg)

    ##################### Leg 7 ##################
    brLeg = Finger('BR-Leg')
    br0 = Joint('joint26')
    br1 = Joint('joint27')
    br2 = Joint('joint28')
    br3 = Joint('joint29')

    #Target Position
    brTarget = Target('BRLeg')

    #Add Joint Chain to the Leg
    brLeg.add_joints([br0,br1,br2,br3])

    #Set the target for the finger
    brLeg.set_target(brTarget)

    #Clear all set key frames
    FINGERS.append(brLeg)


    ##################### Leg 8 ##################
    srLeg = Finger('SR-Leg')
    sr0 = Joint('joint22')
    sr1 = Joint('joint23')
    sr2 = Joint('joint24')
    sr3 = Joint('joint25')

    #Target Position
    srTarget = Target('SRLeg')

    #Add Joint Chain to the Leg
    srLeg.add_joints([sr0,sr1,sr2,sr3])

    #Set the target for the finger
    srLeg.set_target(srTarget)

    #Clear all set key frames
    FINGERS.append(srLeg)

    #############################################
    #############################################
    ## ADD ALL FINGERS as a HAND
    HANDS.append(FINGERS)

    #reset_time()
    for finger in FINGERS:
        #Set initial keyframes
        #print 'settting key frame'
        finger.clear_keyframes()
        finger.set_keyframe(1)
    increment_time()





#################################################
##################### DEMO ######################
#################################################


DEMO_TYPE = 1

USE_CHARACTER = True
if (DEMO_TYPE == 2 and USE_CHARACTER):
    ## RUN THE CHARACTER TEST
    IK_CHARACTER_TEST()
elif (DEMO_TYPE == 3):  
    IK_OCTO_TEST()

else:
    ## RUN THE FINGER TEST
    IK_FINGER_TEST()






############################################################
############################################################
############################################################
##################### END SCRIPT #######################