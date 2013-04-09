import pymel.all as pm
from pymel.core import * 
import sys, Leap, string
import math as Math


#import pymel.api as pma
#PyMel Data Types for Matrices, Quaternions etc
import pymel.core.datatypes as dt
import pymel.core.animation as pma 




#Target point of interest - initial conditions
#TP = [13.327005785798825, 5.933350084777719, 1.6255290651771213];
#TARGET_POINT = Leap.Vector(TP[0],TP[1], TP[2])




#Joints and Positions of finger
TIME_KEY = 'CURRENT_TIME'
EFFECTOR_KEY = 'joint9'
TARGET_KEY = 'targetPoint'
BASE_KEY = 'joint6'#'joint1'
DEG_TO_RAD = Math.pi/180
EPSILON = 0.001

#Mapping Range of Maya CS

MAYA_MAX_X = 20
MAYA_MIN_X = -20
MAYA_RANGE_X = MAYA_MAX_X-MAYA_MIN_X

MAYA_MAX_Y = 12
MAYA_MIN_Y = -12
MAYA_RANGE_Y = MAYA_MAX_Y-MAYA_MIN_Y

MAYA_MAX_Z = 20
MAYA_MIN_Z = -20
MAYA_RANGE_Z = MAYA_MAX_Z-MAYA_MIN_Z

#Configuration of Coordinate Max, Min in Leap CS
LEAP_MAX_X = 110
LEAP_MIN_X = -80
LEAP_RANGE_X = LEAP_MAX_X-LEAP_MIN_X

LEAP_MAX_Y = 430  #450
LEAP_MIN_Y = 280  # 130
LEAP_RANGE_Y = LEAP_MAX_Y-LEAP_MIN_Y

LEAP_MAX_Z = 70   #-50
LEAP_MIN_Z = 20   #180
LEAP_RANGE_Z = LEAP_MAX_Z-LEAP_MIN_Z


CONFIG = {
    'JOINT_ANGLE_MULTIPLIER': 0.5,
    'CURRENT_TIME': 0,                   #Current Animation time
    'INITIAL_MAX_TIME': 100,
    'MAX_TIME': 100,
    'MIN_TIME': 0
}


#create Command port - sets port up to receive data on the mel function for receiveTipPos
# This then forwards the data to the python function
def open_command_port():
    #Check if our port is already open if it isn't open it
    for i in pm.commandPort(q=True,lp=True):
        if (i ==':6001'):
            #Port already exists
            return
    #Otherwise open the port
    pm.commandPort(name=':6001')


JOINTS = {
   # 'joint1': {
   #      'base-pos': Leap.Vector(0, 0, 0),
   #      'pos': Leap.Vector(0, 0, 0),
   #      'rot': Leap.Vector(0, 90, 0),
   #      'name': 'palm'
   #  },
   #  'joint2': {
   #      'base-pos': Leap.Vector(0, 0, -5),
   #      'pos': Leap.Vector(0, 0, -5),
   #      'rot': Leap.Vector(0, 0, 0),
   #      'name': 'knuckle'
   #  },
   #  'joint3': {
   #      'base-pos': Leap.Vector(0, 0, -10),
   #      'pos': Leap.Vector(0, 0, -10),
   #      'rot': Leap.Vector(0, 0, 0),
   #      'name': 'mid-joint'
   #  },
   #  'joint4': {
   #      'base-pos': Leap.Vector(0, 0, -15),
   #      'pos': Leap.Vector(0, 0, -15),
   #      'rot': Leap.Vector(0, 0, 0),
   #      'name': 'tip-joint'
   #  },
   #  'joint5': {
   #      'base-pos': Leap.Vector(0, 0, -20),
   #      'pos': Leap.Vector(0, 0, -20),
   #      'rot': Leap.Vector(0, 0, 0),
   #      'name': 'end-effector'
   #  },
    'joint6': {
        'base-pos': Leap.Vector(0, 0, -5),
        'pos': Leap.Vector(0, 0, -5),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'knuckle'
    },
    'joint7': {
        'base-pos': Leap.Vector(0, 0, -10),
        'pos': Leap.Vector(0, 0, -10),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'mid-joint'
    },
    'joint8': {
        'base-pos': Leap.Vector(0, 0, -15),
        'pos': Leap.Vector(0, 0, -15),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'tip-joint'
    },
    'joint9': {
        'base-pos': Leap.Vector(0, 0, -20),
        'pos': Leap.Vector(0, 0, -20),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'end-effector'
    },
    'targetPoint': {
        'base-pos': Leap.Vector(0,0,0),
        'pos': Leap.Vector(0,0,0),
        'rot': Leap.Vector(0,0,0),
        'name': 'targetPoint'
    }
}



############################################################
############################ HAND ##########################
############################################################
class Hand():
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
class Finger():
    def __init__(self, mayaID, target=None, jointList=[]):
        self.mayaID = mayaID
        self.length = 0
        self.hasTarget = False
        self.jointList = []
        self.add_joints(jointList)
        self.set_target(target)
        self.calculate_length()


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
        #Increment the Global Time
        increment_time()
    
    #Clears all key frames on the finger's joints from
    def clear_keyframes(self, currentMaxTime):
        for joint in self.jointList:
            #print joint
            joint.clear_keyframes(currentMaxTime)

    #Sets the target object that is in maya for this finger
    def set_target(self, target):
        if (isinstance(target, Joint)):
            self.target = target
            self.hasTarget = True
        else:
            self.target = None
            self.hasTarget = False
        #Update all joint and target attributes
        self.update()
        self.calculate_length()

    #Sets the target position for the finger 
    def set_target_position(self, x,y,z):
        self.calculate_length()
        print "set target pos?"

    def has_target(self):
        return self.hasTarget

    def get_target_position(self):
        if (self.has_target()):
            return self.target.get_position()
        else:
            print 'No target defined'
            #TODO: REturn the end effector's current position
            return Leap.Vector(0,0,0)
    
    def get_effector(self):
        joints = self.get_joints()
        numJoints = self.get_num_joints()
        if (numJoints > 0):
            return joints[numJoints-1]
        else:
            print 'No Effector!'
            return None

    def get_effector_position(self):
        print 'getting that position!'
        effector = self.get_effector()

        if (effector is not None):
            print 'getting that position!'
            return effector.get_position()
        else: 
            print 'Cannot Get Effector Position of None!'
            return None

    def set_effector_position(self, position):
        effector = get_effector()
        if (effector is not None):
            effector.set_position(position)
            return True;
        else:
            print 'Cannot set Effector Position of None!'
            return False

    #Updates all joint and target positiings for this finger 
    def update(self):
        #Update all joints
        for joint in self.jointList:
            joint.update()
        #Update the Target
        if self.has_target():
            self.target.update()
            #TODO: Project target position to be in range of arm Length
            #print 'unimplemented'
            # #PRoject to be in range of the armLength
            # armLength = 20 #sum of the arm lengths
            # basePtKey = BASE_KEY
            # sphereCenter = JOINTS[basePtKey]['pos']
            # JOINTS[TARGET_KEY]['pos'] = project_point(TARGET_POINT, sphereCenter, armLength)
            # #print TARGET_POINT
    
    #Calculate the distance between the palm and the first joint of the finger
    def calculate_palm_distance(palm):
        numJoints = len(self.get_joints())
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
    def perform_ccd(self, palmToWristLength=0):
        #No ccd if no target
        if (self.target is None) or (self.get_num_joints() < 2):
            return False

        else:
            #Distance Threshold for ending the ccd
            DISTANCE_THRESHOLD = 0.01
            #Number of iterations to perform per finger
            iterations = 90

            #Update positions for all joints
            self.update()

            #Get the joints and the number of them
            joints = self.get_joints()
            numJoints = self.get_num_joints()

            #Last joint is not the effector so second to last
            currentJointNum = numJoints-2
            maxJointNum = numJoints-1

            #formerly perform_ccd(self,targetTipPos)
            #Config bool that tells us if we should be incrementing jointnums from 1
            #If false we decrement from the max joint joint

            USE_INCREASE_JOINT_NUM = False
            if USE_INCREASE_JOINT_NUM:
                currentJointNum = 0

            #Effector Position and the Target Position
            effector = self.get_effector()
            targetTipPos = self.get_target_position()
            effectorPos = self.get_effector_position()

            #Distance of the effector to the Target
            distance = (targetTipPos - effectorPos).magnitude

            #CCD Main Loop
            while (distance > DISTANCE_THRESHOLD and iterations > 0):
                #Set a Key Frame for the finger
                self.set_keyframe(get_current_time())
                #print distance
                print 'add key frame after'
                #Joint Angle Multiplier for scaling / smoothness
                if (distance > 1):
                    CONFIG['JOINT_ANGLE_MULTIPLIER'] = 1.0
                else:
                    CONFIG['JOINT_ANGLE_MULTIPLIER'] = 0.4

                #Current Joint & Position
                joint = joints[currentJointNum]

                #Update our joint rotation based on the effector Position and the Target Effector Pos
                print 'update rot pos '
                joint.update_rotation(effectorPos, targetTipPos)
                
                #New Effector Position
                effectorPos = self.get_effector_position()
            
                
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
                    if currentJointNum > MAX_JOINT_ID:
                        #keep the joint nums in bounds
                        currentJointNum = MIN_JOINT_ID
                else:
                    #Move onto next joint in the chain
                    currentJointNum = currentJointNum - 1
                    #Cycle look if jointNum goes out of bound
                    if (currentJointNum < MIN_JOINT_ID):
                        currentJointNum = MAX_JOINT_ID
                ############################################
                    #Update all positions of joints in this finger
                    self.update()
                    
            return True

        


############################################################
########################### JOINT ##########################
############################################################
class Joint():
    #Create a joint
    def __init__(self, mayaID, child=None, parent=None, name=None):
        self.mayaID = mayaID
        self.pos = Leap.Vector(0,0,0)
        self.rot = Leap.Vector(0,0,0)
        self.parent = parent
        self.child = child
        self.name = name
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
    def set_position(self):
        print 'unimplemented pos'

    def set_parent(self, parent=None):
        self.parent = parent

    #Set the rotation on this joint in Maya
    def set_rotation(self):
        print 'unimplemented rot'

    #Adds an animation key frame to the joint at at it current attributes
    def set_keyframe(self, currentTime):
        print self
        print self.mayaID
        print currentTime

        pma.setKeyframe(self.mayaID, t=currentTime)
        print 'set that frame!'

    def clear_keyframes(self, currentMaxTime):
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

    #Return this joints current rotation
    def get_parent(self):
        return self.parent

    #Return the child joint of this joint
    def get_child(self):
        return self.child

    #Does this joint has a child joint
    def has_child(self):
        if self.child is None:
            return False
        else:
            return True


    #Computes an optimal rotation for this joint to point towards the targetPosition
    #Applys the rotation via updating the composite matrix
    def update_rotation(self, effectorPos, targetPos):
        #Position of the joiny
        jointPos = self.get_position()

        #The Two Vectors of Interest
        V1 = (effectorPos-jointPos)
        V2 = (targetPos-jointPos)
       
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


############################################################
################## END EFFECTOR (JOINT) ####################
############################################################

class EndEffector(Joint):
    
    def __init__(self, mayaID, parent=None, name=None):
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
        return  '<# TargetID: \' %s \'' % (self.mayaID)

    def __repr__(self):
        return self.__str__()


#############################################################


MIN_JOINT_ID = 6 #1
MAX_JOINT_ID = 8 #4
JOINT_CHAIN_LENGTH = 20


#Initialize our positions from out Maya joint        
def init_joint_positions():
    #Joint Positions
    for key in JOINTS:
        #print key
        #Get position of joint
        pos = pm.xform(key, query=True, ws=True, t=True) #//in an array [x,y,z]
        JOINTS[key]['base-pos'] = Leap.Vector(pos[0], pos[1], pos[2])
        JOINTS[key]['pos'] = Leap.Vector(pos[0], pos[1], pos[2])

        #Set interpolation to be by rotation
        #pm.rotationInterpolation( key+'_rotateX', convert='quaternion' )
        #Get Rotatation of joint
        rot = pm.xform(key, query=True, rotation=True)   
        JOINTS[key]['rot'] = Leap.Vector(rot[0], rot[1], rot[2])

    #Get the Current Target Position
    posX = pm.xform(TARGET_KEY, query=True, t=True, ws=True)
    TARGET_POINT = Leap.Vector(posX[0], posX[1], posX[2])
    #PRoject to be in range of the armLength
    armLength = 20 #sum of the arm lengths
    basePtKey = BASE_KEY
    sphereCenter = JOINTS[basePtKey]['pos']
    JOINTS[TARGET_KEY]['pos'] = project_point(TARGET_POINT, sphereCenter, armLength)
    #print TARGET_POINT


def set_animation_keyframe():
    #Sets a key frame for all attributes of the current joint
    currentTime = get_current_time()
    for key in JOINTS:
        pma.setKeyframe(key, t=currentTime);
    increment_time()


def get_current_time():
    return CONFIG[TIME_KEY]

def get_max_time():
    return CONFIG['MAX_TIME']

def increment_time():
    CONFIG[TIME_KEY] = CONFIG[TIME_KEY] + 15
    pma.playbackOptions(loop='once', maxTime=CONFIG[TIME_KEY])
    currentTime = get_current_time()
    maxTimeBuffer =  get_max_time()-10
    if (currentTime >= maxTimeBuffer):
        increase_max_time()


def increase_max_time():
    CONFIG['MAX_TIME'] = CONFIG['MAX_TIME'] + 200
    pma.playbackOptions(maxTime=CONFIG['MAX_TIME'])



def reset_time(): 
    #Clear All the key frames for every joint
    currentMaxTime = pm.playbackOptions(query=True, maxTime=True)
    for jointKey in JOINTS: 
        #print jointKey 
        pm.cutKey(jointKey, time=(1, currentMaxTime), option='keys')
    #pm.refresh(force=True)
    CONFIG[TIME_KEY] = CONFIG['MIN_TIME']
    CONFIG['MAX_TIME'] = CONFIG['INITIAL_MAX_TIME']
    pma.playbackOptions(maxTime= CONFIG['INITIAL_MAX_TIME'])

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
def perform_ccd():  #formerly perform_ccd(self,targetTipPos)
    #Config bool that tells us if we should be incrementing jointnums from 1
    #If false we decrement from the max joint joint
    DISTANCE_THRESHOLD = 0.01
    iterations = 90
    jointNum = MAX_JOINT_ID

    USE_INCREASE_JOINT_NUM = False
    if USE_INCREASE_JOINT_NUM:
        jointNum = MIN_JOINT_ID

    init_joint_positions()
    targetTipPos = JOINTS[TARGET_KEY]['pos']
    distance = (targetTipPos - JOINTS[EFFECTOR_KEY]['pos']).magnitude


    while (distance > DISTANCE_THRESHOLD and iterations > 0):
        #setKeyframe
        #print get_current_time()
        set_animation_keyframe()
        #print distance
        if (distance > 1):
            CONFIG['JOINT_ANGLE_MULTIPLIER'] = 1.0
        else:
            CONFIG['JOINT_ANGLE_MULTIPLIER'] = 0.4

        #Current Joint & Position
        jointKey = 'joint'+str(jointNum)
        
        #Effector Position
        effectorPos = JOINTS[EFFECTOR_KEY]['pos']
        
        #Update our joint rotation based on the effector Position and the Target Effector Pos
        update_joint_rotation(jointKey, effectorPos, targetTipPos)
        
        #Update the coordinates of our tip position
        newCoords = pm.xform(EFFECTOR_KEY, query=True, ws=True, t=True)

        #Uodate Effector Position
        JOINTS[EFFECTOR_KEY]['pos'] = Leap.Vector(newCoords[0], newCoords[1], newCoords[2])
       
        #Update our distance from the target Point
        oldDistance = distance
        distance = (targetTipPos - JOINTS[EFFECTOR_KEY]['pos']).magnitude

        #Check if we have only moved a minimal amount of distance 
        if (Math.fabs(oldDistance-distance) < EPSILON):
            #Stop CCD
            break 
        #Decrement Iteration
        iterations = iterations - 1
        
        if USE_INCREASE_JOINT_NUM:
            #Move to next joint closest to the TP
            jointNum = jointNum + 1
            if jointNum > MAX_JOINT_ID:
                #keep the joint nums in bounds
                jointNum = MIN_JOINT_ID
        else:
            #Move onto next joint in the chain
            jointNum = jointNum - 1
            #Cycle look if jointNum goes out of bound
            if (jointNum < MIN_JOINT_ID):
                jointNum = MAX_JOINT_ID
                init_joint_positions()
        

def update_joint_rotation(jointKey, effectorPos, targetTipPos):
    #Position of the joiny
    jointPos = JOINTS[jointKey]['pos']
    #The Two Vectors of Interest
    V1 = (effectorPos-jointPos)
    V2 = (targetTipPos-jointPos)

    #Get current matrix - CURRENTLY DOES NOT INCORPORATE SCALE (cutting off values)
    mVals= pm.xform(jointKey, query=True, matrix=True, ws=True)

    #Construct matrix from values of our composite matrix
    mat = [ [ float(mVals[0]),  float(mVals[1]),  float(mVals[2]), float(mVals[3])  ], 
            [ float(mVals[4]),  float(mVals[5]),  float(mVals[6]), float(mVals[7])  ],
            [ float(mVals[8]),  float(mVals[9]), float(mVals[10]), float(mVals[11]) ],
            [ float(mVals[12]), float(mVals[13]), float(mVals[14]), float(mVals[15]) ]  ]

    #Turn mat into a transformation Matrix
    mat = dt.TransformationMatrix(dt.Matrix(mat))

    #Get Axis & Angle - Returns vector and angle in degrees
    axisAngle = pm.angleBetween(v1=(V1[0], V1[1], V1[2]), v2=(V2[0], V2[1], V2[2]))            

    #Make a quaternion from the axis and the angle
    axis = dt.Vector(axisAngle[0], axisAngle[1], axisAngle[2])

    #Convert Angle to Degrees
    angle =  (axisAngle[3]*DEG_TO_RAD)

    #Apply magic number to the joint to prevent moving too sharply to the targetPoint
    angle *= CONFIG['JOINT_ANGLE_MULTIPLIER'] #For radian 
    rotQuat = dt.Quaternion(angle, axis)

    #Apply Rotation to the matrix
    mat.addRotationQuaternion(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w, space='world')   #object space
    mat = mat.asMatrix()                                                    #Convert into a now matrix
    matAsFloatTuple = ( mat.a00, mat.a01, mat.a02, mat.a03, 
                        mat.a10, mat.a11, mat.a12, mat.a13,
                        mat.a20, mat.a21, mat.a22, mat.a23,
                        mat.a30, mat.a31, mat.a32, mat.a33
                        )

    #Set the composition matrix on the object
    pm.xform(jointKey, ws=True, matrix=matAsFloatTuple) #Removed Euler euler=True because it was rotating about local axes
    pm.refresh(force=True)


#Interfacing with a command port thatsend tip position updates
def receive_tip_position_from_leap(tpX, tpY, tpZ, lengthRatio):
    USE_DIRECTION = True
    if not USE_DIRECTION:
        #Map Point Value to maya CS
        print tpX
        clipped = clip_tip_position(tpX, tpY, tpZ)
        tpX = clipped[0]
        tpY = clipped[1]
        tpZ = clipped[2]
        print tpX
        #Flip the x-axis to reflect the leap CDS with MAYA
        tpX *= -1

        #Map the Components to fit into MAYA_CUBE Size RANGE
        tpX = ((tpX-LEAP_MIN_X)/LEAP_RANGE_X)*MAYA_RANGE_X + MAYA_MIN_X
        tpY = ((tpY-LEAP_MIN_Y)/LEAP_RANGE_Y)*MAYA_RANGE_Y + MAYA_MIN_Y
        tpZ = ((tpZ-LEAP_MIN_Z)/LEAP_RANGE_Z)*MAYA_RANGE_Z + MAYA_MIN_Z
    
    #Get the ratio of finger length from the Leap (with the baseline) and use it for our Maya Length
    lengthRatio = JOINT_CHAIN_LENGTH*lengthRatio
    print JOINT_CHAIN_LENGTH
    #Create a vector from the coordinates
    updatedPos = (Leap.Vector (tpX, tpY, tpZ))
    #Apply the joint chain length to the vector of motion & add the base position
    
    updatedPos = updatedPos*lengthRatio + JOINTS[BASE_KEY]['pos']
    #print updatedPos
    JOINTS[TARGET_KEY]['pos'] = updatedPos
    JOINTS[TARGET_KEY]['base-pos'] = updatedPos
    #Note: relative adds translation;absolute sets
    pm.xform(TARGET_KEY, ws=True, t=(updatedPos[0], updatedPos[1], updatedPos[2]), absolute=True)
    pm.refresh(force=True)

    #Perform CCD now :)
    perform_ccd()

def clip_tip_position(tpX, tpY, tpZ):
    #Clip X-Comp
    if (tpX < LEAP_MIN_X):
        tpX = LEAP_MIN_X
    elif (tpX > LEAP_MAX_X):
        tpX = LEAP_MAX_X
    #Clip Y-comp
    if (tpY < LEAP_MIN_Y):
        tpY = LEAP_MIN_Y
    elif (tpY > LEAP_MAX_Y):
        tpY = LEAP_MAX_Y
    #Clip z-comp
    if (tpZ < LEAP_MIN_Z):
        tpZ = LEAP_MIN_Z
    elif (tpZ > LEAP_MAX_Z):
        tpZ = LEAP_MAX_Z
    return [tpX, tpY, tpZ] 



def calculate_joint_chain_length():
    #initialize all joint positions
    init_joint_positions()


    wristPos =  palmPos = pm.xform('hand4_joint1', query=True, ws=True, t=True)
    wristPos =  Leap.Vector(wristPos[0], wristPos[1], wristPos[2])
    #print wristPos

    palmPos = pm.xform('hand4_joint1|joint2', query=True, ws=True, t=True) #//in an array [x,y,z]
    palmPos =  Leap.Vector(palmPos[0], palmPos[1], palmPos[2])
    #print palmPos

    #Get initial palm-to-finger length
    isFirst = True

    #Sum of Segment lengths
    chainLength = 0;

    #Include palm and wrist vect mag
    #chainLength = (palmPos - wristPos).magnitude

    #print chainLength

    #Now get length of vectors between two joints
    for index in range(MIN_JOINT_ID, MAX_JOINT_ID+1):  #PLUS 1 BECAUSE MAX joint ID doesn't go to the EFFECTOR

        firstKey = index
        secondKey = index +1

        firstJoint = JOINTS['joint'+str(firstKey)]
        secondJoint = JOINTS['joint'+str(secondKey)]
        #print 'joint'+str(firstKey) + '-' + 'joint'+str(secondKey)
        #Get the segment length between the two joints
        jointSegment = secondJoint['pos'] - firstJoint['pos']

        segmentLength = jointSegment.magnitude
        #print segmentLength
        if (isFirst):
            #print 'isFirst'
            #jointSegment = firstJoint['pos']-palmPos
            #segmentLength += jointSegment.magnitude
            isFirst = False

        #print segmentLength
        #Add to the chainLength
        chainLength += segmentLength
    #Set our new Chain length
    JOINT_CHAIN_LENGTH = chainLength
    #print chainLength
    print 'FROM OLD CL!:' +str(chainLength)
    #Add an extension so the mapping length is not directly on the sphere
    chainExtension = 5;
    #JOINT_CHAIN_LENGTH +=  chainExtension

#Main Loop for initialization
def main():
    #Close open ports & open our current port
    open_command_port()
    #init the Joint chain length
    calculate_joint_chain_length()

    #Initialize all joint positions
    init_joint_positions()

    #Reset the animation time
    reset_time()

    #Perform CCD (for testing)
    perform_ccd() 

if __name__ == '__main__':
    print 'main!'
    #main()


#Close open ports & open our current port
open_command_port()
#init the Joint chain length
calculate_joint_chain_length()

#Initialize all joint positions
init_joint_positions()

#Reset the animation time
reset_time()

#Perform CCD (for testing)
# perform_ccd() 


############################################################
####################### FINGER TEST ########################
############################################################
def FINGER_TEST():
    finger1 = Finger('R-IndexFinger')
    print finger1
    j6 = Joint('joint6')
    j7 = Joint('joint7')
    j8 = Joint('joint8')
    j9 = Joint('joint9')

    tp = Joint('targetPoint')

    finger1.add_joints([j6,j7,j8,j9])
    finger1.set_target(tp)

    #print tp.get_position() 
    #print finger1

    finger1.perform_ccd()
    #for joint in finger1.get_joints():
        #print joint.get_position()

    # print 'GET LENGTH CALL' + str(finger1.get_length())
    # print "NEW LENGTH: " +str(finger1.length)

FINGER_TEST()

############################################################
############################################################
############################################################




##################### END SCRIPT #######################
