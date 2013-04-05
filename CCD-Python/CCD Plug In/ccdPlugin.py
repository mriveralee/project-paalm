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
    'CURRENT_TIME': 1,                   #Current Animation time
    'MAX_TIME': 100
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


open_command_port()

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
    CONFIG[TIME_KEY] = 1
    CONFIG['MAX_TIME'] = 100

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
        print get_current_time()
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
     
# init_joint_positions()
reset_time()
perform_ccd()
# perform_ccd() 