import pymel.all as pm
from pymel.core import * 
import sys, Leap, string
import math as Math


#import pymel.api as pma
#PyMel Data Types for Matrices, Quaternions etc
import pymel.core.datatypes as dt 






#Target point of interest - initial conditions
#TP = [13.327005785798825, 5.933350084777719, 1.6255290651771213];
#TARGET_POINT = Leap.Vector(TP[0],TP[1], TP[2])


#Joints and Positions of finger

EFFECTOR_KEY = 'joint5'
TARGET_KEY = 'targetPoint'
DEG_TO_RAD = Math.pi/180
EPSILON = 0.001
MIN_JOINT_ID = 1

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
    'JOINT_ANGLE_MULTIPLIER': 0.5
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
   'joint1': {
        'base-pos': Leap.Vector(0, 0, 0),
        'pos': Leap.Vector(0, 0, 0),
        'rot': Leap.Vector(0, 90, 0),
        'name': 'palm'
    },
    'joint2': {
        'base-pos': Leap.Vector(0, 0, -5),
        'pos': Leap.Vector(0, 0, -5),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'knuckle'
    },
    'joint3': {
        'base-pos': Leap.Vector(0, 0, -10),
        'pos': Leap.Vector(0, 0, -10),
        'rot': Leap.Vector(0, 0, 0),
        'name': 'mid-joint'
    },
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
    },
    'targetPoint': {
        'base-pos': Leap.Vector(0,0,0),
        'pos': Leap.Vector(0,0,0),
        'rot': Leap.Vector(0,0,0),
        'name': 'targetPoint'
    }
}


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
    basePtKey = 'joint1'
    sphereCenter = JOINTS[basePtKey]['pos']
    JOINTS[TARGET_KEY]['pos'] = project_point(TARGET_POINT, sphereCenter, armLength)
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
    DISTANCE_THRESHOLD = 0.01
    iterations = 30
    jointNum = 4
    init_joint_positions()
    targetTipPos = JOINTS[TARGET_KEY]['pos']
    distance = (targetTipPos - JOINTS[EFFECTOR_KEY]['pos']).magnitude


    while (distance > DISTANCE_THRESHOLD and iterations > 0):
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
        
        #Move onto next joint in the chain
        jointNum = jointNum - 1
        
        #Cycle look if jointNum goes out of bound
        if (jointNum < MIN_JOINT_ID):
            jointNum = 4
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
def receive_tip_position_from_leap(tpX, tpY, tpZ):
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

    updatedPos = Leap.Vector (tpX, tpY, tpZ)
    print updatedPos
    JOINTS[TARGET_KEY]['pos'] = updatedPos
    JOINTS[TARGET_KEY]['base-pos'] = updatedPos
    #Note: relative adds translation;absolute sets
    pm.xform(TARGET_KEY, ws=True, t=(tpX, tpY, tpY), absolute=True)
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
# perform_ccd() 