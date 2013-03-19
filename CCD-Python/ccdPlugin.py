import pymel.all as pm
import sys, Leap, string
import math as Math


#Target point of interest - initial conditions
#TP = [13.327005785798825, 5.933350084777719, 1.6255290651771213];
#TARGET_POINT = Leap.Vector(TP[0],TP[1], TP[2])

#Joints and Positions of finger
EFFECTOR_KEY = 'joint5'
TARGET_KEY = 'targetPoint'
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
def initializePositions():
    #Joint Positions
    for key in JOINTS:
        #print key
        #Get position of joint
        pos = pm.xform(key, query=True, t=True, ws=True) #//in an array [x,y,z]
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
    JOINTS[TARGET_KEY]['pos'] = projectPoint(TARGET_POINT, sphereCenter, armLength)
    print TARGET_POINT

        
#Projects a point on to the Sphere
def projectPoint(point, sphereCenter, armLength):
    #Radius of our reach sphere
    #armLength = 20
    #Vector in sphere coordinate system
    print "POINT"
    print point
    point = point-sphereCenter
    #Magnitude of the point
    pMag = point.magnitude
    #Vector to projected Point
    Q = (point/pMag)*armLength
    #The point projected onto the Sphere
    projectPt = Q + sphereCenter
    print "Printed"
    print projectPt
    return projectPt

#CCD algorithm - with a targetPos
def perform_ccd():  #formerly perform_ccd(self,targetTipPos)
    initializePositions()
    targetTipPos = JOINTS[TARGET_KEY]['pos']
    #print "ccd test"
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
    effector = 'joint5'
    distance = (JOINTS[EFFECTOR_KEY]['pos']).distance_to(targetTipPos)
    threshold = 0.5
    iterations = 100
    jointNum = 4
    while (distance > threshold and iterations > 0):
        if (jointNum < 1):
            jointNum = 4
            #Current Joint & Position
        jointKey = 'joint'+str(jointNum)
        jointPos = JOINTS[jointKey]['pos']
        #Tip Position
        effectorPos = JOINTS[EFFECTOR_KEY]['pos']

        #The Two Vectors of Interest
        V1 = (jointPos-effectorPos)
        V1 /= V1.magnitude
        V2 = (jointPos - targetTipPos)
        V2 /= V2.magnitude
        
        MAYA_TEST = True
        if (MAYA_TEST):
            #print "Maya Angle Between Test"
            #Now get Euler angles from Maya
            rot = pm.angleBetween(euler=True, v1=V1.to_float_array(), v2=V2.to_float_array()) 
        
            #Now we rotate about these angles on the current Joint
            pm.xform(jointKey, os=True, euler=True, rotation=(rot[0], rot[1], rot[2]));
        else:
            print "Axis-Angle Conversion Test"
            #Angle between the two vectors
            theta = Math.acos(Leap.Vector.dot(V1, V2)) #V1.angle_to(V2) #
            print "Angle: " + str(theta)
            #Get the rotation axis
            axis = Leap.Vector.cross(V1, V2)
            #convert to Euler Angles
            rot = axisAngletoEuler(axis, theta)
            pm.xform(jointKey, os=True, rotation=(rot[0], rot[1], rot[2]));


        #Update the coordinates of our tip position
        newCoords = pm.xform(EFFECTOR_KEY, query=True, t=True, ws=True)
        #print "Old Tip Pos: "
        #print JOINTS[effector]['pos']
        JOINTS[EFFECTOR_KEY]['pos'] = Leap.Vector(newCoords[0], newCoords[1], newCoords[2])
        #print "New Tip Pos: "
        #print JOINTS[effector]['pos']

        #Move onto next joint in the chain
        jointNum = jointNum - 1
        #Update distance
        #print "HERE!"
        distance = (JOINTS[EFFECTOR_KEY]['pos']).distance_to(targetTipPos)
        #print "---EFFECTOR POSITION"
        #print JOINTS[EFFECTOR_KEY]['pos']
        #print "---Target Position"
        #print JOINTS[TARGET_KEY]['pos']
        print "DISTANCE:" + str(distance)
        
        #updatePositionsFromIndex(jointNum)
        initializePositions()
        #Decrement Iteration
        iterations = iterations - 1


#Converts an axis and angle into a Euler Rotation Vector <x,y,z>
#Param <axis> is an array [x,y,z] of the axis components
#Angle is the angle that this axis is rotated by
def axisAngletoEuler(axis, angle):
    #Initialize output Euler angles to be all 0
    eulerAngles = [0,0,0]
    
    #Normalize the axis to be unit vector
    axis = axis.normalized

    #angle components
    s = Math.sin(angle)
    c = Math.cos(angle)
    t = 1.0-c
    #Axis components
    aX = axis[0]
    aY = axis[1]
    aZ = axis[2]

    # if ((aX*aY*t + aZ*s) > 0.998):
    #     eulerAngles[0] = 2*Math.atan2(aX*Math.sin(angle/2), Math.cos(angle/2))
    #     eulerAngles[1] = Math.pi/2
    #     eulerAngles[2] = 0
    # elif ((aX*aY*t + aZ*s) > -0.998):
    #     eulerAngles[0] = -2*Math.atan2(aX*Math.sin(angle/2), Math.cos(angle/2))
    #     eulerAngles[1] = -Math.pi/2
    #     eulerAngles[2] = 0
    # else: 
    eulerAngles[0] = Math.atan2(aY * s - aX * aZ * t, 1 - (aY*aY+ aZ*aZ ) * t)
    eulerAngles[1] = Math.asin(aX * aY * t + aZ * s)
    eulerAngles[2] = Math.atan2(aX * s - aY * aZ * t, 1 - (aX*aX + aZ*aZ) * t)
    #convert Radian angles to degrees
    for i in range(0, len(eulerAngles)):
        eulerAngles[i] = Math.degrees(eulerAngles[i])
    print "Euler angles ---"
    print eulerAngles
    #return out angle
    return eulerAngles
            
def updatePositionsFromIndex(index):
    #Joint Positions
    while (index > 0):
        jointKey = 'joint'+str(index)
        #Get position of joint
        pos = pm.xform(jointKey, query=True, t=True, ws=True) #//in an array [x,y,z]
        JOINTS[jointKey]['base-pos'] = Leap.Vector(pos[0], pos[1], pos[2])
        JOINTS[jointKey]['pos'] = Leap.Vector(pos[0], pos[1], pos[2])

        #Get Rotatation of joint
        rot = pm.xform(jointKey, query=True, rotation=True)   
        JOINTS[jointKey]['rot'] = Leap.Vector(rot[0], rot[1], rot[2])
        index -= 1
initializePositions()
perform_ccd() 