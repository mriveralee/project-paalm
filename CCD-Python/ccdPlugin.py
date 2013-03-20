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
    JOINTS[TARGET_KEY]['pos'] = projectPoint(TARGET_POINT, sphereCenter, armLength)
    #print TARGET_POINT

        
#Projects a point on to the Sphere
def projectPoint(point, sphereCenter, armLength):
    #Radius of our reach sphere
    #armLength = 20
    #Vector in sphere coordinate system
    #print "POINT"
    #return point
    print "Projecting the point"
    print point


    sPoint = point-sphereCenter
    #Magnitude of the point
    pMag = sPoint.magnitude

    if (pMag <= armLength):
        print "in range returning same point"
        return point

    #Vector to projected Point
    Q = (point/pMag)*armLength
    #The point projected onto the Sphere
    projectPt = Q + sphereCenter
    print "Projected as"
    print projectPt
    return projectPt

#CCD algorithm - with a targetPos
def perform_ccd():  #formerly perform_ccd(self,targetTipPos)
    DISTANCE_THRESHOLD = 0.1
    iterations = 1
    jointNum = 4
    MAYA_TEST = True
    #Initialize Pisitions
    initializePositions()
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
    targetTipPos = JOINTS[TARGET_KEY]['pos']
    distance = (targetTipPos - JOINTS[EFFECTOR_KEY]['pos']).magnitude


    while (distance > DISTANCE_THRESHOLD and iterations > 0):
        #Current Joint & Position
        jointKey = 'joint'+str(jointNum)
        jointPos = JOINTS[jointKey]['pos']

        #Effector Position
        effectorPos = JOINTS[EFFECTOR_KEY]['pos']

        #The Two Vectors of Interest
        V1 = (effectorPos-jointPos)
        #V1 /= V1.magnitude
        V2 = (targetTipPos-jointPos)
        #V2 /= V2.magnitude
        print "V1 is "
        print V1
        print "V2 is:"
        print V2
  
        if (MAYA_TEST):
            #print "Maya Angle Between Test"

            #Now get Euler angles from Maya that will move v1 to be v2
        
            rot = pm.angleBetween(euler=True, v1=(V1[0], V1[1], V1[2]), v2=(V2[0], V2[1], V2[2]))
            print "Rotation is:"
            print rot


            #Adjusting 'joints
           # print pm.joint('joint4', edit=True, angleX=-60.27)'


           #axisAngle = pm.angleBetween(v1=(V1.x, V1.y, V1.z), v2=(V2.x, V1.y, V1.z)) 
           #axis = (axisAngle[0], axisAngle[1], axisAngle[2])
           #angle = axisAngle[3]
           #pm.xform(jointKey, ws=True, a=True, rotateAxis=axis,  rotation=(angle, angle,angle))

            #Now we rotate about these angles on the current Joint
            #removed 
            #pm.xform(jointKey, ws=True,  rotation=(rot[0], rot[1], rot[2]))  #Removed Euler euler=True because it was rotating about local axes
            

            #Trying Rotate Function
            #QUICK HACK- NOTICED ROTATING IN SUCCESSION CAUSES ISSUES SO WE MOVE TO ORIGINAL PLACE AND TRY ROTATION
            pm.rotate(jointKey, [0, 0, 0], absolute=True)
            pm.refresh(force=True)
            pm.rotate(jointKey, [rot[0], rot[1], rot[2]] , absolute=True)  #Removed Euler euler=True because it was rotating about local axes
            pm.refresh(force=True)
            #input("Press Enter to continue...")

        #FORCE REFRESH OF VIEWS
        #pm.refresh(force=True)

        #Update the coordinates of our tip position
        newCoords = pm.xform(EFFECTOR_KEY, query=True, ws=True, t=True)
        print 'Current Effector Coords'
        print newCoords
        print 'Target Coords'
        print targetTipPos

        #Uodate Effector Position
        JOINTS[EFFECTOR_KEY]['pos'] = Leap.Vector(newCoords[0], newCoords[1], newCoords[2])
       
        #Update our distance from the target Point
        distance = (targetTipPos - JOINTS[EFFECTOR_KEY]['pos']).magnitude
        print "Our Distance is"
        print distance
        #Decrement Iteration
        iterations = iterations - 1
        #Move onto next joint in the chain
        jointNum = jointNum - 1
        #Cycle look if jointNum goes out of bound
        if (jointNum < 1):
            jointNum = 4
            initializePositions()


initializePositions()
perform_ccd() 