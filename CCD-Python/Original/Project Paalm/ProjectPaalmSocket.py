#!/usr/bin/python
# -*- coding: utf-8 -*- 

#In Maya run MEL command 
#commandPort -name ":6001";


import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import string, math, MayaConnection


#Socket Information
MAYA_PORT = 6001
maya = MayaConnection.MayaConnection(MAYA_PORT)

#Demos
IS_TRACKING_DEMO = True

'''
Joint Names:
    joint1 = palm (no movement)
    joint2 = knuckle
    joint3 = mid-joint
    joint4 = tip-joint
    joint5 = tip-end (end effector position)
    base-pos : initial position <x,y,z> in world space
    pos : position <x,y,z> in world space
    rot : rotation <x,y,z> 
'''
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
    }
}



#Listener for phaleangeal angle approximator
class PAListener(Leap.Listener):


    #On initilization of listener
    def on_init(self, controller):
        #Make a new Maya Connection on port 6001
        self.is_peforming_ccd = False
        print "Initialized"

    #On connect of listener to controller
    def on_connect(self, controller):
        print "Connected"

    #On disconnect of the listener to the controller
    def on_disconnect(self, controller):
        print "Disconnected"

    #On exit of listener
    def on_exit(self, controller):
        maya.close()
        print "Exited"

    #On Frame being read from the Leap Do something
    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        if IS_TRACKING_DEMO:
            self.trackingMovementDemo(frame)

        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))
        #Check if we have hands
        elif (not self.is_peforming_ccd) and (not frame.hands.empty):
         
            # Get the first hand (Let's assume only Right hand for now)
            hand = frame.hands[0]
            #Get Palm Position
            palmPosition = hand.palm_position 
            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.empty:
                self.is_peforming_ccd = True;
                #Sort fingers to get order by 
                fingers = self.sort_fingers_by_x(fingers)
                #print fingers
                #Get first finger (Let's assume the only finger is the INDEX finger)
                indexFinger = fingers[0]
                #Fingers have direction, length, width, tip_velocity, tip_position, etc.
                #Not sure what I need so will have to figure that out later
                #indexPosition = indexFinger.tip_position
                #indexWidth = indexFinger.width
                #indexTipVelocity = 
                
                indexDir = indexFinger.direction
                indexLength = indexFinger.length
                

               # print "Direction: " + str(indexDir) + " Length: " + str(indexLength)
                ''' 
                 # The plan is to use the Direction of the finger vector to 
                 # create a point in Maya's 3D space from the rigged-finger 
                 # base-joint (the knuckle). With this point. We then run ccd
                 # on the rigged hand. We use the length ratio of the 
                 # Leap finger to the Maya Rigged Finger to move the finger appropriately
                '''
                effector = 'joint5'
                base_joint = 'joint2'
                #Effector Base Position in Maya - (Joint connected to the end effector)
                mayaEffectorBasePos = JOINTS[base_joint]['pos']
                #print "Base POS: " + str(mayaEffectorBasePos)

                #Length of effector in Maya
                mayaEffectorLength = ( JOINTS[effector]['pos'] - mayaEffectorBasePos ).magnitude

                ##Tip Length: mayaEffectorLength = ( JOINTS[effector]['pos'] - mayaEffectorBasePos ).magnitude
                
                #print "Length: " + str(mayaEffectorLength)
                
                #Map the finger direction to maya
                mayaDir = self.map_dir_to_maya(indexDir, mayaEffectorLength)
    
                #mayaCoords = maya.get_world_coords(base_joint)
                #print "Base Joint Coords: " + str(mayaCoords)

                #The new tip location in maya
                targetTipPos = self.get_maya_effector_tip_pos(mayaDir, mayaEffectorBasePos)
                #print "End Effector Pos: " + str(mEndTipPos)

                #NOW RUN CCD with the target end tip position
                self.perform_ccd(targetTipPos)

                #Update the position of the joint
                #maya.move(effector, mEndTipPos.x, mEndTipPos.y, mEndTipPos.z)


    #CCD algorithm - with a targetPos
    def perform_ccd(self, targetTipPos):
        print "ccd test"
        effector = 'joint5'
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
            except ValueError,e:
                break

            #Axis of Rotation
            rAxis = Leap.Vector.cross(pE_pC, pT_pC)
            #Get Rotation Matrix from Axis & Angle
            rotMat = Leap.Matrix(rAxis, theta)
            rMat = rotMat.to_array_3x3()

            phi = math.atan2(rMat[6], rMat[7])
            theta = math.acos(rMat[8])
            psi = -1*math.atan2(rMat[2], rMat[5])

            #Update pBase by rotating it
            #pBase = rotMat.transform_point(pBase)
            #JOINTS[jointKey]['pos'] = pBase
            #Update the position of our end effector
            JOINTS[effector]['pos'] = rotMat.transform_point(pEnd)

            maya.rotate(jointKey, phi, theta, psi)
            
            #Move to next Joint
            i -= 1
        #Turn back on frame updates
        self.is_peforming_ccd = False

    #Gets the Effector Tip Position using a maya mapped Direction and the base(knuckle position)
    def get_maya_effector_tip_pos(self, effectorDir, effectorBasePos) :
        return effectorDir + effectorBasePos;

    #Gets a direction vector in maya space
    def map_dir_to_maya(self, leapFingerDir, mayaEffectorLength) :
        #takes a normal vector in 3D space from the leap,
        #maps it to a vector in Maya's 3D space that is used to 
        #compute a new end effector position in maya
        return leapFingerDir.normalized * mayaEffectorLength;

    # Selection Sort the fingers based on  x-position 
    # (left-most == Thumb, Right Hand OR Pinky, Left Hand)
    def sort_fingers_by_x(self, fingers):  
        sortedFingers = [];
        #Make a list of the fingers that isn't const (FingerList is)
        for finger in fingers:
            sortedFingers.append(finger)
            #print finger
        #selection sort the fingers
        for i in range(0, len(sortedFingers)):
            min = i
            for j in range(i+1, len(sortedFingers)):
                x1 = sortedFingers[i].tip_position.x
                x2 = sortedFingers[j].tip_position.x
                if (x2 < x1):
                    min = j
            #Swap the min (leftmost)
            temp = sortedFingers[i]
            sortedFingers[i] = sortedFingers[min]
            sortedFingers[min] = temp
        return sortedFingers



    #TODO: Make a rotation matrix from an axis and angle
    def get_rotation_mat(self, axis, angle):
        print "Not Implemented"


    #TODO: Define initial/base positions through maya socket and joint names
    def initBasePositions(self):
        print "Not Implemented"

    #TODO: Define initial rotation angles for joints through maya socket
    def init_rotations(self):
        print "Not Implemented"



    def trackingMovementDemo(self, frame):
        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))
        #Check if we have hands
        if not frame.hands.empty:
            # Get the first hand (Let's assume only Right hand for now)
            hand = frame.hands[0]
            #Get Palm Position
            palmPosition = hand.palm_position 
            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.empty:
                #Sort fingers to get order by 
                fingers = self.sort_fingers_by_x(fingers)
                #print fingers
                #Get first finger (Let's assume the only finger is the INDEX finger)
                indexFinger = fingers[0]
                #Fingers have direction, length, width, tip_velocity, tip_position, etc.
                #Not sure what I need so will have to figure that out later
                #indexPosition = indexFinger.tip_position
                #indexWidth = indexFinger.width
                #indexTipVelocity = 
                
                indexDir = indexFinger.direction
                indexLength = indexFinger.length
                

               # print "Direction: " + str(indexDir) + " Length: " + str(indexLength)
                ''' 
                 # The plan is to use the Direction of the finger vector to 
                 # create a point in Maya's 3D space from the rigged-finger 
                 # base-joint (the knuckle). With this point. We then run ccd
                 # on the rigged hand. We use the length ratio of the 
                 # Leap finger to the Maya Rigged Finger to move the finger appropriately
                '''
                effector = 'joint5'
                base_joint = 'joint2'
                #Effector Base Position in Maya - (Joint connected to the end effector)
                mayaEffectorBasePos = JOINTS[base_joint]['pos']
                #print "Base POS: " + str(mayaEffectorBasePos)

                #Length of effector in Maya
                mayaEffectorLength = ( JOINTS[effector]['pos'] - mayaEffectorBasePos ).magnitude

                ##Tip Length: mayaEffectorLength = ( JOINTS[effector]['pos'] - mayaEffectorBasePos ).magnitude
                
                #print "Length: " + str(mayaEffectorLength)
                
                #Map the finger direction to maya
                mayaDir = self.map_dir_to_maya(indexDir, mayaEffectorLength)
    
                #mayaEffectorBasePos = maya.get_world_coords(jointBaseName)
                
                #The new tip location in maya
                mEndTipPos = self.get_maya_effector_tip_pos(mayaDir, mayaEffectorBasePos)
                print "End Effector Pos: " + str(mEndTipPos)
                #Select the joint
                jointEndName = "joint5"    
                #jointEndName = "-r joint5"            
                
                #Update the position of the joint
                maya.move(effector, mEndTipPos.x, mEndTipPos.y, mEndTipPos.z)
                #print "End Tip Position: " + str(mEndTipPosition)

######################




def main():
    # Create a sample listener and controller
    
    #Sample Listener usage
    #listener = SampleListener()
    listener = PAListener()        
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()

    # Remove the sample listener when done
    controller.remove_listener(listener)


if __name__ == "__main__":
    main()
