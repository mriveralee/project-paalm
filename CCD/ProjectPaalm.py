 #!/usr/bin/python
# -*- coding: <encoding name> -*-
import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

import string, math


USE_MAYA = False

if USE_MAYA :
    import maya.cmds as cmds


#Listener for phaleangeal
class PAListener(Leap.Listener):


   
    #On initilization of listener
    def on_it(self, controller):
        print "Initialized"

    #On connect of listener to controller
    def on_connect(self, controller):
        print "Connected"

    #On disconnect of the listener to the controller
    def on_disconnect(self, controller):
        print "Disconnected"
    #On exit of listener
    def on_exit(self, controller):
        print "Exited"

    #On Frame being read from the Leap Do something
    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()
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
                fingers = sort_fingers_by_x(fingers)
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
                

                print "Direction: " + str(indexDir) + " Length: " + str(indexLength)
                ''' 
                 # The plan is to use the Direction of the finger vector to 
                 # create a point in Maya's 3D space from the rigged-finger 
                 # base-joint (the knuckle). With this point. We then run ccd
                 # on the rigged hand. We use the length ratio of the 
                 # Leap finger to the Maya Rigged Finger to move the finger appropriately
                '''
                #Length of effector in Maya
                mayaEffectorLength = 10
                #Map the finger direction to maya
                mayaDir = map_dir_to_maya(indexDir, mayaEffectorLength)
                #Effector Base (knuckle) Position in Maya
                mayaEffectorBasePos = Leap.Vector(0.0,0.0,0.0)
                #The new tip location in maya
                mEndTipPosition = get_maya_effector_tip_pos(mayaDir, mayaEffectorBasePos)
                
                if USE_MAYA:
                    cmds.select("joint5")
                    cmds.translate(mEndTipPosition.x, mEndTipPosition.y, mEndTipPosition.z)



                print "End Tip Position: " + str(mEndTipPosition)

                #NOW RUN CCD

    #CCD algorithm
    def perform_ccd(frame):
        print "ccd test"


######################

#Gets the Effector Tip Position using a maya mapped Direction and the base(knuckle position)
def get_maya_effector_tip_pos(effectorDir, effectorBasePos) :
    return effectorDir + effectorBasePos;



#Gets a direction vector in maya space
def map_dir_to_maya(leapFingerDir, mayaEffectorLength) :
    #takes a normal vector in 3D space from the leap,
    #maps it to a vector in Maya's 3D space that is used to 
    #compute a new end effector position in maya
    return leapFingerDir*mayaEffectorLength;





# Selection Sort the fingers based on  x-position 
# (left-most == Thumb, Right Hand OR Pinky, Left Hand)
def sort_fingers_by_x(fingers):  
    sortedFingers = [];
    #Make a list of the fingers that isn't const (FingerList is)
    for finger in fingers:
        sortedFingers.append(finger)
        print finger
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






##################################################################
##################### SAMPLE EXAMPLE LISTENER ####################
##################################################################
'''
class SampleListener(Leap.Listener):
    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        if not frame.hands.empty:
            # Get the first hand
            hand = frame.hands[0]

            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.empty:
                # Calculate the hand's average finger tip position
                avg_pos = Leap.Vector()
                for finger in fingers:
                    avg_pos += finger.tip_position
                avg_pos /= len(fingers)
                print "Hand has %d fingers, average finger tip position: %s" % (
                      len(fingers), avg_pos)

            # Get the hand's sphere radius and palm position
            print "Hand sphere radius: %f mm, palm position: %s" % (
                  hand.sphere_radius, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)

            # Gestures
            for gesture in frame.gestures():
                if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                    circle = CircleGesture(gesture)

                    # Determine clock direction using the angle between the pointable and the circle normal
                    if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
                        clockwiseness = "clockwise"
                    else:
                        clockwiseness = "counterclockwise"

                    # Calculate the angle swept since the last frame
                    swept_angle = 0
                    if circle.state != Leap.Gesture.STATE_START:
                        previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                        swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                    print "Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                            gesture.id, self.state_string(gesture.state),
                            circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

                if gesture.type == Leap.Gesture.TYPE_SWIPE:
                    swipe = SwipeGesture(gesture)
                    print "Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                            gesture.id, self.state_string(gesture.state),
                            swipe.position, swipe.direction, swipe.speed)

                if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                    keytap = KeyTapGesture(gesture)
                    print "Key Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            keytap.position, keytap.direction )

                if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                    screentap = ScreenTapGesture(gesture)
                    print "Screen Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            screentap.position, screentap.direction )

        if not (frame.hands.empty and frame.gestures().empty):
            print ""

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"
'''