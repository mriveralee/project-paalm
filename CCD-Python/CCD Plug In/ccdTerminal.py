#!/usr/bin/python
# -*- coding: utf-8 -*- 

#In Maya run MEL command 
#commandPort -name ":6001";


import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import string, math, MayaConnection
import time


#Socket Information
MAYA_PORT = 6001
maya = MayaConnection.MayaConnection(MAYA_PORT)

#Demos
IS_TRACKING_DEMO = True
MAYA_EFFECTOR_LENGTH = 1
FRAME_SLEEP_TIME = 0.2



#Listener for phaleangeal angle approximator
class PAListener(Leap.Listener):
    NEED_BASELINE = True
    MAX_BASELINE_FRAMES = 1000
    CURRENT_NUM_BASELINE_FRAMES = 0
    MAX_TIP_LENGTH = 0.0

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
        #Verify we have hands to find fingers on - Assume left hand only pointer finger out
        if (not frame.hands.empty):
         
            # Get the first hand (Let's assume only Right hand for now)
            hand = frame.hands[0]
            #Get Palm Position
            palmPosition = hand.palm_position 
            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.empty:

                #Get a BaseLine of length for the index finger
                if self.NEED_BASELINE:
                    self.capture_baseline_lengths(fingers)
                    return


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
                
                #TTRY TIP AND PALM POS
                indexPosition = indexFinger.tip_position

                indexLength = indexFinger.length
                #Direction vector in Maya
                mayaDir = self.map_dir_to_maya(indexDir, MAYA_EFFECTOR_LENGTH)

                #The new tip location in maya - Assume base position is at <0,0,0>
                USE_DIRECTION = True
              
                if (USE_DIRECTION):
                    targetTipPos = indexDir.normalized #mayaDir #self.get_maya_effector_tip_pos(mayaDir, mayaEffectorBasePos)
                else:
                    targetTipPos = indexFinger.tip_position

                #print "End Effector Pos: " + str(mEndTipPos)
                print targetTipPos
                #Flip the X-direction because maya's is the other way
                print (targetTipPos*20)

                lengthRatio = indexLength/self.MAX_TIP_LENGTH
                print "Index Length Ratio: " + str(lengthRatio)
                maya.send_tip_position_to_maya(targetTipPos[0],targetTipPos[1],targetTipPos[2], lengthRatio)
                

                time.sleep(FRAME_SLEEP_TIME)
                #NOW RUN CCD with the target end tip position
                #self.perform_ccd(targetTipPos)

                #Update the position of the joint
                #maya.move(effector, mEndTipPos.x, mEndTipPos.y, mEndTipPos.z)


    #Gets a baseline avg of the finger tip lengths for mapping to Maya
    def capture_baseline_lengths(self, fingers):
        if (self.CURRENT_NUM_BASELINE_FRAMES < self.MAX_BASELINE_FRAMES and (not fingers.empty)):
            #Sort the Fingers
            fingers = self.sort_fingers_by_x(fingers)
            #Grab index finger
            indexFinger = fingers[0]
            #Sum the length into the BASELINE Length
            self.MAX_TIP_LENGTH += indexFinger.length
            print "BASE: " + str(self.CURRENT_NUM_BASELINE_FRAMES)+ ", Length: " + str(indexFinger.length)
            self.CURRENT_NUM_BASELINE_FRAMES += 1
        #When we have the max number of baseline frames, average the values for the fingers
        #Then begin actual target point collection
        if (self.CURRENT_NUM_BASELINE_FRAMES == self.MAX_BASELINE_FRAMES):
            self.MAX_TIP_LENGTH = self.MAX_TIP_LENGTH/self.MAX_BASELINE_FRAMES
            self.NEED_BASELINE = False
            print "Index Average Max Length: " + str(self.MAX_TIP_LENGTH)




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
