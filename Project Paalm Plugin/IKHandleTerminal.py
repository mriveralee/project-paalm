#!/usr/bin/python
# -*- coding: utf-8 -*- 

#In Maya run MEL command 
#commandPort -name ":6001";


import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import string, math, MayaConnection
import time



############################################################
##################### MAYA CONNECTION ######################
############################################################
# Class: MayaConnection
# Basic Socket connection to Maya command port with
# interface for making MEL commands in python

#Socket Information
import socket

#In Maya run MEL command 
#commandPort -pre trs -n ":9100";
#commandPort -name ":6001"; -> Send Port
#commandPort -n ":6002";  -> Receive Poort

#Socket Information
MAYA_PORT = 6001

class MayaConnection():
    
    def __init__(self, PORT):
        self.maya = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.maya.connect(('localhost', PORT))
        self.port = PORT

    #Close the socket port
    def close(self):
        self.maya.close()
             
    #Sends a tip position update to Maya to trigger a perform_ccd() action 
    def send_tip_position(self, tpX, tpY, tpZ, lengthRatio):
        command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        self.maya.send(command)
        #print command

    #Sends a target queue update to Maya to trigger a perform_ccd() action 
    def send_target_queue(self, targetQueue):
        command = 'python(\"receive_target_queue(%s)\")\n' % targetQueue
        #command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        self.maya.send(command)
        #print command

#Demos
IS_TRACKING_DEMO = True
MAYA_EFFECTOR_LENGTH = 1
FRAME_SLEEP_TIME = 0.0


############################################################
####################### FINGER DATA ########################
############################################################
#Finger data just stores information about our max finger Lengths
#With a reference to a finger ID 0 == Thumb, 4 == Pinky

class FingerData():
    def __init__(self, index):
        self.index = index
        self.baseLength = 0
        self.sumBaseLengths = 0
        self.baseLengthCount = 0

    def __repr__(self):
        return self.__str__()
    def __str__(self):
        return '{ "id": %s, "baseLength": %s}' % (self.index, self.baseLength)

    def add_base_length(self, baseLength):
        #print 'index: %s, sum_lengths: %s, count: %s' % (self.index, self.sumBaseLengths, self.baseLengthCount)
        self.sumBaseLengths +=  baseLength
        self.baseLengthCount += 1

    def compute_base_length(self):
        self.baseLength = self.sumBaseLengths/self.baseLengthCount
        print 'Index: %s, Average Length: %s' % (self.index, self.baseLength)

    def get_finger_id(self):
        return self.index

    def get_base_length(self):
        return self.baseLength


############################################################
################ PHALANGEAL ANGLE LISTENNER ################
############################################################
#Listener for phaleangeal angle approximator

class PAListener(Leap.Listener):

    #On initilization of listener
    def on_init(self, controller):
        #Make a new Maya Connection on port 6001
        self.is_peforming_ccd = False
        self.fingerData = []
        self.captureBaseline = True
        self.maxNumBaselineFrames = 250
        self.numBaselineFrames = 0
        self.numFingers = 5
        self.receiveFrame = True
        self.maxSkipCount = 30
        self.skipCount = self.maxSkipCount


        #Initialize finger data
        self.init_fingers()

        #Initialize Maya connection
        self.init_maya_connection()

        print "Initialized"

    #Initialize five fingers for this Listener
    def init_fingers(self):
        #Initialize only 4 fingers for the right hand
        for index in range(0, self.numFingers):
            data = FingerData(index)
            #Add to our data array
            self.fingerData.append(data)

    #Initilize Maya Connection
    def init_maya_connection(self):
        self.mayaConnection= MayaConnection(MAYA_PORT)

    #Get the finger data for an id
    def get_finger_data(self, index):
        return self.fingerData[index]

    #On connect of listener to controller
    def on_connect(self, controller):
        print "Connected"

    #On disconnect of the listener to the controller
    def on_disconnect(self, controller):
        self.mayaConnection.close()
        print "Disconnected"

    #On exit of listener
    def on_exit(self, controller):
        self.mayaConnection.close()
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
            if (not fingers.empty):

                #Get a BaseLine of length for the fingers if we need them
                if self.captureBaseline:
                    self.capture_baseline_lengths(fingers)
                    return

                #Frame control rate for the leap
                if (not self.receiveFrame):
                    self.skipCount = self.skipCount - 1
                    # print self.skipCount
                    if (self.skipCount == 0):
                        self.receiveFrame = True
                        self.skipCount = self.maxSkipCount
                    return;
                self.receiveFrame = False

                #Sort fingers to get order by 
                fingers = self.sort_fingers_by_x(fingers)
                #print fingers 
                
                #Queue for storing our target mapping data
                targetQueue = self.get_full_hand_queue(fingers)

                #Adjust the ids based on the number of fingers
                currentNumFingers = len(fingers)
                if currentNumFingers == 1:
                    print '1 - INDEX'
                    targetQueue = self.get_pointer(targetQueue)

                elif currentNumFingers == 2 :
                    print '2 - PEACE'
                    targetQueue = self.get_peace_sign(targetQueue)
                    #return

                elif currentNumFingers == 3:
                    print '3 - ROCKER'
                    targetQueue = self.get_peace_sign(targetQueue)
                    #targetQueue = self.get_rocker(targetQueue)
                    #print targetQueue
                    #return

                elif currentNumFingers == 4:
                    targetQueue = self.get_spongebob(targetQueue)



                self.mayaConnection.send_target_queue(targetQueue)

                #print fingers

    #set the order of the finger ids to be 1, 2 for index and middle
    def get_peace_sign(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        return targetQueue

    #Set the order of the finger ids to be 0, 1,4 for thumb, index, pinky
    def get_rocker(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        targetQueue[2]['id'] = 4
        return targetQueue

    #Set the finger ID to be '1' for the index
    def get_pointer(self, targetQueue):
        targetQueue[0]['id'] = 0
        return targetQueue

    def get_spongebob(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        targetQueue[2]['id'] = 2
        targetQueue[3]['id'] = 3
        return targetQueue


    #gets a queue of finger data for a full hand
    def get_full_hand_queue(self, fingers):
        #Queue for storing our target mapping data
        targetQueue = []
        for index in range(0, len(fingers)):
            finger = fingers[index]

            mappedTarget = self.get_target_mapping(finger, index)
            #Add to our queue
            targetQueue.append(mappedTarget)
        return targetQueue
        #Take the queue and send all the data to maya!
        #print targetQueue

    #Gets a target mapping for sending to maya 
    def get_target_mapping(self, finger, index):
            #Fingers have direction, length, width, tip_velocity, tip_position, etc.
            #Not sure what I need so will have to figure that out later

            #Get the finger data associated with this fingerID
            #print index
            fingerData = self.get_finger_data(index)
            maxFingerLength = fingerData.get_base_length()

            #Get the finger dir and length
            fDir = finger.direction
            fLength = finger.length
            fPos = finger.tip_position

            #Are we only using direction or do we want positon?
            USE_DIRECTION = True

            if USE_DIRECTION:
                #Use the directional vector for mapping
                targetDir = fDir.normalized
            else:
                #Use the finger tip for mapping
                targetDir = finger.tip_position

            #The ratio of the length to the maxLength
            fLengthRatio = fLength/maxFingerLength

            mappedTarget = {
                'id': index,
                'dir': [targetDir[0], targetDir[1], targetDir[2]],
                'length_ratio': fLengthRatio
            }

            return mappedTarget
    #Returns if we should be capturing baseline length data
    def should_capture_baseline(self):
        return self.captureBaseline
    #Increment the number of baseline frames captures
    def increment_num_baseline_frames(self):
        self.numBaselineFrames += 1
        if (self.numBaselineFrames > self.maxNumBaselineFrames):
            self.captureBaseline = False 
   
    #Returns the current number of baseline frames
    def get_num_baseline_frames(self):
        return self.numBaselineFrames

    #Gets a baseline length for the fingers to compute the average
    def capture_baseline_lengths(self, fingers):
        if (self.should_capture_baseline() and len(fingers) == self.numFingers):
            #Sort the Fingers
            fingers = self.sort_fingers_by_x(fingers)
            
            #For each finger, get its length and then add to the finger data
            for index in range(0, len(fingers)):
                #Get finger at index
                finger = fingers[index]
                #Get finger length
                fLength = finger.length

                #Update the fingerData to include the length
                fingerData = self.get_finger_data(index)
                fingerData.add_base_length(fLength)

            #Sum the length into the BASELINE Length
            print 'Calculating Average of Max Base Lengths - Frame %s of %s' % (self.get_num_baseline_frames(), self.maxNumBaselineFrames)
            
            #Increment our count of baseline frames
            self.increment_num_baseline_frames()

        #When we have the max number of baseline frames, average the values for the fingers
        #Then begin actual target point collection
        if not self.should_capture_baseline():
            #For each fingerData
            for data in self.fingerData:
                #average all the base frame lengths
                data.compute_base_length()

    # Sort the fingers based on  x-position 
    # (left-most == Thumb, Right Hand OR Pinky, Left Hand)
    def sort_fingers_by_x(self, fingers):  
        #Make a list of the fingers that isn't const (FingerList is)
        sortedFingers = [];
        for finger in fingers:
            sortedFingers.append(finger)
            #print finger
        #selection sort the fingers\
        sortedFingers = sorted(sortedFingers, key=lambda finger: finger.tip_position.x)
        return sortedFingers



############################################################
########################### MAIN ###########################
############################################################
def main():
    # Create a listener and controller
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

############################################################
######################## END SCRIPT ########################
############################################################