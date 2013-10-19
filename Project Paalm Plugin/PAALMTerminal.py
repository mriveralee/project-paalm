#!/usr/bin/python
# -*- coding: utf-8 -*- 
'''
'' * Project PAALM
'' * Terminal Plug-in  - for tracking hand data using the Leap Motion Controller
'' @author: Michael Rivera
'' @date: 10/17/2013
''  
'''
############################################################################
import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import string, time, socket
import math as Math

#######################################################
'''
#############################################################
## GLOBALS VARS  ############################################
#############################################################
'''
## Author Information ##
PAALM_AUTHOR = 'Michael Rivera'
PAALM_AUTHOR_WEBSITE = 'http://mikeriv.com'
PAALM_ABOUT_WEBSITE = 'http://projectpaalm.blogspot.com'


## The Leap Controller Reference ##
LEAP_CONTROLLER = Leap.Controller()

## The PAALM Leap Listener Reference ##
PAALM_LEAP_LISTENER = PAListener()

## Config vars for the leap data ##
CONFIG = {
    'PALM_INDEX': 9999,
}

## Command Port for Maya ##
MAYA_PORT = 6001



'''############################################################
''##################### MAYA CONNECTION #######################
''#############################################################
'''
# Class: MayaConnection
# Basic Socket connection to Maya command port with
# interface for making MEL commands in python

# Socket Information
import socket

# In Maya run MEL command 
#commandPort -pre trs -n ":9100"
#commandPort -name ":6001" -> Send Port
#commandPort -n ":6002"  -> Receive Poort

# Socket Information
MAYA_PORT = 6001

class MayaConnection():
    
    def __init__(self, PORT):
        self.mayaSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mayaSocket.connect(('localhost', PORT))
        self.port = PORT

    # Close the socket port
    def close(self):
        self.mayaSocket.close()
             
    # Sends a tip position update to Maya to trigger a perform_ccd() action 
    def send_tip_position(self, tpX, tpY, tpZ, lengthRatio):
        command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        self.mayaSocket.send(command)
        #print command

    # Sends a target queue update to Maya to trigger a perform_ccd() action 
    def send_target_queue(self, targetQueue):
        command = 'python(\"update_IK_targets(%s)\")\n' % targetQueue
        #command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        # print targetQueue
        self.mayaSocket.send(command)
        #print command



'''############################################################
'' ## Calibration / Leap Controller Listening #################
'' ############################################################
'''
############################################################
####################### FINGER DATA ########################
############################################################
# Finger data just stores information about our max finger Lengths
# With a reference to a finger ID 0 == Thumb, 4 == Pinky

class FingerLengthData():
    
    '''
    '' Constructor
    '''
    def __init__(self, index):
        self.index = index
        self.baseLength = 0
        self.sumBaseLengths = 0
        self.baseLengthCount = 0

    '''
    '' Returns the string representation of this object
    '''
    def __repr__(self):
        return self.__str__()

    '''
    '' Returns the string representation of this object
    '''
    def __str__(self):
        return '{ "id": %s, "baseLength": %s}' % (self.index, self.baseLength)

    '''
    '' Adds the baseLength for averaging
    '''
    def add_base_length(self, baseLength):
        # print 'index: %s, sum_lengths: %s, count: %s' % (self.index, self.sumBaseLengths, self.baseLengthCount)
        self.sumBaseLengths +=  baseLength
        self.baseLengthCount += 1

    '''
    '' Averages all the base lengths
    '''
    def compute_base_length(self):
        self.baseLength = self.sumBaseLengths/self.baseLengthCount
        print 'Index: %s, Average Length: %s' % (self.index, self.baseLength)

    '''
    '' Returns this fingerLengthData's id
    '''
    def get_finger_id(self):
        return self.index

    '''
    '' Returns the average length
    '''
    def get_base_length(self):
        return self.baseLength


############################################################
################ PHALANGEAL ANGLE LISTENNER ################
############################################################
# Listener for phaleangeal angle approximator

class PAListener(Leap.Listener):

    '''
    '' Called on initilization of listener
    '''
    def on_init(self, controller):
        self.isTracking = True
        self.fingerLengthData = []
        self.shouldCalibrate = True
        self.maxCalibrationFrames = 250
        self.numCalibrationFrames = 0
        self.numFingers = 5
        self.receiveFrame = True
        self.maxFrameSkipCount = 30
        self.frameSkipCount = self.maxFrameSkipCount
        self.shouldSendPalmData = False
        self.showDebugLogs = False

        # Initialize finger data
        self.init_fingers()

        print "Initialized"

    ''' 
    '' Initialize five fingers for this Listener
    '''
    def init_fingers(self):
        # Initialize only 4 fingers for the right hand
        for index in range(0, self.numFingers):
            data = FingerLengthData(index)
            #Add to our data array
            self.fingerLengthData.append(data)


    '''
    '' Get the finger data for an id
    '''
    def get_finger_data(self, index):
        return self.fingerLengthData[index]

    '''
    '' On connect of listener to controller
    '''
    def on_connect(self, controller):
        print "Connected"

    ''' 
    '' On disconnect of the listener to the controller
    '''
    def on_disconnect(self, controller):
        print "Disconnected"

    '''
    '' On exit of listener
    '''
    def on_exit(self, controller):
        print "Exited"

    '''
    '' On Frame being read from the Leap Do something
    '''
    def on_frame(self, controller):
        # Do not send data if we aren't tracking
        if (not self.is_tracking()):
            return
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        # Verify we have hands to find fingers on - Assume left hand only pointer finger out
        if (not frame.hands.is_empty):
         
            # Get the first hand (Let's assume only Right hand for now)
            hand = frame.hands[0]
            # Get Palm Position
            palmPosition = hand.palm_position 

            palm = {
                'position': hand.palm_position,
                'normal': hand.palm_normal
            }

            # Check if the hand has any fingers
            fingers = hand.fingers
            if (not fingers.is_empty):

                # Get a BaseLine of length for the fingers if we need them
                if (self.should_calibrate()):
                    self.calibrate_lengths(fingers)
                    return

                # Frame control rate for the leap
                if (not self.receiveFrame):
                    self.frameSkipCount = self.frameSkipCount - 1
                    # print self.frameSkipCount
                    if (self.frameSkipCount == 0):
                        self.receiveFrame = True
                        self.frameSkipCount = self.maxFrameSkipCount
                    return
                self.receiveFrame = False

                # Sort fingers to get order by 
                fingers = self.sort_fingers_by_x(fingers)
                #print fingers 
                
                # Queue for storing our target mapping data of fingers and the palm
                targetQueue = self.get_full_hand_queue(fingers, palm)

                # Adjust the ids based on the number of fingers
                currentNumFingers = len(fingers)          
                if currentNumFingers == 1:
                    
                    self.printd('1 - INDEX')
                    targetQueue = self.get_pointer(targetQueue)

                elif currentNumFingers == 2 :
                    self.printd('2 - PEACE')
                    targetQueue = self.get_peace_sign(targetQueue)
                    #return
                elif currentNumFingers == 3:
                    self.printd('3 - ROCKER')
                    targetQueue = self.get_peace_sign(targetQueue)
                    #targetQueue = self.get_rocker(targetQueue)
                    #print targetQueue
                    #return
                elif currentNumFingers == 4:
                    targetQueue = self.get_spongebob(targetQueue)

                #print fingers

                # Update the IK Targets in maya
                # Send data to maya
                self.mayaConnection.send_target_queue(targetQueue)

                #update_IK_targets(targetQueue)

    '''
    '' Prints Debug statements if debugging is enabled
    '''
    def printd(self, statement):
        if (self.showDebugLogs):
            print(statement)

    '''
    '' Set the order of the finger ids to be 1, 2 for index and middle
    '''
    def get_peace_sign(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        return targetQueue

    '''
    '' Set the order of the finger ids to be 0, 1,4 for thumb, index, pinky
    '''
    def get_rocker(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        targetQueue[2]['id'] = 4
        return targetQueue

    '''
    '' Set the finger ID to be '1' for the index
    '''
    def get_pointer(self, targetQueue):
        targetQueue[0]['id'] = 0
        return targetQueue

    '''
    '' Spongebob finger pose
    '''
    def get_spongebob(self, targetQueue):
        targetQueue[0]['id'] = 0
        targetQueue[1]['id'] = 1
        targetQueue[2]['id'] = 2
        targetQueue[3]['id'] = 3
        return targetQueue


    '''
    '' Gets a queue of finger data for a full hand such that
    '' the right-hand thumb corresponds to index 0 and the 
    '' right-hand pinky corresponds to the index 4 
    '''
    def get_full_hand_queue(self, fingers, palm):
        # Queue for storing our target finger mapping data
        targetQueue = []
        for index in range(0, len(fingers)):
            finger = fingers[index]

            mappedTarget = self.get_finger_target_mapping(finger, index)
            # Add to our queue
            targetQueue.append(mappedTarget)

        # Add palm data if necessary
        if (self.is_tracking_palm_position()):
            # Create the palm target
            palmTarget = self.get_palm_target_mapping(palm)
            # Add the palm information to the target queue
            targetQueue.append(palmTarget)

        # Take the queue and send all the data to maya!
        return targetQueue
        #print targetQueue

    '''
    '' Gets a target mapping for sending to maya 
    '''
    def get_finger_target_mapping(self, finger, index):
            # Fingers have direction, length, width, tip_velocity, tip_position, etc.
            # Not sure what I need so will have to figure that out later

            # Get the finger data associated with this fingerID
            #print index
            fingerLengthData = self.get_finger_data(index)
            maxFingerLength = fingerLengthData.get_base_length()

            # Get the finger dir and length
            fDir = finger.direction
            fLength = finger.length
            fPos = finger.tip_position

            if USE_LEAP_DIRECTION:
                # Use the directional vector for mapping
                targetDir = fDir.normalized
            else:
                # Use the finger tip for mapping
                targetDir = finger.tip_position
            # The ratio of the length to the maxLength
            fLengthRatio = fLength/maxFingerLength

            mappedTarget = {
                'id': index,
                'dir': [targetDir[0], targetDir[1], targetDir[2]],
                'length_ratio': fLengthRatio
            }

            return mappedTarget

    '''
    '' Returns a mapping object for the palm position and normal direction
    '''
    def get_palm_target_mapping(self, palm):
        scaleFactor = 0.06
        palmPosition = palm['position']
        palmNormal = palm['normal']
        mappedTarget = {
                'id': CONFIG['PALM_INDEX'],
                'normal': [palmNormal[0], palmNormal[1], palmNormal[2]],
                'position': [
                    scaleFactor * palmPosition[0], 
                    scaleFactor * palmPosition[1], 
                    scaleFactor * palmPosition[2]]
        }
        return mappedTarget

    '''
    '' Returns if we should be capturing calibration length data
    '''
    def should_calibrate(self):
        return self.shouldCalibrate
    
    '''
    '' Increment the number of calibration captures
    '''
    def increment_num_calibration_frames(self):
        self.numCalibrationFrames += 1
        if (self.numCalibrationFrames > self.maxCalibrationFrames):
            self.shouldCalibrate = False 
    
    ''' 
    '' Resets the calibration state
    '''
    def reset_calibration(self):
        self.shouldCalibrate = True
        self.numCalibrationFrames = 0


    ''' 
    '' Returns the current number of calibration frames
    ''' 
    def get_num_calibration_frames(self):
        return self.numCalibrationFrames

    '''
    '' Gets a lengths over maxNumCalibration Frames to get 
    '' an averaged length
    '''
    def calibrate_lengths(self, fingers):
        if (self.should_calibrate() and len(fingers) == self.numFingers):
            #Sort the Fingers
            fingers = self.sort_fingers_by_x(fingers)
            
            #For each finger, get its length and then add to the finger data
            for index in range(0, len(fingers)):
                #Get finger at index
                finger = fingers[index]
                #Get finger length
                fLength = finger.length

                #Update the FingerLengthData to include the length
                fingerLengthData = self.get_finger_data(index)
                fingerLengthData.add_base_length(fLength)

            # Sum the length into the BASELINE Length
            print 'Calibrating - Frame %s of %s' % (self.get_num_calibration_frames(), self.maxCalibrationFrames)
            
            # Increment our count of baseline frames
            self.increment_num_calibration_frames()

        # When we have the max number of baseline frames, average the values for the fingers
        # Then begin actual target point collection
        if not self.should_calibrate():
            # For each FingerLengthData
            for data in self.fingerLengthData:
                # average all the base frame lengths
                data.compute_base_length()

    ''' 
    '' Sort the fingers based on  x-position 
    '' (left-most == Thumb, Right Hand OR Pinky, Left Hand)
    '''
    def sort_fingers_by_x(self, fingers):  
        #Make a list of the fingers that isn't const (FingerList is)
        sortedFingers = []
        for finger in fingers:
            sortedFingers.append(finger)
            #print finger
        # Selection sort the fingers\
        sortedFingers = sorted(sortedFingers, key=lambda finger: finger.tip_position.x)
        return sortedFingers

    '''
    '' Sets whether we should be sending palm data to Maya
    '''
    def set_track_palm_position(self, shouldSend):
        self.shouldSendPalmData = shouldSend
        if (self.shouldSendPalmData):
            print('Sending Palm Data')
        else:
            print('Stopped Sending Palm Data')
            

    '''
    '' Determines if this listener is sending palm data 
    '''
    def is_tracking_palm_position(self):
        return self.shouldSendPalmData

    
    '''
    '' Sets whether we are tracking hands 
    '''
    def set_is_tracking(self, shouldTrack):
        self.isTracking = shouldTrack
        if (self.isTracking):
            print('Tracking')
        else:
            print('Paused Tracking')
            
    '''
    '' Determines if this listener is tracking frames
    '''
    def is_tracking(self):
        return self.isTracking    


#######################################################
'''
#############################################################
## PAALM TRACKING FUNCTIONS #################################
#############################################################
'''

'''
'' Initializes a leap listener and tracker for getting hand data
'''
def init_tracking(self=None):
    # Stop tracking if we are currently
    stop_tracking()
    
    # Add the listener to receive events from the controller
    LEAP_CONTROLLER.add_listener(PAALM_LEAP_LISTENER)

    # Keep this process running until Q is pressed to quit
    print 'Tracking hand gestures!'

'''
'' Temporarily pause hand tracking
'''
def toggle_tracking(self=None):
    # Toggle sending palm data
    wasTracking = PAALM_LEAP_LISTENER.is_tracking()
    isTracking = not wasTracking
    PAALM_LEAP_LISTENER.set_is_tracking(isTracking)
    trackingMsg = 'Tracking Started'
    if (not isTracking): 
        trackingMsg = 'Tracking Paused'
    print trackingMsg

'''
'' Temporarily stop sending palm tracking datah
'''
def toggle_palm_tracking(self=None):
    # Toggle sending palm data
    wasTrackingPalm = PAALM_LEAP_LISTENER.is_tracking_palm_position()
    isTrackingPalm = not wasTrackingPalm
    PAALM_LEAP_LISTENER.set_track_palm_position(isTrackingPalm)
    trackingMsg = 'Tracking Started'
    if (not isTrackingPalm): 
        trackingMsg = 'Tracking Paused'
    print trackingMsg


'''
'' Terminates hand tracking
'''
def stop_tracking(self=None):
    # Remove the sample listener when done
    LEAP_CONTROLLER.remove_listener(PAALM_LEAP_LISTENER)
    print 'Removed Tracking Listener'


'''
'' Calibrates hand tracking data based on finger lengths and
'' a spread open palm
'''
def init_calibration(self=None):
    print 'Hold your hand above the Leap Motion Controller with Open Palms'
    PAALM_LEAP_LISTENER.reset_calibration()

'''
'' Initializes a demo type for IK Targets
'''
def init_demo(demoType):
    if (demoType == HAND_DEMO_TYPE):
        IK_HAND_TEST()
    elif (demoType == CHARACTER_DEMO_TYPE):
        #IK_CHARACTER_TEST()
        print 'Character Demo'
    elif (demoType == OCTOPUS_DEMO_TYPE):
        #IK_OCTO_TEST()
        print 'Octopus Demo'




############################################################
########################### MAIN ###########################
############################################################

def main():
    # Have the sample listener receive events from the controller
    init_tracking()

    # Keep this process running until Q is pressed to quit
    print('Press \'P\' to toggle sending palm data' + '\nPress Q to quit')
    while (True):
        inputKey = sys.stdin.read(1)
        if (inputKey == 'p'):
            toggle_palm_tracking()
        elif (inputKey == ' '):
            toggle_tracking()
        elif (inputKey == 'r'):
            init_calibration()
        elif (inputKey == 'q'):
            # Quit
            stop_tracking()
            return

    # Remove the sample listener when done
    LEAP_CONTROLLER.remove_listener(PAALM_LEAP_LISTENER)

if __name__ == "__main__":
    main()


