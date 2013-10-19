#!/usr/bin/python
# -*- coding: utf-8 -*- 
'''
'' PAALM Plug-in  - for tracking hand data using the Leap Motion Controller
'' @author: Michael Rivera
'' @date: 10/17/2013
'' @information Plug-in Tutorial:
'' http://jeremyyk.com/tutorials/autodesk-s-tutorials-creating-python-plugins-in-maya
''  
'''

# System Imports
import sys, string, math, time
from functools import partial

# Maya Imports
import maya.cmds as cmds
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

# Leap Imports
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


# Socket Information
MAYA_PORT = 6001

class MayaConnection():
    
    def __init__(self, PORT):
        # self.maya = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.maya.connect(('localhost', PORT))
        self.port = PORT

    # Close the socket port
    def close(self):
        pass
        #self.maya.close()
             
    # Sends a tip position update to Maya to trigger a perform_ccd() action 
    def send_tip_position(self, tpX, tpY, tpZ, lengthRatio):
        command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        #self.maya.send(command)
        print 'Sending Tip Position'
        #print command


    # Sends a target queue update to Maya to trigger a perform_ccd() action 
    def send_target_queue(self, targetQueue):
        command = 'pyth/on(\"receive_target_queue(%s)\")\n' % targetQueue
        #command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        # print targetQueue
        
        #self.maya.send(command)
        print 'Sending Target Queue!'



############################################################
####################### FINGER DATA ########################
############################################################
# Finger data just stores information about our max finger Lengths
# With a reference to a finger ID 0 == Thumb, 4 == Pinky

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
        # print 'index: %s, sum_lengths: %s, count: %s' % (self.index, self.sumBaseLengths, self.baseLengthCount)
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
# Listener for phaleangeal angle approximator

class PAListener(Leap.Listener):

    #On initilization of listener
    def on_init(self, controller):
        # Make a new Maya Connection on port 6001
        self.is_peforming_ccd = False
        self.isTracking = True
        self.fingerData = []
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

        # Initialize Maya connection
        self.init_maya_connection()

        print "Initialized"

    # Initialize five fingers for this Listener
    def init_fingers(self):
        # Initialize only 4 fingers for the right hand
        for index in range(0, self.numFingers):
            data = FingerData(index)
            #Add to our data array
            self.fingerData.append(data)

    # Initilize Maya Connection
    def init_maya_connection(self):
        self.mayaConnection = MayaConnection(MAYA_PORT)

    # Get the finger data for an id
    def get_finger_data(self, index):
        return self.fingerData[index]

    # On connect of listener to controller
    def on_connect(self, controller):
        print "Connected"

    # On disconnect of the listener to the controller
    def on_disconnect(self, controller):
        self.mayaConnection.close()
        print "Disconnected"

    # On exit of listener
    def on_exit(self, controller):
        self.mayaConnection.close()
        print "Exited"

    # On Frame being read from the Leap Do something
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

                # Send data to maya
                self.mayaConnection.send_target_queue(targetQueue)

                #print fingers

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
            fingerData = self.get_finger_data(index)
            maxFingerLength = fingerData.get_base_length()

            # Get the finger dir and length
            fDir = finger.direction
            fLength = finger.length
            fPos = finger.tip_position

            # Are we only using direction or do we want positon?
            USE_DIRECTION = True

            if USE_DIRECTION:
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
                'id': PALM_INDEX,
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

                #Update the fingerData to include the length
                fingerData = self.get_finger_data(index)
                fingerData.add_base_length(fLength)

            # Sum the length into the BASELINE Length
            print 'Calibrating - Frame %s of %s' % (self.get_num_calibration_frames(), self.maxCalibrationFrames)
            
            # Increment our count of baseline frames
            self.increment_num_calibration_frames()

        # When we have the max number of baseline frames, average the values for the fingers
        # Then begin actual target point collection
        if not self.should_calibrate():
            # For each fingerData
            for data in self.fingerData:
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


################################################################################
###  Custom Menu Window for PAALM (Buttons) ####################################
################################################################################
class PAALMEditorWindow(object):

    ''' 
    '' Creates the Editor Window 
    '''
    def __init__(self):
        global PAALM_EDITOR_WINDOW_NAME
        global PAALM_EDITOR_WINDOW
        # Create an editor window
        self.name = PAALM_EDITOR_WINDOW_NAME
        self.window = None
        self.layout = None

        # Delete the old window by name if it exists
        if (self.exists()):
            cmds.deleteUI(self.name)

        # Build the window and layout
        self.construct()

        # Store global reference for window
        PAALM_EDITOR_WINDOW = self

    '''
    '' Returns true if this editor window already exists
    '''
    def exists(self):
        return cmds.window(self.name, query=True, exists=True)

    '''
    '' Constructs the window and layout
    '''
    def construct(self):
        self.window =cmds.window(
            self.name,
            menuBar=True, 
            width=450, 
            height=450, 
            title='PAALM Editor',
            maximizeButton=False,
            docTag='PAALM Editor',
            sizeable=False,
            retain=True)
        self.layout = cmds.columnLayout(parent=self.window)

        # Reset button for clearing keyframes
        # TODO: RESET THE KEYFRAME USING THE IKHANDLE PLUGIN RESET FUNCTION
        cmds.button(
            label='Reset', 
            parent=self.layout, 
            command= 'print "Resetting Keyframes"')

        # Calibrate Button
        cmds.button(
            label='Calibrate', 
            parent=self.layout, 
            command='init_calibration()')
               

        # Start Tracking
        cmds.button(
            label='Start Tracking', 
            parent=self.layout, 
            command='init_tracking()')

        # Stop Tracking
        cmds.button(
            label='Pause Tracking', 
            parent=self.layout, 
            command='toggle_tracking()')

    '''
    '' Shows the editor window
    '''
    def show(self):
        # Make the window if none exists
        if (self.exists()):
            # Show the window if it already exists
            cmds.showWindow(self.name)
        else:
            # Show the button window
            self.construct()
            cmds.showWindow(self.window)


################################################################################
###  Custom Drop Down Menu for PAALM  ##########################################
################################################################################

class PAALMDropDownMenu(object):

    '''
    '' Initialize the PAALM DropDown Menu
    '''
    def __init__(self):
        global PAALM_DROP_DOWN_MENU_NAME, PAALM_DROP_DOWN_MENU_LABEL
        gMainWindow = maya.mel.eval('$temp1=$gMainWindow')
        self.name = PAALM_DROP_DOWN_MENU_NAME
        # Delete the old window by name if it exists
        if (self.exists()):
            cmds.deleteUI(self.name)

        dropDownMenu = cmds.menu(
            PAALM_DROP_DOWN_MENU_NAME, 
            label=PAALM_DROP_DOWN_MENU_LABEL, 
            parent=gMainWindow, 
            tearOff=True)
        cmds.menuItem(
            label='Show Editor', 
            parent=dropDownMenu,
            command='show_editor_window()')
        cmds.menuItem(
            label='About',
            parent=dropDownMenu,
            command='show_about_page()')
        #cmds.menuItem(divider=True)
        cmds.menuItem(
            label='Quit',
            parent=dropDownMenu,
            command='quit()')

    '''
    '' Returns true if this menu exists in Maya's top option menu
    '''
    def exists(self):
        return cmds.menu(self.name, query=True, exists=True)
    
#######################################################
''' 
#######################################################
## GLOBALS UI FXNS ####################################
#######################################################
'''

''' 
'' Shows the PAALM Editor Window 
'''
def show_editor_window():
    global PAALM_EDITOR_WINDOW
    PAALM_EDITOR_WINDOW.show()

'''
'' Show the About / PAALM Blog
'''
def show_about_page():
    cmds.showHelp(PAALM_ABOUT_WEBSITE, absolute=True)

'''
'' Quit paalm tracking
'''
def quit():
    stop_tracking()
    print 'No Longer Tracking!'



#######################################################
'''
#######################################################
## PAALM TRACKING FUNCTIONS ###########################
#######################################################
'''

'''
'' Initializes a leap listener and tracker for getting hand data
'''
def init_tracking():
    # Create a listener and controller
    global PAALM_LEAP_LISTENER, LEAP_CONTROLLER 

    # Stop tracking if we are currently
    stop_tracking()
    
    # Add the listener to receive events from the controller
    LEAP_CONTROLLER.add_listener(PAALM_LEAP_LISTENER)

    # Keep this process running until Q is pressed to quit
    print 'Tracking hand gestures!'

'''
'' Temporarily pause hand tracking
'''
def toggle_tracking():
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
def toggle_palm_tracking():
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
def stop_tracking():
    # Remove the sample listener when done
    LEAP_CONTROLLER.remove_listener(PAALM_LEAP_LISTENER)
    print 'Removed Tracking Listener'


'''
'' Calibrates hand tracking data based on finger lengths and
'' a spread open palm
'''
def init_calibration():
    print 'Hold your hand above the Leap Motion Controller with Open Palms'
    PAALM_LEAP_LISTENER.reset_calibration()



#######################################################
'''
#######################################################
## GLOBALS VARS  ######################################
#######################################################
'''

## Author Information ##
PAALM_AUTHOR = 'Michael Rivera'
PAALM_AUTHOR_WEBSITE = 'http://mikeriv.com'
PAALM_ABOUT_WEBSITE = 'http://projectpaalm.blogspot.com'

## Refers to MEL command that starts the PAALM Plugin ##
PAALM_COMMAND_NAME = 'paalm'


## Unique Name of the PAALM Editor Window in memory ##
PAALM_EDITOR_WINDOW_NAME = 'PAALMEditor'

## Reference to the PAALM Editor Window ##
PAALM_EDITOR_WINDOW = PAALMEditorWindow()

## Unique name of the drop down menu in memory ##
PAALM_DROP_DOWN_MENU_NAME = 'PAALMMENU'

## Label of the drop down menu ##
PAALM_DROP_DOWN_MENU_LABEL = 'PAALM'

## Reference to the PAALM Drop Down Menu ##
PAALM_DROP_DOWN_MENU = PAALMDropDownMenu()

## The Leap Controller Reference ##
LEAP_CONTROLLER = Leap.Controller()

## The PAALM Leap Listener Reference ##
PAALM_LEAP_LISTENER = PAListener()

## ##

'''
'' OPTIONS PALM OPTIONS
'''
# Index for Palm Position
PALM_INDEX = 9999

## ##

## ##

## ##


##############################################################
'''
##############################################################
## Maya Plug-in Set up #######################################
##############################################################
'''

''' 
'' PAALM Command Class 
'''
class PAALM(OpenMayaMPx.MPxCommand):

    ''' 
    '' Constructor 
    '''
    def __init__(self):
        OpenMayaMPx.MPxCommand.__init__(self)

    ''' 
    '' Execution of the Command 
    '''
    def doIt(self, args):
        show_editor_window()
        print 'PAALM Loaded'

''' 
'' Creates an instance of our command 
'''
def paalm_command_creator():
    return OpenMayaMPx.asMPxPtr(PAALM())

''' 
'' Initialize the plug-in when Maya loads it 
'''
def initializePlugin(mObject):
    global PAALM_COMMAND_NAME
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.registerCommand(PAALM_COMMAND_NAME, paalm_command_creator)
    except:
        sys.stderr.write('Failed to register command: ' + PAALM_COMMAND_NAME)

''' 
'' Uninitialize the plug-in when Maya un-loads it 
'''
def uninitializePlugin(mObject):
    global PAALM_COMMAND_NAME
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.deregisterCommand(PAALM_COMMAND_NAME)
    except:
        sys.stderr.write('Failed to unregister command: ' + PAALM_COMMAND_NAME)
