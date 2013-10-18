'''
'' PAALM Plug-in  - for tracking hand data using the Leap Motion Controller
'' @author: Michael Rivera
'' @date: 10/17/2013
'' @information Plug-in Tutorial:
'' http://jeremyyk.com/tutorials/autodesk-s-tutorials-creating-python-plugins-in-maya
''  
'''

import sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

''' Refers to MEL command that starts the PAALM Plugin '''
PAALM_COMMAND_NAME = 'paalm'

''' PAALM Command Class '''
class PAALM(OpenMayaMPx.MPxCommand):

    ''' Constructor '''
    def __init__(self):
        OpenMayaMPx.MPxCommand.__init__(self)

    ''' Execution of the Command '''
    def doIt(self, args):
        print 'PAALM Loaded'

''' Creates an instance of our command '''
def paalmCommandCreator():
    return OpenMayaMPx.asMPxPtr(PAALM())

''' Initialize the plug-in when Maya loads it '''
def initializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.registerCommand(PAALM_COMMAND_NAME, paalmCommandCreator)
    except:
        sys.stderr.write('Failed to register command: ' + PAALM_COMMAND_NAME)

''' Uninitialize the plug-in when Maya un-loads it '''
def uninitializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    try:
        mPlugin.deregisterCommand(PAALM_COMMAND_NAME)
    except:
        sys.stderr.write('Failed to unregister command: ' + PAALM_COMMAND_NAME)