# Class: MayaConnection
# Basic Socket connection to Maya command port with
# interface for making MEL commands in python

#Socket Information
import socket

#In Maya run MEL command 
#commandPort -pre trs -n ":9100";
#commandPort -name ":6001"; -> Send Port


#commandPort -n ":6002";  -> Receive Poort

#Use Leap for vectors
import Leap

# mayaSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# maya.connect(('localhost', 6001))
#MAYA_PORT = 6001

class MayaConnection():
    
    def __init__(self, PORT):
        self.maya = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.maya.connect(('localhost', PORT))
        self.port = PORT

    #Close the socket port
    def close(self):
        self.maya.close()
             
    #Sends a tip position update to Maya to trigger a perform_ccd() action 
    def send_target_queue(self, tpX, tpY, tpZ, lengthRatio):
        command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        self.maya.send(command)
        #print command


