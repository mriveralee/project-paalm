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

    #Moves a named object to coordinates <x,y,z>
    def move(self, name, x, y, z):
        self.select(name)
        move_cmd = "move " + str(x) + " " + str(y) + " " + str(z) + ";" #" " + name + ";"
        self.maya.send(move_cmd)
        #Return the World coordinates of the name-object after the move
        #return self.get_world_coords(name)

    #Rotates a named object with angles <x,y,z>
    def rotate(self, name, x, y, z):
        self.select(name)
        rotate_cmd = "rotate " + str(x) + " " + str(y) + " " + str(z) + " "+ name + ";"
        self.maya.send(rotate_cmd)

    #Selects a named object in maya
    def select(self, name):
        select_cmd = "select " + str(name) + ";"
        #print select_cmd
        self.maya.send(select_cmd)

    #Gets world coordinates of a named object
    def get_world_coords(self, name):
        #self.select(name)
        wc_cmd = "xform -ws -q -t " + name + ";"
        #[x, y, z] = self.maya.send(wc_cmd)
        x = self.maya.send(wc_cmd)
        coords = self.receive_vector_from_maya()
        return coords

    #Receives response coordinates and returns them as a Leap Vector 
    def receive_vector_from_maya(self):
        #Received data from maya on port
        coords = self.maya.recv(self.port)
        coords = coords.split()[:-1]
        #Convert strings to floats
        #print coords
        #Make sure we have a vector coming in (3 components)
        if (len(coords) < 3):
            return Leap.Vector(0.0, 0.0, 0.0)
        #Base for Joint4
       # return Leap.Vector(0.148153, 0, -14.999268)
        #Base for Joint2
        #return Leap.Vector(0.0493845, 0.0, -4.999756)
        #return coords;
        i = 0
        while (i < len(coords) and i < 3):
            try:  
                coords[i] = float(coords[i])
                i += 1
            except ValueError,e:
                print "error",e,"on line",i
                #return Leap.Vector(0.0, 0.0, 0.0)
        
        #Send back a Leap Vector
        return Leap.Vector(coords[0], coords[1], coords[2])




    #Close the socket port
    def close(self):
        self.maya.close()
             

    #Creates a sphere with size radius
    def sphere(self, radius):
        sphere_cmd = "sphere " + str(radius) + ";"
        self.maya.send(sphere_cmd)

    #Sends a tip position update to Maya to trigger a perform_ccd() action 
    def send_tip_position_to_maya(self, tpX, tpY, tpZ, lengthRatio):
        command = "python(\"receive_tip_position_from_leap("+str(tpX)+","+str(tpY)+","+str(tpZ)+","+str(lengthRatio)+")\")\n"
        self.maya.send(command)
        #print command
#####




#Maya MEL Command Script
'''
global proc trs(string $inp){
    print $inp;
    print "\n";
    string $tp[];
    $tp=`eval $inp`;
    string $back;
    $back=stringArrayToString($tp,"/");
    print $back;
    print "\n";
    string $c;
    $c="s.sendall(\""+$back+"\")";
    python("import socket, sys");
    python("s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)");
    python("s.connect((\"127.0.0.1\", 6002))");
    python($c);
}
commandPort -pre trs -n ":6001"; //Receive From Python & run function trs
commandPort -name ":6002"; //-> Send Back to Python Port
'''