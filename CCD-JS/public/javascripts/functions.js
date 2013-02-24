
//Makes sure all Files are loaded before running sockets or the Leap
$(document).ready(function() {

 


    //############### LEAP MOTION #######################
    var previousFrame;
        
        Leap.loop( function(frame){
                  
          //Display Frame object data
          var frameOutput = document.getElementById("frameData");
          
          var frameString = "Frame ID: " + frame.id  + "<br />"
                          + "Timestamp: " + frame.timestamp + "<br />";
          
          //Frame motion factors
          if( previousFrame )
          {
              var translation = frame.translation( previousFrame );
              frameString += "Translation: (" 
                              + translation[0].toFixed(1) + "," 
                              + translation[1].toFixed(1) + "," 
                              + translation[2].toFixed(1) + ") mm <br />";
              
              var rotation = frame.rotationAngle( previousFrame );
              frameString += "Rotation: " + rotation.toFixed(2) + " radians <br />";
              
              var scale = frame.scaleFactor( previousFrame );
              frameString += "Scale: " + scale.toFixed(2) + "<br />";
          }
          frameOutput.innerHTML = frameString;
          
          //Display Hand object data
          var handOutput = document.getElementById("handData");
          var handString = "";
          if( frame.hands.length > 0 )
          {
            for( var i = 0; i < frame.hands.length; i++ )
            {
              handString += "<div style='width:300px; float:left; padding:20px'>";
          
              var hand = frame.hands[i];
              handString += "Hand ID: " + hand.id + "<br />";
                
              var position = hand.palmPosition;
              handString += "Palm position: (" 
                + position[0].toFixed(1) + "," 
                + position[1].toFixed(1) + "," 
                + position[2].toFixed(1) + ") mm<br />";
      
              var direction = hand.direction;
              handString += "Direction: (" 
                + direction[0].toFixed(1) + "," 
                + direction[1].toFixed(1) + "," 
                + direction[2].toFixed(1) + ") mm<br />";

              var normal = hand.palmNormal;
              handString += "Normal: (" 
                + normal[0].toFixed(1) + "," 
                + normal[1].toFixed(1) + "," 
                + normal[2].toFixed(1) + ") mm<br />";

              var velocity = hand.palmVelocity;
              handString += "Palm velocity: (" 
                + velocity[0].toFixed(1) + "," 
                + velocity[1].toFixed(1) + "," 
                + velocity[2].toFixed(1) + ") mm/s<br />";

              handString += "Sphere radius: " + hand.sphereRadius + "mm<br />";
              var spherePosition = hand.sphereCenter;
              handString += "Sphere center point: (" 
                + spherePosition[0].toFixed(1) + "," 
                + spherePosition[1].toFixed(1) + "," 
                + spherePosition[2].toFixed(1) + ") mm<br />";
      
              //Hand motion factors
              if( previousFrame )
              {
                  var translation = hand.translation( previousFrame );
                  handString += "Translation: (" 
                                + translation[0].toFixed(1) + "," 
                                + translation[1].toFixed(1) + "," 
                                + translation[2].toFixed(1) + ") mm <br />";
                  
                  var rotation = hand.rotationAngle( previousFrame );
                  handString += "Rotation: " + rotation.toFixed(2) + " radians <br />";
                  
                  var scale = hand.scaleFactor( previousFrame );
                  handString += "Scale: " + scale.toFixed(2) + "<br />";
              }

              //IDs of pointables (fingers and tools) associated with this hand
              if( hand.pointables.length > 0 )
              {
                var fingerIds = [];
                var toolIds = [];
                for( var j = 0; j < hand.pointables.length; j++ )
                {                          
                    var pointable = hand.pointables[j];
                    if( pointable.tool ) 
                    {
                        toolIds.push( pointable.id );
                    }
                    else
                    {
                        fingerIds.push( pointable.id );
                    }
                }
                 handString += "Fingers: " + fingerIds + "<br />";
                 handString += "Tools: " + toolIds + "<br />";
              }
      
              handString += "</div>";
            }
          }
          else
          {
            handString += "<div>No hands</div>";
          }
          handOutput.innerHTML = handString;
          
          //Display Pointable (fingers and tools) object data
          var pointableOutput = document.getElementById("pointableData");
          var pointableString = "";
          if( frame.pointables.length > 0 )
          {
            for( var i = 0; i < frame.pointables.length; i++ )
            {
              pointableString += "<div style='width:250px; float:left; padding:20px'>";
              
              var pointable = frame.pointables[i];
              pointableString += "Pointable ID: " + pointable.id + "<br />";
              pointableString += "Belongs to hand with ID: " + pointable.handId + "<br />";
      
              if( pointable.tool )
              {
                pointableString += "Classified as a tool <br />"; 
                pointableString += "Length: " + pointable.length.toFixed(1) + "mm <br />";
                pointableString += "Width: "  + pointable.width.toFixed(1) + "mm <br />";
              }
              else
              {
                 pointableString += "Classified as a finger<br />";
                 pointableString += "Length: " + pointable.length.toFixed(1) + "mm <br />";
              }
      
              var position = pointable.tipPosition;
              pointableString += "Tip position: (" 
              + position[0].toFixed(1) + "," 
              + position[1].toFixed(1) + "," 
              + position[2].toFixed(1) + ") mm<br />";

              var direction = pointable.direction;
              pointableString += "Direction: (" 
              + direction[0].toFixed(1) + "," 
              + direction[1].toFixed(1) + "," 
              + direction[2].toFixed(1) + ") mm<br />";

              var velocity = pointable.tipVelocity;
              pointableString += "Tip velocity: (" 
              + velocity[0].toFixed(1) + "," 
              + velocity[1].toFixed(1) + "," 
              + velocity[2].toFixed(1) + ") mm/s<br />";

              pointableString += "</div>";
            }
          }
          else
          {
            pointableString += "<div>No pointables</div>";
          }
          pointableOutput.innerHTML = pointableString;
          
          //Store frame for motion functions
          previousFrame = frame;
        });


  




  //############# LEAP HANDLING ####################

  //Gets the Effector Tip Position using a maya mapped Direction and the base(knuckle position)
  function getMayaEffectorTipPos(effectorDir, effectorBasePos) {
    return effectorDir + effectorBasePos;
  }


  //Gets a direction vector in maya space
  function mapDirToMaya(leapFingerDir, mayaEffectorLength) {
    //takes a normal vector in 3D space from the leap,
    //maps it to a vector in Maya's 3D space that is used to 
    //compute a new end effector position in maya
    return leapFingerDir*mayaEffectorLength;
  }


  //Selection Sort the fingers based on  x-position 
  // (left-most == Thumb, Right Hand OR Pinky, Left Hand)
  function sort_fingers_by_x(fingers) {  
    sortedFingers = [];
    //Make a list of the fingers that isn't const (FingerList is)
    for (finger in fingers) {
        sortedFingers.push(finger);
        console.log(finger);
    }
    //selection sort the fingers
    for (var i =0; i<sortedFingers.length; i++) {
        min = i;
        for (var j = i+1; sortedFingers.length; j++) {
            x1 = sortedFingers[i].tip_position.x;
            x2 = sortedFingers[j].tip_position.x;
            if (x2 < x1) 
              min = j;
        }
        //Swap the min (leftmost)
        temp = sortedFingers[i]
        sortedFingers[i] = sortedFingers[min];
        sortedFingers[min] = temp;
    }
    return sortedFingers;
  }






  //############# SOCKET IO ########################
  var socket = io.connect('http://localhost');

  //Receives the server event and logs the message
  socket.on('my server event', function (data) {
    console.log("We have a message!");
    console.log(data);
    //emits an event to the server (the server recieves this using socket.on() )
    socket.emit('my client event', { hi: "I was sent from the client!" });
  });
}); //Close document.ready



//Checks to make sure the Leap JS-SDK has been loaded
var checkLibrary = function()
{
    if( typeof Leap === "undefined" )
    {
        alert("leap.js not found, please download the latest version from https://github.com/leapmotion/leapjs");
        document.getElementById("main").innerHTML = "The leap.js library file was not found. You can download the library from <a href='https://github.com/leapmotion/leapjs'>https://github.com/leapmotion/leapjs</a>."
    }

}

