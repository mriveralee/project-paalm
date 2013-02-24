
//Makes sure all Files are loaded before running sockets
$(document).ready(function() {


 var socket = io.connect('http://localhost');
  
 //Receives the server event and logs the message
  socket.on('my server event', function (data) {
    console.log("We have a message!");
    console.log(data);
    //emits an event to the server (the server recieves this using socket.on() )
    socket.emit('my client event', { hi: "I was sent from the client!" });
  });



}); //Close document.ready